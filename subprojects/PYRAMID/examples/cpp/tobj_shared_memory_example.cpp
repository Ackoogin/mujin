// Showcase: two-component shared-memory Tactical Objects example.
//
// Composes two hand-written pcl::Component subclasses:
//   TacticalObjectsComponent (provider role) -- owns the InterestStore and
//                                                a ProvidedService binding.
//   HmiClientComponent       (consumer role) -- owns a ConsumedService.
//
// Single-threaded model: main() owns both executors and drives them with
// spinOnce(). No SpinThread, no std::atomic, no pcl::await -- futures are
// polled while the executor that resolves them is ticked, and stream
// callbacks fire on the consumer's executor thread (i.e. this thread).
//
// Demonstrated flows:
//   1. Unary create_requirement repeated three times to seed the store.
//   2. Streaming read_requirement that the server ends after one frame.
//   3. Streaming read_requirement that the client cancels after one frame.
//   4. Unary delete_requirement to drain the store.

#include "hmi_client_component.hpp"
#include "tactical_objects_component.hpp"

#include "pyramid_services_tactical_objects_provided_components.hpp"

#include <pcl/shared_memory_participant.hpp>

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace ProvidedSvc =
    pyramid::components::tactical_objects::services::provided;
namespace model = pyramid::domain_model;

constexpr int kMaxSpinTicks   = 500;   // ~5 s worst case at 10 ms ticks
constexpr uint32_t kTickMs    = 10;

std::string uniqueBusName() {
  const auto ticks = std::chrono::steady_clock::now()
                         .time_since_epoch()
                         .count();
  return "tobj_showcase_" + std::to_string(ticks);
}

model::ObjectInterestRequirement makeRequirement(std::string id,
                                                  double lat, double lon) {
  model::ObjectInterestRequirement r;
  r.base.id     = std::move(id);
  r.base.source = "example-client";
  r.policy      = model::DataPolicy::Obtain;
  r.dimension.push_back(model::BattleDimension::Air);
  model::Point point;
  point.position.latitude  = lat;
  point.position.longitude = lon;
  r.point                  = point;
  return r;
}

// Spin both executors until \p done() returns true, or until kMaxSpinTicks
// have elapsed. Returns whether the condition fired before the deadline.
bool spinUntil(pcl::Executor& server_exec, pcl::Executor& client_exec,
               const std::function<bool()>& done) {
  for (int i = 0; i < kMaxSpinTicks; ++i) {
    if (done()) return true;
    server_exec.spinOnce(kTickMs);
    client_exec.spinOnce(kTickMs);
  }
  return done();
}

// Drive both executors until a unary future resolves (or timeout). Returns
// the resolved Result<T>; on timeout the Result carries PCL_ERR_TIMEOUT.
template <class T>
ProvidedSvc::Result<T> awaitResult(pcl::Executor& server_exec,
                                   pcl::Executor& client_exec,
                                   std::future<ProvidedSvc::Result<T>> future) {
  const auto ready = [&] {
    return future.wait_for(std::chrono::milliseconds(0)) ==
           std::future_status::ready;
  };
  if (!spinUntil(server_exec, client_exec, ready)) {
    return ProvidedSvc::Result<T>{PCL_ERR_TIMEOUT, {}};
  }
  return future.get();
}

// -- Showcase steps ---------------------------------------------------------

bool createOne(tobj_example::HmiClientComponent& hmi,
               pcl::Executor& server_exec, pcl::Executor& client_exec,
               const model::ObjectInterestRequirement& req) {
  auto result = awaitResult(server_exec, client_exec,
                            hmi.createRequirementAsync(req));
  if (!result.ok() || result.value != req.base.id) {
    std::cerr << "[showcase] create failed (status=" << result.status
              << ", id=" << result.value << ")\n";
    return false;
  }
  return true;
}

bool deleteOne(tobj_example::HmiClientComponent& hmi,
               pcl::Executor& server_exec, pcl::Executor& client_exec,
               const model::Identifier& id) {
  auto result = awaitResult(server_exec, client_exec,
                            hmi.deleteRequirementAsync(id));
  if (!result.ok() || !result.value.success) {
    std::cerr << "[showcase] delete failed (status=" << result.status << ")\n";
    return false;
  }
  return true;
}

// Single-frame stream that ends from the server: query exactly one id. Frames
// land in the HMI client's local vector via on_frame; on_end flips a flag.
//
// All callback state is plain (no std::atomic) because every PCL callback --
// including on_frame and on_end -- runs on the client executor's thread,
// which is also this thread.
bool readOneByIdServerEnded(tobj_example::HmiClientComponent& hmi,
                            pcl::Executor& server_exec,
                            pcl::Executor& client_exec,
                            const model::Identifier& id) {
  std::vector<model::ObjectInterestRequirement> frames;
  bool         ended    = false;
  pcl_status_t end_code = PCL_OK;

  model::Query q;
  q.id.push_back(id);

  auto handle = hmi.streamReadRequirement(
      q,
      [&](const model::ObjectInterestRequirement& f) { frames.push_back(f); },
      [&](pcl_status_t s) { ended = true; end_code = s; });
  if (!handle) {
    std::cerr << "[showcase] server-ended read: invoke failed\n";
    return false;
  }

  if (!spinUntil(server_exec, client_exec, [&] { return ended; })) {
    std::cerr << "[showcase] server-ended read: on_end never fired\n";
    return false;
  }
  if (end_code != PCL_OK) {
    std::cerr << "[showcase] server-ended read: on_end status=" << end_code
              << "\n";
    return false;
  }
  if (frames.size() != 1u || frames.front().base.id != id) {
    std::cerr << "[showcase] server-ended read: unexpected frames (count="
              << frames.size() << ")\n";
    return false;
  }
  std::cout << "[showcase] server-ended stream: 1 frame received from server\n";
  return true;
}

// Multi-frame stream that ends from the client: query all, cancel after the
// first frame from inside on_frame. Demonstrates StreamHandle::cancel()
// suppressing later frames and on_end firing with PCL_ERR_CANCELLED.
bool readAllClientCancelled(tobj_example::HmiClientComponent& hmi,
                            pcl::Executor& server_exec,
                            pcl::Executor& client_exec,
                            int expected_total_seedset) {
  int          frames_received = 0;
  bool         ended           = false;
  pcl_status_t end_code        = PCL_OK;

  ProvidedSvc::StreamHandle handle;
  handle = hmi.streamReadRequirement(
      model::Query{},  // empty -> "all"
      [&](const model::ObjectInterestRequirement&) {
        ++frames_received;
        if (frames_received == 1) {
          handle.cancel();
        }
      },
      [&](pcl_status_t s) { ended = true; end_code = s; });
  if (!handle && !ended) {
    std::cerr << "[showcase] client-cancelled stream: invoke failed\n";
    return false;
  }

  if (!spinUntil(server_exec, client_exec, [&] { return ended; })) {
    std::cerr << "[showcase] client-cancelled stream: on_end never fired\n";
    return false;
  }
  if (end_code != PCL_ERR_CANCELLED) {
    std::cerr << "[showcase] client-cancelled stream: on_end status="
              << end_code << " want=" << PCL_ERR_CANCELLED << "\n";
    return false;
  }
  if (frames_received < 1) {
    std::cerr << "[showcase] expected at least one frame before cancel\n";
    return false;
  }
  if (frames_received >= expected_total_seedset) {
    std::cerr << "[showcase] cancel did not suppress later frames (got "
              << frames_received << " of " << expected_total_seedset << ")\n";
    return false;
  }
  std::cout << "[showcase] client-cancelled stream: " << frames_received
            << " frame(s) before cancel of " << expected_total_seedset
            << " available, on_end=CANCELLED\n";
  return true;
}

// -- Top-level orchestration -----------------------------------------------

bool runSharedMemoryShowcase(const char* content_type) {
  const auto bus = uniqueBusName();

  pcl::SharedMemoryParticipant server{bus, "server", pcl::WithGateway::Yes};
  pcl::SharedMemoryParticipant client{bus, "client"};
  if (!server || !client) return false;

  tobj_example::TacticalObjectsComponent tactical{server.executor(),
                                                  content_type};
  tobj_example::HmiClientComponent       hmi{client.executor(), content_type};

  if (tactical.configure() != PCL_OK)             return false;
  if (tactical.activate()  != PCL_OK)             return false;
  if (server.executor().add(tactical) != PCL_OK)  return false;
  if (tactical.routeProvidedTo("client") != PCL_OK) return false;

  if (hmi.configure() != PCL_OK)                  return false;
  if (hmi.activate()  != PCL_OK)                  return false;
  if (client.executor().add(hmi) != PCL_OK)       return false;
  if (hmi.routeProvidedDefault() != PCL_OK)       return false;

  // Seed three entries so the multi-frame stream has multiple frames.
  const auto req1 = makeRequirement("interest-1", 51.4778, -0.0015);
  const auto req2 = makeRequirement("interest-2", 51.5000, -0.1000);
  const auto req3 = makeRequirement("interest-3", 52.0000, -1.0000);
  if (!createOne(hmi, server.executor(), client.executor(), req1)) return false;
  if (!createOne(hmi, server.executor(), client.executor(), req2)) return false;
  if (!createOne(hmi, server.executor(), client.executor(), req3)) return false;

  if (!readOneByIdServerEnded(hmi, server.executor(), client.executor(),
                              req2.base.id)) {
    return false;
  }

  if (!readAllClientCancelled(hmi, server.executor(), client.executor(),
                              /*expected_total_seedset=*/3)) {
    return false;
  }

  if (!deleteOne(hmi, server.executor(), client.executor(), req1.base.id)) return false;
  if (!deleteOne(hmi, server.executor(), client.executor(), req2.base.id)) return false;
  if (!deleteOne(hmi, server.executor(), client.executor(), req3.base.id)) return false;

  if (!tactical.store().empty()) {
    std::cerr << "[showcase] store should be empty, size="
              << tactical.store().size() << "\n";
    return false;
  }
  return true;
}

}  // namespace

int main() {
  if (!runSharedMemoryShowcase(ProvidedSvc::kFlatBuffersContentType)) {
    std::cerr << "tactical objects shared-memory showcase failed\n";
    return 1;
  }
  std::cout << "tactical objects shared-memory showcase completed\n";
  return 0;
}
