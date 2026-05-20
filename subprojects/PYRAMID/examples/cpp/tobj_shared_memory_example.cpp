// Showcase: two-component shared-memory Tactical Objects service example.
//
// Demonstrates the component-shaped C++ facade emitted by the PYRAMID binding
// generator. The user-facing layer is:
//
//   tobj_interest_store.hpp/.cpp
//       Subclasses the generated ProvidedHandler. Contains nothing but typed
//       business logic.
//
//   tobj_shared_memory_example.cpp (this file)
//       Bus + executor bring-up, component construction, and the demonstration
//       sequence. No PCL types, no codec branches, no manual stream plumbing.
//
// The streaming RPC (object_of_interest.read_requirement) is exercised in two
// modes:
//   1. Server-ended single-frame stream -- ask for exactly one ID. The server
//      emits one frame, then end. Consumer collects via the future-returning
//      Async variant.
//   2. Client-cancelled multi-frame stream -- ask for the full set. The
//      consumer uses the push-mode Streaming variant, cancels its StreamHandle
//      after a single frame, and observes on_end firing with PCL_ERR_CANCELLED.

#include "tobj_interest_store.hpp"

#include "pyramid_services_tactical_objects_provided_components.hpp"

#include <pcl/await.hpp>
#include <pcl/shared_memory_participant.hpp>
#include <pcl/spin_thread.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <string>

namespace {

namespace svc   = pyramid::components::tactical_objects::services::provided;
namespace model = pyramid::domain_model;

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

// -- Showcase steps ---------------------------------------------------------

bool createOne(svc::ConsumedComponent& consumer, pcl::Executor& exec,
               const model::ObjectInterestRequirement& req) {
  auto fut    = consumer.objectOfInterestCreateRequirementAsync(req);
  const auto result = pcl::awaitValue(exec, std::move(fut));
  if (!result || !result->ok() || result->value != req.base.id) {
    std::cerr << "[showcase] create failed\n";
    return false;
  }
  return true;
}

// Single-frame stream that ends from the server: query exactly one id.
bool readOneByIdServerEnded(svc::ConsumedComponent& consumer,
                            pcl::Executor& exec,
                            const model::Identifier& id) {
  model::Query q;
  q.id.push_back(id);
  auto fut    = consumer.objectOfInterestReadRequirementAsync(q);
  const auto r = pcl::awaitValue(exec, std::move(fut));
  if (!r || !r->ok() || r->value.size() != 1u || r->value.front().base.id != id) {
    std::cerr << "[showcase] server-ended read failed\n";
    return false;
  }
  std::cout << "[showcase] server-ended stream: 1 frame received from server\n";
  return true;
}

// Multi-frame stream that ends from the client: query all, cancel after the
// first frame. Demonstrates StreamHandle::cancel() suppressing subsequent
// on_frame callbacks and on_end firing with PCL_ERR_CANCELLED.
bool readAllClientCancelled(svc::ConsumedComponent& consumer,
                            pcl::Executor& exec,
                            int expected_total_seedset) {
  std::atomic<int>            frames_received{0};
  std::promise<pcl_status_t>  end_promise;
  auto                        end_future = end_promise.get_future();
  std::atomic<bool>           end_fired{false};

  svc::StreamHandle handle;
  handle = consumer.objectOfInterestReadRequirementStreaming(
      model::Query{},  // empty -> "all"
      [&](const model::ObjectInterestRequirement& /*frame*/) {
        const int seen = ++frames_received;
        if (seen == 1) {
          // Client decides the stream is done after one frame.
          handle.cancel();
        }
      },
      [&](pcl_status_t status) {
        if (!end_fired.exchange(true)) {
          end_promise.set_value(status);
        }
      });
  if (!handle) {
    std::cerr << "[showcase] streaming invoke failed to start\n";
    return false;
  }

  const auto end_status_opt = pcl::awaitValue(exec, std::move(end_future));
  if (!end_status_opt) {
    std::cerr << "[showcase] client-cancelled stream did not fire on_end\n";
    return false;
  }
  if (*end_status_opt != PCL_ERR_CANCELLED) {
    std::cerr << "[showcase] client-cancelled stream on_end status="
              << *end_status_opt << " want=" << PCL_ERR_CANCELLED << "\n";
    return false;
  }
  if (frames_received.load() < 1) {
    std::cerr << "[showcase] expected at least one frame before cancel\n";
    return false;
  }
  if (frames_received.load() >= expected_total_seedset) {
    std::cerr << "[showcase] cancel did not suppress later frames (got "
              << frames_received.load() << " of " << expected_total_seedset
              << ")\n";
    return false;
  }
  std::cout << "[showcase] client-cancelled stream: "
            << frames_received.load() << " frame(s) before cancel of "
            << expected_total_seedset << " available, on_end=CANCELLED\n";
  return true;
}

bool deleteOne(svc::ConsumedComponent& consumer, pcl::Executor& exec,
               const model::Identifier& id) {
  auto fut    = consumer.objectOfInterestDeleteRequirementAsync(id);
  const auto r = pcl::awaitValue(exec, std::move(fut));
  if (!r || !r->ok() || !r->value.success) {
    std::cerr << "[showcase] delete failed\n";
    return false;
  }
  return true;
}

// -- Top-level orchestration -----------------------------------------------

bool runSharedMemoryShowcase(const char* content_type) {
  const auto bus = uniqueBusName();

  pcl::SharedMemoryParticipant server{bus, "server", pcl::WithGateway::Yes};
  pcl::SharedMemoryParticipant client{bus, "client"};
  if (!server || !client) return false;

  tobj_example::InterestStore store;
  svc::ProvidedComponent provider{server.executor(), store, content_type};
  svc::ConsumedComponent consumer{client.executor(), content_type};

  if (!provider.start() || !consumer.start()) return false;
  if (provider.routeAllRemote("client") != PCL_OK) return false;
  if (consumer.routeAllRemote() != PCL_OK)         return false;

  pcl::SpinThread server_spin{server.executor()};

  // Seed three entries so the multi-frame stream has multiple frames.
  const auto req1 = makeRequirement("interest-1", 51.4778, -0.0015);
  const auto req2 = makeRequirement("interest-2", 51.5000, -0.1000);
  const auto req3 = makeRequirement("interest-3", 52.0000, -1.0000);
  if (!createOne(consumer, client.executor(), req1)) return false;
  if (!createOne(consumer, client.executor(), req2)) return false;
  if (!createOne(consumer, client.executor(), req3)) return false;

  // Stream demo 1: server-ended single-frame stream.
  if (!readOneByIdServerEnded(consumer, client.executor(), req2.base.id)) {
    return false;
  }

  // Stream demo 2: client-cancelled multi-frame stream.
  if (!readAllClientCancelled(consumer, client.executor(),
                              /*expected_total_seedset=*/3)) {
    return false;
  }

  // Tidy up.
  if (!deleteOne(consumer, client.executor(), req1.base.id)) return false;
  if (!deleteOne(consumer, client.executor(), req2.base.id)) return false;
  if (!deleteOne(consumer, client.executor(), req3.base.id)) return false;

  if (!store.empty()) {
    std::cerr << "[showcase] store should be empty, size=" << store.size()
              << "\n";
    return false;
  }
  return true;
}

}  // namespace

int main() {
  if (!runSharedMemoryShowcase(svc::kFlatBuffersContentType)) {
    std::cerr << "tactical objects shared-memory showcase failed\n";
    return 1;
  }
  std::cout << "tactical objects shared-memory showcase completed\n";
  return 0;
}
