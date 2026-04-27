/// \file test_binding_performance.cpp
/// \brief Codec/transport performance benchmark via generated PYRAMID bindings.
///
/// For each transport (local in-process, shared-memory bus, TCP socket) and each
/// codec (JSON, FlatBuffers, Protobuf), N messages of a non-trivial ObjectDetail
/// payload are transferred sequentially and the per-message latency is recorded.
/// gRPC (TCP and Unix socket) is benchmarked separately when the transport is
/// available.  A formatted report is printed at the end of the test suite.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_transport_socket.h>
#include <pcl/pcl_transport_shared_memory.h>
#include <pcl/pcl_types.h>
}

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"
// NOTE: pyramid_data_model_types.hpp (via consumed.hpp) and the protobuf
// generated headers (via pyramid_generated_codecs link) both define types in
// pyramid::data_model::tactical.  Avoid constructing ObjectDetail by value
// in this translation unit to prevent ODR destructor collisions at runtime.

#if defined(PYRAMID_HAS_GRPC)
#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cons = pyramid::services::tactical_objects::consumed;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr int kNMsgs     = 500;   ///< messages per PCL combo
static constexpr int kNMsgsGrpc = 200;   ///< messages per gRPC combo
static constexpr int kMsgTimeoutMs = 2000;

// ---------------------------------------------------------------------------
// Payload factory
// ---------------------------------------------------------------------------

static cons::ObjectDetail makePayload() {
  cons::ObjectDetail ev;
  ev.id            = "binding-perf-entity-000000000001-abcdef";
  ev.identity      = pyramid::data_model::StandardIdentity::Hostile;
  ev.dimension     = pyramid::data_model::BattleDimension::Ground;
  ev.position.latitude  = 51.477811;
  ev.position.longitude = -0.001475;
  ev.creation_time = 1717000000.0;
  ev.quality       = 0.987654321;
  return ev;
}

// ---------------------------------------------------------------------------
// Timing helpers
// ---------------------------------------------------------------------------

using Clock = std::chrono::high_resolution_clock;

static double toUs(Clock::time_point t0, Clock::time_point t1) {
  return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

// ---------------------------------------------------------------------------
// PerfResult and report
// ---------------------------------------------------------------------------

struct PerfResult {
  std::string label;
  int         n_msgs      = 0;
  size_t      payload_bytes = 0;
  double      avg_us      = 0.0;
  double      min_us      = 0.0;
  double      max_us      = 0.0;
  double      p50_us      = 0.0;
  double      p99_us      = 0.0;
  bool        skipped     = false;
};

static PerfResult computeResult(const std::string& label,
                                int n,
                                size_t payload_bytes,
                                std::vector<double>& v) {
  PerfResult r;
  r.label        = label;
  r.n_msgs       = n;
  r.payload_bytes = payload_bytes;
  if (v.empty()) { r.skipped = true; return r; }

  std::sort(v.begin(), v.end());
  double sum = 0.0;
  for (double x : v) sum += x;
  r.avg_us = sum / static_cast<double>(v.size());
  r.min_us = v.front();
  r.max_us = v.back();
  r.p50_us = v[v.size() / 2];
  r.p99_us = v[static_cast<size_t>(v.size() * 0.99)];
  return r;
}

static void printReport(const std::vector<PerfResult>& results) {
  // separator widths
  printf("\n");
  printf("============================================================"
         "==============================\n");
  printf(" PYRAMID Binding Performance Report\n");
  printf(" Payload: ObjectDetail  |  Iterations per combo: %d (gRPC: %d)\n",
         kNMsgs, kNMsgsGrpc);
  printf("============================================================"
         "==============================\n");
  printf("%-28s | %9s | %8s | %8s | %8s | %8s | %8s\n",
         "Transport/Codec", "Bytes", "Avg(us)", "Min(us)",
         "P50(us)", "P99(us)", "Max(us)");
  printf("%-28s-+-%9s-+-%8s-+-%8s-+-%8s-+-%8s-+-%8s\n",
         "----------------------------", "---------", "--------",
         "--------", "--------", "--------", "--------");

  for (const auto& r : results) {
    if (r.skipped) {
      printf("%-28s | %9s   (skipped)\n", r.label.c_str(), "");
      continue;
    }
    printf("%-28s | %9zu | %8.1f | %8.1f | %8.1f | %8.1f | %8.1f\n",
           r.label.c_str(), r.payload_bytes,
           r.avg_us, r.min_us, r.p50_us, r.p99_us, r.max_us);
  }
  printf("============================================================"
         "==============================\n\n");
  fflush(stdout);
}

// ---------------------------------------------------------------------------
// Shared receive state (used across transport benchmarks)
// ---------------------------------------------------------------------------

struct RecvState {
  std::atomic<int> count{0};
  const char*      content_type = nullptr;
};

static void recvCallback(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
  auto* s = static_cast<RecvState*>(ud);
  // Count bytes received (avoid constructing ObjectDetail by-value here:
  // pyramid::data_model::tactical::ObjectDetail is defined in both the
  // generated C++ bindings and the protobuf code; constructing a local
  // of that type in a translation unit that links both causes an ODR
  // destructor collision.  Encoding cost is already captured by the
  // timing window that starts in publishObjectEvidence.)
  if (msg && msg->size > 0)
    s->count.fetch_add(1, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Publisher on_configure helper
// ---------------------------------------------------------------------------

struct PubSetup {
  pcl_port_t* port        = nullptr;
  const char* content_type = nullptr;
};

static pcl_status_t onConfigurePub(pcl_container_t* c, void* ud) {
  auto* s = static_cast<PubSetup*>(ud);
  s->port = pcl_container_add_publisher(c, cons::kTopicObjectEvidence,
                                        s->content_type);
  return s->port ? PCL_OK : PCL_ERR_NOMEM;
}

// ---------------------------------------------------------------------------
// Subscriber on_configure helper
// ---------------------------------------------------------------------------

static pcl_status_t onConfigureSub(pcl_container_t* c, void* ud) {
  auto* s = static_cast<RecvState*>(ud);
  cons::subscribeObjectEvidence(c, recvCallback, ud, s->content_type);
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Unique ID helpers for transport names
// ---------------------------------------------------------------------------

static std::atomic<int> s_uid{0};

static std::string uniqueId(const char* prefix) {
  return std::string(prefix) + std::to_string(s_uid.fetch_add(1));
}

// ---------------------------------------------------------------------------
// 1. LOCAL (in-process, no transport)
// ---------------------------------------------------------------------------

static PerfResult runLocal(const char* label, const char* ct,
                           const cons::ObjectDetail& ev) {
  pcl_executor_t* exec = pcl_executor_create();

  PubSetup pub_setup{nullptr, ct};
  pcl_callbacks_t pub_cbs{};
  pub_cbs.on_configure = onConfigurePub;
  auto* pub_c = pcl_container_create("lperf_pub", &pub_cbs, &pub_setup);
  pcl_container_configure(pub_c);
  pcl_container_activate(pub_c);
  pcl_executor_add(exec, pub_c);

  RecvState recv{};
  recv.content_type = ct;
  pcl_callbacks_t sub_cbs{};
  sub_cbs.on_configure = onConfigureSub;
  auto* sub_c = pcl_container_create("lperf_sub", &sub_cbs, &recv);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(exec, sub_c);

  // Measure encoded size
  std::string sample;
  cons::encodeObjectEvidence(ev, ct, &sample);
  size_t payload_bytes = sample.size();

  std::vector<double> latencies;
  latencies.reserve(kNMsgs);

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = recv.count.load(std::memory_order_acquire);
    auto t0  = Clock::now();
    cons::publishObjectEvidence(pub_setup.port, ev, ct);
    for (int s = 0; s < 20; ++s) {
      pcl_executor_spin_once(exec, 0);
      if (recv.count.load(std::memory_order_acquire) > prev) break;
    }
    latencies.push_back(toUs(t0, Clock::now()));
  }

  pcl_executor_destroy(exec);
  pcl_container_destroy(sub_c);
  pcl_container_destroy(pub_c);

  return computeResult(label, kNMsgs, payload_bytes, latencies);
}

// ---------------------------------------------------------------------------
// 2. SHARED MEMORY bus transport
// ---------------------------------------------------------------------------

static PerfResult runShmem(const char* label, const char* ct,
                           const cons::ObjectDetail& ev) {
  std::string bus = uniqueId("perfbus");
  std::string pub_id = uniqueId("pub");
  std::string sub_id = uniqueId("sub");

  // --- Subscriber participant (set up first so it's registered on the bus) ---
  pcl_executor_t* sub_exec = pcl_executor_create();
  auto* sub_t = pcl_shared_memory_transport_create(
      bus.c_str(), sub_id.c_str(), sub_exec);
  if (!sub_t) {
    pcl_executor_destroy(sub_exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }
  pcl_executor_set_transport(sub_exec,
      pcl_shared_memory_transport_get_transport(sub_t));

  RecvState recv{};
  recv.content_type = ct;
  pcl_callbacks_t sub_cbs{};
  sub_cbs.on_configure = onConfigureSub;
  auto* sub_c = pcl_container_create("shmperf_sub", &sub_cbs, &recv);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(sub_exec, sub_c);

  // Subscriber spin thread
  std::atomic<bool> sub_stop{false};
  std::thread sub_thread([&] {
    while (!sub_stop.load(std::memory_order_relaxed)) {
      pcl_executor_spin_once(sub_exec, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  // --- Publisher participant ---
  pcl_executor_t* pub_exec = pcl_executor_create();
  auto* pub_t = pcl_shared_memory_transport_create(
      bus.c_str(), pub_id.c_str(), pub_exec);
  if (!pub_t) {
    sub_stop.store(true);
    sub_thread.join();
    pcl_shared_memory_transport_destroy(sub_t);
    pcl_executor_destroy(sub_exec);
    pcl_container_destroy(sub_c);
    pcl_executor_destroy(pub_exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }
  pcl_executor_set_transport(pub_exec,
      pcl_shared_memory_transport_get_transport(pub_t));

  PubSetup pub_setup{nullptr, ct};
  pcl_callbacks_t pub_cbs{};
  pub_cbs.on_configure = onConfigurePub;
  auto* pub_c = pcl_container_create("shmperf_pub", &pub_cbs, &pub_setup);
  pcl_container_configure(pub_c);
  pcl_container_activate(pub_c);
  pcl_executor_add(pub_exec, pub_c);

  // Measure encoded size
  std::string sample;
  cons::encodeObjectEvidence(ev, ct, &sample);
  size_t payload_bytes = sample.size();

  std::vector<double> latencies;
  latencies.reserve(kNMsgs);

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = recv.count.load(std::memory_order_acquire);
    auto t0  = Clock::now();
    cons::publishObjectEvidence(pub_setup.port, ev, ct);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    while (recv.count.load(std::memory_order_acquire) <= prev &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    if (recv.count.load(std::memory_order_acquire) > prev)
      latencies.push_back(toUs(t0, Clock::now()));
  }

  // Teardown
  sub_stop.store(true);
  sub_thread.join();

  pcl_shared_memory_transport_destroy(pub_t);
  pcl_executor_destroy(pub_exec);
  pcl_container_destroy(pub_c);

  pcl_shared_memory_transport_destroy(sub_t);
  pcl_executor_destroy(sub_exec);
  pcl_container_destroy(sub_c);

  return computeResult(label, kNMsgs, payload_bytes, latencies);
}

// ---------------------------------------------------------------------------
// 3. TCP SOCKET transport
// ---------------------------------------------------------------------------

/// Server-side state shared between the server thread and the test body.
struct SocketServerCtx {
  volatile uint16_t port_ready = 0;
  std::atomic<bool> transport_ready{false};
  std::atomic<bool> stop{false};
  bool              failed = false;
  RecvState*        recv   = nullptr;
};

static void socketServerThread(SocketServerCtx* ctx) {
  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) { ctx->failed = true; ctx->transport_ready.store(true); return; }

  auto* t = pcl_socket_transport_create_server_ex(0, exec, &ctx->port_ready);
  ctx->transport_ready.store(true);   // port_ready written before this
  if (!t) {
    ctx->failed = true;
    pcl_executor_destroy(exec);
    return;
  }
  pcl_executor_set_transport(exec, pcl_socket_transport_get_transport(t));

  pcl_callbacks_t sub_cbs{};
  sub_cbs.on_configure = onConfigureSub;
  auto* sub_c = pcl_container_create("skperf_sub", &sub_cbs, ctx->recv);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(exec, sub_c);

  while (!ctx->stop.load(std::memory_order_relaxed)) {
    pcl_executor_spin_once(exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  pcl_socket_transport_destroy(t);
  pcl_executor_destroy(exec);
  pcl_container_destroy(sub_c);
}

static PerfResult runSocket(const char* label, const char* ct,
                            const cons::ObjectDetail& ev) {
  RecvState recv{};
  recv.content_type = ct;

  SocketServerCtx ctx{};
  ctx.recv = &recv;

  std::thread server(socketServerThread, &ctx);

  // Wait for server to be listening (port_ready set before transport_ready)
  while (!ctx.transport_ready.load(std::memory_order_acquire))
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

  if (ctx.failed) {
    server.join();
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

  // Give accept() time to block
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  uint16_t port = ctx.port_ready;

  // Client side
  pcl_executor_t* client_exec = pcl_executor_create();
  auto* client_t = pcl_socket_transport_create_client(
      "127.0.0.1", port, client_exec);
  if (!client_t) {
    ctx.stop.store(true);
    server.join();
    pcl_executor_destroy(client_exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }
  pcl_executor_set_transport(client_exec,
      pcl_socket_transport_get_transport(client_t));

  PubSetup pub_setup{nullptr, ct};
  pcl_callbacks_t pub_cbs{};
  pub_cbs.on_configure = onConfigurePub;
  auto* pub_c = pcl_container_create("skperf_pub", &pub_cbs, &pub_setup);
  pcl_container_configure(pub_c);
  pcl_container_activate(pub_c);
  pcl_executor_add(client_exec, pub_c);

  // Small delay to let both endpoints complete the TCP handshake
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  std::string sample;
  cons::encodeObjectEvidence(ev, ct, &sample);
  size_t payload_bytes = sample.size();

  std::vector<double> latencies;
  latencies.reserve(kNMsgs);

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = recv.count.load(std::memory_order_acquire);
    auto t0  = Clock::now();
    cons::publishObjectEvidence(pub_setup.port, ev, ct);
    // spin client executor to flush send queue
    pcl_executor_spin_once(client_exec, 0);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    while (recv.count.load(std::memory_order_acquire) <= prev &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    if (recv.count.load(std::memory_order_acquire) > prev)
      latencies.push_back(toUs(t0, Clock::now()));
  }

  ctx.stop.store(true);
  pcl_socket_transport_destroy(client_t);
  pcl_executor_destroy(client_exec);
  pcl_container_destroy(pub_c);
  server.join();

  return computeResult(label, kNMsgs, payload_bytes, latencies);
}

// ---------------------------------------------------------------------------
// 4. gRPC benchmarks (conditional)
// ---------------------------------------------------------------------------

#if defined(PYRAMID_HAS_GRPC)

namespace provided = pyramid::services::tactical_objects::provided;

/// Forward declaration matching the generated grpc transport implementation.
namespace pyramid::services::tactical_objects::provided {

class GrpcServer {
public:
  GrpcServer();
  GrpcServer(GrpcServer&&) noexcept;
  GrpcServer& operator=(GrpcServer&&) noexcept;
  GrpcServer(const GrpcServer&) = delete;
  GrpcServer& operator=(const GrpcServer&) = delete;
  ~GrpcServer();
  bool started() const;
  void shutdown();
private:
  struct Impl;
  explicit GrpcServer(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
  friend GrpcServer buildGrpcServer(const std::string& listen_address,
                                    pcl_executor_t* executor);
};

GrpcServer buildGrpcServer(const std::string& listen_address,
                           pcl_executor_t* executor);

} // namespace pyramid::services::tactical_objects::provided

namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_base      = pyramid::data_model::base;
namespace proto_common    = pyramid::data_model::common;
namespace proto_tactical  = pyramid::data_model::tactical;

/// Returns a non-trivial protobuf request matching the ObjectDetail payload.
static proto_tactical::ObjectInterestRequirement makeGrpcRequest() {
  proto_tactical::ObjectInterestRequirement req;
  req.set_policy(proto_common::DATA_POLICY_OBTAIN);
  req.add_dimension(proto_tactical::BATTLE_DIMENSION_GROUND);
  req.add_dimension(proto_tactical::BATTLE_DIMENSION_AIR);
  auto* base = req.mutable_base();
  base->set_id("grpc-perf-interest-000000000001-abcdef");
  return req;
}

static pcl_status_t grpcServiceHandler(pcl_container_t*,
                                       const pcl_msg_t* req_msg,
                                       pcl_msg_t* resp_msg,
                                       pcl_svc_context_t*,
                                       void*) {
  proto_base::Identifier id;
  id.set_value("perf-resp-ok");
  auto bytes = id.SerializeAsString();
  auto* storage = static_cast<char*>(std::malloc(bytes.size()));
  if (!storage) return PCL_ERR_NOMEM;
  std::memcpy(storage, bytes.data(), bytes.size());
  resp_msg->data      = storage;
  resp_msg->size      = static_cast<uint32_t>(bytes.size());
  resp_msg->type_name = "application/protobuf";
  (void)req_msg;
  return PCL_OK;
}

static pcl_status_t grpcOnConfigure(pcl_container_t* c, void*) {
  auto* port = pcl_container_add_service(
      c, "object_of_interest.create_requirement",
      "application/protobuf", grpcServiceHandler, nullptr);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

static PerfResult runGrpc(const char* label, const std::string& address) {
  pcl_callbacks_t cbs{};
  cbs.on_configure = grpcOnConfigure;

  pcl_executor_t* exec = pcl_executor_create();
  auto* container = pcl_container_create("grpc_perf_svc", &cbs, nullptr);
  pcl_container_configure(container);
  pcl_container_activate(container);
  pcl_executor_add(exec, container);

  std::atomic<bool> spin_stop{false};
  std::thread spin_thread([&] {
    while (!spin_stop.load(std::memory_order_relaxed)) {
      pcl_executor_spin_once(exec, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  auto server = provided::buildGrpcServer(address, exec);
  if (!server.started()) {
    spin_stop.store(true);
    spin_thread.join();
    pcl_container_destroy(container);
    pcl_executor_destroy(exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

  auto channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
  auto stub    = proto_services::Object_Of_Interest_Service::NewStub(channel);

  auto grpc_req = makeGrpcRequest();
  // Estimate payload bytes from serialized request
  size_t payload_bytes = static_cast<size_t>(grpc_req.ByteSizeLong());

  std::vector<double> latencies;
  latencies.reserve(kNMsgsGrpc);

  for (int i = 0; i < kNMsgsGrpc; ++i) {
    proto_base::Identifier resp;
    grpc::ClientContext ctx;
    auto t0 = Clock::now();
    auto status = stub->CreateRequirement(&ctx, grpc_req, &resp);
    latencies.push_back(toUs(t0, Clock::now()));
    if (!status.ok()) break;
  }

  server.shutdown();
  spin_stop.store(true);
  spin_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(exec);

  return computeResult(label, kNMsgsGrpc, payload_bytes, latencies);
}

#endif  // PYRAMID_HAS_GRPC

// ---------------------------------------------------------------------------
// Test fixture — accumulates results, prints report in TearDownTestSuite
// ---------------------------------------------------------------------------

class BindingPerformanceTest : public ::testing::Test {
protected:
  static std::vector<PerfResult> results_;

  static void TearDownTestSuite() {
    printReport(results_);
  }

  static const cons::ObjectDetail& payload() {
    static const auto ev = makePayload();
    return ev;
  }
};

std::vector<PerfResult> BindingPerformanceTest::results_;

// ---------------------------------------------------------------------------
// Local (in-process) benchmarks
// ---------------------------------------------------------------------------

TEST_F(BindingPerformanceTest, Local_Json) {
  auto r = runLocal("local / json", "application/json", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Local_FlatBuffers) {
  auto r = runLocal("local / flatbuffers", "application/flatbuffers", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Local_Protobuf) {
  auto r = runLocal("local / protobuf", "application/protobuf", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// Shared-memory bus benchmarks
// ---------------------------------------------------------------------------

TEST_F(BindingPerformanceTest, Shmem_Json) {
  auto r = runShmem("shmem / json", "application/json", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Shmem_FlatBuffers) {
  auto r = runShmem("shmem / flatbuffers", "application/flatbuffers", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Shmem_Protobuf) {
  auto r = runShmem("shmem / protobuf", "application/protobuf", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// TCP socket benchmarks
// ---------------------------------------------------------------------------

TEST_F(BindingPerformanceTest, Socket_Json) {
  auto r = runSocket("socket / json", "application/json", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Socket_FlatBuffers) {
  auto r = runSocket("socket / flatbuffers", "application/flatbuffers", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Socket_Protobuf) {
  auto r = runSocket("socket / protobuf", "application/protobuf", payload());
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// gRPC benchmarks (compiled only when pyramid_grpc_transport is available)
// ---------------------------------------------------------------------------

#if defined(PYRAMID_HAS_GRPC)

TEST_F(BindingPerformanceTest, Grpc_Tcp) {
  auto r = runGrpc("grpc / tcp", "127.0.0.1:50171");
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}

#ifndef _WIN32
TEST_F(BindingPerformanceTest, Grpc_UnixSocket) {
  auto r = runGrpc("grpc / unix", "unix:///tmp/pcl_grpc_perf.sock");
  results_.push_back(r);
  std::printf("[PERF] %-28s  avg=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.avg_us, r.payload_bytes);
  SUCCEED();
}
#endif

#endif  // PYRAMID_HAS_GRPC
