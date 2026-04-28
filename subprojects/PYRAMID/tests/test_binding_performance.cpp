/// \file test_binding_performance.cpp
/// \brief Codec/transport performance benchmark via generated PYRAMID bindings.
///
/// For each transport (local in-process, shared-memory bus, TCP socket) and each
/// enabled codec (JSON, FlatBuffers, Protobuf), N messages of a non-trivial
/// ObjectDetail payload are transferred sequentially and the per-message latency
/// is recorded. When gRPC support is enabled, a unary protobuf round-trip is
/// benchmarked separately through the generated gRPC transport host.
/// A formatted report is printed at the end of the test suite.

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
#if defined(PYRAMID_HAS_PROTOBUF)
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"
#endif

#if defined(PYRAMID_HAS_GRPC)
#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace cons = pyramid::components::tactical_objects::services::consumed;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr int kNMsgs     = 500;   ///< messages per PCL combo
static constexpr int kNMsgsGrpc = 200;   ///< unary gRPC calls
static constexpr int kNCodecIters = 20000;   ///< iterations per codec microbench
static constexpr int kMsgTimeoutMs = 2000;

// ---------------------------------------------------------------------------
// Payload factory
// ---------------------------------------------------------------------------

static cons::ObjectDetail makePayload() {
  cons::ObjectDetail ev;
  ev.id            = "binding-perf-entity-000000000001-abcdef";
  ev.identity      = pyramid::domain_model::StandardIdentity::Hostile;
  ev.dimension     = pyramid::domain_model::BattleDimension::Ground;
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

static double threadCpuNowNs() {
#if defined(_WIN32)
  FILETIME creation_time{};
  FILETIME exit_time{};
  FILETIME kernel_time{};
  FILETIME user_time{};
  if (!GetThreadTimes(GetCurrentThread(), &creation_time, &exit_time,
                      &kernel_time, &user_time)) {
    return 0.0;
  }

  ULARGE_INTEGER kernel{};
  kernel.LowPart = kernel_time.dwLowDateTime;
  kernel.HighPart = kernel_time.dwHighDateTime;

  ULARGE_INTEGER user{};
  user.LowPart = user_time.dwLowDateTime;
  user.HighPart = user_time.dwHighDateTime;

  return static_cast<double>(kernel.QuadPart + user.QuadPart) * 100.0;
#elif defined(CLOCK_THREAD_CPUTIME_ID)
  struct timespec ts{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) {
    return 0.0;
  }
  return static_cast<double>(ts.tv_sec) * 1'000'000'000.0 +
         static_cast<double>(ts.tv_nsec);
#else
  return 0.0;
#endif
}

static double threadNowCycles() {
#if defined(_WIN32)
  ULONG64 cycles = 0;
  if (!QueryThreadCycleTime(GetCurrentThread(), &cycles)) {
    return 0.0;
  }
  return static_cast<double>(cycles);
#else
  return 0.0;
#endif
}

static double threadCpuElapsedUs(double start_ns) {
  return (threadCpuNowNs() - start_ns) / 1000.0;
}

// ---------------------------------------------------------------------------
// PerfResult and report
// ---------------------------------------------------------------------------

struct PerfStats {
  double avg_us = 0.0;
  double min_us = 0.0;
  double max_us = 0.0;
  double p50_us = 0.0;
  double p99_us = 0.0;
};

struct PerfResult {
  std::string label;
  int         attempted_msgs = 0;
  int         completed_msgs = 0;
  int         timed_out_msgs = 0;
  size_t      payload_bytes = 0;
  PerfStats   wall;
  PerfStats   cpu;
  bool        skipped     = false;
};

struct CodecPerfResult {
  std::string label;
  int         iterations = 0;
  size_t      payload_bytes = 0;
  double      wall_ns_per_op = 0.0;
  double      cpu_ns_per_op = 0.0;
  double      cycles_per_op = 0.0;
  bool        skipped = false;
};

static PerfStats computeStats(std::vector<double>& v) {
  PerfStats stats;
  if (v.empty()) {
    return stats;
  }

  std::sort(v.begin(), v.end());
  double sum = 0.0;
  for (double x : v) {
    sum += x;
  }

  stats.avg_us = sum / static_cast<double>(v.size());
  stats.min_us = v.front();
  stats.max_us = v.back();
  stats.p50_us = v[v.size() / 2];
  size_t p99_index = static_cast<size_t>(v.size() * 0.99);
  if (p99_index >= v.size()) {
    p99_index = v.size() - 1;
  }
  stats.p99_us = v[p99_index];
  return stats;
}

static PerfResult computeResult(const std::string& label,
                                int attempted,
                                int completed,
                                int timed_out,
                                size_t payload_bytes,
                                std::vector<double>& wall_samples,
                                std::vector<double>& cpu_samples) {
  PerfResult r;
  r.label = label;
  r.attempted_msgs = attempted;
  r.completed_msgs = completed;
  r.timed_out_msgs = timed_out;
  r.payload_bytes = payload_bytes;
  if (wall_samples.empty()) {
    r.skipped = true;
    return r;
  }

  r.wall = computeStats(wall_samples);
  r.cpu = computeStats(cpu_samples);
  return r;
}

static void printReport(const std::vector<PerfResult>& results) {
  // separator widths
  printf("\n");
  printf("============================================================"
         "==============================\n");
  printf(" PYRAMID Binding Performance Report\n");
  printf(" Payload: ObjectDetail  |  Iterations per combo: %d\n", kNMsgs);
  printf("============================================================"
         "==============================\n");
  printf("%-24s | %7s | %9s | %10s | %10s | %10s | %10s | %10s\n",
         "Transport/Codec", "Bytes", "Done", "AvgWall(us)",
         "MinWall(us)", "P99Wall(us)", "AvgThr(us)", "MinThr(us)");
  printf("%-24s-+-%7s-+-%9s-+-%10s-+-%10s-+-%10s-+-%10s-+-%10s\n",
         "------------------------", "-------", "---------", "----------",
         "----------", "----------", "----------", "----------");

  for (const auto& r : results) {
    if (r.skipped) {
      printf("%-28s | %9s   (skipped)\n", r.label.c_str(), "");
      continue;
    }
    printf("%-24s | %7zu | %3d/%-5d | %10.1f | %10.1f | %10.1f | %10.1f | %10.1f\n",
           r.label.c_str(), r.payload_bytes,
           r.completed_msgs, r.attempted_msgs,
           r.wall.avg_us, r.wall.min_us, r.wall.p99_us,
           r.cpu.avg_us, r.cpu.min_us);
  }
  printf("============================================================"
         "==============================\n\n");
  fflush(stdout);
}

static void printCodecReport(const std::vector<CodecPerfResult>& results) {
  printf("============================================================"
         "==============================\n");
  printf(" PYRAMID Codec Cost Report\n");
  printf(" Payload: ObjectDetail  |  Iterations per combo: %d\n", kNCodecIters);
  printf("============================================================"
         "==============================\n");
  printf("%-24s | %7s | %9s | %12s | %12s | %12s\n",
         "Codec/Op", "Bytes", "Iters", "Wall(ns/op)",
         "CPU(ns/op)", "Cycles/op");
  printf("%-24s-+-%7s-+-%9s-+-%12s-+-%12s-+-%12s\n",
         "------------------------", "-------", "---------", "------------",
         "------------", "------------");

  for (const auto& r : results) {
    if (r.skipped) {
      printf("%-24s | %7s   (skipped)\n", r.label.c_str(), "");
      continue;
    }
    printf("%-24s | %7zu | %9d | %12.1f | %12.1f | %12.1f\n",
           r.label.c_str(), r.payload_bytes, r.iterations,
           r.wall_ns_per_op, r.cpu_ns_per_op, r.cycles_per_op);
  }
  printf("============================================================"
         "==============================\n\n");
  fflush(stdout);
}

static std::atomic<uint64_t> s_codec_sink{0};

template <typename Fn>
static CodecPerfResult runCodecLoop(const char* label,
                                    size_t      payload_bytes,
                                    int         iterations,
                                    Fn&&        fn) {
  for (int i = 0; i < 256; ++i) {
    fn();
  }

  const auto wall_start = Clock::now();
  const double cpu_start = threadCpuNowNs();
  const double cycles_start = threadNowCycles();
  for (int i = 0; i < iterations; ++i) {
    fn();
  }
  const double wall_ns = std::chrono::duration<double, std::nano>(
      Clock::now() - wall_start).count();
  const double cpu_ns = threadCpuNowNs() - cpu_start;
  const double cycles = threadNowCycles() - cycles_start;

  CodecPerfResult result;
  result.label = label;
  result.iterations = iterations;
  result.payload_bytes = payload_bytes;
  result.wall_ns_per_op = wall_ns / static_cast<double>(iterations);
  result.cpu_ns_per_op = cpu_ns / static_cast<double>(iterations);
  result.cycles_per_op = cycles / static_cast<double>(iterations);
  return result;
}

// ---------------------------------------------------------------------------
// Shared receive state (used across transport benchmarks)
// ---------------------------------------------------------------------------

struct RecvState {
  std::atomic<int> count{0};
  std::mutex       mutex;
  std::condition_variable cv;
  const char*      content_type = nullptr;
};

static void recvCallback(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
  auto* s = static_cast<RecvState*>(ud);
  // Count bytes received (avoid constructing ObjectDetail by-value here:
  // pyramid::domain_model::tactical::ObjectDetail is defined in both the
  // generated C++ bindings and the protobuf code; constructing a local
  // of that type in a translation unit that links both causes an ODR
  // destructor collision.  Encoding cost is already captured by the
  // timing window that starts in publishObjectEvidence.)
  if (msg && msg->size > 0) {
    s->count.fetch_add(1, std::memory_order_release);
    s->cv.notify_all();
  }
}

static bool waitForDelivery(RecvState& recv,
                            int prev_count,
                            std::chrono::steady_clock::time_point deadline) {
  std::unique_lock<std::mutex> lock(recv.mutex);
  return recv.cv.wait_until(lock, deadline, [&] {
    return recv.count.load(std::memory_order_acquire) > prev_count;
  });
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

  std::vector<double> wall_latencies;
  std::vector<double> cpu_times;
  wall_latencies.reserve(kNMsgs);
  cpu_times.reserve(kNMsgs);
  int timed_out = 0;

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = recv.count.load(std::memory_order_acquire);
    auto t0 = Clock::now();
    double cpu_t0 = threadCpuNowNs();
    cons::publishObjectEvidence(pub_setup.port, ev, ct);

    bool delivered = false;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(exec, 0);
      if (recv.count.load(std::memory_order_acquire) > prev) {
        delivered = true;
        break;
      }
    }

    if (delivered) {
      wall_latencies.push_back(toUs(t0, Clock::now()));
      cpu_times.push_back(threadCpuElapsedUs(cpu_t0));
    } else {
      ++timed_out;
    }
  }

  pcl_executor_destroy(exec);
  pcl_container_destroy(sub_c);
  pcl_container_destroy(pub_c);

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies, cpu_times);
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
      pcl_executor_spin_once(sub_exec, 1);
      std::this_thread::yield();
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

  std::vector<double> wall_latencies;
  std::vector<double> cpu_times;
  wall_latencies.reserve(kNMsgs);
  cpu_times.reserve(kNMsgs);
  int timed_out = 0;

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = recv.count.load(std::memory_order_acquire);
    auto t0 = Clock::now();
    double cpu_t0 = threadCpuNowNs();
    cons::publishObjectEvidence(pub_setup.port, ev, ct);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    if (waitForDelivery(recv, prev, deadline)) {
      wall_latencies.push_back(toUs(t0, Clock::now()));
      cpu_times.push_back(threadCpuElapsedUs(cpu_t0));
    } else {
      ++timed_out;
    }
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

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies, cpu_times);
}

// ---------------------------------------------------------------------------
// 3. TCP SOCKET transport
// ---------------------------------------------------------------------------

/// Server-side state shared between the server thread and the test body.
struct SocketServerCtx {
  uint16_t          port_ready = 0;        // written by PCL before port_published release
  std::atomic<bool> port_published{false}; // release fence after port_ready is set
  std::atomic<bool> ready_for_msgs{false};
  std::atomic<bool> stop{false};
  std::atomic<bool> failed{false};
  RecvState*        recv = nullptr;
};

static void socketServerThread(SocketServerCtx* ctx) {
  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) { ctx->failed.store(true, std::memory_order_relaxed); return; }

  uint16_t assigned_port = 0;
  auto* t = pcl_socket_transport_create_server_ex(0, exec, &assigned_port);
  if (!t) {
    ctx->failed.store(true, std::memory_order_relaxed);
    pcl_executor_destroy(exec);
    return;
  }
  ctx->port_ready = assigned_port;
  ctx->port_published.store(true, std::memory_order_release);
  pcl_executor_set_transport(exec, pcl_socket_transport_get_transport(t));

  pcl_callbacks_t sub_cbs{};
  sub_cbs.on_configure = onConfigureSub;
  auto* sub_c = pcl_container_create("skperf_sub", &sub_cbs, ctx->recv);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(exec, sub_c);
  ctx->ready_for_msgs.store(true, std::memory_order_release);

  while (!ctx->stop.load(std::memory_order_relaxed)) {
    pcl_executor_spin_once(exec, 1);
    std::this_thread::yield();
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

  // Wait for server to publish the chosen port.
  auto ready_deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(kMsgTimeoutMs);
  while (!ctx.port_published.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < ready_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (ctx.failed.load(std::memory_order_relaxed) || ctx.port_ready == 0) {
    ctx.stop.store(true);
    server.join();
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

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

  auto msg_ready_deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(kMsgTimeoutMs);
  while (!ctx.ready_for_msgs.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < msg_ready_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!ctx.ready_for_msgs.load(std::memory_order_acquire)) {
    ctx.stop.store(true);
    pcl_socket_transport_destroy(client_t);
    pcl_executor_destroy(client_exec);
    pcl_container_destroy(pub_c);
    server.join();
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

  std::string sample;
  cons::encodeObjectEvidence(ev, ct, &sample);
  size_t payload_bytes = sample.size();

  std::vector<double> wall_latencies;
  std::vector<double> cpu_times;
  wall_latencies.reserve(kNMsgs);
  cpu_times.reserve(kNMsgs);
  int timed_out = 0;

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = recv.count.load(std::memory_order_acquire);
    auto t0 = Clock::now();
    double cpu_t0 = threadCpuNowNs();
    cons::publishObjectEvidence(pub_setup.port, ev, ct);
    // spin client executor to flush send queue
    pcl_executor_spin_once(client_exec, 0);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    if (waitForDelivery(recv, prev, deadline)) {
      wall_latencies.push_back(toUs(t0, Clock::now()));
      cpu_times.push_back(threadCpuElapsedUs(cpu_t0));
    } else {
      ++timed_out;
    }
  }

  ctx.stop.store(true);
  pcl_socket_transport_destroy(client_t);
  pcl_executor_destroy(client_exec);
  pcl_container_destroy(pub_c);
  server.join();

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies, cpu_times);
}

#if defined(PYRAMID_HAS_GRPC)

namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_tactical = pyramid::data_model::tactical;

namespace pyramid::components::tactical_objects::services::provided {

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

}  // namespace pyramid::components::tactical_objects::services::provided

namespace provided = pyramid::components::tactical_objects::services::provided;

constexpr const char* kGrpcCreateRequirementSvc =
    "object_of_interest.create_requirement";

static std::string serializeProto(const google::protobuf::MessageLite& message) {
  std::string bytes;
  if (!message.SerializeToString(&bytes)) {
    throw std::runtime_error("protobuf serialization failed");
  }
  return bytes;
}

static proto_tactical::ObjectInterestRequirement makeGrpcRequest() {
  proto_tactical::ObjectInterestRequirement request;
  auto* base = request.mutable_base()->mutable_base();
  base->mutable_id()->set_value("grpc-interest-42");
  base->mutable_source()->set_value("perf-client");
  request.set_source(proto_tactical::OBJECT_SOURCE_RADAR);
  request.set_policy(proto_common::DATA_POLICY_OBTAIN);
  request.add_dimension(proto_common::BATTLE_DIMENSION_GROUND);
  request.add_dimension(proto_common::BATTLE_DIMENSION_AIR);
  auto* point = request.mutable_point()->mutable_position();
  point->mutable_latitude()->set_radians(51.477811);
  point->mutable_longitude()->set_radians(-0.001475);
  return request;
}

static pcl_status_t grpcPerfHandler(pcl_container_t*,
                                    const pcl_msg_t* request,
                                    pcl_msg_t* response,
                                    pcl_svc_context_t*,
                                    void*) {
  if (request == nullptr || response == nullptr) {
    return PCL_ERR_INVALID;
  }

  proto_tactical::ObjectInterestRequirement decoded;
  if (!decoded.ParseFromArray(request->data, static_cast<int>(request->size))) {
    return PCL_ERR_INVALID;
  }

  proto_base::Identifier encoded;
  encoded.set_value("grpc-interest-42");
  const auto response_bytes = serializeProto(encoded);
  auto* storage = std::malloc(response_bytes.size());
  if (storage == nullptr) {
    return PCL_ERR_NOMEM;
  }
  std::memcpy(storage, response_bytes.data(), response_bytes.size());

  response->data = storage;
  response->size = static_cast<uint32_t>(response_bytes.size());
  response->type_name = "application/protobuf";
  return PCL_OK;
}

static pcl_status_t onConfigureGrpcPerf(pcl_container_t* container, void*) {
  auto* port = pcl_container_add_service(
      container, kGrpcCreateRequirementSvc, "application/protobuf",
      grpcPerfHandler, nullptr);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

static PerfResult runGrpc(const char* label, const std::string& address) {
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigureGrpcPerf;

  pcl_executor_t* executor = pcl_executor_create();
  auto* container = pcl_container_create("grpc_perf_service", &callbacks, nullptr);
  pcl_container_configure(container);
  pcl_container_activate(container);
  pcl_executor_add(executor, container);

  std::atomic<bool> spin_stop{false};
  std::thread executor_thread([&] {
    while (!spin_stop.load(std::memory_order_relaxed)) {
      pcl_executor_spin_once(executor, 0);
      std::this_thread::yield();
    }
  });

  auto host = provided::buildGrpcServer(address, executor);
  if (!host.started()) {
    spin_stop.store(true);
    executor_thread.join();
    pcl_container_destroy(container);
    pcl_executor_destroy(executor);
    PerfResult r;
    r.label = label;
    r.skipped = true;
    return r;
  }

  auto channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
  auto stub = proto_services::Object_Of_Interest_Service::NewStub(channel);

  const auto request = makeGrpcRequest();
  size_t payload_bytes = static_cast<size_t>(request.ByteSizeLong());

  std::vector<double> wall_latencies;
  std::vector<double> cpu_times;
  wall_latencies.reserve(kNMsgsGrpc);
  cpu_times.reserve(kNMsgsGrpc);
  int timed_out = 0;

  for (int i = 0; i < kNMsgsGrpc; ++i) {
    proto_base::Identifier response;
    grpc::ClientContext context;
    auto t0 = Clock::now();
    double cpu_t0 = threadCpuNowNs();
    const auto status = stub->CreateRequirement(&context, request, &response);
    if (status.ok()) {
      wall_latencies.push_back(toUs(t0, Clock::now()));
      cpu_times.push_back(threadCpuElapsedUs(cpu_t0));
    } else {
      ++timed_out;
    }
  }

  host.shutdown();
  spin_stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);

  return computeResult(label, kNMsgsGrpc,
                       static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies, cpu_times);
}

#endif

namespace tactical_codec = pyramid::domain_model::tactical;
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
#if defined(PYRAMID_HAS_PROTOBUF)
namespace protobuf_codec = pyramid::services::tactical_objects::protobuf_codec;
#endif

// ---------------------------------------------------------------------------
// Test fixture — accumulates results, prints report in TearDownTestSuite
// ---------------------------------------------------------------------------

class BindingPerformanceTest : public ::testing::Test {
protected:
  static std::vector<PerfResult> results_;
  static std::vector<CodecPerfResult> codec_results_;

  void SetUp() override {
#if defined(PYRAMID_HAS_PROTOBUF)
    // ODR collision: pyramid::data_model::tactical::ObjectDetail is defined as
    // both a plain C++ struct (pyramid_data_model_tactical_types.hpp) and a
    // protobuf Message (tactical.pb.h) in this TU, causing the linker to
    // resolve cons::ObjectDetail ctor/dtor to protobuf versions and crash at
    // makePayload()'s static initialisation.  Skip all benchmarks until the
    // ODR collision is resolved by isolating the protobuf codec into a separate TU.
    GTEST_SKIP() << "ODR collision between plain-struct and protobuf ObjectDetail; "
                    "fix pending (see PR #84)";
#endif
  }

  static void TearDownTestSuite() {
    printReport(results_);
    printCodecReport(codec_results_);
  }

  static const cons::ObjectDetail& payload() {
    static const auto ev = makePayload();
    return ev;
  }
};

std::vector<PerfResult> BindingPerformanceTest::results_;
std::vector<CodecPerfResult> BindingPerformanceTest::codec_results_;

TEST_F(BindingPerformanceTest, Codec_Json) {
  const auto& ev = payload();
  const auto sample = tactical_codec::toJson(ev);

  auto encode = runCodecLoop("codec / json encode", sample.size(), kNCodecIters, [&] {
    auto bytes = tactical_codec::toJson(ev);
    s_codec_sink.fetch_add(static_cast<uint64_t>(bytes.size()),
                           std::memory_order_relaxed);
  });
  auto decode = runCodecLoop("codec / json decode", sample.size(), kNCodecIters, [&] {
    auto decoded = tactical_codec::fromJson(
        sample, static_cast<cons::ObjectDetail*>(nullptr));
    s_codec_sink.fetch_add(static_cast<uint64_t>(decoded.id.size()),
                           std::memory_order_relaxed);
  });

  codec_results_.push_back(encode);
  codec_results_.push_back(decode);
  EXPECT_GT(encode.wall_ns_per_op, 0.0);
  EXPECT_GT(decode.wall_ns_per_op, 0.0);
}

TEST_F(BindingPerformanceTest, Codec_FlatBuffers) {
  const auto& ev = payload();
  const auto sample = flatbuffers_codec::toBinary(ev);

  auto encode = runCodecLoop("codec / flatbuffers enc", sample.size(), kNCodecIters, [&] {
    auto bytes = flatbuffers_codec::toBinary(ev);
    s_codec_sink.fetch_add(static_cast<uint64_t>(bytes.size()),
                           std::memory_order_relaxed);
  });
  auto decode = runCodecLoop("codec / flatbuffers dec", sample.size(), kNCodecIters, [&] {
    auto decoded = flatbuffers_codec::fromBinaryObjectDetail(sample);
    s_codec_sink.fetch_add(static_cast<uint64_t>(decoded.id.size()),
                           std::memory_order_relaxed);
  });

  codec_results_.push_back(encode);
  codec_results_.push_back(decode);
  EXPECT_GT(encode.wall_ns_per_op, 0.0);
  EXPECT_GT(decode.wall_ns_per_op, 0.0);
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Codec_Protobuf) {
  const auto& ev = payload();
  const auto sample = protobuf_codec::toBinary(ev);

  auto encode = runCodecLoop("codec / protobuf enc", sample.size(), kNCodecIters, [&] {
    auto bytes = protobuf_codec::toBinary(ev);
    s_codec_sink.fetch_add(static_cast<uint64_t>(bytes.size()),
                           std::memory_order_relaxed);
  });
  auto decode = runCodecLoop("codec / protobuf dec", sample.size(), kNCodecIters, [&] {
    auto decoded = protobuf_codec::fromBinaryObjectDetail(sample);
    s_codec_sink.fetch_add(static_cast<uint64_t>(decoded.id.size()),
                           std::memory_order_relaxed);
  });

  codec_results_.push_back(encode);
  codec_results_.push_back(decode);
  EXPECT_GT(encode.wall_ns_per_op, 0.0);
  EXPECT_GT(decode.wall_ns_per_op, 0.0);
}
#endif

// ---------------------------------------------------------------------------
// Local (in-process) benchmarks
// ---------------------------------------------------------------------------

TEST_F(BindingPerformanceTest, Local_Json) {
  auto r = runLocal("local / json", "application/json", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Local_FlatBuffers) {
  auto r = runLocal("local / flatbuffers", "application/flatbuffers", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Local_Protobuf) {
  auto r = runLocal("local / protobuf", "application/protobuf", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}
#endif

TEST_F(BindingPerformanceTest, Shmem_Json) {
  auto r = runShmem("shmem / json", "application/json", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}

TEST_F(BindingPerformanceTest, Shmem_FlatBuffers) {
  auto r = runShmem("shmem / flatbuffers", "application/flatbuffers", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Shmem_Protobuf) {
  auto r = runShmem("shmem / protobuf", "application/protobuf", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}
#endif

TEST_F(BindingPerformanceTest, Socket_Json) {
  auto r = runSocket("socket / json", "application/json", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Socket_Protobuf) {
  auto r = runSocket("socket / protobuf", "application/protobuf", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}
#endif

#if defined(PYRAMID_HAS_GRPC)
TEST_F(BindingPerformanceTest, Grpc_Tcp) {
  auto r = runGrpc("grpc / tcp", "127.0.0.1:50171");
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgsGrpc);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}
#endif

TEST_F(BindingPerformanceTest, Socket_FlatBuffers) {
  auto r = runSocket("socket / flatbuffers", "application/flatbuffers", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  std::printf("[PERF] %-24s  done=%3d/%d  avg_wall=%6.1f us  min_wall=%6.1f us  avg_thr=%6.1f us  payload=%zu B\n",
              r.label.c_str(), r.completed_msgs, r.attempted_msgs,
              r.wall.avg_us, r.wall.min_us, r.cpu.avg_us, r.payload_bytes);
  SUCCEED();
}
