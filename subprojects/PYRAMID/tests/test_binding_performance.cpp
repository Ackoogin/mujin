/// \file test_binding_performance.cpp
/// \brief Codec/transport performance benchmark via generated PYRAMID bindings.
///
/// For each transport (local in-process, shared-memory bus, TCP socket) and each
/// enabled codec (JSON, FlatBuffers, Protobuf), N unary create-requirement
/// service calls are transferred sequentially and the per-call latency is
/// recorded. When gRPC support is enabled, the same unary protobuf round-trip
/// is benchmarked through the generated gRPC transport host.
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

#include "pyramid_services_tactical_objects_provided.hpp"
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
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace svc = pyramid::components::tactical_objects::services::provided;

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

static svc::ObjectInterestRequirement makePayload() {
  svc::ObjectInterestRequirement request;
  request.base.id = "binding-perf-interest-000000000001-abcdef";
  request.base.source = "perf-client";
  request.source = pyramid::domain_model::ObjectSource::Radar;
  request.policy = pyramid::domain_model::DataPolicy::Obtain;
  request.dimension = {
      pyramid::domain_model::BattleDimension::Ground,
      pyramid::domain_model::BattleDimension::Air,
  };
  request.point = pyramid::domain_model::Point{{
      51.477811,
      -0.001475,
  }};
  return request;
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
  printf(" Payload: ObjectInterestRequirement -> Identifier"
         "  |  Iterations per combo: %d\n", kNMsgs);
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
  printf(" Payload: ObjectInterestRequirement"
         "  |  Iterations per combo: %d\n", kNCodecIters);
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
// Shared unary response state (used across transport benchmarks)
// ---------------------------------------------------------------------------

struct UnaryResponseState {
  std::atomic<int> count{0};
  std::mutex       mutex;
  std::condition_variable cv;
  bool             decoded = false;
  svc::Identifier  response_id;
};

static void createRequirementResponse(const pcl_msg_t* msg, void* ud) {
  auto* s = static_cast<UnaryResponseState*>(ud);
  s->decoded =
      svc::decodeObjectOfInterestCreateRequirementResponse(msg, &s->response_id);
  s->count.fetch_add(1, std::memory_order_release);
  s->cv.notify_all();
}

static bool waitForResponse(UnaryResponseState& state,
                            int prev_count,
                            std::chrono::steady_clock::time_point deadline) {
  std::unique_lock<std::mutex> lock(state.mutex);
  return state.cv.wait_until(lock, deadline, [&] {
    return state.count.load(std::memory_order_acquire) > prev_count;
  });
}

// ---------------------------------------------------------------------------
// Unary service helpers
// ---------------------------------------------------------------------------

struct UnaryServiceSetup {
  const char* content_type = nullptr;
  std::string response_buffer;
};

class UnaryServiceHandler : public svc::ServiceHandler {
public:
  svc::Identifier handleObjectOfInterestCreateRequirement(
      const svc::ObjectInterestRequirement&) override {
    return "binding-perf-interest-000000000001-abcdef";
  }
};

static pcl_status_t handleCreateRequirement(pcl_container_t*,
                                            const pcl_msg_t* request,
                                            pcl_msg_t* response,
                                            pcl_svc_context_t*,
                                            void* user_data) {
  if (response == nullptr) {
    return PCL_ERR_INVALID;
  }

  auto* setup = static_cast<UnaryServiceSetup*>(user_data);
  UnaryServiceHandler handler;
  void* response_buf = nullptr;
  size_t response_size = 0;
  svc::dispatch(handler, svc::ServiceChannel::ObjectOfInterestCreateRequirement,
                request ? request->data : nullptr,
                request ? request->size : 0u,
                request ? request->type_name : setup->content_type,
                &response_buf, &response_size);

  setup->response_buffer.clear();
  if (response_buf != nullptr && response_size > 0) {
    setup->response_buffer.assign(static_cast<const char*>(response_buf),
                                  response_size);
    std::free(response_buf);
  }

  response->data = setup->response_buffer.empty()
                       ? nullptr
                       : const_cast<char*>(setup->response_buffer.data());
  response->size = static_cast<uint32_t>(setup->response_buffer.size());
  response->type_name = request ? request->type_name : setup->content_type;
  return PCL_OK;
}

static pcl_status_t onConfigureUnaryService(pcl_container_t* container,
                                            void* ud) {
  auto* setup = static_cast<UnaryServiceSetup*>(ud);
  auto* port = pcl_container_add_service(
      container, svc::kSvcObjectOfInterestCreateRequirement, setup->content_type,
      handleCreateRequirement, ud);
  if (port == nullptr) {
    return PCL_ERR_NOMEM;
  }
  return pcl_port_set_route(port, PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE,
                            nullptr, 0u);
}

static bool addRemoteGateway(pcl_executor_t* exec, pcl_container_t* gateway) {
  if (gateway == nullptr) {
    return false;
  }
  if (pcl_container_configure(gateway) != PCL_OK) {
    return false;
  }
  if (pcl_container_activate(gateway) != PCL_OK) {
    return false;
  }
  return pcl_executor_add(exec, gateway) == PCL_OK;
}

static bool setRemoteEndpointRoute(pcl_executor_t* exec,
                                   const char* endpoint_name,
                                   const char* peer_id) {
  const char* peers[] = {peer_id};
  pcl_endpoint_route_t route{};
  route.endpoint_name = endpoint_name;
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_REMOTE;
  route.peer_ids = peers;
  route.peer_count = 1u;
  return pcl_executor_set_endpoint_route(exec, &route) == PCL_OK;
}

static size_t encodedPayloadSize(const svc::ObjectInterestRequirement& request,
                                 const char* content_type) {
  if (std::strcmp(content_type, "application/json") == 0) {
    return pyramid::domain_model::tactical::toJson(request).size();
  }
  if (std::strcmp(content_type, "application/flatbuffers") == 0) {
    return pyramid::services::tactical_objects::flatbuffers_codec::toBinary(
        request).size();
  }
#if defined(PYRAMID_HAS_PROTOBUF)
  if (std::strcmp(content_type, "application/protobuf") == 0) {
    return pyramid::services::tactical_objects::protobuf_codec::toBinary(
        request).size();
  }
#endif
  return 0u;
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
                           const svc::ObjectInterestRequirement& request) {
  pcl_executor_t* exec = pcl_executor_create();
  UnaryServiceSetup service_setup{ct};
  pcl_callbacks_t service_cbs{};
  service_cbs.on_configure = onConfigureUnaryService;
  auto* service_c =
      pcl_container_create("lperf_service", &service_cbs, &service_setup);
  pcl_container_configure(service_c);
  pcl_container_activate(service_c);
  pcl_executor_add(exec, service_c);

  const size_t payload_bytes = encodedPayloadSize(request, ct);

  std::vector<double> wall_latencies;
  std::vector<double> cpu_times;
  wall_latencies.reserve(kNMsgs);
  cpu_times.reserve(kNMsgs);
  int timed_out = 0;
  UnaryResponseState response_state{};

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = response_state.count.load(std::memory_order_acquire);
    auto t0 = Clock::now();
    double cpu_t0 = threadCpuNowNs();
    response_state.decoded = false;
    const auto rc = svc::invokeObjectOfInterestCreateRequirement(
        exec, request, createRequirementResponse, &response_state, nullptr, ct);

    bool delivered = false;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    while (rc == PCL_OK && std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(exec, 0);
      if (response_state.count.load(std::memory_order_acquire) > prev) {
        delivered = true;
        break;
      }
    }

    if (delivered && response_state.decoded) {
      wall_latencies.push_back(toUs(t0, Clock::now()));
      cpu_times.push_back(threadCpuElapsedUs(cpu_t0));
    } else {
      ++timed_out;
    }
  }

  pcl_executor_destroy(exec);
  pcl_container_destroy(service_c);

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies, cpu_times);
}

// ---------------------------------------------------------------------------
// 2. SHARED MEMORY bus transport
// ---------------------------------------------------------------------------

static PerfResult runShmem(const char* label, const char* ct,
                           const svc::ObjectInterestRequirement& request) {
  std::string bus = uniqueId("perfbus");
  std::string client_id = uniqueId("client");
  std::string server_id = uniqueId("server");

  // --- Server participant ---
  pcl_executor_t* server_exec = pcl_executor_create();
  auto* sub_t = pcl_shared_memory_transport_create(
      bus.c_str(), server_id.c_str(), server_exec);
  if (!sub_t) {
    pcl_executor_destroy(server_exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }
  pcl_executor_set_transport(server_exec,
      pcl_shared_memory_transport_get_transport(sub_t));
  if (!addRemoteGateway(
          server_exec,
          pcl_shared_memory_transport_gateway_container(sub_t))) {
    pcl_shared_memory_transport_destroy(sub_t);
    pcl_executor_destroy(server_exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

  UnaryServiceSetup service_setup{ct};
  pcl_callbacks_t service_cbs{};
  service_cbs.on_configure = onConfigureUnaryService;
  auto* service_c =
      pcl_container_create("shmperf_service", &service_cbs, &service_setup);
  pcl_container_configure(service_c);
  pcl_container_activate(service_c);
  pcl_executor_add(server_exec, service_c);

  std::atomic<bool> server_stop{false};
  std::thread server_thread([&] {
    while (!server_stop.load(std::memory_order_relaxed)) {
      pcl_executor_spin_once(server_exec, 1);
      std::this_thread::yield();
    }
  });

  // --- Client participant ---
  pcl_executor_t* client_exec = pcl_executor_create();
  auto* pub_t = pcl_shared_memory_transport_create(
      bus.c_str(), client_id.c_str(), client_exec);
  if (!pub_t) {
    server_stop.store(true);
    server_thread.join();
    pcl_shared_memory_transport_destroy(sub_t);
    pcl_executor_destroy(server_exec);
    pcl_container_destroy(service_c);
    pcl_executor_destroy(client_exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }
  pcl_executor_set_transport(client_exec,
      pcl_shared_memory_transport_get_transport(pub_t));

  const size_t payload_bytes = encodedPayloadSize(request, ct);

  std::vector<double> wall_latencies;
  std::vector<double> cpu_times;
  wall_latencies.reserve(kNMsgs);
  cpu_times.reserve(kNMsgs);
  int timed_out = 0;
  UnaryResponseState response_state{};

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = response_state.count.load(std::memory_order_acquire);
    auto t0 = Clock::now();
    double cpu_t0 = threadCpuNowNs();
    response_state.decoded = false;
    const auto rc = svc::invokeObjectOfInterestCreateRequirement(
        client_exec, request, createRequirementResponse, &response_state,
        nullptr, ct);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    while (rc == PCL_OK &&
           response_state.count.load(std::memory_order_acquire) <= prev &&
           std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(client_exec, 0);
      std::this_thread::yield();
    }
    if (response_state.count.load(std::memory_order_acquire) > prev &&
        response_state.decoded) {
      wall_latencies.push_back(toUs(t0, Clock::now()));
      cpu_times.push_back(threadCpuElapsedUs(cpu_t0));
    } else {
      ++timed_out;
    }
  }

  // Teardown
  server_stop.store(true);
  server_thread.join();

  pcl_shared_memory_transport_destroy(pub_t);
  pcl_executor_destroy(client_exec);

  pcl_shared_memory_transport_destroy(sub_t);
  pcl_executor_destroy(server_exec);
  pcl_container_destroy(service_c);

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies, cpu_times);
}

// ---------------------------------------------------------------------------
// 3. TCP SOCKET transport
// ---------------------------------------------------------------------------

/// Server-side state shared between the server thread and the test body.
struct SocketServerCtx {
  uint16_t          port_ready = 0;
  std::atomic<bool> ready_for_msgs{false};
  std::atomic<bool> stop{false};
  std::atomic<bool> failed{false};
  UnaryServiceSetup* service = nullptr;
};

static uint16_t pickSocketPort() {
#if defined(_WIN32)
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    return 0;
  }
  SOCKET tmp = socket(AF_INET, SOCK_STREAM, 0);
  if (tmp == INVALID_SOCKET) {
    return 0;
  }
#else
  int tmp = socket(AF_INET, SOCK_STREAM, 0);
  if (tmp < 0) {
    return 0;
  }
#endif

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;

  if (bind(tmp, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
#if defined(_WIN32)
    closesocket(tmp);
#else
    close(tmp);
#endif
    return 0;
  }

#if defined(_WIN32)
  int len = static_cast<int>(sizeof(addr));
#else
  socklen_t len = sizeof(addr);
#endif
  if (getsockname(tmp, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
#if defined(_WIN32)
    closesocket(tmp);
#else
    close(tmp);
#endif
    return 0;
  }

  const uint16_t port = ntohs(addr.sin_port);
#if defined(_WIN32)
  closesocket(tmp);
#else
  close(tmp);
#endif
  return port;
}

static void socketServerThread(SocketServerCtx* ctx) {
  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) { ctx->failed.store(true, std::memory_order_relaxed); return; }

  auto* t = pcl_socket_transport_create_server(ctx->port_ready, exec);
  if (!t) {
    ctx->failed.store(true, std::memory_order_relaxed);
    pcl_executor_destroy(exec);
    return;
  }
  if (pcl_socket_transport_set_peer_id(t, "client") != PCL_OK) {
    ctx->failed.store(true, std::memory_order_relaxed);
    pcl_socket_transport_destroy(t);
    pcl_executor_destroy(exec);
    return;
  }
  pcl_executor_set_transport(exec, pcl_socket_transport_get_transport(t));
  pcl_executor_register_transport(exec, "client",
      pcl_socket_transport_get_transport(t));
  if (!addRemoteGateway(exec, pcl_socket_transport_gateway_container(t))) {
    ctx->failed.store(true, std::memory_order_relaxed);
    pcl_socket_transport_destroy(t);
    pcl_executor_destroy(exec);
    return;
  }

  pcl_callbacks_t service_cbs{};
  service_cbs.on_configure = onConfigureUnaryService;
  auto* service_c =
      pcl_container_create("skperf_service", &service_cbs, ctx->service);
  pcl_container_configure(service_c);
  pcl_container_activate(service_c);
  pcl_executor_add(exec, service_c);
  ctx->ready_for_msgs.store(true, std::memory_order_release);

  while (!ctx->stop.load(std::memory_order_relaxed)) {
    pcl_executor_spin_once(exec, 1);
    std::this_thread::yield();
  }

  pcl_socket_transport_destroy(t);
  pcl_executor_destroy(exec);
  pcl_container_destroy(service_c);
}

static PerfResult runSocket(const char* label, const char* ct,
                            const svc::ObjectInterestRequirement& request) {
  UnaryServiceSetup service_setup{ct};

  SocketServerCtx ctx{};
  ctx.service = &service_setup;
  ctx.port_ready = pickSocketPort();
  if (ctx.port_ready == 0) {
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

  std::thread server(socketServerThread, &ctx);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (ctx.failed.load(std::memory_order_relaxed)) {
    ctx.stop.store(true);
    server.join();
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

  // Client side
  pcl_executor_t* client_exec = pcl_executor_create();
  pcl_socket_client_opts_t client_opts{};
  client_opts.connect_timeout_ms = kMsgTimeoutMs;
  client_opts.max_retries = 8u;
  auto* client_t = pcl_socket_transport_create_client_ex(
      "127.0.0.1", ctx.port_ready, client_exec, &client_opts);
  if (!client_t) {
    ctx.stop.store(true);
    server.join();
    pcl_executor_destroy(client_exec);
    PerfResult r; r.label = label; r.skipped = true; return r;
  }
  if (pcl_socket_transport_set_peer_id(client_t, "server") != PCL_OK) {
    ctx.stop.store(true);
    pcl_socket_transport_destroy(client_t);
    pcl_executor_destroy(client_exec);
    server.join();
    PerfResult r; r.label = label; r.skipped = true; return r;
  }
  pcl_executor_set_transport(client_exec,
      pcl_socket_transport_get_transport(client_t));
  pcl_executor_register_transport(client_exec, "server",
      pcl_socket_transport_get_transport(client_t));
  if (!setRemoteEndpointRoute(client_exec,
                              svc::kSvcObjectOfInterestCreateRequirement,
                              "server")) {
    ctx.stop.store(true);
    pcl_socket_transport_destroy(client_t);
    pcl_executor_destroy(client_exec);
    server.join();
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

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
    server.join();
    PerfResult r; r.label = label; r.skipped = true; return r;
  }

  const size_t payload_bytes = encodedPayloadSize(request, ct);

  std::vector<double> wall_latencies;
  std::vector<double> cpu_times;
  wall_latencies.reserve(kNMsgs);
  cpu_times.reserve(kNMsgs);
  int timed_out = 0;
  UnaryResponseState response_state{};

  for (int i = 0; i < kNMsgs; ++i) {
    int prev = response_state.count.load(std::memory_order_acquire);
    auto t0 = Clock::now();
    double cpu_t0 = threadCpuNowNs();
    response_state.decoded = false;
    const auto rc = svc::invokeObjectOfInterestCreateRequirement(
        client_exec, request, createRequirementResponse, &response_state,
        nullptr, ct);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(kMsgTimeoutMs);
    while (rc == PCL_OK &&
           response_state.count.load(std::memory_order_acquire) <= prev &&
           std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(client_exec, 0);
      std::this_thread::yield();
    }
    if (response_state.count.load(std::memory_order_acquire) > prev &&
        response_state.decoded) {
      wall_latencies.push_back(toUs(t0, Clock::now()));
      cpu_times.push_back(threadCpuElapsedUs(cpu_t0));
    } else {
      ++timed_out;
    }
  }

  ctx.stop.store(true);
  pcl_socket_transport_destroy(client_t);
  pcl_executor_destroy(client_exec);
  server.join();

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies, cpu_times);
}

#if defined(PYRAMID_HAS_GRPC)

namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_tactical = pyramid::data_model::tactical;

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

  static void TearDownTestSuite() {
    printReport(results_);
    printCodecReport(codec_results_);
  }

  static const svc::ObjectInterestRequirement& payload() {
    static const auto request = makePayload();
    return request;
  }
};

std::vector<PerfResult> BindingPerformanceTest::results_;
std::vector<CodecPerfResult> BindingPerformanceTest::codec_results_;

TEST_F(BindingPerformanceTest, Codec_Json) {
  const auto& request = payload();
  const auto sample = tactical_codec::toJson(request);

  auto encode = runCodecLoop("codec / json encode", sample.size(), kNCodecIters, [&] {
    auto bytes = tactical_codec::toJson(request);
    s_codec_sink.fetch_add(static_cast<uint64_t>(bytes.size()),
                           std::memory_order_relaxed);
  });
  auto decode = runCodecLoop("codec / json decode", sample.size(), kNCodecIters, [&] {
    auto decoded = tactical_codec::fromJson(
        sample, static_cast<svc::ObjectInterestRequirement*>(nullptr));
    s_codec_sink.fetch_add(static_cast<uint64_t>(decoded.base.id.size()),
                           std::memory_order_relaxed);
  });

  codec_results_.push_back(encode);
  codec_results_.push_back(decode);
  EXPECT_GT(encode.wall_ns_per_op, 0.0);
  EXPECT_GT(decode.wall_ns_per_op, 0.0);
}

TEST_F(BindingPerformanceTest, Codec_FlatBuffers) {
  const auto& request = payload();
  const auto sample = flatbuffers_codec::toBinary(request);

  auto encode = runCodecLoop("codec / flatbuffers enc", sample.size(), kNCodecIters, [&] {
    auto bytes = flatbuffers_codec::toBinary(request);
    s_codec_sink.fetch_add(static_cast<uint64_t>(bytes.size()),
                           std::memory_order_relaxed);
  });
  auto decode = runCodecLoop("codec / flatbuffers dec", sample.size(), kNCodecIters, [&] {
    auto decoded = flatbuffers_codec::fromBinaryObjectInterestRequirement(sample);
    s_codec_sink.fetch_add(static_cast<uint64_t>(decoded.base.id.size()),
                           std::memory_order_relaxed);
  });

  codec_results_.push_back(encode);
  codec_results_.push_back(decode);
  EXPECT_GT(encode.wall_ns_per_op, 0.0);
  EXPECT_GT(decode.wall_ns_per_op, 0.0);
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Codec_Protobuf) {
  const auto& request = payload();
  const auto sample = protobuf_codec::toBinary(request);

  auto encode = runCodecLoop("codec / protobuf enc", sample.size(), kNCodecIters, [&] {
    auto bytes = protobuf_codec::toBinary(request);
    s_codec_sink.fetch_add(static_cast<uint64_t>(bytes.size()),
                           std::memory_order_relaxed);
  });
  auto decode = runCodecLoop("codec / protobuf dec", sample.size(), kNCodecIters, [&] {
    auto decoded = protobuf_codec::fromBinaryObjectInterestRequirement(sample);
    s_codec_sink.fetch_add(static_cast<uint64_t>(decoded.base.id.size()),
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
