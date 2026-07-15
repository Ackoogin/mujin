/// \file test_binding_performance.cpp
/// \brief Codec/transport performance benchmark via generated PYRAMID bindings.
///
/// Local in-process, shared-memory, and TCP transports run sequential unary
/// calls for every enabled general-purpose codec. UDP runs matched one-way
/// pub/sub delivery. Completion is timestamped in the response or subscriber
/// callback while client and server executors run on independent threads. The
/// measured interval therefore does not include a benchmark-side executor
/// polling loop.
///
/// Every transport has a raw-PCL row, which moves pre-encoded bytes through the
/// PCL transport, and a PYRAMID row, which uses the generated typed facade and
/// codec plugins. When gRPC support is enabled, a raw generated-protobuf gRPC
/// service is compared with the same gRPC service backed by a PYRAMID port.
/// A formatted report is printed at the end of the test suite.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_transport_socket.h>
#include <pcl/pcl_transport_shared_memory.h>
#include <pcl/pcl_transport_udp.h>
#include <pcl/pcl_types.h>
}

#if defined(PYRAMID_HAS_OMS_JSON)
#include <pyramid/oms_json_types.h>
#endif

#include "pyramid_services_tactical_objects_provided.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#if defined(PYRAMID_HAS_PROTOBUF)
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"
#endif

#if defined(PYRAMID_HAS_GRPC)
#include "pyramid/components/pyramid.components.tactical_objects.services.provided.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
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

static constexpr int kNMsgs     = 500;   ///< measured messages per PCL combo
static constexpr int kNMsgsGrpc = 200;   ///< measured unary gRPC calls
static constexpr int kNWarmupMsgs = 50;  ///< untimed warm-up messages
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

using Clock = std::chrono::steady_clock;

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
                                std::vector<double>& wall_samples) {
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
  return r;
}

static void printReport(const std::vector<PerfResult>& results,
                        const char* title,
                        const char* payload,
                        const char* metric) {
  // separator widths
  printf("\n");
  printf("============================================================"
         "==============================\n");
  printf(" %s (callback-timestamped)\n", title);
  printf(" Payload: %s  |  Iterations per combo: %d\n", payload, kNMsgs);
  printf("============================================================"
         "==============================\n");
  printf("%-32s | %7s | %9s | %10s | %10s | %10s | %10s\n",
         "Layer / transport / codec", "Bytes", "Done", metric,
         "Minimum(us)", "P50(us)", "P99(us)");
  printf("%-32s-+-%7s-+-%9s-+-%10s-+-%10s-+-%10s-+-%10s\n",
         "--------------------------------", "-------", "---------", "----------",
         "----------", "----------", "----------");

  for (const auto& r : results) {
    if (r.skipped) {
      printf("%-36s | %9s   (skipped)\n", r.label.c_str(), "");
      continue;
    }
    printf("%-32s | %7zu | %3d/%-5d | %10.1f | %10.1f | %10.1f | %10.1f\n",
           r.label.c_str(), r.payload_bytes,
           r.completed_msgs, r.attempted_msgs,
           r.wall.avg_us, r.wall.min_us, r.wall.p50_us, r.wall.p99_us);
  }
  printf("============================================================"
         "==============================\n\n");
  fflush(stdout);
}

static void printCodecReport(const std::vector<CodecPerfResult>& results) {
  printf("============================================================"
         "==============================\n");
  printf(" PYRAMID Codec Cost Report\n");
  printf(" Payload varies by row; Bytes is the encoded size"
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
// Callback-timestamped unary response state
// ---------------------------------------------------------------------------

struct UnaryResponseState {
  std::mutex       mutex;
  std::condition_variable cv;
  bool             complete = false;
  bool             decoded = false;
  Clock::time_point started_at{};
  Clock::time_point completed_at{};
  svc::Identifier  response_id;
};

static void createRequirementResponse(const pcl_msg_t* msg, void* ud) {
  auto* s = static_cast<UnaryResponseState*>(ud);
  const bool decoded =
      svc::decodeObjectOfInterestCreateRequirementResponse(msg, &s->response_id);
  const auto completed_at = Clock::now();
  {
    std::lock_guard<std::mutex> lock(s->mutex);
    s->decoded = decoded;
    s->completed_at = completed_at;
    s->complete = true;
  }
  s->cv.notify_all();
}

static void rawResponse(const pcl_msg_t* msg, void* ud) {
  auto* s = static_cast<UnaryResponseState*>(ud);
  const bool decoded = msg != nullptr && msg->data != nullptr && msg->size > 0u;
  const auto completed_at = Clock::now();
  {
    std::lock_guard<std::mutex> lock(s->mutex);
    s->decoded = decoded;
    s->completed_at = completed_at;
    s->complete = true;
  }
  s->cv.notify_all();
}

static void prepareResponse(UnaryResponseState& state) {
  std::lock_guard<std::mutex> lock(state.mutex);
  state.complete = false;
  state.decoded = false;
  state.started_at = Clock::time_point{};
  state.completed_at = Clock::time_point{};
}

static bool waitForResponse(UnaryResponseState& state,
                            Clock::time_point deadline,
                            Clock::time_point* started_at,
                            Clock::time_point* completed_at) {
  std::unique_lock<std::mutex> lock(state.mutex);
  if (!state.cv.wait_until(lock, deadline, [&] { return state.complete; })) {
    return false;
  }
  *started_at = state.started_at;
  *completed_at = state.completed_at;
  return state.decoded;
}

class ExecutorRunner {
public:
  explicit ExecutorRunner(pcl_executor_t* executor) : executor_(executor) {
    thread_ = std::thread([this] {
      while (!stop_.load(std::memory_order_acquire)) {
        std::function<void()> command;
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          if (!commands_.empty()) {
            command = std::move(commands_.front());
            commands_.pop_front();
          }
        }
        if (command) {
          command();
        }
        pcl_executor_spin_once(executor_, 0);
        std::this_thread::yield();
      }
    });
  }

  ~ExecutorRunner() {
    stop();
  }

  void stop() {
    if (!thread_.joinable()) {
      return;
    }
    stop_.store(true, std::memory_order_release);
    thread_.join();
  }

  void post(std::function<void()> command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    commands_.push_back(std::move(command));
  }

private:
  pcl_executor_t* executor_ = nullptr;
  std::atomic<bool> stop_{false};
  std::mutex command_mutex_;
  std::deque<std::function<void()>> commands_;
  std::thread thread_;
};

// ---------------------------------------------------------------------------
// Unary service helpers
// ---------------------------------------------------------------------------

struct UnaryServiceSetup {
  const char* content_type = nullptr;
  bool use_typed_binding = true;
  std::string raw_response;
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
  if (setup->use_typed_binding) {
    UnaryServiceHandler handler;
    void* response_buf = nullptr;
    size_t response_size = 0;
    svc::dispatch(handler,
                  svc::ServiceChannel::ObjectOfInterestCreateRequirement,
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
  } else {
    setup->response_buffer = setup->raw_response;
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

static std::string encodedPayload(
    const svc::ObjectInterestRequirement& request,
    const char* content_type) {
  if (std::strcmp(content_type, "application/json") == 0) {
    return pyramid::domain_model::tactical::toJson(request);
  }
  if (std::strcmp(content_type, "application/flatbuffers") == 0) {
    return pyramid::services::tactical_objects::flatbuffers_codec::toBinary(
        request);
  }
#if defined(PYRAMID_HAS_PROTOBUF)
  if (std::strcmp(content_type, "application/protobuf") == 0) {
    return pyramid::services::tactical_objects::protobuf_codec::toBinary(
        request);
  }
#endif
  return {};
}

static std::string rawResponseWithMatchingSize(const char* content_type) {
  const svc::Identifier response =
      "binding-perf-interest-000000000001-abcdef";
  size_t size = response.size();
  if (std::strcmp(content_type, "application/json") == 0) {
    // Identifier is a string alias, so its JSON representation adds quotes.
    size += 2u;
  }
  if (std::strcmp(content_type, "application/flatbuffers") == 0) {
    size = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(
        response).size();
  }
#if defined(PYRAMID_HAS_PROTOBUF)
  if (std::strcmp(content_type, "application/protobuf") == 0) {
    size = pyramid::services::tactical_objects::protobuf_codec::toBinary(
        response).size();
  }
#endif
  return std::string(size, 'r');
}

static pcl_status_t invokeRaw(pcl_executor_t* executor,
                              const std::string& request,
                              const char* content_type,
                              UnaryResponseState* state) {
  pcl_msg_t msg{};
  msg.data = request.data();
  msg.size = static_cast<uint32_t>(request.size());
  msg.type_name = content_type;
  return pcl_executor_invoke_async(
      executor, svc::kSvcObjectOfInterestCreateRequirement, &msg,
      rawResponse, state);
}

template <typename Invoke>
static void runUnarySamples(int measured_messages,
                            ExecutorRunner* executor_runner,
                            Invoke&& invoke,
                            UnaryResponseState& state,
                            std::vector<double>* wall_latencies,
                            int* timed_out) {
  for (int i = -kNWarmupMsgs; i < measured_messages; ++i) {
    prepareResponse(state);
    const auto submit = [&] {
      {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.started_at = Clock::now();
      }
      const auto rc = invoke();
      if (rc != PCL_OK) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.decoded = false;
        state.completed_at = Clock::now();
        state.complete = true;
        state.cv.notify_all();
      }
    };
    if (executor_runner != nullptr) {
      executor_runner->post(submit);
    } else {
      submit();
    }
    Clock::time_point started_at;
    Clock::time_point completed_at;
    const bool delivered = waitForResponse(
        state, Clock::now() + std::chrono::milliseconds(kMsgTimeoutMs),
        &started_at, &completed_at);
    if (i < 0) {
      continue;
    }
    if (delivered) {
      wall_latencies->push_back(toUs(started_at, completed_at));
    } else {
      ++*timed_out;
    }
  }
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
                           const svc::ObjectInterestRequirement& request,
                           bool use_typed_binding = true) {
  pcl_executor_t* exec = pcl_executor_create();
  UnaryServiceSetup service_setup;
  service_setup.content_type = ct;
  service_setup.use_typed_binding = use_typed_binding;
  service_setup.raw_response = rawResponseWithMatchingSize(ct);
  pcl_callbacks_t service_cbs{};
  service_cbs.on_configure = onConfigureUnaryService;
  auto* service_c =
      pcl_container_create("lperf_service", &service_cbs, &service_setup);
  pcl_container_configure(service_c);
  pcl_container_activate(service_c);
  pcl_executor_add(exec, service_c);

  const size_t payload_bytes = encodedPayloadSize(request, ct);

  std::vector<double> wall_latencies;
  wall_latencies.reserve(kNMsgs);
  int timed_out = 0;
  UnaryResponseState response_state{};
  const auto raw_request = encodedPayload(request, ct);
  runUnarySamples(kNMsgs, nullptr, [&] {
    if (use_typed_binding) {
      return svc::invokeObjectOfInterestCreateRequirement(
          exec, request, createRequirementResponse, &response_state, nullptr,
          ct);
    }
    return invokeRaw(exec, raw_request, ct, &response_state);
  }, response_state, &wall_latencies, &timed_out);

  pcl_executor_destroy(exec);
  pcl_container_destroy(service_c);

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies);
}

// ---------------------------------------------------------------------------
// 2. SHARED MEMORY bus transport
// ---------------------------------------------------------------------------

static PerfResult runShmem(const char* label, const char* ct,
                           const svc::ObjectInterestRequirement& request,
                           bool use_typed_binding = true) {
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

  UnaryServiceSetup service_setup;
  service_setup.content_type = ct;
  service_setup.use_typed_binding = use_typed_binding;
  service_setup.raw_response = rawResponseWithMatchingSize(ct);
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
      pcl_executor_spin_once(server_exec, 0);
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
  wall_latencies.reserve(kNMsgs);
  int timed_out = 0;
  UnaryResponseState response_state{};
  const auto raw_request = encodedPayload(request, ct);
  ExecutorRunner client_runner(client_exec);
  runUnarySamples(kNMsgs, &client_runner, [&] {
    if (use_typed_binding) {
      return svc::invokeObjectOfInterestCreateRequirement(
          client_exec, request, createRequirementResponse, &response_state,
          nullptr, ct);
    }
    return invokeRaw(client_exec, raw_request, ct, &response_state);
  }, response_state, &wall_latencies, &timed_out);

  // Teardown
  server_stop.store(true);
  server_thread.join();
  client_runner.stop();

  pcl_shared_memory_transport_destroy(pub_t);
  pcl_executor_destroy(client_exec);

  pcl_shared_memory_transport_destroy(sub_t);
  pcl_executor_destroy(server_exec);
  pcl_container_destroy(service_c);

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies);
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

static uint16_t pickUdpPort() {
#if defined(_WIN32)
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    return 0;
  }
  SOCKET tmp = socket(AF_INET, SOCK_DGRAM, 0);
  if (tmp == INVALID_SOCKET) {
    return 0;
  }
#else
  int tmp = socket(AF_INET, SOCK_DGRAM, 0);
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
    pcl_executor_spin_once(exec, 0);
    std::this_thread::yield();
  }

  pcl_socket_transport_destroy(t);
  pcl_executor_destroy(exec);
  pcl_container_destroy(service_c);
}

static PerfResult runSocket(const char* label, const char* ct,
                            const svc::ObjectInterestRequirement& request,
                            bool use_typed_binding = true) {
  UnaryServiceSetup service_setup;
  service_setup.content_type = ct;
  service_setup.use_typed_binding = use_typed_binding;
  service_setup.raw_response = rawResponseWithMatchingSize(ct);

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
  wall_latencies.reserve(kNMsgs);
  int timed_out = 0;
  UnaryResponseState response_state{};
  const auto raw_request = encodedPayload(request, ct);
  ExecutorRunner client_runner(client_exec);
  runUnarySamples(kNMsgs, &client_runner, [&] {
    if (use_typed_binding) {
      return svc::invokeObjectOfInterestCreateRequirement(
          client_exec, request, createRequirementResponse, &response_state,
          nullptr, ct);
    }
    return invokeRaw(client_exec, raw_request, ct, &response_state);
  }, response_state, &wall_latencies, &timed_out);

  ctx.stop.store(true);
  client_runner.stop();
  pcl_socket_transport_destroy(client_t);
  pcl_executor_destroy(client_exec);
  server.join();

  return computeResult(label, kNMsgs, static_cast<int>(wall_latencies.size()),
                       timed_out, payload_bytes, wall_latencies);
}

// ---------------------------------------------------------------------------
// 4. UDP pub/sub transport (one-way application delivery)
// ---------------------------------------------------------------------------

struct PubsubResponseState {
  std::mutex mutex;
  std::condition_variable cv;
  bool complete = false;
  bool decoded = false;
  Clock::time_point started_at{};
  Clock::time_point completed_at{};
  bool use_typed_binding = true;
};

static void entityMatchesResponse(pcl_container_t*, const pcl_msg_t* msg,
                                  void* user_data) {
  auto* state = static_cast<PubsubResponseState*>(user_data);
  bool decoded = msg != nullptr && msg->data != nullptr && msg->size > 0u;
  if (decoded && state->use_typed_binding) {
    std::vector<svc::ObjectMatch> value;
    decoded = svc::decodeEntityMatches(msg, &value) && !value.empty();
  }
  const auto completed_at = Clock::now();
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->decoded = decoded;
    state->completed_at = completed_at;
    state->complete = true;
  }
  state->cv.notify_all();
}

static PerfResult runUdp(const char* label, const char* content_type,
                         bool use_typed_binding) {
  const uint16_t receive_port = pickUdpPort();
  const uint16_t send_port = pickUdpPort();
  if (receive_port == 0 || send_port == 0 || receive_port == send_port) {
    PerfResult result;
    result.label = label;
    result.skipped = true;
    return result;
  }

  auto* receive_executor = pcl_executor_create();
  auto* send_executor = pcl_executor_create();
  auto* receiver = pcl_udp_transport_create(
      receive_port, "127.0.0.1", send_port, receive_executor);
  auto* sender = pcl_udp_transport_create(
      send_port, "127.0.0.1", receive_port, send_executor);
  if (receiver == nullptr || sender == nullptr) {
    pcl_udp_transport_destroy(sender);
    pcl_udp_transport_destroy(receiver);
    pcl_executor_destroy(send_executor);
    pcl_executor_destroy(receive_executor);
    PerfResult result;
    result.label = label;
    result.skipped = true;
    return result;
  }

  pcl_udp_transport_set_peer_id(receiver, "sender");
  pcl_udp_transport_set_peer_id(sender, "receiver");
  pcl_executor_register_transport(
      receive_executor, "sender", pcl_udp_transport_get_transport(receiver));
  pcl_executor_register_transport(
      send_executor, "receiver", pcl_udp_transport_get_transport(sender));

  PubsubResponseState state;
  state.use_typed_binding = use_typed_binding;
  pcl_callbacks_t subscriber_callbacks{};
  struct SubscriberSetup {
    PubsubResponseState* state;
    const char* content_type;
  } subscriber_setup{&state, content_type};
  subscriber_callbacks.on_configure = [](pcl_container_t* container,
                                         void* user_data) -> pcl_status_t {
    auto* setup = static_cast<SubscriberSetup*>(user_data);
    auto* port = svc::subscribeEntityMatches(
        container, entityMatchesResponse, setup->state, setup->content_type);
    if (port == nullptr) {
      return PCL_ERR_NOMEM;
    }
    const char* peers[] = {"sender"};
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1u);
  };
  auto* subscriber = pcl_container_create(
      "udp_perf_subscriber", &subscriber_callbacks, &subscriber_setup);
  pcl_container_configure(subscriber);
  pcl_container_activate(subscriber);
  pcl_executor_add(receive_executor, subscriber);

  pcl_port_t* publisher_port = nullptr;
  pcl_callbacks_t publisher_callbacks{};
  publisher_callbacks.on_configure = [](pcl_container_t* container,
                                        void* user_data) -> pcl_status_t {
    auto** port = static_cast<pcl_port_t**>(user_data);
    *port = pcl_container_add_publisher(
        container, svc::kTopicEntityMatches, "application/octet-stream");
    if (*port == nullptr) {
      return PCL_ERR_NOMEM;
    }
    const char* peers[] = {"receiver"};
    return pcl_port_set_route(*port, PCL_ROUTE_REMOTE, peers, 1u);
  };
  auto* publisher = pcl_container_create(
      "udp_perf_publisher", &publisher_callbacks, &publisher_port);
  pcl_container_configure(publisher);
  pcl_container_activate(publisher);
  pcl_executor_add(send_executor, publisher);

  std::vector<svc::ObjectMatch> typed_payload(1);
  typed_payload[0].id = "binding-perf-object-1";
  typed_payload[0].source = "perf-client";
  typed_payload[0].matching_object_id = "binding-perf-match-1";
  std::string encoded_payload;
  if (!svc::encodeEntityMatches(typed_payload, content_type, &encoded_payload)) {
    encoded_payload.clear();
  }

  ExecutorRunner receive_runner(receive_executor);
  std::vector<double> latencies;
  latencies.reserve(kNMsgs);
  int timed_out = 0;
  for (int i = -kNWarmupMsgs; i < kNMsgs; ++i) {
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      state.complete = false;
      state.decoded = false;
      state.started_at = Clock::now();
    }
    pcl_status_t rc = PCL_ERR_INVALID;
    if (use_typed_binding) {
      rc = svc::publishEntityMatches(
          publisher_port, typed_payload, content_type);
    } else {
      pcl_msg_t msg{};
      msg.data = encoded_payload.data();
      msg.size = static_cast<uint32_t>(encoded_payload.size());
      msg.type_name = content_type;
      rc = pcl_port_publish(publisher_port, &msg);
    }

    Clock::time_point started_at;
    Clock::time_point completed_at;
    bool delivered = false;
    {
      std::unique_lock<std::mutex> lock(state.mutex);
      delivered = rc == PCL_OK && state.cv.wait_until(
          lock, Clock::now() + std::chrono::milliseconds(kMsgTimeoutMs),
          [&] { return state.complete; });
      delivered = delivered && state.decoded;
      started_at = state.started_at;
      completed_at = state.completed_at;
    }
    if (i < 0) {
      continue;
    }
    if (delivered) {
      latencies.push_back(toUs(started_at, completed_at));
    } else {
      ++timed_out;
    }
  }

  receive_runner.stop();
  pcl_executor_remove(send_executor, publisher);
  pcl_executor_remove(receive_executor, subscriber);
  pcl_container_destroy(publisher);
  pcl_container_destroy(subscriber);
  pcl_udp_transport_destroy(sender);
  pcl_udp_transport_destroy(receiver);
  pcl_executor_destroy(send_executor);
  pcl_executor_destroy(receive_executor);
  return computeResult(label, kNMsgs, static_cast<int>(latencies.size()),
                       timed_out, encoded_payload.size(), latencies);
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

static PerfResult runGrpcCalls(const char* label,
                               const std::string& address) {
  auto channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
  auto stub = proto_services::Object_Of_Interest_Service::NewStub(channel);
  const auto request = makeGrpcRequest();
  const size_t payload_bytes = static_cast<size_t>(request.ByteSizeLong());

  std::vector<double> wall_latencies;
  wall_latencies.reserve(kNMsgsGrpc);
  int timed_out = 0;
  for (int i = -kNWarmupMsgs; i < kNMsgsGrpc; ++i) {
    proto_base::Identifier response;
    grpc::ClientContext context;
    const auto started_at = Clock::now();
    const auto status = stub->CreateRequirement(&context, request, &response);
    const auto completed_at = Clock::now();
    if (i < 0) {
      continue;
    }
    if (status.ok()) {
      wall_latencies.push_back(toUs(started_at, completed_at));
    } else {
      ++timed_out;
    }
  }
  return computeResult(label, kNMsgsGrpc,
                       static_cast<int>(wall_latencies.size()), timed_out,
                       payload_bytes, wall_latencies);
}

class RawGrpcService final
    : public proto_services::Object_Of_Interest_Service::Service {
public:
  grpc::Status CreateRequirement(
      grpc::ServerContext*,
      const proto_tactical::ObjectInterestRequirement*,
      proto_base::Identifier* response) override {
    response->set_value("grpc-interest-42");
    return grpc::Status::OK;
  }
};

static PerfResult runRawGrpc(const char* label, const std::string& address) {
  RawGrpcService service;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  auto server = builder.BuildAndStart();
  if (!server) {
    PerfResult result;
    result.label = label;
    result.skipped = true;
    return result;
  }
  auto result = runGrpcCalls(label, address);
  server->Shutdown();
  server->Wait();
  return result;
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

  auto result = runGrpcCalls(label, address);

  host.shutdown();
  spin_stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);

  return result;
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
  static std::vector<PerfResult> pubsub_results_;
  static std::vector<CodecPerfResult> codec_results_;

  static void TearDownTestSuite() {
    printReport(results_, "Unary request/response performance",
                "ObjectInterestRequirement -> Identifier", "AverageRTT(us)");
    printReport(pubsub_results_, "One-way pub/sub performance",
                "ObjectMatch[1]", "Average(us)");
    printCodecReport(codec_results_);
  }

  static const svc::ObjectInterestRequirement& payload() {
    static const auto request = makePayload();
    return request;
  }
};

std::vector<PerfResult> BindingPerformanceTest::results_;
std::vector<PerfResult> BindingPerformanceTest::pubsub_results_;
std::vector<CodecPerfResult> BindingPerformanceTest::codec_results_;

static void printResult(const PerfResult& result) {
  std::printf(
      "[PERF] %-32s done=%3d/%d avg=%7.1f us p50=%7.1f us "
      "p99=%7.1f us payload=%zu B\n",
      result.label.c_str(), result.completed_msgs, result.attempted_msgs,
      result.wall.avg_us, result.wall.p50_us, result.wall.p99_us,
      result.payload_bytes);
}

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

#if defined(PYRAMID_HAS_OMS_JSON)
static void fillOmsPositionReport(pyramid_uci_position_report_c* report) {
  std::memset(report, 0, sizeof(*report));
  std::snprintf(report->header.classification,
                sizeof(report->header.classification), "U");
  std::snprintf(report->header.owner_producer,
                sizeof(report->header.owner_producer), "USA");
  std::snprintf(report->header.system_uuid,
                sizeof(report->header.system_uuid),
                "550e8400-e29b-41d4-a716-446655440000");
  std::snprintf(report->header.timestamp,
                sizeof(report->header.timestamp),
                "2026-07-15T12:00:00Z");
  std::snprintf(report->header.schema_version,
                sizeof(report->header.schema_version), "002.5.0");
  std::snprintf(report->header.mode, sizeof(report->header.mode), "LIVE");
  std::snprintf(report->report_system_uuid,
                sizeof(report->report_system_uuid),
                "550e8400-e29b-41d4-a716-446655440000");
  std::snprintf(report->source, sizeof(report->source), "ACTUAL");
  std::snprintf(report->current_operating_domain,
                sizeof(report->current_operating_domain), "AIR");
  report->latitude_rad = 0.898973719;
  report->longitude_rad = -0.000025744;
  report->altitude_m = 1200.0;
  std::snprintf(report->position_timestamp,
                sizeof(report->position_timestamp),
                "2026-07-15T12:00:00Z");
  report->has_altitude_reference = true;
  std::snprintf(report->altitude_reference,
                sizeof(report->altitude_reference), "WGS84");
}

TEST_F(BindingPerformanceTest, Codec_OmsJson_Plugin) {
  auto* handle = pcl_plugin_open(PYRAMID_OMS_JSON_CODEC_PATH);
  ASSERT_NE(handle, nullptr);
  using CodecEntry = const pcl_codec_t* (*)(const char*);
  auto entry = reinterpret_cast<CodecEntry>(
      pcl_plugin_symbol(handle, PCL_CODEC_PLUGIN_ENTRY_SYMBOL));
  ASSERT_NE(entry, nullptr);
  const auto* codec = entry(nullptr);
  ASSERT_NE(codec, nullptr);

  pyramid_uci_position_report_c request;
  fillOmsPositionReport(&request);
  pcl_msg_t sample{};
  ASSERT_EQ(codec->encode(codec->codec_ctx, "PositionReport", &request,
                          &sample), PCL_OK);
  const size_t payload_bytes = sample.size;
  std::string sample_bytes(static_cast<const char*>(sample.data), sample.size);
  codec->free_msg(codec->codec_ctx, &sample);

  auto encode = runCodecLoop(
      "codec / oms-json plugin enc", payload_bytes, kNCodecIters, [&] {
        pcl_msg_t encoded{};
        if (codec->encode(codec->codec_ctx, "PositionReport", &request,
                          &encoded) == PCL_OK) {
          s_codec_sink.fetch_add(encoded.size, std::memory_order_relaxed);
          codec->free_msg(codec->codec_ctx, &encoded);
        }
      });
  auto decode = runCodecLoop(
      "codec / oms-json plugin dec", payload_bytes, kNCodecIters, [&] {
        pcl_msg_t encoded{sample_bytes.data(),
                          static_cast<uint32_t>(sample_bytes.size()),
                          "application/oms-json"};
        pyramid_uci_position_report_c decoded{};
        if (codec->decode(codec->codec_ctx, "PositionReport", &encoded,
                          &decoded) == PCL_OK) {
          s_codec_sink.fetch_add(
              static_cast<uint64_t>(decoded.altitude_m),
              std::memory_order_relaxed);
        }
      });
  codec_results_.push_back(encode);
  codec_results_.push_back(decode);
  EXPECT_GT(encode.wall_ns_per_op, 0.0);
  EXPECT_GT(decode.wall_ns_per_op, 0.0);
  pcl_plugin_unload(handle);
}
#endif

// ---------------------------------------------------------------------------
// Local (in-process) benchmarks
// ---------------------------------------------------------------------------

TEST_F(BindingPerformanceTest, Local_Json) {
  auto r = runLocal("pyramid / local / json", "application/json", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Local_Json) {
  auto r = runLocal("raw-pcl / local / json", "application/json", payload(),
                    false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, Local_FlatBuffers) {
  auto r = runLocal("pyramid / local / flatbuffers",
                    "application/flatbuffers", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Local_FlatBuffers) {
  auto r = runLocal("raw-pcl / local / flatbuffers",
                    "application/flatbuffers", payload(), false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

#if defined(PYRAMID_HAS_PROTOBUF)
// The generated facade resolves codecs through the codec-plugin registry, so the
// generic transport rows (local/shmem/socket) require a registered codec for the
// content type. The application/protobuf codec plugin
// (pyramid_codec_protobuf_<component>) now registers for components that have a
// protobuf service codec (tactical_objects), so these rows run there. Where no
// protobuf codec is registered, skip honestly (recording a skipped summary entry)
// rather than report a misleading 0/N transport failure.
static bool skipIfNoProtobufCodec(
    std::vector<PerfResult>& results, const char* label) {
  if (svc::supportsContentType("application/protobuf")) {
    return false;
  }
  PerfResult skipped;
  skipped.label = label;
  skipped.skipped = true;
  results.push_back(skipped);
  return true;
}

TEST_F(BindingPerformanceTest, Local_Protobuf) {
  if (skipIfNoProtobufCodec(results_, "pyramid / local / protobuf")) {
    GTEST_SKIP() << "no application/protobuf codec plugin registered";
  }
  auto r = runLocal("pyramid / local / protobuf", "application/protobuf",
                    payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Local_Protobuf) {
  auto r = runLocal("raw-pcl / local / protobuf", "application/protobuf",
                    payload(), false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}
#endif

// ---------------------------------------------------------------------------
// UDP one-way pub/sub benchmarks
// ---------------------------------------------------------------------------

TEST_F(BindingPerformanceTest, Udp_Json) {
  auto r = runUdp("pyramid / udp / json", "application/json", true);
  pubsub_results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Udp_Json) {
  auto r = runUdp("raw-pcl / udp / json", "application/json", false);
  pubsub_results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, Udp_FlatBuffers) {
  auto r = runUdp("pyramid / udp / flatbuffers", "application/flatbuffers",
                  true);
  pubsub_results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Udp_FlatBuffers) {
  auto r = runUdp("raw-pcl / udp / flatbuffers",
                  "application/flatbuffers", false);
  pubsub_results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Udp_Protobuf) {
  if (!svc::supportsContentType("application/protobuf")) {
    GTEST_SKIP() << "no application/protobuf codec plugin registered";
  }
  auto r = runUdp("pyramid / udp / protobuf", "application/protobuf", true);
  pubsub_results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Udp_Protobuf) {
  auto r = runUdp("raw-pcl / udp / protobuf", "application/protobuf", false);
  pubsub_results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}
#endif

TEST_F(BindingPerformanceTest, Shmem_Json) {
  auto r = runShmem("pyramid / shmem / json", "application/json", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Shmem_Json) {
  auto r = runShmem("raw-pcl / shmem / json", "application/json", payload(),
                    false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, Shmem_FlatBuffers) {
  auto r = runShmem("pyramid / shmem / flatbuffers",
                    "application/flatbuffers", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Shmem_FlatBuffers) {
  auto r = runShmem("raw-pcl / shmem / flatbuffers",
                    "application/flatbuffers", payload(), false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
#if !defined(_WIN32)
  EXPECT_LT(r.wall.p50_us, 750.0)
      << "shared-memory receive regressed to the old 1 ms polling cadence";
#endif
  printResult(r);
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Shmem_Protobuf) {
  if (skipIfNoProtobufCodec(results_, "pyramid / shmem / protobuf")) {
    GTEST_SKIP() << "no application/protobuf codec plugin registered";
  }
  auto r = runShmem("pyramid / shmem / protobuf", "application/protobuf",
                    payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Shmem_Protobuf) {
  auto r = runShmem("raw-pcl / shmem / protobuf", "application/protobuf",
                    payload(), false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}
#endif

TEST_F(BindingPerformanceTest, Socket_Json) {
  auto r = runSocket("pyramid / socket / json", "application/json", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Socket_Json) {
  auto r = runSocket("raw-pcl / socket / json", "application/json", payload(),
                     false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

#if defined(PYRAMID_HAS_PROTOBUF)
TEST_F(BindingPerformanceTest, Socket_Protobuf) {
  if (skipIfNoProtobufCodec(results_, "pyramid / socket / protobuf")) {
    GTEST_SKIP() << "no application/protobuf codec plugin registered";
  }
  auto r = runSocket("pyramid / socket / protobuf", "application/protobuf",
                     payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Socket_Protobuf) {
  auto r = runSocket("raw-pcl / socket / protobuf", "application/protobuf",
                     payload(), false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}
#endif

#if defined(PYRAMID_HAS_GRPC)
TEST_F(BindingPerformanceTest, Grpc_Tcp) {
  const auto port = pickSocketPort();
  ASSERT_NE(port, 0);
  auto r = runGrpc("pyramid-port / grpc / protobuf",
                   "127.0.0.1:" + std::to_string(port));
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgsGrpc);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawGrpc_Tcp) {
  const auto port = pickSocketPort();
  ASSERT_NE(port, 0);
  auto r = runRawGrpc("raw-grpc / grpc / protobuf",
                      "127.0.0.1:" + std::to_string(port));
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgsGrpc);
  printResult(r);
}
#endif

TEST_F(BindingPerformanceTest, Socket_FlatBuffers) {
  auto r = runSocket("pyramid / socket / flatbuffers",
                     "application/flatbuffers", payload());
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}

TEST_F(BindingPerformanceTest, RawPcl_Socket_FlatBuffers) {
  auto r = runSocket("raw-pcl / socket / flatbuffers",
                     "application/flatbuffers", payload(), false);
  results_.push_back(r);
  EXPECT_EQ(r.completed_msgs, kNMsgs);
  printResult(r);
}
