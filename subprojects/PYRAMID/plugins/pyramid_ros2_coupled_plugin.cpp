/// \file pyramid_ros2_coupled_plugin.cpp
/// \brief Coupled ROS2 target plugin for application/ros2.
///
/// Mirrors the coupled gRPC target (pyramid_grpc_coupled_plugin.cpp): a single
/// MODULE .so exposes BOTH a PCL transport vtable and a PCL codec vtable under
/// one content type (application/ros2). Unlike the structural gRPC stub, this
/// plugin is wired to the real, tested ROS2 runtime -- it owns an rclcpp node,
/// a RclcppRuntimeAdapter, and a background spin thread -- so a client that only
/// links pcl_core and dlopen()s this plugin gets working ROS2 pub/sub plus
/// service/stream advertising. All ROS2/rclcpp/transport/codec linkage is bundled
/// privately into the .so; consumers add no extra static linkage.
///
/// Transport vtable:
///   - publish(topic, msg)        -> publishOutboundEnvelope (executor -> ROS2)
///   - subscribe(topic, type)     -> bindTopicIngress        (ROS2 -> executor)
///   - invoke_async(service, ...) -> call remote ROS2 unary service
///   - invoke_stream(service, ...) -> call remote ROS2 server-stream service
///   - shutdown()                 -> stop spinning, drop subscriptions
///
/// Plugin-private entry symbols (resolved via pcl_plugin_symbol), covering the
/// server-side service/stream paths the standard transport vtable has no setup
/// hook for:
///   - pcl_ros2_transport_plugin_bind_topic(transport, pcl_topic)
///   - pcl_ros2_transport_plugin_advertise_unary(transport, pcl_service)
///   - pcl_ros2_transport_plugin_advertise_stream(transport, pcl_service)
///   - pcl_ros2_transport_plugin_destroy(transport)
///
/// Codec vtable (application/ros2): a passthrough envelope codec. ROS2 carries
/// already-encoded bytes in the envelope payload tagged with the payload's own
/// content_type, so the typed value crossing the codec ABI is itself a pcl_msg_t
/// of pre-encoded bytes. The real payload codec (JSON/protobuf/flatbuffers) is a
/// separately selected plugin.

#include <pcl/pcl_codec.h>
#include <pcl/pcl_plugin.h>

extern "C" {
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
}

#include "pyramid_ros2_transport/rclcpp_runtime_adapter.hpp"
#include "pyramid_ros2_transport_support.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(_WIN32)
#  define PYRAMID_ROS2_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PYRAMID_ROS2_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PYRAMID_ROS2_PLUGIN_EXPORT
#endif

namespace {

constexpr const char* kRos2ContentType = "application/ros2";

// -- Minimal JSON readers (mirror the socket/shm transport plugins) -------

const char* findJsonValue(const char* json, const char* key) {
  if (!json || !key) return nullptr;
  char needle[64];
  if (std::strlen(key) + 3U >= sizeof(needle)) return nullptr;
  needle[0] = '"';
  std::strcpy(needle + 1U, key);
  std::strcat(needle, "\"");

  const char* p = std::strstr(json, needle);
  if (!p) return nullptr;
  p += std::strlen(needle);
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
  if (*p != ':') return nullptr;
  ++p;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
  return p;
}

bool readJsonString(const char* json, const char* key, char* out, size_t out_size) {
  if (!out || out_size == 0U) return false;
  out[0] = '\0';
  const char* p = findJsonValue(json, key);
  if (!p || *p != '"') return false;
  ++p;
  size_t n = 0U;
  while (p[n] && p[n] != '"') {
    if (n + 1U >= out_size) return false;
    out[n] = p[n];
    ++n;
  }
  if (p[n] != '"') return false;
  out[n] = '\0';
  return true;
}

bool readJsonU64(const char* json, const char* key, uint64_t* out) {
  if (!out) return false;
  const char* p = findJsonValue(json, key);
  if (!p) return false;
  if (*p == '"') ++p;
  char* end = nullptr;
  const uint64_t value = static_cast<uint64_t>(std::strtoull(p, &end, 0));
  if (end == p) return false;
  *out = value;
  return true;
}

bool readJsonBool(const char* json, const char* key, bool default_value) {
  const char* p = findJsonValue(json, key);
  if (!p) return default_value;
  if (std::strncmp(p, "true", 4U) == 0) return true;
  if (std::strncmp(p, "false", 5U) == 0) return false;
  if (*p == '"') {
    ++p;
    if (std::strncmp(p, "true", 4U) == 0 ||
        std::strncmp(p, "envelope", 8U) == 0) {
      return true;
    }
    if (std::strncmp(p, "false", 5U) == 0 ||
        std::strncmp(p, "typed", 5U) == 0) {
      return false;
    }
  }
  return default_value;
}

pcl_executor_t* readExecutor(const char* config_json) {
  uint64_t raw = 0U;
  if (!readJsonU64(config_json, "executor", &raw) || raw == 0U) {
    return nullptr;
  }
  return reinterpret_cast<pcl_executor_t*>(static_cast<uintptr_t>(raw));
}

// -- Process-wide rclcpp ownership ----------------------------------------
//
// rclcpp::init()/shutdown() act on a process-global default context, so this
// .so must NOT shut rclcpp down while sibling plugin instances are still
// spinning (a manifest with more than one ROS2 peer loads several contexts in
// one process). We refcount every context that needs rclcpp running and only
// shut it down once the last one is released -- and only if we were the ones
// who initialised it, so a context the host application brought up is left
// untouched.
std::mutex g_rclcpp_mutex;
int g_rclcpp_refcount = 0;
bool g_rclcpp_initialised_by_plugin = false;

// Ensure rclcpp is running and register one reference. Returns true when rclcpp
// is available afterwards. Must be paired with releaseRclcpp().
bool acquireRclcpp() {
  std::lock_guard<std::mutex> lock(g_rclcpp_mutex);
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    g_rclcpp_initialised_by_plugin = true;
  }
  if (!rclcpp::ok()) return false;
  ++g_rclcpp_refcount;
  return true;
}

// Drop one reference; shut rclcpp down once the last plugin-owned reference is
// gone (and only if this plugin initialised it).
void releaseRclcpp() {
  std::lock_guard<std::mutex> lock(g_rclcpp_mutex);
  if (g_rclcpp_refcount > 0) {
    --g_rclcpp_refcount;
  }
  if (g_rclcpp_refcount == 0 && g_rclcpp_initialised_by_plugin && rclcpp::ok()) {
    rclcpp::shutdown();
    g_rclcpp_initialised_by_plugin = false;
  }
}

// -- Transport adapter context -------------------------------------------

// One queued consumed-side RPC. The executor-facing invoke_async/invoke_stream
// vtable entries only copy the request into one of these and enqueue it; the
// plugin-owned worker thread runs the (bounded, blocking) ROS2 call so the
// executor thread is never blocked on a remote service or stream completion.
enum class Ros2WorkKind {
  Unary,
  Stream,
};

struct Ros2WorkItem {
  Ros2WorkKind kind = Ros2WorkKind::Unary;
  std::string service_name;
  std::vector<unsigned char> request;
  std::string request_type;  // preserved so the worker rebuilds the envelope
  pcl_resp_cb_fn_t response_callback = nullptr;
  pcl_stream_msg_fn_t stream_callback = nullptr;
  void* user_data = nullptr;
};

struct Ros2TransportContext {
  pcl_executor_t* executor = nullptr;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<pyramid::transport::ros2::RclcppRuntimeAdapter> adapter;
  rclcpp::executors::SingleThreadedExecutor ros_executor;
  std::thread spin_thread;
  std::atomic<bool> spinning{false};
  bool holds_rclcpp_ref = false;  // released via releaseRclcpp() on destroy
  // Consumed-side egress worker: owns blocking invokeRemoteUnary/Stream calls.
  std::mutex work_mutex;
  std::condition_variable work_cv;
  std::deque<Ros2WorkItem> work_queue;
  std::thread work_thread;
  bool work_stop = false;
  pcl_transport_t transport{};
};

Ros2TransportContext* contextOf(const pcl_transport_t* transport) {
  if (!transport) return nullptr;
  return static_cast<Ros2TransportContext*>(transport->adapter_ctx);
}

// -- Executor-thread postback plumbing (mirrors the gRPC coupled plugin) ---
//
// The worker delivers unary responses via pcl_executor_post_response_msg (which
// deep-copies the payload) and stream frames via a small event trampoline so
// user callbacks always run on the executor thread, never on the worker.

struct StreamEvent {
  pcl_stream_msg_fn_t callback;
  void* user_data;
  bool end;
  pcl_status_t status;
};

void streamEventCallback(const pcl_msg_t* msg, void* user_data) {
  std::unique_ptr<StreamEvent> event(static_cast<StreamEvent*>(user_data));
  if (!event || !event->callback) return;
  event->callback(msg, event->end, event->status, event->user_data);
}

pcl_status_t postStreamEvent(Ros2TransportContext* ctx,
                             pcl_stream_msg_fn_t callback,
                             void* user_data,
                             const pcl_msg_t* msg,
                             bool end,
                             pcl_status_t status) {
  if (!ctx || !ctx->executor || !callback) return PCL_ERR_INVALID;
  auto event = std::unique_ptr<StreamEvent>(new (std::nothrow) StreamEvent());
  if (!event) return PCL_ERR_NOMEM;
  event->callback = callback;
  event->user_data = user_data;
  event->end = end;
  event->status = status;
  pcl_msg_t empty{};
  const pcl_msg_t* queued_msg = msg ? msg : &empty;
  const pcl_status_t rc = pcl_executor_post_response_msg(
      ctx->executor, streamEventCallback, event.get(), queued_msg);
  if (rc == PCL_OK) {
    event.release();
  }
  return rc;
}

// Thunks handed to the generated invokeRemote* helpers. The helpers call these
// inline on the worker thread; the thunks re-post to the executor so the real
// user callback runs on the executor thread.
struct UnaryThunk {
  pcl_executor_t* executor;
  pcl_resp_cb_fn_t callback;
  void* user_data;
  bool posted;
};

void unaryThunkCallback(const pcl_msg_t* response, void* user) {
  auto* t = static_cast<UnaryThunk*>(user);
  if (!t) return;
  t->posted = true;
  (void)pcl_executor_post_response_msg(t->executor, t->callback, t->user_data,
                                       response);
}

struct StreamThunk {
  Ros2TransportContext* ctx;
  pcl_stream_msg_fn_t callback;
  void* user_data;
  bool terminated;
};

void streamThunkCallback(const pcl_msg_t* msg, bool end, pcl_status_t status,
                         void* user) {
  auto* t = static_cast<StreamThunk*>(user);
  if (!t) return;
  if (end) t->terminated = true;
  (void)postStreamEvent(t->ctx, t->callback, t->user_data, msg, end, status);
}

void ros2WorkerMain(Ros2TransportContext* ctx) {
  for (;;) {
    Ros2WorkItem item;
    {
      std::unique_lock<std::mutex> lock(ctx->work_mutex);
      ctx->work_cv.wait(lock, [ctx] {
        return ctx->work_stop || !ctx->work_queue.empty();
      });
      if (ctx->work_queue.empty()) {
        if (ctx->work_stop) break;
        continue;
      }
      item = std::move(ctx->work_queue.front());
      ctx->work_queue.pop_front();
    }

    pcl_msg_t req{};
    req.data = item.request.empty() ? nullptr : item.request.data();
    req.size = static_cast<uint32_t>(item.request.size());
    req.type_name = item.request_type.empty() ? nullptr : item.request_type.c_str();

    if (item.kind == Ros2WorkKind::Unary) {
      UnaryThunk thunk{ctx->executor, item.response_callback, item.user_data,
                       false};
      const pcl_status_t rc = pyramid::transport::ros2::invokeRemoteUnary(
          *ctx->adapter, item.service_name.c_str(), &req, unaryThunkCallback,
          &thunk);
      if (!thunk.posted) {
        // Bounded ROS2 failure (timeout / no service): still fire the client
        // callback on the executor thread with an empty response.
        pcl_msg_t empty{};
        empty.type_name = "application/ros2";
        (void)rc;
        (void)pcl_executor_post_response_msg(ctx->executor,
                                             item.response_callback,
                                             item.user_data, &empty);
      }
      continue;
    }

    StreamThunk thunk{ctx, item.stream_callback, item.user_data, false};
    const pcl_status_t rc = pyramid::transport::ros2::invokeRemoteStream(
        *ctx->adapter, item.service_name.c_str(), &req, streamThunkCallback,
        &thunk);
    if (!thunk.terminated) {
      pcl_msg_t terminal{};
      terminal.type_name = "application/ros2";
      (void)postStreamEvent(ctx, item.stream_callback, item.user_data,
                            &terminal, true, rc == PCL_OK ? PCL_OK : rc);
    }
  }
}

bool startWorker(Ros2TransportContext* ctx) {
  if (!ctx) return false;
  try {
    ctx->work_thread = std::thread(ros2WorkerMain, ctx);
  } catch (...) {
    return false;
  }
  return true;
}

// Stop the egress worker. Must run while the spin thread is still alive so an
// in-flight (bounded) invokeRemote* call can complete before the join.
void stopWorker(Ros2TransportContext* ctx) {
  if (!ctx) return;
  {
    std::lock_guard<std::mutex> lock(ctx->work_mutex);
    ctx->work_stop = true;
  }
  ctx->work_cv.notify_all();
  if (ctx->work_thread.joinable()) {
    ctx->work_thread.join();
  }
}

pcl_status_t enqueueWork(Ros2TransportContext* ctx, Ros2WorkItem item) {
  if (!ctx) return PCL_ERR_INVALID;
  {
    std::lock_guard<std::mutex> lock(ctx->work_mutex);
    if (ctx->work_stop || !ctx->work_thread.joinable()) {
      return PCL_ERR_STATE;
    }
    try {
      ctx->work_queue.push_back(std::move(item));
    } catch (...) {
      return PCL_ERR_NOMEM;
    }
  }
  ctx->work_cv.notify_one();
  return PCL_OK;
}

void stopSpin(Ros2TransportContext* ctx) {
  if (!ctx) return;
  // Signal the spin loop to exit, then nudge the executor. We do NOT rely on
  // cancel() alone: SingleThreadedExecutor::cancel() is racy when it runs before
  // spin() has entered its wait (the cancel is lost and spin() blocks forever).
  // The spin loop below uses spin_once() with a timeout and re-checks this flag,
  // so stopping is deterministic regardless of timing.
  ctx->spinning.store(false, std::memory_order_release);
  ctx->ros_executor.cancel();
  if (ctx->spin_thread.joinable()) {
    ctx->spin_thread.join();
  }
}

void destroyContext(Ros2TransportContext* ctx) {
  if (!ctx) return;
  // Stop the egress worker first, while the spin thread is still alive: an
  // in-flight invokeRemote* call is serviced by the spin thread and is bounded
  // (~5s), so joining the worker here cannot hang.
  stopWorker(ctx);
  stopSpin(ctx);
  if (ctx->node) {
    ctx->ros_executor.remove_node(ctx->node);
  }
  ctx->adapter.reset();
  ctx->node.reset();
  if (ctx->holds_rclcpp_ref) {
    releaseRclcpp();
  }
  delete ctx;
}

// -- Transport vtable ------------------------------------------------------

pcl_status_t ros2Publish(void* ctx_v, const char* topic, const pcl_msg_t* msg) {
  auto* ctx = static_cast<Ros2TransportContext*>(ctx_v);
  if (!ctx || !ctx->adapter || !topic || !msg) return PCL_ERR_INVALID;
  return pyramid::transport::ros2::publishOutboundEnvelope(*ctx->adapter, topic, msg);
}

pcl_status_t ros2Subscribe(void* ctx_v, const char* topic, const char* /*type_name*/) {
  auto* ctx = static_cast<Ros2TransportContext*>(ctx_v);
  if (!ctx || !ctx->adapter || !ctx->executor || !topic) return PCL_ERR_INVALID;
  pyramid::transport::ros2::bindTopicIngress(*ctx->adapter, ctx->executor, topic);
  return PCL_OK;
}

pcl_status_t ros2InvokeAsync(void* ctx_v, const char* service_name,
                             const pcl_msg_t* request, pcl_resp_cb_fn_t callback,
                             void* user_data) {
  auto* ctx = static_cast<Ros2TransportContext*>(ctx_v);
  if (!ctx || !ctx->adapter || !service_name || !request || !callback) {
    return PCL_ERR_INVALID;
  }
  if (request->size > 0U && !request->data) return PCL_ERR_INVALID;
  // Consumed (client) side: copy the request and queue it. The worker owns the
  // blocking ROS2 unary call and posts the response back on the executor thread
  // -- the executor-facing entry never blocks on the remote service.
  Ros2WorkItem item;
  item.kind = Ros2WorkKind::Unary;
  item.service_name = service_name;
  item.response_callback = callback;
  item.user_data = user_data;
  if (request->type_name) item.request_type = request->type_name;
  try {
    if (request->size > 0U) {
      const auto* begin = static_cast<const unsigned char*>(request->data);
      item.request.assign(begin, begin + request->size);
    }
  } catch (...) {
    return PCL_ERR_NOMEM;
  }
  return enqueueWork(ctx, std::move(item));
}

pcl_status_t ros2InvokeStream(void* ctx_v, const char* service_name,
                              const pcl_msg_t* request,
                              pcl_stream_msg_fn_t callback, void* user_data,
                              void** stream_handle) {
  if (stream_handle) *stream_handle = nullptr;
  auto* ctx = static_cast<Ros2TransportContext*>(ctx_v);
  if (!ctx || !ctx->adapter || !service_name || !request || !callback) {
    return PCL_ERR_INVALID;
  }
  if (request->size > 0U && !request->data) return PCL_ERR_INVALID;
  // Consumed (client) side: copy the request and queue it. The worker owns the
  // blocking ROS2 server-stream call and posts each frame (and the terminal
  // status) back on the executor thread.
  Ros2WorkItem item;
  item.kind = Ros2WorkKind::Stream;
  item.service_name = service_name;
  item.stream_callback = callback;
  item.user_data = user_data;
  if (request->type_name) item.request_type = request->type_name;
  try {
    if (request->size > 0U) {
      const auto* begin = static_cast<const unsigned char*>(request->data);
      item.request.assign(begin, begin + request->size);
    }
  } catch (...) {
    return PCL_ERR_NOMEM;
  }
  return enqueueWork(ctx, std::move(item));
}

void ros2Shutdown(void* ctx_v) {
  auto* ctx = static_cast<Ros2TransportContext*>(ctx_v);
  stopWorker(ctx);
  stopSpin(ctx);
}

// -- Codec vtable: passthrough envelope codec for application/ros2 --------

pcl_status_t ros2CodecEncode(void* /*codec_ctx*/,
                             const char* /*schema_id*/,
                             const void* value,
                             pcl_msg_t*  out_msg) {
  if (!value || !out_msg) return PCL_ERR_INVALID;
  const auto* in = static_cast<const pcl_msg_t*>(value);
  out_msg->data = nullptr;
  out_msg->size = 0U;
  out_msg->type_name = in->type_name;
  if (in->data && in->size > 0U) {
    void* buffer = std::malloc(in->size);
    if (!buffer) return PCL_ERR_NOMEM;
    std::memcpy(buffer, in->data, in->size);
    out_msg->data = buffer;
    out_msg->size = in->size;
  }
  return PCL_OK;
}

pcl_status_t ros2CodecDecode(void* /*codec_ctx*/,
                             const char* /*schema_id*/,
                             const pcl_msg_t* msg,
                             void*            out_value) {
  if (!msg || !out_value) return PCL_ERR_INVALID;
  auto* out = static_cast<pcl_msg_t*>(out_value);
  out->data = nullptr;
  out->size = 0U;
  out->type_name = msg->type_name;
  if (msg->data && msg->size > 0U) {
    void* buffer = std::malloc(msg->size);
    if (!buffer) return PCL_ERR_NOMEM;
    std::memcpy(buffer, msg->data, msg->size);
    out->data = buffer;
    out->size = msg->size;
  }
  return PCL_OK;
}

void ros2CodecFreeMsg(void* /*codec_ctx*/, pcl_msg_t* msg) {
  if (!msg) return;
  std::free(const_cast<void*>(msg->data));
  msg->data = nullptr;
  msg->size = 0U;
}

const pcl_codec_t ros2_codec = {
    PCL_CODEC_ABI_VERSION,
    kRos2ContentType,
    ros2CodecEncode,
    ros2CodecDecode,
    ros2CodecFreeMsg,
    nullptr,
};

}  // namespace

extern "C" {

PYRAMID_ROS2_PLUGIN_EXPORT uint32_t pcl_transport_abi_version() {
  return PCL_TRANSPORT_ABI_VERSION;
}

/// ROS2 transport: pub/sub via the vtable, server-ingress service advertising
/// via private symbols, and consumed unary/server-streaming egress via the
/// invoke_async/invoke_stream vtable slots. The private advertise_* symbols are
/// not vtable slots, so explicit capability declaration remains required.
PYRAMID_ROS2_PLUGIN_EXPORT pcl_transport_caps_t pcl_transport_plugin_caps(
    const char* config_json) {
  (void)config_json;
  return PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM;
}

/// The runtime adapter advertises/subscribes its ROS2 topics and service
/// frames with reliable QoS, so declare the reliable floor. Without this export
/// the loader reports UNSPECIFIED and any `route ... reliable` line targeting a
/// ROS2 peer fails compose-time validation even though delivery is reliable.
PYRAMID_ROS2_PLUGIN_EXPORT pcl_qos_t pcl_transport_plugin_qos(
    const char* config_json) {
  (void)config_json;
  pcl_qos_t qos;
  qos.reliability = PCL_QOS_RELIABILITY_RELIABLE;
  return qos;
}

PYRAMID_ROS2_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  if (!config_json) return nullptr;

  pcl_executor_t* executor = readExecutor(config_json);
  if (!executor) return nullptr;

  char node_name[256];
  if (!readJsonString(config_json, "node_name", node_name, sizeof(node_name)) ||
      node_name[0] == '\0') {
    std::strcpy(node_name, "pyramid_ros2_plugin");
  }

  // rclcpp must be initialised before the rclcpp executor member is constructed
  // (its guard condition binds the default context), so acquire here -- ahead of
  // allocating the context struct that owns the executor by value. The reference
  // is process-wide refcounted so a sibling instance's teardown cannot shut
  // rclcpp down underneath this one (see acquireRclcpp/releaseRclcpp).
  if (!acquireRclcpp()) {
    return nullptr;
  }

  auto* ctx = new (std::nothrow) Ros2TransportContext();
  if (!ctx) {
    releaseRclcpp();
    return nullptr;
  }
  ctx->executor = executor;
  ctx->holds_rclcpp_ref = true;

  try {
    ctx->node = std::make_shared<rclcpp::Node>(node_name);
    pyramid::transport::ros2::RclcppRuntimeAdapter::Options adapter_options;
    adapter_options.use_envelope_wire =
        readJsonBool(config_json, "use_envelope_wire", false) ||
        readJsonBool(config_json, "envelope_wire", false) ||
        readJsonBool(config_json, "wire_mode", false);
    ctx->adapter =
        std::make_unique<pyramid::transport::ros2::RclcppRuntimeAdapter>(
            ctx->node, adapter_options);
    ctx->ros_executor.add_node(ctx->node);
    ctx->spinning.store(true, std::memory_order_release);
    ctx->spin_thread = std::thread([ctx]() {
      // Cooperative spin loop: spin_once() returns after the timeout even with no
      // work, so the stop flag is observed promptly and shutdown never deadlocks
      // on a lost cancel() (see stopSpin). Guards on rclcpp::ok() so a global
      // shutdown also breaks the loop.
      while (ctx->spinning.load(std::memory_order_acquire) && rclcpp::ok()) {
        ctx->ros_executor.spin_once(std::chrono::milliseconds(50));
      }
    });
  } catch (...) {
    destroyContext(ctx);
    return nullptr;
  }

  // Consumed-side egress worker: services queued invoke_async/invoke_stream
  // calls off the executor thread.
  if (!startWorker(ctx)) {
    destroyContext(ctx);
    return nullptr;
  }

  ctx->transport.publish = ros2Publish;
  ctx->transport.subscribe = ros2Subscribe;
  ctx->transport.invoke_async = ros2InvokeAsync;
  ctx->transport.invoke_stream = ros2InvokeStream;
  ctx->transport.shutdown = ros2Shutdown;
  ctx->transport.adapter_ctx = ctx;
  return &ctx->transport;
}

PYRAMID_ROS2_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(const char*) {
  return &ros2_codec;
}

PYRAMID_ROS2_PLUGIN_EXPORT pcl_status_t pcl_ros2_transport_plugin_bind_topic(
    const pcl_transport_t* transport,
    const char*            pcl_topic) {
  Ros2TransportContext* ctx = contextOf(transport);
  if (!ctx || !ctx->adapter || !ctx->executor || !pcl_topic) return PCL_ERR_INVALID;
  pyramid::transport::ros2::bindTopicIngress(*ctx->adapter, ctx->executor, pcl_topic);
  return PCL_OK;
}

PYRAMID_ROS2_PLUGIN_EXPORT pcl_status_t pcl_ros2_transport_plugin_advertise_unary(
    const pcl_transport_t* transport,
    const char*            pcl_service) {
  Ros2TransportContext* ctx = contextOf(transport);
  if (!ctx || !ctx->adapter || !ctx->executor || !pcl_service) return PCL_ERR_INVALID;
  pyramid::transport::ros2::bindUnaryServiceIngress(*ctx->adapter, ctx->executor,
                                                    pcl_service);
  return PCL_OK;
}

PYRAMID_ROS2_PLUGIN_EXPORT pcl_status_t pcl_ros2_transport_plugin_advertise_stream(
    const pcl_transport_t* transport,
    const char*            pcl_service) {
  Ros2TransportContext* ctx = contextOf(transport);
  if (!ctx || !ctx->adapter || !ctx->executor || !pcl_service) return PCL_ERR_INVALID;
  pyramid::transport::ros2::bindStreamServiceIngress(*ctx->adapter, ctx->executor,
                                                     pcl_service);
  return PCL_OK;
}

PYRAMID_ROS2_PLUGIN_EXPORT void pcl_ros2_transport_plugin_destroy(
    const pcl_transport_t* transport) {
  destroyContext(contextOf(transport));
}

// Standard teardown symbol so a generic caller can use pcl_plugin_unload_transport
// to release the rclcpp node + spin thread BEFORE the .so is dlclose'd.
PYRAMID_ROS2_PLUGIN_EXPORT void pcl_transport_plugin_teardown(
    const pcl_transport_t* transport) {
  destroyContext(contextOf(transport));
}

}  // extern "C"
