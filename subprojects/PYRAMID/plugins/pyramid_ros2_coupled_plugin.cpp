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
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <thread>

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

pcl_executor_t* readExecutor(const char* config_json) {
  uint64_t raw = 0U;
  if (!readJsonU64(config_json, "executor", &raw) || raw == 0U) {
    return nullptr;
  }
  return reinterpret_cast<pcl_executor_t*>(static_cast<uintptr_t>(raw));
}

// -- Transport adapter context -------------------------------------------

struct Ros2TransportContext {
  pcl_executor_t* executor = nullptr;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<pyramid::transport::ros2::RclcppRuntimeAdapter> adapter;
  rclcpp::executors::SingleThreadedExecutor ros_executor;
  std::thread spin_thread;
  std::atomic<bool> spinning{false};
  bool owns_rclcpp = false;
  pcl_transport_t transport{};
};

Ros2TransportContext* contextOf(const pcl_transport_t* transport) {
  if (!transport) return nullptr;
  return static_cast<Ros2TransportContext*>(transport->adapter_ctx);
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
  stopSpin(ctx);
  if (ctx->node) {
    ctx->ros_executor.remove_node(ctx->node);
  }
  ctx->adapter.reset();
  ctx->node.reset();
  if (ctx->owns_rclcpp && rclcpp::ok()) {
    rclcpp::shutdown();
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
  // Consumed (client) side: PCL invokes a remote ROS2 unary service. The adapter
  // blocks for the response (serviced by this context's spin thread), then the
  // support helper delivers it through the PCL callback.
  return pyramid::transport::ros2::invokeRemoteUnary(
      *ctx->adapter, service_name, request, callback, user_data);
}

void ros2Shutdown(void* ctx_v) {
  stopSpin(static_cast<Ros2TransportContext*>(ctx_v));
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

/// Server-ingress ROS2 transport: pub/sub via the vtable, plus unary and
/// server-streaming service ingress via the advertise_unary/advertise_stream
/// symbols (RPC provided side, not exposed as vtable slots, so derivation cannot
/// see them). The consumed side (invoke_async/invoke_stream) is not yet
/// implemented -- see WIP D2 -- but the transport can carry those patterns on
/// the provided side.
PYRAMID_ROS2_PLUGIN_EXPORT pcl_transport_caps_t pcl_transport_plugin_caps(
    const char* config_json) {
  (void)config_json;
  return PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM;
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
  // (its guard condition binds the default context), so init here -- ahead of
  // allocating the context struct that owns the executor by value.
  bool owns_rclcpp = false;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    owns_rclcpp = true;
  }

  auto* ctx = new (std::nothrow) Ros2TransportContext();
  if (!ctx) {
    if (owns_rclcpp && rclcpp::ok()) rclcpp::shutdown();
    return nullptr;
  }
  ctx->executor = executor;
  ctx->owns_rclcpp = owns_rclcpp;

  try {
    ctx->node = std::make_shared<rclcpp::Node>(node_name);
    ctx->adapter =
        std::make_unique<pyramid::transport::ros2::RclcppRuntimeAdapter>(ctx->node);
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

  ctx->transport.publish = ros2Publish;
  ctx->transport.subscribe = ros2Subscribe;
  ctx->transport.invoke_async = ros2InvokeAsync;
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
