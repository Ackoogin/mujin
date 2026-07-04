/// \file pyramid_grpc_coupled_plugin.cpp
/// \brief Coupled gRPC target plugin for application/grpc.
///
/// Mirrors the coupled ROS2 target (pyramid_ros2_coupled_plugin.cpp): a single
/// MODULE .so exposes BOTH a PCL transport vtable and a PCL codec vtable under
/// one content type (application/grpc). The plugin is wired to the real, tested
/// gRPC runtime -- on load it starts a gRPC server for the configured
/// component/role (via the generated, proto-driven aggregator) that dispatches
/// inbound RPCs to the PCL executor. A client that only links pcl_core and
/// dlopen()s this plugin gets a working gRPC server: Ada or C++ gRPC clients
/// call the configured proto services and the handlers run on the executor
/// thread. All gRPC/protobuf/transport linkage is bundled privately into the
/// .so; consumers add no extra static linkage.
///
/// The plugin completes the full PCL transport contract in BOTH directions, so
/// a component can both provide and consume gRPC services through it:
///   PCL provider  <- plugin (server mode)  <- remote gRPC client
///   PCL consumer  -> plugin (client mode)  -> remote gRPC server
/// (The same .so also supports one-sided interop, e.g. a PCL consumer driving a
/// native, non-PCL gRPC server -- see doc/architecture/transport_codec_plugin_system.md.)
///
/// Config contract (config_json passed to pcl_transport_plugin_entry):
///   {
///     "executor":  <uintptr to pcl_executor_t* as decimal/hex>,  (required)
///     "mode":      "server" | "client",    (optional, default "server")
///     "component": "tactical_objects",     (server mode: proto component name)
///     "role":      "provided",             (server mode, default "provided")
///     "address":   "127.0.0.1:50111"       (server: listen addr; client: dial addr)
///   }
///
/// Server (provided) mode: starts a real gRPC server for the configured
/// component that routes inbound RPCs to the executor. The generic
/// publish/serve/subscribe entries fail closed (PCL_ERR_NOT_FOUND): ingress is
/// served by the gRPC server, not the generic pub/sub vtable.
///
/// Client (consumed) mode: dials *address* and implements invoke_async (unary)
/// and invoke_stream (server-streaming) by routing a component's consumed-service
/// calls through the generated typed gRPC stubs (the aggregator client dispatch).
///
/// Plugin-private entry symbols (resolved via pcl_plugin_symbol):
///   - pcl_grpc_transport_plugin_destroy(transport)
///
/// Codec vtable (application/grpc): a passthrough envelope codec. gRPC carries
/// already-encoded protobuf bytes; the typed value crossing the codec ABI is a
/// pcl_msg_t of pre-encoded bytes, copied through unchanged. Generated gRPC
/// transport owns protobuf serialization, so application/grpc must NOT be
/// expected to behave like the JSON/FlatBuffers schema codecs.

#include <pcl/pcl_codec.h>
#include <pcl/pcl_plugin.h>

extern "C" {
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
}

#include "pyramid_grpc_plugin_aggregator.hpp"

#include <atomic>
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
#  define PYRAMID_GRPC_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PYRAMID_GRPC_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PYRAMID_GRPC_PLUGIN_EXPORT
#endif

namespace {

constexpr const char* kGrpcContentType = "application/grpc";
constexpr const char* kDefaultMode = "server";
constexpr const char* kDefaultRole = "provided";
constexpr const char* kDefaultAddress = "0.0.0.0:50051";

// -- Minimal JSON readers (mirror the socket/shm/ros2 transport plugins) ---

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

enum class GrpcWorkKind {
  Unary,
  Stream,
};

struct GrpcWorkItem {
  GrpcWorkKind kind = GrpcWorkKind::Unary;
  std::string service_name;
  std::vector<unsigned char> request;
  pcl_resp_cb_fn_t response_callback = nullptr;
  pcl_stream_msg_fn_t stream_callback = nullptr;
  void* user_data = nullptr;
};

struct GrpcTransportContext {
  pcl_executor_t* executor = nullptr;
  pyramid_grpc_plugin_server* server = nullptr;  // server (provided) mode only
  std::string channel;                            // client (consumed) mode only
  bool is_client = false;
  std::mutex client_mutex;
  std::condition_variable client_cv;
  std::deque<GrpcWorkItem> client_queue;
  std::thread client_worker;
  bool client_stop = false;
  pcl_transport_t transport{};
};

GrpcTransportContext* contextOf(const pcl_transport_t* transport) {
  if (!transport) return nullptr;
  return static_cast<GrpcTransportContext*>(transport->adapter_ctx);
}

void stopServer(GrpcTransportContext* ctx) {
  if (!ctx || !ctx->server) return;
  pyramid_grpc_plugin_server_stop(ctx->server);
  ctx->server = nullptr;
}

struct StreamTrampoline {
  GrpcTransportContext* ctx;
  pcl_stream_msg_fn_t callback;
  void* user_data;
};

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

pcl_status_t postStreamEvent(GrpcTransportContext* ctx,
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

void grpcStreamOnItem(const void* data, unsigned size, void* user);
void grpcClientWorkerMain(GrpcTransportContext* ctx);

bool startClientWorker(GrpcTransportContext* ctx) {
  if (!ctx) return false;
  try {
    ctx->client_worker = std::thread(grpcClientWorkerMain, ctx);
  } catch (...) {
    return false;
  }
  return true;
}

void stopClientWorker(GrpcTransportContext* ctx) {
  if (!ctx) return;
  {
    std::lock_guard<std::mutex> lock(ctx->client_mutex);
    ctx->client_stop = true;
  }
  ctx->client_cv.notify_all();
  if (ctx->client_worker.joinable()) {
    ctx->client_worker.join();
  }
}

void destroyContext(GrpcTransportContext* ctx) {
  if (!ctx) return;
  stopClientWorker(ctx);
  stopServer(ctx);
  delete ctx;
}

// -- Transport vtable ------------------------------------------------------
//
// gRPC ingress is served by the started server, so the generic pub/sub and
// generic client entries fail closed. They remain present (non-null) because
// the loader and composition checks require callable pointers.

pcl_status_t grpcPublish(void*, const char*, const pcl_msg_t*) {
  return PCL_ERR_NOT_FOUND;
}

pcl_status_t grpcServe(void*, const char*, const pcl_msg_t*, pcl_msg_t*) {
  return PCL_ERR_NOT_FOUND;
}

pcl_status_t grpcSubscribe(void*, const char*, const char*) {
  return PCL_ERR_NOT_FOUND;
}

pcl_status_t grpcInvokeAsync(void*, const char*, const pcl_msg_t*,
                             pcl_resp_cb_fn_t, void*) {
  return PCL_ERR_NOT_FOUND;
}

void grpcShutdown(void* ctx_v) {
  auto* ctx = static_cast<GrpcTransportContext*>(ctx_v);
  stopClientWorker(ctx);
  stopServer(ctx);
}

// -- Client (consumed) vtable: invoke remote gRPC services -----------------
//
// In client mode the plugin dials a remote gRPC endpoint and routes a
// component's consumed-service calls through the generated typed stubs (the
// aggregator's client dispatch). The executor-facing vtable only copies and
// queues outbound work; the worker owns blocking gRPC calls and posts
// completions back to the PCL executor.

pcl_status_t enqueueClientWork(GrpcTransportContext* ctx, GrpcWorkItem item) {
  if (!ctx) return PCL_ERR_INVALID;
  {
    std::lock_guard<std::mutex> lock(ctx->client_mutex);
    if (ctx->client_stop || !ctx->client_worker.joinable()) {
      return PCL_ERR_STATE;
    }
    try {
      ctx->client_queue.push_back(std::move(item));
    } catch (...) {
      return PCL_ERR_NOMEM;
    }
  }
  ctx->client_cv.notify_one();
  return PCL_OK;
}

void grpcClientWorkerMain(GrpcTransportContext* ctx) {
  for (;;) {
    GrpcWorkItem item;
    {
      std::unique_lock<std::mutex> lock(ctx->client_mutex);
      ctx->client_cv.wait(lock, [ctx] {
        return ctx->client_stop || !ctx->client_queue.empty();
      });
      if (ctx->client_queue.empty()) {
        if (ctx->client_stop) break;
        continue;
      }
      item = std::move(ctx->client_queue.front());
      ctx->client_queue.pop_front();
    }

    const void* request_data =
        item.request.empty() ? nullptr : item.request.data();
    const unsigned request_size =
        static_cast<unsigned>(item.request.size());

    if (item.kind == GrpcWorkKind::Unary) {
      void* out = nullptr;
      unsigned out_size = 0U;
      const int rc = pyramid_grpc_plugin_client_invoke_unary(
          ctx->channel.c_str(), item.service_name.c_str(), request_data,
          request_size, &out, &out_size);

      pcl_msg_t resp{};
      resp.type_name = "application/protobuf";
      if (rc == 0) {
        resp.data = out;
        resp.size = out_size;
      }
      (void)pcl_executor_post_response_msg(ctx->executor,
                                           item.response_callback,
                                           item.user_data,
                                           &resp);
      std::free(out);
      continue;
    }

    StreamTrampoline trampoline{ctx, item.stream_callback, item.user_data};
    const int rc = pyramid_grpc_plugin_client_invoke_stream(
        ctx->channel.c_str(), item.service_name.c_str(), request_data,
        request_size, grpcStreamOnItem, &trampoline);

    pcl_msg_t terminal{};
    terminal.type_name = "application/protobuf";
    (void)postStreamEvent(ctx,
                          item.stream_callback,
                          item.user_data,
                          &terminal,
                          true,
                          rc == 0 ? PCL_OK : PCL_ERR_NOT_FOUND);
  }
}

pcl_status_t grpcClientInvokeAsync(void*            ctx_v,
                                   const char*      service_name,
                                   const pcl_msg_t* request,
                                   pcl_resp_cb_fn_t callback,
                                   void*            user_data) {
  auto* ctx = static_cast<GrpcTransportContext*>(ctx_v);
  if (!ctx || ctx->channel.empty() || !service_name || !request || !callback) {
    return PCL_ERR_INVALID;
  }
  if (request->size > 0U && !request->data) return PCL_ERR_INVALID;

  GrpcWorkItem item;
  item.kind = GrpcWorkKind::Unary;
  item.service_name = service_name;
  item.response_callback = callback;
  item.user_data = user_data;
  try {
    if (request->size > 0U) {
      const auto* begin = static_cast<const unsigned char*>(request->data);
      item.request.assign(begin, begin + request->size);
    }
  } catch (...) {
    return PCL_ERR_NOMEM;
  }
  return enqueueClientWork(ctx, std::move(item));
}

void grpcStreamOnItem(const void* data, unsigned size, void* user) {
  auto* t = static_cast<StreamTrampoline*>(user);
  pcl_msg_t msg{};
  msg.data = data;
  msg.size = size;
  msg.type_name = "application/protobuf";
  (void)postStreamEvent(t->ctx, t->callback, t->user_data, &msg, false, PCL_OK);
}

pcl_status_t grpcClientInvokeStream(void*               ctx_v,
                                    const char*         service_name,
                                    const pcl_msg_t*    request,
                                    pcl_stream_msg_fn_t callback,
                                    void*               user_data,
                                    void**              stream_handle) {
  if (stream_handle) *stream_handle = nullptr;
  auto* ctx = static_cast<GrpcTransportContext*>(ctx_v);
  if (!ctx || ctx->channel.empty() || !service_name || !request || !callback) {
    return PCL_ERR_INVALID;
  }
  if (request->size > 0U && !request->data) return PCL_ERR_INVALID;

  GrpcWorkItem item;
  item.kind = GrpcWorkKind::Stream;
  item.service_name = service_name;
  item.stream_callback = callback;
  item.user_data = user_data;
  try {
    if (request->size > 0U) {
      const auto* begin = static_cast<const unsigned char*>(request->data);
      item.request.assign(begin, begin + request->size);
    }
  } catch (...) {
    return PCL_ERR_NOMEM;
  }
  return enqueueClientWork(ctx, std::move(item));
}

// -- Codec vtable: passthrough envelope codec for application/grpc --------

pcl_status_t grpcCodecEncode(void* /*codec_ctx*/,
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

pcl_status_t grpcCodecDecode(void* /*codec_ctx*/,
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

void grpcCodecFreeMsg(void* /*codec_ctx*/, pcl_msg_t* msg) {
  if (!msg) return;
  std::free(const_cast<void*>(msg->data));
  msg->data = nullptr;
  msg->size = 0U;
}

const pcl_codec_t grpc_codec = {
    PCL_CODEC_ABI_VERSION,
    kGrpcContentType,
    grpcCodecEncode,
    grpcCodecDecode,
    grpcCodecFreeMsg,
    nullptr,
};

}  // namespace

extern "C" {

PYRAMID_GRPC_PLUGIN_EXPORT uint32_t pcl_transport_abi_version() {
  return PCL_TRANSPORT_ABI_VERSION;
}

/// gRPC transport: unary + server-streaming RPC, both directions (server mode
/// hosts ingress, client mode invokes egress) -- the same capability set either
/// way. It does NOT carry PCL-native topic pub/sub: the publish/subscribe vtable
/// slots are fail-closed stubs (pub/sub over gRPC is modelled as rpc at the
/// contract level, or would need an explicit adapter -- WIP D5, deferred), so
/// vtable derivation would wrongly imply PUBSUB. Hence the explicit declaration.
PYRAMID_GRPC_PLUGIN_EXPORT pcl_transport_caps_t pcl_transport_plugin_caps(
    const char* config_json) {
  (void)config_json;
  return PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM;
}

/// gRPC is a reliable transport (HTTP/2 over TCP: ordered, retransmitted). The
/// loader treats a plugin with no QoS export as UNSPECIFIED, which would fail
/// compose-time validation for any `route ... reliable` line targeting gRPC even
/// though gRPC satisfies it -- so declare the reliable floor explicitly.
PYRAMID_GRPC_PLUGIN_EXPORT pcl_qos_t pcl_transport_plugin_qos(
    const char* config_json) {
  (void)config_json;
  pcl_qos_t qos;
  qos.reliability = PCL_QOS_RELIABILITY_RELIABLE;
  return qos;
}

PYRAMID_GRPC_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  if (!config_json) return nullptr;

  pcl_executor_t* executor = readExecutor(config_json);
  if (!executor) return nullptr;

  // Directional selector. The cross-plugin convention is role: "provided" (host
  // services) | "consumed" (dial a remote). mode: "server" | "client" is a
  // back-compatible alias; "server"/"client" are also accepted as role values.
  // When both are present, mode wins.
  char role[32];
  if (!readJsonString(config_json, "role", role, sizeof(role))) {
    role[0] = '\0';
  }

  char mode[16];
  if (!readJsonString(config_json, "mode", mode, sizeof(mode)) ||
      mode[0] == '\0') {
    if (std::strcmp(role, "consumed") == 0 || std::strcmp(role, "client") == 0) {
      std::strcpy(mode, "client");
    } else if (std::strcmp(role, "provided") == 0 ||
               std::strcmp(role, "server") == 0) {
      std::strcpy(mode, "server");
    } else {
      std::strcpy(mode, kDefaultMode);
    }
  }
  const bool is_client = std::strcmp(mode, "client") == 0 ||
                         std::strcmp(mode, "consumed") == 0;

  char address[256];
  if (!readJsonString(config_json, "address", address, sizeof(address)) ||
      address[0] == '\0') {
    std::strcpy(address, kDefaultAddress);
  }

  // -- Client (consumed) mode: dial a remote endpoint, no local server. --
  if (is_client) {
    auto* ctx = new (std::nothrow) GrpcTransportContext();
    if (!ctx) return nullptr;
    ctx->executor = executor;
    ctx->is_client = true;
    ctx->channel = address;
    ctx->transport.publish = grpcPublish;
    ctx->transport.serve = grpcServe;
    ctx->transport.subscribe = grpcSubscribe;
    ctx->transport.invoke_async = grpcClientInvokeAsync;
    ctx->transport.invoke_stream = grpcClientInvokeStream;
    ctx->transport.shutdown = grpcShutdown;
    ctx->transport.adapter_ctx = ctx;
    if (!startClientWorker(ctx)) {
      delete ctx;
      return nullptr;
    }
    return &ctx->transport;
  }

  // -- Server (provided) mode: host the configured component's services. --
  char component[128];
  if (!readJsonString(config_json, "component", component, sizeof(component)) ||
      component[0] == '\0') {
    return nullptr;
  }

  // Aggregator service role: the provided/consumed service set the server hosts.
  // A "consumed" role routes to the client branch above, and "server"/empty are
  // not aggregator roles, so a server's service set defaults to "provided".
  const char* service_role =
      (std::strcmp(role, "provided") == 0 || std::strcmp(role, "consumed") == 0)
          ? role
          : kDefaultRole;

  pyramid_grpc_plugin_server* server =
      pyramid_grpc_plugin_server_start(component, service_role, address,
                                       executor);
  if (!server) return nullptr;

  auto* ctx = new (std::nothrow) GrpcTransportContext();
  if (!ctx) {
    pyramid_grpc_plugin_server_stop(server);
    return nullptr;
  }
  ctx->executor = executor;
  ctx->server = server;
  ctx->transport.publish = grpcPublish;
  ctx->transport.serve = grpcServe;
  ctx->transport.subscribe = grpcSubscribe;
  ctx->transport.invoke_async = grpcInvokeAsync;
  ctx->transport.shutdown = grpcShutdown;
  ctx->transport.adapter_ctx = ctx;
  return &ctx->transport;
}

PYRAMID_GRPC_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(const char*) {
  return &grpc_codec;
}

PYRAMID_GRPC_PLUGIN_EXPORT void pcl_grpc_transport_plugin_destroy(
    const pcl_transport_t* transport) {
  destroyContext(contextOf(transport));
}

// Standard teardown symbol so a generic caller can use pcl_plugin_unload_transport
// to release the gRPC server/client + context BEFORE the .so is dlclose'd.
PYRAMID_GRPC_PLUGIN_EXPORT void pcl_transport_plugin_teardown(
    const pcl_transport_t* transport) {
  destroyContext(contextOf(transport));
}

}  // extern "C"
