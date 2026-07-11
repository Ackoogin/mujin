/// \file pyramid_lacal_transport_plugin.cpp
/// \brief Runtime-loadable LA-CAL (OMS WebSocket Protocol) transport plugin.
///
/// Rung 1 of the OMS CAL join (doc/plans/PYRAMID/la_cal_integration_plan.md):
/// a PUBSUB-only PCL transport that joins an OMS Abstract Service Bus through a
/// Language-Agnostic CAL server (OMSC-SPC-013). It wraps the PCL-free `owp`
/// client core (subprojects/PYRAMID/src/owp) -- the client owns the WebSocket
/// worker thread; this plugin only maps the PCL transport vtable onto it.
///
/// Structural model: pcl_transport_udp_plugin.c (the existing PUBSUB-only
/// plugin). Like it, the loader threads an opaque config_json carrying the
/// executor pointer to bind ingress to.
///
/// Config contract (config_json passed to pcl_transport_plugin_entry):
///   {
///     "executor":            <uintptr to pcl_executor_t* as decimal/hex>, (required)
///     "url":                 "ws://127.0.0.1:PORT",                        (required)
///     "service_id":          "my-service",                                 (required)
///     "schema":              "002.5.0",              (optional, default "002.5.0")
///     "content_type":        "application/oms-json", (optional; guarded, see D1)
///     "peer_id":             "asb",                  (optional, default "lacal")
///     "verbose":             true,                   (optional, default true)
///     "connect_timeout_ms":  3000,                   (optional, default 3000)
///     "declare_reliability": "reliable"              (optional; see D2/QoS)
///   }
///
/// D1 (content-type guard): the PCL message ABI (pcl_msg_t) carries no content
/// type, so a per-publish check is not expressible. Instead the guard is applied
/// at pairing time: if config declares a content_type other than
/// application/oms-json the entry point fails closed (returns NULL), so this
/// transport can never be wired behind a codec that emits, e.g., proto3-JSON.
///
/// D2 (QoS): OMSC-SPC-013 does not specify delivery guarantees, so the plugin
/// declares BEST_EFFORT by default. A deployment that knows its CAL server
/// buffers/redelivers may opt in with "declare_reliability":"reliable"; that is
/// the only way this transport advertises RELIABLE (mirrors the UDP downgrade
/// convention).
///
/// D3 (fail-closed init): pcl_transport_plugin_entry performs the INIT -> INFO
/// handshake synchronously with a bounded timeout and returns NULL if INFO does
/// not arrive -- the loud surface for a CAL server's silent rejection of an
/// unregistered service. Diagnostics name the URL and service_id.
///
/// D6 (identity from INFO): the System/Service UUIDs returned in INFO are stored
/// on the adapter and exposed via pyramid_lacal_transport_system_uuid /
/// _service_uuid for a future bridge's header population.

#include <pcl/pcl_plugin.h>

extern "C" {
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
}

#include "owp_client.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>

#if defined(_WIN32)
#  define PCL_LACAL_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_LACAL_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_LACAL_PLUGIN_EXPORT
#endif

namespace {

// -- Minimal opaque-JSON config reads (same convention as the udp plugin) ----

const char* find_json_value(const char* json, const char* key) {
  if (!json || !key) return nullptr;
  char needle[64];
  if (std::strlen(key) + 3u >= sizeof(needle)) return nullptr;
  needle[0] = '"';
  std::strcpy(needle + 1u, key);
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

bool read_json_string(const char* json, const char* key, std::string* out) {
  const char* p = find_json_value(json, key);
  if (!p || *p != '"') return false;
  ++p;
  out->clear();
  while (*p && *p != '"') {
    *out += *p;
    ++p;
  }
  return *p == '"';
}

bool read_json_u64(const char* json, const char* key, uint64_t* out) {
  const char* p = find_json_value(json, key);
  if (!p) return false;
  if (*p == '"') ++p;
  char* end = nullptr;
  const uint64_t value = std::strtoull(p, &end, 0);
  if (end == p) return false;
  *out = value;
  return true;
}

// -- Adapter context ---------------------------------------------------------

struct LacalContext {
  pcl_transport_t     transport{};
  pyramid::owp::Client client;
  pcl_executor_t*     executor = nullptr;
  std::string         peer_id;
  std::string         content_type;
  pyramid::owp::Info  identity;
  uint64_t            sub_counter = 0u;
  std::mutex          mutex;                                 // guards topic_type + sub_counter
  std::unordered_map<std::string, std::string> topic_type;  // topic -> UCI message name
};

LacalContext* ctx_of(void* adapter_ctx) {
  return static_cast<LacalContext*>(adapter_ctx);
}

// -- Vtable: PUBSUB only -----------------------------------------------------

pcl_status_t lacal_publish(void* adapter_ctx, const char* topic,
                           const pcl_msg_t* msg) {
  LacalContext* ctx = ctx_of(adapter_ctx);
  if (!ctx || !topic || !msg) return PCL_ERR_INVALID;
  try {
    // owp PUB carries the codec-produced OMS JSON body verbatim; publish()
    // validates the topic, copies, and enqueues -- it never blocks on the
    // socket, per the PCL transport threading contract.
    ctx->client.publish(
        topic,
        std::string(static_cast<const char*>(msg->data), msg->size));
  } catch (const std::exception&) {
    return PCL_ERR_STATE;
  }
  return PCL_OK;
}

pcl_status_t lacal_subscribe(void* adapter_ctx, const char* topic,
                             const char* type_name) {
  LacalContext* ctx = ctx_of(adapter_ctx);
  if (!ctx || !topic) return PCL_ERR_INVALID;
  // owp SUB needs the UCI message name; the contract's type_name carries it.
  // Fall back to the topic when absent (D5: topic == UCI message name).
  const std::string message_name =
      (type_name && *type_name) ? std::string(type_name) : std::string(topic);
  std::string sub_id;
  {
    std::lock_guard<std::mutex> lock(ctx->mutex);
    ctx->topic_type[topic] = message_name;
    sub_id = "s" + std::to_string(ctx->sub_counter++);
  }
  try {
    ctx->client.subscribe(sub_id, message_name, topic);
  } catch (const std::exception&) {
    return PCL_ERR_STATE;
  }
  return PCL_OK;
}

// -- Ingress: owp MSG (worker thread) -> executor ----------------------------

void install_ingress(LacalContext* ctx) {
  ctx->client.setMessageHandler(
      [ctx](const std::string& /*sub_id*/, const std::string& topic,
            const std::string& body) {
        // Runs on the owp worker thread. post_remote_incoming deep-copies the
        // message into the executor's ingress queue, so the borrowed pointers
        // below need only outlive this call.
        std::string type_name;
        {
          std::lock_guard<std::mutex> lock(ctx->mutex);
          auto it = ctx->topic_type.find(topic);
          if (it != ctx->topic_type.end()) type_name = it->second;
        }
        pcl_msg_t msg;
        msg.data = body.data();
        msg.size = static_cast<uint32_t>(body.size());
        msg.type_name = type_name.c_str();
        (void)pcl_executor_post_remote_incoming(
            ctx->executor, ctx->peer_id.c_str(), topic.c_str(), &msg);
      });
}

}  // namespace

extern "C" {

PCL_LACAL_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}

/* PUBSUB-only: serve / invoke_* / stream_* vtable slots are NULL, so vtable
   derivation would agree, but declare explicitly for clarity/robustness. */
PCL_LACAL_PLUGIN_EXPORT pcl_transport_caps_t pcl_transport_plugin_caps(
    const char* config_json) {
  (void)config_json;
  return PCL_CAP_PUBSUB;
}

PCL_LACAL_PLUGIN_EXPORT pcl_qos_t pcl_transport_plugin_qos(
    const char* config_json) {
  pcl_qos_t qos;
  std::string declared;
  if (read_json_string(config_json, "declare_reliability", &declared) &&
      declared == "reliable") {
    qos.reliability = PCL_QOS_RELIABILITY_RELIABLE;
  } else {
    qos.reliability = PCL_QOS_RELIABILITY_BEST_EFFORT;
  }
  return qos;
}

PCL_LACAL_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  if (!config_json) return nullptr;

  std::string url;
  std::string service_id;
  if (!read_json_string(config_json, "url", &url) || url.empty()) return nullptr;
  if (!read_json_string(config_json, "service_id", &service_id) ||
      service_id.empty()) {
    return nullptr;
  }

  std::string content_type;
  if (read_json_string(config_json, "content_type", &content_type) &&
      content_type != "application/oms-json") {
    // D1: fail closed on a mispaired codec content type.
    std::fprintf(stderr,
                 "[lacal] refusing content_type '%s' (only application/oms-json)\n",
                 content_type.c_str());
    return nullptr;
  }
  if (content_type.empty()) content_type = "application/oms-json";

  std::string schema;
  if (!read_json_string(config_json, "schema", &schema) || schema.empty()) {
    schema = "002.5.0";
  }
  std::string peer_id;
  if (!read_json_string(config_json, "peer_id", &peer_id) || peer_id.empty()) {
    peer_id = "lacal";
  }

  uint64_t executor_raw = 0u;
  if (!read_json_u64(config_json, "executor", &executor_raw) ||
      executor_raw == 0u) {
    return nullptr;
  }

  bool verbose = true;
  {
    std::string verbose_str;
    if (read_json_string(config_json, "verbose", &verbose_str)) {
      verbose = (verbose_str != "false");
    } else {
      const char* v = find_json_value(config_json, "verbose");
      if (v && std::strncmp(v, "false", 5) == 0) verbose = false;
    }
  }

  uint64_t timeout_ms = 3000u;
  read_json_u64(config_json, "connect_timeout_ms", &timeout_ms);

  auto* ctx = new (std::nothrow) LacalContext();
  if (!ctx) return nullptr;
  ctx->executor = reinterpret_cast<pcl_executor_t*>(
      static_cast<uintptr_t>(executor_raw));
  ctx->peer_id = peer_id;
  ctx->content_type = content_type;
  install_ingress(ctx);

  // D3: synchronous, bounded INIT -> INFO. A CAL server that silently rejects
  // an unregistered service never answers INFO; surface it loudly and fail
  // closed rather than hand back a half-open transport.
  try {
    const auto timeout = std::chrono::milliseconds(static_cast<long>(timeout_ms));
    ctx->client.connect(url, timeout);
    ctx->identity =
        ctx->client.init({"1.0"}, schema, service_id, verbose, timeout);
  } catch (const std::exception& e) {
    std::fprintf(stderr,
                 "[lacal] INIT failed for url=%s service_id=%s: %s\n",
                 url.c_str(), service_id.c_str(), e.what());
    delete ctx;
    return nullptr;
  }

  ctx->transport.publish = lacal_publish;
  ctx->transport.subscribe = lacal_subscribe;
  ctx->transport.adapter_ctx = ctx;
  return &ctx->transport;
}

PCL_LACAL_PLUGIN_EXPORT void pcl_transport_plugin_teardown(
    const pcl_transport_t* transport) {
  if (!transport) return;
  LacalContext* ctx = ctx_of(transport->adapter_ctx);
  if (!ctx) return;
  ctx->client.close();  // stops + joins the owp worker before dlclose
  delete ctx;
}

// -- D6 identity accessors (for the future agra_c2_bridge header population) --

PCL_LACAL_PLUGIN_EXPORT const char* pyramid_lacal_transport_system_uuid(
    const pcl_transport_t* transport) {
  if (!transport) return nullptr;
  LacalContext* ctx = ctx_of(transport->adapter_ctx);
  return ctx ? ctx->identity.uuids.system.c_str() : nullptr;
}

PCL_LACAL_PLUGIN_EXPORT const char* pyramid_lacal_transport_service_uuid(
    const pcl_transport_t* transport) {
  if (!transport) return nullptr;
  LacalContext* ctx = ctx_of(transport->adapter_ctx);
  return ctx ? ctx->identity.uuids.service.c_str() : nullptr;
}

}  // extern "C"
