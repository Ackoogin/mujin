/// \file pcl_transport_udp_plugin.c
/// \brief Runtime-loadable UDP datagram transport plugin.
///
/// Mirrors pcl_transport_socket_plugin.c and
/// pcl_transport_shared_memory_plugin.c: the loader threads an opaque
/// config_json into the entry point, which here carries the local bind
/// port, the remote peer endpoint, an optional logical peer id, and the
/// executor pointer to bind the transport to.
///
/// UDP is pub/sub-only (no service RPC), so this plugin exports no
/// gateway-container hook -- only the create/destroy lifecycle the loader
/// needs for a publisher/subscriber transport.
#include "pcl/pcl_plugin.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport_udp.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  define PCL_UDP_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_UDP_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_UDP_PLUGIN_EXPORT
#endif

static const char* find_json_value(const char* json, const char* key) {
  const char* p;
  char needle[64];

  if (!json || !key) return NULL;
  if (strlen(key) + 3u >= sizeof(needle)) return NULL;

  needle[0] = '"';
  strcpy(needle + 1u, key);
  strcat(needle, "\"");

  p = strstr(json, needle);
  if (!p) return NULL;
  p += strlen(needle);
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
  if (*p != ':') return NULL;
  ++p;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
  return p;
}

static int read_json_string(const char* json,
                            const char* key,
                            char*       out,
                            size_t      out_size) {
  const char* p;
  size_t      n = 0u;

  if (!out || out_size == 0u) return 0;
  out[0] = '\0';
  p = find_json_value(json, key);
  if (!p || *p != '"') return 0;
  ++p;
  while (p[n] && p[n] != '"') {
    if (n + 1u >= out_size) return 0;
    out[n] = p[n];
    ++n;
  }
  if (p[n] != '"') return 0;
  out[n] = '\0';
  return 1;
}

static int read_json_u64(const char* json, const char* key, uint64_t* out) {
  const char* p;
  char*       end;
  uint64_t    value;

  if (!out) return 0;
  p = find_json_value(json, key);
  if (!p) return 0;
  if (*p == '"') ++p;
  value = (uint64_t)strtoull(p, &end, 0);
  if (end == p) return 0;
  *out = value;
  return 1;
}

static pcl_executor_t* read_executor(const char* config_json) {
  uint64_t raw = 0u;
  if (!read_json_u64(config_json, "executor", &raw) || raw == 0u) {
    return NULL;
  }
  return (pcl_executor_t*)(uintptr_t)raw;
}

PCL_UDP_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}

/* Datagram pub/sub transport: publish + subscribe only. */
PCL_UDP_PLUGIN_EXPORT pcl_transport_caps_t pcl_transport_plugin_caps(
    const char* config_json) {
  (void)config_json;
  return PCL_CAP_PUBSUB;
}

PCL_UDP_PLUGIN_EXPORT pcl_qos_t pcl_transport_plugin_qos(
    const char* config_json) {
  pcl_qos_t qos;
  (void)config_json;
  /* UDP datagrams may be dropped or reordered. */
  qos.reliability = PCL_QOS_RELIABILITY_BEST_EFFORT;
  return qos;
}

PCL_UDP_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  char                 remote_host[256];
  char                 peer_id[128];
  uint64_t             local_port_raw  = 0u;
  uint64_t             remote_port_raw = 0u;
  pcl_executor_t*      executor;
  pcl_udp_transport_t* udp_transport;

  if (!config_json) return NULL;
  if (!read_json_string(config_json, "remote_host", remote_host,
                        sizeof(remote_host))) {
    return NULL;
  }
  if (!read_json_u64(config_json, "remote_port", &remote_port_raw) ||
      remote_port_raw > 65535u) {
    return NULL;
  }
  /* local_port is optional: absent or 0 means an OS-assigned ephemeral port. */
  if (read_json_u64(config_json, "local_port", &local_port_raw) &&
      local_port_raw > 65535u) {
    return NULL;
  }
  executor = read_executor(config_json);
  if (!executor) return NULL;

  udp_transport = pcl_udp_transport_create(
      (uint16_t)local_port_raw, remote_host, (uint16_t)remote_port_raw,
      executor);
  if (!udp_transport) return NULL;

  /* peer_id is optional; only override the default when supplied. */
  if (read_json_string(config_json, "peer_id", peer_id, sizeof(peer_id)) &&
      peer_id[0] != '\0') {
    pcl_udp_transport_set_peer_id(udp_transport, peer_id);
  }

  return pcl_udp_transport_get_transport(udp_transport);
}

PCL_UDP_PLUGIN_EXPORT void pcl_udp_transport_plugin_destroy(
    const pcl_transport_t* transport) {
  if (!transport) return;
  pcl_udp_transport_destroy((pcl_udp_transport_t*)transport->adapter_ctx);
}

/* Standard teardown symbol: stop the recv thread + close the socket before the
   .so is unloaded (pcl_plugin_unload_transport), so manifest-driven routing can
   release the transport without leaking its executor-bound thread. */
PCL_UDP_PLUGIN_EXPORT void pcl_transport_plugin_teardown(
    const pcl_transport_t* transport) {
  if (!transport) return;
  pcl_udp_transport_destroy((pcl_udp_transport_t*)transport->adapter_ctx);
}
