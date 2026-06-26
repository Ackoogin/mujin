/// \file pcl_transport_socket_plugin.c
/// \brief Runtime-loadable TCP socket transport plugin.
#include "pcl/pcl_plugin.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport_socket.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  define PCL_SOCKET_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_SOCKET_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_SOCKET_PLUGIN_EXPORT
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

PCL_SOCKET_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}

PCL_SOCKET_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  char                    role[16];
  char                    host[256];
  uint64_t                port_raw = 0u;
  pcl_executor_t*         executor;
  pcl_socket_transport_t* socket_transport;

  if (!config_json) return NULL;
  if (!read_json_string(config_json, "role", role, sizeof(role))) return NULL;
  if (!read_json_u64(config_json, "port", &port_raw) || port_raw > 65535u) {
    return NULL;
  }
  executor = read_executor(config_json);
  if (!executor) return NULL;

  if (strcmp(role, "server") == 0) {
    socket_transport = pcl_socket_transport_create_server(
        (uint16_t)port_raw, executor);
  } else if (strcmp(role, "client") == 0) {
    pcl_socket_client_opts_t opts;
    memset(&opts, 0, sizeof(opts));
    opts.connect_timeout_ms = 3000u;
    opts.max_retries = 30u;

    if (!read_json_string(config_json, "host", host, sizeof(host))) {
      strcpy(host, "127.0.0.1");
    }
    socket_transport = pcl_socket_transport_create_client_ex(
        host, (uint16_t)port_raw, executor, &opts);
  } else {
    return NULL;
  }

  return pcl_socket_transport_get_transport(socket_transport);
}

PCL_SOCKET_PLUGIN_EXPORT pcl_container_t* pcl_socket_transport_plugin_gateway(
    const pcl_transport_t* transport) {
  if (!transport) return NULL;
  return pcl_socket_transport_gateway_container(
      (pcl_socket_transport_t*)transport->adapter_ctx);
}

PCL_SOCKET_PLUGIN_EXPORT void pcl_socket_transport_plugin_destroy(
    const pcl_transport_t* transport) {
  if (!transport) return;
  pcl_socket_transport_destroy((pcl_socket_transport_t*)transport->adapter_ctx);
}
