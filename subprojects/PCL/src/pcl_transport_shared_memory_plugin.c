/// \file pcl_transport_shared_memory_plugin.c
/// \brief Runtime-loadable shared-memory bus transport plugin.
///
/// Mirrors pcl_transport_socket_plugin.c: the loader threads an opaque
/// config_json into the entry point, which here carries the bus name, the
/// participant id, and the executor pointer to bind the transport to.
#include "pcl/pcl_plugin.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport_shared_memory.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  define PCL_SHM_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_SHM_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_SHM_PLUGIN_EXPORT
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

PCL_SHM_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}

PCL_SHM_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  char                           bus_name[256];
  char                           participant_id[128];
  pcl_executor_t*                executor;
  pcl_shared_memory_transport_t* shm_transport;

  if (!config_json) return NULL;
  if (!read_json_string(config_json, "bus_name", bus_name, sizeof(bus_name))) {
    return NULL;
  }
  if (!read_json_string(config_json, "participant_id", participant_id,
                        sizeof(participant_id))) {
    return NULL;
  }
  executor = read_executor(config_json);
  if (!executor) return NULL;

  shm_transport =
      pcl_shared_memory_transport_create(bus_name, participant_id, executor);
  if (!shm_transport) return NULL;

  return pcl_shared_memory_transport_get_transport(shm_transport);
}

PCL_SHM_PLUGIN_EXPORT pcl_container_t* pcl_shm_transport_plugin_gateway(
    const pcl_transport_t* transport) {
  if (!transport) return NULL;
  return pcl_shared_memory_transport_gateway_container(
      (pcl_shared_memory_transport_t*)transport->adapter_ctx);
}

PCL_SHM_PLUGIN_EXPORT void pcl_shm_transport_plugin_destroy(
    const pcl_transport_t* transport) {
  if (!transport) return;
  pcl_shared_memory_transport_destroy(
      (pcl_shared_memory_transport_t*)transport->adapter_ctx);
}
