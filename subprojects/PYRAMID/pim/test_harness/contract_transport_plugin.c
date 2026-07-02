/// \file contract_transport_plugin.c
/// \brief Minimal reliable transport plugin for contract-routing validation.
#include "pcl/pcl_plugin.h"

#include <string.h>

#if defined(_WIN32)
#  define CONTRACT_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define CONTRACT_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define CONTRACT_PLUGIN_EXPORT
#endif

static pcl_transport_t contract_transport = {
  NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

static int config_has(const char* config_json, const char* needle) {
  return config_json && strstr(config_json, needle) != NULL;
}

CONTRACT_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}

CONTRACT_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  (void)config_json;
  return &contract_transport;
}

CONTRACT_PLUGIN_EXPORT pcl_transport_caps_t pcl_transport_plugin_caps(
    const char* config_json) {
  if (config_has(config_json, "\"mode\":\"pubsub\"")) {
    return PCL_CAP_PUBSUB;
  }
  if (config_has(config_json, "\"mode\":\"rpc\"")) {
    return PCL_CAP_RPC_UNARY;
  }
  return PCL_CAP_NONE;
}

CONTRACT_PLUGIN_EXPORT pcl_qos_t pcl_transport_plugin_qos(
    const char* config_json) {
  pcl_qos_t qos;
  (void)config_json;
  qos.reliability = PCL_QOS_RELIABILITY_RELIABLE;
  return qos;
}
