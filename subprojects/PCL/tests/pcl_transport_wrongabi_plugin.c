/// \file pcl_transport_wrongabi_plugin.c
/// \brief Test transport plugin that intentionally reports an unsupported ABI.
#include "pcl/pcl_plugin.h"

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

PCL_TEST_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION + 1000u;
}

PCL_TEST_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  (void)config_json;
  return (const pcl_transport_t*)0;
}
