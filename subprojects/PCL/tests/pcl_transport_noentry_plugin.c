/// \file pcl_transport_noentry_plugin.c
/// \brief Test transport plugin with a valid ABI symbol but no entry point.
#include "pcl/pcl_plugin.h"

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

PCL_TEST_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}
