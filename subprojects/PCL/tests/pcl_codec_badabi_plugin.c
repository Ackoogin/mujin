/// \file pcl_codec_badabi_plugin.c
/// \brief Test codec plugin that intentionally reports an unsupported ABI.
#include "pcl/pcl_codec.h"

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

static pcl_codec_t badabi_codec = {
  999u,
  "application/badabi",
  NULL,
  NULL,
  NULL,
  NULL
};

PCL_TEST_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(
    const char* config_json) {
  (void)config_json;
  return &badabi_codec;
}
