/// \file pcl_codec_sequence_plugin.c
/// \brief Test plugin returning distinct codec vtables on repeated loads.
#include "pcl/pcl_codec.h"

#include <string.h>

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

#define PCL_TEST_CODEC_COUNT 32u

static pcl_codec_t codecs[PCL_TEST_CODEC_COUNT];
static uint32_t next_codec;
static int initialized;

static pcl_status_t sequence_encode(
    void* codec_ctx,
    const char* schema_id,
    const void* value,
    pcl_msg_t* out_msg) {
  (void)codec_ctx;
  (void)schema_id;
  (void)value;
  if (!out_msg) return PCL_ERR_INVALID;
  memset(out_msg, 0, sizeof(*out_msg));
  return PCL_OK;
}

static pcl_status_t sequence_decode(
    void* codec_ctx,
    const char* schema_id,
    const pcl_msg_t* msg,
    void* out_value) {
  (void)codec_ctx;
  (void)schema_id;
  (void)msg;
  return out_value ? PCL_OK : PCL_ERR_INVALID;
}

static void sequence_free(void* codec_ctx, pcl_msg_t* msg) {
  (void)codec_ctx;
  if (msg) memset(msg, 0, sizeof(*msg));
}

PCL_TEST_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(
    const char* config_json) {
  uint32_t index;
  (void)config_json;
  if (!initialized) {
    for (index = 0u; index < PCL_TEST_CODEC_COUNT; ++index) {
      codecs[index].abi_version = PCL_CODEC_ABI_VERSION;
      codecs[index].content_type = "application/pcl-runtime-sequence";
      codecs[index].encode = sequence_encode;
      codecs[index].decode = sequence_decode;
      codecs[index].free_msg = sequence_free;
      codecs[index].codec_ctx = NULL;
    }
    initialized = 1;
  }
  if (next_codec >= PCL_TEST_CODEC_COUNT) {
    return &codecs[PCL_TEST_CODEC_COUNT - 1u];
  }
  return &codecs[next_codec++];
}
