/// \file pcl_codec_ports2_plugin.c
/// \brief Test codec plugin used only by the per-port `port_codec` directive
///        test. It registers one fixed vtable under a content type no other
///        test uses, so loading it via a ports file does not collide with the
///        process-global codec registry state the other codec tests rely on.
#include "pcl/pcl_codec.h"

#include <string.h>

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

static pcl_status_t ports2_encode(void*       codec_ctx,
                                 const char* schema_id,
                                 const void* value,
                                 pcl_msg_t*  out_msg) {
  (void)codec_ctx;
  (void)schema_id;
  (void)value;
  if (!out_msg) return PCL_ERR_INVALID;
  memset(out_msg, 0, sizeof(*out_msg));
  return PCL_OK;
}

static pcl_status_t ports2_decode(void*            codec_ctx,
                                 const char*      schema_id,
                                 const pcl_msg_t* msg,
                                 void*            out_value) {
  (void)codec_ctx;
  (void)schema_id;
  (void)msg;
  return out_value ? PCL_OK : PCL_ERR_INVALID;
}

static void ports2_free_msg(void* codec_ctx, pcl_msg_t* msg) {
  (void)codec_ctx;
  if (msg) memset(msg, 0, sizeof(*msg));
}

static pcl_codec_t ports2_codec = {
  PCL_CODEC_ABI_VERSION,
  "application/pcl-ports-test-2",
  ports2_encode,
  ports2_decode,
  ports2_free_msg,
  NULL
};

PCL_TEST_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(
    const char* config_json) {
  (void)config_json;
  return &ports2_codec;
}
