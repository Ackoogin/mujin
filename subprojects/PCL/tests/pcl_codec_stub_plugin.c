/// \file pcl_codec_stub_plugin.c
/// \brief Test codec plugin used by plugin-loader integration tests.
#include "pcl/pcl_codec.h"

#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

static const char stub_marker[] = "STUB_ENCODED";

static pcl_status_t stub_encode(void*       codec_ctx,
                                const char* schema_id,
                                const void* value,
                                pcl_msg_t*  out_msg) {
  char* copy;
  size_t size;

  (void)codec_ctx;
  (void)value;

  if (!out_msg) return PCL_ERR_INVALID;

  size = strlen(stub_marker) + 1u;
  copy = (char*)malloc(size);
  if (!copy) return PCL_ERR_NOMEM;

  memcpy(copy, stub_marker, size);
  out_msg->data = copy;
  out_msg->size = (uint32_t)size;
  out_msg->type_name = schema_id;
  return PCL_OK;
}

static pcl_status_t stub_decode(void*            codec_ctx,
                                const char*      schema_id,
                                const pcl_msg_t* msg,
                                void*            out_value) {
  (void)codec_ctx;
  (void)schema_id;
  (void)msg;

  if (!out_value) return PCL_ERR_INVALID;
  memcpy(out_value, stub_marker, sizeof(stub_marker));
  return PCL_OK;
}

static void stub_free_msg(void* codec_ctx, pcl_msg_t* msg) {
  (void)codec_ctx;
  if (!msg) return;
  free((void*)msg->data);
  msg->data = NULL;
  msg->size = 0u;
  msg->type_name = NULL;
}

static pcl_codec_t stub_codec = {
  PCL_CODEC_ABI_VERSION,
  "application/stub",
  stub_encode,
  stub_decode,
  stub_free_msg,
  NULL
};

PCL_TEST_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(void) {
  return &stub_codec;
}
