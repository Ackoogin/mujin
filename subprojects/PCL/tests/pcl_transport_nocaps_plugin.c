/// \file pcl_transport_nocaps_plugin.c
/// \brief Test transport plugin that deliberately omits the caps symbol.
///
/// Exercises the loader's fallback path: when no pcl_transport_plugin_caps
/// symbol is exported, capabilities are derived from the non-NULL vtable slots.
/// Its vtable advertises publish (PUBSUB) and invoke_async (RPC_UNARY) only.
#include "pcl/pcl_plugin.h"

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

static pcl_status_t nocaps_publish(void*            adapter_ctx,
                                   const char*      topic,
                                   const pcl_msg_t* msg) {
  (void)adapter_ctx;
  (void)topic;
  (void)msg;
  return PCL_OK;
}

static pcl_status_t nocaps_invoke_async(void*            adapter_ctx,
                                        const char*      service_name,
                                        const pcl_msg_t* request,
                                        pcl_resp_cb_fn_t callback,
                                        void*            user_data) {
  (void)adapter_ctx;
  (void)service_name;
  (void)request;
  (void)callback;
  (void)user_data;
  return PCL_OK;
}

static pcl_transport_t nocaps_transport = {
  nocaps_publish,       /* publish */
  NULL,                 /* serve */
  NULL,                 /* subscribe */
  nocaps_invoke_async,  /* invoke_async */
  NULL,                 /* respond */
  NULL,                 /* shutdown */
  NULL,                 /* invoke_stream */
  NULL,                 /* stream_send */
  NULL,                 /* stream_end */
  NULL,                 /* stream_cancel */
  NULL                  /* adapter_ctx */
};

PCL_TEST_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}

PCL_TEST_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  (void)config_json;
  return &nocaps_transport;
}

/* Intentionally no pcl_transport_plugin_caps symbol. */
