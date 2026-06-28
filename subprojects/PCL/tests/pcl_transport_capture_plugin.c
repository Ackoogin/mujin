/// \file pcl_transport_capture_plugin.c
/// \brief Test transport plugin that records publish and serve calls.
#include "pcl/pcl_plugin.h"

#include <string.h>

#if defined(_WIN32)
#  define PCL_TEST_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_TEST_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_TEST_PLUGIN_EXPORT
#endif

typedef struct {
  uint32_t publish_count;
  uint32_t serve_count;
  char     last_topic[128];
} capture_state_t;

static capture_state_t capture_state;

static pcl_status_t capture_publish(void*            adapter_ctx,
                                    const char*      topic,
                                    const pcl_msg_t* msg) {
  capture_state_t* state = (capture_state_t*)adapter_ctx;
  (void)msg;

  if (!state || !topic) return PCL_ERR_INVALID;
  state->publish_count++;
  strncpy(state->last_topic, topic, sizeof(state->last_topic) - 1u);
  state->last_topic[sizeof(state->last_topic) - 1u] = '\0';
  return PCL_OK;
}

static pcl_status_t capture_serve(void*            adapter_ctx,
                                  const char*      service_name,
                                  const pcl_msg_t* request,
                                  pcl_msg_t*       response) {
  capture_state_t* state = (capture_state_t*)adapter_ctx;
  (void)service_name;
  (void)request;
  (void)response;

  if (!state) return PCL_ERR_INVALID;
  state->serve_count++;
  return PCL_OK;
}

static pcl_transport_t capture_transport = {
  capture_publish,
  capture_serve,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  &capture_state
};

PCL_TEST_PLUGIN_EXPORT uint32_t pcl_transport_abi_version(void) {
  return PCL_TRANSPORT_ABI_VERSION;
}

PCL_TEST_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char* config_json) {
  (void)config_json;
  return &capture_transport;
}

/* Declare capabilities explicitly: pub/sub + unary from the vtable, plus an
 * action capability that has no vtable slot and so could never be derived.
 * This exercises the loader's explicit-symbol path overriding derivation. */
PCL_TEST_PLUGIN_EXPORT pcl_transport_caps_t pcl_transport_plugin_caps(
    const char* config_json) {
  (void)config_json;
  return PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_ACTION;
}

PCL_TEST_PLUGIN_EXPORT void pcl_capture_reset(void) {
  memset(&capture_state, 0, sizeof(capture_state));
}

PCL_TEST_PLUGIN_EXPORT uint32_t pcl_capture_publish_count(void) {
  return capture_state.publish_count;
}

PCL_TEST_PLUGIN_EXPORT const char* pcl_capture_last_topic(void) {
  return capture_state.last_topic;
}

PCL_TEST_PLUGIN_EXPORT uint32_t pcl_capture_serve_count(void) {
  return capture_state.serve_count;
}
