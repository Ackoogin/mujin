/// \file pcl_bridge.c
/// \brief PCL bridge implementation.
///
/// A bridge is a thin managed container.  Its only port is one subscriber on
/// the input topic.  When a message arrives the transform function is called
/// and, on success, the result is dispatched synchronously to the output topic
/// via pcl_executor_dispatch_incoming() — i.e. on the same executor tick,
/// zero-copy from the transform's stack allocation.
#include "pcl/pcl_bridge.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// -- Internal representation ----------------------------------------------

struct pcl_bridge_t {
  pcl_container_t* container;
  pcl_executor_t*  executor;
  pcl_bridge_fn_t  fn;
  void*            user_data;
  char             in_topic [128];
  char             in_type  [128];
  char             out_topic[128];
  char             out_type [128];
};

// -- Subscriber callback -------------------------------------------------

static void bridge_sub_cb(pcl_container_t* c,
                           const pcl_msg_t* in,
                           void*            ud) {
  pcl_bridge_t* b = (pcl_bridge_t*)ud;
  pcl_msg_t     out;
  pcl_status_t  rc;

  (void)c;

  memset(&out, 0, sizeof(out));
  out.type_name = b->out_type;   /* pre-fill; transform may override */

  rc = b->fn(in, &out, b->user_data);
  if (rc != PCL_OK) {
    pcl_log(c, PCL_LOG_DEBUG, "bridge transform suppressed message (rc=%d)", rc);
    return;
  }

  /* Dispatch synchronously on the executor thread.
   * NOT_FOUND is benign — no downstream subscriber yet; log at DEBUG. */
  rc = pcl_executor_dispatch_incoming(b->executor, b->out_topic, &out);
  if (rc == PCL_ERR_NOT_FOUND) {
    pcl_log(c, PCL_LOG_DEBUG,
            "bridge '%s': no subscriber on out_topic '%s'",
            pcl_container_name(c), b->out_topic);
  }
}

// -- on_configure: wire up the subscriber port ---------------------------

static pcl_status_t bridge_on_configure(pcl_container_t* c, void* ud) {
  pcl_bridge_t* b = (pcl_bridge_t*)ud;
  pcl_port_t*   p;

  p = pcl_container_add_subscriber(c, b->in_topic, b->in_type,
                                   bridge_sub_cb, b);
  if (!p) {
    pcl_log(c, PCL_LOG_ERROR, "bridge failed to add subscriber for '%s'",
            b->in_topic);
    return PCL_ERR_NOMEM;
  }
  return PCL_OK;
}

// -- Public API ---------------------------------------------------------

pcl_bridge_t* pcl_bridge_create(pcl_executor_t*  executor,
                                 const char*      name,
                                 const char*      in_topic,
                                 const char*      in_type,
                                 const char*      out_topic,
                                 const char*      out_type,
                                 pcl_bridge_fn_t  fn,
                                 void*            user_data) {
  pcl_bridge_t*   b;
  pcl_callbacks_t cbs;

  if (!executor || !name || !in_topic || !in_type ||
      !out_topic || !out_type || !fn) {
    return NULL;
  }

  b = (pcl_bridge_t*)calloc(1, sizeof(pcl_bridge_t));
  if (!b) return NULL;

  b->executor  = executor;
  b->fn        = fn;
  b->user_data = user_data;
  snprintf(b->in_topic,  sizeof(b->in_topic),  "%s", in_topic);
  snprintf(b->in_type,   sizeof(b->in_type),   "%s", in_type);
  snprintf(b->out_topic, sizeof(b->out_topic), "%s", out_topic);
  snprintf(b->out_type,  sizeof(b->out_type),  "%s", out_type);

  memset(&cbs, 0, sizeof(cbs));
  cbs.on_configure = bridge_on_configure;

  b->container = pcl_container_create(name, &cbs, b);
  if (!b->container) {
    free(b);
    return NULL;
  }

  return b;
}

pcl_container_t* pcl_bridge_container(pcl_bridge_t* b) {
  if (!b) return NULL;
  return b->container;
}

void pcl_bridge_destroy(pcl_bridge_t* b) {
  if (!b) return;
  pcl_container_destroy(b->container);
  free(b);
}
