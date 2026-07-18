/// \file pcl_container.c
/// \brief PCL container implementation — lifecycle, parameters, and ports.
#include "pcl_internal.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_alloc.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// -- Helpers -------------------------------------------------------------

static pcl_param_t* find_param(const pcl_container_t* c, const char* key) {
  uint32_t i;
  for (i = 0; i < c->param_count; ++i) {
    if (strcmp(c->params[i].key, key) == 0) {
      return (pcl_param_t*)&c->params[i];
    }
  }
  return NULL;
}

/* Implements: REQ_PCL_016. */
static pcl_param_t* find_or_add_param(pcl_container_t* c, const char* key) {
  pcl_param_t* p = find_param(c, key);
  if (p) return p;
  if (c->param_count >= PCL_MAX_PARAMS) return NULL;
  p = &c->params[c->param_count++];
  snprintf(p->key, sizeof(p->key), "%s", key);
  return p;
}

/* Implements: REQ_PCL_020, REQ_PCL_021, REQ_PCL_022. */
static int validate_port_definition(pcl_container_t* c,
                                    const char*      endpoint_name,
                                    const char*      content_type,
                                    const char*      port_kind) {
  if (!c) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "cannot add %s port: container is null", port_kind);
    return 0;
  }
  if (!endpoint_name) {
    pcl_log(c, PCL_LOG_ERROR,
            "cannot add %s port: endpoint name is null", port_kind);
    return 0;
  }
  if (!content_type) {
    pcl_log(c, PCL_LOG_ERROR,
            "cannot add %s port '%s': content type is missing",
            port_kind, endpoint_name);
    return 0;
  }
  if (!c->configuring) {
    pcl_log(c, PCL_LOG_ERROR,
            "cannot add %s port '%s': ports may only be added during configuration",
            port_kind, endpoint_name);
    return 0;
  }
  if (c->port_count >= PCL_MAX_PORTS) {
    pcl_log(c, PCL_LOG_ERROR,
            "cannot add %s port '%s': container limit of %u ports was reached",
            port_kind, endpoint_name, (unsigned)PCL_MAX_PORTS);
    return 0;
  }
  return 1;
}

/* Implements: REQ_PCL_235. */
static pcl_status_t copy_route_config(pcl_port_t*              port,
                                      uint32_t                 route_mode,
                                      const char* const*       peer_ids,
                                      uint32_t                 peer_count) {
  uint32_t i;

  if (!port) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "port route configuration failed: port is null");
    return PCL_ERR_INVALID;
  }
  if (route_mode == PCL_ROUTE_NONE) {
    pcl_log(port->owner, PCL_LOG_ERROR,
            "port route configuration failed for '%s': route mode is NONE",
            port->name);
    return PCL_ERR_INVALID;
  }
  if (peer_count > PCL_MAX_ENDPOINT_PEERS) {
    pcl_log(port->owner, PCL_LOG_ERROR,
            "port route configuration failed for '%s': %u peers exceeds the limit of %u",
            port->name, (unsigned)peer_count,
            (unsigned)PCL_MAX_ENDPOINT_PEERS);
    return PCL_ERR_INVALID;
  }
  if (peer_count > 0u && !peer_ids) {
    pcl_log(port->owner, PCL_LOG_ERROR,
            "port route configuration failed for '%s': peer count is %u but the peer list is null",
            port->name, (unsigned)peer_count);
    return PCL_ERR_INVALID;
  }
  if ((route_mode & PCL_ROUTE_REMOTE) == 0u && peer_count > 0u) {
    pcl_log(port->owner, PCL_LOG_ERROR,
            "port route configuration failed for '%s': peer IDs require a remote route",
            port->name);
    return PCL_ERR_INVALID;
  }

  port->route_mode = route_mode;
  port->route_configured = 1;
  port->peer_count = peer_count;
  for (i = 0; i < PCL_MAX_ENDPOINT_PEERS; ++i) {
    port->peer_ids[i][0] = '\0';
  }
  for (i = 0; i < peer_count; ++i) {
    if (!peer_ids[i]) {
      pcl_log(port->owner, PCL_LOG_ERROR,
              "port route configuration failed for '%s': peer ID %u is null",
              port->name, (unsigned)i);
      return PCL_ERR_INVALID;
    }
    snprintf(port->peer_ids[i], sizeof(port->peer_ids[i]), "%s", peer_ids[i]);
  }
  return PCL_OK;
}

// -- Create / destroy ----------------------------------------------------

/* Implements: REQ_PCL_001, REQ_PCL_002, REQ_PCL_028, REQ_PCL_114. */
pcl_container_t* pcl_container_create(const char*            name,
                                      const pcl_callbacks_t* callbacks,
                                      void*                  user_data) {
  if (!name) return NULL;

  pcl_container_t* c = (pcl_container_t*)pcl_calloc(1, sizeof(pcl_container_t));
  if (!c) return NULL;

  snprintf(c->name, sizeof(c->name), "%s", name);
  c->state         = PCL_STATE_UNCONFIGURED;
  c->tick_rate_hz  = 100.0;
  c->configuring   = false;
  c->port_count    = 0;
  c->param_count   = 0;
  c->user_data     = user_data;

  if (callbacks) {
    c->callbacks = *callbacks;
  } else {
    memset(&c->callbacks, 0, sizeof(c->callbacks));
  }

  return c;
}

/* Implements: REQ_PCL_003, REQ_PCL_111. */
void pcl_container_destroy(pcl_container_t* c) {
  if (!c) return;
  // Detach from the executor (if attached) before freeing the storage --
  // otherwise the executor's container list keeps a dangling pointer that
  // transport recv threads and pcl_executor_destroy will later dereference.
  if (c->executor) {
    pcl_executor_remove(c->executor, c);
  }
  pcl_free(c);
}

// -- Lifecycle transitions -----------------------------------------------

/* Implements: REQ_PCL_004, REQ_PCL_005, REQ_PCL_007, REQ_PCL_008. */
pcl_status_t pcl_container_configure(pcl_container_t* c) {
  pcl_status_t rc = PCL_OK;
  if (!c) return PCL_ERR_INVALID;
  if (c->state != PCL_STATE_UNCONFIGURED) return PCL_ERR_STATE;

  pcl_log(c, PCL_LOG_INFO, "configuring");

  c->configuring = true;
  if (c->callbacks.on_configure) {
    rc = c->callbacks.on_configure(c, c->user_data);
  }
  c->configuring = false;

  if (rc == PCL_OK) {
    c->state = PCL_STATE_CONFIGURED;
    pcl_log(c, PCL_LOG_INFO, "configured (%u ports, %u params)",
            c->port_count, c->param_count);
  } else {
    pcl_log(c, PCL_LOG_ERROR, "configure failed (rc=%d)", (int)rc);
  }
  return rc;
}

/* Implements: REQ_PCL_004, REQ_PCL_005, REQ_PCL_008, REQ_PCL_011. */
pcl_status_t pcl_container_activate(pcl_container_t* c) {
  pcl_status_t rc = PCL_OK;
  if (!c) return PCL_ERR_INVALID;
  if (c->state != PCL_STATE_CONFIGURED) return PCL_ERR_STATE;

  pcl_log(c, PCL_LOG_INFO, "activating");

  if (c->callbacks.on_activate) {
    rc = c->callbacks.on_activate(c, c->user_data);
  }

  if (rc == PCL_OK) {
    c->state = PCL_STATE_ACTIVE;
    c->tick_accumulator = 0.0;
    pcl_log(c, PCL_LOG_INFO, "active (%.1f Hz)", c->tick_rate_hz);
  } else {
    pcl_log(c, PCL_LOG_ERROR, "activate failed (rc=%d)", (int)rc);
  }
  return rc;
}

/* Implements: REQ_PCL_004, REQ_PCL_005, REQ_PCL_008, REQ_PCL_009. */
pcl_status_t pcl_container_deactivate(pcl_container_t* c) {
  pcl_status_t rc = PCL_OK;
  if (!c) return PCL_ERR_INVALID;
  if (c->state != PCL_STATE_ACTIVE) return PCL_ERR_STATE;

  pcl_log(c, PCL_LOG_INFO, "deactivating");

  if (c->callbacks.on_deactivate) {
    rc = c->callbacks.on_deactivate(c, c->user_data);
  }

  if (rc == PCL_OK) {
    c->state = PCL_STATE_CONFIGURED;
    pcl_log(c, PCL_LOG_INFO, "deactivated");
  } else {
    pcl_log(c, PCL_LOG_ERROR, "deactivate failed (rc=%d)", (int)rc);
  }
  return rc;
}

/* Implements: REQ_PCL_004, REQ_PCL_005, REQ_PCL_008, REQ_PCL_010,
   REQ_PCL_012. */
pcl_status_t pcl_container_cleanup(pcl_container_t* c) {
  pcl_status_t rc = PCL_OK;
  if (!c) return PCL_ERR_INVALID;
  if (c->state != PCL_STATE_CONFIGURED) return PCL_ERR_STATE;

  pcl_log(c, PCL_LOG_INFO, "cleaning up");

  if (c->callbacks.on_cleanup) {
    rc = c->callbacks.on_cleanup(c, c->user_data);
  }

  if (rc == PCL_OK) {
    c->state = PCL_STATE_UNCONFIGURED;
    c->port_count = 0;
    pcl_log(c, PCL_LOG_INFO, "cleaned up");
  } else {
    pcl_log(c, PCL_LOG_ERROR, "cleanup failed (rc=%d)", (int)rc);
  }
  return rc;
}

/* Implements: REQ_PCL_004, REQ_PCL_006. */
pcl_status_t pcl_container_shutdown(pcl_container_t* c) {
  if (!c) return PCL_ERR_INVALID;
  if (c->state == PCL_STATE_FINALIZED) return PCL_ERR_STATE;

  pcl_log(c, PCL_LOG_INFO, "shutting down");

  if (c->callbacks.on_shutdown) {
    c->callbacks.on_shutdown(c, c->user_data);
  }

  c->state = PCL_STATE_FINALIZED;
  pcl_log(c, PCL_LOG_INFO, "finalized");
  return PCL_OK;
}

// -- State query ---------------------------------------------------------

/* Implements: REQ_PCL_111. */
pcl_state_t pcl_container_state(const pcl_container_t* c) {
  return c ? c->state : PCL_STATE_FINALIZED;
}

/* Implements: REQ_PCL_111. */
const char* pcl_container_name(const pcl_container_t* c) {
  return c ? c->name : "";
}

// -- Tick rate -----------------------------------------------------------

/* Implements: REQ_PCL_029, REQ_PCL_030. */
pcl_status_t pcl_container_set_tick_rate_hz(pcl_container_t* c, double hz) {
  if (!c) return PCL_ERR_INVALID;
  if (hz <= 0.0) return PCL_ERR_INVALID;
  c->tick_rate_hz = hz;
  return PCL_OK;
}

/* Implements: REQ_PCL_029, REQ_PCL_111. */
double pcl_container_get_tick_rate_hz(const pcl_container_t* c) {
  return c ? c->tick_rate_hz : 0.0;
}

// -- Parameters ----------------------------------------------------------

/* Implements: REQ_PCL_013, REQ_PCL_015, REQ_PCL_018. */
pcl_status_t pcl_container_set_param_str(pcl_container_t* c,
                                         const char* key, const char* value) {
  pcl_param_t* p;
  if (!c || !key || !value) return PCL_ERR_INVALID;
  p = find_or_add_param(c, key);
  if (!p) return PCL_ERR_NOMEM;
  p->kind = PCL_PARAM_STR;
  snprintf(p->u.str_val, sizeof(p->u.str_val), "%s", value);
  return PCL_OK;
}

/* Implements: REQ_PCL_014, REQ_PCL_015, REQ_PCL_018. */
pcl_status_t pcl_container_set_param_f64(pcl_container_t* c,
                                         const char* key, double value) {
  pcl_param_t* p;
  if (!c || !key) return PCL_ERR_INVALID;
  p = find_or_add_param(c, key);
  if (!p) return PCL_ERR_NOMEM;
  p->kind = PCL_PARAM_F64;
  p->u.f64_val = value;
  return PCL_OK;
}

/* Implements: REQ_PCL_014, REQ_PCL_015, REQ_PCL_018. */
pcl_status_t pcl_container_set_param_i64(pcl_container_t* c,
                                         const char* key, int64_t value) {
  pcl_param_t* p;
  if (!c || !key) return PCL_ERR_INVALID;
  p = find_or_add_param(c, key);
  if (!p) return PCL_ERR_NOMEM;
  p->kind = PCL_PARAM_I64;
  p->u.i64_val = value;
  return PCL_OK;
}

/* Implements: REQ_PCL_014, REQ_PCL_015, REQ_PCL_018. */
pcl_status_t pcl_container_set_param_bool(pcl_container_t* c,
                                          const char* key, bool value) {
  pcl_param_t* p;
  if (!c || !key) return PCL_ERR_INVALID;
  p = find_or_add_param(c, key);
  if (!p) return PCL_ERR_NOMEM;
  p->kind = PCL_PARAM_BOOL;
  p->u.bool_val = value;
  return PCL_OK;
}

/* Implements: REQ_PCL_013, REQ_PCL_017, REQ_PCL_018. */
const char* pcl_container_get_param_str(const pcl_container_t* c,
                                        const char* key,
                                        const char* default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_STR) return default_val;
  return p->u.str_val;
}

/* Implements: REQ_PCL_014, REQ_PCL_017, REQ_PCL_018. */
double pcl_container_get_param_f64(const pcl_container_t* c,
                                   const char* key, double default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_F64) return default_val;
  return p->u.f64_val;
}

/* Implements: REQ_PCL_014, REQ_PCL_017, REQ_PCL_018. */
int64_t pcl_container_get_param_i64(const pcl_container_t* c,
                                    const char* key, int64_t default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_I64) return default_val;
  return p->u.i64_val;
}

/* Implements: REQ_PCL_014, REQ_PCL_017, REQ_PCL_018. */
bool pcl_container_get_param_bool(const pcl_container_t* c,
                                  const char* key, bool default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_BOOL) return default_val;
  return p->u.bool_val;
}

// -- Port creation -------------------------------------------------------

/* Implements: REQ_PCL_019, REQ_PCL_020, REQ_PCL_021, REQ_PCL_022. */
pcl_port_t* pcl_container_add_publisher(pcl_container_t* c,
                                        const char*      topic,
                                        const char*      type_name) {
  pcl_port_t* p;
  if (!validate_port_definition(c, topic, type_name, "publisher")) return NULL;

  p = &c->ports[c->port_count++];
  memset(p, 0, sizeof(*p));
  p->type  = PCL_PORT_PUBLISHER;
  p->owner = c;
  p->route_mode = PCL_ROUTE_NONE;
  snprintf(p->name, sizeof(p->name), "%s", topic);
  snprintf(p->type_name, sizeof(p->type_name), "%s", type_name);
  return p;
}

/* Implements: REQ_PCL_019, REQ_PCL_020, REQ_PCL_021, REQ_PCL_022. */
pcl_port_t* pcl_container_add_subscriber(pcl_container_t* c,
                                         const char*         topic,
                                         const char*         type_name,
                                         pcl_sub_callback_t  cb,
                                         void*               user_data) {
  pcl_port_t* p;
  if (!validate_port_definition(c, topic, type_name, "subscriber")) return NULL;
  if (!cb) {
    pcl_log(c, PCL_LOG_ERROR,
            "cannot add subscriber port '%s': callback is null", topic);
    return NULL;
  }

  p = &c->ports[c->port_count++];
  memset(p, 0, sizeof(*p));
  p->type          = PCL_PORT_SUBSCRIBER;
  p->owner         = c;
  p->sub_cb        = cb;
  p->sub_user_data = user_data;
  p->route_mode    = PCL_ROUTE_NONE;
  snprintf(p->name, sizeof(p->name), "%s", topic);
  snprintf(p->type_name, sizeof(p->type_name), "%s", type_name);
  return p;
}

/* Implements: REQ_PCL_019, REQ_PCL_020, REQ_PCL_021, REQ_PCL_022. */
pcl_port_t* pcl_container_add_service(pcl_container_t*      c,
                                      const char*           service_name,
                                      const char*           type_name,
                                      pcl_service_handler_t handler,
                                      void*                 user_data) {
  pcl_port_t* p;
  if (!validate_port_definition(c, service_name, type_name, "service")) return NULL;
  if (!handler) {
    pcl_log(c, PCL_LOG_ERROR,
            "cannot add service port '%s': handler is null", service_name);
    return NULL;
  }

  p = &c->ports[c->port_count++];
  memset(p, 0, sizeof(*p));
  p->type          = PCL_PORT_SERVICE;
  p->owner         = c;
  p->svc_handler   = handler;
  p->svc_user_data = user_data;
  p->route_mode    = PCL_ROUTE_NONE;
  snprintf(p->name, sizeof(p->name), "%s", service_name);
  snprintf(p->type_name, sizeof(p->type_name), "%s", type_name);
  return p;
}

// -- Publishing ----------------------------------------------------------

/* Implements: REQ_PCL_023, REQ_PCL_024, REQ_PCL_025, REQ_PCL_026,
   REQ_PCL_027, REQ_PCL_174. */
pcl_status_t pcl_port_publish(pcl_port_t* port, const pcl_msg_t* msg) {
  if (!port || !msg) return PCL_ERR_INVALID;
  if (port->type != PCL_PORT_PUBLISHER) return PCL_ERR_INVALID;
  if (!port->owner || port->owner->state != PCL_STATE_ACTIVE) {
    return PCL_ERR_PORT_CLOSED;
  }
  if (port->owner->executor) {
    return pcl_executor_publish_port(port->owner->executor, port, msg);
  }
  return PCL_OK;
}

/* No LLR: thin convenience wrapper with no dedicated requirement of its
   own; it forwards to pcl_executor_invoke_async(), whose behaviour is
   covered by REQ_PCL_164, REQ_PCL_165, and REQ_PCL_166. */
pcl_status_t pcl_container_invoke_async(pcl_container_t* c,
                                        const char*      service_name,
                                        const pcl_msg_t* request,
                                        pcl_resp_cb_fn_t callback,
                                        void*            user_data) {
  if (!c || !service_name || !request || !callback) return PCL_ERR_INVALID;
  if (!c->executor) return PCL_ERR_STATE;
  return pcl_executor_invoke_async(c->executor, service_name, request,
                                   callback, user_data);
}

/* Implements: REQ_PCL_459, REQ_PCL_460, REQ_PCL_461. */
pcl_status_t pcl_service_respond(pcl_svc_context_t* ctx,
                                 const pcl_msg_t*   response) {
  if (!ctx || !response) return PCL_ERR_INVALID;

  // If transport has respond function, use it (for remote callers)
  if (ctx->transport && ctx->transport->respond) {
    pcl_status_t rc = ctx->transport->respond(
        ctx->transport->adapter_ctx, ctx, response);
    pcl_free(ctx);
    return rc;
  }

  if (ctx->executor && ctx->executor->has_transport &&
      ctx->executor->transport.respond) {
    pcl_status_t rc = ctx->executor->transport.respond(
        ctx->executor->transport.adapter_ctx, ctx, response);
    pcl_free(ctx);
    return rc;
  }

  // Intra-process: fire the callback directly
  if (ctx->callback) {
    ctx->callback(response, ctx->user_data);
  }
  pcl_free(ctx);
  return PCL_OK;
}

/* Implements: REQ_PCL_223. */
void pcl_service_context_free(pcl_svc_context_t* ctx) {
  pcl_free(ctx);
}

// -- Streaming service API ------------------------------------------------

/* Implements: REQ_PCL_172. */
pcl_port_t* pcl_container_add_stream_service(pcl_container_t*     c,
                                             const char*          service_name,
                                             const char*          type_name,
                                             pcl_stream_handler_t handler,
                                             void*                user_data) {
  pcl_port_t* p;
  if (!validate_port_definition(c, service_name, type_name,
                                "stream service")) return NULL;
  if (!handler) {
    pcl_log(c, PCL_LOG_ERROR,
            "cannot add stream service port '%s': handler is null",
            service_name);
    return NULL;
  }

  p = &c->ports[c->port_count++];
  memset(p, 0, sizeof(*p));
  p->type             = PCL_PORT_STREAM_SERVICE;
  p->owner            = c;
  p->stream_handler   = handler;
  p->stream_user_data = user_data;
  p->route_mode       = PCL_ROUTE_NONE;
  snprintf(p->name, sizeof(p->name), "%s", service_name);
  snprintf(p->type_name, sizeof(p->type_name), "%s", type_name);
  return p;
}

/* Implements: REQ_PCL_235. */
pcl_status_t pcl_port_set_route(pcl_port_t*              port,
                                uint32_t                 route_mode,
                                const char* const*       peer_ids,
                                uint32_t                 peer_count) {
  return copy_route_config(port, route_mode, peer_ids, peer_count);
}

/* Implements: REQ_PCL_167, REQ_PCL_098, REQ_PCL_171. */
pcl_status_t pcl_stream_send(pcl_stream_context_t* ctx, const pcl_msg_t* msg) {
  if (!ctx || !msg) return PCL_ERR_INVALID;
  if (ctx->ended) return PCL_ERR_STATE;
  if (ctx->cancelled) return PCL_ERR_CANCELLED;

  // If transport has stream_send, use it
  if (ctx->transport && ctx->transport->stream_send) {
    return ctx->transport->stream_send(
        ctx->transport->adapter_ctx, ctx->transport_ctx, msg);
  }

  if (ctx->executor && ctx->executor->has_transport &&
      ctx->executor->transport.stream_send) {
    return ctx->executor->transport.stream_send(
        ctx->executor->transport.adapter_ctx, ctx->transport_ctx, msg);
  }

  // Intra-process: fire callback directly
  if (ctx->callback) {
    ctx->callback(msg, false, PCL_OK, ctx->user_data);
  }
  return PCL_OK;
}

/* Implements: REQ_PCL_097, REQ_PCL_171. */
pcl_status_t pcl_stream_end(pcl_stream_context_t* ctx) {
  if (!ctx) return PCL_ERR_INVALID;
  if (ctx->ended) return PCL_ERR_STATE;

  ctx->ended = 1;

  // If transport has stream_end, use it
  if (ctx->transport && ctx->transport->stream_end) {
    pcl_status_t rc = ctx->transport->stream_end(
        ctx->transport->adapter_ctx, ctx->transport_ctx, PCL_OK);
    pcl_free(ctx);
    return rc;
  }

  if (ctx->executor && ctx->executor->has_transport &&
      ctx->executor->transport.stream_end) {
    pcl_status_t rc = ctx->executor->transport.stream_end(
        ctx->executor->transport.adapter_ctx, ctx->transport_ctx, PCL_OK);
    pcl_free(ctx);
    return rc;
  }

  // Intra-process: fire callback with end=true
  if (ctx->callback) {
    pcl_msg_t empty = {0};
    ctx->callback(&empty, true, PCL_OK, ctx->user_data);
  }
  pcl_free(ctx);
  return PCL_OK;
}

/* Implements: REQ_PCL_169, REQ_PCL_171. */
pcl_status_t pcl_stream_abort(pcl_stream_context_t* ctx, pcl_status_t error_code) {
  if (!ctx) return PCL_ERR_INVALID;
  if (ctx->ended) return PCL_ERR_STATE;

  ctx->ended = 1;

  // If transport has stream_end, use it with error
  if (ctx->transport && ctx->transport->stream_end) {
    pcl_status_t rc = ctx->transport->stream_end(
        ctx->transport->adapter_ctx, ctx->transport_ctx, error_code);
    pcl_free(ctx);
    return rc;
  }

  if (ctx->executor && ctx->executor->has_transport &&
      ctx->executor->transport.stream_end) {
    pcl_status_t rc = ctx->executor->transport.stream_end(
        ctx->executor->transport.adapter_ctx, ctx->transport_ctx, error_code);
    pcl_free(ctx);
    return rc;
  }

  // Intra-process: fire callback with error status
  if (ctx->callback) {
    pcl_msg_t empty = {0};
    ctx->callback(&empty, true, error_code, ctx->user_data);
  }
  pcl_free(ctx);
  return PCL_OK;
}

/* Implements: REQ_PCL_168, REQ_PCL_171. */
bool pcl_stream_is_cancelled(const pcl_stream_context_t* ctx) {
  if (!ctx) return false;
  return ctx->cancelled != 0;
}

/* Implements: REQ_PCL_168, REQ_PCL_171. */
pcl_status_t pcl_stream_cancel(pcl_stream_context_t* ctx) {
  if (!ctx) return PCL_ERR_INVALID;
  if (ctx->ended) return PCL_ERR_STATE;

  ctx->cancelled = 1;

  // If transport has stream_cancel, use it
  if (ctx->transport && ctx->transport->stream_cancel) {
    return ctx->transport->stream_cancel(
        ctx->transport->adapter_ctx, ctx->transport_ctx);
  }

  if (ctx->executor && ctx->executor->has_transport &&
      ctx->executor->transport.stream_cancel) {
    return ctx->executor->transport.stream_cancel(
        ctx->executor->transport.adapter_ctx, ctx->transport_ctx);
  }

  return PCL_OK;
}
