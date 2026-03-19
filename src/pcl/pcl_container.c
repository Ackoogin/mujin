/// \file pcl_container.c
/// \brief PCL container implementation — lifecycle, parameters, and ports.
#include "pcl_internal.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"

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

static pcl_param_t* find_or_add_param(pcl_container_t* c, const char* key) {
  pcl_param_t* p = find_param(c, key);
  if (p) return p;
  if (c->param_count >= PCL_MAX_PARAMS) return NULL;
  p = &c->params[c->param_count++];
  snprintf(p->key, sizeof(p->key), "%s", key);
  return p;
}

// -- Create / destroy ----------------------------------------------------

pcl_container_t* pcl_container_create(const char*            name,
                                      const pcl_callbacks_t* callbacks,
                                      void*                  user_data) {
  if (!name) return NULL;

  pcl_container_t* c = (pcl_container_t*)calloc(1, sizeof(pcl_container_t));
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

void pcl_container_destroy(pcl_container_t* c) {
  if (!c) return;
  free(c);
}

// -- Lifecycle transitions -----------------------------------------------

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

pcl_state_t pcl_container_state(const pcl_container_t* c) {
  return c ? c->state : PCL_STATE_FINALIZED;
}

const char* pcl_container_name(const pcl_container_t* c) {
  return c ? c->name : "";
}

// -- Tick rate -----------------------------------------------------------

pcl_status_t pcl_container_set_tick_rate_hz(pcl_container_t* c, double hz) {
  if (!c) return PCL_ERR_INVALID;
  if (hz <= 0.0) return PCL_ERR_INVALID;
  c->tick_rate_hz = hz;
  return PCL_OK;
}

double pcl_container_get_tick_rate_hz(const pcl_container_t* c) {
  return c ? c->tick_rate_hz : 0.0;
}

// -- Parameters ----------------------------------------------------------

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

const char* pcl_container_get_param_str(const pcl_container_t* c,
                                        const char* key,
                                        const char* default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_STR) return default_val;
  return p->u.str_val;
}

double pcl_container_get_param_f64(const pcl_container_t* c,
                                   const char* key, double default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_F64) return default_val;
  return p->u.f64_val;
}

int64_t pcl_container_get_param_i64(const pcl_container_t* c,
                                    const char* key, int64_t default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_I64) return default_val;
  return p->u.i64_val;
}

bool pcl_container_get_param_bool(const pcl_container_t* c,
                                  const char* key, bool default_val) {
  const pcl_param_t* p;
  if (!c || !key) return default_val;
  p = find_param(c, key);
  if (!p || p->kind != PCL_PARAM_BOOL) return default_val;
  return p->u.bool_val;
}

// -- Port creation -------------------------------------------------------

pcl_port_t* pcl_container_add_publisher(pcl_container_t* c,
                                        const char*      topic,
                                        const char*      type_name) {
  pcl_port_t* p;
  if (!c || !topic || !type_name) return NULL;
  if (!c->configuring) return NULL;
  if (c->port_count >= PCL_MAX_PORTS) return NULL;

  p = &c->ports[c->port_count++];
  memset(p, 0, sizeof(*p));
  p->type  = PCL_PORT_PUBLISHER;
  p->owner = c;
  snprintf(p->name, sizeof(p->name), "%s", topic);
  snprintf(p->type_name, sizeof(p->type_name), "%s", type_name);
  return p;
}

pcl_port_t* pcl_container_add_subscriber(pcl_container_t* c,
                                         const char*         topic,
                                         const char*         type_name,
                                         pcl_sub_callback_t  cb,
                                         void*               user_data) {
  pcl_port_t* p;
  if (!c || !topic || !type_name || !cb) return NULL;
  if (!c->configuring) return NULL;
  if (c->port_count >= PCL_MAX_PORTS) return NULL;

  p = &c->ports[c->port_count++];
  memset(p, 0, sizeof(*p));
  p->type          = PCL_PORT_SUBSCRIBER;
  p->owner         = c;
  p->sub_cb        = cb;
  p->sub_user_data = user_data;
  snprintf(p->name, sizeof(p->name), "%s", topic);
  snprintf(p->type_name, sizeof(p->type_name), "%s", type_name);
  return p;
}

pcl_port_t* pcl_container_add_service(pcl_container_t*      c,
                                      const char*           service_name,
                                      const char*           type_name,
                                      pcl_service_handler_t handler,
                                      void*                 user_data) {
  pcl_port_t* p;
  if (!c || !service_name || !type_name || !handler) return NULL;
  if (!c->configuring) return NULL;
  if (c->port_count >= PCL_MAX_PORTS) return NULL;

  p = &c->ports[c->port_count++];
  memset(p, 0, sizeof(*p));
  p->type          = PCL_PORT_SERVICE;
  p->owner         = c;
  p->svc_handler   = handler;
  p->svc_user_data = user_data;
  snprintf(p->name, sizeof(p->name), "%s", service_name);
  snprintf(p->type_name, sizeof(p->type_name), "%s", type_name);
  return p;
}

// -- Publishing ----------------------------------------------------------

pcl_status_t pcl_port_publish(pcl_port_t* port, const pcl_msg_t* msg) {
  if (!port || !msg) return PCL_ERR_INVALID;
  if (port->type != PCL_PORT_PUBLISHER) return PCL_ERR_INVALID;
  if (!port->owner || port->owner->state != PCL_STATE_ACTIVE) {
    return PCL_ERR_PORT_CLOSED;
  }
  if (port->owner->executor) {
    return pcl_executor_publish(port->owner->executor, port->name, msg);
  }
  return PCL_OK;
}
