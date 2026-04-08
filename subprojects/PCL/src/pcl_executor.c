/// \file pcl_executor.c
/// \brief PCL executor implementation — tick loop, dispatch, lifecycle.
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#  define _POSIX_C_SOURCE 199309L
#endif
#include "pcl_internal.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <time.h>
#  include <errno.h>
#  include <pthread.h>
#endif

// -- Platform time helpers -----------------------------------------------

static double pcl_clock_now(void) {
#ifdef _WIN32
  static LARGE_INTEGER freq;
  static int freq_init = 0;
  LARGE_INTEGER now;
  if (!freq_init) {
    QueryPerformanceFrequency(&freq);
    freq_init = 1;
  }
  QueryPerformanceCounter(&now);
  return (double)now.QuadPart / (double)freq.QuadPart;
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

static void pcl_sleep_seconds(double seconds) {
  if (seconds <= 0.0) return;
#ifdef _WIN32
  Sleep((DWORD)(seconds * 1000.0));
#else
  struct timespec req;
  req.tv_sec  = (time_t)seconds;
  req.tv_nsec = (long)((seconds - (double)req.tv_sec) * 1e9);
  while (nanosleep(&req, &req) == -1 && errno == EINTR) { }
#endif
}

// -- Mutex helpers -------------------------------------------------------

#ifdef _WIN32
static void pcl_mutex_init(pcl_mutex_t* mutex) {
  InitializeCriticalSection(mutex);
}

static void pcl_mutex_destroy(pcl_mutex_t* mutex) {
  DeleteCriticalSection(mutex);
}

static void pcl_mutex_lock(pcl_mutex_t* mutex) {
  EnterCriticalSection(mutex);
}

static void pcl_mutex_unlock(pcl_mutex_t* mutex) {
  LeaveCriticalSection(mutex);
}
#else
static void pcl_mutex_init(pcl_mutex_t* mutex) {
  pthread_mutex_init(mutex, NULL);
}

static void pcl_mutex_destroy(pcl_mutex_t* mutex) {
  pthread_mutex_destroy(mutex);
}

static void pcl_mutex_lock(pcl_mutex_t* mutex) {
  pthread_mutex_lock(mutex);
}

static void pcl_mutex_unlock(pcl_mutex_t* mutex) {
  pthread_mutex_unlock(mutex);
}
#endif

static char* pcl_strdup_local(const char* src) {
  size_t len;
  char* copy;

  if (!src) return NULL;

  len = strlen(src) + 1u;
  copy = (char*)malloc(len);
  if (!copy) return NULL;

  memcpy(copy, src, len);
  return copy;
}

static void free_pending_msg(pcl_pending_msg_t* pending) {
  if (!pending) return;
  free(pending->topic);
  free(pending->type_name);
  free(pending->source_peer_id);
  free(pending->data);
  free(pending);
}

static pcl_endpoint_kind_t endpoint_kind_from_port_type(pcl_port_type_t type) {
  switch (type) {
    case PCL_PORT_PUBLISHER:      return PCL_ENDPOINT_PUBLISHER;
    case PCL_PORT_SUBSCRIBER:     return PCL_ENDPOINT_SUBSCRIBER;
    case PCL_PORT_SERVICE:        return PCL_ENDPOINT_PROVIDED;
    case PCL_PORT_CLIENT:         return PCL_ENDPOINT_CONSUMED;
    case PCL_PORT_STREAM_SERVICE: return PCL_ENDPOINT_STREAM_PROVIDED;
    default:                      return PCL_ENDPOINT_PUBLISHER;
  }
}

static const pcl_transport_t* find_named_transport(const pcl_executor_t* e,
                                                   const char*           peer_id) {
  uint32_t i;

  if (!e || !peer_id) return NULL;
  for (i = 0; i < e->transport_count; ++i) {
    if (e->transports[i].in_use &&
        strcmp(e->transports[i].peer_id, peer_id) == 0) {
      return &e->transports[i].transport;
    }
  }
  return NULL;
}

static pcl_endpoint_route_entry_t* find_endpoint_route_entry(pcl_executor_t*        e,
                                                             const char*            endpoint_name,
                                                             pcl_endpoint_kind_t    endpoint_kind) {
  uint32_t i;

  if (!e || !endpoint_name) return NULL;
  for (i = 0; i < e->endpoint_route_count; ++i) {
    pcl_endpoint_route_entry_t* entry = &e->endpoint_routes[i];
    if (entry->in_use &&
        entry->endpoint_kind == endpoint_kind &&
        strcmp(entry->endpoint_name, endpoint_name) == 0) {
      return entry;
    }
  }
  return NULL;
}

static const pcl_endpoint_route_entry_t* find_endpoint_route_entry_const(
    const pcl_executor_t*     e,
    const char*               endpoint_name,
    pcl_endpoint_kind_t       endpoint_kind) {
  return find_endpoint_route_entry((pcl_executor_t*)e, endpoint_name, endpoint_kind);
}

static uint32_t port_route_mode(const pcl_executor_t* e, const pcl_port_t* port) {
  const pcl_endpoint_route_entry_t* entry;

  if (!port) return PCL_ROUTE_NONE;
  entry = find_endpoint_route_entry_const(e, port->name,
                                          endpoint_kind_from_port_type(port->type));
  if (entry) return entry->route_mode;
  if (port->route_configured && port->route_mode != PCL_ROUTE_NONE) {
    return port->route_mode;
  }

  if (e && e->has_transport) {
    if (port->type == PCL_PORT_PUBLISHER || port->type == PCL_PORT_CLIENT) {
      return PCL_ROUTE_REMOTE;
    }
    return PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE;
  }

  return PCL_ROUTE_LOCAL;
}

static uint32_t port_peer_count(const pcl_executor_t* e, const pcl_port_t* port) {
  const pcl_endpoint_route_entry_t* entry;
  if (!port) return 0u;
  entry = find_endpoint_route_entry_const(e, port->name,
                                          endpoint_kind_from_port_type(port->type));
  return entry ? entry->peer_count : port->peer_count;
}

static const char* port_peer_id_at(const pcl_executor_t* e,
                                   const pcl_port_t*     port,
                                   uint32_t              index) {
  const pcl_endpoint_route_entry_t* entry;

  if (!port) return NULL;
  entry = find_endpoint_route_entry_const(e, port->name,
                                          endpoint_kind_from_port_type(port->type));
  if (entry) {
    return (index < entry->peer_count) ? entry->peer_ids[index] : NULL;
  }
  return (index < port->peer_count) ? port->peer_ids[index] : NULL;
}

static int route_accepts(uint32_t route_mode,
                         uint32_t source_route_mode) {
  return (route_mode & source_route_mode) != 0u;
}

static int peer_is_allowed(const pcl_executor_t* e,
                           const pcl_port_t*     port,
                           const char*           peer_id) {
  uint32_t i;
  uint32_t count;

  if (!peer_id) return 0;
  count = port_peer_count(e, port);
  if (count == 0u) return 1;
  for (i = 0; i < count; ++i) {
    const char* configured = port_peer_id_at(e, port, i);
    if (configured && strcmp(configured, peer_id) == 0) {
      return 1;
    }
  }
  return 0;
}

// -- Create / destroy ----------------------------------------------------

pcl_executor_t* pcl_executor_create(void) {
  pcl_executor_t* e = (pcl_executor_t*)calloc(1, sizeof(pcl_executor_t));
  if (!e) return NULL;
  e->container_count    = 0;
  e->has_transport      = 0;
  e->shutdown_requested = 0;
  e->prev_time          = 0.0;
  pcl_mutex_init(&e->incoming_lock);
  pcl_mutex_init(&e->resp_cb_lock);
  return e;
}

void pcl_executor_destroy(pcl_executor_t* e) {
  pcl_pending_msg_t* pending;
  uint32_t i;
  if (!e) return;
  for (i = 0; i < e->container_count; ++i) {
    e->containers[i]->executor = NULL;
  }
  if (e->has_transport && e->transport.shutdown) {
    e->transport.shutdown(e->transport.adapter_ctx);
  }
  for (i = 0; i < e->transport_count; ++i) {
    if (e->transports[i].in_use &&
        e->transports[i].transport.shutdown) {
      e->transports[i].transport.shutdown(
          e->transports[i].transport.adapter_ctx);
    }
  }

  pcl_mutex_lock(&e->incoming_lock);
  pending = e->incoming_head;
  e->incoming_head = NULL;
  e->incoming_tail = NULL;
  pcl_mutex_unlock(&e->incoming_lock);

  while (pending) {
    pcl_pending_msg_t* next = pending->next;
    free_pending_msg(pending);
    pending = next;
  }

  {
    pcl_resp_cb_node_t* node;
    pcl_mutex_lock(&e->resp_cb_lock);
    node = e->resp_cb_head;
    e->resp_cb_head = NULL;
    e->resp_cb_tail = NULL;
    pcl_mutex_unlock(&e->resp_cb_lock);
    while (node) {
      pcl_resp_cb_node_t* next = node->next;
      free(node->data);
      free(node);
      node = next;
    }
  }
  pcl_mutex_destroy(&e->resp_cb_lock);
  pcl_mutex_destroy(&e->incoming_lock);
  free(e);
}

// -- Container management ------------------------------------------------

pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c) {
  if (!e || !c) return PCL_ERR_INVALID;
  if (e->container_count >= PCL_MAX_CONTAINERS) return PCL_ERR_NOMEM;
  e->containers[e->container_count++] = c;
  c->executor = e;
  return PCL_OK;
}

pcl_status_t pcl_executor_remove(pcl_executor_t* e, pcl_container_t* c) {
  uint32_t i;
  if (!e || !c) return PCL_ERR_INVALID;
  for (i = 0; i < e->container_count; ++i) {
    if (e->containers[i] == c) {
      c->executor = NULL;
      memmove(&e->containers[i], &e->containers[i + 1],
              (e->container_count - i - 1) * sizeof(e->containers[0]));
      e->container_count--;
      return PCL_OK;
    }
  }
  return PCL_ERR_NOT_FOUND;
}

// -- Intra-process dispatch ----------------------------------------------

static pcl_status_t dispatch_incoming_now(pcl_executor_t*  e,
                                          const char*      topic,
                                          const pcl_msg_t* msg,
                                          uint32_t         source_route_mode,
                                          const char*      source_peer_id) {
  uint32_t ci, pi;
  int delivered = 0;

  if (!e || !topic || !msg) return PCL_ERR_INVALID;

  for (ci = 0; ci < e->container_count; ++ci) {
    pcl_container_t* container = e->containers[ci];
    for (pi = 0; pi < container->port_count; ++pi) {
      struct pcl_port_t* port = &container->ports[pi];
      uint32_t route_mode;

      if (port->type != PCL_PORT_SUBSCRIBER ||
          strcmp(port->name, topic) != 0 ||
          port->sub_cb == NULL) {
        continue;
      }

      route_mode = port_route_mode(e, port);
      if (!route_accepts(route_mode, source_route_mode)) {
        continue;
      }
      if (source_route_mode == PCL_ROUTE_REMOTE &&
          !peer_is_allowed(e, port, source_peer_id)) {
        continue;
      }

      port->sub_cb(port->owner, msg, port->sub_user_data);
      delivered = 1;
    }
  }

  return delivered ? PCL_OK : PCL_ERR_NOT_FOUND;
}

static void drain_resp_cb_queue(pcl_executor_t* e) {
  for (;;) {
    pcl_resp_cb_node_t* node;
    pcl_msg_t           resp;

    pcl_mutex_lock(&e->resp_cb_lock);
    node = e->resp_cb_head;
    if (node) {
      e->resp_cb_head = node->next;
      if (!e->resp_cb_head) e->resp_cb_tail = NULL;
    }
    pcl_mutex_unlock(&e->resp_cb_lock);

    if (!node) break;

    memset(&resp, 0, sizeof(resp));
    resp.data      = node->data;
    resp.size      = node->size;
    resp.type_name = NULL;

    node->cb(&resp, node->user_data);

    free(node->data);
    free(node);
  }
}

static pcl_status_t drain_incoming_queue(pcl_executor_t* e) {
  pcl_pending_msg_t* pending;
  pcl_status_t       rc = PCL_OK;

  if (!e) return PCL_ERR_INVALID;

  for (;;) {
    pcl_msg_t view;

    pcl_mutex_lock(&e->incoming_lock);
    pending = e->incoming_head;
    if (pending) {
      e->incoming_head = pending->next;
      if (!e->incoming_head) {
        e->incoming_tail = NULL;
      }
    }
    pcl_mutex_unlock(&e->incoming_lock);

    if (!pending) break;

    memset(&view, 0, sizeof(view));
    view.data = pending->data;
    view.size = pending->size;
    view.type_name = pending->type_name;

    rc = dispatch_incoming_now(e,
                               pending->topic,
                               &view,
                               pending->source_route_mode ? pending->source_route_mode
                                                          : PCL_ROUTE_LOCAL,
                               pending->source_peer_id);
    free_pending_msg(pending);

    if (rc != PCL_OK) {
      return rc;
    }
  }

  return PCL_OK;
}

// -- Tick one container --------------------------------------------------

static void tick_container(pcl_container_t* c, double dt) {
  double tick_period;
  double actual_dt;

  if (c->state != PCL_STATE_ACTIVE) return;

  tick_period = 1.0 / c->tick_rate_hz;
  c->tick_accumulator += dt;

  if (c->tick_accumulator >= tick_period) {
    actual_dt = c->tick_accumulator;
    c->tick_accumulator = 0.0;

    if (c->callbacks.on_tick) {
      c->callbacks.on_tick(c, actual_dt, c->user_data);
    }
  }
}

// -- Spin ----------------------------------------------------------------

pcl_status_t pcl_executor_spin(pcl_executor_t* e) {
  double fastest_hz;
  double base_period;
  double t_prev;
  pcl_status_t rc;
  uint32_t i;

  if (!e) return PCL_ERR_INVALID;

  pcl_log(NULL, PCL_LOG_INFO, "executor spinning (%u containers)",
          e->container_count);

  fastest_hz = 100.0;
  for (i = 0; i < e->container_count; ++i) {
    double hz = pcl_container_get_tick_rate_hz(e->containers[i]);
    if (hz > fastest_hz) fastest_hz = hz;
  }
  base_period = 1.0 / fastest_hz;

  t_prev = pcl_clock_now();

  while (!e->shutdown_requested) {
    double t_now = pcl_clock_now();
    double dt = t_now - t_prev;
    double elapsed;
    double remaining;
    t_prev = t_now;

    rc = drain_incoming_queue(e);
    if (rc != PCL_OK) {
      return rc;
    }

    drain_resp_cb_queue(e);

    for (i = 0; i < e->container_count; ++i) {
      tick_container(e->containers[i], dt);
    }

    elapsed = pcl_clock_now() - t_now;
    remaining = base_period - elapsed;
    if (remaining > 0.0) {
      pcl_sleep_seconds(remaining);
    }
  }

  pcl_log(NULL, PCL_LOG_INFO, "executor spin stopped");
  return PCL_OK;
}

pcl_status_t pcl_executor_spin_once(pcl_executor_t* e, uint32_t timeout_ms) {
  double t_now;
  double dt;
  pcl_status_t rc;
  uint32_t i;

  if (!e) return PCL_ERR_INVALID;
  (void)timeout_ms;

  t_now = pcl_clock_now();
  dt = (e->prev_time > 0.0) ? (t_now - e->prev_time) : 0.001;
  e->prev_time = t_now;

  rc = drain_incoming_queue(e);
  if (rc != PCL_OK) return rc;

  drain_resp_cb_queue(e);

  for (i = 0; i < e->container_count; ++i) {
    tick_container(e->containers[i], dt);
  }

  return PCL_OK;
}

// -- Shutdown ------------------------------------------------------------

void pcl_executor_request_shutdown(pcl_executor_t* e) {
  if (e) e->shutdown_requested = 1;
}

pcl_status_t pcl_executor_shutdown_graceful(pcl_executor_t* e,
                                            uint32_t        timeout_ms) {
  double deadline;
  int timed_out = 0;
  uint32_t i;

  if (!e) return PCL_ERR_INVALID;

  pcl_executor_request_shutdown(e);

  deadline = pcl_clock_now() + (double)timeout_ms / 1000.0;

  for (i = 0; i < e->container_count; ++i) {
    pcl_container_t* c = e->containers[i];

    if (timeout_ms > 0 && pcl_clock_now() >= deadline) {
      timed_out = 1;
      pcl_log(c, PCL_LOG_WARN, "shutdown timeout - forcing finalize");
    }

    if (pcl_container_state(c) == PCL_STATE_ACTIVE) {
      pcl_container_deactivate(c);
    }

    if (pcl_container_state(c) != PCL_STATE_FINALIZED) {
      pcl_container_shutdown(c);
    }
  }

  return timed_out ? PCL_ERR_TIMEOUT : PCL_OK;
}

// -- Transport -----------------------------------------------------------

pcl_status_t pcl_executor_set_transport(pcl_executor_t*        e,
                                        const pcl_transport_t* transport) {
  if (!e) return PCL_ERR_INVALID;
  if (transport) {
    e->transport     = *transport;
    e->has_transport = 1;
  } else {
    memset(&e->transport, 0, sizeof(e->transport));
    e->has_transport = 0;
  }
  return PCL_OK;
}

pcl_status_t pcl_executor_register_transport(pcl_executor_t*        e,
                                             const char*            peer_id,
                                             const pcl_transport_t* transport) {
  uint32_t i;

  if (!e || !peer_id) return PCL_ERR_INVALID;

  for (i = 0; i < e->transport_count; ++i) {
    if (e->transports[i].in_use &&
        strcmp(e->transports[i].peer_id, peer_id) == 0) {
      if (transport) {
        e->transports[i].transport = *transport;
      } else {
        memset(&e->transports[i], 0, sizeof(e->transports[i]));
      }
      return PCL_OK;
    }
  }

  if (!transport) return PCL_OK;
  if (e->transport_count >= PCL_MAX_TRANSPORTS) return PCL_ERR_NOMEM;

  snprintf(e->transports[e->transport_count].peer_id,
           sizeof(e->transports[e->transport_count].peer_id),
           "%s",
           peer_id);
  e->transports[e->transport_count].transport = *transport;
  e->transports[e->transport_count].in_use = 1;
  e->transport_count++;
  return PCL_OK;
}

pcl_status_t pcl_executor_set_endpoint_route(pcl_executor_t*           e,
                                             const pcl_endpoint_route_t* route) {
  pcl_endpoint_route_entry_t* entry;
  uint32_t i;

  if (!e || !route || !route->endpoint_name) return PCL_ERR_INVALID;
  if (route->route_mode == PCL_ROUTE_NONE) return PCL_ERR_INVALID;
  if (route->peer_count > PCL_MAX_ENDPOINT_PEERS) return PCL_ERR_INVALID;
  if ((route->route_mode & PCL_ROUTE_REMOTE) == 0u && route->peer_count > 0u) {
    return PCL_ERR_INVALID;
  }
  if (route->endpoint_kind == PCL_ENDPOINT_CONSUMED &&
      route->route_mode == (PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE)) {
    return PCL_ERR_INVALID;
  }

  entry = find_endpoint_route_entry(e, route->endpoint_name, route->endpoint_kind);
  if (!entry) {
    if (e->endpoint_route_count >= PCL_MAX_ENDPOINT_ROUTES) return PCL_ERR_NOMEM;
    entry = &e->endpoint_routes[e->endpoint_route_count++];
    memset(entry, 0, sizeof(*entry));
    entry->in_use = 1;
  }

  snprintf(entry->endpoint_name, sizeof(entry->endpoint_name), "%s",
           route->endpoint_name);
  entry->endpoint_kind = route->endpoint_kind;
  entry->route_mode = route->route_mode;
  entry->peer_count = route->peer_count;
  for (i = 0; i < PCL_MAX_ENDPOINT_PEERS; ++i) {
    entry->peer_ids[i][0] = '\0';
  }
  for (i = 0; i < route->peer_count; ++i) {
    if (!route->peer_ids || !route->peer_ids[i]) return PCL_ERR_INVALID;
    snprintf(entry->peer_ids[i], sizeof(entry->peer_ids[i]), "%s",
             route->peer_ids[i]);
  }

  return PCL_OK;
}

pcl_status_t pcl_executor_dispatch_incoming(pcl_executor_t*  e,
                                            const char*      topic,
                                            const pcl_msg_t* msg) {
  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  return dispatch_incoming_now(e, topic, msg, PCL_ROUTE_LOCAL, NULL);
}

static struct pcl_port_t* find_service(pcl_executor_t* e,
                                       const char*     name,
                                       uint32_t        source_route_mode,
                                       const char*     source_peer_id) {
  uint32_t ci, pi;
  for (ci = 0; ci < e->container_count; ++ci) {
    pcl_container_t* c = e->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      struct pcl_port_t* port = &c->ports[pi];
      uint32_t route_mode;
      if (port->type == PCL_PORT_SERVICE &&
          strcmp(port->name, name) == 0 &&
          port->svc_handler != NULL) {
        route_mode = port_route_mode(e, port);
        if (!route_accepts(route_mode, source_route_mode)) {
          continue;
        }
        if (source_route_mode == PCL_ROUTE_REMOTE &&
            !peer_is_allowed(e, port, source_peer_id)) {
          continue;
        }
        return port;
      }
    }
  }
  return NULL;
}

static struct pcl_port_t* find_stream_service(pcl_executor_t* e, const char* name) {
  uint32_t ci, pi;
  for (ci = 0; ci < e->container_count; ++ci) {
    pcl_container_t* c = e->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      struct pcl_port_t* port = &c->ports[pi];
      if (port->type == PCL_PORT_STREAM_SERVICE &&
          strcmp(port->name, name) == 0 &&
          port->stream_handler != NULL) {
        return port;
      }
    }
  }
  return NULL;
}

pcl_status_t pcl_executor_invoke_service(pcl_executor_t*  e,
                                         const char*      service_name,
                                         const pcl_msg_t* request,
                                         pcl_msg_t*       response) {
  struct pcl_port_t* port;
  pcl_svc_context_t  ctx = {0};

  if (!e || !service_name || !request || !response) return PCL_ERR_INVALID;

  port = find_service(e, service_name, PCL_ROUTE_LOCAL, NULL);
  if (!port) return PCL_ERR_NOT_FOUND;

  ctx.executor = e;
  return port->svc_handler(port->owner, request, response, &ctx, port->svc_user_data);
}

pcl_status_t pcl_executor_invoke_service_remote(pcl_executor_t*  e,
                                                const char*      peer_id,
                                                const char*      service_name,
                                                const pcl_msg_t* request,
                                                pcl_msg_t*       response) {
  struct pcl_port_t*       port;
  pcl_svc_context_t        ctx = {0};
  const pcl_transport_t*   transport;

  if (!e || !peer_id || !service_name || !request || !response) {
    return PCL_ERR_INVALID;
  }

  port = find_service(e, service_name, PCL_ROUTE_REMOTE, peer_id);
  if (!port) return PCL_ERR_NOT_FOUND;

  transport = find_named_transport(e, peer_id);
  if (!transport && e->has_transport) {
    transport = &e->transport;
  }

  ctx.executor = e;
  ctx.transport = transport;
  snprintf(ctx.peer_id, sizeof(ctx.peer_id), "%s", peer_id);
  return port->svc_handler(port->owner, request, response, &ctx, port->svc_user_data);
}

pcl_status_t pcl_executor_publish_port(pcl_executor_t*   e,
                                       const pcl_port_t* port,
                                       const pcl_msg_t*  msg) {
  uint32_t route_mode;
  pcl_status_t rc = PCL_OK;
  int delivered = 0;

  if (!e || !port || !msg) return PCL_ERR_INVALID;

  route_mode = port_route_mode(e, port);
  if (route_mode & PCL_ROUTE_LOCAL) {
    rc = dispatch_incoming_now(e, port->name, msg, PCL_ROUTE_LOCAL, NULL);
    if (rc == PCL_OK) delivered = 1;
  }

  if (route_mode & PCL_ROUTE_REMOTE) {
    uint32_t i;
    uint32_t peer_count = port_peer_count(e, port);
    if (peer_count == 0u) {
      if (e->has_transport && e->transport.publish) {
        rc = e->transport.publish(e->transport.adapter_ctx, port->name, msg);
        if (rc == PCL_OK) delivered = 1;
      } else if (!delivered) {
        rc = PCL_ERR_NOT_FOUND;
      }
    } else {
      for (i = 0; i < peer_count; ++i) {
        const char* peer_id = port_peer_id_at(e, port, i);
        const pcl_transport_t* transport = find_named_transport(e, peer_id);
        if (!transport || !transport->publish) {
          rc = PCL_ERR_NOT_FOUND;
          continue;
        }
        rc = transport->publish(transport->adapter_ctx, port->name, msg);
        if (rc == PCL_OK) delivered = 1;
      }
    }
  }

  return delivered ? PCL_OK : rc;
}

pcl_status_t pcl_executor_publish(pcl_executor_t*  e,
                                  const char*      topic,
                                  const pcl_msg_t* msg) {
  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  if (e->has_transport && e->transport.publish) {
    return e->transport.publish(e->transport.adapter_ctx, topic, msg);
  }
  return dispatch_incoming_now(e, topic, msg, PCL_ROUTE_LOCAL, NULL);
}

pcl_status_t pcl_executor_invoke_async(pcl_executor_t*  e,
                                       const char*      service_name,
                                       const pcl_msg_t* request,
                                       pcl_resp_cb_fn_t callback,
                                       void*            user_data) {
  if (!e || !service_name || !request || !callback) return PCL_ERR_INVALID;

  {
    const pcl_endpoint_route_entry_t* route =
        find_endpoint_route_entry_const(e, service_name, PCL_ENDPOINT_CONSUMED);
    if (route) {
      if (route->route_mode == PCL_ROUTE_LOCAL) {
        /* handled below */
      } else if (route->route_mode == PCL_ROUTE_REMOTE) {
        const pcl_transport_t* transport = NULL;
        if (route->peer_count > 1u) return PCL_ERR_INVALID;
        if (route->peer_count == 1u) {
          transport = find_named_transport(e, route->peer_ids[0]);
        } else if (e->has_transport) {
          transport = &e->transport;
        }
        if (!transport || !transport->invoke_async) return PCL_ERR_NOT_FOUND;
        return transport->invoke_async(transport->adapter_ctx, service_name,
                                       request, callback, user_data);
      } else {
        return PCL_ERR_INVALID;
      }
    }
  }

  // Legacy executor-wide transport fallback
  if (e->has_transport && e->transport.invoke_async) {
    return e->transport.invoke_async(e->transport.adapter_ctx, service_name,
                                     request, callback, user_data);
  }

  // Intra-process fallback: invoke service, handle immediate or deferred response
  {
    struct pcl_port_t* port = find_service(e, service_name, PCL_ROUTE_LOCAL, NULL);
    pcl_msg_t          response = {0};
    pcl_svc_context_t* ctx;
    pcl_status_t       rc;

    if (!port) return PCL_ERR_NOT_FOUND;

    // Allocate context for potential deferred response
    ctx = (pcl_svc_context_t*)calloc(1, sizeof(*ctx));
    if (!ctx) return PCL_ERR_NOMEM;

    ctx->executor  = e;
    ctx->callback  = callback;
    ctx->user_data = user_data;

    rc = port->svc_handler(port->owner, request, &response, ctx, port->svc_user_data);

    if (rc == PCL_OK) {
      // Immediate response — fire callback and free context
      callback(&response, user_data);
      free(ctx);
    } else if (rc == PCL_PENDING) {
      // Deferred — handler saved ctx, will call pcl_service_respond later
      rc = PCL_OK;
    } else {
      // Error — free context
      free(ctx);
    }
    return rc;
  }
}

pcl_status_t pcl_executor_invoke_stream(pcl_executor_t*        e,
                                        const char*            service_name,
                                        const pcl_msg_t*       request,
                                        pcl_stream_msg_fn_t    callback,
                                        void*                  user_data,
                                        pcl_stream_context_t** out_ctx) {
  if (!e || !service_name || !request || !callback) return PCL_ERR_INVALID;

  // If transport has invoke_stream, use it
  if (e->has_transport && e->transport.invoke_stream) {
    void* stream_handle = NULL;
    pcl_status_t rc = e->transport.invoke_stream(
        e->transport.adapter_ctx, service_name, request,
        callback, user_data, &stream_handle);
    if (rc == PCL_OK || rc == PCL_STREAMING) {
      if (out_ctx) {
        pcl_stream_context_t* ctx = (pcl_stream_context_t*)calloc(1, sizeof(*ctx));
        if (ctx) {
          ctx->executor      = e;
          ctx->callback      = callback;
          ctx->user_data     = user_data;
          ctx->transport     = &e->transport;
          ctx->transport_ctx = stream_handle;
          *out_ctx = ctx;
        }
      }
    }
    return rc;
  }

  // Intra-process fallback: invoke streaming service
  {
    struct pcl_port_t*    port = find_stream_service(e, service_name);
    pcl_stream_context_t* ctx;
    pcl_status_t          rc;

    if (!port) return PCL_ERR_NOT_FOUND;

    // Allocate stream context
    ctx = (pcl_stream_context_t*)calloc(1, sizeof(*ctx));
    if (!ctx) return PCL_ERR_NOMEM;

    ctx->executor  = e;
    ctx->callback  = callback;
    ctx->user_data = user_data;

    rc = port->stream_handler(port->owner, request, ctx, port->stream_user_data);

    if (rc == PCL_STREAMING) {
      // Handler saved ctx, will use pcl_stream_send/end/abort
      if (out_ctx) *out_ctx = ctx;
      return PCL_OK;
    } else {
      // Error or unexpected return — free context
      free(ctx);
      if (out_ctx) *out_ctx = NULL;
      return rc;
    }
  }
}

pcl_status_t pcl_executor_post_response_cb(pcl_executor_t*  e,
                                           pcl_resp_cb_fn_t cb,
                                           void*            user_data,
                                           const void*      data,
                                           uint32_t         size) {
  pcl_resp_cb_node_t* node;

  if (!e || !cb) return PCL_ERR_INVALID;

  node = (pcl_resp_cb_node_t*)calloc(1, sizeof(*node));
  if (!node) return PCL_ERR_NOMEM;

  node->cb        = cb;
  node->user_data = user_data;
  node->size      = size;

  if (size > 0u && data) {
    node->data = malloc(size);
    if (!node->data) {
      free(node);
      return PCL_ERR_NOMEM;
    }
    memcpy(node->data, data, size);
  }

  pcl_mutex_lock(&e->resp_cb_lock);
  if (e->resp_cb_tail) {
    e->resp_cb_tail->next = node;
  } else {
    e->resp_cb_head = node;
  }
  e->resp_cb_tail = node;
  pcl_mutex_unlock(&e->resp_cb_lock);

  return PCL_OK;
}

static pcl_status_t enqueue_incoming_message(pcl_executor_t*  e,
                                             const char*      topic,
                                             const pcl_msg_t* msg,
                                             uint32_t         source_route_mode,
                                             const char*      source_peer_id) {
  pcl_pending_msg_t* pending;

  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  if (msg->size > 0u && !msg->data) return PCL_ERR_INVALID;
  if (!msg->type_name) return PCL_ERR_INVALID;

  pending = (pcl_pending_msg_t*)calloc(1, sizeof(pcl_pending_msg_t));
  if (!pending) return PCL_ERR_NOMEM;

  pending->topic = pcl_strdup_local(topic);
  pending->type_name = pcl_strdup_local(msg->type_name);
  pending->source_route_mode = source_route_mode;
  if (source_peer_id) {
    pending->source_peer_id = pcl_strdup_local(source_peer_id);
  }

  if (!pending->topic || !pending->type_name ||
      (source_peer_id && !pending->source_peer_id)) {
    free_pending_msg(pending);
    return PCL_ERR_NOMEM;
  }

  if (msg->size > 0u) {
    pending->data = malloc(msg->size);
    if (!pending->data) {
      free_pending_msg(pending);
      return PCL_ERR_NOMEM;
    }
    memcpy(pending->data, msg->data, msg->size);
  }
  pending->size = msg->size;

  pcl_mutex_lock(&e->incoming_lock);
  if (e->incoming_tail) {
    e->incoming_tail->next = pending;
  } else {
    e->incoming_head = pending;
  }
  e->incoming_tail = pending;
  pcl_mutex_unlock(&e->incoming_lock);

  return PCL_OK;
}

pcl_status_t pcl_executor_post_incoming(pcl_executor_t*  e,
                                        const char*      topic,
                                        const pcl_msg_t* msg) {
  return enqueue_incoming_message(e, topic, msg, PCL_ROUTE_LOCAL, NULL);
}

pcl_status_t pcl_executor_post_remote_incoming(pcl_executor_t*  e,
                                               const char*      peer_id,
                                               const char*      topic,
                                               const pcl_msg_t* msg) {
  if (!peer_id) return PCL_ERR_INVALID;
  return enqueue_incoming_message(e, topic, msg, PCL_ROUTE_REMOTE, peer_id);
}
