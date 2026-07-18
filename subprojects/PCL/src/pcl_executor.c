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
#include "pcl/pcl_alloc.h"

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

static void pcl_idle_wait_ms(uint32_t timeout_ms) {
  if (timeout_ms == 0u) return;
#ifdef _WIN32
  if (timeout_ms <= 1u) {
    if (!SwitchToThread()) {
      Sleep(0);
    }
    return;
  }
#endif
  pcl_sleep_seconds((double)timeout_ms / 1000.0);
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
  copy = (char*)pcl_alloc(len);
  if (!copy) return NULL;

  memcpy(copy, src, len);
  return copy;
}

/* Implements: REQ_PCL_052. */
static void free_pending_msg(pcl_pending_msg_t* pending) {
  if (!pending) return;
  pcl_free(pending->topic);
  pcl_free(pending->type_name);
  pcl_free(pending->source_peer_id);
  pcl_free(pending->data);
  pcl_free(pending);
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

/* Implements: REQ_PCL_315, REQ_PCL_316, REQ_PCL_326, REQ_PCL_327. */
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

/* Implements: REQ_PCL_495, REQ_PCL_497. */
static pcl_status_t subscribe_port_with_transport(
    const pcl_executor_t* e,
    const pcl_port_t*     port,
    const char*           peer_id,
    const pcl_transport_t* transport) {
  uint32_t i;
  uint32_t peer_count;

  if (!e || !port || !transport || port->type != PCL_PORT_SUBSCRIBER) {
    return PCL_OK;
  }
  if ((port_route_mode(e, port) & PCL_ROUTE_REMOTE) == 0u) return PCL_OK;

  peer_count = port_peer_count(e, port);
  if (peer_id) {
    for (i = 0; i < peer_count; ++i) {
      const char* routed_peer = port_peer_id_at(e, port, i);
      if (routed_peer && strcmp(routed_peer, peer_id) == 0) break;
    }
    if (i == peer_count) return PCL_OK;
  } else if (peer_count != 0u) {
    /* Default-transport pass (peer_id NULL): a port that selected specific
       peers binds only to its named transports, never the default. */
    return PCL_OK;
  }

  if (!transport->subscribe) {
    pcl_log(port->owner, PCL_LOG_ERROR,
            "remote subscriber configuration failed for '%s': selected transport "
            "does not support subscriptions",
            port->name);
    return PCL_ERR_NOT_FOUND;
  }
  return transport->subscribe(transport->adapter_ctx, port->name,
                              port->type_name);
}

/* Implements: REQ_PCL_495, REQ_PCL_236. */
static pcl_status_t subscribe_container_ports(pcl_executor_t* e,
                                              pcl_container_t* c) {
  uint32_t pi;

  for (pi = 0; pi < c->port_count; ++pi) {
    const pcl_port_t* port = &c->ports[pi];
    uint32_t peer_count;
    uint32_t i;
    pcl_status_t rc;

    if (port->type != PCL_PORT_SUBSCRIBER ||
        (port_route_mode(e, port) & PCL_ROUTE_REMOTE) == 0u) {
      continue;
    }

    peer_count = port_peer_count(e, port);
    if (peer_count == 0u) {
      if (!e->has_transport) continue;
      rc = subscribe_port_with_transport(e, port, NULL, &e->transport);
      if (rc != PCL_OK) return rc;
      continue;
    }

    for (i = 0; i < peer_count; ++i) {
      const char* peer_id = port_peer_id_at(e, port, i);
      const pcl_transport_t* transport = find_named_transport(e, peer_id);
      if (!transport) continue;
      rc = subscribe_port_with_transport(e, port, peer_id, transport);
      if (rc != PCL_OK) return rc;
    }
  }
  return PCL_OK;
}

/* Implements: REQ_PCL_496, REQ_PCL_229. */
static pcl_status_t subscribe_existing_ports(pcl_executor_t* e,
                                             const char* peer_id,
                                             const pcl_transport_t* transport) {
  uint32_t ci;
  uint32_t pi;

  for (ci = 0; ci < e->container_count; ++ci) {
    const pcl_container_t* c = e->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      pcl_status_t rc = subscribe_port_with_transport(
          e, &c->ports[pi], peer_id, transport);
      if (rc != PCL_OK) return rc;
    }
  }
  return PCL_OK;
}

/* Implements: REQ_PCL_173. */
static int route_accepts(uint32_t route_mode,
                         uint32_t source_route_mode) {
  return (route_mode & source_route_mode) != 0u;
}

/* Implements: REQ_PCL_173. */
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

/* Implements: REQ_PCL_031. */
pcl_executor_t* pcl_executor_create(void) {
  pcl_executor_t* e = (pcl_executor_t*)pcl_calloc(1, sizeof(pcl_executor_t));
  if (!e) return NULL;
  e->container_count    = 0;
  e->has_transport      = 0;
  e->shutdown_requested = 0;
  e->prev_time          = 0.0;
  pcl_mutex_init(&e->containers_lock);
  pcl_mutex_init(&e->incoming_lock);
  pcl_mutex_init(&e->resp_cb_lock);
  pcl_mutex_init(&e->svc_req_lock);
  return e;
}

/* No LLR: internal cross-translation-unit locking primitive documented in
   pcl_internal.h, not a requirement-bearing API of its own; its correctness
   is exercised through the concurrency requirements it protects (see
   REQ_PCL_053 and the threading-model conformance suite, section 34 of
   LLR.md). */
void pcl_executor_containers_lock(pcl_executor_t* e) {
  if (!e) return;
  pcl_mutex_lock(&e->containers_lock);
}

/* No LLR: see pcl_executor_containers_lock() above. */
void pcl_executor_containers_unlock(pcl_executor_t* e) {
  if (!e) return;
  pcl_mutex_unlock(&e->containers_lock);
}

/* Implements: REQ_PCL_094, REQ_PCL_052, REQ_PCL_061, REQ_PCL_185. */
void pcl_executor_destroy(pcl_executor_t* e) {
  pcl_pending_msg_t* pending;
  uint32_t i;
  if (!e) return;
  pcl_mutex_lock(&e->containers_lock);
  for (i = 0; i < e->container_count; ++i) {
    e->containers[i]->executor = NULL;
  }
  e->container_count = 0;
  pcl_mutex_unlock(&e->containers_lock);
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
  e->incoming_count = 0u;
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
      pcl_free(node->data);
      pcl_free(node->type_name);
      pcl_free(node);
      node = next;
    }
  }
  pcl_mutex_destroy(&e->resp_cb_lock);

  {
    pcl_pending_svc_req_t* node;
    pcl_mutex_lock(&e->svc_req_lock);
    node = e->svc_req_head;
    e->svc_req_head = NULL;
    e->svc_req_tail = NULL;
    pcl_mutex_unlock(&e->svc_req_lock);
    while (node) {
      pcl_pending_svc_req_t* next = node->next;
      pcl_free(node->service_name);
      pcl_free(node->type_name);
      pcl_free(node->data);
      pcl_free(node->source_peer_id);
      pcl_free(node);
      node = next;
    }
  }
  pcl_mutex_destroy(&e->svc_req_lock);
  pcl_mutex_destroy(&e->incoming_lock);
  pcl_mutex_destroy(&e->containers_lock);
  pcl_free(e);
}

// -- Container management ------------------------------------------------

/* Implements: REQ_PCL_038, REQ_PCL_037, REQ_PCL_495, REQ_PCL_236,
   REQ_PCL_230. */
pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c) {
  pcl_status_t rc;
  if (!e || !c) return PCL_ERR_INVALID;
  pcl_mutex_lock(&e->containers_lock);
  if (e->container_count >= PCL_MAX_CONTAINERS) {
    pcl_mutex_unlock(&e->containers_lock);
    return PCL_ERR_NOMEM;
  }
  pcl_mutex_unlock(&e->containers_lock);

  rc = subscribe_container_ports(e, c);
  if (rc != PCL_OK) return rc;

  pcl_mutex_lock(&e->containers_lock);
  e->containers[e->container_count++] = c;
  c->executor = e;
  pcl_mutex_unlock(&e->containers_lock);
  return PCL_OK;
}

/* Implements: REQ_PCL_039. */
pcl_status_t pcl_executor_remove(pcl_executor_t* e, pcl_container_t* c) {
  uint32_t i;
  if (!e || !c) return PCL_ERR_INVALID;
  pcl_mutex_lock(&e->containers_lock);
  for (i = 0; i < e->container_count; ++i) {
    if (e->containers[i] == c) {
      c->executor = NULL;
      memmove(&e->containers[i], &e->containers[i + 1],
              (e->container_count - i - 1) * sizeof(e->containers[0]));
      e->container_count--;
      pcl_mutex_unlock(&e->containers_lock);
      return PCL_OK;
    }
  }
  pcl_mutex_unlock(&e->containers_lock);
  return PCL_ERR_NOT_FOUND;
}

// -- Intra-process dispatch ----------------------------------------------

/* Implements: REQ_PCL_044, REQ_PCL_045, REQ_PCL_046, REQ_PCL_173. */
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

/* Implements: REQ_PCL_057, REQ_PCL_058, REQ_PCL_059. */
static uint32_t drain_resp_cb_queue(pcl_executor_t* e) {
  uint32_t drained = 0u;
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
    resp.type_name = node->type_name;

    node->cb(&resp, node->user_data);

    pcl_free(node->data);
    pcl_free(node->type_name);
    pcl_free(node);
    ++drained;
  }
  return drained;
}

/* Implements: REQ_PCL_054. */
static pcl_status_t drain_incoming_queue(pcl_executor_t* e,
                                         uint32_t*        drained_count) {
  pcl_pending_msg_t* pending;
  pcl_status_t       rc = PCL_OK;
  uint32_t           drained = 0u;

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
      --e->incoming_count;
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
    ++drained;

    if (rc != PCL_OK) {
      return rc;
    }
  }

  if (drained_count) *drained_count = drained;
  return PCL_OK;
}

/* Forward declaration — find_service is defined later in this translation unit. */
static struct pcl_port_t* find_service(pcl_executor_t* e,
                                       const char*     name,
                                       uint32_t        source_route_mode,
                                       const char*     source_peer_id);

// -- Service request queue drain -----------------------------------------
// Runs on the executor thread.  For each queued request:
//   1. Locate the service port (local route only).
//   2. Invoke the handler synchronously on the executor thread.
//   3. Fire the response callback — also on the executor thread.
// If the service is not found, the callback is still fired with an empty
// message so the caller is never silently abandoned.

/* Implements: REQ_PCL_181, REQ_PCL_182, REQ_PCL_183, REQ_PCL_459. */
static uint32_t drain_svc_req_queue(pcl_executor_t* e) {
  uint32_t drained = 0u;
  for (;;) {
    pcl_pending_svc_req_t* node;

    pcl_mutex_lock(&e->svc_req_lock);
    node = e->svc_req_head;
    if (node) {
      e->svc_req_head = node->next;
      if (!e->svc_req_head) e->svc_req_tail = NULL;
    }
    pcl_mutex_unlock(&e->svc_req_lock);

    if (!node) break;

    {
      pcl_msg_t req;
      memset(&req, 0, sizeof(req));
      req.data      = node->data;
      req.size      = node->size;
      req.type_name = node->type_name;

      struct pcl_port_t* port = node->source_peer_id
          ? find_service(e, node->service_name,
                         PCL_ROUTE_REMOTE, node->source_peer_id)
          : find_service(e, node->service_name,
                         PCL_ROUTE_LOCAL, NULL);
      if (port) {
        pcl_svc_context_t* ctx = (pcl_svc_context_t*)pcl_calloc(1, sizeof(*ctx));
        if (ctx) {
          pcl_msg_t    resp = {0};
          pcl_status_t rc;

          ctx->executor  = e;
          ctx->callback  = node->callback;
          ctx->user_data = node->user_data;

          rc = port->svc_handler(port->owner, &req, &resp,
                                 ctx, port->svc_user_data);
          if (rc == PCL_OK) {
            node->callback(&resp, node->user_data);
            pcl_free(ctx);
          } else if (rc == PCL_PENDING) {
            /* handler saved ctx; will fire callback via pcl_service_respond */
          } else {
            /* handler error — notify caller with empty response */
            pcl_msg_t empty = {0};
            node->callback(&empty, node->user_data);
            pcl_free(ctx);
          }
        } else {
          /* OOM allocating context — notify caller */
          pcl_msg_t empty = {0};
          node->callback(&empty, node->user_data);
        }
      } else {
        /* Service not registered — notify caller so it is not silently abandoned */
        pcl_msg_t empty = {0};
        node->callback(&empty, node->user_data);
      }
    }

    pcl_free(node->service_name);
    pcl_free(node->type_name);
    pcl_free(node->data);
    pcl_free(node->source_peer_id);
    pcl_free(node);
    ++drained;
  }
  return drained;
}

// -- Tick one container --------------------------------------------------

/* Implements: REQ_PCL_032, REQ_PCL_033, REQ_PCL_034. */
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

/* Implements: REQ_PCL_035, REQ_PCL_055. */
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
    uint32_t incoming_drained = 0u;
    double t_now = pcl_clock_now();
    double dt = t_now - t_prev;
    double elapsed;
    double remaining;
    t_prev = t_now;

    rc = drain_incoming_queue(e, &incoming_drained);
    if (rc != PCL_OK) {
      return rc;
    }

    (void)drain_svc_req_queue(e);
    (void)drain_resp_cb_queue(e);

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

/* Implements: REQ_PCL_032, REQ_PCL_043, REQ_PCL_091, REQ_PCL_092,
   REQ_PCL_093. */
pcl_status_t pcl_executor_spin_once(pcl_executor_t* e, uint32_t timeout_ms) {
  double t_now;
  double dt;
  pcl_status_t rc;
  uint32_t i;
  uint32_t incoming_drained = 0u;
  uint32_t svc_drained = 0u;
  uint32_t resp_drained = 0u;

  if (!e) return PCL_ERR_INVALID;

  t_now = pcl_clock_now();
  dt = (e->prev_time > 0.0) ? (t_now - e->prev_time) : 0.001;
  e->prev_time = t_now;

  rc = drain_incoming_queue(e, &incoming_drained);
  if (rc != PCL_OK) return rc;

  svc_drained = drain_svc_req_queue(e);
  resp_drained = drain_resp_cb_queue(e);

  for (i = 0; i < e->container_count; ++i) {
    tick_container(e->containers[i], dt);
  }

  if (timeout_ms > 0u &&
      incoming_drained == 0u &&
      svc_drained == 0u &&
      resp_drained == 0u) {
    pcl_idle_wait_ms(timeout_ms);
  }

  return PCL_OK;
}

// -- Shutdown ------------------------------------------------------------

/* Implements: REQ_PCL_035, REQ_PCL_113. */
void pcl_executor_request_shutdown(pcl_executor_t* e) {
  if (e) e->shutdown_requested = 1;
}

/* Implements: REQ_PCL_036, REQ_PCL_041, REQ_PCL_042. */
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

/* Implements: REQ_PCL_062, REQ_PCL_063, REQ_PCL_384, REQ_PCL_229,
   REQ_PCL_230. */
pcl_status_t pcl_executor_set_transport_caps(pcl_executor_t*        e,
                                             const pcl_transport_t* transport,
                                             pcl_transport_caps_t   caps) {
  pcl_status_t rc;
  if (!e) return PCL_ERR_INVALID;
  if (transport) {
    rc = subscribe_existing_ports(e, NULL, transport);
    if (rc != PCL_OK) return rc;
    e->transport       = *transport;
    e->transport_caps  = caps;
    e->has_transport   = 1;
  } else {
    memset(&e->transport, 0, sizeof(e->transport));
    e->transport_caps  = PCL_CAP_NONE;
    e->has_transport   = 0;
  }
  return PCL_OK;
}

/* Implements: REQ_PCL_062, REQ_PCL_063, REQ_PCL_384. */
pcl_status_t pcl_executor_set_transport(pcl_executor_t*        e,
                                        const pcl_transport_t* transport) {
  return pcl_executor_set_transport_caps(
      e, transport, pcl_transport_caps_from_vtable(transport));
}

/* No LLR: plain accessor for the default transport slot with no dedicated
   requirement; exercised implicitly wherever tests install a transport via
   pcl_executor_set_transport() (REQ_PCL_062) and then act on it. */
const pcl_transport_t* pcl_executor_get_transport(const pcl_executor_t* e) {
  if (!e || !e->has_transport) return NULL;
  return &e->transport;
}

/* No LLR: plain accessor delegating to find_named_transport(); no
   requirement names it directly. */
const pcl_transport_t* pcl_executor_get_transport_for_peer(
    const pcl_executor_t* e,
    const char*           peer_id) {
  return find_named_transport(e, peer_id);
}

/* Implements: REQ_PCL_178, REQ_PCL_496, REQ_PCL_425, REQ_PCL_230. */
pcl_status_t pcl_executor_register_transport_caps(pcl_executor_t*        e,
                                                  const char*            peer_id,
                                                  const pcl_transport_t* transport,
                                                  pcl_transport_caps_t   caps) {
  uint32_t i;
  pcl_status_t rc;

  if (!e || !peer_id) return PCL_ERR_INVALID;

  if (transport) {
    rc = subscribe_existing_ports(e, peer_id, transport);
    if (rc != PCL_OK) return rc;
  }

  for (i = 0; i < e->transport_count; ++i) {
    if (e->transports[i].in_use &&
        strcmp(e->transports[i].peer_id, peer_id) == 0) {
      if (transport) {
        e->transports[i].transport = *transport;
        e->transports[i].caps      = caps;
      } else {
        /* Compact so the freed slot is reused: move the last live slot into
           this one and shrink the count. Without this, repeated manifest
           load/destroy cycles on one executor leak slots and eventually hit
           PCL_MAX_TRANSPORTS even though nothing remains registered. Lookups
           are by peer_id, so reordering the array is safe. */
        uint32_t last = e->transport_count - 1u;
        if (i != last) {
          e->transports[i] = e->transports[last];
        }
        memset(&e->transports[last], 0, sizeof(e->transports[last]));
        e->transport_count--;
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
  e->transports[e->transport_count].caps      = caps;
  e->transports[e->transport_count].in_use = 1;
  e->transport_count++;
  return PCL_OK;
}

/* Implements: REQ_PCL_178, REQ_PCL_496, REQ_PCL_425, REQ_PCL_230. */
pcl_status_t pcl_executor_register_transport(pcl_executor_t*        e,
                                             const char*            peer_id,
                                             const pcl_transport_t* transport) {
  return pcl_executor_register_transport_caps(
      e, peer_id, transport, pcl_transport_caps_from_vtable(transport));
}

/* Implements: REQ_PCL_377. */
pcl_status_t pcl_executor_set_transport_qos(pcl_executor_t* e, pcl_qos_t qos) {
  if (!e) return PCL_ERR_INVALID;
  e->transport_qos = qos;
  return PCL_OK;
}

/* Implements: REQ_PCL_378. */
pcl_status_t pcl_executor_register_transport_qos(pcl_executor_t* e,
                                                 const char*     peer_id,
                                                 pcl_qos_t       qos) {
  uint32_t i;
  if (!e || !peer_id) return PCL_ERR_INVALID;
  for (i = 0; i < e->transport_count; ++i) {
    if (e->transports[i].in_use &&
        strcmp(e->transports[i].peer_id, peer_id) == 0) {
      e->transports[i].qos = qos;
      return PCL_OK;
    }
  }
  return PCL_ERR_NOT_FOUND;
}

static const char* caps_name(pcl_transport_caps_t cap) {
  switch (cap) {
    case PCL_CAP_PUBSUB:     return "PUBSUB";
    case PCL_CAP_RPC_UNARY:  return "RPC_UNARY";
    case PCL_CAP_RPC_STREAM: return "RPC_STREAM";
    // GCOVR_EXCL_START: diagnostics only run for caps an endpoint can require,
    // and pcl_endpoint_required_caps never yields RPC_ACTION or NONE.
    case PCL_CAP_RPC_ACTION: return "RPC_ACTION";
    default:                 return "NONE";
    // GCOVR_EXCL_STOP
  }
}

/* Implements: REQ_PCL_365, REQ_PCL_366, REQ_PCL_367, REQ_PCL_371,
   REQ_PCL_372, REQ_PCL_373, REQ_PCL_374. */
static pcl_status_t validate_one_transport(pcl_transport_caps_t have,
                                           int                  found,
                                           pcl_transport_caps_t required,
                                           pcl_qos_t            offered,
                                           pcl_qos_t            floor,
                                           const char*          endpoint_name,
                                           const char*          peer_label,
                                           char*                diag,
                                           size_t               diag_size) {
  if (!found) {
    if (diag && diag_size) {
      snprintf(diag, diag_size,
               "endpoint '%s' routes to %s but no transport is registered there",
               endpoint_name ? endpoint_name : "?", peer_label);
    }
    return PCL_ERR_NOT_FOUND;
  }
  if (!pcl_transport_caps_supports(have, required)) {
    if (diag && diag_size) {
      snprintf(diag, diag_size,
               "endpoint '%s' requires %s but %s provides caps 0x%x",
               endpoint_name ? endpoint_name : "?", caps_name(required),
               peer_label, (unsigned)have);
    }
    return PCL_ERR_STATE;
  }
  if (!pcl_qos_satisfies(offered, floor)) {
    if (diag && diag_size) {
      snprintf(diag, diag_size,
               "endpoint '%s' requires reliability '%s' but %s offers '%s'",
               endpoint_name ? endpoint_name : "?",
               pcl_qos_reliability_name(floor.reliability),
               peer_label,
               pcl_qos_reliability_name(offered.reliability));
    }
    return PCL_ERR_STATE;
  }
  return PCL_OK;
}

/* Implements: REQ_PCL_365, REQ_PCL_366, REQ_PCL_367, REQ_PCL_368,
   REQ_PCL_369, REQ_PCL_371, REQ_PCL_372, REQ_PCL_373, REQ_PCL_374,
   REQ_PCL_379, REQ_PCL_380. */
pcl_status_t pcl_executor_validate_endpoint_route(
    const pcl_executor_t*       e,
    const pcl_endpoint_route_t* route,
    char*                       diag,
    size_t                      diag_size) {
  char local_diag[256] = {0};
  char* active_diag;
  size_t active_diag_size;
  pcl_transport_caps_t required;
  uint32_t p;

  if (diag && diag_size) diag[0] = '\0';
  if (!e || !route) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "endpoint route validation failed: executor and route are required");
    return PCL_ERR_INVALID;
  }
  active_diag = (diag && diag_size) ? diag : local_diag;
  active_diag_size = (diag && diag_size) ? diag_size : sizeof(local_diag);

  required = pcl_endpoint_required_caps(route->endpoint_kind);
  /* No requirement (unknown kind) or no remote leg: nothing to validate. */
  if (required == PCL_CAP_NONE) return PCL_OK;
  if ((route->route_mode & PCL_ROUTE_REMOTE) == 0u) return PCL_OK;

  if (route->peer_count == 0u) {
    /* Remote via the default transport. */
    char label[80];
    pcl_status_t rc;
    snprintf(label, sizeof(label), "the default transport");
    rc = validate_one_transport(e->transport_caps, e->has_transport, required,
                                e->transport_qos, route->qos_floor,
                                route->endpoint_name, label,
                                active_diag, active_diag_size);
    if (rc != PCL_OK) {
      pcl_log(NULL, PCL_LOG_ERROR,
              "endpoint route validation failed: %s (rc=%d)",
              active_diag, (int)rc);
    }
    return rc;
  }

  for (p = 0; p < route->peer_count; ++p) {
    const char* peer_id = route->peer_ids ? route->peer_ids[p] : NULL;
    pcl_transport_caps_t have = PCL_CAP_NONE;
    pcl_qos_t offered = {PCL_QOS_RELIABILITY_UNSPECIFIED};
    int found = 0;
    uint32_t i;
    char label[96];
    pcl_status_t rc;

    for (i = 0; i < e->transport_count; ++i) {
      if (e->transports[i].in_use && peer_id &&
          strcmp(e->transports[i].peer_id, peer_id) == 0) {
        have = e->transports[i].caps;
        offered = e->transports[i].qos;
        found = 1;
        break;
      }
    }
    snprintf(label, sizeof(label), "peer '%s'", peer_id ? peer_id : "?");
    rc = validate_one_transport(have, found, required, offered, route->qos_floor,
                                route->endpoint_name, label,
                                active_diag, active_diag_size);
    if (rc != PCL_OK) {
      pcl_log(NULL, PCL_LOG_ERROR,
              "endpoint route validation failed: %s (rc=%d)",
              active_diag, (int)rc);
      return rc;
    }
  }
  return PCL_OK;
}

/* No LLR: argument-validation and storage primitive for the executor's
   endpoint route table with no requirement of its own; its port-level twin
   pcl_port_set_route() carries the analogous REQ_PCL_235, and this
   function's effect is exercised through the manifest-routing
   requirements (e.g. REQ_PCL_416-REQ_PCL_428) that call it internally. */
pcl_status_t pcl_executor_set_endpoint_route(pcl_executor_t*           e,
                                             const pcl_endpoint_route_t* route) {
  pcl_endpoint_route_entry_t* entry;
  uint32_t i;

  if (!e || !route || !route->endpoint_name) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "endpoint route configuration failed: executor, route, and endpoint name are required");
    return PCL_ERR_INVALID;
  }
  if (route->route_mode == PCL_ROUTE_NONE) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "endpoint route configuration failed for '%s': route mode is NONE",
            route->endpoint_name);
    return PCL_ERR_INVALID;
  }
  if (route->peer_count > PCL_MAX_ENDPOINT_PEERS) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "endpoint route configuration failed for '%s': %u peers exceeds the limit of %u",
            route->endpoint_name, (unsigned)route->peer_count,
            (unsigned)PCL_MAX_ENDPOINT_PEERS);
    return PCL_ERR_INVALID;
  }
  if ((route->route_mode & PCL_ROUTE_REMOTE) == 0u && route->peer_count > 0u) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "endpoint route configuration failed for '%s': peer IDs require a remote route",
            route->endpoint_name);
    return PCL_ERR_INVALID;
  }
  if ((route->endpoint_kind == PCL_ENDPOINT_CONSUMED ||
       route->endpoint_kind == PCL_ENDPOINT_STREAM_CONSUMED) &&
      route->route_mode == (PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE)) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "endpoint route configuration failed for '%s': consumed endpoints "
            "cannot route both locally and remotely",
            route->endpoint_name);
    return PCL_ERR_INVALID;
  }

  entry = find_endpoint_route_entry(e, route->endpoint_name, route->endpoint_kind);
  if (!entry) {
    if (e->endpoint_route_count >= PCL_MAX_ENDPOINT_ROUTES) {
      pcl_log(NULL, PCL_LOG_ERROR,
              "endpoint route configuration failed for '%s': executor limit of %u routes was reached",
              route->endpoint_name, (unsigned)PCL_MAX_ENDPOINT_ROUTES);
      return PCL_ERR_NOMEM;
    }
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
    if (!route->peer_ids || !route->peer_ids[i]) {
      pcl_log(NULL, PCL_LOG_ERROR,
              "endpoint route configuration failed for '%s': peer ID %u is missing",
              route->endpoint_name, (unsigned)i);
      return PCL_ERR_INVALID;
    }
    snprintf(entry->peer_ids[i], sizeof(entry->peer_ids[i]), "%s",
             route->peer_ids[i]);
  }

  return PCL_OK;
}

/* No LLR: plain query accessor for a single route-kind lookup with no
   requirement of its own; pcl_executor_endpoint_route_exists_any_kind()
   below is the kind-agnostic query the exclusivity requirements name. */
int pcl_executor_endpoint_route_exists(const pcl_executor_t* e,
                                       const char*           endpoint_name,
                                       pcl_endpoint_kind_t   endpoint_kind) {
  if (!e || !endpoint_name) return 0;
  return find_endpoint_route_entry_const(e, endpoint_name, endpoint_kind) != NULL;
}

/* Implements: REQ_PCL_474, REQ_PCL_475. */
int pcl_executor_endpoint_route_exists_any_kind(const pcl_executor_t* e,
                                                const char*           endpoint_name) {
  uint32_t i;
  if (!e || !endpoint_name) return 0;
  for (i = 0; i < e->endpoint_route_count; ++i) {
    if (e->endpoint_routes[i].in_use &&
        strcmp(e->endpoint_routes[i].endpoint_name, endpoint_name) == 0) {
      return 1;
    }
  }
  /* The endpoint route table (above) is not the only place a route can
     live: pcl_port_set_route() (pcl::Port::routeRemote() et al.) writes
     directly onto a concrete port's route_configured/route_mode fields --
     typically how a publisher/provided port gets routed post-bind(),
     never touching e->endpoint_routes at all. A caller asking "is this
     endpoint name routed at all" (e.g. D5 exclusive-group enforcement)
     must see both stores or it misses routes installed this second way. */
  for (i = 0; i < e->container_count; ++i) {
    pcl_container_t* container = e->containers[i];
    uint32_t          pi;
    for (pi = 0; pi < container->port_count; ++pi) {
      const pcl_port_t* port = &container->ports[pi];
      if (port->route_configured && port->route_mode != PCL_ROUTE_NONE &&
          strcmp(port->name, endpoint_name) == 0) {
        return 1;
      }
    }
  }
  return 0;
}

/* No LLR: route-table removal primitive with no requirement of its own;
   used internally by manifest-routing rollback (e.g. REQ_PCL_421,
   REQ_PCL_468) to undo a partially-installed route set. */
pcl_status_t pcl_executor_clear_endpoint_route(pcl_executor_t*     e,
                                               const char*         endpoint_name,
                                               pcl_endpoint_kind_t endpoint_kind) {
  pcl_endpoint_route_entry_t* entry;
  pcl_endpoint_route_entry_t* last;

  if (!e || !endpoint_name) return PCL_ERR_INVALID;
  entry = find_endpoint_route_entry(e, endpoint_name, endpoint_kind);
  if (!entry) return PCL_OK; /* idempotent: nothing installed for this endpoint */

  /* Compact the array so live entries stay contiguous in [0, count): move the
     last live entry into the freed slot and shrink the count. */
  last = &e->endpoint_routes[e->endpoint_route_count - 1u];
  if (entry != last) *entry = *last;
  memset(last, 0, sizeof(*last));
  e->endpoint_route_count--;
  return PCL_OK;
}

/* Implements: REQ_PCL_044, REQ_PCL_046. */
pcl_status_t pcl_executor_dispatch_incoming(pcl_executor_t*  e,
                                            const char*      topic,
                                            const pcl_msg_t* msg) {
  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  return dispatch_incoming_now(e, topic, msg, PCL_ROUTE_LOCAL, NULL);
}

/* Implements: REQ_PCL_040, REQ_PCL_173. */
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

/* Implements: REQ_PCL_170. */
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

/* Implements: REQ_PCL_040. */
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

/* Implements: REQ_PCL_177, REQ_PCL_163. */
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

/* Implements: REQ_PCL_027, REQ_PCL_174. */
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

/* Implements: REQ_PCL_047, REQ_PCL_048. */
pcl_status_t pcl_executor_publish(pcl_executor_t*  e,
                                  const char*      topic,
                                  const pcl_msg_t* msg) {
  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  if (e->has_transport && e->transport.publish) {
    return e->transport.publish(e->transport.adapter_ctx, topic, msg);
  }
  return dispatch_incoming_now(e, topic, msg, PCL_ROUTE_LOCAL, NULL);
}

/* Implements: REQ_PCL_164, REQ_PCL_165, REQ_PCL_166, REQ_PCL_175,
   REQ_PCL_473. */
pcl_status_t pcl_executor_invoke_async(pcl_executor_t*  e,
                                       const char*      service_name,
                                       const pcl_msg_t* request,
                                       pcl_resp_cb_fn_t callback,
                                       void*            user_data) {
  int route_forces_local = 0;

  if (!e || !service_name || !request || !callback) return PCL_ERR_INVALID;

  {
    const pcl_endpoint_route_entry_t* route =
        find_endpoint_route_entry_const(e, service_name, PCL_ENDPOINT_CONSUMED);
    if (route) {
      if (route->route_mode == PCL_ROUTE_LOCAL) {
        /* An explicit LOCAL route is a deliberate override -- it must skip
           the legacy executor-wide transport fallback below, not merely
           "fall through to it because no remote transport is configured
           for this endpoint". Enforced via route_forces_local rather than
           an early return so the intra-process fallback code stays in one
           place. */
        route_forces_local = 1;
      } else if (route->route_mode == PCL_ROUTE_REMOTE) {
        const pcl_transport_t* transport = NULL;
        if (route->peer_count > 1u) {
          pcl_log(NULL, PCL_LOG_ERROR,
                  "service invocation configuration failed for '%s': a consumed "
                  "endpoint may route to at most one peer",
                  service_name);
          return PCL_ERR_INVALID;
        }
        if (route->peer_count == 1u) {
          transport = find_named_transport(e, route->peer_ids[0]);
        } else if (e->has_transport) {
          transport = &e->transport;
        }
        if (!transport || !transport->invoke_async) {
          pcl_log(NULL, PCL_LOG_ERROR,
                  "service invocation configuration failed for '%s': the selected "
                  "remote route has no unary RPC transport",
                  service_name);
          return PCL_ERR_NOT_FOUND;
        }
        return transport->invoke_async(transport->adapter_ctx, service_name,
                                       request, callback, user_data);
      } else {
        pcl_log(NULL, PCL_LOG_ERROR,
                "service invocation configuration failed for '%s': route mode 0x%x is invalid",
                service_name, (unsigned)route->route_mode);
        return PCL_ERR_INVALID;
      }
    }
  }

  // Legacy executor-wide transport fallback (skipped when an explicit
  // LOCAL route exists for this endpoint -- see route_forces_local above).
  if (!route_forces_local && e->has_transport && e->transport.invoke_async) {
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
    ctx = (pcl_svc_context_t*)pcl_calloc(1, sizeof(*ctx));
    if (!ctx) return PCL_ERR_NOMEM;

    ctx->executor  = e;
    ctx->callback  = callback;
    ctx->user_data = user_data;

    rc = port->svc_handler(port->owner, request, &response, ctx, port->svc_user_data);

    if (rc == PCL_OK) {
      // Immediate response — fire callback and free context
      callback(&response, user_data);
      pcl_free(ctx);
    } else if (rc == PCL_PENDING) {
      // Deferred — handler saved ctx, will call pcl_service_respond later
      rc = PCL_OK;
    } else {
      // Error — free context
      pcl_free(ctx);
    }
    return rc;
  }
}

/* Implements: REQ_PCL_170, REQ_PCL_470, REQ_PCL_473. */
pcl_status_t pcl_executor_invoke_stream(pcl_executor_t*        e,
                                        const char*            service_name,
                                        const pcl_msg_t*       request,
                                        pcl_stream_msg_fn_t    callback,
                                        void*                  user_data,
                                        pcl_stream_context_t** out_ctx) {
  int route_forces_local = 0;

  if (!e || !service_name || !request || !callback) return PCL_ERR_INVALID;

  // Per-endpoint route table first, mirroring pcl_executor_invoke_async:
  // a manifest-driven deployment (pcl_transport_routing_load) may route
  // different endpoints to different named transports, so the legacy
  // single executor-wide e->transport below must not be consulted before
  // this per-endpoint route is checked. Looked up under
  // PCL_ENDPOINT_STREAM_CONSUMED, not PCL_ENDPOINT_CONSUMED (unary invoke's
  // kind) -- a streaming client route must require PCL_CAP_RPC_STREAM at
  // compose time (pcl_endpoint_required_caps()), not just PCL_CAP_RPC_UNARY,
  // or a unary-only peer composes successfully and only fails once this
  // call is actually made.
  {
    const pcl_endpoint_route_entry_t* route = find_endpoint_route_entry_const(
        e, service_name, PCL_ENDPOINT_STREAM_CONSUMED);
    if (route) {
      if (route->route_mode == PCL_ROUTE_LOCAL) {
        /* An explicit LOCAL route is a deliberate override -- see the
           identical route_forces_local comment in
           pcl_executor_invoke_async(); it must skip the legacy
           executor-wide transport fallback below, not merely fall through
           to it. */
        route_forces_local = 1;
      } else if (route->route_mode == PCL_ROUTE_REMOTE) {
        const pcl_transport_t* transport = NULL;
        if (route->peer_count > 1u) {
          pcl_log(NULL, PCL_LOG_ERROR,
                  "stream invocation configuration failed for '%s': a consumed "
                  "endpoint may route to at most one peer",
                  service_name);
          return PCL_ERR_INVALID;
        }
        if (route->peer_count == 1u) {
          transport = find_named_transport(e, route->peer_ids[0]);
        } else if (e->has_transport) {
          transport = &e->transport;
        }
        if (!transport || !transport->invoke_stream) {
          pcl_log(NULL, PCL_LOG_ERROR,
                  "stream invocation configuration failed for '%s': the selected "
                  "remote route has no streaming RPC transport",
                  service_name);
          return PCL_ERR_NOT_FOUND;
        }
        {
          void* stream_handle = NULL;
          pcl_status_t rc = transport->invoke_stream(
              transport->adapter_ctx, service_name, request,
              callback, user_data, &stream_handle);
          if (rc == PCL_OK || rc == PCL_STREAMING) {
            if (out_ctx) {
              pcl_stream_context_t* ctx =
                  (pcl_stream_context_t*)pcl_calloc(1, sizeof(*ctx));
              if (ctx) {
                ctx->executor      = e;
                ctx->callback      = callback;
                ctx->user_data     = user_data;
                ctx->transport     = transport;
                ctx->transport_ctx = stream_handle;
                *out_ctx = ctx;
              }
            }
          }
          return rc;
        }
      } else {
        pcl_log(NULL, PCL_LOG_ERROR,
                "stream invocation configuration failed for '%s': route mode 0x%x is invalid",
                service_name, (unsigned)route->route_mode);
        return PCL_ERR_INVALID;
      }
    }
  }

  // Legacy executor-wide transport fallback (skipped when an explicit
  // LOCAL route exists for this endpoint -- see route_forces_local above).
  if (!route_forces_local && e->has_transport && e->transport.invoke_stream) {
    void* stream_handle = NULL;
    pcl_status_t rc = e->transport.invoke_stream(
        e->transport.adapter_ctx, service_name, request,
        callback, user_data, &stream_handle);
    if (rc == PCL_OK || rc == PCL_STREAMING) {
      if (out_ctx) {
        pcl_stream_context_t* ctx = (pcl_stream_context_t*)pcl_calloc(1, sizeof(*ctx));
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
    ctx = (pcl_stream_context_t*)pcl_calloc(1, sizeof(*ctx));
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
      pcl_free(ctx);
      if (out_ctx) *out_ctx = NULL;
      return rc;
    }
  }
}

/* Implements: REQ_PCL_057, REQ_PCL_058. */
pcl_status_t pcl_executor_post_response_cb(pcl_executor_t*  e,
                                           pcl_resp_cb_fn_t cb,
                                           void*            user_data,
                                           const void*      data,
                                           uint32_t         size) {
  pcl_msg_t msg;

  memset(&msg, 0, sizeof(msg));
  msg.data = data;
  msg.size = size;
  msg.type_name = NULL;
  return pcl_executor_post_response_msg(e, cb, user_data, &msg);
}

/* Implements: REQ_PCL_057, REQ_PCL_058, REQ_PCL_060, REQ_PCL_090. */
pcl_status_t pcl_executor_post_response_msg(pcl_executor_t*  e,
                                            pcl_resp_cb_fn_t cb,
                                            void*            user_data,
                                            const pcl_msg_t* msg) {
  pcl_resp_cb_node_t* node;

  if (!e || !cb || !msg) return PCL_ERR_INVALID;

  node = (pcl_resp_cb_node_t*)pcl_calloc(1, sizeof(*node));
  if (!node) return PCL_ERR_NOMEM;

  node->cb        = cb;
  node->user_data = user_data;
  node->size      = msg->size;

  if (msg->size > 0u && msg->data) {
    node->data = pcl_alloc(msg->size);
    if (!node->data) {
      pcl_free(node);
      return PCL_ERR_NOMEM;
    }
    memcpy(node->data, msg->data, msg->size);
  }

  if (msg->type_name) {
    node->type_name = pcl_strdup_local(msg->type_name);
    if (!node->type_name) {
      pcl_free(node->data);
      pcl_free(node);
      return PCL_ERR_NOMEM;
    }
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

/* Implements: REQ_PCL_049, REQ_PCL_050, REQ_PCL_051, REQ_PCL_085,
   REQ_PCL_086, REQ_PCL_087, REQ_PCL_088, REQ_PCL_231, REQ_PCL_232. */
static pcl_status_t enqueue_incoming_message(pcl_executor_t*  e,
                                             const char*      topic,
                                             const pcl_msg_t* msg,
                                             uint32_t         source_route_mode,
                                             const char*      source_peer_id) {
  pcl_pending_msg_t* pending;

  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  if (msg->size > 0u && !msg->data) return PCL_ERR_INVALID;
  if (!msg->type_name) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "incoming message for topic '%s' was rejected: no content type was provided",
            topic);
    return PCL_ERR_INVALID;
  }

  pending = (pcl_pending_msg_t*)pcl_calloc(1, sizeof(pcl_pending_msg_t));
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
    pending->data = pcl_alloc(msg->size);
    if (!pending->data) {
      free_pending_msg(pending);
      return PCL_ERR_NOMEM;
    }
    memcpy(pending->data, msg->data, msg->size);
  }
  pending->size = msg->size;

  pcl_mutex_lock(&e->incoming_lock);
  if (e->incoming_limit != 0u && e->incoming_count >= e->incoming_limit) {
    pcl_mutex_unlock(&e->incoming_lock);
    free_pending_msg(pending);
    return PCL_ERR_NOMEM;
  }
  if (e->incoming_tail) {
    e->incoming_tail->next = pending;
  } else {
    e->incoming_head = pending;
  }
  e->incoming_tail = pending;
  ++e->incoming_count;
  pcl_mutex_unlock(&e->incoming_lock);

  return PCL_OK;
}

/* Implements: REQ_PCL_498. */
pcl_status_t pcl_executor_set_incoming_queue_limit(pcl_executor_t* e,
                                                    uint32_t        limit) {
  if (!e) return PCL_ERR_INVALID;
  pcl_mutex_lock(&e->incoming_lock);
  e->incoming_limit = limit;
  pcl_mutex_unlock(&e->incoming_lock);
  return PCL_OK;
}

/* Implements: REQ_PCL_499. */
uint32_t pcl_executor_get_incoming_queue_depth(pcl_executor_t* e) {
  uint32_t count;
  if (!e) return 0u;
  pcl_mutex_lock(&e->incoming_lock);
  count = e->incoming_count;
  pcl_mutex_unlock(&e->incoming_lock);
  return count;
}

/* Implements: REQ_PCL_049, REQ_PCL_050, REQ_PCL_051, REQ_PCL_053,
   REQ_PCL_056, REQ_PCL_095. */
pcl_status_t pcl_executor_post_incoming(pcl_executor_t*  e,
                                        const char*      topic,
                                        const pcl_msg_t* msg) {
  return enqueue_incoming_message(e, topic, msg, PCL_ROUTE_LOCAL, NULL);
}

/* Implements: REQ_PCL_173, REQ_PCL_201, REQ_PCL_327. */
pcl_status_t pcl_executor_post_remote_incoming(pcl_executor_t*  e,
                                               const char*      peer_id,
                                               const char*      topic,
                                               const pcl_msg_t* msg) {
  if (!peer_id) return PCL_ERR_INVALID;
  return enqueue_incoming_message(e, topic, msg, PCL_ROUTE_REMOTE, peer_id);
}

/* Shared enqueue path for post_service_request and
 * post_service_request_remote.  When source_peer_id is non-NULL, the
 * drained request is dispatched with PCL_ROUTE_REMOTE filtering so
 * remote-exposure rules apply. */
/* Implements: REQ_PCL_180, REQ_PCL_184. */
static pcl_status_t enqueue_svc_req(pcl_executor_t*  e,
                                    const char*      service_name,
                                    const pcl_msg_t* request,
                                    pcl_resp_cb_fn_t callback,
                                    void*            user_data,
                                    const char*      source_peer_id) {
  pcl_pending_svc_req_t* node;

  if (!e || !service_name || !request || !callback) return PCL_ERR_INVALID;

  node = (pcl_pending_svc_req_t*)pcl_calloc(1, sizeof(*node));
  if (!node) return PCL_ERR_NOMEM;

  node->service_name = pcl_strdup_local(service_name);
  if (!node->service_name) {
    pcl_free(node);
    return PCL_ERR_NOMEM;
  }

  if (request->type_name) {
    node->type_name = pcl_strdup_local(request->type_name);
    if (!node->type_name) {
      pcl_free(node->service_name);
      pcl_free(node);
      return PCL_ERR_NOMEM;
    }
  }

  if (request->size > 0u && request->data) {
    node->data = pcl_alloc(request->size);
    if (!node->data) {
      pcl_free(node->service_name);
      pcl_free(node->type_name);
      pcl_free(node);
      return PCL_ERR_NOMEM;
    }
    memcpy(node->data, request->data, request->size);
  }
  node->size      = request->size;
  node->callback  = callback;
  node->user_data = user_data;

  if (source_peer_id) {
    node->source_peer_id = pcl_strdup_local(source_peer_id);
    if (!node->source_peer_id) {
      pcl_free(node->service_name);
      pcl_free(node->type_name);
      pcl_free(node->data);
      pcl_free(node);
      return PCL_ERR_NOMEM;
    }
  }

  pcl_mutex_lock(&e->svc_req_lock);
  if (e->svc_req_tail) {
    e->svc_req_tail->next = node;
  } else {
    e->svc_req_head = node;
  }
  e->svc_req_tail = node;
  pcl_mutex_unlock(&e->svc_req_lock);

  return PCL_OK;
}

/* Implements: REQ_PCL_180, REQ_PCL_183, REQ_PCL_184. */
pcl_status_t pcl_executor_post_service_request(pcl_executor_t*  e,
                                               const char*      service_name,
                                               const pcl_msg_t* request,
                                               pcl_resp_cb_fn_t callback,
                                               void*            user_data) {
  return enqueue_svc_req(e, service_name, request, callback, user_data, NULL);
}

/* Implements: REQ_PCL_438. */
pcl_status_t pcl_executor_post_service_request_remote(pcl_executor_t*  e,
                                                      const char*      source_peer_id,
                                                      const char*      service_name,
                                                      const pcl_msg_t* request,
                                                      pcl_resp_cb_fn_t callback,
                                                      void*            user_data) {
  if (!source_peer_id || !source_peer_id[0]) return PCL_ERR_INVALID;
  return enqueue_svc_req(e, service_name, request, callback, user_data,
                         source_peer_id);
}
