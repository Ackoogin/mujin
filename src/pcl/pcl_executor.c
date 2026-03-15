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

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <time.h>
#  include <errno.h>
#  include <pthread.h>
#endif

// ── Platform time helpers ───────────────────────────────────────────────

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

// ── Limits ──────────────────────────────────────────────────────────────

#define PCL_MAX_CONTAINERS 64

typedef struct pcl_pending_msg_t {
  char*                     topic;
  char*                     type_name;
  void*                     data;
  uint32_t                  size;
  struct pcl_pending_msg_t* next;
} pcl_pending_msg_t;

typedef struct pcl_resp_cb_node_t {
  pcl_resp_cb_fn_t            cb;
  void*                       user_data;
  void*                       data;
  uint32_t                    size;
  struct pcl_resp_cb_node_t*  next;
} pcl_resp_cb_node_t;

#ifdef _WIN32
typedef CRITICAL_SECTION pcl_mutex_t;

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
typedef pthread_mutex_t pcl_mutex_t;

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
  free(pending->data);
  free(pending);
}

// ── Internal executor representation ────────────────────────────────────

struct pcl_executor_t {
  pcl_container_t* containers[PCL_MAX_CONTAINERS];
  uint32_t         container_count;

  pcl_transport_t  transport;
  int              has_transport;

  volatile int     shutdown_requested;

  double           prev_time;

  pcl_pending_msg_t* incoming_head;
  pcl_pending_msg_t* incoming_tail;
  pcl_mutex_t        incoming_lock;

  pcl_resp_cb_node_t* resp_cb_head;
  pcl_resp_cb_node_t* resp_cb_tail;
  pcl_mutex_t         resp_cb_lock;
};

// ── Create / destroy ────────────────────────────────────────────────────

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

// ── Container management ────────────────────────────────────────────────

pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c) {
  if (!e || !c) return PCL_ERR_INVALID;
  if (e->container_count >= PCL_MAX_CONTAINERS) return PCL_ERR_NOMEM;
  e->containers[e->container_count++] = c;
  c->executor = e;
  return PCL_OK;
}

// ── Intra-process dispatch ──────────────────────────────────────────────

static struct pcl_port_t* find_subscriber(pcl_executor_t* e,
                                          const char* topic) {
  uint32_t ci, pi;
  for (ci = 0; ci < e->container_count; ++ci) {
    pcl_container_t* container = e->containers[ci];
    for (pi = 0; pi < container->port_count; ++pi) {
      struct pcl_port_t* port = &container->ports[pi];
      if (port->type == PCL_PORT_SUBSCRIBER &&
          strcmp(port->name, topic) == 0 &&
          port->sub_cb != NULL) {
        return port;
      }
    }
  }
  return NULL;
}

static pcl_status_t dispatch_incoming_now(pcl_executor_t*  e,
                                          const char*      topic,
                                          const pcl_msg_t* msg) {
  struct pcl_port_t* sub;

  if (!e || !topic || !msg) return PCL_ERR_INVALID;

  sub = find_subscriber(e, topic);
  if (!sub) return PCL_ERR_NOT_FOUND;

  sub->sub_cb(sub->owner, msg, sub->sub_user_data);
  return PCL_OK;
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

    rc = dispatch_incoming_now(e, pending->topic, &view);
    free_pending_msg(pending);

    if (rc != PCL_OK) {
      return rc;
    }
  }

  return PCL_OK;
}

// ── Tick one container ──────────────────────────────────────────────────

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

// ── Spin ────────────────────────────────────────────────────────────────

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

// ── Shutdown ────────────────────────────────────────────────────────────

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

// ── Transport ───────────────────────────────────────────────────────────

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

pcl_status_t pcl_executor_dispatch_incoming(pcl_executor_t*  e,
                                            const char*      topic,
                                            const pcl_msg_t* msg) {
  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  return dispatch_incoming_now(e, topic, msg);
}

static struct pcl_port_t* find_service(pcl_executor_t* e, const char* name) {
  uint32_t ci, pi;
  for (ci = 0; ci < e->container_count; ++ci) {
    pcl_container_t* c = e->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      struct pcl_port_t* port = &c->ports[pi];
      if (port->type == PCL_PORT_SERVICE &&
          strcmp(port->name, name) == 0 &&
          port->svc_handler != NULL) {
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

  if (!e || !service_name || !request || !response) return PCL_ERR_INVALID;

  port = find_service(e, service_name);
  if (!port) return PCL_ERR_NOT_FOUND;

  return port->svc_handler(port->owner, request, response, port->svc_user_data);
}

pcl_status_t pcl_executor_publish(pcl_executor_t*  e,
                                  const char*      topic,
                                  const pcl_msg_t* msg) {
  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  if (e->has_transport && e->transport.publish) {
    return e->transport.publish(e->transport.adapter_ctx, topic, msg);
  }
  return dispatch_incoming_now(e, topic, msg);
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

pcl_status_t pcl_executor_post_incoming(pcl_executor_t*  e,
                                        const char*      topic,
                                        const pcl_msg_t* msg) {
  pcl_pending_msg_t* pending;

  if (!e || !topic || !msg) return PCL_ERR_INVALID;
  if (msg->size > 0u && !msg->data) return PCL_ERR_INVALID;
  if (!msg->type_name) return PCL_ERR_INVALID;

  pending = (pcl_pending_msg_t*)calloc(1, sizeof(pcl_pending_msg_t));
  if (!pending) return PCL_ERR_NOMEM;

  pending->topic = pcl_strdup_local(topic);
  pending->type_name = pcl_strdup_local(msg->type_name);

  if (!pending->topic || !pending->type_name) {
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
