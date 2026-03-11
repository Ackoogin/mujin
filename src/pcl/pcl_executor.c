/// \file pcl_executor.c
/// \brief PCL executor implementation — tick loop, dispatch, lifecycle.
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

// ── Internal executor representation ────────────────────────────────────

struct pcl_executor_t {
  pcl_container_t* containers[PCL_MAX_CONTAINERS];
  uint32_t         container_count;

  pcl_transport_t  transport;
  int              has_transport;

  volatile int     shutdown_requested;

  double           prev_time;
};

// ── Create / destroy ────────────────────────────────────────────────────

pcl_executor_t* pcl_executor_create(void) {
  pcl_executor_t* e = (pcl_executor_t*)calloc(1, sizeof(pcl_executor_t));
  if (!e) return NULL;
  e->container_count    = 0;
  e->has_transport      = 0;
  e->shutdown_requested = 0;
  e->prev_time          = 0.0;
  return e;
}

void pcl_executor_destroy(pcl_executor_t* e) {
  if (!e) return;
  free(e);
}

// ── Container management ────────────────────────────────────────────────

pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c) {
  if (!e || !c) return PCL_ERR_INVALID;
  if (e->container_count >= PCL_MAX_CONTAINERS) return PCL_ERR_NOMEM;
  e->containers[e->container_count++] = c;
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
  uint32_t i;

  if (!e) return PCL_ERR_INVALID;
  (void)timeout_ms;

  t_now = pcl_clock_now();
  dt = (e->prev_time > 0.0) ? (t_now - e->prev_time) : 0.001;
  e->prev_time = t_now;

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
  struct pcl_port_t* sub;
  if (!e || !topic || !msg) return PCL_ERR_INVALID;

  sub = find_subscriber(e, topic);
  if (!sub) return PCL_ERR_NOT_FOUND;

  sub->sub_cb(sub->owner, msg, sub->sub_user_data);
  return PCL_OK;
}
