/// \file wm_container_example.c
/// \brief Example: WorldModel reimagined as a PCL container.
///
/// Demonstrates the full PCL lifecycle: create container, configure with
/// parameters and ports, activate, tick, then graceful shutdown.
///
/// Build:
///   cmake --build build --target wm_container_example
///
/// Run:
///   ./build/examples/wm_container_example
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"

#include <stdio.h>
#include <string.h>

// -- Application state ---------------------------------------------------

typedef struct {
  int        tick_count;
  bool       state_dirty;
  pcl_port_t* pub_state;
  pcl_port_t* srv_get_fact;
} WmData;

// -- Service handler -----------------------------------------------------

static pcl_status_t handle_get_fact(pcl_container_t* c,
                                    const pcl_msg_t* request,
                                    pcl_msg_t*       response,
                                    pcl_svc_context_t* ctx,
                                    void*            ud) {
  (void)ctx;
  (void)request;
  WmData* d = (WmData*)ud;
  pcl_log(c, PCL_LOG_INFO, "get_fact service called (tick %d)",
          d->tick_count);

  // echo back a dummy response
  static const char reply[] = "fact_value=true";
  response->data      = reply;
  response->size      = (uint32_t)strlen(reply);
  response->type_name = "GetFactResponse";
  return PCL_OK;
}

// -- Lifecycle callbacks -------------------------------------------------

static pcl_status_t wm_on_configure(pcl_container_t* c, void* ud) {
  WmData* d = (WmData*)ud;

  const char* domain = pcl_container_get_param_str(c, "domain.pddl_file", "");
  pcl_log(c, PCL_LOG_INFO, "domain file: %s",
          domain[0] ? domain : "(none)");

  d->pub_state = pcl_container_add_publisher(c, "world_state", "WorldState");
  if (!d->pub_state) return PCL_ERR_CALLBACK;

  d->srv_get_fact = pcl_container_add_service(
      c, "get_fact", "GetFact", handle_get_fact, ud);
  if (!d->srv_get_fact) return PCL_ERR_CALLBACK;

  return PCL_OK;
}

static pcl_status_t wm_on_activate(pcl_container_t* c, void* ud) {
  WmData* d = (WmData*)ud;
  d->tick_count  = 0;
  d->state_dirty = true;
  pcl_log(c, PCL_LOG_INFO, "world model active");
  return PCL_OK;
}

static pcl_status_t wm_on_tick(pcl_container_t* c, double dt, void* ud) {
  WmData* d = (WmData*)ud;
  d->tick_count++;

  if (d->state_dirty) {
    // publish world state
    const char* state_data = "wm_version=1";
    pcl_msg_t msg;
    msg.data      = state_data;
    msg.size      = (uint32_t)strlen(state_data);
    msg.type_name = "WorldState";
    pcl_port_publish(d->pub_state, &msg);

    pcl_log(c, PCL_LOG_INFO, "published world state (tick %d, dt=%.3fs)",
            d->tick_count, dt);
    d->state_dirty = false;
  }

  // stop after 5 ticks for the demo
  if (d->tick_count >= 5) {
    pcl_log(c, PCL_LOG_INFO, "demo complete after %d ticks", d->tick_count);
    return PCL_ERR_CALLBACK; // signal to stop
  }

  return PCL_OK;
}

static pcl_status_t wm_on_deactivate(pcl_container_t* c, void* ud) {
  (void)ud;
  pcl_log(c, PCL_LOG_INFO, "world model deactivated");
  return PCL_OK;
}

static pcl_status_t wm_on_shutdown(pcl_container_t* c, void* ud) {
  (void)ud;
  pcl_log(c, PCL_LOG_INFO, "world model shutdown");
  return PCL_OK;
}

// -- Main ----------------------------------------------------------------

int main(void) {
  printf("=== PCL WorldModel Container Example ===\n\n");

  WmData data;
  memset(&data, 0, sizeof(data));

  pcl_callbacks_t cbs;
  memset(&cbs, 0, sizeof(cbs));
  cbs.on_configure  = wm_on_configure;
  cbs.on_activate   = wm_on_activate;
  cbs.on_tick       = wm_on_tick;
  cbs.on_deactivate = wm_on_deactivate;
  cbs.on_shutdown   = wm_on_shutdown;

  pcl_container_t* wm = pcl_container_create("world_model", &cbs, &data);
  pcl_container_set_param_str(wm, "domain.pddl_file",
                               "domains/uav_search/domain.pddl");
  pcl_container_set_tick_rate_hz(wm, 10.0);

  // lifecycle: configure → activate
  pcl_container_configure(wm);
  pcl_container_activate(wm);

  // run in executor
  pcl_executor_t* exec = pcl_executor_create();
  pcl_executor_add(exec, wm);

  // spin — will stop when on_tick returns error after 5 ticks
  // (in production, use pcl_executor_request_shutdown from a signal handler)
  for (int i = 0; i < 50; i++) {
    if (pcl_executor_spin_once(exec, 0) != PCL_OK) break;
    if (data.tick_count >= 5) break;
  }

  // graceful shutdown
  pcl_executor_shutdown_graceful(exec, 5000);

  printf("\n=== Done ===\n");

  pcl_executor_destroy(exec);
  pcl_container_destroy(wm);
  return 0;
}
