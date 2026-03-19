/// \file pcl_container.h
/// \brief PYRAMID Container Library container API.
///
/// A container encapsulates a single component's business logic behind a
/// lifecycle state machine.  All callbacks execute on the owning executor's
/// single thread — no internal synchronization required. External I/O threads
/// must enqueue ingress through the executor rather than calling callbacks
/// directly.
///
/// Ports (publishers, subscribers, services) must be created during
/// on_configure and are immutable after that point.
#ifndef PCL_CONTAINER_H
#define PCL_CONTAINER_H

#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// -- Callback signatures -------------------------------------------------

/// \brief Subscriber callback — invoked on executor thread when a message arrives.
typedef void (*pcl_sub_callback_t)(pcl_container_t* c,
                                   const pcl_msg_t*  msg,
                                   void*             user_data);

/// \brief Service handler — invoked on executor thread, must populate response.
typedef pcl_status_t (*pcl_service_handler_t)(pcl_container_t* c,
                                              const pcl_msg_t* request,
                                              pcl_msg_t*       response,
                                              void*            user_data);

// -- Lifecycle callbacks (user implements) --------------------------------

/// \brief Lifecycle and tick callbacks provided by the user.
///
/// All callbacks are optional (NULL = no-op).
/// Return PCL_OK to indicate success.  Any other value aborts the
/// transition and leaves the container in its previous state.
typedef struct {
  pcl_status_t (*on_configure)  (pcl_container_t* self, void* user_data);
  pcl_status_t (*on_activate)   (pcl_container_t* self, void* user_data);
  pcl_status_t (*on_deactivate) (pcl_container_t* self, void* user_data);
  pcl_status_t (*on_cleanup)    (pcl_container_t* self, void* user_data);
  pcl_status_t (*on_shutdown)   (pcl_container_t* self, void* user_data);

  /// \brief Periodic tick — called at the container's configured rate while ACTIVE.
  pcl_status_t (*on_tick)(pcl_container_t* self, double dt_seconds,
                          void* user_data);
} pcl_callbacks_t;

// -- Create / destroy ----------------------------------------------------

/// \brief Create a new container in the UNCONFIGURED state.
/// \param name       Human-readable component name (copied).
/// \param callbacks  Lifecycle + tick callbacks (struct is copied).
/// \param user_data  Opaque pointer passed to every callback.
/// \return New container handle, or NULL on allocation failure.
pcl_container_t* pcl_container_create(const char*            name,
                                      const pcl_callbacks_t* callbacks,
                                      void*                  user_data);

/// \brief Destroy a container and free all associated resources.
void pcl_container_destroy(pcl_container_t* c);

// -- Lifecycle transitions -----------------------------------------------

/// \brief Transition UNCONFIGURED → CONFIGURED.
pcl_status_t pcl_container_configure(pcl_container_t* c);

/// \brief Transition CONFIGURED → ACTIVE.
pcl_status_t pcl_container_activate(pcl_container_t* c);

/// \brief Transition ACTIVE → CONFIGURED.
pcl_status_t pcl_container_deactivate(pcl_container_t* c);

/// \brief Transition CONFIGURED → UNCONFIGURED.
pcl_status_t pcl_container_cleanup(pcl_container_t* c);

/// \brief Transition any state → FINALIZED.
pcl_status_t pcl_container_shutdown(pcl_container_t* c);

// -- State query ---------------------------------------------------------

/// \brief Get the current lifecycle state.
pcl_state_t pcl_container_state(const pcl_container_t* c);

/// \brief Get the container name.
const char* pcl_container_name(const pcl_container_t* c);

// -- Tick rate -----------------------------------------------------------

/// \brief Set the container's tick rate in Hz.
///
/// Default is 100 Hz.  Can be called at any time; takes effect on the
/// next tick cycle.
pcl_status_t pcl_container_set_tick_rate_hz(pcl_container_t* c, double hz);

/// \brief Get the container's configured tick rate in Hz.
double pcl_container_get_tick_rate_hz(const pcl_container_t* c);

// -- Parameters (key-value configuration) --------------------------------

/// \brief Set a string parameter.
pcl_status_t pcl_container_set_param_str(pcl_container_t* c,
                                         const char* key, const char* value);

/// \brief Set a double parameter.
pcl_status_t pcl_container_set_param_f64(pcl_container_t* c,
                                         const char* key, double value);

/// \brief Set a 64-bit integer parameter.
pcl_status_t pcl_container_set_param_i64(pcl_container_t* c,
                                         const char* key, int64_t value);

/// \brief Set a boolean parameter.
pcl_status_t pcl_container_set_param_bool(pcl_container_t* c,
                                          const char* key, bool value);

/// \brief Retrieve a string parameter.
/// \return Pointer to internal storage (valid until next set or destroy).
///         Returns default_val if key not found.
const char* pcl_container_get_param_str(const pcl_container_t* c,
                                        const char* key,
                                        const char* default_val);

/// \brief Retrieve a double parameter.
double pcl_container_get_param_f64(const pcl_container_t* c,
                                   const char* key, double default_val);

/// \brief Retrieve a 64-bit integer parameter.
int64_t pcl_container_get_param_i64(const pcl_container_t* c,
                                    const char* key, int64_t default_val);

/// \brief Retrieve a boolean parameter.
bool pcl_container_get_param_bool(const pcl_container_t* c,
                                  const char* key, bool default_val);

// -- Port creation (valid only during on_configure) ----------------------

/// \brief Add a publisher port.
pcl_port_t* pcl_container_add_publisher(pcl_container_t* c,
                                        const char*      topic,
                                        const char*      type_name);

/// \brief Add a subscriber port with a message callback.
pcl_port_t* pcl_container_add_subscriber(pcl_container_t* c,
                                         const char*         topic,
                                         const char*         type_name,
                                         pcl_sub_callback_t  cb,
                                         void*               user_data);

/// \brief Add a service server port with a request handler.
pcl_port_t* pcl_container_add_service(pcl_container_t*      c,
                                      const char*           service_name,
                                      const char*           type_name,
                                      pcl_service_handler_t handler,
                                      void*                 user_data);

// -- Publishing ----------------------------------------------------------

/// \brief Publish a message on a publisher port.  Only valid while ACTIVE.
pcl_status_t pcl_port_publish(pcl_port_t* port, const pcl_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif // PCL_CONTAINER_H
