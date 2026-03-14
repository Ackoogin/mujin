/// \file pcl_executor.h
/// \brief PYRAMID Container Library executor API.
///
/// The executor runs one or more containers on a single thread.
/// It drives the tick loop, dispatches incoming messages to subscriber
/// callbacks, and routes service requests to handlers — all sequentially.
#ifndef PCL_EXECUTOR_H
#define PCL_EXECUTOR_H

#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Create / destroy ────────────────────────────────────────────────────

/// \brief Create a new executor.
pcl_executor_t* pcl_executor_create(void);

/// \brief Destroy the executor and free resources.
///
/// Does NOT destroy managed containers — caller retains ownership.
void pcl_executor_destroy(pcl_executor_t* e);

// ── Container management ────────────────────────────────────────────────

/// \brief Add a container to the executor.
///
/// Must be called before spin().  The executor does NOT take ownership —
/// caller is responsible for destroying containers after the executor
/// is destroyed.
pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c);

// ── Spin ────────────────────────────────────────────────────────────────

/// \brief Block and run the tick loop for all managed containers.
///
/// Returns PCL_OK when shutdown is requested.
pcl_status_t pcl_executor_spin(pcl_executor_t* e);

/// \brief Process one round of pending work then return.
///
/// Drains queued ingress, dispatches incoming messages, and runs one tick
/// per eligible container.
/// \param timeout_ms  Maximum time to wait for pending work (0 = no wait).
pcl_status_t pcl_executor_spin_once(pcl_executor_t* e, uint32_t timeout_ms);

// ── Shutdown ────────────────────────────────────────────────────────────

/// \brief Request the executor to stop spinning.
///
/// Thread-safe and async-signal-safe (uses atomic store).
/// Does NOT call lifecycle transitions — containers remain in current state.
void pcl_executor_request_shutdown(pcl_executor_t* e);

/// \brief Graceful shutdown with timeout.
///
/// For each ACTIVE container, calls deactivate then shutdown.
/// If any transition exceeds timeout_ms, forces termination.
/// \param timeout_ms  Maximum wall-clock time for the entire shutdown
///                    sequence (0 = no timeout).
/// \return PCL_OK on clean shutdown, PCL_ERR_TIMEOUT if deadline exceeded.
pcl_status_t pcl_executor_shutdown_graceful(pcl_executor_t* e,
                                            uint32_t        timeout_ms);

// ── Cross-thread ingress ────────────────────────────────────────────────

/// \brief Queue an incoming message from an external I/O thread.
///
/// Safe to call from threads other than the executor thread. The executor
/// makes a deep copy of the topic, type name, and payload bytes before the
/// call returns, so the caller may release or reuse the original buffer
/// immediately afterwards.
///
/// Queued messages are dispatched on the executor thread during the next
/// \ref pcl_executor_spin_once or \ref pcl_executor_spin cycle.
///
/// Use this function for middleware callbacks, sensor threads, gRPC
/// completions, or other external producers. Use
/// \ref pcl_executor_dispatch_incoming only when already running on the
/// executor thread.
pcl_status_t pcl_executor_post_incoming(pcl_executor_t*  e,
                                        const char*      topic,
                                        const pcl_msg_t* msg);

// ── Intra-process service invocation (for testing / direct call) ────────

/// \brief Invoke a service by name, dispatching to the registered handler.
///
/// Searches all containers for a service port matching the given name.
/// Caller must ensure response->data points to a buffer; handler populates it.
/// Safe to call from the executor thread only.
pcl_status_t pcl_executor_invoke_service(pcl_executor_t*  e,
                                         const char*      service_name,
                                         const pcl_msg_t* request,
                                         pcl_msg_t*       response);

/// \brief Publish a message to a topic (intra-process dispatch).
///
/// When a container calls pcl_port_publish, the port's owner must have been
/// added to an executor. This routes the message to subscribers on the topic.
/// If a transport adapter is set, it is used; otherwise dispatch_incoming.
/// Safe to call from the executor thread only.
pcl_status_t pcl_executor_publish(pcl_executor_t*  e,
                                  const char*      topic,
                                  const pcl_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif // PCL_EXECUTOR_H
