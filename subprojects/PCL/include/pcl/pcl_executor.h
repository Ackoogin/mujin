/// \file pcl_executor.h
/// \brief PYRAMID Container Library executor API.
///
/// The executor runs one or more containers on a single thread.
/// It drives the tick loop, dispatches incoming messages to subscriber
/// callbacks, and routes service requests to handlers — all sequentially.
///
/// ## Thread Safety Summary
///
/// Most executor functions **must only be called on the executor thread**
/// (the thread that calls pcl_executor_spin / pcl_executor_spin_once).
/// The functions below are the **only** ones safe to call from any thread:
///
///   | Function                              | Any thread safe? |
///   |---------------------------------------|-----------------|
///   | pcl_executor_request_shutdown()       | YES             |
///   | pcl_executor_post_incoming()          | YES             |
///   | pcl_executor_post_remote_incoming()   | YES (transport) |
///   | pcl_executor_post_response_cb()       | YES (transport) |
///   | pcl_executor_post_response_msg()      | YES (transport) |
///   | pcl_executor_post_service_request()   | YES             |
///   | All other functions                   | Executor thread only |
///
/// The three post_response_* functions are intended for transport recv
/// threads returning async call results; pcl_executor_post_incoming and
/// pcl_executor_post_service_request are the general-purpose cross-thread
/// ingress points for subscribers and service callers respectively.
#ifndef PCL_EXECUTOR_H
#define PCL_EXECUTOR_H

#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// -- Create / destroy ----------------------------------------------------

/// \brief Create a new executor.
///
/// \note Executor-thread only once spinning begins.
pcl_executor_t* pcl_executor_create(void);

/// \brief Destroy the executor and free resources.
///
/// Does NOT destroy managed containers — caller retains ownership.
///
/// \note Must not be called while spin() is running.
void pcl_executor_destroy(pcl_executor_t* e);

// -- Container management ------------------------------------------------

/// \brief Add a container to the executor.
///
/// Must be called before spin().  The executor does NOT take ownership —
/// caller is responsible for destroying containers after the executor
/// is destroyed.
///
/// \note Executor-thread only.
pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c);

/// \brief Remove a container from the executor without destroying it.
///
/// Clears the container's back-pointer to the executor and removes it from
/// the internal containers array.  The container is NOT destroyed — caller
/// retains ownership.  Safe to call before pcl_container_destroy when the
/// container was previously added with pcl_executor_add.
///
/// \note Executor-thread only.
pcl_status_t pcl_executor_remove(pcl_executor_t* e, pcl_container_t* c);

// -- Spin ----------------------------------------------------------------

/// \brief Block and run the tick loop for all managed containers.
///
/// Returns PCL_OK when shutdown is requested.
///
/// \note This IS the executor thread for the duration of the call.
pcl_status_t pcl_executor_spin(pcl_executor_t* e);

/// \brief Process one round of pending work then return.
///
/// Drains the incoming message queue, the service request queue, and the
/// response callback queue; then runs one tick per eligible container.
/// \param timeout_ms  Maximum time to wait for pending work (0 = no wait).
///
/// \note This IS the executor thread for the duration of the call.
pcl_status_t pcl_executor_spin_once(pcl_executor_t* e, uint32_t timeout_ms);

// -- Shutdown ------------------------------------------------------------

/// \brief Request the executor to stop spinning.
///
/// \thread-safe  Safe to call from any thread, including signal handlers.
/// Does NOT call lifecycle transitions — containers remain in current state.
void pcl_executor_request_shutdown(pcl_executor_t* e);

/// \brief Graceful shutdown with timeout.
///
/// For each ACTIVE container, calls deactivate then shutdown.
/// If any transition exceeds timeout_ms, forces termination.
/// \param timeout_ms  Maximum wall-clock time for the entire shutdown
///                    sequence (0 = no timeout).
/// \return PCL_OK on clean shutdown, PCL_ERR_TIMEOUT if deadline exceeded.
///
/// \note Executor-thread only (or after spin() has returned).
pcl_status_t pcl_executor_shutdown_graceful(pcl_executor_t* e,
                                            uint32_t        timeout_ms);

// ============================================================
// Cross-thread ingress  (safe to call from ANY thread)
// ============================================================

/// \brief Queue an incoming pub/sub message from an external thread.
///
/// \thread-safe  Safe to call from any thread concurrently with spin().
///
/// Deep-copies the topic, type name, and payload before returning — the
/// caller may release or reuse its buffers immediately.  The message is
/// dispatched to matching \b subscriber callbacks on the executor thread
/// during the next pcl_executor_spin_once / pcl_executor_spin cycle.
///
/// Use this for middleware callbacks, sensor I/O threads, gRPC completions,
/// or any external pub/sub producer.
///
/// \note For service invocations from external threads use
///       pcl_executor_post_service_request() instead — this function only
///       dispatches to subscriber (PCL_PORT_SUBSCRIBER) ports.
pcl_status_t pcl_executor_post_incoming(pcl_executor_t*  e,
                                        const char*      topic,
                                        const pcl_msg_t* msg);

/// \brief Queue an intra-process service call from an external thread.
///
/// \thread-safe  Safe to call from any thread concurrently with spin().
///
/// Deep-copies the service name, type name, and request payload before
/// returning — the caller may release or reuse its buffers immediately.
///
/// On the next spin cycle the executor thread will:
///   1. Locate the named service port (PCL_ROUTE_LOCAL).
///   2. Invoke the service handler on the executor thread.
///   3. Fire \p callback with the response on the executor thread.
///      If the service is not found, \p callback is still fired with an
///      empty (zero-size, NULL-data) message so the caller is not silently
///      abandoned.
///
/// If the handler returns PCL_PENDING (deferred response), it has saved
/// the service context and will fire \p callback later via
/// pcl_service_respond() — still on the executor thread.
///
/// \param service_name  Wire name of the service to invoke.
/// \param request       Request message to deep-copy.
/// \param callback      Response callback — always fires on executor thread.
/// \param user_data     Passed through to \p callback.
///
/// \note Do NOT use pcl_executor_invoke_service() from external threads —
///       that function calls the handler directly on the calling thread,
///       violating the single-threaded execution guarantee (D2/D5).
pcl_status_t pcl_executor_post_service_request(pcl_executor_t*  e,
                                               const char*      service_name,
                                               const pcl_msg_t* request,
                                               pcl_resp_cb_fn_t callback,
                                               void*            user_data);

// ============================================================
// Async response delivery  (safe to call from ANY thread)
// Used by transport recv threads returning remote call results.
// ============================================================

/// \brief Enqueue a service response callback for delivery on the executor thread.
///
/// \thread-safe  Safe to call from any thread (intended for transport recv threads).
///
/// Deep-copies \p data before returning — caller may free immediately.
/// The callback fires during the next pcl_executor_spin_once / pcl_executor_spin.
///
/// \param cb         Callback matching pcl_resp_cb_fn_t.
/// \param user_data  Passed through to \p cb.
/// \param data       Response payload bytes (may be NULL if size == 0).
/// \param size       Payload size in bytes.
pcl_status_t pcl_executor_post_response_cb(pcl_executor_t*  e,
                                           pcl_resp_cb_fn_t cb,
                                           void*            user_data,
                                           const void*      data,
                                           uint32_t         size);

/// \brief Enqueue a full response message for delivery on the executor thread.
///
/// \thread-safe  Safe to call from any thread (intended for transport recv threads).
///
/// Like pcl_executor_post_response_cb(), but preserves \p msg->type_name so
/// async service clients can dispatch on transport-level content type.
///
/// \param cb         Callback matching pcl_resp_cb_fn_t.
/// \param user_data  Passed through to \p cb.
/// \param msg        Response message to deep-copy (may have NULL data/type_name).
pcl_status_t pcl_executor_post_response_msg(pcl_executor_t*  e,
                                            pcl_resp_cb_fn_t cb,
                                            void*            user_data,
                                            const pcl_msg_t* msg);

// ============================================================
// Executor-thread-only dispatch and service invocation
// ============================================================

/// \brief Intra-process service invocation — executor thread only.
///
/// Searches all containers for a service port matching \p service_name and
/// calls the handler synchronously.  Response is written into \p response.
///
/// \note Executor-thread only.  Calling from another thread violates D2/D5.
///       Use pcl_executor_post_service_request() for cross-thread calls.
pcl_status_t pcl_executor_invoke_service(pcl_executor_t*  e,
                                         const char*      service_name,
                                         const pcl_msg_t* request,
                                         pcl_msg_t*       response);

/// \brief Invoke a service request that arrived from a specific remote peer.
///
/// Remote service exposure rules are applied before dispatching to the local
/// handler.
///
/// \note Executor-thread only.
pcl_status_t pcl_executor_invoke_service_remote(pcl_executor_t*  e,
                                                const char*      peer_id,
                                                const char*      service_name,
                                                const pcl_msg_t* request,
                                                pcl_msg_t*       response);

/// \brief Publish a message to a topic (intra-process dispatch).
///
/// Routes the message to subscribers on the topic.
/// If a transport adapter is set, it is used; otherwise direct dispatch.
///
/// \note Executor-thread only.
pcl_status_t pcl_executor_publish(pcl_executor_t*  e,
                                  const char*      topic,
                                  const pcl_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif // PCL_EXECUTOR_H
