/// \file pcl_transport.h
/// \brief PYRAMID Container Library transport adapter interface.
///
/// The transport adapter connects container ports to real middleware
/// (ROS2, DDS, sockets, shared memory, etc.).  If no adapter is set,
/// the executor uses intra-process direct dispatch (zero-copy pointer
/// handoff between containers in the same executor).
#ifndef PCL_TRANSPORT_H
#define PCL_TRANSPORT_H

#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// -- Transport adapter vtable --------------------------------------------

/// \brief Function pointers that a transport adapter implements.
///
/// All functions are called on the executor thread.
typedef struct {
  /// \brief Called when a container publishes a message.
  ///
  /// The adapter should serialize and send it over the wire.
  /// For intra-process, the default adapter forwards directly.
  pcl_status_t (*publish)(void*            adapter_ctx,
                          const char*      topic,
                          const pcl_msg_t* msg);

  /// \brief Called when a service request arrives from the wire.
  ///
  /// The adapter should deserialize, dispatch to the container's handler,
  /// then serialize and return the response.
  /// For intra-process, the default adapter calls the handler directly.
  pcl_status_t (*serve)(void*            adapter_ctx,
                        const char*      service_name,
                        const pcl_msg_t* request,
                        pcl_msg_t*       response);

  /// \brief Called once during executor setup for each subscriber port.
  ///
  /// The adapter should begin listening for messages on the given topic
/// and either:
/// - call pcl_executor_post_incoming() from an external I/O thread, or
/// - call pcl_executor_dispatch_incoming() when already on the executor
///   thread.
///
/// May be NULL if the adapter only handles outbound traffic.
  pcl_status_t (*subscribe)(void*       adapter_ctx,
                            const char* topic,
                            const char* type_name);

  /// \brief Called when a container invokes a remote service asynchronously.
  ///
  /// The adapter should serialize and send the request over the wire.
  /// When the response arrives, the adapter calls the callback on the
  /// executor thread (via pcl_executor_post_response_cb or directly if
  /// already on that thread).
  ///
  /// May be NULL if the adapter does not support client-side service calls.
  /// Intra-process adapters may dispatch synchronously and invoke the
  /// callback before returning.
  pcl_status_t (*invoke_async)(void*            adapter_ctx,
                               const char*      service_name,
                               const pcl_msg_t* request,
                               pcl_resp_cb_fn_t callback,
                               void*            user_data);

  /// \brief Called to send a deferred service response.
  ///
  /// The adapter should serialize and send the response back to the original
  /// caller.  For intra-process, this is a no-op (response sent directly).
  /// May be NULL if the adapter does not support deferred responses.
  ///
  /// \param adapter_ctx  Adapter context.
  /// \param svc_ctx      Service context from the original request.
  /// \param response     Response message to send.
  pcl_status_t (*respond)(void*              adapter_ctx,
                          pcl_svc_context_t* svc_ctx,
                          const pcl_msg_t*   response);

  /// \brief Called once during executor teardown.
  ///
  /// Adapter should release any resources associated with subscriptions
  /// and connections.  May be NULL.
  void (*shutdown)(void* adapter_ctx);

  /// \brief Called to invoke a streaming service (client-side).
  ///
  /// The adapter should serialize and send the request, then call the callback
  /// for each response message.  The callback receives end=true on the final
  /// message, or an error status if the stream is aborted.
  ///
  /// May be NULL if the adapter does not support streaming.
  pcl_status_t (*invoke_stream)(void*               adapter_ctx,
                                const char*         service_name,
                                const pcl_msg_t*    request,
                                pcl_stream_msg_fn_t callback,
                                void*               user_data,
                                void**              stream_handle);

  /// \brief Called to send a stream message (server-side).
  ///
  /// The adapter should serialize and send the message to the client.
  /// May be NULL if the adapter does not support streaming.
  pcl_status_t (*stream_send)(void*            adapter_ctx,
                              void*            stream_handle,
                              const pcl_msg_t* msg);

  /// \brief Called to end or abort a stream (server-side).
  ///
  /// If status is PCL_OK, the stream ended normally.  Otherwise, the stream
  /// was aborted with the given error code.
  /// May be NULL if the adapter does not support streaming.
  pcl_status_t (*stream_end)(void*        adapter_ctx,
                             void*        stream_handle,
                             pcl_status_t status);

  /// \brief Called to cancel an in-flight stream (client-side).
  ///
  /// The adapter should notify the server that the stream is cancelled.
  /// May be NULL if the adapter does not support streaming.
  pcl_status_t (*stream_cancel)(void* adapter_ctx,
                                void* stream_handle);

  /// \brief Opaque context owned by the adapter implementation.
  void* adapter_ctx;
} pcl_transport_t;

// -- Executor <-> transport wiring -----------------------------------------

/// \brief Set the transport adapter for an executor.
///
/// Must be called before spin().  Pass NULL to revert to the default
/// intra-process adapter.
///
/// The executor does NOT take ownership of the transport struct or its
/// adapter_ctx -- caller is responsible for lifetime management.
pcl_status_t pcl_executor_set_transport(pcl_executor_t*        e,
                                        const pcl_transport_t* transport);

/// \brief Register a named peer transport on an executor.
///
/// Multiple peer transports may be registered on the same executor. Endpoint
/// routes can then target a specific \p peer_id.
pcl_status_t pcl_executor_register_transport(pcl_executor_t*        e,
                                             const char*            peer_id,
                                             const pcl_transport_t* transport);

/// \brief Get the currently configured default transport adapter.
///
/// Returns the vtable pointer last passed to
/// pcl_executor_set_transport(), or NULL if no default transport is
/// installed.  Intended for transport adapters that need to check
/// whether they are the active default before clearing it during
/// teardown -- destroy() must not blindly wipe an unrelated
/// transport that another adapter installed.
const pcl_transport_t* pcl_executor_get_transport(const pcl_executor_t* e);

/// \brief Get the named-peer transport registered for a specific peer ID.
///
/// Returns the vtable pointer registered for \p peer_id via
/// pcl_executor_register_transport(), or NULL if no transport is
/// registered for that peer.  Intended for transport adapters that need
/// to verify they still own a peer slot before clearing it during
/// teardown -- an alias may have been rebound to a different adapter
/// since it was first registered, and destroy() must not wipe a slot
/// it no longer owns.
const pcl_transport_t* pcl_executor_get_transport_for_peer(
    const pcl_executor_t* e,
    const char*           peer_id);

/// \brief Dispatch an incoming message to the appropriate subscriber callback.
///
/// Called by the transport adapter when a message arrives from the wire and
/// the adapter is already executing on the executor thread.
///
/// For external I/O threads, use \ref pcl_executor_post_incoming instead.
pcl_status_t pcl_executor_dispatch_incoming(pcl_executor_t*  e,
                                            const char*      topic,
                                            const pcl_msg_t* msg);

/// \brief Queue an incoming remote message from a named peer.
///
/// Like \ref pcl_executor_post_incoming, but the queued message is marked as
/// remote ingress from \p peer_id so subscriber exposure rules can filter it.
pcl_status_t pcl_executor_post_remote_incoming(pcl_executor_t*  e,
                                               const char*      peer_id,
                                               const char*      topic,
                                               const pcl_msg_t* msg);

/// \brief Invoke a service asynchronously through the configured transport.
///
/// Routes the request through the transport adapter's invoke_async function.
/// Returns PCL_ERR_NOT_FOUND if no transport is set or the transport does not
/// support client-side service invocation (invoke_async is NULL).
///
/// The callback is fired on the executor thread when the response arrives.
pcl_status_t pcl_executor_invoke_async(pcl_executor_t*  e,
                                       const char*      service_name,
                                       const pcl_msg_t* request,
                                       pcl_resp_cb_fn_t callback,
                                       void*            user_data);

/// \brief Configure a per-endpoint route on an executor.
///
/// This is primarily used for consumed/client endpoints where there is no
/// concrete server port object to attach a route to.
pcl_status_t pcl_executor_set_endpoint_route(pcl_executor_t*         e,
                                             const pcl_endpoint_route_t* route);

/// \brief Invoke a streaming service through the configured transport.
///
/// Routes the request through the transport adapter's invoke_stream function,
/// or uses intra-process dispatch if no transport is set.
///
/// The callback is fired for each response message; end=true on the final one.
///
/// \param e            Executor to invoke through.
/// \param service_name Target streaming service name.
/// \param request      Request message.
/// \param callback     Fired for each stream message.
/// \param user_data    Passed through to callback.
/// \param out_ctx      Receives stream context for cancellation (may be NULL).
/// \return PCL_OK on success, error code on failure.
pcl_status_t pcl_executor_invoke_stream(pcl_executor_t*       e,
                                        const char*           service_name,
                                        const pcl_msg_t*      request,
                                        pcl_stream_msg_fn_t   callback,
                                        void*                 user_data,
                                        pcl_stream_context_t** out_ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_H
