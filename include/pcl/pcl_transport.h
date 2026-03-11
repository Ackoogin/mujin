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

// ── Transport adapter vtable ────────────────────────────────────────────

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
  /// and call pcl_executor_dispatch_incoming() when messages arrive.
  /// May be NULL if the adapter only handles outbound traffic.
  pcl_status_t (*subscribe)(void*       adapter_ctx,
                            const char* topic,
                            const char* type_name);

  /// \brief Called once during executor teardown.
  ///
  /// Adapter should release any resources associated with subscriptions
  /// and connections.  May be NULL.
  void (*shutdown)(void* adapter_ctx);

  /// \brief Opaque context owned by the adapter implementation.
  void* adapter_ctx;
} pcl_transport_t;

// ── Executor ↔ transport wiring ─────────────────────────────────────────

/// \brief Set the transport adapter for an executor.
///
/// Must be called before spin().  Pass NULL to revert to the default
/// intra-process adapter.
///
/// The executor does NOT take ownership of the transport struct or its
/// adapter_ctx — caller is responsible for lifetime management.
pcl_status_t pcl_executor_set_transport(pcl_executor_t*        e,
                                        const pcl_transport_t* transport);

/// \brief Dispatch an incoming message to the appropriate subscriber callback.
///
/// Called by the transport adapter when a message arrives from the wire.
/// Must be called on the executor thread.
pcl_status_t pcl_executor_dispatch_incoming(pcl_executor_t*  e,
                                            const char*      topic,
                                            const pcl_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_H
