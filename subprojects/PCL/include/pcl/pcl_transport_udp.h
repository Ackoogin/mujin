/// \file pcl_transport_udp.h
/// \brief UDP datagram transport adapter for cross-process PCL pub/sub.
///
/// Pub/sub-only transport layered on UDP/IPv4.  Service RPC
/// (invoke_async, respond) and streaming services are NOT supported --
/// use \ref pcl_transport_socket.h (TCP) for those.
///
/// Protocol (single datagram, no length prefix -- UDP preserves boundaries):
///   [1:type=PUBLISH=0x00][2:topic_len][topic]
///   [2:type_len][type_name][4:data_len][data]
///
/// Each transport instance:
///   - binds a local UDP socket (\p local_port, or 0 for ephemeral)
///   - sends published datagrams to a single configured peer (\p remote_host:\p remote_port)
///   - dispatches inbound datagrams to the executor as remote ingress from the configured peer_id
///
/// For many-to-one or one-to-many pub/sub, instantiate one UDP transport
/// per peer and register each one separately via
/// pcl_executor_register_transport().
///
/// Rationale: UDP is appropriate for high-frequency telemetry, sensor
/// feeds, and state broadcasts where best-effort delivery is acceptable
/// and the latency/overhead cost of TCP framing is not.  Service RPC
/// demands reliable delivery and ordered framing, which UDP does not
/// provide; keeping those capabilities exclusive to the TCP transport
/// keeps the UDP adapter simple and predictable.
#ifndef PCL_TRANSPORT_UDP_H
#define PCL_TRANSPORT_UDP_H

#include "pcl/pcl_transport.h"
#include "pcl/pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Opaque UDP transport handle.
typedef struct pcl_udp_transport_t pcl_udp_transport_t;

/// \brief Create a UDP transport bound to \p local_port, targeting \p remote_host:\p remote_port.
///
/// Binds a UDP socket to \p local_port (0 = ephemeral) and records the
/// remote endpoint used by publish().  Spawns a receive thread that
/// decodes PUBLISH datagrams and delivers them to the executor via
/// pcl_executor_post_remote_incoming().
///
/// \param local_port    Local UDP port to bind (0 = OS-assigned ephemeral).
/// \param remote_host   Remote peer hostname or IP string.
/// \param remote_port   Remote peer UDP port.
/// \param executor      Executor for post_remote_incoming().
/// \return Handle, or NULL on failure.  Freed by pcl_udp_transport_destroy().
pcl_udp_transport_t* pcl_udp_transport_create(uint16_t        local_port,
                                              const char*     remote_host,
                                              uint16_t        remote_port,
                                              pcl_executor_t* executor);

/// \brief Return the local UDP port the transport is bound to.
///
/// When created with \p local_port == 0, returns the ephemeral port
/// assigned by the OS.  Returns 0 for a NULL handle.
uint16_t pcl_udp_transport_get_local_port(const pcl_udp_transport_t* ctx);

/// \brief Set the logical peer identifier used for endpoint routing.
///
/// Use the same identifier when registering the transport with
/// pcl_executor_register_transport() and when configuring subscriber
/// or publisher peer lists.
pcl_status_t pcl_udp_transport_set_peer_id(pcl_udp_transport_t* ctx,
                                           const char*          peer_id);

/// \brief Get the transport vtable for pcl_executor_set_transport /
///        pcl_executor_register_transport.
const pcl_transport_t* pcl_udp_transport_get_transport(pcl_udp_transport_t* ctx);

/// \brief Destroy a UDP transport and release all resources.
///
/// Signals the receive thread to stop, closes the socket, joins the
/// thread, and frees the handle.
void pcl_udp_transport_destroy(pcl_udp_transport_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_UDP_H
