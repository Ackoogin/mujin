/// \file pcl_transport_socket.h
/// \brief TCP socket transport adapter for cross-process PCL.
///
/// Protocol: [4-byte length big-endian][1-byte type][payload]
/// Type 0: PUBLISH   - [topic_len:2][topic][type_len:2][type_name][data_len:4][data]
/// Type 1: SERVICE_REQ  - [svc_len:2][svc_name][req_len:4][req_data]
/// Type 2: SERVICE_RESP - [resp_len:4][resp_data]
///
/// Server: listens, accepts one client, receives PUBLISH→post_incoming,
/// SERVICE_REQ→gateway invokes and sends response.
/// Client: connects, receives PUBLISH→post_incoming, invoke_remote sends
/// SERVICE_REQ and blocks for SERVICE_RESP.
#ifndef PCL_TRANSPORT_SOCKET_H
#define PCL_TRANSPORT_SOCKET_H

#include "pcl/pcl_transport.h"
#include "pcl/pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Opaque socket transport handle.
typedef struct pcl_socket_transport_t pcl_socket_transport_t;

/// \brief Create a server-mode transport that listens on the given port.
///
/// Caller must add the transport via pcl_executor_set_transport(ctx->transport)
/// and add the gateway via pcl_socket_transport_gateway_container.
///
/// \param port  TCP port to listen on (0 = pick ephemeral).
/// \param executor  Executor for post_incoming and invoke_service.
/// \return Handle, or NULL on failure.  Use pcl_socket_transport_destroy to free.
pcl_socket_transport_t* pcl_socket_transport_create_server(uint16_t        port,
                                                           pcl_executor_t* executor);

/// \brief Create a client-mode transport that connects to host:port.
///
/// Caller must add the transport via pcl_executor_set_transport.
/// Use pcl_socket_transport_invoke_remote for remote service calls.
///
/// \param host  Server hostname or IP.
/// \param port  Server TCP port.
/// \param executor  Executor for post_incoming (topic messages).
/// \return Handle, or NULL on failure.
pcl_socket_transport_t* pcl_socket_transport_create_client(const char*      host,
                                                           uint16_t        port,
                                                           pcl_executor_t* executor);

/// \brief Get the transport adapter for pcl_executor_set_transport.
const pcl_transport_t* pcl_socket_transport_get_transport(pcl_socket_transport_t* ctx);

/// \brief Get the gateway container for server mode.
///
/// The gateway subscribes to __pcl_svc_req, invokes the service, and sends
/// the response.  Must be added to the executor when using server transport.
pcl_container_t* pcl_socket_transport_gateway_container(pcl_socket_transport_t* ctx);

/// \brief Invoke a remote service (client mode only).
///
/// Sends SERVICE_REQ and blocks until SERVICE_RESP.  Call from executor thread.
///
/// \param ctx  Handle from pcl_socket_transport_create_client.
/// \param service_name  Service name (e.g. "subscribe_interest").
/// \param request   Request message.
/// \param response  Response buffer (caller allocates data, handler populates).
/// \return PCL_OK on success.
pcl_status_t pcl_socket_transport_invoke_remote(pcl_socket_transport_t* ctx,
                                                const char*             service_name,
                                                const pcl_msg_t*        request,
                                                pcl_msg_t*              response);

/// \brief Destroy a socket transport and release resources.
void pcl_socket_transport_destroy(pcl_socket_transport_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_SOCKET_H
