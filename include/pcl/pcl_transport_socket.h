/// \file pcl_transport_socket.h
/// \brief TCP socket transport adapter for cross-process PCL.
///
/// Protocol: [4-byte length big-endian][payload]
/// Payload type byte (first byte of payload):
///   Type 0: PUBLISH   - [0x00][topic_len:2][topic][type_len:2][type_name][data_len:4][data]
///   Type 1: SVC_REQ   - [0x01][seq_id:4][svc_len:2][svc_name][req_len:4][req_data]
///   Type 2: SVC_RESP  - [0x02][seq_id:4][resp_len:4][resp_data]
///
/// Server: listens, accepts one client, recv_thread posts PUBLISH → post_incoming,
///   SVC_REQ → gateway container invokes service and enqueues SVC_RESP to send_thread.
/// Client: connects, recv_thread posts PUBLISH → post_incoming,
///   SVC_RESP → post_response_cb on executor thread.
///   invoke_remote_async enqueues SVC_REQ to send_thread (non-blocking).
///
/// ALL socket writes are performed exclusively by the dedicated send_thread
/// via a mutex-protected FIFO queue.  No blocking I/O occurs on the PCL
/// executor (main) thread.
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
/// Blocks until one client connects, then spawns recv_thread and send_thread.
/// Caller must:
///   1. pcl_executor_set_transport(exec, pcl_socket_transport_get_transport(t))
///   2. pcl_executor_add(exec, pcl_socket_transport_gateway_container(t))
///      after configuring and activating the gateway container.
///
/// \param port      TCP port to listen on (0 = pick ephemeral).
/// \param executor  Executor for post_incoming and invoke_service.
/// \return Handle, or NULL on failure.  Freed by pcl_socket_transport_destroy.
pcl_socket_transport_t* pcl_socket_transport_create_server(uint16_t        port,
                                                           pcl_executor_t* executor);

/// \brief Create a client-mode transport that connects to host:port.
///
/// Spawns recv_thread and send_thread on success.
/// Caller must pcl_executor_set_transport after creation.
///
/// \param host      Server hostname or IP string.
/// \param port      Server TCP port.
/// \param executor  Executor for post_incoming and post_response_cb.
/// \return Handle, or NULL on failure.
pcl_socket_transport_t* pcl_socket_transport_create_client(const char*      host,
                                                           uint16_t         port,
                                                           pcl_executor_t*  executor);

/// \brief Get the transport vtable for pcl_executor_set_transport.
const pcl_transport_t* pcl_socket_transport_get_transport(pcl_socket_transport_t* ctx);

/// \brief Get the gateway container (server mode only).
///
/// Subscribes to __pcl_svc_req, invokes the registered service handler, and
/// enqueues the SVC_RESP frame to the send_thread.
/// Must be configured, activated, and added to the executor before spinning.
pcl_container_t* pcl_socket_transport_gateway_container(pcl_socket_transport_t* ctx);

/// \brief Invoke a remote service asynchronously (client mode only).
///
/// Enqueues a SVC_REQ frame to the send_thread and returns immediately.
/// \p callback is invoked on the executor thread (during spin_once/spin)
/// when the SVC_RESP arrives.  Never blocks the caller.
///
/// \param ctx          Handle from pcl_socket_transport_create_client.
/// \param service_name Remote service name.
/// \param request      Request message (deep-copied before return).
/// \param callback     Fired on executor thread with the response.
/// \param user_data    Passed through to \p callback.
/// \return PCL_OK if the request was successfully enqueued.
pcl_status_t pcl_socket_transport_invoke_remote_async(
    pcl_socket_transport_t* ctx,
    const char*             service_name,
    const pcl_msg_t*        request,
    pcl_resp_cb_fn_t        callback,
    void*                   user_data);

/// \brief Destroy a socket transport and release all resources.
///
/// Signals send_thread to drain and exit, closes socket (unblocks recv_thread),
/// joins both threads, frees pending async calls.
void pcl_socket_transport_destroy(pcl_socket_transport_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_SOCKET_H
