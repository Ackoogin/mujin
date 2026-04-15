/// \file pcl_transport_socket.h
/// \brief TCP socket transport adapter for cross-process PCL.
///
/// Protocol: [4-byte length big-endian][payload]
/// Payload type byte (first byte of payload):
///   Type 0: PUBLISH   - [0x00][topic_len:2][topic][type_len:2][type_name][data_len:4][data]
///   Type 1: SVC_REQ   - [0x01][seq_id:4][svc_len:2][svc_name]
///                          [type_len:2][type_name][req_len:4][req_data]
///   Type 2: SVC_RESP  - [0x02][seq_id:4][type_len:2][type_name][resp_len:4][resp_data]
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

/// \brief Connection state for a socket transport.
typedef enum {
  PCL_SOCKET_STATE_CONNECTING   = 0,  ///< TCP connect in progress or retrying.
  PCL_SOCKET_STATE_CONNECTED    = 1,  ///< TCP session established.
  PCL_SOCKET_STATE_DISCONNECTED = 2   ///< Connection lost (or not yet attempted).
} pcl_socket_state_t;

/// \brief Callback fired when the connection state changes.
///
/// Invoked from the creating thread during initial connect and from
/// recv_thread during auto-reconnect.  Implementations must be
/// thread-safe.
typedef void (*pcl_socket_state_cb_t)(pcl_socket_state_t state,
                                      void*              user_data);

/// \brief Options for pcl_socket_transport_create_client_ex.
///
/// Zero-initialise for legacy single-attempt behaviour.
typedef struct {
  uint32_t              connect_timeout_ms;  ///< Total deadline for initial connect (0 = unlimited).
  uint32_t              max_retries;         ///< Max retry attempts (0 = no retry).
  int                   auto_reconnect;      ///< Non-zero to auto-reconnect after disconnect.
  pcl_socket_state_cb_t state_cb;            ///< Optional state-change callback (may be NULL).
  void*                 state_cb_data;       ///< Opaque pointer passed to state_cb.
} pcl_socket_client_opts_t;

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

/// \brief Like pcl_socket_transport_create_server but exposes the bound port
///        before blocking on accept.
///
/// When \p port is 0 and the OS assigns an ephemeral port, \p port_ready (if
/// non-NULL) is written with the assigned port number immediately after
/// getsockname — before the blocking accept() call.  A concurrent thread can
/// spin on \p port_ready and use the value to call
/// pcl_socket_transport_create_client once it becomes non-zero.
///
/// \param port        TCP port to listen on (0 = pick ephemeral).
/// \param executor    Executor for post_incoming and invoke_service.
/// \param port_ready  Output pointer written before accept() blocks.  May be NULL.
/// \return Handle, or NULL on failure.
pcl_socket_transport_t* pcl_socket_transport_create_server_ex(
    uint16_t           port,
    pcl_executor_t*    executor,
    volatile uint16_t* port_ready);

/// \brief Return the TCP port the transport is bound to.
///
/// For servers created with port=0, returns the ephemeral port assigned by
/// the OS.  For client transports, returns the port passed to create_client.
/// Returns 0 for a NULL handle.
uint16_t pcl_socket_transport_get_port(const pcl_socket_transport_t* ctx);

/// \brief Create a client-mode transport that connects to host:port.
///
/// Single-attempt connect; fails immediately if the server is not
/// listening.  For retry-on-refused / auto-reconnect semantics use
/// \ref pcl_socket_transport_create_client_ex.
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

/// \brief Create a client-mode transport with retry, backoff, and state callbacks.
///
/// The initial connect retries with exponential backoff (100 ms, 200 ms,
/// 400 ms, ..., capped at 2 s) subject to \p opts->max_retries and
/// \p opts->connect_timeout_ms.  If both are zero the behaviour matches
/// \ref pcl_socket_transport_create_client (single attempt).
///
/// When \p opts->auto_reconnect is non-zero, recv_thread transparently
/// reconnects using the same backoff policy whenever the socket drops,
/// so the caller never has to re-create the transport after a remote
/// restart.  Pending outbound frames enqueued while disconnected are
/// dropped (pub/sub semantics); in-flight async service calls never
/// receive a response.
///
/// If \p opts->state_cb is non-NULL it fires on every state transition
/// (CONNECTING → CONNECTED → DISCONNECTED → CONNECTING → ...).
///
/// TCP keepalive is enabled on the socket so silent peer death is
/// detected within a few seconds.
///
/// \param host      Server hostname or IP string.
/// \param port      Server TCP port.
/// \param executor  Executor for post_incoming and post_response_cb.
/// \param opts      Connection options, or NULL for legacy defaults.
/// \return Handle, or NULL on failure (initial connect exhausted retries).
pcl_socket_transport_t* pcl_socket_transport_create_client_ex(
    const char*                     host,
    uint16_t                        port,
    pcl_executor_t*                 executor,
    const pcl_socket_client_opts_t* opts);

/// \brief Query the current connection state.
///
/// Safe to call from any thread.  Returns PCL_SOCKET_STATE_DISCONNECTED
/// for a NULL handle.
pcl_socket_state_t pcl_socket_transport_get_state(const pcl_socket_transport_t* ctx);

/// \brief Set the logical peer identifier used for endpoint routing.
///
/// Use the same identifier when registering the transport with
/// pcl_executor_register_transport().
pcl_status_t pcl_socket_transport_set_peer_id(pcl_socket_transport_t* ctx,
                                              const char*             peer_id);

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
