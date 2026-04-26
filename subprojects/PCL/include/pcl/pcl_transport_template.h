/// \file pcl_transport_template.h
/// \brief Template/skeleton transport adapter for PCL.
///
/// This header defines a ready-to-use scaffold that an engineer can copy
/// (or instantiate directly) when integrating a new wire protocol —
/// serial, custom RPC, vendor middleware, mock, etc. — into the PCL
/// executor.  The scaffold provides the **threading model** and the
/// **lifecycle plumbing**; the engineer only fills in four blocking
/// I/O hooks via \ref pcl_template_io_hooks_t.
///
/// ## Why a template?
///
/// Every PCL transport (TCP socket, UDP datagram, shared-memory bus)
/// repeats the same structure:
///
///   1. A dedicated **send_thread** drains a FIFO of outbound frames so
///      the executor never blocks on I/O.
///   2. A dedicated **recv_thread** loops on a blocking read and posts
///      decoded frames into the executor's ingress queue with the
///      thread-safe helpers in `pcl_executor.h`.
///   3. The vtable callbacks (publish, subscribe, shutdown) on the
///      executor thread do nothing more than enqueue / unblock work.
///
/// This template captures that structure once.  The engineer plugs in
/// their wire protocol by implementing the hooks; the threading and
/// lifetime guarantees come for free.
///
/// ## Threading model (preserved by the template)
///
/// | Thread          | What it does                                          |
/// |-----------------|-------------------------------------------------------|
/// | Executor thread | publish() / subscribe() vtable calls — never blocks   |
/// | send_thread     | Calls hooks.send_blocking() — blocks freely           |
/// | recv_thread     | Calls hooks.recv_blocking() — blocks freely           |
///
/// The engineer's hook implementations **may block as long as needed**
/// (synchronous send, recv with read timeout, etc.).  They run on the
/// dedicated worker threads — never on the PCL executor (D2/D5 single
/// -threaded execution guarantee is preserved).
///
/// To deliver inbound frames back into the executor, the recv_thread
/// uses only the thread-safe ingress functions documented in
/// `pcl_executor.h` (post_remote_incoming).  No locks need to be taken
/// by the engineer's code; the template owns all synchronisation.
///
/// ## Capabilities
///
/// The template implements **pub/sub and async service RPC** end-to-end:
///
///   - Publish / subscribe — every PUBLISH frame is delivered to the
///     executor as remote ingress from the configured peer.
///   - Service client (`invoke_async`) — outbound SVC_REQ frames are
///     correlated by a 32-bit sequence id and routed back to the
///     caller's response callback when the matching SVC_RESP arrives.
///   - Service server — inbound SVC_REQ frames are dispatched into the
///     executor with `pcl_executor_post_service_request`; the handler's
///     response is enqueued back as a SVC_RESP frame.
///
/// Streaming services are out of scope.  Extend by following the
/// correlation pattern already present for SVC_REQ/RESP in
/// `pcl_transport_template.c`.
///
/// ## Engineer checklist
///
///   1. Implement the four hooks in \ref pcl_template_io_hooks_t.
///      `send_blocking` and `recv_blocking` are required; `open` and
///      `close` are optional (pass NULL if unused).
///   2. Allocate a \ref pcl_template_io_hooks_t, fill it in, and pass
///      it to \ref pcl_transport_template_create.
///   3. Wire the returned vtable into the executor with
///      `pcl_executor_set_transport()` (or `register_transport()` for
///      multi-peer setups).
///   4. Call `pcl_transport_template_destroy` during teardown.  The
///      template signals stop, calls `hooks.wake` (if provided) so any
///      in-flight blocking recv unblocks promptly, joins both worker
///      threads, then calls `hooks.close`.
///
/// See `subprojects/PCL/src/pcl_transport_template.c` for the reference
/// scaffold; the inline `// TODO(engineer):` markers point at the four
/// places where wire-protocol logic belongs.
#ifndef PCL_TRANSPORT_TEMPLATE_H
#define PCL_TRANSPORT_TEMPLATE_H

#include "pcl/pcl_transport.h"
#include "pcl/pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// -- Frame model --------------------------------------------------------

/// \brief Wire-frame kinds carried by the template transport.
///
/// The template only routes \ref PCL_TEMPLATE_FRAME_PUBLISH frames
/// today; the additional kinds are reserved so the engineer can extend
/// the template to RPC without having to rev the public ABI.
typedef enum {
  PCL_TEMPLATE_FRAME_PUBLISH  = 0,  ///< Pub/sub message.
  PCL_TEMPLATE_FRAME_SVC_REQ  = 1,  ///< Reserved — service request (extension point).
  PCL_TEMPLATE_FRAME_SVC_RESP = 2,  ///< Reserved — service response (extension point).
} pcl_template_frame_kind_t;

/// \brief One framed message exchanged between the template and the engineer's hooks.
///
/// On the **send path** (executor → wire) the template owns every
/// pointer and guarantees they remain valid for the duration of the
/// `send_blocking` call.  Hooks must treat the fields as borrowed and
/// must not retain pointers after returning.
///
/// On the **recv path** (wire → executor) the engineer's
/// `recv_blocking` hook fills the struct.  Strings (`topic`,
/// `type_name`) and the payload buffer must be heap-allocated with
/// `malloc`; the template `free()`s them after delivery to the
/// executor.  If a field is unused (e.g. `type_name` for an untyped
/// payload) leave it NULL.
typedef struct {
  pcl_template_frame_kind_t kind;          ///< Frame kind — PUBLISH, SVC_REQ, SVC_RESP.
  uint32_t                  seq_id;        ///< Correlates SVC_REQ with SVC_RESP (0 for PUBLISH).
  const char*               topic;         ///< NUL-terminated topic name (PUBLISH only).
  const char*               service_name;  ///< NUL-terminated service name (SVC_REQ only).
  const char*               type_name;     ///< NUL-terminated message type name, or NULL.
  const void*               payload;       ///< Payload bytes, or NULL when payload_size == 0.
  uint32_t                  payload_size;  ///< Payload size in bytes.
} pcl_template_frame_t;

// -- Engineer-supplied I/O hooks ----------------------------------------

/// \brief Engineer-supplied blocking I/O hooks.
///
/// Populate the function pointers, set `user_data` to whatever your
/// implementation needs (socket handle, serial fd, etc.) and pass the
/// struct to \ref pcl_transport_template_create.  The template makes
/// an internal copy — the struct itself does not need to outlive the
/// create() call, but anything reachable through `user_data` must.
///
/// All hooks are invoked **off the executor thread**; the template's
/// own locks have been released before each invocation, so hooks may
/// block freely.
typedef struct {
  /// \brief Opaque pointer forwarded to every hook.
  void* user_data;

  /// \brief Establish the underlying session.  Optional (may be NULL).
  ///
  /// Called once during \ref pcl_transport_template_create on the
  /// **creating thread** — before the worker threads start.  If the
  /// hook returns anything other than `PCL_OK`, create() reports
  /// failure and no threads are spawned.
  ///
  /// Use this for connecting sockets, opening serial ports, joining
  /// busses, etc.  Anything that blocks here delays `create()` only.
  pcl_status_t (*open)(void* user_data);

  /// \brief Tear down the underlying session.  Optional (may be NULL).
  ///
  /// Called once during \ref pcl_transport_template_destroy, **after**
  /// both worker threads have been joined and `wake` has been invoked.
  /// At this point no other hook is in flight; release session-owned
  /// resources here.
  void (*close)(void* user_data);

  /// \brief Push one frame onto the wire.  Required.
  ///
  /// Invoked on the dedicated **send_thread** for every frame the
  /// executor enqueues.  May block until the bytes are accepted by the
  /// underlying transport (TCP socket, fwrite, vendor send(), ...).
  ///
  /// On error, return any negative `pcl_status_t`; the template logs
  /// the failure and drops the frame (pub/sub best-effort semantics).
  /// The send_thread keeps running so subsequent frames can still
  /// flow once the underlying medium recovers.
  ///
  /// \param user_data  Opaque pointer from the hooks struct.
  /// \param frame      Borrowed frame; do NOT retain pointers.
  /// \return PCL_OK on success, negative pcl_status_t on failure.
  pcl_status_t (*send_blocking)(void*                       user_data,
                                const pcl_template_frame_t* frame);

  /// \brief Pull the next frame from the wire.  Required.
  ///
  /// Invoked repeatedly on the dedicated **recv_thread**.  The hook
  /// must block for *at most* `timeout_ms` milliseconds and then
  /// return so the recv loop can poll its stop flag.  A short timeout
  /// (e.g. 100–500 ms) gives the engineer's stack a chance to honour
  /// shutdown requests promptly.
  ///
  /// On success: set `*out_frame` and return `PCL_OK`.  Strings and
  /// payload must be heap-allocated with `malloc`; the template owns
  /// them from this point on and frees them after dispatch.
  ///
  /// On clean timeout: return `PCL_ERR_TIMEOUT` (no allocation needed).
  /// The recv loop will simply re-enter the hook.
  ///
  /// On any other error: return any other negative `pcl_status_t`.
  /// The recv loop logs and continues, so transient I/O errors do not
  /// kill the transport.  Permanent failure should be signalled by
  /// `wake()` returning the loop and letting `recv_blocking` fail
  /// every call thereafter — the loop ends when destroy() runs.
  ///
  /// \param user_data   Opaque pointer from the hooks struct.
  /// \param out_frame   Frame to populate on success.
  /// \param timeout_ms  Upper bound on the call's blocking time.
  /// \return PCL_OK on success, PCL_ERR_TIMEOUT on idle, other on error.
  pcl_status_t (*recv_blocking)(void*                 user_data,
                                pcl_template_frame_t* out_frame,
                                uint32_t              timeout_ms);

  /// \brief Unblock an in-flight `recv_blocking` early.  Optional.
  ///
  /// Called from the destroying thread after the stop flag is set.
  /// A typical implementation closes the underlying file descriptor or
  /// signals an internal eventfd so a blocking `recv` returns
  /// immediately.  If the hook is NULL, the recv_thread relies on the
  /// `timeout_ms` polling interval instead, which means destroy() may
  /// take up to one timeout to return.
  void (*wake)(void* user_data);
} pcl_template_io_hooks_t;

// -- Public API ---------------------------------------------------------

/// \brief Opaque template transport handle.
typedef struct pcl_transport_template_t pcl_transport_template_t;

/// \brief Create a template transport bound to an executor.
///
/// On success, the function:
///
///   1. Calls `hooks.open` on the calling thread (if provided).
///   2. Spawns the **send_thread** and **recv_thread**.
///   3. Returns a handle that the caller wires into the executor with
///      `pcl_executor_set_transport()` /
///      `pcl_executor_register_transport()`.
///
/// The handle is freed by \ref pcl_transport_template_destroy.
///
/// \param hooks     Engineer-supplied I/O hooks.  `send_blocking` and
///                  `recv_blocking` are required.
/// \param executor  Executor that will receive inbound frames.  Must
///                  remain valid until the template is destroyed.
/// \return Handle on success, NULL on failure (missing required hook,
///         `hooks.open` failed, or thread creation failed).
pcl_transport_template_t* pcl_transport_template_create(
    const pcl_template_io_hooks_t* hooks,
    pcl_executor_t*                executor);

/// \brief Set the logical peer identifier used for endpoint routing.
///
/// Inbound PUBLISH frames are delivered to the executor as remote
/// ingress from this peer (`pcl_executor_post_remote_incoming`).  Use
/// the same identifier when registering the transport with
/// `pcl_executor_register_transport()` and when configuring per-port
/// peer routing.  Default: `"default"`.
///
/// Safe to call any time before a frame is delivered.
pcl_status_t pcl_transport_template_set_peer_id(
    pcl_transport_template_t* ctx,
    const char*               peer_id);

/// \brief Get the transport vtable for `pcl_executor_set_transport`.
const pcl_transport_t* pcl_transport_template_get_transport(
    pcl_transport_template_t* ctx);

/// \brief Destroy the transport and release all resources.
///
/// Sequence:
///
///   1. Detach from the executor (`pcl_executor_set_transport(e, NULL)`).
///   2. Set the stop flag and signal the send_thread's condition
///      variable so it drains any pending frame queue and exits.
///   3. Call `hooks.wake` (if provided) so any in-flight
///      `recv_blocking` returns promptly.
///   4. Join both worker threads.
///   5. Call `hooks.close` (if provided) — guaranteed to run on a
///      single thread with no concurrent hook activity.
///   6. Free the handle.
///
/// Pending outbound frames not yet sent at step 2 are freed.  Calling
/// destroy() with a NULL handle is a no-op.
///
/// \note RPC callbacks scheduled via `invoke_async` and SVC_REQ
///       dispatch contexts scheduled via `pcl_executor_post_service_request`
///       are owned by the executor's response queue.  The caller MUST
///       stop the executor (or ensure no further `spin_once` calls)
///       before invoking destroy(), otherwise a callback may fire after
///       the transport memory has been freed.
void pcl_transport_template_destroy(pcl_transport_template_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_TEMPLATE_H
