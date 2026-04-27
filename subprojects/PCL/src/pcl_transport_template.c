/// \file pcl_transport_template.c
/// \brief Template transport — threading scaffold + engineer hook seam.
///
/// What this file gives you (free):
///   - One **send_thread** that drains a mutex-protected FIFO and calls
///     `hooks.send_blocking` for each frame.  The PCL executor enqueues
///     and returns immediately, so D2/D5 single-threaded execution is
///     preserved even if the engineer's send is fully blocking.
///   - One **recv_thread** that calls `hooks.recv_blocking` in a loop
///     and posts decoded frames into the executor:
///       * PUBLISH      → pcl_executor_post_remote_incoming()
///       * SVC_REQ      → pcl_executor_post_service_request_remote(),
///                        keyed by our peer_id so remote-exposure
///                        rules apply (services configured local-only
///                        or restricted to a peer allow-list are not
///                        silently exposed to wire traffic).  The
///                        executor invokes the matching handler and
///                        the response is fed back into our send
///                        queue as a SVC_RESP frame.
///       * SVC_RESP     → pcl_executor_post_response_msg(), keyed off
///                        a sequence-id correlation table populated by
///                        invoke_async on the executor thread.
///   - Lifecycle: open → spawn threads → run → stop flag + wake →
///     join → close.  No leaks, no hook called concurrently with itself.
///
/// What you (the engineer) plug in:
///   - The four hooks in pcl_template_io_hooks_t.  Search this file for
///     `// TODO(engineer):` to find every fill-in point.  Everything
///     else (correlation, queueing, dispatch into the executor) is
///     already wired.
///
/// All inline comments below describe the threading invariants this
/// template upholds.  If you fork the file to add RPC or streaming,
/// keep those invariants intact — the executor's contract assumes them.
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#  define _POSIX_C_SOURCE 200809L
#endif

#include "pcl/pcl_transport_template.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <pthread.h>
#endif

// -- Tunables -----------------------------------------------------------

/// Recv hook timeout — short enough for prompt shutdown, long enough to
/// keep wakeups cheap.  The engineer's recv_blocking() must honour it.
#define PCL_TEMPLATE_RECV_TIMEOUT_MS 200u

// -- Internal frame queue node ------------------------------------------
//
// Each enqueued outbound frame is a single allocation that owns deep
// copies of every string and the payload, so the executor thread is
// free to release its own buffers the moment publish() returns.

typedef struct pcl_template_outbound_t {
  pcl_template_frame_kind_t       kind;
  uint32_t                        seq_id;
  char*                           topic;
  char*                           service_name;
  char*                           type_name;
  uint8_t*                        payload;
  uint32_t                        payload_size;
  struct pcl_template_outbound_t* next;
} pcl_template_outbound_t;

// -- Pending RPC entries ------------------------------------------------
//
// Client side: each outbound SVC_REQ tracks the caller's response
// callback in pending_head, keyed by seq_id, until the matching
// SVC_RESP arrives on the recv_thread.

typedef struct pcl_template_pending_t {
  uint32_t                       seq_id;
  pcl_resp_cb_fn_t               cb;
  void*                          user_data;
  struct pcl_template_pending_t* next;
} pcl_template_pending_t;

// Server side: forward declaration — defined after the transport struct
// so the dispatch context can hold a back-pointer.
struct pcl_transport_template_t;

typedef struct {
  struct pcl_transport_template_t* ctx;
  uint32_t                         seq_id;
  char*                            type_name; /* preserved for the SVC_RESP frame */
} pcl_template_svc_dispatch_t;

// -- Peer-alias tracking ------------------------------------------------
//
// Every peer_id this transport is known by gets recorded so destroy()
// can walk the list and clear *every* executor slot pointing at us —
// even ones registered under names that have since been replaced by
// set_peer_id().  Otherwise a stale alias would still hold a pointer
// to the freed adapter_ctx.

typedef struct pcl_template_peer_alias_t {
  char*                              peer_id;
  struct pcl_template_peer_alias_t*  next;
} pcl_template_peer_alias_t;

// -- Main transport struct ----------------------------------------------

struct pcl_transport_template_t {
  pcl_template_io_hooks_t hooks;
  pcl_executor_t*         executor;
  pcl_transport_t         transport;

  char                    peer_id[64];

  // Every value peer_id has held since creation, so destroy() can
  // unregister all of them.  Protected by pending_lock (we already
  // pay for that mutex; aliases mutate only on set_peer_id which is
  // not on a hot path).
  pcl_template_peer_alias_t* peer_aliases;

  // Send-side: queue + lock + cond.  send_thread waits on send_cond
  // for new work or stop; vtable publish() pushes and signals.
  volatile int                  send_stop;
  pcl_template_outbound_t*      send_head;
  pcl_template_outbound_t*      send_tail;

  // Recv-side: stop flag is the only synchronisation needed because
  // recv_thread polls it between blocking recvs.
  volatile int                  recv_stop;

  // Client-side correlation table for invoke_async / SVC_RESP.
  // Protected by pending_lock.
  volatile uint32_t       next_seq_id;        /* monotonically increasing, never 0 */
  pcl_template_pending_t* pending_head;

#ifdef _WIN32
  HANDLE             send_thread;
  HANDLE             recv_thread;
  CRITICAL_SECTION   send_lock;
  CONDITION_VARIABLE send_cond;
  CRITICAL_SECTION   pending_lock;
#else
  pthread_t       send_thread;
  pthread_t       recv_thread;
  pthread_mutex_t send_lock;
  pthread_cond_t  send_cond;
  pthread_mutex_t pending_lock;
#endif
};

// -- Cross-platform sync wrappers ---------------------------------------

static void tpl_lock(struct pcl_transport_template_t* ctx) {
#ifdef _WIN32
  EnterCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_lock(&ctx->send_lock);
#endif
}

static void tpl_unlock(struct pcl_transport_template_t* ctx) {
#ifdef _WIN32
  LeaveCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_unlock(&ctx->send_lock);
#endif
}

static void tpl_signal(struct pcl_transport_template_t* ctx) {
#ifdef _WIN32
  WakeAllConditionVariable(&ctx->send_cond);
#else
  pthread_cond_broadcast(&ctx->send_cond);
#endif
}

static void tpl_wait(struct pcl_transport_template_t* ctx) {
#ifdef _WIN32
  SleepConditionVariableCS(&ctx->send_cond, &ctx->send_lock, INFINITE);
#else
  pthread_cond_wait(&ctx->send_cond, &ctx->send_lock);
#endif
}

static void tpl_pending_lock(struct pcl_transport_template_t* ctx) {
#ifdef _WIN32
  EnterCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_lock(&ctx->pending_lock);
#endif
}

static void tpl_pending_unlock(struct pcl_transport_template_t* ctx) {
#ifdef _WIN32
  LeaveCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_unlock(&ctx->pending_lock);
#endif
}

// -- Outbound frame helpers ---------------------------------------------

static char* tpl_strdup_or_null(const char* s) {
  if (!s) return NULL;
  size_t n = strlen(s);
  char*  out = (char*)malloc(n + 1u);
  if (!out) return NULL;
  memcpy(out, s, n + 1u);
  return out;
}

static void tpl_outbound_free(pcl_template_outbound_t* f) {
  if (!f) return;
  free(f->topic);
  free(f->service_name);
  free(f->type_name);
  free(f->payload);
  free(f);
}

static pcl_template_outbound_t* tpl_outbound_clone(
    pcl_template_frame_kind_t kind,
    uint32_t                  seq_id,
    const char*               topic,
    const char*               service_name,
    const pcl_msg_t*          msg) {
  pcl_template_outbound_t* f =
      (pcl_template_outbound_t*)calloc(1u, sizeof(*f));
  if (!f) return NULL;

  f->kind   = kind;
  f->seq_id = seq_id;

  if (topic) {
    f->topic = tpl_strdup_or_null(topic);
    if (!f->topic) { tpl_outbound_free(f); return NULL; }
  }

  if (service_name) {
    f->service_name = tpl_strdup_or_null(service_name);
    if (!f->service_name) { tpl_outbound_free(f); return NULL; }
  }

  if (msg && msg->type_name) {
    f->type_name = tpl_strdup_or_null(msg->type_name);
    if (!f->type_name) { tpl_outbound_free(f); return NULL; }
  }

  if (msg && msg->size > 0u && msg->data) {
    f->payload = (uint8_t*)malloc(msg->size);
    if (!f->payload) { tpl_outbound_free(f); return NULL; }
    memcpy(f->payload, msg->data, msg->size);
    f->payload_size = msg->size;
  }
  return f;
}

// -- Send-queue enqueue helper ------------------------------------------
//
// Used by both publish() (executor thread) and the SVC_REQ dispatch
// callback (executor thread) and the SVC_RESP dispatch callback
// (executor thread).  Always under send_lock.
//
// Returns PCL_OK on successful enqueue; PCL_ERR_STATE if the send
// thread has already been told to stop (the frame is freed in that
// case so the caller does not need to clean it up).

static pcl_status_t tpl_send_enqueue(struct pcl_transport_template_t* ctx,
                                     pcl_template_outbound_t*         frame) {
  tpl_lock(ctx);
  if (ctx->send_stop) {
    tpl_unlock(ctx);
    tpl_outbound_free(frame);
    return PCL_ERR_STATE;
  }
  if (ctx->send_tail) {
    ctx->send_tail->next = frame;
  } else {
    ctx->send_head = frame;
  }
  ctx->send_tail = frame;
  tpl_signal(ctx);
  tpl_unlock(ctx);
  return PCL_OK;
}

// -- Pending correlation table helpers ----------------------------------

static uint32_t tpl_alloc_seq_id(struct pcl_transport_template_t* ctx) {
  uint32_t id;
  tpl_pending_lock(ctx);
  id = ++ctx->next_seq_id;
  if (id == 0u) id = ++ctx->next_seq_id;  /* skip 0 — reserved */
  tpl_pending_unlock(ctx);
  return id;
}

static void tpl_pending_insert(struct pcl_transport_template_t* ctx,
                               pcl_template_pending_t*          entry) {
  tpl_pending_lock(ctx);
  entry->next = ctx->pending_head;
  ctx->pending_head = entry;
  tpl_pending_unlock(ctx);
}

static pcl_template_pending_t* tpl_pending_take(
    struct pcl_transport_template_t* ctx, uint32_t seq_id) {
  pcl_template_pending_t* entry = NULL;
  pcl_template_pending_t** link;
  tpl_pending_lock(ctx);
  for (link = &ctx->pending_head; *link; link = &(*link)->next) {
    if ((*link)->seq_id == seq_id) {
      entry = *link;
      *link = entry->next;
      entry->next = NULL;
      break;
    }
  }
  tpl_pending_unlock(ctx);
  return entry;
}

static void tpl_pending_drain(struct pcl_transport_template_t* ctx) {
  pcl_template_pending_t* head;
  tpl_pending_lock(ctx);
  head = ctx->pending_head;
  ctx->pending_head = NULL;
  tpl_pending_unlock(ctx);
  while (head) {
    pcl_template_pending_t* next = head->next;
    free(head);
    head = next;
  }
}

// -- Peer-alias helpers -------------------------------------------------

static pcl_status_t tpl_alias_remember(struct pcl_transport_template_t* ctx,
                                        const char*                       peer_id) {
  pcl_template_peer_alias_t* node;
  pcl_template_peer_alias_t* it;

  if (!peer_id || !peer_id[0]) return PCL_OK;

  tpl_pending_lock(ctx);
  for (it = ctx->peer_aliases; it; it = it->next) {
    if (it->peer_id && strcmp(it->peer_id, peer_id) == 0) {
      tpl_pending_unlock(ctx);
      return PCL_OK;  /* already tracked */
    }
  }
  node = (pcl_template_peer_alias_t*)calloc(1u, sizeof(*node));
  if (!node) { tpl_pending_unlock(ctx); return PCL_ERR_NOMEM; }
  node->peer_id = tpl_strdup_or_null(peer_id);
  if (!node->peer_id) { free(node); tpl_pending_unlock(ctx); return PCL_ERR_NOMEM; }
  node->next        = ctx->peer_aliases;
  ctx->peer_aliases = node;
  tpl_pending_unlock(ctx);
  return PCL_OK;
}

// -- Server-side SVC_REQ → SVC_RESP plumbing ----------------------------
//
// Fires on the executor thread once the local service handler returns.
// We rebuild a SVC_RESP outbound frame, hand it to the send_thread, and
// release the per-request dispatch context.  If the transport has been
// torn down (send_stop set), the enqueue helper drops the frame and
// frees it.

static void tpl_svc_response_cb(const pcl_msg_t* resp, void* user_data) {
  pcl_template_svc_dispatch_t* slot =
      (pcl_template_svc_dispatch_t*)user_data;
  if (!slot) return;

  if (slot->ctx) {
    pcl_msg_t out;
    pcl_template_outbound_t* frame;
    memset(&out, 0, sizeof(out));
    if (resp) {
      out.data      = resp->data;
      out.size      = resp->size;
      out.type_name = resp->type_name ? resp->type_name : slot->type_name;
    } else {
      out.type_name = slot->type_name;
    }

    frame = tpl_outbound_clone(PCL_TEMPLATE_FRAME_SVC_RESP, slot->seq_id,
                               NULL, NULL, &out);
    if (frame) tpl_send_enqueue(slot->ctx, frame);
  }

  free(slot->type_name);
  free(slot);
}

// -- Send thread --------------------------------------------------------
//
// Invariant: only this thread invokes hooks.send_blocking().  We hold
// send_lock to inspect the queue and to wait on the condvar; the lock
// is released across the blocking hook call so publish() on the
// executor thread never stalls behind a slow wire.

#ifdef _WIN32
static DWORD WINAPI tpl_send_thread_main(LPVOID arg)
#else
static void* tpl_send_thread_main(void* arg)
#endif
{
  struct pcl_transport_template_t* ctx =
      (struct pcl_transport_template_t*)arg;

  for (;;) {
    pcl_template_outbound_t* frame = NULL;

    tpl_lock(ctx);
    while (!ctx->send_stop && ctx->send_head == NULL) {
      tpl_wait(ctx);
    }
    if (ctx->send_head) {
      frame = ctx->send_head;
      ctx->send_head = frame->next;
      if (ctx->send_head == NULL) ctx->send_tail = NULL;
      frame->next = NULL;
    }
    tpl_unlock(ctx);

    if (!frame) {
      // Woken with empty queue → must be a stop signal.  Drain any
      // late arrivals (none expected, but cheap to cover) and exit.
      if (ctx->send_stop) break;
      continue;
    }

    /* Abort if stop was signalled while we were dequeuing.  This avoids
     * entering send_blocking after destroy() begins — which matters when
     * hooks.wake cannot interrupt an in-flight send.  If we are already
     * inside send_blocking when wake fires, the hook must unblock it. */
    if (ctx->send_stop) {
      tpl_outbound_free(frame);
      break;
    }

    {
      pcl_template_frame_t out;
      pcl_status_t         rc;
      memset(&out, 0, sizeof(out));
      out.kind         = frame->kind;
      out.seq_id       = frame->seq_id;
      out.topic        = frame->topic;
      out.service_name = frame->service_name;
      out.type_name    = frame->type_name;
      out.payload      = frame->payload;
      out.payload_size = frame->payload_size;

      rc = ctx->hooks.send_blocking(ctx->hooks.user_data, &out);
      if (rc != PCL_OK) {
        pcl_log(NULL, PCL_LOG_WARN,
                "pcl_transport_template: send_blocking failed (%d), frame dropped",
                (int)rc);
      }
    }

    tpl_outbound_free(frame);
  }

  // Drain remaining frames so destroy() does not leak.
  for (;;) {
    pcl_template_outbound_t* frame = NULL;
    tpl_lock(ctx);
    if (ctx->send_head) {
      frame = ctx->send_head;
      ctx->send_head = frame->next;
      if (ctx->send_head == NULL) ctx->send_tail = NULL;
    }
    tpl_unlock(ctx);
    if (!frame) break;
    tpl_outbound_free(frame);
  }

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

// -- Recv thread --------------------------------------------------------
//
// Invariant: only this thread invokes hooks.recv_blocking().  When a
// frame is delivered we own its strings and payload (per the hook
// contract) and free them after dispatch.  The executor's
// post_remote_incoming() deep-copies before returning, so a single
// free() pass here is safe.

#ifdef _WIN32
static DWORD WINAPI tpl_recv_thread_main(LPVOID arg)
#else
static void* tpl_recv_thread_main(void* arg)
#endif
{
  struct pcl_transport_template_t* ctx =
      (struct pcl_transport_template_t*)arg;

  while (!ctx->recv_stop) {
    pcl_template_frame_t in;
    pcl_status_t         rc;

    memset(&in, 0, sizeof(in));
    rc = ctx->hooks.recv_blocking(ctx->hooks.user_data, &in,
                                  PCL_TEMPLATE_RECV_TIMEOUT_MS);
    if (rc == PCL_ERR_TIMEOUT) {
      // Idle wakeup — re-check stop flag.  No allocations to free.
      continue;
    }
    if (rc != PCL_OK) {
      pcl_log(NULL, PCL_LOG_WARN,
              "pcl_transport_template: recv_blocking failed (%d)",
              (int)rc);
      // Don't busy-spin on a permanently broken stack: caller is
      // expected to invoke destroy() once they notice the failure.
      // We loop and retry; if recv_stop is set we exit on the next
      // iteration.
      continue;
    }

    if (in.kind == PCL_TEMPLATE_FRAME_PUBLISH && in.topic) {
      pcl_msg_t msg;
      memset(&msg, 0, sizeof(msg));
      msg.data      = in.payload;
      msg.size      = in.payload_size;
      /* The executor rejects post_remote_incoming with NULL type_name
       * (PCL_ERR_INVALID).  Normalise to "" so untyped wire frames
       * still flow — subscribers gate on topic + peer, not on
       * matching an empty type. */
      msg.type_name = in.type_name ? in.type_name : "";

      pcl_executor_post_remote_incoming(ctx->executor, ctx->peer_id,
                                        in.topic, &msg);
    } else if (in.kind == PCL_TEMPLATE_FRAME_SVC_REQ && in.service_name) {
      // Server-side dispatch: route the request to a service handler
      // via the *remote-aware* ingress helper, passing our peer_id as
      // the source.  This preserves remote-exposure rules — services
      // restricted to a peer allow-list (or local-only) are not
      // silently exposed to arbitrary inbound wire traffic, which
      // would have happened with the plain post_service_request.
      pcl_template_svc_dispatch_t* slot =
          (pcl_template_svc_dispatch_t*)calloc(1u, sizeof(*slot));
      if (slot) {
        pcl_msg_t req;
        slot->ctx       = ctx;
        slot->seq_id    = in.seq_id;
        slot->type_name = tpl_strdup_or_null(in.type_name);

        memset(&req, 0, sizeof(req));
        req.data      = in.payload;
        req.size      = in.payload_size;
        /* post_service_request_remote requires non-NULL type_name. */
        req.type_name = in.type_name ? in.type_name : "";

        if (pcl_executor_post_service_request_remote(ctx->executor,
                                                     ctx->peer_id,
                                                     in.service_name,
                                                     &req,
                                                     tpl_svc_response_cb,
                                                     slot) != PCL_OK) {
          free(slot->type_name);
          free(slot);
        }
      }
    } else if (in.kind == PCL_TEMPLATE_FRAME_SVC_RESP) {
      // Client-side: match the seq_id back to the caller's pending
      // entry and deliver the response on the executor thread.
      pcl_template_pending_t* entry = tpl_pending_take(ctx, in.seq_id);
      if (entry) {
        pcl_msg_t resp;
        memset(&resp, 0, sizeof(resp));
        resp.data      = in.payload;
        resp.size      = in.payload_size;
        resp.type_name = in.type_name ? in.type_name : "";

        pcl_executor_post_response_msg(ctx->executor, entry->cb,
                                       entry->user_data, &resp);
        free(entry);
      } else {
        pcl_log(NULL, PCL_LOG_DEBUG,
                "pcl_transport_template: SVC_RESP for unknown seq %u",
                (unsigned)in.seq_id);
      }
    } else {
      pcl_log(NULL, PCL_LOG_DEBUG,
              "pcl_transport_template: ignored malformed frame kind %d",
              (int)in.kind);
    }

    // The hook handed us ownership; free everything before the next
    // recv so peak memory stays one frame.
    free((void*)in.topic);
    free((void*)in.service_name);
    free((void*)in.type_name);
    free((void*)in.payload);
  }

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

// -- Transport vtable: publish (executor thread → send queue) ----------
//
// Runs on the executor thread.  Must NOT block.  We deep-copy the
// caller's borrowed buffers, append to the FIFO under send_lock, and
// signal the send_thread.

static pcl_status_t tpl_publish(void*            adapter_ctx,
                                const char*      topic,
                                const pcl_msg_t* msg) {
  struct pcl_transport_template_t* ctx =
      (struct pcl_transport_template_t*)adapter_ctx;
  pcl_template_outbound_t* frame;

  if (!ctx || !topic || !msg) return PCL_ERR_INVALID;
  if (ctx->send_stop)         return PCL_ERR_STATE;

  frame = tpl_outbound_clone(PCL_TEMPLATE_FRAME_PUBLISH, 0u, topic, NULL, msg);
  if (!frame) return PCL_ERR_NOMEM;

  return tpl_send_enqueue(ctx, frame);
}

// -- Transport vtable: invoke_async (client-side service RPC) ----------
//
// Runs on the executor thread.  Allocates a sequence id, registers the
// caller's response callback in the correlation table, and enqueues a
// SVC_REQ frame for the send_thread.  The matching SVC_RESP arrives on
// the recv_thread which delivers the response back via
// pcl_executor_post_response_msg().

static pcl_status_t tpl_invoke_async(void*            adapter_ctx,
                                     const char*      service_name,
                                     const pcl_msg_t* request,
                                     pcl_resp_cb_fn_t callback,
                                     void*            user_data) {
  struct pcl_transport_template_t* ctx =
      (struct pcl_transport_template_t*)adapter_ctx;
  pcl_template_pending_t*  entry;
  pcl_template_outbound_t* frame;

  if (!ctx || !service_name || !request || !callback) return PCL_ERR_INVALID;
  if (ctx->send_stop)                                  return PCL_ERR_STATE;

  entry = (pcl_template_pending_t*)calloc(1u, sizeof(*entry));
  if (!entry) return PCL_ERR_NOMEM;
  entry->seq_id    = tpl_alloc_seq_id(ctx);
  entry->cb        = callback;
  entry->user_data = user_data;

  frame = tpl_outbound_clone(PCL_TEMPLATE_FRAME_SVC_REQ, entry->seq_id,
                             NULL, service_name, request);
  if (!frame) {
    free(entry);
    return PCL_ERR_NOMEM;
  }

  /* Insert pending entry BEFORE enqueuing the frame so an absurdly fast
   * response can never race the lookup.  If enqueue fails (send thread
   * already stopped) we must reclaim the pending slot — otherwise the
   * caller's callback would be orphaned and never fired. */
  tpl_pending_insert(ctx, entry);
  {
    pcl_status_t rc = tpl_send_enqueue(ctx, frame);
    if (rc != PCL_OK) {
      pcl_template_pending_t* taken = tpl_pending_take(ctx, entry->seq_id);
      if (taken) free(taken);
      /* tpl_send_enqueue already freed `frame` on the error path. */
      return rc;
    }
  }
  return PCL_OK;
}

// -- Transport vtable: subscribe ----------------------------------------
//
// The template is wire-agnostic: the engineer's recv_blocking already
// surfaces every inbound frame.  There is nothing per-topic to do.
// Concrete transports that *do* need per-topic registration (e.g. DDS
// reader creation) should fan out to their stack here.

static pcl_status_t tpl_subscribe(void*       adapter_ctx,
                                  const char* topic,
                                  const char* type_name) {
  (void)adapter_ctx; (void)topic; (void)type_name;
  // TODO(engineer): if your wire stack requires per-topic subscription
  //                 (DDS reader, MQTT SUBSCRIBE, etc.), forward the
  //                 call to your I/O context here.  Otherwise leave
  //                 this as a no-op.
  return PCL_OK;
}

// -- Transport vtable: shutdown -----------------------------------------
//
// Called by the executor during set_transport(NULL) and during destroy.
// We must not join here (the executor thread is calling us) — we only
// raise the stop flags and let destroy() perform the actual join.

static void tpl_shutdown(void* adapter_ctx) {
  struct pcl_transport_template_t* ctx =
      (struct pcl_transport_template_t*)adapter_ctx;
  if (!ctx) return;

  tpl_lock(ctx);
  ctx->send_stop = 1;
  tpl_signal(ctx);
  tpl_unlock(ctx);

  ctx->recv_stop = 1;
  if (ctx->hooks.wake) {
    ctx->hooks.wake(ctx->hooks.user_data);
  }
}

// -- Public API ---------------------------------------------------------

pcl_transport_template_t* pcl_transport_template_create(
    const pcl_template_io_hooks_t* hooks,
    pcl_executor_t*                executor) {
  struct pcl_transport_template_t* ctx;

  // Engineer contract: send_blocking + recv_blocking are the minimum
  // viable set.  open / close / wake are optional (NULL-safe).
  if (!hooks || !executor)         return NULL;
  if (!hooks->send_blocking)       return NULL;
  if (!hooks->recv_blocking)       return NULL;

  ctx = (struct pcl_transport_template_t*)calloc(1u, sizeof(*ctx));
  if (!ctx) return NULL;

  ctx->hooks    = *hooks;
  ctx->executor = executor;
  snprintf(ctx->peer_id, sizeof(ctx->peer_id), "%s", "default");

  ctx->transport.publish      = tpl_publish;
  ctx->transport.subscribe    = tpl_subscribe;
  ctx->transport.invoke_async = tpl_invoke_async;
  ctx->transport.shutdown     = tpl_shutdown;
  ctx->transport.adapter_ctx  = ctx;

#ifdef _WIN32
  InitializeCriticalSection(&ctx->send_lock);
  InitializeCriticalSection(&ctx->pending_lock);
  InitializeConditionVariable(&ctx->send_cond);
#else
  if (pthread_mutex_init(&ctx->send_lock, NULL) != 0) {
    free(ctx);
    return NULL;
  }
  if (pthread_mutex_init(&ctx->pending_lock, NULL) != 0) {
    pthread_mutex_destroy(&ctx->send_lock);
    free(ctx);
    return NULL;
  }
  if (pthread_cond_init(&ctx->send_cond, NULL) != 0) {
    pthread_mutex_destroy(&ctx->pending_lock);
    pthread_mutex_destroy(&ctx->send_lock);
    free(ctx);
    return NULL;
  }
#endif

  // TODO(engineer): hooks.open is your chance to dial the socket,
  //                 open the serial port, join the bus, etc.  We hold
  //                 NO locks here, the worker threads do not exist
  //                 yet, and a non-PCL_OK return aborts create()
  //                 cleanly without leaking resources.
  if (ctx->hooks.open) {
    pcl_status_t rc = ctx->hooks.open(ctx->hooks.user_data);
    if (rc != PCL_OK) {
#ifdef _WIN32
      DeleteCriticalSection(&ctx->pending_lock);
      DeleteCriticalSection(&ctx->send_lock);
#else
      pthread_cond_destroy(&ctx->send_cond);
      pthread_mutex_destroy(&ctx->pending_lock);
      pthread_mutex_destroy(&ctx->send_lock);
#endif
      free(ctx);
      return NULL;
    }
  }

#ifdef _WIN32
  ctx->send_thread =
      CreateThread(NULL, 0, tpl_send_thread_main, ctx, 0, NULL);
  if (!ctx->send_thread) {
    if (ctx->hooks.close) ctx->hooks.close(ctx->hooks.user_data);
    DeleteCriticalSection(&ctx->pending_lock);
    DeleteCriticalSection(&ctx->send_lock);
    free(ctx);
    return NULL;
  }
  ctx->recv_thread =
      CreateThread(NULL, 0, tpl_recv_thread_main, ctx, 0, NULL);
  if (!ctx->recv_thread) {
    ctx->send_stop = 1;
    tpl_lock(ctx); tpl_signal(ctx); tpl_unlock(ctx);
    WaitForSingleObject(ctx->send_thread, INFINITE);
    CloseHandle(ctx->send_thread);
    if (ctx->hooks.close) ctx->hooks.close(ctx->hooks.user_data);
    DeleteCriticalSection(&ctx->pending_lock);
    DeleteCriticalSection(&ctx->send_lock);
    free(ctx);
    return NULL;
  }
#else
  if (pthread_create(&ctx->send_thread, NULL, tpl_send_thread_main, ctx) != 0) {
    if (ctx->hooks.close) ctx->hooks.close(ctx->hooks.user_data);
    pthread_cond_destroy(&ctx->send_cond);
    pthread_mutex_destroy(&ctx->pending_lock);
    pthread_mutex_destroy(&ctx->send_lock);
    free(ctx);
    return NULL;
  }
  if (pthread_create(&ctx->recv_thread, NULL, tpl_recv_thread_main, ctx) != 0) {
    ctx->send_stop = 1;
    tpl_lock(ctx); tpl_signal(ctx); tpl_unlock(ctx);
    pthread_join(ctx->send_thread, NULL);
    if (ctx->hooks.close) ctx->hooks.close(ctx->hooks.user_data);
    pthread_cond_destroy(&ctx->send_cond);
    pthread_mutex_destroy(&ctx->pending_lock);
    pthread_mutex_destroy(&ctx->send_lock);
    free(ctx);
    return NULL;
  }
#endif

  /* Pre-seed the alias list with the implicit default so destroy()
   * unregisters it even if the caller never explicitly renamed.
   * Done after thread creation so the error-cleanup paths above don't
   * need to deal with the alias list. */
  if (tpl_alias_remember(ctx, "default") != PCL_OK) {
    /* OOM on the alias seed: the transport was never returned to the
     * caller so it was never registered in the executor.  destroy()
     * will stop the threads, call close, and free cleanly. */
    pcl_transport_template_destroy(ctx);
    return NULL;
  }

  return ctx;
}

pcl_status_t pcl_transport_template_set_peer_id(
    pcl_transport_template_t* ctx,
    const char*               peer_id) {
  char         old_id[sizeof(ctx->peer_id)];
  pcl_status_t rc;

  if (!ctx || !peer_id || !peer_id[0]) return PCL_ERR_INVALID;

  /* Snapshot the current ID so we can restore it if alias tracking
   * fails — otherwise ctx->peer_id would hold the new name while the
   * executor slot and alias list still reflect the old one. */
  memcpy(old_id, ctx->peer_id, sizeof(old_id));
  snprintf(ctx->peer_id, sizeof(ctx->peer_id), "%s", peer_id);

  /* Use the *truncated* form (ctx->peer_id) rather than the raw
   * caller-supplied string: pcl_executor_register_transport stores
   * peer ids in a fixed 64-byte buffer too, so comparing against the
   * truncated value is what actually matches the executor's slot. */
  rc = tpl_alias_remember(ctx, ctx->peer_id);
  if (rc != PCL_OK) {
    memcpy(ctx->peer_id, old_id, sizeof(ctx->peer_id));
  }
  return rc;
}

const pcl_transport_t* pcl_transport_template_get_transport(
    pcl_transport_template_t* ctx) {
  if (!ctx) return NULL;
  return &ctx->transport;
}

void pcl_transport_template_destroy(pcl_transport_template_t* ctx) {
  if (!ctx) return;

  if (ctx->executor) {
    pcl_template_peer_alias_t* alias;

    /* Only clear the executor's *default* transport slot if it is
     * actually pointing at us.  In multi-transport setups this
     * adapter may only have been registered as a named peer, with
     * an unrelated transport installed as the default — wiping it
     * unconditionally would silently break the executor's default
     * routing for everyone else. */
    {
      const pcl_transport_t* current =
          pcl_executor_get_transport(ctx->executor);
      if (current && current->adapter_ctx == ctx) {
        pcl_executor_set_transport(ctx->executor, NULL);
      }
    }

    /* Walk every peer_id this transport has *ever* been known by and
     * unregister each from the executor.  Without this, a caller who
     * mutated peer_id via set_peer_id() would leave the original
     * alias still bound to the soon-to-be-freed adapter_ctx, and any
     * later routing to that alias would dereference stale memory. */
    tpl_pending_lock(ctx);
    alias = ctx->peer_aliases;
    ctx->peer_aliases = NULL;
    tpl_pending_unlock(ctx);
    while (alias) {
      pcl_template_peer_alias_t* next = alias->next;
      if (alias->peer_id) {
        pcl_executor_register_transport(ctx->executor, alias->peer_id, NULL);
      }
      free(alias->peer_id);
      free(alias);
      alias = next;
    }
  }

  // Stop signal + wake — see invariants on tpl_shutdown.
  tpl_lock(ctx);
  ctx->send_stop = 1;
  tpl_signal(ctx);
  tpl_unlock(ctx);

  ctx->recv_stop = 1;
  if (ctx->hooks.wake) {
    ctx->hooks.wake(ctx->hooks.user_data);
  }

#ifdef _WIN32
  if (ctx->send_thread) {
    WaitForSingleObject(ctx->send_thread, INFINITE);
    CloseHandle(ctx->send_thread);
    ctx->send_thread = NULL;
  }
  if (ctx->recv_thread) {
    WaitForSingleObject(ctx->recv_thread, INFINITE);
    CloseHandle(ctx->recv_thread);
    ctx->recv_thread = NULL;
  }
#else
  if (ctx->send_thread) {
    pthread_join(ctx->send_thread, NULL);
    ctx->send_thread = 0;
  }
  if (ctx->recv_thread) {
    pthread_join(ctx->recv_thread, NULL);
    ctx->recv_thread = 0;
  }
#endif

  // After both workers are joined, no other thread touches the pending
  // table; safe to drop any remaining entries (callers whose responses
  // never arrived) without leaking memory.
  tpl_pending_drain(ctx);

#ifdef _WIN32
  DeleteCriticalSection(&ctx->pending_lock);
  DeleteCriticalSection(&ctx->send_lock);
#else
  pthread_cond_destroy(&ctx->send_cond);
  pthread_mutex_destroy(&ctx->pending_lock);
  pthread_mutex_destroy(&ctx->send_lock);
#endif

  // hooks.close runs *after* both workers are joined, so the engineer's
  // teardown observes a quiescent system — no concurrent send/recv.
  if (ctx->hooks.close) {
    ctx->hooks.close(ctx->hooks.user_data);
  }

  free(ctx);
}
