/// \file test_pcl_template_transport.cpp
/// \brief Tests for the engineer-extendable template transport.
///
/// Two layers of coverage:
///
///   1. **Template-specific tests** -- exercise edge cases of the
///      scaffold itself (NULL hook validation, idempotent destroy,
///      open() failure path, peer id round-trip).
///   2. **Conformance tests** -- drive the same pub/sub and service
///      round-trip helpers (`pcl_transport_conformance.hpp`) that any
///      other transport can reuse.  The conformance suite is wired up
///      with an in-memory `LoopbackLink` pair of hooks, so the test
///      runs entirely without sockets.
///
/// To run the conformance suite against a *new* transport, mirror the
/// `MakeLoopbackPair()` helper below but plug in your own
/// pcl_template_io_hooks_t (or skip the template entirely and hand
/// pcl_conformance helpers a TransportPair that uses your transport's
/// concrete create/get_transport/destroy API).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_transport_template.h"
}

#include "pcl_transport_conformance.hpp"

namespace {

// ---------------------------------------------------------------------------
// In-memory loopback hooks
//
// Two LoopbackEnd instances share two queues -- one in each direction.
// send_blocking() pushes into the peer's inbound queue; recv_blocking()
// pops from this end's inbound queue.  The hooks deliberately respect
// the C ownership rules from pcl_transport_template.h: outbound payload
// pointers are borrowed (we copy with malloc on send), inbound payload
// pointers are heap-allocated and handed to the template (which frees).
// ---------------------------------------------------------------------------

struct OwnedFrame {
  pcl_template_frame_kind_t kind = PCL_TEMPLATE_FRAME_PUBLISH;
  uint32_t                  seq_id = 0;
  char*                     topic = nullptr;
  char*                     service_name = nullptr;
  char*                     type_name = nullptr;
  uint8_t*                  payload = nullptr;
  uint32_t                  payload_size = 0;

  OwnedFrame() = default;
  OwnedFrame(const OwnedFrame&)            = delete;
  OwnedFrame& operator=(const OwnedFrame&) = delete;
  OwnedFrame(OwnedFrame&& other) noexcept { *this = std::move(other); }
  OwnedFrame& operator=(OwnedFrame&& other) noexcept {
    if (this != &other) {
      reset();
      kind         = other.kind;
      seq_id       = other.seq_id;
      topic        = other.topic;        other.topic = nullptr;
      service_name = other.service_name; other.service_name = nullptr;
      type_name    = other.type_name;    other.type_name = nullptr;
      payload      = other.payload;      other.payload = nullptr;
      payload_size = other.payload_size;
    }
    return *this;
  }
  ~OwnedFrame() { reset(); }

  void reset() {
    free(topic);        topic = nullptr;
    free(service_name); service_name = nullptr;
    free(type_name);    type_name = nullptr;
    free(payload);      payload = nullptr;
    payload_size = 0;
  }

  static char* dup(const char* s) {
    if (!s) return nullptr;
    size_t n = std::strlen(s);
    char* out = static_cast<char*>(std::malloc(n + 1));
    std::memcpy(out, s, n + 1);
    return out;
  }

  static OwnedFrame fromBorrowed(const pcl_template_frame_t& src) {
    OwnedFrame out;
    out.kind         = src.kind;
    out.seq_id       = src.seq_id;
    out.topic        = dup(src.topic);
    out.service_name = dup(src.service_name);
    out.type_name    = dup(src.type_name);
    if (src.payload && src.payload_size > 0) {
      out.payload = static_cast<uint8_t*>(std::malloc(src.payload_size));
      std::memcpy(out.payload, src.payload, src.payload_size);
      out.payload_size = src.payload_size;
    }
    return out;
  }
};

struct LoopbackLink {
  std::mutex                mu;
  std::condition_variable   cv;
  std::deque<OwnedFrame>    queue;
};

struct LoopbackEnd {
  LoopbackLink*     outbound = nullptr;
  LoopbackLink*     inbound  = nullptr;
  std::atomic<bool> stopped{false};
  std::atomic<int>  send_count{0};
  std::atomic<int>  recv_count{0};
};

extern "C" pcl_status_t loopback_send(void* ud, const pcl_template_frame_t* f) {
  auto* end = static_cast<LoopbackEnd*>(ud);
  if (!end || !f) return PCL_ERR_INVALID;
  end->send_count.fetch_add(1);
  OwnedFrame copy = OwnedFrame::fromBorrowed(*f);
  std::lock_guard<std::mutex> lk(end->outbound->mu);
  end->outbound->queue.emplace_back(std::move(copy));
  end->outbound->cv.notify_all();
  return PCL_OK;
}

extern "C" pcl_status_t loopback_recv(void* ud, pcl_template_frame_t* out,
                                      uint32_t timeout_ms) {
  auto* end = static_cast<LoopbackEnd*>(ud);
  if (!end || !out) return PCL_ERR_INVALID;

  std::unique_lock<std::mutex> lk(end->inbound->mu);
  end->inbound->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
      [&] { return !end->inbound->queue.empty() || end->stopped.load(); });
  if (end->inbound->queue.empty()) return PCL_ERR_TIMEOUT;

  OwnedFrame frame = std::move(end->inbound->queue.front());
  end->inbound->queue.pop_front();
  lk.unlock();

  // Hand ownership over to the template -- it frees with C free().
  out->kind         = frame.kind;
  out->seq_id       = frame.seq_id;
  out->topic        = frame.topic;        frame.topic = nullptr;
  out->service_name = frame.service_name; frame.service_name = nullptr;
  out->type_name    = frame.type_name;    frame.type_name = nullptr;
  out->payload      = frame.payload;      frame.payload = nullptr;
  out->payload_size = frame.payload_size;
  end->recv_count.fetch_add(1);
  return PCL_OK;
}

extern "C" void loopback_wake(void* ud) {
  auto* end = static_cast<LoopbackEnd*>(ud);
  if (!end) return;
  end->stopped = true;
  // Wake recv on both possible queues -- the template only reads from
  // `inbound` but wake on both is harmless and keeps the symmetry.
  if (end->inbound)  { std::lock_guard<std::mutex> lk(end->inbound->mu);  end->inbound->cv.notify_all(); }
  if (end->outbound) { std::lock_guard<std::mutex> lk(end->outbound->mu); end->outbound->cv.notify_all(); }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}
void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

/// Build a fully-wired loopback transport pair: two executors, two
/// template transports, two pairs of hooks talking back-to-back through
/// a pair of in-memory queues.  All resources are owned by the
/// returned bundle and released by its destructor.
struct LoopbackPair {
  // Two queues -- A->B and B->A.
  LoopbackLink ab;
  LoopbackLink ba;

  LoopbackEnd  a_end{ &ab, &ba };
  LoopbackEnd  b_end{ &ba, &ab };

  pcl_executor_t*           exec_a = nullptr;
  pcl_executor_t*           exec_b = nullptr;
  pcl_transport_template_t* tpl_a  = nullptr;
  pcl_transport_template_t* tpl_b  = nullptr;

  LoopbackPair() {
    exec_a = pcl_executor_create();
    exec_b = pcl_executor_create();

    pcl_template_io_hooks_t hooks_a = {};
    hooks_a.user_data     = &a_end;
    hooks_a.send_blocking = &loopback_send;
    hooks_a.recv_blocking = &loopback_recv;
    hooks_a.wake          = &loopback_wake;

    pcl_template_io_hooks_t hooks_b = {};
    hooks_b.user_data     = &b_end;
    hooks_b.send_blocking = &loopback_send;
    hooks_b.recv_blocking = &loopback_recv;
    hooks_b.wake          = &loopback_wake;

    tpl_a = pcl_transport_template_create(&hooks_a, exec_a);
    tpl_b = pcl_transport_template_create(&hooks_b, exec_b);

    pcl_transport_template_set_peer_id(tpl_a, "node_b");
    pcl_transport_template_set_peer_id(tpl_b, "node_a");

    // Each side registers the *other* side's peer id with its own
    // transport (executor publish() / invoke_async() use the bound
    // transport, and inbound ingress is tagged by the recv_thread's
    // peer_id setting).
    pcl_executor_set_transport(exec_a,
        pcl_transport_template_get_transport(tpl_a));
    pcl_executor_set_transport(exec_b,
        pcl_transport_template_get_transport(tpl_b));
    pcl_executor_register_transport(exec_a, "node_b",
        pcl_transport_template_get_transport(tpl_a));
    pcl_executor_register_transport(exec_b, "node_a",
        pcl_transport_template_get_transport(tpl_b));
  }

  ~LoopbackPair() {
    // Clear executor registrations using the originally registered peer
    // ids -- robust even if a test mutated the transport's peer_id mid-
    // life (the template can only clear its *current* peer_id).
    if (exec_a) {
      pcl_executor_set_transport(exec_a, nullptr);
      pcl_executor_register_transport(exec_a, "node_b", nullptr);
    }
    if (exec_b) {
      pcl_executor_set_transport(exec_b, nullptr);
      pcl_executor_register_transport(exec_b, "node_a", nullptr);
    }
    if (tpl_a) pcl_transport_template_destroy(tpl_a);
    if (tpl_b) pcl_transport_template_destroy(tpl_b);
    if (exec_a) pcl_executor_destroy(exec_a);
    if (exec_b) pcl_executor_destroy(exec_b);
  }

  // Shape the pair for the conformance helpers in either direction.
  pcl_conformance::TransportPair conformancePair_AtoB() const {
    pcl_conformance::TransportPair p;
    p.sender_exec      = exec_a;
    p.receiver_exec    = exec_b;
    p.sender_vtable    = pcl_transport_template_get_transport(tpl_a);
    p.receiver_vtable  = pcl_transport_template_get_transport(tpl_b);
    p.sender_peer_id   = "node_a";  // identity B sees on incoming traffic
    p.receiver_peer_id = "node_b";
    return p;
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Template-specific tests
// ---------------------------------------------------------------------------

TEST(PclTransportTemplate, RejectsMissingHooks) {
  silence_logs();
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  pcl_template_io_hooks_t hooks = {};
  EXPECT_EQ(pcl_transport_template_create(&hooks, e), nullptr)
      << "create() must reject hooks with no send_blocking/recv_blocking";

  hooks.send_blocking = &loopback_send;
  EXPECT_EQ(pcl_transport_template_create(&hooks, e), nullptr)
      << "create() must reject hooks with no recv_blocking";

  EXPECT_EQ(pcl_transport_template_create(nullptr, e), nullptr);
  hooks.recv_blocking = &loopback_recv;
  EXPECT_EQ(pcl_transport_template_create(&hooks, nullptr), nullptr);

  pcl_executor_destroy(e);
  restore_logs();
}

TEST(PclTransportTemplate, OpenFailureAbortsCreate) {
  silence_logs();
  auto* e = pcl_executor_create();

  struct OpenCtx { std::atomic<int> open_calls{0}, close_calls{0}; } ctx;
  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &ctx;
  hooks.send_blocking = &loopback_send;
  hooks.recv_blocking = &loopback_recv;
  hooks.open  = [](void* ud) -> pcl_status_t {
    static_cast<OpenCtx*>(ud)->open_calls++;
    return PCL_ERR_STATE;
  };
  hooks.close = [](void* ud) {
    static_cast<OpenCtx*>(ud)->close_calls++;
  };

  EXPECT_EQ(pcl_transport_template_create(&hooks, e), nullptr);
  EXPECT_EQ(ctx.open_calls.load(), 1);
  EXPECT_EQ(ctx.close_calls.load(), 0)  // close must NOT run when open fails
      << "close should only run after a successful open";

  pcl_executor_destroy(e);
  restore_logs();
}

TEST(PclTransportTemplate, DestroyNullIsNoOp) {
  pcl_transport_template_destroy(nullptr);
  SUCCEED();
}

TEST(PclTransportTemplate, SetPeerIdValidation) {
  silence_logs();
  // A standalone transport -- no executor registrations to keep in sync,
  // so we can rename peer_id freely.
  auto* e = pcl_executor_create();
  LoopbackLink dummy_a, dummy_b;
  LoopbackEnd  end{ &dummy_a, &dummy_b };
  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &end;
  hooks.send_blocking = &loopback_send;
  hooks.recv_blocking = &loopback_recv;
  hooks.wake          = &loopback_wake;
  auto* t = pcl_transport_template_create(&hooks, e);
  ASSERT_NE(t, nullptr);

  EXPECT_EQ(pcl_transport_template_set_peer_id(nullptr, "x"),  PCL_ERR_INVALID);
  EXPECT_EQ(pcl_transport_template_set_peer_id(t, nullptr),    PCL_ERR_INVALID);
  EXPECT_EQ(pcl_transport_template_set_peer_id(t, ""),         PCL_ERR_INVALID);
  EXPECT_EQ(pcl_transport_template_set_peer_id(t, "alpha"),    PCL_OK);

  pcl_transport_template_destroy(t);
  pcl_executor_destroy(e);
  restore_logs();
}

TEST(PclTransportTemplate, VtableShape) {
  silence_logs();
  LoopbackPair p;
  pcl_conformance::expectVtableShape(
      pcl_transport_template_get_transport(p.tpl_a),
      /*require_invoke_async=*/true);
  restore_logs();
}

TEST(PclTransportTemplate, CloseRunsAfterDestroy) {
  silence_logs();
  auto* e = pcl_executor_create();

  LoopbackLink loopA, loopB;
  LoopbackEnd  endA{ &loopA, &loopB };
  std::atomic<int> close_calls{0};

  struct Ctx { LoopbackEnd* end; std::atomic<int>* close_calls; } ctx{ &endA, &close_calls };

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &ctx;
  hooks.send_blocking = [](void* ud, const pcl_template_frame_t* f) {
    return loopback_send(static_cast<Ctx*>(ud)->end, f);
  };
  hooks.recv_blocking = [](void* ud, pcl_template_frame_t* out, uint32_t t) {
    return loopback_recv(static_cast<Ctx*>(ud)->end, out, t);
  };
  hooks.wake  = [](void* ud) { loopback_wake(static_cast<Ctx*>(ud)->end); };
  hooks.close = [](void* ud) {
    static_cast<Ctx*>(ud)->close_calls->fetch_add(1);
  };

  auto* t = pcl_transport_template_create(&hooks, e);
  ASSERT_NE(t, nullptr);
  pcl_transport_template_destroy(t);
  EXPECT_EQ(close_calls.load(), 1) << "close must fire exactly once on destroy";

  pcl_executor_destroy(e);
  restore_logs();
}

// Regression: a transport that has been renamed via set_peer_id() must
// still unregister *every* alias from its executor on destroy, otherwise
// the executor keeps a dangling pointer into the freed adapter_ctx.
TEST(PclTransportTemplate, DestroyClearsRenamedPeerAlias) {
  silence_logs();

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  LoopbackLink loopA, loopB;
  LoopbackEnd  end{ &loopA, &loopB };
  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &end;
  hooks.send_blocking = &loopback_send;
  hooks.recv_blocking = &loopback_recv;
  hooks.wake          = &loopback_wake;

  auto* t = pcl_transport_template_create(&hooks, e);
  ASSERT_NE(t, nullptr);

  /* Caller registers the transport under its initial alias, then
   * renames it.  Without the fix the original alias still points into
   * the soon-to-be-freed adapter_ctx after destroy(). */
  ASSERT_EQ(pcl_transport_template_set_peer_id(t, "first"), PCL_OK);
  pcl_executor_register_transport(e, "first",
      pcl_transport_template_get_transport(t));
  ASSERT_EQ(pcl_transport_template_set_peer_id(t, "second"), PCL_OK);

  pcl_transport_template_destroy(t);
  /* If destroy did NOT clear "first", pcl_executor_destroy will invoke
   * tpl_shutdown on the freed adapter_ctx (caught by ASAN). */
  pcl_executor_destroy(e);
  restore_logs();
}

// Regression: hooks may report inbound PUBLISH frames with type_name=NULL.
// The template must normalise that to "" so the executor's
// post_remote_incoming() does not silently reject them.
TEST(PclTransportTemplate, RecvPublishNormalisesNullTypeName) {
  silence_logs();
  LoopbackPair p;

  /* Synthesise a frame directly into A's inbound queue with type_name
   * left NULL, then drive A's executor and assert the subscriber on
   * exec_a observes it. */
  struct SubState {
    std::atomic<bool> got{false};
  } state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = { "node_b" };
    pcl_port_t* port = pcl_container_add_subscriber(
        c, "untyped/topic", "",
        [](pcl_container_t*, const pcl_msg_t*, void* sub_ud) {
          static_cast<SubState*>(sub_ud)->got = true;
        }, ud);
    pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("recv_null", &cbs, &state);
  ASSERT_EQ(pcl_container_configure(sub_c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(sub_c),  PCL_OK);
  ASSERT_EQ(pcl_executor_add(p.exec_a, sub_c), PCL_OK);

  /* Inject a PUBLISH frame into A's inbound queue (the BA link) with
   * type_name explicitly NULL -- this is what a real hook returning
   * a typeless wire datagram would deliver. */
  {
    OwnedFrame f;
    f.kind         = PCL_TEMPLATE_FRAME_PUBLISH;
    f.topic        = OwnedFrame::dup("untyped/topic");
    f.type_name    = nullptr;  /* the deliberate edge case */
    f.payload      = static_cast<uint8_t*>(std::malloc(4));
    std::memcpy(f.payload, "data", 4);
    f.payload_size = 4;
    std::lock_guard<std::mutex> lk(p.ba.mu);
    p.ba.queue.emplace_back(std::move(f));
    p.ba.cv.notify_all();
  }

  for (int i = 0; i < 100 && !state.got; ++i) {
    pcl_executor_spin_once(p.exec_a, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(state.got) << "subscriber dropped a PUBLISH with NULL type_name";

  pcl_executor_remove(p.exec_a, sub_c);
  pcl_container_destroy(sub_c);
  restore_logs();
}

// Regression: invoke_async must surface PCL_ERR_STATE when the send
// thread is already stopping, instead of silently leaking a pending
// callback that will never fire.
TEST(PclTransportTemplate, InvokeAsyncFailsWhenSendStopped) {
  silence_logs();

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  LoopbackLink loopA, loopB;
  LoopbackEnd  end{ &loopA, &loopB };
  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &end;
  hooks.send_blocking = &loopback_send;
  hooks.recv_blocking = &loopback_recv;
  hooks.wake          = &loopback_wake;
  auto* t = pcl_transport_template_create(&hooks, e);
  ASSERT_NE(t, nullptr);

  const pcl_transport_t* vt = pcl_transport_template_get_transport(t);

  /* Drive shutdown manually: this is what pcl_executor_destroy ends up
   * doing.  After it the send thread refuses new frames. */
  vt->shutdown(vt->adapter_ctx);

  std::atomic<bool> cb_fired{false};
  pcl_msg_t req = {};
  const char* p = "x";
  req.data      = p;
  req.size      = 1;
  req.type_name = "T";

  pcl_status_t rc = vt->invoke_async(vt->adapter_ctx, "svc", &req,
      [](const pcl_msg_t*, void* ud) { *static_cast<std::atomic<bool>*>(ud) = true; },
      &cb_fired);
  EXPECT_EQ(rc, PCL_ERR_STATE)
      << "invoke_async must report failure when the send queue has stopped";
  EXPECT_FALSE(cb_fired.load());

  pcl_transport_template_destroy(t);
  pcl_executor_destroy(e);
  restore_logs();
}

// Regression: inbound SVC_REQ from a remote peer must be dispatched
// through the remote-aware executor path so that services configured
// LOCAL-only (or restricted to a different peer allow-list) are not
// silently exposed.  The previous implementation routed via
// pcl_executor_post_service_request(), which always treats requests
// as local -- bypassing remote-exposure rules.
TEST(PclTransportTemplate, RemoteSvcReqHonoursLocalOnlyExposure) {
  silence_logs();
  LoopbackPair p;

  /* Server-side container: registers a service marked LOCAL-only.
   * A request that arrives from node_a (the remote peer) should NOT
   * reach the handler -- the executor must drop it as not-exposed. */
  std::atomic<int> handler_calls{0};
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_port_t* port = pcl_container_add_service(
        c, "secure/svc", "SvcType",
        [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t* resp,
           pcl_svc_context_t*, void* svc_ud) -> pcl_status_t {
          static_cast<std::atomic<int>*>(svc_ud)->fetch_add(1);
          static const char body[] = "leaked";
          resp->data      = body;
          resp->size      = (uint32_t)sizeof(body) - 1u;
          resp->type_name = "SvcType";
          return PCL_OK;
        }, ud);
    pcl_port_set_route(port, PCL_ROUTE_LOCAL, nullptr, 0);  /* LOCAL only */
    return PCL_OK;
  };
  auto* svc_c = pcl_container_create("local_only_svc", &cbs, &handler_calls);
  ASSERT_EQ(pcl_container_configure(svc_c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc_c),  PCL_OK);
  ASSERT_EQ(pcl_executor_add(p.exec_b, svc_c), PCL_OK);

  /* Caller fires the request from exec_a via the transport. */
  struct ClientCtx {
    std::atomic<bool> got_response{false};
    std::string       payload;
  } client;

  pcl_msg_t req = {};
  const char* payload = "ping";
  req.data      = payload;
  req.size      = (uint32_t)strlen(payload);
  req.type_name = "SvcType";

  const pcl_transport_t* vt_a =
      pcl_transport_template_get_transport(p.tpl_a);
  ASSERT_EQ(vt_a->invoke_async(vt_a->adapter_ctx, "secure/svc", &req,
      [](const pcl_msg_t* resp, void* ud) {
        auto* c = static_cast<ClientCtx*>(ud);
        if (resp && resp->data && resp->size) {
          c->payload.assign(static_cast<const char*>(resp->data), resp->size);
        }
        c->got_response = true;
      },
      &client), PCL_OK);

  /* The executor responds with an empty message when the service is
   * not exposed -- give the round trip plenty of time to complete. */
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(2);
  while (!client.got_response && std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(p.exec_a, 0);
    pcl_executor_spin_once(p.exec_b, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(handler_calls.load(), 0)
      << "LOCAL-only service must not be invoked by remote SVC_REQ";
  EXPECT_TRUE(client.got_response.load())
      << "caller must always observe a response (empty when not exposed)";
  EXPECT_TRUE(client.payload.empty())
      << "response must be empty when the service is not exposed remotely";

  pcl_executor_remove(p.exec_b, svc_c);
  pcl_container_destroy(svc_c);
  restore_logs();
}

// Regression: alias keys must be stored in the executor-truncated
// form.  set_peer_id snprintfs into a 64-byte buffer; if alias_remember
// kept the *raw* untruncated input, destroy()'s register_transport(NULL)
// would compare the long alias against the executor's truncated slot
// key and miss it -- leaving a dangling adapter_ctx in the executor.
TEST(PclTransportTemplate, DestroyClearsLongPeerIdAlias) {
  silence_logs();

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  LoopbackLink loopA, loopB;
  LoopbackEnd  end{ &loopA, &loopB };
  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &end;
  hooks.send_blocking = &loopback_send;
  hooks.recv_blocking = &loopback_recv;
  hooks.wake          = &loopback_wake;

  auto* t = pcl_transport_template_create(&hooks, e);
  ASSERT_NE(t, nullptr);

  /* peer_id deliberately exceeds the executor's 64-byte slot. */
  std::string long_id(80, 'p');
  ASSERT_EQ(pcl_transport_template_set_peer_id(t, long_id.c_str()), PCL_OK);
  pcl_executor_register_transport(e, long_id.c_str(),
      pcl_transport_template_get_transport(t));

  pcl_transport_template_destroy(t);
  /* If the alias key was stored untruncated, register_transport(NULL)
   * inside destroy() would have missed the executor slot and we would
   * crash here invoking tpl_shutdown on the freed adapter_ctx. */
  pcl_executor_destroy(e);
  restore_logs();
}

// Regression: destroy() must NOT clear the executor's default
// transport unless this template instance is *currently* installed
// as that default.  Otherwise destroying a template that was only
// registered as a named peer would silently wipe an unrelated
// default transport.
TEST(PclTransportTemplate, DestroyPreservesUnrelatedDefaultTransport) {
  silence_logs();

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  /* Build a tiny standalone "other" transport whose vtable points at
   * a stack-local adapter_ctx token.  We just need to observe whether
   * the executor's default slot still references it after destroying
   * the template. */
  int other_ctx = 0;
  pcl_transport_t other = {};
  other.adapter_ctx = &other_ctx;
  other.publish     = [](void*, const char*, const pcl_msg_t*) -> pcl_status_t {
    return PCL_OK;
  };
  other.shutdown    = [](void*) {};
  ASSERT_EQ(pcl_executor_set_transport(e, &other), PCL_OK);

  LoopbackLink loopA, loopB;
  LoopbackEnd  end{ &loopA, &loopB };
  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &end;
  hooks.send_blocking = &loopback_send;
  hooks.recv_blocking = &loopback_recv;
  hooks.wake          = &loopback_wake;

  auto* t = pcl_transport_template_create(&hooks, e);
  ASSERT_NE(t, nullptr);

  /* Template is only registered as a named peer -- never as default. */
  pcl_transport_template_set_peer_id(t, "tpl_peer");
  pcl_executor_register_transport(e, "tpl_peer",
      pcl_transport_template_get_transport(t));

  pcl_transport_template_destroy(t);

  /* The default must still point at `other`, not have been wiped. */
  const pcl_transport_t* current = pcl_executor_get_transport(e);
  ASSERT_NE(current, nullptr)
      << "destroying a named-peer-only template wiped the default transport";
  EXPECT_EQ(current->adapter_ctx, &other_ctx);

  pcl_executor_set_transport(e, nullptr);
  pcl_executor_destroy(e);
  restore_logs();
}

// ---------------------------------------------------------------------------
// Conformance -- the same suite any transport can extend.
// ---------------------------------------------------------------------------

TEST(PclTransportTemplate_Conformance, PublishRoundTrip) {
  silence_logs();
  LoopbackPair p;
  pcl_conformance::expectPublishRoundTrip(p.conformancePair_AtoB(),
                                          "telemetry/heartbeat",
                                          "Heartbeat",
                                          "tick-tock");
  restore_logs();
}

TEST(PclTransportTemplate_Conformance, ServiceRoundTrip) {
  silence_logs();
  LoopbackPair p;
  pcl_conformance::expectServiceRoundTrip(p.conformancePair_AtoB(),
                                          "math/echo",
                                          "ping",
                                          "pong");
  restore_logs();
}

TEST(PclTransportTemplate_Conformance, MultiplePublishesPreserveOrder) {
  silence_logs();
  LoopbackPair p;
  for (int i = 0; i < 3; ++i) {
    pcl_conformance::expectPublishRoundTrip(p.conformancePair_AtoB(),
                                            "stream/data",
                                            "Sample",
                                            std::string("frame-") + std::to_string(i));
  }
  restore_logs();
}

// ---------------------------------------------------------------------------
// Additional scaffold edge cases (statement-coverage driven)
// ---------------------------------------------------------------------------

///< REQ_PCL_295: subscribe is a wire-agnostic no-op in the scaffold.
TEST(PclTransportTemplate, SubscribeVtableIsNoOp) {
  LoopbackPair pair;
  const pcl_transport_t* vt = pcl_transport_template_get_transport(pair.tpl_a);
  ASSERT_NE(vt, nullptr);
  ASSERT_NE(vt->subscribe, nullptr);
  EXPECT_EQ(vt->subscribe(vt->adapter_ctx, "some/topic", "SomeType"), PCL_OK);
}

///< REQ_PCL_286: re-setting the same peer id is idempotent (alias already
///< tracked).
TEST(PclTransportTemplate, SetPeerIdSameValueIsIdempotent) {
  LoopbackPair pair;
  EXPECT_EQ(pcl_transport_template_set_peer_id(pair.tpl_a, "node_b"), PCL_OK);
  EXPECT_EQ(pcl_transport_template_set_peer_id(pair.tpl_a, "node_b"), PCL_OK);
}

namespace {

extern "C" pcl_status_t failing_send(void* ud, const pcl_template_frame_t* f) {
  auto* count = static_cast<std::atomic<int>*>(ud);
  (void)f;
  count->fetch_add(1);
  return PCL_ERR_STATE;
}

extern "C" pcl_status_t idle_recv(void* ud, pcl_template_frame_t* out,
                                  uint32_t timeout_ms) {
  (void)ud; (void)out;
  std::this_thread::sleep_for(std::chrono::milliseconds(
      std::min<uint32_t>(timeout_ms, 5u)));
  return PCL_ERR_TIMEOUT;
}

extern "C" pcl_status_t broken_recv(void* ud, pcl_template_frame_t* out,
                                    uint32_t timeout_ms) {
  auto* count = static_cast<std::atomic<int>*>(ud);
  (void)out; (void)timeout_ms;
  count->fetch_add(1);
  // Permanently broken wire stack: a hard error, not a timeout.  Slow the
  // retry loop down so the test does not busy-spin.
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return PCL_ERR_STATE;
}

}  // namespace

///< REQ_PCL_292: a failing send hook drops the frame (with a warning) and the
///< send thread keeps servicing later frames.
TEST(PclTransportTemplate, SendHookFailureDropsFrame) {
  silence_logs();
  std::atomic<int> send_attempts{0};

  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &send_attempts;
  hooks.send_blocking = &failing_send;
  hooks.recv_blocking = &idle_recv;

  pcl_transport_template_t* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);

  const pcl_transport_t* vt = pcl_transport_template_get_transport(tpl);
  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1u;
  msg.type_name = "T";
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "topic/a", &msg), PCL_OK);
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "topic/b", &msg), PCL_OK);

  // Both frames reach the failing hook (dropped, not wedged).
  for (int i = 0; i < 200 && send_attempts.load() < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_GE(send_attempts.load(), 2);

  pcl_transport_template_destroy(tpl);
  pcl_executor_destroy(exec);
  restore_logs();
}

///< REQ_PCL_293: a hard recv_blocking failure logs and retries; destroy still
///< tears the transport down cleanly.
TEST(PclTransportTemplate, RecvHookHardFailureRetriesUntilDestroy) {
  silence_logs();
  std::atomic<int> recv_attempts{0};

  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &recv_attempts;
  hooks.send_blocking = &failing_send;  // unused
  hooks.recv_blocking = &broken_recv;

  pcl_transport_template_t* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);

  for (int i = 0; i < 200 && recv_attempts.load() < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_GE(recv_attempts.load(), 3);

  pcl_transport_template_destroy(tpl);
  pcl_executor_destroy(exec);
  restore_logs();
}

///< REQ_PCL_294: inbound frames with an unknown kind, or an SVC_RESP whose
///< seq id matches no pending request, are ignored without crashing.
TEST(PclTransportTemplate, MalformedInboundFramesIgnored) {
  silence_logs();
  LoopbackPair pair;

  // Frame with a nonsense kind.
  {
    OwnedFrame f;
    f.kind = static_cast<pcl_template_frame_kind_t>(0x7eadbeef);
    f.seq_id = 1u;
    f.topic = OwnedFrame::dup("junk");
    std::lock_guard<std::mutex> lk(pair.ab.mu);
    pair.ab.queue.emplace_back(std::move(f));
    pair.ab.cv.notify_all();
  }
  // SVC_RESP nobody is waiting for.
  {
    OwnedFrame f;
    f.kind = PCL_TEMPLATE_FRAME_SVC_RESP;
    f.seq_id = 0x12345u;
    std::lock_guard<std::mutex> lk(pair.ab.mu);
    pair.ab.queue.emplace_back(std::move(f));
    pair.ab.cv.notify_all();
  }
  // SVC_REQ with no service name: the dispatch post is rejected and the
  // per-request slot is reclaimed.
  {
    OwnedFrame f;
    f.kind = PCL_TEMPLATE_FRAME_SVC_REQ;
    f.seq_id = 7u;
    f.service_name = nullptr;
    std::lock_guard<std::mutex> lk(pair.ab.mu);
    pair.ab.queue.emplace_back(std::move(f));
    pair.ab.cv.notify_all();
  }

  // Give the recv thread time to consume all three frames.
  for (int i = 0; i < 200 && pair.b_end.recv_count.load() < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_GE(pair.b_end.recv_count.load(), 3);
  restore_logs();
}

namespace {

struct GatedSend {
  std::mutex mu;
  std::condition_variable cv;
  bool release = false;
  std::atomic<int> entered{0};
};

extern "C" pcl_status_t gated_send(void* ud, const pcl_template_frame_t* f) {
  auto* gate = static_cast<GatedSend*>(ud);
  (void)f;
  gate->entered.fetch_add(1);
  // Hold the send thread long enough for destroy() to raise the stop flag
  // while frames are still queued behind this one.
  std::unique_lock<std::mutex> lk(gate->mu);
  gate->cv.wait_for(lk, std::chrono::milliseconds(300),
                    [&] { return gate->release; });
  return PCL_OK;
}

}  // namespace

///< REQ_PCL_290: frames queued behind a slow send hook accumulate in FIFO
///< order and destroy() drains the backlog without leaking.
TEST(PclTransportTemplate, DestroyDrainsQueuedFramesBehindSlowSend) {
  silence_logs();
  GatedSend gate;

  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &gate;
  hooks.send_blocking = &gated_send;
  hooks.recv_blocking = &idle_recv;

  pcl_transport_template_t* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);

  const pcl_transport_t* vt = pcl_transport_template_get_transport(tpl);
  pcl_msg_t msg = {};
  msg.data = "y";
  msg.size = 1u;
  msg.type_name = "T";

  // First publish occupies the send hook; the rest stack up in the queue.
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/0", &msg), PCL_OK);
  for (int i = 0; i < 100 && gate.entered.load() < 1; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/1", &msg), PCL_OK);
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/2", &msg), PCL_OK);

  // Destroy while the hook still holds the first frame: the stop flag is
  // observed with frames queued, so the abort path and the exit drain both
  // run instead of the normal send loop.
  pcl_transport_template_destroy(tpl);
  pcl_executor_destroy(exec);
  restore_logs();
}

///< REQ_PCL_290: once the vtable shutdown has stopped the send thread,
///< publish and invoke_async fail with PCL_ERR_STATE and a pending async
///< entry is rolled back rather than orphaned.
TEST(PclTransportTemplate, PublishAndInvokeFailAfterVtableShutdown) {
  LoopbackPair pair;
  const pcl_transport_t* vt = pcl_transport_template_get_transport(pair.tpl_a);
  ASSERT_NE(vt->shutdown, nullptr);
  vt->shutdown(vt->adapter_ctx);

  pcl_msg_t msg = {};
  msg.data = "z";
  msg.size = 1u;
  msg.type_name = "T";
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "topic", &msg), PCL_ERR_STATE);

  ASSERT_NE(vt->invoke_async, nullptr);
  auto cb = [](const pcl_msg_t*, void*) {};
  EXPECT_EQ(vt->invoke_async(vt->adapter_ctx, "svc", &msg, cb, nullptr),
            PCL_ERR_STATE);
}

///< REQ_PCL_288: destroying a transport that *is* the executor's default
///< transport clears the default slot.
TEST(PclTransportTemplate, DestroyClearsDefaultTransportWhenSelf) {
  std::atomic<int> send_attempts{0};
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &send_attempts;
  hooks.send_blocking = &failing_send;
  hooks.recv_blocking = &idle_recv;

  pcl_transport_template_t* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);
  ASSERT_EQ(pcl_executor_set_transport(
                exec, pcl_transport_template_get_transport(tpl)),
            PCL_OK);

  pcl_transport_template_destroy(tpl);
  EXPECT_EQ(pcl_executor_get_transport(exec), nullptr);
  pcl_executor_destroy(exec);
}

///< REQ_PCL_290: destroying a transport with un-responded async requests
///< drains and frees the pending correlation entries.
TEST(PclTransportTemplate, DestroyFreesUnansweredPendingRequests) {
  silence_logs();
  std::atomic<int> send_attempts{0};

  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data     = &send_attempts;
  hooks.send_blocking = &failing_send;  // request leaves, response never comes
  hooks.recv_blocking = &idle_recv;

  pcl_transport_template_t* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);

  const pcl_transport_t* vt = pcl_transport_template_get_transport(tpl);
  pcl_msg_t req = {};
  req.data = "q";
  req.size = 1u;
  req.type_name = "T";
  auto cb = [](const pcl_msg_t*, void*) {};
  EXPECT_EQ(vt->invoke_async(vt->adapter_ctx, "svc/one", &req, cb, nullptr),
            PCL_OK);
  EXPECT_EQ(vt->invoke_async(vt->adapter_ctx, "svc/two", &req, cb, nullptr),
            PCL_OK);

  // Destroy with both requests still pending: the drain frees the entries.
  pcl_transport_template_destroy(tpl);
  pcl_executor_destroy(exec);
  restore_logs();
}
