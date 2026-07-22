/// \file test_pcl_native_binding.cpp
/// \brief Tests for the high-efficiency in-process (Tier-A native) binding
///        path: the raw-struct payload sentinel, the fail-closed egress guards
///        (N2), the transport-ingress provenance guards (N2b), the
///        forbid-mixed-route rule (N6), and native service response routing
///        (N4). See doc/plans/PYRAMID/high_efficiency_process_bindings_plan.md.
#include <gtest/gtest.h>

extern "C" {
#include "pcl/pcl_executor.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_types.h"
}

// The standard binding-use surface: the C++ port abstraction.
#include "pcl/component.hpp"
#include "pcl/executor.hpp"

namespace {

// A live native object handed over by pointer instead of serialized.
struct WorldState {
  int    a;
  double b;
};

const char* kNativeType = PCL_NATIVE_CONTENT_TYPE ";WorldState";

pcl_msg_t make_native(const WorldState* ws) {
  pcl_msg_t m = {};
  m.data = ws;
  m.size = static_cast<uint32_t>(sizeof(*ws));
  m.type_name = kNativeType;
  return m;
}

pcl_msg_t make_serialized(const void* data, uint32_t size) {
  pcl_msg_t m = {};
  m.data = data;
  m.size = size;
  m.type_name = "WorldState";
  return m;
}

// -- Capture transport: records whether an egress hook was ever reached ------
struct CaptureTransport {
  int publish_count = 0;
  int invoke_count = 0;
  int respond_count = 0;
};

pcl_status_t cap_publish(void* ctx, const char*, const pcl_msg_t*) {
  static_cast<CaptureTransport*>(ctx)->publish_count++;
  return PCL_OK;
}
pcl_status_t cap_invoke_async(void* ctx, const char*, const pcl_msg_t*,
                              pcl_resp_cb_fn_t cb, void* ud) {
  static_cast<CaptureTransport*>(ctx)->invoke_count++;
  if (cb) { pcl_msg_t r = {}; cb(&r, ud); }
  return PCL_OK;
}
pcl_status_t cap_respond(void* ctx, pcl_svc_context_t*, const pcl_msg_t*) {
  static_cast<CaptureTransport*>(ctx)->respond_count++;
  return PCL_OK;
}

pcl_transport_t capture_vtable(CaptureTransport* state) {
  pcl_transport_t t = {};
  t.publish = cap_publish;
  t.invoke_async = cap_invoke_async;
  t.respond = cap_respond;
  t.adapter_ctx = state;
  return t;
}

// -- Subscriber that casts the native pointer back --------------------------
struct SubState {
  int  got = 0;
  bool saw_native = false;
  int  world_a = 0;
};

void sub_cb(pcl_container_t*, const pcl_msg_t* m, void* ud) {
  auto* s = static_cast<SubState*>(ud);
  s->got++;
  if (pcl_msg_is_native(m)) {
    s->saw_native = true;
    s->world_a = static_cast<const WorldState*>(m->data)->a;
  }
}

// Publisher/subscriber container built with a chosen publisher route.
struct PubCfg {
  SubState*          sub;
  uint32_t           route;
  const char* const* peers;
  uint32_t           peer_count;
  pcl_port_t*        pub;  // out
};

pcl_status_t configure_pub(pcl_container_t* c, void* ud) {
  auto* cfg = static_cast<PubCfg*>(ud);
  pcl_port_t* sub = pcl_container_add_subscriber(c, "world", "WorldState",
                                                 sub_cb, cfg->sub);
  pcl_port_t* pub = pcl_container_add_publisher(c, "world", "WorldState");
  if (!sub || !pub) return PCL_ERR_NOMEM;
  pcl_port_set_route(sub, PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, nullptr, 0);
  cfg->pub = pub;
  return pcl_port_set_route(pub, cfg->route, cfg->peers, cfg->peer_count);
}

// Fan-out subscriber that records the object address it was handed, so a test
// can prove every subscriber shares one object (no per-subscriber copy).
struct FanoutSub {
  int         got = 0;
  int         world_a = 0;
  const void* addr = nullptr;
};

void fanout_sub_cb(pcl_container_t*, const pcl_msg_t* m, void* ud) {
  auto* s = static_cast<FanoutSub*>(ud);
  s->got++;
  if (const WorldState* w = static_cast<const WorldState*>(
          pcl_msg_is_native(m) ? m->data : nullptr)) {
    s->world_a = w->a;
    s->addr = m->data;
  }
}

}  // namespace

// -- Sentinel helper --------------------------------------------------------

TEST(PclNativeBinding, HelperRecognisesSentinel) {
  WorldState ws{1, 2.0};
  pcl_msg_t native = make_native(&ws);
  EXPECT_TRUE(pcl_msg_is_native(&native));

  pcl_msg_t bare = {};
  bare.type_name = PCL_NATIVE_CONTENT_TYPE;  // no schema suffix
  EXPECT_TRUE(pcl_msg_is_native(&bare));

  pcl_msg_t ser = make_serialized("x", 1);
  EXPECT_FALSE(pcl_msg_is_native(&ser));

  pcl_msg_t none = {};
  EXPECT_FALSE(pcl_msg_is_native(&none));
}

// -- Tier-A native local publish (N3a substrate) ----------------------------

TEST(PclNativeBinding, LocalNativePublishDeliversByPointer) {
  SubState sub;
  PubCfg cfg{&sub, PCL_ROUTE_LOCAL, nullptr, 0, nullptr};
  pcl_callbacks_t cbs = {};
  cbs.on_configure = configure_pub;
  auto* c = pcl_container_create("p", &cbs, &cfg);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);
  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);

  WorldState ws{42, 3.14};
  pcl_msg_t native = make_native(&ws);
  EXPECT_EQ(pcl_port_publish(cfg.pub, &native), PCL_OK);

  EXPECT_EQ(sub.got, 1);
  EXPECT_TRUE(sub.saw_native);
  EXPECT_EQ(sub.world_a, 42);  // the live object arrived by pointer, no decode

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- Local native fan-out: one const object shared by many subscribers -------

TEST(PclNativeBinding, LocalNativePublishFansOutToManySubscribersZeroCopy) {
  FanoutSub s1, s2, s3;
  struct Cfg { FanoutSub* a; FanoutSub* b; FanoutSub* c; pcl_port_t* pub; } cfg{
      &s1, &s2, &s3, nullptr};
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* cfg = static_cast<Cfg*>(ud);
    // Three subscribers on one topic, plus the publisher, all local.
    pcl_container_add_subscriber(c, "world", "WorldState", fanout_sub_cb, cfg->a);
    pcl_container_add_subscriber(c, "world", "WorldState", fanout_sub_cb, cfg->b);
    pcl_container_add_subscriber(c, "world", "WorldState", fanout_sub_cb, cfg->c);
    cfg->pub = pcl_container_add_publisher(c, "world", "WorldState");
    return cfg->pub ? pcl_port_set_route(cfg->pub, PCL_ROUTE_LOCAL, nullptr, 0)
                    : PCL_ERR_NOMEM;
  };
  auto* c = pcl_container_create("fan", &cbs, &cfg);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);
  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);

  WorldState ws{88, 1.0};
  pcl_msg_t native = make_native(&ws);
  EXPECT_EQ(pcl_port_publish(cfg.pub, &native), PCL_OK);

  // Every subscriber ran once, saw the same value, and -- the zero-copy claim --
  // was handed the *same* live object address (&ws), no per-subscriber copy.
  EXPECT_EQ(s1.got, 1);
  EXPECT_EQ(s2.got, 1);
  EXPECT_EQ(s3.got, 1);
  EXPECT_EQ(s1.world_a, 88);
  EXPECT_EQ(s2.world_a, 88);
  EXPECT_EQ(s3.world_a, 88);
  EXPECT_EQ(s1.addr, &ws);
  EXPECT_EQ(s2.addr, &ws);
  EXPECT_EQ(s3.addr, &ws);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- Multiple port instances on one topic, each with its own route -----------
// Proves per-port routing works today and that two ports can carry different
// representations: a native-local port and a serialized-remote port together
// express a mixed fan-out without the single-port mixed route N6 forbids.

TEST(PclNativeBinding, MultiplePortInstancesPerTopicCarryDifferentRepresentations) {
  struct Cfg {
    FanoutSub*  sub;
    pcl_port_t* pub_local  = nullptr;  // native, local only
    pcl_port_t* pub_remote = nullptr;  // serialized, remote only
  } cfg;
  FanoutSub sub;
  cfg.sub = &sub;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* cfg = static_cast<Cfg*>(ud);
    pcl_port_t* s = pcl_container_add_subscriber(c, "world", "WorldState",
                                                 fanout_sub_cb, cfg->sub);
    // Two publisher ports, same topic name, different routes -- no uniqueness
    // constraint prevents this.
    cfg->pub_local  = pcl_container_add_publisher(c, "world", "WorldState");
    cfg->pub_remote = pcl_container_add_publisher(c, "world", "WorldState");
    if (!s || !cfg->pub_local || !cfg->pub_remote) return PCL_ERR_NOMEM;
    pcl_port_set_route(s, PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, nullptr, 0);
    const char* peers[] = {"peer_a"};
    pcl_port_set_route(cfg->pub_local, PCL_ROUTE_LOCAL, nullptr, 0);
    return pcl_port_set_route(cfg->pub_remote, PCL_ROUTE_REMOTE, peers, 1);
  };
  auto* c = pcl_container_create("multi", &cbs, &cfg);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);
  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);
  CaptureTransport cap;
  pcl_transport_t tr = capture_vtable(&cap);
  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", &tr), PCL_OK);

  // Native on the local port: delivered by pointer, nothing to the wire.
  WorldState ws{77, 1.0};
  pcl_msg_t native = make_native(&ws);
  EXPECT_EQ(pcl_port_publish(cfg.pub_local, &native), PCL_OK);
  EXPECT_EQ(sub.got, 1);
  EXPECT_EQ(sub.addr, &ws);
  EXPECT_EQ(cap.publish_count, 0);

  // Serialized on the remote port: reaches the transport, no extra local
  // delivery (the remote-only port has no local leg).
  const char blob[] = "x";
  pcl_msg_t ser = make_serialized(blob, 1);
  EXPECT_EQ(pcl_port_publish(cfg.pub_remote, &ser), PCL_OK);
  EXPECT_EQ(cap.publish_count, 1);
  EXPECT_EQ(sub.got, 1);  // unchanged: remote-only port did not dispatch locally

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- N2 + N6: native publish on a REMOTE-inclusive route is refused up front -

TEST(PclNativeBinding, NativePublishOnMixedRouteRefusedBeforeLocalDelivery) {
  SubState sub;
  const char* peers[] = {"peer_a"};
  PubCfg cfg{&sub, PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, peers, 1, nullptr};
  pcl_callbacks_t cbs = {};
  cbs.on_configure = configure_pub;
  auto* c = pcl_container_create("p", &cbs, &cfg);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);
  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);
  CaptureTransport cap;
  pcl_transport_t tr = capture_vtable(&cap);
  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", &tr), PCL_OK);

  WorldState ws{7, 1.0};
  pcl_msg_t native = make_native(&ws);
  EXPECT_EQ(pcl_port_publish(cfg.pub, &native), PCL_ERR_INVALID);
  EXPECT_EQ(sub.got, 0);            // N6: no local side effect leaked past guard
  EXPECT_EQ(cap.publish_count, 0);  // N2: nothing reached the transport

  // A serialized publish on the same mixed route still fans out both legs.
  const char blob[] = "x";
  pcl_msg_t ser = make_serialized(blob, 1);
  EXPECT_EQ(pcl_port_publish(cfg.pub, &ser), PCL_OK);
  EXPECT_EQ(sub.got, 1);
  EXPECT_EQ(cap.publish_count, 1);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- N2b: transport-ingress provenance guards -------------------------------

TEST(PclNativeBinding, IngressQueuesRejectNativeSentinel) {
  WorldState ws{5, 0.0};
  pcl_msg_t native = make_native(&ws);
  auto* e = pcl_executor_create();

  // pub/sub cross-thread ingress (local and remote)
  EXPECT_EQ(pcl_executor_post_incoming(e, "world", &native), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_post_remote_incoming(e, "peer_a", "world", &native),
            PCL_ERR_INVALID);
  // service request cross-thread ingress (local and remote)
  auto noop_resp = [](const pcl_msg_t*, void*) {};
  EXPECT_EQ(pcl_executor_post_service_request(e, "svc", &native, noop_resp, nullptr),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_post_service_request_remote(e, "peer_a", "svc", &native,
                                                     noop_resp, nullptr),
            PCL_ERR_INVALID);
  // response cross-thread ingress (transport recv thread path)
  EXPECT_EQ(pcl_executor_post_response_msg(e, noop_resp, nullptr, &native),
            PCL_ERR_INVALID);

  // A serialized message on the same queues is still accepted.
  const char blob[] = "x";
  pcl_msg_t ser = make_serialized(blob, 1);
  EXPECT_EQ(pcl_executor_post_incoming(e, "world", &ser), PCL_OK);

  pcl_executor_destroy(e);
}

// -- N2: native request never reaches a transport ---------------------------

TEST(PclNativeBinding, NativeRequestRefusedOnDefaultTransport) {
  CaptureTransport cap;
  pcl_transport_t tr = capture_vtable(&cap);
  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_set_transport(e, &tr), PCL_OK);

  WorldState ws{9, 0.0};
  pcl_msg_t native = make_native(&ws);
  bool fired = false;
  auto cb = [](const pcl_msg_t*, void* ud) { *static_cast<bool*>(ud) = true; };

  // No local route -> would fall through to the default transport; refused.
  EXPECT_EQ(pcl_executor_invoke_async(e, "svc", &native, cb, &fired),
            PCL_ERR_INVALID);
  EXPECT_EQ(cap.invoke_count, 0);
  EXPECT_FALSE(fired);

  pcl_executor_destroy(e);
}

// -- N4: native unary service works locally even with a transport installed --

namespace {
struct EchoService {
  WorldState owned_response{0, 0.0};  // owned slot -- outlives handler return
  bool deferred = false;
  pcl_svc_context_t* saved_ctx = nullptr;
};

// Immediate native reply: point the response at an owned object.
pcl_status_t echo_handler(pcl_container_t*, const pcl_msg_t* req,
                          pcl_msg_t* resp, pcl_svc_context_t* ctx, void* ud) {
  auto* svc = static_cast<EchoService*>(ud);
  const auto* in = static_cast<const WorldState*>(req->data);
  svc->owned_response.a = in->a + 1;
  if (svc->deferred) {
    svc->saved_ctx = ctx;
    return PCL_PENDING;
  }
  resp->data = &svc->owned_response;
  resp->size = sizeof(svc->owned_response);
  resp->type_name = kNativeType;
  return PCL_OK;
}

pcl_status_t configure_service(pcl_container_t* c, void* ud) {
  pcl_port_t* p = pcl_container_add_service(c, "echo", "WorldState",
                                            echo_handler, ud);
  if (!p) return PCL_ERR_NOMEM;
  return pcl_port_set_route(p, PCL_ROUTE_LOCAL, nullptr, 0);
}
}  // namespace

TEST(PclNativeBinding, NativeUnaryServiceLocalRouteBypassesTransport) {
  EchoService svc;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = configure_service;
  auto* c = pcl_container_create("svc", &cbs, &svc);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);

  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);

  // A default transport is installed, but an explicit LOCAL route on the
  // consumed endpoint forces the intra-process path.
  CaptureTransport cap;
  pcl_transport_t tr = capture_vtable(&cap);
  ASSERT_EQ(pcl_executor_set_transport(e, &tr), PCL_OK);
  pcl_endpoint_route_t route = {};
  route.endpoint_name = "echo";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_LOCAL;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  WorldState req{100, 0.0};
  pcl_msg_t native_req = make_native(&req);
  struct Result { bool fired = false; bool native = false; int a = 0; } res;
  auto cb = [](const pcl_msg_t* r, void* ud) {
    auto* out = static_cast<Result*>(ud);
    out->fired = true;
    out->native = pcl_msg_is_native(r);
    if (out->native) out->a = static_cast<const WorldState*>(r->data)->a;
  };
  EXPECT_EQ(pcl_executor_invoke_async(e, "echo", &native_req, cb, &res), PCL_OK);

  EXPECT_TRUE(res.fired);
  EXPECT_TRUE(res.native);
  EXPECT_EQ(res.a, 101);            // native response delivered, no decode
  EXPECT_EQ(cap.invoke_count, 0);   // transport never consulted

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- Port abstraction: publishNative / nativePayload (standard surface) ------

namespace {
// A component that publishes and subscribes to "world" natively, using only
// the standard C++ port abstraction (pcl::Port), not the generated facade.
class NativePortComponent : public pcl::Component {
public:
  explicit NativePortComponent(SubState* sub)
      : pcl::Component("native_port"), sub_(sub) {}

  pcl::Port pub;

protected:
  pcl_status_t on_configure() override {
    sub_port_ = addSubscriber(
        "world", "WorldState",
        [](pcl_container_t*, const pcl_msg_t* msg, void* ud) {
          auto* s = static_cast<SubState*>(ud);
          s->got++;
          // Standard read helper: typed pointer for native, nullptr otherwise.
          if (const WorldState* w = pcl::nativePayload<WorldState>(msg)) {
            s->saw_native = true;
            s->world_a = w->a;
          }
        },
        sub_);
    pub = addPublisher("world", "WorldState");
    if (!sub_port_ || !pub) return PCL_ERR_NOMEM;
    sub_port_.routeLocal();
    return pub.routeLocal();
  }

private:
  SubState*  sub_;
  pcl::Port  sub_port_;
};
}  // namespace

TEST(PclNativeBinding, PortAbstractionPublishNativeDeliversByPointer) {
  SubState sub;
  NativePortComponent comp(&sub);
  ASSERT_EQ(comp.configure(), PCL_OK);
  ASSERT_EQ(comp.activate(), PCL_OK);

  pcl::Executor exec;
  ASSERT_EQ(exec.add(comp), PCL_OK);

  WorldState ws{55, 2.5};
  // The standard port surface -- typed native publish, no generated facade.
  EXPECT_EQ(comp.pub.publishNative(ws, "WorldState"), PCL_OK);

  EXPECT_EQ(sub.got, 1);
  EXPECT_TRUE(sub.saw_native);
  EXPECT_EQ(sub.world_a, 55);
}

TEST(PclNativeBinding, PortAbstractionNativeRefusedOnRemoteRoute) {
  SubState sub;
  NativePortComponent comp(&sub);
  ASSERT_EQ(comp.configure(), PCL_OK);
  ASSERT_EQ(comp.activate(), PCL_OK);

  pcl::Executor exec;
  ASSERT_EQ(exec.add(comp), PCL_OK);
  CaptureTransport cap;
  pcl_transport_t tr = capture_vtable(&cap);
  ASSERT_EQ(exec.registerTransport("peer_a", &tr), PCL_OK);

  // Re-route the publisher to include a remote leg; a native publish must then
  // be refused up front, before any local delivery, and never reach the wire.
  ASSERT_EQ(comp.pub.routeLocalAndRemote("peer_a"), PCL_OK);

  WorldState ws{1, 0.0};
  EXPECT_EQ(comp.pub.publishNative(ws, "WorldState"), PCL_ERR_INVALID);
  EXPECT_EQ(sub.got, 0);
  EXPECT_EQ(cap.publish_count, 0);
}

// -- N4: deferred native reply bypasses the executor-default transport ------

TEST(PclNativeBinding, NativeDeferredReplyBypassesDefaultTransport) {
  EchoService svc;
  svc.deferred = true;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = configure_service;
  auto* c = pcl_container_create("svc", &cbs, &svc);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);

  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);
  CaptureTransport cap;
  pcl_transport_t tr = capture_vtable(&cap);
  ASSERT_EQ(pcl_executor_set_transport(e, &tr), PCL_OK);
  pcl_endpoint_route_t route = {};
  route.endpoint_name = "echo";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_LOCAL;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  WorldState req{200, 0.0};
  pcl_msg_t native_req = make_native(&req);
  struct Result { bool fired = false; bool native = false; int a = 0; } res;
  auto cb = [](const pcl_msg_t* r, void* ud) {
    auto* out = static_cast<Result*>(ud);
    out->fired = true;
    out->native = pcl_msg_is_native(r);
    if (out->native) out->a = static_cast<const WorldState*>(r->data)->a;
  };
  // Handler returns PCL_PENDING; nothing delivered yet.
  EXPECT_EQ(pcl_executor_invoke_async(e, "echo", &native_req, cb, &res), PCL_OK);
  EXPECT_FALSE(res.fired);
  ASSERT_NE(svc.saved_ctx, nullptr);

  // Complete the deferred reply with a native response. Even though a default
  // transport with a respond hook is installed, the native reply must go
  // straight to the saved local callback, not the transport (N4).
  pcl_msg_t native_resp = make_native(&svc.owned_response);
  EXPECT_EQ(pcl_service_respond(svc.saved_ctx, &native_resp), PCL_OK);

  EXPECT_TRUE(res.fired);
  EXPECT_TRUE(res.native);
  EXPECT_EQ(res.a, 201);
  EXPECT_EQ(cap.respond_count, 0);  // transport.respond never called

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}
