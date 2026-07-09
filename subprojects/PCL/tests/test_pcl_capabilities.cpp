#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <string>

extern "C" {
#include "pcl/pcl_capabilities.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_plugin_loader.h"
#include "pcl/pcl_transport.h"
}

namespace {

// A vtable with every slot NULL; tests fill in the slots they care about.
pcl_transport_t EmptyVtable() { return pcl_transport_t{}; }

}  // namespace

// -- Derivation from vtable ----------------------------------------------

///< REQ_PCL_347: capability derivation from a NULL vtable returns PCL_CAP_NONE.
TEST(PclCapabilities, NullVtableHasNoCaps) {
  EXPECT_EQ(pcl_transport_caps_from_vtable(nullptr), PCL_CAP_NONE);
}

///< REQ_PCL_348: capability derivation from a vtable with every slot NULL returns PCL_CAP_NONE.
TEST(PclCapabilities, EmptyVtableHasNoCaps) {
  pcl_transport_t t = EmptyVtable();
  EXPECT_EQ(pcl_transport_caps_from_vtable(&t), PCL_CAP_NONE);
}

///< REQ_PCL_349: a non-NULL publish or subscribe slot derives PCL_CAP_PUBSUB.
TEST(PclCapabilities, PublishOrSubscribeImpliesPubsub) {
  pcl_transport_t pub = EmptyVtable();
  pub.publish = reinterpret_cast<decltype(pub.publish)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&pub), PCL_CAP_PUBSUB);

  pcl_transport_t sub = EmptyVtable();
  sub.subscribe = reinterpret_cast<decltype(sub.subscribe)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&sub), PCL_CAP_PUBSUB);
}

///< REQ_PCL_350: a non-NULL serve or invoke_async slot derives PCL_CAP_RPC_UNARY.
TEST(PclCapabilities, ServeOrInvokeAsyncImpliesUnary) {
  pcl_transport_t srv = EmptyVtable();
  srv.serve = reinterpret_cast<decltype(srv.serve)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&srv), PCL_CAP_RPC_UNARY);

  pcl_transport_t cli = EmptyVtable();
  cli.invoke_async = reinterpret_cast<decltype(cli.invoke_async)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&cli), PCL_CAP_RPC_UNARY);
}

///< REQ_PCL_351: a non-NULL invoke_stream or stream_send slot derives PCL_CAP_RPC_STREAM.
TEST(PclCapabilities, InvokeStreamOrStreamSendImpliesStream) {
  pcl_transport_t cli = EmptyVtable();
  cli.invoke_stream = reinterpret_cast<decltype(cli.invoke_stream)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&cli), PCL_CAP_RPC_STREAM);

  pcl_transport_t srv = EmptyVtable();
  srv.stream_send = reinterpret_cast<decltype(srv.stream_send)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&srv), PCL_CAP_RPC_STREAM);
}

///< REQ_PCL_352: PCL_CAP_RPC_ACTION is never derived from a vtable, regardless of which other slots are set.
TEST(PclCapabilities, ActionIsNeverDerived) {
  // Every slot non-NULL -- still no ACTION bit, which has no vtable slot.
  pcl_transport_t t = EmptyVtable();
  t.publish = reinterpret_cast<decltype(t.publish)>(0x1);
  t.serve = reinterpret_cast<decltype(t.serve)>(0x1);
  t.invoke_stream = reinterpret_cast<decltype(t.invoke_stream)>(0x1);
  pcl_transport_caps_t caps = pcl_transport_caps_from_vtable(&t);
  EXPECT_EQ(caps, PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM);
  EXPECT_EQ(caps & PCL_CAP_RPC_ACTION, 0u);
}

// -- Loader-resolved capabilities ----------------------------------------

///< REQ_PCL_353: pcl_plugin_transport_caps(NULL, ...) fails closed with PCL_ERR_INVALID.
TEST(PclCapabilities, LoaderNullHandleFailsClosed) {
  pcl_transport_caps_t caps = PCL_CAP_NONE;
  EXPECT_EQ(pcl_plugin_transport_caps(nullptr, nullptr, nullptr, &caps),
            PCL_ERR_INVALID);
}

///< REQ_PCL_354: the loader prefers a plugin's explicit pcl_transport_plugin_caps symbol over vtable derivation.
TEST(PclCapabilities, LoaderUsesExplicitCapsSymbol) {
  // The capture plugin exports pcl_transport_plugin_caps declaring an ACTION
  // capability that its vtable could never imply (no vtable slot exists for
  // it), so a returned ACTION bit proves the explicit symbol was consulted.
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(CAPTURE_PLUGIN_PATH, "{}",
                                      &handle, &transport),
            PCL_OK);

  pcl_transport_caps_t caps = PCL_CAP_NONE;
  ASSERT_EQ(pcl_plugin_transport_caps(handle, "{}", transport, &caps), PCL_OK);
  EXPECT_EQ(caps, PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM |
                      PCL_CAP_RPC_ACTION);

  pcl_plugin_unload(handle);
}

///< REQ_PCL_355: the loader falls back to vtable derivation when a plugin exports no caps symbol.
TEST(PclCapabilities, LoaderDerivesWhenSymbolAbsent) {
  // The no-caps fixture plugin exports no caps symbol, so the loader must fall
  // back to vtable derivation: publish + invoke_async => PUBSUB | RPC_UNARY.
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(NOCAPS_TRANSPORT_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_OK);

  pcl_transport_caps_t caps = PCL_CAP_NONE;
  ASSERT_EQ(pcl_plugin_transport_caps(handle, nullptr, transport, &caps),
            PCL_OK);
  EXPECT_EQ(caps, pcl_transport_caps_from_vtable(transport));
  EXPECT_EQ(caps, PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY);
  EXPECT_EQ(caps & PCL_CAP_RPC_ACTION, 0u);

  pcl_plugin_unload(handle);
}

// -- Shipped generic-plugin capability matrix (declared, config-free) ----
//
// The caps symbol does not require constructing a transport, so these assert
// the declared matrix via pcl_plugin_open + the symbol directly, with no
// per-plugin config. Each declared mask must also agree with what its honest
// vtable would derive (asserted separately where a vtable is available).

namespace {

pcl_transport_caps_t DeclaredCaps(const char* path) {
  pcl_plugin_handle_t* handle = pcl_plugin_open(path);
  EXPECT_NE(handle, nullptr) << path;
  if (!handle) return PCL_CAP_NONE;
  auto caps_fn = reinterpret_cast<pcl_transport_caps_t (*)(const char*)>(
      pcl_plugin_symbol(handle, "pcl_transport_plugin_caps"));
  EXPECT_NE(caps_fn, nullptr) << "missing caps symbol: " << path;
  pcl_transport_caps_t caps = caps_fn ? caps_fn(nullptr) : PCL_CAP_NONE;
  pcl_plugin_unload(handle);
  return caps;
}

pcl_qos_reliability_t DeclaredReliability(const char* path) {
  pcl_plugin_handle_t* handle = pcl_plugin_open(path);
  EXPECT_NE(handle, nullptr) << path;
  if (!handle) return PCL_QOS_RELIABILITY_UNSPECIFIED;
  pcl_qos_t qos{PCL_QOS_RELIABILITY_UNSPECIFIED};
  EXPECT_EQ(pcl_plugin_transport_qos(handle, nullptr, &qos), PCL_OK) << path;
  pcl_plugin_unload(handle);
  return qos.reliability;
}

}  // namespace

///< REQ_PCL_356: the UDP plugin declares PCL_CAP_PUBSUB only.
TEST(PclCapabilities, UdpPluginDeclaresPubsub) {
  EXPECT_EQ(DeclaredCaps(UDP_TRANSPORT_PLUGIN_PATH), PCL_CAP_PUBSUB);
}

///< REQ_PCL_357: the socket plugin declares PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY.
TEST(PclCapabilities, SocketPluginDeclaresPubsubUnary) {
  EXPECT_EQ(DeclaredCaps(SOCKET_TRANSPORT_PLUGIN_PATH),
            PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY);
}

///< REQ_PCL_358: the shared-memory plugin declares PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM.
TEST(PclCapabilities, ShmPluginDeclaresPubsubUnaryStream) {
  EXPECT_EQ(DeclaredCaps(SHM_TRANSPORT_PLUGIN_PATH),
            PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM);
}

// -- Shipped generic-plugin offered-QoS matrix (§2.D.4) ------------------

///< REQ_PCL_359: the UDP plugin declares PCL_QOS_RELIABILITY_BEST_EFFORT.
TEST(PclCapabilities, UdpPluginOffersBestEffort) {
  EXPECT_EQ(DeclaredReliability(UDP_TRANSPORT_PLUGIN_PATH),
            PCL_QOS_RELIABILITY_BEST_EFFORT);
}

///< REQ_PCL_360: the socket plugin declares PCL_QOS_RELIABILITY_RELIABLE.
TEST(PclCapabilities, SocketPluginOffersReliable) {
  EXPECT_EQ(DeclaredReliability(SOCKET_TRANSPORT_PLUGIN_PATH),
            PCL_QOS_RELIABILITY_RELIABLE);
}

///< REQ_PCL_361: the shared-memory plugin declares PCL_QOS_RELIABILITY_RELIABLE.
TEST(PclCapabilities, ShmPluginOffersReliable) {
  EXPECT_EQ(DeclaredReliability(SHM_TRANSPORT_PLUGIN_PATH),
            PCL_QOS_RELIABILITY_RELIABLE);
}

///< REQ_PCL_362: pcl_plugin_transport_qos(handle, cfg, NULL) fails closed with PCL_ERR_INVALID on a NULL out-param.
TEST(PclCapabilities, LoaderQosUnspecifiedWhenSymbolAbsent) {
  // The codec stub plugin exports no transport QoS symbol.
  pcl_plugin_handle_t* handle = pcl_plugin_open(SOCKET_TRANSPORT_PLUGIN_PATH);
  ASSERT_NE(handle, nullptr);
  pcl_qos_t qos{PCL_QOS_RELIABILITY_RELIABLE};
  // NULL out-param fails closed.
  EXPECT_EQ(pcl_plugin_transport_qos(handle, nullptr, nullptr), PCL_ERR_INVALID);
  pcl_plugin_unload(handle);
}

// -- Endpoint required caps + supports -----------------------------------

///< REQ_PCL_363: pcl_endpoint_required_caps() maps every endpoint kind to its required capability.
TEST(PclCapabilities, EndpointRequiredCaps) {
  EXPECT_EQ(pcl_endpoint_required_caps(PCL_ENDPOINT_PUBLISHER), PCL_CAP_PUBSUB);
  EXPECT_EQ(pcl_endpoint_required_caps(PCL_ENDPOINT_SUBSCRIBER), PCL_CAP_PUBSUB);
  EXPECT_EQ(pcl_endpoint_required_caps(PCL_ENDPOINT_PROVIDED), PCL_CAP_RPC_UNARY);
  EXPECT_EQ(pcl_endpoint_required_caps(PCL_ENDPOINT_CONSUMED), PCL_CAP_RPC_UNARY);
  EXPECT_EQ(pcl_endpoint_required_caps(PCL_ENDPOINT_STREAM_PROVIDED),
            PCL_CAP_RPC_STREAM);
  EXPECT_EQ(pcl_endpoint_required_caps(PCL_ENDPOINT_STREAM_CONSUMED),
            PCL_CAP_RPC_STREAM);
}

///< REQ_PCL_364: pcl_transport_caps_supports() reports whether a capability mask includes every required bit.
TEST(PclCapabilities, CapsSupports) {
  EXPECT_TRUE(pcl_transport_caps_supports(
      PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY, PCL_CAP_RPC_UNARY));
  EXPECT_FALSE(pcl_transport_caps_supports(PCL_CAP_PUBSUB, PCL_CAP_RPC_UNARY));
  EXPECT_TRUE(pcl_transport_caps_supports(PCL_CAP_NONE, PCL_CAP_NONE));
  EXPECT_TRUE(pcl_transport_caps_supports(PCL_CAP_PUBSUB, PCL_CAP_NONE));
}

// -- Compose-time validation ---------------------------------------------

namespace {

pcl_endpoint_route_t RemoteRoute(const char* name, pcl_endpoint_kind_t kind,
                                 const char* const* peers, uint32_t n) {
  pcl_endpoint_route_t r{};
  r.endpoint_name = name;
  r.endpoint_kind = kind;
  r.route_mode = PCL_ROUTE_REMOTE;
  r.peer_ids = peers;
  r.peer_count = n;
  return r;
}

}  // namespace

///< REQ_PCL_365: a remote route validates successfully when its named peer transport declares the required capability.
TEST(PclCapabilities, ValidateRoutePassesWhenTransportHasCap) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};  // caps are explicit; vtable content irrelevant
  ASSERT_EQ(pcl_executor_register_transport_caps(
                e, "grpc", &vt, PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM),
            PCL_OK);

  const char* peers[] = {"grpc"};
  auto route = RemoteRoute("obj_service", PCL_ENDPOINT_CONSUMED, peers, 1);
  char diag[128] = "x";
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_OK);
  EXPECT_STREQ(diag, "");
  pcl_executor_destroy(e);
}

///< REQ_PCL_366: a remote route fails closed with PCL_ERR_STATE and a diagnostic naming the missing capability.
TEST(PclCapabilities, ValidateRouteFailsClosedOnMissingCap) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};
  // A pub/sub-only transport cannot serve a unary-RPC endpoint.
  ASSERT_EQ(pcl_executor_register_transport_caps(e, "udp", &vt, PCL_CAP_PUBSUB),
            PCL_OK);

  const char* peers[] = {"udp"};
  auto route = RemoteRoute("obj_service", PCL_ENDPOINT_CONSUMED, peers, 1);
  char diag[128] = "";
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_NE(std::string(diag).find("RPC_UNARY"), std::string::npos);
  EXPECT_NE(std::string(diag).find("obj_service"), std::string::npos);
  pcl_executor_destroy(e);
}

///< REQ_PCL_367: a remote route naming an unregistered peer fails closed with PCL_ERR_NOT_FOUND and a diagnostic naming the peer.
TEST(PclCapabilities, ValidateRouteFailsClosedOnMissingPeer) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const char* peers[] = {"nope"};
  auto route = RemoteRoute("obj_service", PCL_ENDPOINT_SUBSCRIBER, peers, 1);
  char diag[128] = "";
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_ERR_NOT_FOUND);
  EXPECT_NE(std::string(diag).find("nope"), std::string::npos);
  pcl_executor_destroy(e);
}

///< REQ_PCL_368: a local-only route validates successfully without any transport registered.
TEST(PclCapabilities, ValidateLocalOnlyRouteNeedsNoTransport) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const char* peers[] = {"anything"};
  pcl_endpoint_route_t route{};
  route.endpoint_name = "local_ep";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_LOCAL;  // no remote leg
  route.peer_ids = peers;
  route.peer_count = 1;
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, nullptr, 0), PCL_OK);
  pcl_executor_destroy(e);
}

///< REQ_PCL_369: a subscriber endpoint validates successfully over a pub/sub-capable named transport.
TEST(PclCapabilities, ValidatePubsubEndpointOverPubsubTransport) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};
  ASSERT_EQ(pcl_executor_register_transport_caps(e, "udp", &vt, PCL_CAP_PUBSUB),
            PCL_OK);
  const char* peers[] = {"udp"};
  auto route = RemoteRoute("evidence", PCL_ENDPOINT_SUBSCRIBER, peers, 1);
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, nullptr, 0), PCL_OK);
  pcl_executor_destroy(e);
}

// -- QoS floor (§2.D.4) --------------------------------------------------

///< REQ_PCL_370: pcl_qos_satisfies() implements the ordered reliability floor check (unspecified < best_effort < reliable).
TEST(PclCapabilities, QosSatisfiesIsOrderedReliability) {
  const pcl_qos_t unspec{PCL_QOS_RELIABILITY_UNSPECIFIED};
  const pcl_qos_t best{PCL_QOS_RELIABILITY_BEST_EFFORT};
  const pcl_qos_t rel{PCL_QOS_RELIABILITY_RELIABLE};
  // No floor: anything satisfies.
  EXPECT_TRUE(pcl_qos_satisfies(unspec, unspec));
  EXPECT_TRUE(pcl_qos_satisfies(best, unspec));
  EXPECT_TRUE(pcl_qos_satisfies(rel, unspec));
  // Reliable floor: only a reliable offer meets it.
  EXPECT_TRUE(pcl_qos_satisfies(rel, rel));
  EXPECT_FALSE(pcl_qos_satisfies(best, rel));
  EXPECT_FALSE(pcl_qos_satisfies(unspec, rel));
  // Best-effort floor: best-effort or reliable meets it; undeclared does not.
  EXPECT_TRUE(pcl_qos_satisfies(best, best));
  EXPECT_TRUE(pcl_qos_satisfies(rel, best));
  EXPECT_FALSE(pcl_qos_satisfies(unspec, best));
}

///< REQ_PCL_371: a route validates successfully when its transport's declared QoS meets the endpoint's floor.
TEST(PclCapabilities, ValidateRoutePassesWhenTransportMeetsQosFloor) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};
  ASSERT_EQ(pcl_executor_register_transport_caps(e, "grpc", &vt,
                                                 PCL_CAP_RPC_UNARY),
            PCL_OK);
  ASSERT_EQ(pcl_executor_register_transport_qos(
                e, "grpc", pcl_qos_t{PCL_QOS_RELIABILITY_RELIABLE}),
            PCL_OK);

  const char* peers[] = {"grpc"};
  auto route = RemoteRoute("obj_service", PCL_ENDPOINT_CONSUMED, peers, 1);
  route.qos_floor.reliability = PCL_QOS_RELIABILITY_RELIABLE;
  char diag[160] = "x";
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_OK);
  EXPECT_STREQ(diag, "");
  pcl_executor_destroy(e);
}

///< REQ_PCL_372: a route fails closed with a diagnostic naming both the required and offered reliability when the QoS floor is unmet.
TEST(PclCapabilities, ValidateRouteFailsClosedWhenQosFloorUnmet) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};
  // udp carries pub/sub and is only best-effort.
  ASSERT_EQ(pcl_executor_register_transport_caps(e, "udp", &vt, PCL_CAP_PUBSUB),
            PCL_OK);
  ASSERT_EQ(pcl_executor_register_transport_qos(
                e, "udp", pcl_qos_t{PCL_QOS_RELIABILITY_BEST_EFFORT}),
            PCL_OK);

  const char* peers[] = {"udp"};
  auto route = RemoteRoute("evidence", PCL_ENDPOINT_SUBSCRIBER, peers, 1);
  route.qos_floor.reliability = PCL_QOS_RELIABILITY_RELIABLE;  // demand reliable
  char diag[160] = "";
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_NE(std::string(diag).find("reliable"), std::string::npos);
  EXPECT_NE(std::string(diag).find("best_effort"), std::string::npos);
  pcl_executor_destroy(e);
}

///< REQ_PCL_373: a route with a declared QoS floor fails closed when the transport's reliability is undeclared (unspecified).
TEST(PclCapabilities, ValidateRouteFailsClosedWhenQosUndeclared) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};
  // Caps fine, but the transport never declared its reliability.
  ASSERT_EQ(pcl_executor_register_transport_caps(e, "grpc", &vt,
                                                 PCL_CAP_RPC_UNARY),
            PCL_OK);
  const char* peers[] = {"grpc"};
  auto route = RemoteRoute("obj_service", PCL_ENDPOINT_CONSUMED, peers, 1);
  route.qos_floor.reliability = PCL_QOS_RELIABILITY_RELIABLE;
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, nullptr, 0),
            PCL_ERR_STATE);
  pcl_executor_destroy(e);
}

///< REQ_PCL_374: a route with no declared QoS floor validates on capabilities alone, ignoring the transport's offered QoS.
TEST(PclCapabilities, ValidateRouteIgnoresQosWhenNoFloor) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};
  // Undeclared QoS, but the endpoint asks for no floor: passes on caps alone.
  ASSERT_EQ(pcl_executor_register_transport_caps(e, "udp", &vt, PCL_CAP_PUBSUB),
            PCL_OK);
  const char* peers[] = {"udp"};
  auto route = RemoteRoute("evidence", PCL_ENDPOINT_SUBSCRIBER, peers, 1);
  // route.qos_floor left zero-initialised (UNSPECIFIED).
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, nullptr, 0), PCL_OK);
  pcl_executor_destroy(e);
}

///< REQ_PCL_375: pcl_endpoint_required_caps() returns PCL_CAP_NONE for an unrecognised endpoint kind.
TEST(PclCapabilities, EndpointRequiredCapsUnknownKindIsNone) {
  // An unrecognised endpoint kind imposes no capability requirement.
  EXPECT_EQ(pcl_endpoint_required_caps(static_cast<pcl_endpoint_kind_t>(0x7fff)),
            PCL_CAP_NONE);
}

///< REQ_PCL_376: pcl_qos_reliability_name() returns the correct human-readable name for each reliability level, defaulting to "unspecified" for an unrecognised value.
TEST(PclCapabilities, QosReliabilityNames) {
  EXPECT_STREQ(pcl_qos_reliability_name(PCL_QOS_RELIABILITY_BEST_EFFORT),
               "best_effort");
  EXPECT_STREQ(pcl_qos_reliability_name(PCL_QOS_RELIABILITY_RELIABLE),
               "reliable");
  EXPECT_STREQ(pcl_qos_reliability_name(PCL_QOS_RELIABILITY_UNSPECIFIED),
               "unspecified");
  EXPECT_STREQ(
      pcl_qos_reliability_name(static_cast<pcl_qos_reliability_t>(0x7fff)),
      "unspecified");
}

///< REQ_PCL_377: pcl_executor_set_transport_qos() sets the QoS of the executor's default transport slot and rejects a NULL executor.
TEST(PclCapabilities, SetTransportQosOnDefaultTransport) {
  EXPECT_EQ(pcl_executor_set_transport_qos(
                nullptr, pcl_qos_t{PCL_QOS_RELIABILITY_RELIABLE}),
            PCL_ERR_INVALID);

  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(pcl_executor_set_transport_qos(
                e, pcl_qos_t{PCL_QOS_RELIABILITY_RELIABLE}),
            PCL_OK);
  pcl_executor_destroy(e);
}

///< REQ_PCL_378: pcl_executor_register_transport_qos() rejects NULL arguments and returns PCL_ERR_NOT_FOUND for an unregistered peer.
TEST(PclCapabilities, RegisterTransportQosUnknownPeerNotFound) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(pcl_executor_register_transport_qos(
                nullptr, "x", pcl_qos_t{PCL_QOS_RELIABILITY_RELIABLE}),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_register_transport_qos(
                e, nullptr, pcl_qos_t{PCL_QOS_RELIABILITY_RELIABLE}),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_register_transport_qos(
                e, "never_registered", pcl_qos_t{PCL_QOS_RELIABILITY_RELIABLE}),
            PCL_ERR_NOT_FOUND);
  pcl_executor_destroy(e);
}

///< REQ_PCL_379: a remote route with no named peers validates against the executor's default transport slot, failing closed when absent or under-capable.
TEST(PclCapabilities, ValidateRouteAgainstDefaultTransport) {
  // A remote route with no named peers validates against the executor's
  // default transport slot.
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  // No default transport at all: fail closed, diag names the default slot.
  auto route = RemoteRoute("evidence", PCL_ENDPOINT_SUBSCRIBER, nullptr, 0);
  char diag[160] = "";
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_ERR_NOT_FOUND);
  EXPECT_NE(std::string(diag).find("default transport"), std::string::npos);

  // Default transport with pub/sub: subscriber routes pass.
  pcl_transport_t vt{};
  vt.publish = [](void*, const char*, const pcl_msg_t*) { return PCL_OK; };
  ASSERT_EQ(pcl_executor_set_transport(e, &vt), PCL_OK);
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_OK);

  // A stream-provided endpoint fails closed against the pub/sub-only default
  // transport, and the diag names the missing RPC_STREAM capability.
  auto stream_route =
      RemoteRoute("tiles", PCL_ENDPOINT_STREAM_PROVIDED, nullptr, 0);
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &stream_route, diag,
                                                 sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_NE(std::string(diag).find("RPC_STREAM"), std::string::npos);
  pcl_executor_destroy(e);
}

///< REQ_PCL_380: a publisher endpoint routed to a unary-RPC-only transport fails closed with a diagnostic naming the missing PUBSUB capability.
TEST(PclCapabilities, ValidatePublisherRouteFailsClosedWithoutPubsubCap) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_t vt{};
  // A unary-RPC-only transport cannot carry a publisher endpoint.
  ASSERT_EQ(pcl_executor_register_transport_caps(e, "grpc", &vt,
                                                 PCL_CAP_RPC_UNARY),
            PCL_OK);
  const char* peers[] = {"grpc"};
  auto route = RemoteRoute("evidence", PCL_ENDPOINT_PUBLISHER, peers, 1);
  char diag[160] = "";
  EXPECT_EQ(pcl_executor_validate_endpoint_route(e, &route, diag, sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_NE(std::string(diag).find("PUBSUB"), std::string::npos);
  pcl_executor_destroy(e);
}
