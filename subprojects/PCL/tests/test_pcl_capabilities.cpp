#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>

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

TEST(PclCapabilities, NullVtableHasNoCaps) {
  EXPECT_EQ(pcl_transport_caps_from_vtable(nullptr), PCL_CAP_NONE);
}

TEST(PclCapabilities, EmptyVtableHasNoCaps) {
  pcl_transport_t t = EmptyVtable();
  EXPECT_EQ(pcl_transport_caps_from_vtable(&t), PCL_CAP_NONE);
}

TEST(PclCapabilities, PublishOrSubscribeImpliesPubsub) {
  pcl_transport_t pub = EmptyVtable();
  pub.publish = reinterpret_cast<decltype(pub.publish)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&pub), PCL_CAP_PUBSUB);

  pcl_transport_t sub = EmptyVtable();
  sub.subscribe = reinterpret_cast<decltype(sub.subscribe)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&sub), PCL_CAP_PUBSUB);
}

TEST(PclCapabilities, ServeOrInvokeAsyncImpliesUnary) {
  pcl_transport_t srv = EmptyVtable();
  srv.serve = reinterpret_cast<decltype(srv.serve)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&srv), PCL_CAP_RPC_UNARY);

  pcl_transport_t cli = EmptyVtable();
  cli.invoke_async = reinterpret_cast<decltype(cli.invoke_async)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&cli), PCL_CAP_RPC_UNARY);
}

TEST(PclCapabilities, InvokeStreamOrStreamSendImpliesStream) {
  pcl_transport_t cli = EmptyVtable();
  cli.invoke_stream = reinterpret_cast<decltype(cli.invoke_stream)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&cli), PCL_CAP_RPC_STREAM);

  pcl_transport_t srv = EmptyVtable();
  srv.stream_send = reinterpret_cast<decltype(srv.stream_send)>(0x1);
  EXPECT_EQ(pcl_transport_caps_from_vtable(&srv), PCL_CAP_RPC_STREAM);
}

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

TEST(PclCapabilities, LoaderNullHandleFailsClosed) {
  pcl_transport_caps_t caps = PCL_CAP_NONE;
  EXPECT_EQ(pcl_plugin_transport_caps(nullptr, nullptr, nullptr, &caps),
            PCL_ERR_INVALID);
}

TEST(PclCapabilities, LoaderUsesExplicitCapsSymbol) {
  // The capture plugin exports pcl_transport_plugin_caps declaring an ACTION
  // capability that its vtable (publish + serve only) could never imply, so a
  // returned ACTION bit proves the explicit symbol was consulted.
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(CAPTURE_PLUGIN_PATH, "{}",
                                      &handle, &transport),
            PCL_OK);

  pcl_transport_caps_t caps = PCL_CAP_NONE;
  ASSERT_EQ(pcl_plugin_transport_caps(handle, "{}", transport, &caps), PCL_OK);
  EXPECT_EQ(caps, PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_ACTION);

  pcl_plugin_unload(handle);
}

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

}  // namespace

TEST(PclCapabilities, UdpPluginDeclaresPubsub) {
  EXPECT_EQ(DeclaredCaps(UDP_TRANSPORT_PLUGIN_PATH), PCL_CAP_PUBSUB);
}

TEST(PclCapabilities, SocketPluginDeclaresPubsubUnary) {
  EXPECT_EQ(DeclaredCaps(SOCKET_TRANSPORT_PLUGIN_PATH),
            PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY);
}

TEST(PclCapabilities, ShmPluginDeclaresPubsubUnaryStream) {
  EXPECT_EQ(DeclaredCaps(SHM_TRANSPORT_PLUGIN_PATH),
            PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM);
}
