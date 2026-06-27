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
  // The shared-memory plugin does not export a caps symbol, so the loader must
  // fall back to vtable derivation -- equal to a direct derive, and no ACTION.
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);

  char config[256];
  std::snprintf(config, sizeof(config),
                "{\"bus_name\":\"pcl_caps_plugin_test_bus\","
                "\"participant_id\":\"caps_test\",\"executor\":%llu}",
                static_cast<unsigned long long>(
                    reinterpret_cast<std::uintptr_t>(executor)));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(SHM_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_OK);

  pcl_transport_caps_t caps = PCL_CAP_NONE;
  ASSERT_EQ(pcl_plugin_transport_caps(handle, config, transport, &caps),
            PCL_OK);
  EXPECT_EQ(caps, pcl_transport_caps_from_vtable(transport));
  EXPECT_EQ(caps & PCL_CAP_RPC_ACTION, 0u);

  auto destroy = reinterpret_cast<void (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(handle, "pcl_shm_transport_plugin_destroy"));
  ASSERT_NE(destroy, nullptr);
  destroy(transport);
  pcl_plugin_unload(handle);
  pcl_executor_destroy(executor);
}
