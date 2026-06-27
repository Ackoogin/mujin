/// \file test_pcl_transport_routing.cpp
/// \brief Manifest-driven per-endpoint transport routing (§2.D.5).
///
/// Proves a single component can compose heterogeneous middleware from one
/// manifest: each endpoint is routed to a named transport plugin, capabilities
/// and QoS are validated at compose time (fail closed), and real traffic flows
/// through a manifest-registered transport.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_capabilities.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_transport_routing.h>

// Internal layout so the rollback test can inspect the executor's route table.
#include "pcl_internal.h"
}

#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>

namespace {

// Write a manifest to a unique temp path and return it.
std::string WriteManifest(const std::string& body) {
  char path[] = "/tmp/pcl_routing_XXXXXX";
  int fd = mkstemp(path);
  EXPECT_NE(fd, -1);
  if (fd != -1) {
    EXPECT_EQ(write(fd, body.data(), body.size()),
              static_cast<ssize_t>(body.size()));
    close(fd);
  }
  return std::string(path);
}

using CountFn = uint32_t (*)();
using LastTopicFn = const char* (*)();
using ResetFn = void (*)();

}  // namespace

TEST(PclTransportRouting, MissingManifestFailsClosed) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_routing_t* routing = nullptr;
  char diag[160] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, "/no/such/manifest", &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_NOT_FOUND);
  EXPECT_EQ(routing, nullptr);
  EXPECT_NE(std::string(diag).find("manifest not found"), std::string::npos);
  pcl_executor_destroy(e);
}

TEST(PclTransportRouting, EmptyManifestIsOkWithNoTransports) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const auto path = WriteManifest("# just a comment\n\n   \n");
  pcl_transport_routing_t* routing = nullptr;
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, nullptr, 0),
            PCL_OK);
  ASSERT_NE(routing, nullptr);
  EXPECT_EQ(pcl_transport_routing_transport_count(routing), 0u);
  pcl_transport_routing_destroy(routing);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// Mixed middleware: a recorder (capture) for a unary service + UDP for a topic,
// composed from one manifest, both routes validated.
TEST(PclTransportRouting, MixedMiddlewareLoadsAndValidates) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport svc_recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "transport topic_udp " + UDP_TRANSPORT_PLUGIN_PATH +
      " {\"remote_host\":\"127.0.0.1\",\"remote_port\":48650}\n" +
      "route create_requirement consumed svc_recorder\n" +
      "route object_evidence publisher topic_udp best_effort\n";
  const auto path = WriteManifest(body);

  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK)
      << diag;
  ASSERT_NE(routing, nullptr);
  EXPECT_EQ(pcl_transport_routing_transport_count(routing), 2u);
  // Both peers are registered as distinct named transports.
  EXPECT_NE(pcl_executor_get_transport_for_peer(e, "svc_recorder"), nullptr);
  EXPECT_NE(pcl_executor_get_transport_for_peer(e, "topic_udp"), nullptr);

  pcl_transport_routing_destroy(routing);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// Real traffic: publish through the manifest-registered recorder transport and
// confirm it was carried (the capture plugin records publishes in static state).
TEST(PclTransportRouting, RealTrafficFlowsThroughRoutedTransport) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  // Separate handle to the same capture .so to read its recorded state.
  pcl_plugin_handle_t* probe = pcl_plugin_open(CAPTURE_PLUGIN_PATH);
  ASSERT_NE(probe, nullptr);
  auto reset = reinterpret_cast<ResetFn>(
      pcl_plugin_symbol(probe, "pcl_capture_reset"));
  auto publish_count = reinterpret_cast<CountFn>(
      pcl_plugin_symbol(probe, "pcl_capture_publish_count"));
  auto last_topic = reinterpret_cast<LastTopicFn>(
      pcl_plugin_symbol(probe, "pcl_capture_last_topic"));
  ASSERT_NE(reset, nullptr);
  ASSERT_NE(publish_count, nullptr);
  ASSERT_NE(last_topic, nullptr);
  reset();

  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "route object_evidence publisher recorder\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, nullptr, 0),
            PCL_OK);

  // Drive a publish through the transport the manifest registered for "recorder".
  const pcl_transport_t* vt = pcl_executor_get_transport_for_peer(e, "recorder");
  ASSERT_NE(vt, nullptr);
  ASSERT_NE(vt->publish, nullptr);
  const char payload[] = "evidence";
  pcl_msg_t msg{};
  msg.data = payload;
  msg.size = sizeof(payload);
  msg.type_name = "application/json";
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "standard.object_evidence", &msg),
            PCL_OK);

  EXPECT_EQ(publish_count(), 1u);
  EXPECT_STREQ(last_topic(), "standard.object_evidence");

  pcl_transport_routing_destroy(routing);
  pcl_plugin_unload(probe);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

TEST(PclTransportRouting, FailsClosedWhenTransportLacksRequiredCap) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  // UDP carries only PUBSUB; routing a unary-RPC endpoint to it must fail.
  const std::string body =
      std::string("transport topic_udp ") + UDP_TRANSPORT_PLUGIN_PATH +
      " {\"remote_host\":\"127.0.0.1\",\"remote_port\":48651}\n" +
      "route create_requirement consumed topic_udp\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_EQ(routing, nullptr);  // failed load leaves nothing registered
  EXPECT_NE(std::string(diag).find("RPC_UNARY"), std::string::npos);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// A manifest with an earlier valid route and a later failing line must leave
// NOTHING installed: the route installed by the first line is rolled back so the
// executor is not left routing an endpoint to a peer whose transport is torn
// down on the error path.
TEST(PclTransportRouting, RollsBackRoutesWhenLaterLineFails) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "route create_requirement consumed recorder\n" +  // valid: installs a route
      "route object_evidence wobble recorder\n";         // malformed kind: fails
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_INVALID);
  EXPECT_EQ(routing, nullptr);  // failed load leaves nothing registered
  // The route installed by the first line must have been rolled back.
  EXPECT_EQ(e->endpoint_route_count, 0u);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

TEST(PclTransportRouting, FailsClosedWhenQosFloorUnmet) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  // UDP offers best_effort; a topic demanding reliable must fail closed.
  const std::string body =
      std::string("transport topic_udp ") + UDP_TRANSPORT_PLUGIN_PATH +
      " {\"remote_host\":\"127.0.0.1\",\"remote_port\":48652}\n" +
      "route object_evidence publisher topic_udp reliable\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_NE(std::string(diag).find("reliable"), std::string::npos);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

TEST(PclTransportRouting, FailsClosedOnUnknownPeer) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const auto path =
      WriteManifest("route object_evidence publisher ghost_peer\n");
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_NOT_FOUND);
  EXPECT_NE(std::string(diag).find("ghost_peer"), std::string::npos);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

TEST(PclTransportRouting, FailsClosedOnMalformedLine) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const auto path = WriteManifest("route object_evidence wobble recorder\n");
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_INVALID);
  EXPECT_NE(std::string(diag).find("unknown kind"), std::string::npos);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}
