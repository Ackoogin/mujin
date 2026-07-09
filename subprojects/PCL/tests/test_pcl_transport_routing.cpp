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
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_transport_routing.h>

// Internal layout so the rollback test can inspect the executor's route table.
#include "pcl_internal.h"
}

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <unistd.h>
#endif

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

namespace {

// Generate a unique temp file path (the file itself need not exist yet).
std::string UniqueTempPath() {
#ifdef _WIN32
  char temp_dir[MAX_PATH];
  char file_name[MAX_PATH];
  GetTempPathA(MAX_PATH, temp_dir);
  GetTempFileNameA(temp_dir, "pcr", 0, file_name);
  return file_name;
#else
  char path[] = "/tmp/pcl_routing_XXXXXX";
  int fd = mkstemp(path);
  EXPECT_NE(fd, -1);
  if (fd != -1) close(fd);
  return std::string(path);
#endif
}

// Write a manifest to a unique temp path and return it.
std::string WriteManifest(const std::string& body) {
  const std::string path = UniqueTempPath();
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  EXPECT_TRUE(out.good());
  out.write(body.data(), static_cast<std::streamsize>(body.size()));
  out.close();
  return path;
}

using CountFn = uint32_t (*)();
using LastTopicFn = const char* (*)();
using ResetFn = void (*)();

}  // namespace

///< REQ_PCL_416: pcl_transport_routing_load() with a nonexistent manifest path fails closed with PCL_ERR_NOT_FOUND and a diagnostic naming the missing file.
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

///< REQ_PCL_417: an empty or comment-only manifest loads successfully with an empty routing handle (zero transports).
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
///< REQ_PCL_418: one manifest composes heterogeneous transports (a capture recorder for a unary service, UDP for a topic) and validates both routes.
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
///< REQ_PCL_419: a publish issued through a manifest-registered named transport is actually carried by that transport.
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

///< REQ_PCL_420: a manifest route to a transport lacking the endpoint's required capability fails closed with a diagnostic naming the missing capability, leaving nothing installed.
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
///< REQ_PCL_421: a route installed by an earlier manifest line is rolled back when a later line fails, leaving the executor's route table empty.
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

// A pre-existing route (e.g. a programmatic default) must survive a failed
// manifest load: the loader rejects a duplicate route line instead of
// overwriting -- then losing on rollback -- the original.
///< REQ_PCL_422: a manifest route line duplicating a pre-existing endpoint route fails closed without overwriting the original route.
TEST(PclTransportRouting, PreservesPreExistingRouteOnDuplicate) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  // Install a default local route for object_evidence/publisher up front.
  pcl_endpoint_route_t prior;
  memset(&prior, 0, sizeof(prior));
  prior.endpoint_name = "object_evidence";
  prior.endpoint_kind = PCL_ENDPOINT_PUBLISHER;
  prior.route_mode    = PCL_ROUTE_LOCAL;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &prior), PCL_OK);
  ASSERT_EQ(e->endpoint_route_count, 1u);

  // A manifest routing the same endpoint must fail closed and leave the
  // pre-existing route untouched.
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "route object_evidence publisher recorder\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_INVALID);
  EXPECT_EQ(routing, nullptr);
  EXPECT_NE(std::string(diag).find("already routed"), std::string::npos);
  // The original route is still installed, unchanged.
  ASSERT_EQ(e->endpoint_route_count, 1u);
  EXPECT_EQ(e->endpoint_routes[0].route_mode, (uint32_t)PCL_ROUTE_LOCAL);

  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// Loading and destroying a manifest repeatedly on the same executor must not
// leak named-transport slots: the unregister-on-destroy has to free the slot so
// later cycles do not saturate PCL_MAX_TRANSPORTS and fail with PCL_ERR_NOMEM.
// A manifest must not overwrite (and then, on teardown, delete) a transport it
// did not create: a duplicate peer id is rejected, leaving the original intact.
///< REQ_PCL_423: a manifest transport line reusing a peer id already registered on the executor fails closed, leaving the original transport untouched.
TEST(PclTransportRouting, RejectsDuplicateTransportPeer) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n";

  const auto path1 = WriteManifest(body);
  pcl_transport_routing_t* routing1 = nullptr;
  ASSERT_EQ(pcl_transport_routing_load(e, path1.c_str(), &routing1, nullptr, 0),
            PCL_OK);
  ASSERT_NE(routing1, nullptr);
  ASSERT_NE(pcl_executor_get_transport_for_peer(e, "recorder"), nullptr);

  // A second manifest reusing the same peer fails closed and leaves the first
  // manifest's transport untouched.
  const auto path2 = WriteManifest(body);
  pcl_transport_routing_t* routing2 = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path2.c_str(), &routing2, diag,
                                       sizeof(diag)),
            PCL_ERR_INVALID);
  EXPECT_EQ(routing2, nullptr);
  EXPECT_NE(std::string(diag).find("already registered"), std::string::npos);
  EXPECT_NE(pcl_executor_get_transport_for_peer(e, "recorder"), nullptr);
  EXPECT_EQ(e->transport_count, 1u);

  pcl_transport_routing_destroy(routing1);
  std::remove(path1.c_str());
  std::remove(path2.c_str());
  pcl_executor_destroy(e);
}

// Tearing down a manifest-owned named transport must not wipe an unrelated
// default transport that another owner installed on the executor.
///< REQ_PCL_424: destroying a manifest-owned named transport does not clear an unrelated default transport installed by another owner.
TEST(PclTransportRouting, PreservesDefaultTransportOnNamedPeerTeardown) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  // Install an unrelated default transport (sentinel adapter_ctx, no vtable
  // fns -- executor teardown guards on transport.shutdown).
  int sentinel = 0;
  pcl_transport_t def{};
  def.adapter_ctx = &sentinel;
  ASSERT_EQ(pcl_executor_set_transport(e, &def), PCL_OK);
  ASSERT_NE(pcl_executor_get_transport(e), nullptr);

  // Load then destroy a manifest UDP named peer.
  const std::string body =
      std::string("transport topic_udp ") + UDP_TRANSPORT_PLUGIN_PATH +
      " {\"remote_host\":\"127.0.0.1\",\"remote_port\":48655}\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK)
      << diag;
  ASSERT_NE(routing, nullptr);
  pcl_transport_routing_destroy(routing);

  // The unrelated default transport is still installed and unchanged.
  const pcl_transport_t* still = pcl_executor_get_transport(e);
  ASSERT_NE(still, nullptr);
  EXPECT_EQ(still->adapter_ctx, &sentinel);

  pcl_executor_set_transport(e, nullptr);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

///< REQ_PCL_425: repeated load/destroy cycles of the same manifest on one executor free the named-transport slot each time, never leaking it.
TEST(PclTransportRouting, ReusesTransportSlotsAcrossLoadDestroyCycles) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n";
  const auto path = WriteManifest(body);

  for (unsigned i = 0; i < PCL_MAX_TRANSPORTS + 4u; ++i) {
    pcl_transport_routing_t* routing = nullptr;
    char diag[200] = "";
    ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                         sizeof(diag)),
              PCL_OK)
        << "cycle " << i << ": " << diag;
    ASSERT_NE(routing, nullptr);
    pcl_transport_routing_destroy(routing);
    // Destroy unregisters the transport, freeing the slot for the next cycle.
    EXPECT_EQ(e->transport_count, 0u) << "after cycle " << i;
  }
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

///< REQ_PCL_426: a manifest route demanding a QoS floor the named transport does not meet fails closed with a diagnostic naming the reliability mismatch.
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

///< REQ_PCL_427: a manifest route naming a peer with no matching transport line fails closed with PCL_ERR_NOT_FOUND and a diagnostic naming the peer.
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

///< REQ_PCL_428: a manifest route line with an unrecognised endpoint kind fails closed with PCL_ERR_INVALID and a diagnostic naming the bad kind.
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

///< REQ_PCL_275: every malformed manifest shape fails closed with a precise
///< diagnostic: short transport line, missing plugin file, short route line,
///< peer-list overflow, unknown reliability, unknown directive.
TEST(PclTransportRouting, FailsClosedOnEachMalformedManifestShape) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  struct Case {
    const char* manifest;
    pcl_status_t expected;
    const char* diag_contains;
  };
  const Case cases[] = {
      // transport line with a peer id but no plugin path.
      {"transport lonely_peer\n", PCL_ERR_INVALID, "transport line needs"},
      // transport plugin path that does not exist.
      {"transport ghost /no/such/plugin.so\n", PCL_ERR_NOT_FOUND,
       "failed to load"},
      // route line missing its peer list.
      {"route object_evidence publisher\n", PCL_ERR_INVALID,
       "route line needs"},
      // route line with an unknown reliability token.
      {"route object_evidence publisher peer_a sometimes\n", PCL_ERR_INVALID,
       "unknown reliability"},
      // unknown directive.
      {"teleport object_evidence\n", PCL_ERR_INVALID,
       "unknown manifest directive"},
  };

  for (const Case& c : cases) {
    const auto path = WriteManifest(c.manifest);
    pcl_transport_routing_t* routing = nullptr;
    char diag[200] = "";
    EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                         sizeof(diag)),
              c.expected)
        << c.manifest;
    EXPECT_NE(std::string(diag).find(c.diag_contains), std::string::npos)
        << "diag was: " << diag;
    EXPECT_EQ(routing, nullptr);
    std::remove(path.c_str());
  }

  // Peer list longer than PCL_ROUTING_MAX_PEERS.
  {
    std::string peers = "p0";
    for (int i = 1; i < 40; ++i) peers += ",p" + std::to_string(i);
    const auto path =
        WriteManifest("route object_evidence publisher " + peers + "\n");
    pcl_transport_routing_t* routing = nullptr;
    char diag[200] = "";
    EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                         sizeof(diag)),
              PCL_ERR_INVALID);
    EXPECT_NE(std::string(diag).find("too many peers"), std::string::npos);
    std::remove(path.c_str());
  }

  pcl_executor_destroy(e);
}

///< REQ_PCL_317: transport_count is NULL-safe and destroy(NULL) is a no-op.
TEST(PclTransportRouting, NullHandleAccessorsAreSafe) {
  EXPECT_EQ(pcl_transport_routing_transport_count(nullptr), 0u);
  pcl_transport_routing_destroy(nullptr);

  // Bad arguments to load fail closed.
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_transport_routing_t* routing = nullptr;
  EXPECT_EQ(pcl_transport_routing_load(nullptr, "m.txt", &routing, nullptr, 0),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_transport_routing_load(e, nullptr, &routing, nullptr, 0),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_transport_routing_load(e, "m.txt", nullptr, nullptr, 0),
            PCL_ERR_INVALID);
  pcl_executor_destroy(e);
}

///< REQ_PCL_318: exhausting the executor's fixed transport table fails the
///< manifest load closed with a register diagnostic and rolls everything back.
TEST(PclTransportRouting, FailsClosedWhenExecutorTransportSlotsExhausted) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  // PCL_MAX_TRANSPORTS is 16: seventeen manifest transports overflow it.
  std::string body;
  for (int i = 0; i < 17; ++i) {
    body += "transport peer_" + std::to_string(i) + " " + CAPTURE_PLUGIN_PATH +
            "\n";
  }
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_NE(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK);
  EXPECT_NE(std::string(diag).find("register failed"), std::string::npos);
  EXPECT_EQ(routing, nullptr);
  // Rollback: every slot the failed load registered is free again.
  EXPECT_EQ(pcl_executor_get_transport_for_peer(e, "peer_0"), nullptr);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

///< REQ_PCL_319: exhausting the executor's endpoint-route table fails the
///< manifest route line closed with a set diagnostic.
TEST(PclTransportRouting, FailsClosedWhenRouteTableExhausted) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  // PCL_MAX_ENDPOINT_ROUTES is 128: one transport plus 129 routes overflows.
  std::string body = std::string("transport rec ") + CAPTURE_PLUGIN_PATH + "\n";
  for (int i = 0; i < 129; ++i) {
    body += "route ep_" + std::to_string(i) + " publisher rec\n";
  }
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_NE(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK);
  EXPECT_NE(std::string(diag).find("set failed"), std::string::npos);
  EXPECT_EQ(routing, nullptr);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// D5 two-sided `exclusive` grammar (doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md
// §3 D5): a group declares two named sides, e.g. the rpc realization
// (multiple command endpoints) vs. the pub/sub realization (one topic
// endpoint) of the same logical leg. Any number of same-side endpoints route
// together freely; routing from both sides is a compose-time error.

///< REQ_PCL_463: a route completing both sides of a declared `exclusive` group (one side-A endpoint + one side-B endpoint routed) fails closed with PCL_ERR_STATE and a diagnostic naming the group and both endpoints.
TEST(PclTransportRouting, ExclusiveGroupTwoSidedConflictFailsClosed) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "exclusive ma_action.request_leg ma_action.create,ma_action.update,ma_action.cancel agra.ma_action.request\n" +
      "route ma_action.create consumed recorder\n" +
      "route agra.ma_action.request publisher recorder\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_EQ(routing, nullptr);
  const std::string d(diag);
  EXPECT_NE(d.find("ma_action.request_leg"), std::string::npos) << diag;
  EXPECT_NE(d.find("ma_action.create"), std::string::npos) << diag;
  EXPECT_NE(d.find("agra.ma_action.request"), std::string::npos) << diag;
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

///< REQ_PCL_464: multiple same-side endpoints of one `exclusive` group (all three rpc command endpoints) route together successfully with no side-B member routed.
TEST(PclTransportRouting, ExclusiveGroupMultipleSameSideEndpointsRouteTogether) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "exclusive ma_action.request_leg ma_action.create,ma_action.update,ma_action.cancel agra.ma_action.request\n" +
      "route ma_action.create consumed recorder\n" +
      "route ma_action.update consumed recorder\n" +
      "route ma_action.cancel consumed recorder\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK)
      << diag;
  ASSERT_NE(routing, nullptr);
  EXPECT_NE(pcl_executor_get_transport_for_peer(e, "recorder"), nullptr);
  pcl_transport_routing_destroy(routing);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

///< REQ_PCL_465: an endpoint named in no `exclusive` group is unaffected by exclusivity, even when a group is declared and routed alongside it.
TEST(PclTransportRouting, ExclusiveGroupLeavesUngroupedEndpointUnaffected) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "exclusive ma_action.request_leg ma_action.create,ma_action.update,ma_action.cancel agra.ma_action.request\n" +
      "route ma_action.create consumed recorder\n" +
      // Unrelated endpoint, member of no group: routing it alongside a
      // routed group member (ma_action.create, side A) must not trip
      // exclusivity.
      "route object_evidence publisher recorder\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK)
      << diag;
  ASSERT_NE(routing, nullptr);
  pcl_transport_routing_destroy(routing);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

///< REQ_PCL_466: a malformed `exclusive` line (missing side B) fails closed with PCL_ERR_INVALID and a diagnostic naming the missing clause.
TEST(PclTransportRouting, ExclusiveGroupFailsClosedOnMalformedLine) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const auto path = WriteManifest(
      "exclusive ma_action.request_leg ma_action.create,ma_action.update\n");
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_INVALID);
  EXPECT_NE(std::string(diag).find("exclusive line needs"), std::string::npos)
      << diag;
  EXPECT_EQ(routing, nullptr);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// A second malformed shape: a doubled comma leaves an empty member in the
// middle of one side's endpoint list -- also fails closed rather than
// silently dropping it (a trailing comma with nothing after it is not
// ambiguous the same way, since there is no token left to misparse; the
// doubled-comma-mid-list shape is the one that would otherwise silently
// produce a phantom empty endpoint name).
///< REQ_PCL_467: an `exclusive` line with an empty endpoint name in a side's list (doubled comma) fails closed with PCL_ERR_INVALID and a diagnostic naming the group and side.
TEST(PclTransportRouting, ExclusiveGroupFailsClosedOnEmptyListMember) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const auto path = WriteManifest(
      "exclusive ma_action.request_leg ma_action.create,,ma_action.update agra.ma_action.request\n");
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_INVALID);
  const std::string d(diag);
  EXPECT_NE(d.find("ma_action.request_leg"), std::string::npos) << diag;
  EXPECT_NE(d.find("side A"), std::string::npos) << diag;
  EXPECT_EQ(routing, nullptr);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

///< REQ_PCL_469: a two-sided conflict fails closed even when both conflicting `route` lines appear BEFORE the `exclusive` line that groups them -- the check is order-independent, not declare-before-use.
TEST(PclTransportRouting, ExclusiveGroupConflictDetectedRegardlessOfDeclarationOrder) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      // Both conflicting routes appear before the `exclusive` declaration.
      "route ma_action.create consumed recorder\n" +
      "route agra.ma_action.request publisher recorder\n" +
      "exclusive ma_action.request_leg ma_action.create,ma_action.update,ma_action.cancel agra.ma_action.request\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_EQ(routing, nullptr);
  const std::string d(diag);
  EXPECT_NE(d.find("ma_action.request_leg"), std::string::npos) << diag;
  EXPECT_NE(d.find("ma_action.create"), std::string::npos) << diag;
  EXPECT_NE(d.find("agra.ma_action.request"), std::string::npos) << diag;
  // Both routes -- installed successfully at parse time -- must be rolled
  // back once the final exclusivity pass fails the whole load.
  EXPECT_EQ(e->endpoint_route_count, 0u);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// Rollback must cover exclusive-group routes too: routes installed for a
// group's side A must be cleared when a later, unrelated line fails -- same
// guarantee REQ_PCL_421 proves for plain routes, exercised here with a
// declared `exclusive` group in play.
///< REQ_PCL_468: routes installed for an `exclusive` group's side are rolled back when a later manifest line fails for an unrelated reason, leaving the executor's route table empty.
TEST(PclTransportRouting, ExclusiveGroupRoutesRolledBackWhenLaterLineFails) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "exclusive ma_action.request_leg ma_action.create,ma_action.update,ma_action.cancel agra.ma_action.request\n" +
      "route ma_action.create consumed recorder\n" +  // valid: side A, installs a route
      "route ma_action.update consumed recorder\n" +  // valid: side A, installs a route
      "route object_evidence wobble recorder\n";      // malformed kind: fails
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_INVALID);
  EXPECT_EQ(routing, nullptr);
  // Both routes installed for the group's side A must have been rolled back.
  EXPECT_EQ(e->endpoint_route_count, 0u);
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// ---------------------------------------------------------------------------
// Manifest-routed streaming invoke (rpc_pubsub_interchangeability_plan.md
// Phase 5, PCL.078 / REQ_PCL_470): pcl_executor_invoke_stream() must consult
// the per-endpoint route table pcl_transport_routing_load() installs, the
// same way pcl_executor_invoke_async() already does -- before this fix it
// only ever checked the legacy single executor-wide transport, so a
// manifest-routed streaming endpoint silently fell through to the
// intra-process fallback and failed with PCL_ERR_NOT_FOUND.
// ---------------------------------------------------------------------------

///< REQ_PCL_470: a streaming invoke for a manifest-routed `stream_consumed`
///< endpoint dispatches through the named peer's transport (PCL_STREAMING,
///< and the transport's own invoke_stream is actually called) rather than
///< falling through to the always-empty intra-process fallback.
TEST(PclTransportRouting, StreamingInvokeDispatchesThroughRoutedTransport) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  pcl_plugin_handle_t* probe = pcl_plugin_open(CAPTURE_PLUGIN_PATH);
  ASSERT_NE(probe, nullptr);
  auto reset = reinterpret_cast<void (*)(void)>(
      pcl_plugin_symbol(probe, "pcl_capture_reset"));
  auto stream_count = reinterpret_cast<uint32_t (*)(void)>(
      pcl_plugin_symbol(probe, "pcl_capture_invoke_stream_count"));
  ASSERT_NE(reset, nullptr);
  ASSERT_NE(stream_count, nullptr);
  reset();

  // The capture plugin declares PCL_CAP_RPC_STREAM (it implements
  // invoke_stream), so a stream_consumed route to it composes -- capability
  // *rejection* for a stream-incapable peer is
  // StreamingRouteRejectsStreamIncapableTransport below.
  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n" +
      "route some.read stream_consumed recorder\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK)
      << diag;

  const char payload[] = "query";
  pcl_msg_t req{};
  req.data = payload;
  req.size = sizeof(payload);
  req.type_name = "application/json";
  pcl_stream_context_t* ctx = nullptr;
  const pcl_status_t rc = pcl_executor_invoke_stream(
      e, "some.read", &req,
      [](const pcl_msg_t*, bool, pcl_status_t, void*) {}, nullptr, &ctx);

  EXPECT_EQ(rc, PCL_STREAMING);
  EXPECT_EQ(stream_count(), 1u);

  std::remove(path.c_str());
  pcl_plugin_unload(probe);
  pcl_transport_routing_destroy(routing);
  pcl_executor_destroy(e);
}

///< REQ_PCL_470: a stream_consumed route to a transport that is unary-only
///< (publish + invoke_async, no invoke_stream -- exactly the "unary-only
///< peer" scenario a stream_consumed/consumed kind conflation would have let
///< through) fails closed with a diagnostic naming RPC_STREAM, leaving
///< nothing installed. Before this fix, PCL_ENDPOINT_STREAM_CONSUMED did not
///< exist: a streaming client route was indistinguishable from a unary one
///< (both PCL_ENDPOINT_CONSUMED, requiring only PCL_CAP_RPC_UNARY), so this
///< exact peer composed successfully and only failed once the stream was
///< actually invoked at runtime.
TEST(PclTransportRouting, StreamingRouteRejectsStreamIncapableTransport) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  const std::string body =
      std::string("transport unary_only ") + NOCAPS_TRANSPORT_PLUGIN_PATH +
      "\n" + "route some.read stream_consumed unary_only\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  EXPECT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_EQ(routing, nullptr);
  EXPECT_NE(std::string(diag).find("RPC_STREAM"), std::string::npos) << diag;
  std::remove(path.c_str());
  pcl_executor_destroy(e);
}

// ---------------------------------------------------------------------------
// Gateway container discovery (rpc_pubsub_interchangeability_plan.md Phase 5,
// PCL.078 / REQ_PCL_471): a manifest-driven peer whose transport plugin
// exposes a gateway container (shared-memory, socket) must be discoverable
// through the routing handle so a provider component can wire it into the
// executor itself -- the loader does not do this automatically (see
// pcl_transport_routing.h's doc comment for why).
// ---------------------------------------------------------------------------

///< REQ_PCL_471: a manifest-routed peer whose transport exposes a gateway
///< container (shared memory) is discoverable via
///< pcl_transport_routing_get_gateway(), and the returned container is a
///< real, usable pcl_container_t (configure/activate/add all succeed).
TEST(PclTransportRouting, GetGatewayReturnsUsableContainerForShmPeer) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  const std::string body =
      std::string("transport shm_peer ") + SHM_TRANSPORT_PLUGIN_PATH +
      " {\"bus_name\":\"routing_gateway_test_bus\",\"participant_id\":\"a\"}\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  char diag[200] = "";
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_OK)
      << diag;

  pcl_container_t* gateway = nullptr;
  EXPECT_EQ(pcl_transport_routing_get_gateway(routing, "shm_peer", &gateway),
            PCL_OK);
  ASSERT_NE(gateway, nullptr);
  EXPECT_EQ(pcl_container_configure(gateway), PCL_OK);
  EXPECT_EQ(pcl_container_activate(gateway), PCL_OK);
  EXPECT_EQ(pcl_executor_add(e, gateway), PCL_OK);

  std::remove(path.c_str());
  pcl_transport_routing_destroy(routing);
  pcl_executor_destroy(e);
}

///< REQ_PCL_471: a peer whose transport exposes no gateway symbol (the test
///< capture plugin) returns PCL_OK with a NULL container -- not an error.
TEST(PclTransportRouting, GetGatewayReturnsNullForPluginWithoutGateway) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, nullptr, 0),
            PCL_OK);

  pcl_container_t* gateway = reinterpret_cast<pcl_container_t*>(0x1);
  EXPECT_EQ(pcl_transport_routing_get_gateway(routing, "recorder", &gateway),
            PCL_OK);
  EXPECT_EQ(gateway, nullptr);

  std::remove(path.c_str());
  pcl_transport_routing_destroy(routing);
  pcl_executor_destroy(e);
}

///< REQ_PCL_471: an unrecognized peer id fails closed with
///< PCL_ERR_NOT_FOUND, and NULL/invalid arguments fail with PCL_ERR_INVALID.
TEST(PclTransportRouting, GetGatewayFailsClosedOnUnknownPeerAndBadArgs) {
  pcl_executor_t* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  const std::string body =
      std::string("transport recorder ") + CAPTURE_PLUGIN_PATH + "\n";
  const auto path = WriteManifest(body);
  pcl_transport_routing_t* routing = nullptr;
  ASSERT_EQ(pcl_transport_routing_load(e, path.c_str(), &routing, nullptr, 0),
            PCL_OK);

  pcl_container_t* gateway = nullptr;
  EXPECT_EQ(pcl_transport_routing_get_gateway(routing, "no_such_peer", &gateway),
            PCL_ERR_NOT_FOUND);
  EXPECT_EQ(pcl_transport_routing_get_gateway(nullptr, "recorder", &gateway),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_transport_routing_get_gateway(routing, "recorder", nullptr),
            PCL_ERR_INVALID);

  std::remove(path.c_str());
  pcl_transport_routing_destroy(routing);
  pcl_executor_destroy(e);
}
