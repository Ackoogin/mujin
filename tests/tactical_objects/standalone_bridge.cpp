/// \file standalone_bridge.cpp
/// \brief Standalone StandardBridge process with dual TCP transport.
///
/// Sits between TacticalObjectsComponent (backend, TCP client) and an
/// external Ada/test client (frontend, TCP server).  Translates between
/// the standard pyramid JSON protocol and the internal wire format.
///
/// Architecture:
///   [TacticalObjectsComponent] <--TCP--> [StandaloneBridge] <--TCP--> [Ada Client]
///
/// Backend executor (client transport):
///   - Invokes subscribe_interest remotely
///   - Subscribes to entity_updates and evidence_requirements topics
///   - Publishes observation_ingress topic (forwarded from standard.object_evidence)
///
/// Frontend executor (server transport):
///   - Provides object_of_interest.create_requirement service
///   - Publishes standard.entity_matches and standard.evidence_requirements
///   - Subscribes to standard.object_evidence
///
/// Usage: standalone_bridge --backend-host HOST --backend-port PORT
///                          --frontend-port PORT [--port-file PATH]
///                          [--timeout SECS]
#include <TacticalObjectsCodec.h>
#include <StreamingCodec.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace tactical_objects;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

static std::atomic<bool> g_shutdown{false};
static void signal_handler(int) { g_shutdown.store(true); }

// ---------------------------------------------------------------------------
// Shared state between callbacks and main loop
// ---------------------------------------------------------------------------

struct PendingPublish {
  std::string topic;
  std::string type_name;
  std::string data;
};

struct BridgeState {
  // Backend transport (client to TacticalObjectsComponent)
  pcl_executor_t*            backend_exec     = nullptr;
  pcl_socket_transport_t*    backend_transport = nullptr;

  // Frontend transport (server for Ada client)
  pcl_executor_t*            frontend_exec     = nullptr;
  pcl_socket_transport_t*    frontend_transport = nullptr;

  // Frontend publisher ports
  pcl_port_t*  pub_entity_matches    = nullptr;
  pcl_port_t*  pub_evidence_reqs     = nullptr;

  // Pending messages to forward between executors
  std::mutex mu;
  std::vector<PendingPublish> to_frontend;   // entity matches, evidence reqs
  std::vector<PendingPublish> to_backend;    // observations

  // Response buffer for service handler (frontend thread)
  std::string resp_buf;
};

static BridgeState g_bridge;

// ---------------------------------------------------------------------------
// Standard ↔ Internal enum converters (same as StandardBridge.cpp)
// ---------------------------------------------------------------------------

static std::string affiliationToStdIdentity(Affiliation a) {
  switch (a) {
    case Affiliation::Friendly:      return "STANDARD_IDENTITY_FRIENDLY";
    case Affiliation::Hostile:       return "STANDARD_IDENTITY_HOSTILE";
    case Affiliation::Neutral:       return "STANDARD_IDENTITY_NEUTRAL";
    case Affiliation::Unknown:       return "STANDARD_IDENTITY_UNKNOWN";
    case Affiliation::AssumedFriend: return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
    case Affiliation::Suspect:       return "STANDARD_IDENTITY_SUSPECT";
    case Affiliation::Joker:         return "STANDARD_IDENTITY_JOKER";
    case Affiliation::Faker:         return "STANDARD_IDENTITY_FAKER";
    case Affiliation::Pending:       return "STANDARD_IDENTITY_PENDING";
  }
  return "STANDARD_IDENTITY_UNKNOWN";
}

static std::string battleDimToStd(BattleDimension d) {
  switch (d) {
    case BattleDimension::Ground:     return "BATTLE_DIMENSION_GROUND";
    case BattleDimension::Air:        return "BATTLE_DIMENSION_AIR";
    case BattleDimension::SeaSurface: return "BATTLE_DIMENSION_SEA_SURFACE";
    case BattleDimension::Subsurface: return "BATTLE_DIMENSION_SUBSURFACE";
    case BattleDimension::Space:      return "BATTLE_DIMENSION_SPACE";
    case BattleDimension::SOF:        return "BATTLE_DIMENSION_SOF";
  }
  return "BATTLE_DIMENSION_GROUND";
}

static double degToRad(double d) { return d * 0.017453292519943295; }
static double radToDeg(double r) { return r * 57.29577951308232; }

// ---------------------------------------------------------------------------
// Backend subscriber: entity_updates (binary) → standard.entity_matches (JSON)
// ---------------------------------------------------------------------------

static void backend_entity_updates_cb(pcl_container_t*, const pcl_msg_t* msg,
                                      void*) {
  if (!msg || !msg->data || msg->size == 0) return;

  auto frames = StreamingCodec::decodeBatchFrame(
      static_cast<const uint8_t*>(msg->data), msg->size);

  json arr = json::array();
  for (const auto& upd : frames) {
    if (upd.message_type == STREAM_MSG_ENTITY_DELETE) continue;

    json obj;
    obj["object_id"] = TacticalObjectsCodec::encodeUUID(upd.entity_id)
                           .get<std::string>();
    if (upd.affiliation) {
      obj["identity"] = affiliationToStdIdentity(*upd.affiliation);
    }
    if (upd.mil_class) {
      obj["dimension"] = battleDimToStd(upd.mil_class->battle_dim);
    }
    if (upd.position) {
      obj["latitude_rad"]  = degToRad(upd.position->lat);
      obj["longitude_rad"] = degToRad(upd.position->lon);
    }
    if (upd.confidence) {
      obj["confidence"] = *upd.confidence;
    }
    arr.push_back(obj);
  }

  if (arr.empty()) return;

  std::string payload = arr.dump();
  std::lock_guard<std::mutex> lock(g_bridge.mu);
  g_bridge.to_frontend.push_back(
      {"standard.entity_matches", "application/json", std::move(payload)});
}

// ---------------------------------------------------------------------------
// Backend subscriber: evidence_requirements (JSON) → standard.evidence_requirements
// ---------------------------------------------------------------------------

static void backend_evidence_reqs_cb(pcl_container_t*, const pcl_msg_t* msg,
                                     void*) {
  if (!msg || !msg->data || msg->size == 0) return;

  std::string payload(static_cast<const char*>(msg->data), msg->size);
  std::lock_guard<std::mutex> lock(g_bridge.mu);
  g_bridge.to_frontend.push_back(
      {"standard.evidence_requirements", "application/json",
       std::move(payload)});
}

// ---------------------------------------------------------------------------
// Backend container on_configure
// ---------------------------------------------------------------------------

static pcl_status_t backend_on_configure(pcl_container_t* c, void*) {
  pcl_container_add_subscriber(c, "entity_updates",
                               "application/octet-stream",
                               backend_entity_updates_cb, nullptr);
  pcl_container_add_subscriber(c, "evidence_requirements",
                               "application/json",
                               backend_evidence_reqs_cb, nullptr);
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Frontend subscriber: standard.object_evidence → observation_ingress
// ---------------------------------------------------------------------------

static void frontend_object_evidence_cb(pcl_container_t*, const pcl_msg_t* msg,
                                        void*) {
  if (!msg || !msg->data || msg->size == 0) return;

  std::string str(static_cast<const char*>(msg->data), msg->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return;
  }

  // Translate standard observation to internal format (must match
  // TacticalObjectsCodec::decodeObservation expected schema).
  json internal;
  internal["observations"] = json::array();
  json obs;
  obs["observation_id"] = pyramid::core::uuid::UUIDHelper::toString(
      pyramid::core::uuid::UUIDHelper::generateV4());
  obs["position"] = json::object();
  obs["position"]["lat"] = radToDeg(j.value("latitude_rad", 0.0));
  obs["position"]["lon"] = radToDeg(j.value("longitude_rad", 0.0));
  obs["position"]["alt"] = 0.0;
  obs["confidence"] = j.value("confidence", 0.0);
  obs["observed_at"] = j.value("observed_at", 0.0);
  obs["object_hint_type"] = "Platform";
  obs["affiliation_hint"] = "Unknown";

  std::string identity = j.value("identity", "");
  if (identity == "STANDARD_IDENTITY_HOSTILE")  obs["affiliation_hint"] = "Hostile";
  else if (identity == "STANDARD_IDENTITY_FRIENDLY") obs["affiliation_hint"] = "Friendly";
  else if (identity == "STANDARD_IDENTITY_NEUTRAL")  obs["affiliation_hint"] = "Neutral";

  std::string dimension = j.value("dimension", "");
  if (dimension == "BATTLE_DIMENSION_SEA_SURFACE")
    obs["source_sidc"] = "SHSP------*****";
  else if (dimension == "BATTLE_DIMENSION_AIR")
    obs["source_sidc"] = "SHAP------*****";
  else if (dimension == "BATTLE_DIMENSION_SUBSURFACE")
    obs["source_sidc"] = "SHUP------*****";
  else if (dimension == "BATTLE_DIMENSION_GROUND")
    obs["source_sidc"] = "SHGP------*****";

  internal["observations"].push_back(obs);

  std::string payload = internal.dump();
  std::lock_guard<std::mutex> lock(g_bridge.mu);
  g_bridge.to_backend.push_back(
      {"observation_ingress", "application/json", std::move(payload)});
}

// ---------------------------------------------------------------------------
// Frontend service: object_of_interest.create_requirement
// ---------------------------------------------------------------------------

static pcl_status_t frontend_create_requirement(pcl_container_t*,
                                                 const pcl_msg_t* request,
                                                 pcl_msg_t* response,
                                                 void*) {
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json req_j;
  try {
    req_j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  std::fprintf(stderr, "[bridge] create_requirement: %s\n", str.c_str());

  // Translate standard → internal subscribe_interest format
  json internal;

  std::string policy = req_j.value("policy", "DATA_POLICY_OBTAIN");
  internal["query_mode"] = (policy == "DATA_POLICY_OBTAIN")
                               ? "active_find"
                               : "read_current";

  std::string identity = req_j.value("identity", "");
  if (identity == "STANDARD_IDENTITY_HOSTILE")       internal["affiliation"] = "Hostile";
  else if (identity == "STANDARD_IDENTITY_FRIENDLY") internal["affiliation"] = "Friendly";
  else if (identity == "STANDARD_IDENTITY_NEUTRAL")  internal["affiliation"] = "Neutral";
  else if (identity == "STANDARD_IDENTITY_UNKNOWN")  internal["affiliation"] = "Unknown";

  std::string dimension = req_j.value("dimension", "");
  if (dimension == "BATTLE_DIMENSION_GROUND")      internal["battle_dimension"] = "Ground";
  else if (dimension == "BATTLE_DIMENSION_AIR")    internal["battle_dimension"] = "Air";
  else if (dimension == "BATTLE_DIMENSION_SEA_SURFACE") internal["battle_dimension"] = "SeaSurface";
  else if (dimension == "BATTLE_DIMENSION_SUBSURFACE")  internal["battle_dimension"] = "Subsurface";

  if (req_j.contains("min_lat_rad")) {
    json area;
    area["min_lat"] = radToDeg(req_j.value("min_lat_rad", 0.0));
    area["max_lat"] = radToDeg(req_j.value("max_lat_rad", 0.0));
    area["min_lon"] = radToDeg(req_j.value("min_lon_rad", 0.0));
    area["max_lon"] = radToDeg(req_j.value("max_lon_rad", 0.0));
    internal["area"] = area;
    internal["expires_at"] = 9999;
  }

  std::string internal_str = internal.dump();
  std::fprintf(stderr, "[bridge] → subscribe_interest: %s\n",
               internal_str.c_str());

  // Invoke subscribe_interest on backend via async remote call
  pcl_msg_t req_msg = {};
  req_msg.data      = internal_str.data();
  req_msg.size      = static_cast<uint32_t>(internal_str.size());
  req_msg.type_name = "application/json";

  struct AsyncCtx {
    std::string body;
    std::atomic<bool> done{false};
  } ctx;

  auto resp_cb = [](const pcl_msg_t* resp, void* ud) {
    auto* c = static_cast<AsyncCtx*>(ud);
    if (resp && resp->data && resp->size > 0)
      c->body.assign(static_cast<const char*>(resp->data), resp->size);
    c->done.store(true);
  };

  pcl_status_t rc = pcl_socket_transport_invoke_remote_async(
      g_bridge.backend_transport, "subscribe_interest", &req_msg,
      resp_cb, &ctx);

  if (rc != PCL_OK) {
    g_bridge.resp_buf = "{\"error\":\"enqueue failed\"}";
    response->data      = g_bridge.resp_buf.data();
    response->size      = static_cast<uint32_t>(g_bridge.resp_buf.size());
    response->type_name = "application/json";
    return PCL_OK;
  }

  // Spin backend executor until response arrives or timeout
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!ctx.done.load() &&
         std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(g_bridge.backend_exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (!ctx.done.load()) {
    g_bridge.resp_buf = "{\"error\":\"subscribe_interest timeout\"}";
    response->data      = g_bridge.resp_buf.data();
    response->size      = static_cast<uint32_t>(g_bridge.resp_buf.size());
    response->type_name = "application/json";
    return PCL_OK;
  }

  std::fprintf(stderr, "[bridge] ← subscribe_interest: %s\n",
               ctx.body.c_str());

  // Parse internal response, forward evidence_requirements to frontend
  json iresp;
  try {
    iresp = json::parse(ctx.body);
  } catch (...) {
    iresp = json::object();
  }

  if (iresp.contains("evidence_requirements") &&
      iresp["evidence_requirements"].is_array()) {
    std::lock_guard<std::mutex> lock(g_bridge.mu);
    for (const auto& ev : iresp["evidence_requirements"]) {
      g_bridge.to_frontend.push_back(
          {"standard.evidence_requirements", "application/json", ev.dump()});
    }
  }

  // Build standard response
  json std_resp;
  std_resp["interest_id"] = iresp.value("interest_id", "");
  if (iresp.contains("solution_id")) {
    std_resp["solution_id"] = iresp["solution_id"];
  }

  g_bridge.resp_buf    = std_resp.dump();
  response->data       = g_bridge.resp_buf.data();
  response->size       = static_cast<uint32_t>(g_bridge.resp_buf.size());
  response->type_name  = "application/json";
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Frontend container on_configure
// ---------------------------------------------------------------------------

static pcl_status_t frontend_on_configure(pcl_container_t* c, void*) {
  pcl_container_add_subscriber(c, "standard.object_evidence",
                               "application/json",
                               frontend_object_evidence_cb, nullptr);
  pcl_container_add_service(c, "object_of_interest.create_requirement",
                            "application/json",
                            frontend_create_requirement, nullptr);
  g_bridge.pub_entity_matches =
      pcl_container_add_publisher(c, "standard.entity_matches",
                                  "application/json");
  g_bridge.pub_evidence_reqs =
      pcl_container_add_publisher(c, "standard.evidence_requirements",
                                  "application/json");
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  const char* backend_host = "127.0.0.1";
  uint16_t backend_port  = 19123;
  uint16_t frontend_port = 19124;
  std::string port_file;
  int timeout_secs = 30;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--backend-host") == 0 && i + 1 < argc) {
      backend_host = argv[++i];
    } else if (std::strcmp(argv[i], "--backend-port") == 0 && i + 1 < argc) {
      backend_port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--frontend-port") == 0 && i + 1 < argc) {
      frontend_port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--port-file") == 0 && i + 1 < argc) {
      port_file = argv[++i];
    } else if (std::strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
      timeout_secs = std::atoi(argv[++i]);
    }
  }

  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT, signal_handler);

  // -- Backend executor: TCP client to TacticalObjectsComponent ---------------

  std::fprintf(stderr,
               "[bridge] Connecting to backend at %s:%d...\n",
               backend_host, backend_port);

  g_bridge.backend_exec = pcl_executor_create();
  if (!g_bridge.backend_exec) {
    std::fprintf(stderr, "[bridge] Failed to create backend executor\n");
    return 1;
  }

  g_bridge.backend_transport = pcl_socket_transport_create_client(
      backend_host, backend_port, g_bridge.backend_exec);
  if (!g_bridge.backend_transport) {
    std::fprintf(stderr,
                 "[bridge] Failed to connect to backend at %s:%d\n",
                 backend_host, backend_port);
    pcl_executor_destroy(g_bridge.backend_exec);
    return 1;
  }

  pcl_executor_set_transport(
      g_bridge.backend_exec,
      pcl_socket_transport_get_transport(g_bridge.backend_transport));

  // Backend subscriber container
  pcl_callbacks_t backend_cbs = {};
  backend_cbs.on_configure = backend_on_configure;
  pcl_container_t* backend_container =
      pcl_container_create("bridge_backend", &backend_cbs, nullptr);
  pcl_container_configure(backend_container);
  pcl_container_activate(backend_container);
  pcl_executor_add(g_bridge.backend_exec, backend_container);

  std::fprintf(stderr, "[bridge] Connected to backend.\n");

  // -- Frontend executor: TCP server for Ada client ---------------------------

  // Write port file before blocking accept so clients know the port
  if (!port_file.empty()) {
    std::ofstream pf(port_file);
    pf << frontend_port << std::endl;
    pf.close();
    std::fprintf(stderr, "[bridge] Wrote frontend port %d to %s\n",
                 frontend_port, port_file.c_str());
  }

  std::fprintf(stderr,
               "[bridge] Waiting for frontend client on port %d...\n",
               frontend_port);

  g_bridge.frontend_exec = pcl_executor_create();
  if (!g_bridge.frontend_exec) {
    std::fprintf(stderr, "[bridge] Failed to create frontend executor\n");
    pcl_socket_transport_destroy(g_bridge.backend_transport);
    pcl_executor_destroy(g_bridge.backend_exec);
    return 1;
  }

  g_bridge.frontend_transport = pcl_socket_transport_create_server(
      frontend_port, g_bridge.frontend_exec);
  if (!g_bridge.frontend_transport) {
    std::fprintf(stderr,
                 "[bridge] Failed to create frontend server on port %d\n",
                 frontend_port);
    pcl_socket_transport_destroy(g_bridge.backend_transport);
    pcl_executor_destroy(g_bridge.backend_exec);
    pcl_executor_destroy(g_bridge.frontend_exec);
    return 1;
  }

  pcl_executor_set_transport(
      g_bridge.frontend_exec,
      pcl_socket_transport_get_transport(g_bridge.frontend_transport));

  // Frontend bridge container (service + subscribers + publishers)
  pcl_callbacks_t frontend_cbs = {};
  frontend_cbs.on_configure = frontend_on_configure;
  pcl_container_t* frontend_container =
      pcl_container_create("bridge_frontend", &frontend_cbs, nullptr);
  pcl_container_configure(frontend_container);
  pcl_container_activate(frontend_container);
  pcl_executor_add(g_bridge.frontend_exec, frontend_container);

  // Frontend gateway (dispatches SVC_REQ from Ada client)
  pcl_container_t* frontend_gateway =
      pcl_socket_transport_gateway_container(g_bridge.frontend_transport);
  pcl_container_configure(frontend_gateway);
  pcl_container_activate(frontend_gateway);
  pcl_executor_add(g_bridge.frontend_exec, frontend_gateway);

  std::fprintf(stderr, "[bridge] Frontend client connected. Spinning...\n");

  // -- Main loop: spin both executors and forward messages --------------------

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::seconds(timeout_secs);
  while (!g_shutdown.load() &&
         std::chrono::steady_clock::now() < deadline) {

    // Spin backend — receives entity_updates, evidence_requirements from server
    pcl_executor_spin_once(g_bridge.backend_exec, 0);

    // Forward pending messages from backend → frontend
    {
      std::lock_guard<std::mutex> lock(g_bridge.mu);
      for (auto& p : g_bridge.to_frontend) {
        if (p.topic == "standard.entity_matches" &&
            g_bridge.pub_entity_matches) {
          pcl_msg_t msg = {};
          msg.data      = p.data.data();
          msg.size      = static_cast<uint32_t>(p.data.size());
          msg.type_name = p.type_name.c_str();
          pcl_port_publish(g_bridge.pub_entity_matches, &msg);
        } else if (p.topic == "standard.evidence_requirements" &&
                   g_bridge.pub_evidence_reqs) {
          pcl_msg_t msg = {};
          msg.data      = p.data.data();
          msg.size      = static_cast<uint32_t>(p.data.size());
          msg.type_name = p.type_name.c_str();
          pcl_port_publish(g_bridge.pub_evidence_reqs, &msg);
        }
      }
      g_bridge.to_frontend.clear();
    }

    // Spin frontend — serves Ada client, receives standard.object_evidence
    pcl_executor_spin_once(g_bridge.frontend_exec, 0);

    // Forward pending messages from frontend → backend
    {
      std::lock_guard<std::mutex> lock(g_bridge.mu);
      for (auto& p : g_bridge.to_backend) {
        pcl_msg_t msg = {};
        msg.data      = p.data.data();
        msg.size      = static_cast<uint32_t>(p.data.size());
        msg.type_name = p.type_name.c_str();
        pcl_executor_publish(g_bridge.backend_exec, p.topic.c_str(), &msg);
      }
      g_bridge.to_backend.clear();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  std::fprintf(stderr, "[bridge] Shutting down.\n");

  pcl_socket_transport_destroy(g_bridge.frontend_transport);
  pcl_executor_destroy(g_bridge.frontend_exec);
  pcl_container_destroy(frontend_container);

  pcl_socket_transport_destroy(g_bridge.backend_transport);
  pcl_executor_destroy(g_bridge.backend_exec);
  pcl_container_destroy(backend_container);

  return 0;
}
