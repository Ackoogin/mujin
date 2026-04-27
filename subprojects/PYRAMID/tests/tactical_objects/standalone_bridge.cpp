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
#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#endif
#include <pcl/pcl_executor.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>

#include <nlohmann/json.hpp>

#include <algorithm>
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
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
#endif
namespace data_model = pyramid::data_model;
namespace tactical_codec = pyramid::data_model::tactical;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

static std::atomic<bool> g_shutdown{false};
static void signal_handler(int) { g_shutdown.store(true); }
static constexpr const char* kJsonContentType = "application/json";
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
static constexpr const char* kFlatBuffersContentType = "application/flatbuffers";
#endif

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
  std::string frontend_content_type = kJsonContentType;
};

static BridgeState g_bridge;

static bool is_flatbuffers_content_type(const char* content_type) {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
  return content_type &&
         std::strcmp(content_type, kFlatBuffersContentType) == 0;
#else
  (void)content_type;
  return false;
#endif
}

static const char* normalize_content_type(const char* content_type) {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
  return is_flatbuffers_content_type(content_type)
             ? kFlatBuffersContentType
             : kJsonContentType;
#else
  (void)content_type;
  return kJsonContentType;
#endif
}

static data_model::StandardIdentity standardIdentityFromAffiliation(Affiliation a) {
  switch (a) {
    case Affiliation::Friendly: return data_model::StandardIdentity::Friendly;
    case Affiliation::Hostile: return data_model::StandardIdentity::Hostile;
    case Affiliation::Neutral: return data_model::StandardIdentity::Neutral;
    case Affiliation::Unknown: return data_model::StandardIdentity::Unknown;
    case Affiliation::AssumedFriend: return data_model::StandardIdentity::AssumedFriendly;
    case Affiliation::Suspect: return data_model::StandardIdentity::Suspect;
    case Affiliation::Joker: return data_model::StandardIdentity::Joker;
    case Affiliation::Faker: return data_model::StandardIdentity::Faker;
    case Affiliation::Pending: return data_model::StandardIdentity::Pending;
  }
  return data_model::StandardIdentity::Unknown;
}

static data_model::BattleDimension standardBattleDimension(BattleDimension d) {
  switch (d) {
    case BattleDimension::Ground: return data_model::BattleDimension::Ground;
    case BattleDimension::Air: return data_model::BattleDimension::Air;
    case BattleDimension::SeaSurface: return data_model::BattleDimension::SeaSurface;
    case BattleDimension::Subsurface: return data_model::BattleDimension::Subsurface;
    case BattleDimension::Space: return data_model::BattleDimension::Unknown;
    case BattleDimension::SOF: return data_model::BattleDimension::Unknown;
  }
  return data_model::BattleDimension::Unspecified;
}

static std::string standardIdentityToString(data_model::StandardIdentity identity) {
  switch (identity) {
    case data_model::StandardIdentity::Friendly: return "STANDARD_IDENTITY_FRIENDLY";
    case data_model::StandardIdentity::Hostile: return "STANDARD_IDENTITY_HOSTILE";
    case data_model::StandardIdentity::Neutral: return "STANDARD_IDENTITY_NEUTRAL";
    case data_model::StandardIdentity::Unknown: return "STANDARD_IDENTITY_UNKNOWN";
    case data_model::StandardIdentity::Suspect: return "STANDARD_IDENTITY_SUSPECT";
    case data_model::StandardIdentity::Pending: return "STANDARD_IDENTITY_PENDING";
    case data_model::StandardIdentity::Joker: return "STANDARD_IDENTITY_JOKER";
    case data_model::StandardIdentity::Faker: return "STANDARD_IDENTITY_FAKER";
    case data_model::StandardIdentity::AssumedFriendly: return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
    case data_model::StandardIdentity::Unspecified: break;
  }
  return "STANDARD_IDENTITY_UNSPECIFIED";
}

static data_model::StandardIdentity standardIdentityFromString(const std::string& identity) {
  if (identity == "STANDARD_IDENTITY_FRIENDLY") return data_model::StandardIdentity::Friendly;
  if (identity == "STANDARD_IDENTITY_HOSTILE") return data_model::StandardIdentity::Hostile;
  if (identity == "STANDARD_IDENTITY_NEUTRAL") return data_model::StandardIdentity::Neutral;
  if (identity == "STANDARD_IDENTITY_UNKNOWN") return data_model::StandardIdentity::Unknown;
  if (identity == "STANDARD_IDENTITY_SUSPECT") return data_model::StandardIdentity::Suspect;
  if (identity == "STANDARD_IDENTITY_PENDING") return data_model::StandardIdentity::Pending;
  if (identity == "STANDARD_IDENTITY_JOKER") return data_model::StandardIdentity::Joker;
  if (identity == "STANDARD_IDENTITY_FAKER") return data_model::StandardIdentity::Faker;
  if (identity == "STANDARD_IDENTITY_ASSUMED_FRIENDLY") return data_model::StandardIdentity::AssumedFriendly;
  return data_model::StandardIdentity::Unspecified;
}

static std::string battleDimensionToString(data_model::BattleDimension dimension) {
  switch (dimension) {
    case data_model::BattleDimension::Ground: return "BATTLE_DIMENSION_GROUND";
    case data_model::BattleDimension::Air: return "BATTLE_DIMENSION_AIR";
    case data_model::BattleDimension::SeaSurface: return "BATTLE_DIMENSION_SEA_SURFACE";
    case data_model::BattleDimension::Subsurface: return "BATTLE_DIMENSION_SUBSURFACE";
    case data_model::BattleDimension::Unknown: return "BATTLE_DIMENSION_UNKNOWN";
    case data_model::BattleDimension::Unspecified: break;
  }
  return "BATTLE_DIMENSION_UNSPECIFIED";
}

static data_model::BattleDimension battleDimensionFromString(const std::string& dimension) {
  if (dimension == "BATTLE_DIMENSION_GROUND") return data_model::BattleDimension::Ground;
  if (dimension == "BATTLE_DIMENSION_AIR") return data_model::BattleDimension::Air;
  if (dimension == "BATTLE_DIMENSION_SEA_SURFACE") return data_model::BattleDimension::SeaSurface;
  if (dimension == "BATTLE_DIMENSION_SUBSURFACE") return data_model::BattleDimension::Subsurface;
  if (dimension == "BATTLE_DIMENSION_UNKNOWN") return data_model::BattleDimension::Unknown;
  return data_model::BattleDimension::Unspecified;
}

static data_model::BattleDimension battleDimensionFromOrdinal(int dimension) {
  switch (dimension) {
    case 1: return data_model::BattleDimension::Ground;
    case 2: return data_model::BattleDimension::Subsurface;
    case 3: return data_model::BattleDimension::SeaSurface;
    case 4: return data_model::BattleDimension::Air;
    case 5: return data_model::BattleDimension::Unknown;
    default: return data_model::BattleDimension::Unspecified;
  }
}

static data_model::DataPolicy dataPolicyFromString(const std::string& policy) {
  if (policy == "DATA_POLICY_OBTAIN") return data_model::DataPolicy::Obtain;
  if (policy == "DATA_POLICY_QUERY") return data_model::DataPolicy::Query;
  return data_model::DataPolicy::Unspecified;
}

static data_model::DataPolicy dataPolicyFromOrdinal(int policy) {
  switch (policy) {
    case 1: return data_model::DataPolicy::Query;
    case 2: return data_model::DataPolicy::Obtain;
    default: return data_model::DataPolicy::Unspecified;
  }
}

static std::string dataPolicyToString(data_model::DataPolicy policy) {
  switch (policy) {
    case data_model::DataPolicy::Obtain: return "DATA_POLICY_OBTAIN";
    case data_model::DataPolicy::Query: return "DATA_POLICY_QUERY";
    case data_model::DataPolicy::Unspecified: break;
  }
  return "DATA_POLICY_UNSPECIFIED";
}

static std::string encode_entity_matches_payload(const std::vector<data_model::ObjectMatch>& matches,
                                                 const char* content_type) {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
  if (is_flatbuffers_content_type(content_type)) {
    return flatbuffers_codec::toBinary(matches);
  }
#else
  (void)content_type;
#endif
  std::string payload = "[";
  bool first = true;
  for (const auto& match : matches) {
    if (!first) payload += ",";
    first = false;
    payload += tactical_codec::toJson(match);
  }
  payload += "]";
  return payload;
}

static std::string encode_evidence_requirement_payload(const data_model::ObjectEvidenceRequirement& req,
                                                       const char* content_type) {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
  if (is_flatbuffers_content_type(content_type)) {
    return flatbuffers_codec::toBinary(req);
  }
#else
  (void)content_type;
#endif
  return tactical_codec::toJson(req);
}

static std::string encode_identifier_payload(const data_model::Identifier& id,
                                             const char* content_type) {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
  if (is_flatbuffers_content_type(content_type)) {
    return flatbuffers_codec::toBinary(id);
  }
#else
  (void)content_type;
#endif
  return json(id).dump();
}

static bool decode_object_evidence(const pcl_msg_t* msg, data_model::ObjectDetail& evidence) {
  if (!msg || !msg->data || msg->size == 0) {
    return false;
  }
  try {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
    if (is_flatbuffers_content_type(msg->type_name)) {
      evidence = flatbuffers_codec::fromBinaryObjectDetail(msg->data, msg->size);
    } else {
#endif
      evidence = tactical_codec::fromJson(
          std::string(static_cast<const char*>(msg->data), msg->size),
          static_cast<data_model::ObjectDetail*>(nullptr));
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
    }
#endif
    return true;
  } catch (...) {
    return false;
  }
}

static data_model::PolyArea make_poly_area(double min_lat_rad,
                                           double max_lat_rad,
                                           double min_lon_rad,
                                           double max_lon_rad) {
  data_model::PolyArea area;
  area.points.push_back({min_lat_rad, min_lon_rad});
  area.points.push_back({min_lat_rad, max_lon_rad});
  area.points.push_back({max_lat_rad, max_lon_rad});
  area.points.push_back({max_lat_rad, min_lon_rad});
  return area;
}

static data_model::ObjectEvidenceRequirement evidence_requirement_from_internal_json(const json& j) {
  data_model::ObjectEvidenceRequirement req{};
  req.base.id = j.value("id", "");
  if (j.contains("policy") && j["policy"].is_number_integer()) {
    req.policy = dataPolicyFromOrdinal(j["policy"].get<int>());
  } else {
    req.policy = dataPolicyFromString(j.value("policy", ""));
  }
  const auto dim =
      j.contains("dimension") && j["dimension"].is_number_integer()
          ? battleDimensionFromOrdinal(j["dimension"].get<int>())
          : battleDimensionFromString(j.value("dimension", ""));
  if (dim != data_model::BattleDimension::Unspecified) {
    req.dimension.push_back(dim);
  }
  if (j.contains("min_lat_rad") && j.contains("max_lat_rad") &&
      j.contains("min_lon_rad") && j.contains("max_lon_rad")) {
    req.poly_area = make_poly_area(
        j.value("min_lat_rad", 0.0),
        j.value("max_lat_rad", 0.0),
        j.value("min_lon_rad", 0.0),
        j.value("max_lon_rad", 0.0));
  }
  return req;
}

static bool decode_create_requirement(const pcl_msg_t* msg,
                                      data_model::ObjectInterestRequirement& req) {
  if (!msg || !msg->data || msg->size == 0) {
    return false;
  }
  try {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
    if (is_flatbuffers_content_type(msg->type_name)) {
      req = flatbuffers_codec::fromBinaryObjectInterestRequirement(msg->data, msg->size);
    } else {
#endif
      req = tactical_codec::fromJson(
          std::string(static_cast<const char*>(msg->data), msg->size),
          static_cast<data_model::ObjectInterestRequirement*>(nullptr));
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
    }
#endif
    return true;
  } catch (...) {
    return false;
  }
}

// ---------------------------------------------------------------------------
// Standard <-> Internal enum converters (same as StandardBridge.cpp)
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
// Backend subscriber: entity_updates (binary) -> standard.entity_matches (JSON)
// ---------------------------------------------------------------------------

static void backend_entity_updates_cb(pcl_container_t*, const pcl_msg_t* msg,
                                      void*) {
  if (!msg || !msg->data || msg->size == 0) return;

  auto frames = StreamingCodec::decodeBatchFrame(
      static_cast<const uint8_t*>(msg->data), msg->size);

  std::vector<data_model::ObjectMatch> matches;
  for (const auto& upd : frames) {
    if (upd.message_type == STREAM_MSG_ENTITY_DELETE) continue;

    data_model::ObjectMatch obj;
    obj.id = TacticalObjectsCodec::encodeUUID(upd.entity_id).get<std::string>();
    obj.matching_object_id = obj.id;
    matches.push_back(obj);
  }

  if (matches.empty()) return;

  std::string payload = encode_entity_matches_payload(
      matches, g_bridge.frontend_content_type.c_str());
  std::lock_guard<std::mutex> lock(g_bridge.mu);
  g_bridge.to_frontend.push_back(
      {"standard.entity_matches", g_bridge.frontend_content_type,
       std::move(payload)});
}

// ---------------------------------------------------------------------------
// Backend subscriber: evidence_requirements (JSON) -> standard.evidence_requirements
// ---------------------------------------------------------------------------

static void backend_evidence_reqs_cb(pcl_container_t*, const pcl_msg_t* msg,
                                     void*) {
  if (!msg || !msg->data || msg->size == 0) return;

  try {
    const auto j = json::parse(
        std::string(static_cast<const char*>(msg->data), msg->size));
    auto req = evidence_requirement_from_internal_json(j);
    std::string payload = encode_evidence_requirement_payload(
        req, g_bridge.frontend_content_type.c_str());
    std::lock_guard<std::mutex> lock(g_bridge.mu);
    g_bridge.to_frontend.push_back(
        {"standard.evidence_requirements", g_bridge.frontend_content_type,
         std::move(payload)});
  } catch (...) {
    return;
  }
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
// Frontend subscriber: standard.object_evidence -> observation_ingress
// ---------------------------------------------------------------------------

static void frontend_object_evidence_cb(pcl_container_t*, const pcl_msg_t* msg,
                                        void*) {
  data_model::ObjectDetail evidence;
  if (!decode_object_evidence(msg, evidence)) return;

  // Translate standard observation to internal format (must match
  // TacticalObjectsCodec::decodeObservation expected schema).
  json internal;
  internal["observations"] = json::array();
  json obs;
  obs["observation_id"] = pyramid::core::uuid::UUIDHelper::toString(
      pyramid::core::uuid::UUIDHelper::generateV4());
  obs["position"] = json::object();
  obs["position"]["lat"] = radToDeg(evidence.position.latitude);
  obs["position"]["lon"] = radToDeg(evidence.position.longitude);
  obs["position"]["alt"] = 0.0;
  obs["confidence"] = evidence.quality.value_or(0.0);
  obs["observed_at"] = evidence.creation_time;
  obs["object_hint_type"] = "Platform";
  obs["affiliation_hint"] = "Unknown";

  std::string identity = standardIdentityToString(evidence.identity);
  if (identity == "STANDARD_IDENTITY_HOSTILE")  obs["affiliation_hint"] = "Hostile";
  else if (identity == "STANDARD_IDENTITY_FRIENDLY") obs["affiliation_hint"] = "Friendly";
  else if (identity == "STANDARD_IDENTITY_NEUTRAL")  obs["affiliation_hint"] = "Neutral";

  std::string dimension = battleDimensionToString(evidence.dimension);
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
                                                 pcl_svc_context_t*,
                                                 void*) {
  data_model::ObjectInterestRequirement req;
  if (!decode_create_requirement(request, req)) return PCL_ERR_INVALID;
  const char* frontend_content_type = normalize_content_type(
      request ? request->type_name : nullptr);
  std::fprintf(stderr, "[bridge] create_requirement received type=%s\n",
               frontend_content_type);

  // Translate standard -> internal subscribe_interest format
  json internal;

  internal["query_mode"] = (req.policy == data_model::DataPolicy::Obtain)
                               ? "active_find"
                               : "read_current";

  if (!req.dimension.empty()) {
    const std::string dimension = battleDimensionToString(req.dimension.front());
    if (dimension == "BATTLE_DIMENSION_GROUND") internal["battle_dimension"] = "Ground";
    else if (dimension == "BATTLE_DIMENSION_AIR") internal["battle_dimension"] = "Air";
    else if (dimension == "BATTLE_DIMENSION_SEA_SURFACE") internal["battle_dimension"] = "SeaSurface";
    else if (dimension == "BATTLE_DIMENSION_SUBSURFACE") internal["battle_dimension"] = "Subsurface";
  }

  auto set_bbox = [&](double min_lat, double max_lat, double min_lon, double max_lon) {
    json area;
    area["min_lat"] = radToDeg(min_lat);
    area["max_lat"] = radToDeg(max_lat);
    area["min_lon"] = radToDeg(min_lon);
    area["max_lon"] = radToDeg(max_lon);
    internal["area"] = area;
    internal["expires_at"] = 9999;
  };

  if (req.point.has_value()) {
    const auto& p = req.point->position;
    set_bbox(p.latitude, p.latitude, p.longitude, p.longitude);
  } else if (req.circle_area.has_value()) {
    const auto& c = req.circle_area.value();
    set_bbox(c.position.latitude - c.radius,
             c.position.latitude + c.radius,
             c.position.longitude - c.radius,
             c.position.longitude + c.radius);
  } else if (req.poly_area.has_value() && !req.poly_area->points.empty()) {
    double min_lat = req.poly_area->points.front().latitude;
    double max_lat = min_lat;
    double min_lon = req.poly_area->points.front().longitude;
    double max_lon = min_lon;
    for (const auto& point : req.poly_area->points) {
      min_lat = std::min(min_lat, point.latitude);
      max_lat = std::max(max_lat, point.latitude);
      min_lon = std::min(min_lon, point.longitude);
      max_lon = std::max(max_lon, point.longitude);
    }
    set_bbox(min_lat, max_lat, min_lon, max_lon);
  }

  std::string internal_str = internal.dump();
  std::fprintf(stderr, "[bridge] -> subscribe_interest: %s\n",
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
    g_bridge.resp_buf = encode_identifier_payload({}, frontend_content_type);
    response->data      = g_bridge.resp_buf.data();
    response->size      = static_cast<uint32_t>(g_bridge.resp_buf.size());
    response->type_name = frontend_content_type;
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
    g_bridge.resp_buf = encode_identifier_payload({}, frontend_content_type);
    response->data      = g_bridge.resp_buf.data();
    response->size      = static_cast<uint32_t>(g_bridge.resp_buf.size());
    response->type_name = frontend_content_type;
    return PCL_OK;
  }

  std::fprintf(stderr, "[bridge] <- subscribe_interest: %s\n",
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
      try {
        auto req_payload = evidence_requirement_from_internal_json(ev);
        g_bridge.to_frontend.push_back(
            {"standard.evidence_requirements", g_bridge.frontend_content_type,
             encode_evidence_requirement_payload(
                 req_payload, g_bridge.frontend_content_type.c_str())});
      } catch (...) {
        continue;
      }
    }
  }

  g_bridge.resp_buf = encode_identifier_payload(
      iresp.value("interest_id", ""), frontend_content_type);
  response->data       = g_bridge.resp_buf.data();
  response->size       = static_cast<uint32_t>(g_bridge.resp_buf.size());
  response->type_name  = frontend_content_type;
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Frontend container on_configure
// ---------------------------------------------------------------------------

static pcl_status_t frontend_on_configure(pcl_container_t* c, void*) {
  const char* frontend_type = g_bridge.frontend_content_type.c_str();
  pcl_container_add_subscriber(c, "standard.object_evidence",
                               frontend_type,
                               frontend_object_evidence_cb, nullptr);
  pcl_container_add_service(c, "object_of_interest.create_requirement",
                            frontend_type,
                            frontend_create_requirement, nullptr);
  g_bridge.pub_entity_matches =
      pcl_container_add_publisher(c, "standard.entity_matches",
                                  frontend_type);
  g_bridge.pub_evidence_reqs =
      pcl_container_add_publisher(c, "standard.evidence_requirements",
                                  frontend_type);
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
    } else if (std::strcmp(argv[i], "--frontend-content-type") == 0 && i + 1 < argc) {
      g_bridge.frontend_content_type = normalize_content_type(argv[++i]);
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

    // Spin backend -- receives entity_updates, evidence_requirements from server
    pcl_executor_spin_once(g_bridge.backend_exec, 0);

    // Forward pending messages from backend -> frontend
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

    // Spin frontend -- serves Ada client, receives standard.object_evidence
    pcl_executor_spin_once(g_bridge.frontend_exec, 0);

    // Forward pending messages from frontend -> backend
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
