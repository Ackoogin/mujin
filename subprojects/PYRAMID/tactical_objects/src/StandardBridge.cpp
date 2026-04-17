#include <StandardBridge.h>
#include <TacticalObjectsCodec.h>
#include <StreamingCodec.h>
#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include <uuid/UUIDHelper.h>

#include <nlohmann/json.hpp>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_log.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace tactical_objects {

using json = nlohmann::json;
namespace tactical_codec = pyramid::data_model::tactical;
namespace data_model = pyramid::data_model;

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
  req.policy  = static_cast<data_model::DataPolicy>(j.value("policy", 0));
  int dim_int = j.value("dimension", 0);
  if (dim_int != 0) {
    req.dimension.push_back(static_cast<data_model::BattleDimension>(dim_int));
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

// ---------------------------------------------------------------------------
// Topic / service name constants
// ---------------------------------------------------------------------------

static const char* SVC_CREATE_REQUIREMENT     = "object_of_interest.create_requirement";
static const char* SVC_READ_MATCH             = "matching_objects.read_match";
static const char* SVC_READ_DETAIL            = "specific_object_detail.read_detail";
static const char* SVC_SUBSCRIBE_INTEREST     = "subscribe_interest";
static const char* SVC_QUERY                  = "query";
static const char* SVC_CREATE_EVIDENCE_REQ    = "object_solution_evidence.create_requirement";

static const char* TOPIC_STD_ENTITY_MATCHES   = "standard.entity_matches";
static const char* TOPIC_STD_EVIDENCE_REQS    = "standard.evidence_requirements";
static const char* TOPIC_STD_OBJECT_EVIDENCE  = "standard.object_evidence";

static const char* TYPE_JSON   = "application/json";

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StandardBridge::StandardBridge(TacticalObjectsRuntime& runtime, pcl_executor_t* exec,
                               bool expose_consumed_interface)
  : pcl::Component("standard_bridge"),
    runtime_(runtime),
    exec_(exec),
    expose_consumed_interface_(expose_consumed_interface)
{}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

pcl_status_t StandardBridge::on_configure() {
  // Provided standard services
  if (!addService(SVC_CREATE_REQUIREMENT, TYPE_JSON, handleCreateRequirement, this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_READ_MATCH,         TYPE_JSON, handleReadMatch,         this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_READ_DETAIL,        TYPE_JSON, handleReadDetail,        this)) return PCL_ERR_CALLBACK;

  // Publisher for standard output matches
  pub_entity_matches_ = addPublisher(TOPIC_STD_ENTITY_MATCHES,  TYPE_JSON);

  if (expose_consumed_interface_) {
    // Subscribe to standard input topic from local PYRAMID evidence providers.
    addSubscriber(TOPIC_STD_OBJECT_EVIDENCE, TYPE_JSON, onStandardObjectEvidence, this);
    pub_evidence_reqs_  = addPublisher(TOPIC_STD_EVIDENCE_REQS, TYPE_JSON);
  }

  return PCL_OK;
}

pcl_status_t StandardBridge::on_activate() {
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Service: object_of_interest.create_requirement
// ---------------------------------------------------------------------------

pcl_status_t StandardBridge::handleCreateRequirement(pcl_container_t*,
                                                      const pcl_msg_t* request,
                                                      pcl_msg_t* response,
                                                      pcl_svc_context_t*,
                                                      void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  tactical_codec::ObjectInterestRequirement req;
  try {
    req = tactical_codec::fromJson(
        str, static_cast<tactical_codec::ObjectInterestRequirement*>(nullptr));
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  // --- Translate proto-native request fields to internal subscribe_interest format ---

  json internal;

  internal["query_mode"] =
      (req.policy == pyramid::data_model::common::DataPolicy::Obtain)
          ? "active_find"
          : "read_current";

  if (!req.dimension.empty()) {
    // Ordinals now match — direct cast to internal BattleDimension.
    switch (static_cast<BattleDimension>(static_cast<int>(req.dimension.front()))) {
      case BattleDimension::Ground:     internal["battle_dimension"] = "Ground";     break;
      case BattleDimension::Air:        internal["battle_dimension"] = "Air";        break;
      case BattleDimension::SeaSurface: internal["battle_dimension"] = "SeaSurface"; break;
      case BattleDimension::Subsurface: internal["battle_dimension"] = "Subsurface"; break;
      case BattleDimension::Space:      internal["battle_dimension"] = "Space";      break;
      case BattleDimension::SOF:        internal["battle_dimension"] = "SOF";        break;
      default: break;  // Unspecified / Unknown: no dimension filter
    }
  }

  auto set_bbox = [&](double min_lat, double max_lat, double min_lon, double max_lon) {
    json area;
    area["min_lat"] = min_lat;
    area["max_lat"] = max_lat;
    area["min_lon"] = min_lon;
    area["max_lon"] = max_lon;
    internal["area"] = area;
    internal["expires_at"] = 9999;
  };

  if (req.point.has_value()) {
    const auto& point = req.point.value().position;
    set_bbox(point.latitude, point.latitude, point.longitude, point.longitude);
  } else if (req.circle_area.has_value()) {
    const auto& area = req.circle_area.value();
    set_bbox(area.position.latitude - area.radius,
             area.position.latitude + area.radius,
             area.position.longitude - area.radius,
             area.position.longitude + area.radius);
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

  // --- Call internal subscribe_interest via executor ---

  std::string internal_str = internal.dump();
  pcl_msg_t ireq = {};
  ireq.data      = internal_str.data();
  ireq.size      = static_cast<uint32_t>(internal_str.size());
  ireq.type_name = TYPE_JSON;

  // Buffer for subscribe_interest response
  char resp_raw[2048] = {};
  pcl_msg_t iresp = {};
  iresp.data      = resp_raw;
  iresp.size      = sizeof(resp_raw);
  iresp.type_name = TYPE_JSON;

  pcl_status_t st = pcl_executor_invoke_service(self->exec_,
                                                 SVC_SUBSCRIBE_INTEREST,
                                                 &ireq, &iresp);
  if (st != PCL_OK) {
    self->resp_buf_ = "{\"error\":\"subscribe_interest failed\"}";
    response->data      = self->resp_buf_.data();
    response->size      = static_cast<uint32_t>(self->resp_buf_.size());
    response->type_name = TYPE_JSON;
    return PCL_OK;  // Return OK so the Ada client gets a response
  }

  // Parse internal response and return standard format
  std::string iresp_str(static_cast<const char*>(iresp.data), iresp.size);
  json iresp_j;
  try {
    iresp_j = json::parse(iresp_str);
  } catch (...) {
    iresp_j = json::object();
  }

  // Register a streaming subscription handle so on_tick can call assembleStreamFrame.
  std::string interest_id_str = iresp_j.value("interest_id", "");
  if (!interest_id_str.empty()) {
    auto parsed = pyramid::core::uuid::UUIDHelper::fromString(interest_id_str);
    if (parsed.second) {
      UUIDKey interest_key(parsed.first);
      SubscriptionHandle handle = self->runtime_.registerStreamSubscriber(
          interest_key,
          [](const UUIDKey&, const UUIDKey&){});
      self->interests_.emplace_back(interest_key, handle);
      pcl_log(nullptr, PCL_LOG_INFO,
              "[StandardBridge] registered interest %s handle=%llu",
              interest_id_str.c_str(), (unsigned long long)handle);
    }
  }

  // For ActiveFind: forward evidence requirements to the local PYRAMID consumed
  // service and optionally publish the standard topic for in-process consumers.
  if (self->expose_consumed_interface_ &&
      iresp_j.contains("evidence_requirements") &&
      iresp_j["evidence_requirements"].is_array()) {
    for (const auto& ev : iresp_j["evidence_requirements"]) {
      const auto req = evidence_requirement_from_internal_json(ev);
      const std::string ev_str = tactical_codec::toJson(req);

      pcl_msg_t ev_req = {};
      ev_req.data = ev_str.data();
      ev_req.size = static_cast<uint32_t>(ev_str.size());
      ev_req.type_name = TYPE_JSON;

      char ev_resp_raw[1024] = {};
      pcl_msg_t ev_resp = {};
      ev_resp.data = ev_resp_raw;
      ev_resp.size = sizeof(ev_resp_raw);
      ev_resp.type_name = TYPE_JSON;

      const pcl_status_t evidence_rc =
          pcl_executor_invoke_service(self->exec_, SVC_CREATE_EVIDENCE_REQ, &ev_req, &ev_resp);
      if (evidence_rc != PCL_OK) {
        pcl_log(nullptr, PCL_LOG_WARN,
                "[StandardBridge] local consumed service %s unavailable",
                SVC_CREATE_EVIDENCE_REQ);
      }

      if (self->pub_evidence_reqs_) {
        pcl_msg_t ev_msg   = {};
        ev_msg.data        = ev_str.data();
        ev_msg.size        = static_cast<uint32_t>(ev_str.size());
        ev_msg.type_name   = TYPE_JSON;
        pcl_port_publish(self->pub_evidence_reqs_, &ev_msg);
      }
    }
  }

  // Build standard response — expose interest_id for Ada clients
  json std_resp;
  std_resp["interest_id"] = interest_id_str;
  if (iresp_j.contains("solution_id")) {
    std_resp["solution_id"] = iresp_j["solution_id"];
  }

  self->resp_buf_     = std_resp.dump();
  response->data      = self->resp_buf_.data();
  response->size      = static_cast<uint32_t>(self->resp_buf_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Service: matching_objects.read_match
// ---------------------------------------------------------------------------

pcl_status_t StandardBridge::handleReadMatch(pcl_container_t*, const pcl_msg_t* request,
                                              pcl_msg_t* response, pcl_svc_context_t*,
                                              void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);

  // Parse standard Query — extract optional id[] filter list
  std::vector<std::string> filter_ids;
  if (request->data && request->size > 0) {
    try {
      auto q = json::parse(std::string(static_cast<const char*>(request->data), request->size));
      if (q.contains("id") && q["id"].is_array()) {
        for (const auto& item : q["id"]) filter_ids.push_back(item.get<std::string>());
      }
    } catch (...) {}
  }

  json result_arr = json::array();

  auto call_query = [&](const std::string& payload) {
    char raw[65536] = {};
    pcl_msg_t ireq{}, iresp{};
    ireq.data      = payload.empty() ? nullptr : static_cast<const void*>(payload.data());
    ireq.size      = static_cast<uint32_t>(payload.size());
    ireq.type_name = TYPE_JSON;
    iresp.data     = raw;
    iresp.size     = sizeof(raw);
    if (pcl_executor_invoke_service(self->exec_, SVC_QUERY, &ireq, &iresp) != PCL_OK) return;
    try {
      auto jr = json::parse(std::string(static_cast<const char*>(iresp.data), iresp.size));
      for (const auto& entry : jr.value("entries", json::array())) {
        std::string id_str = entry.value("id", "");
        if (id_str.empty()) continue;
        data_model::ObjectMatch m{};
        m.id = id_str;
        m.matching_object_id = id_str;
        m.source = "tactical_objects";
        result_arr.push_back(json::parse(tactical_codec::toJson(m)));
      }
    } catch (...) {}
  };

  if (filter_ids.empty()) {
    call_query("");
  } else {
    for (const auto& id_str : filter_ids) {
      call_query(json{{"by_uuid", id_str}}.dump());
    }
  }

  self->resp_buf_     = result_arr.dump();
  response->data      = self->resp_buf_.data();
  response->size      = static_cast<uint32_t>(self->resp_buf_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Service: specific_object_detail.read_detail
// ---------------------------------------------------------------------------

pcl_status_t StandardBridge::handleReadDetail(pcl_container_t*, const pcl_msg_t* request,
                                               pcl_msg_t* response, pcl_svc_context_t*,
                                               void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);

  std::vector<std::string> ids;
  if (request->data && request->size > 0) {
    try {
      auto q = json::parse(std::string(static_cast<const char*>(request->data), request->size));
      if (q.contains("id") && q["id"].is_array()) {
        for (const auto& item : q["id"]) ids.push_back(item.get<std::string>());
      }
    } catch (...) {}
  }

  json result_arr = json::array();
  for (const auto& id_str : ids) {
    auto parsed = pyramid::core::uuid::UUIDHelper::fromString(id_str);
    if (!parsed.second) continue;
    UUIDKey obj_id(parsed.first);

    if (!self->runtime_.getRecord(obj_id)) continue;

    data_model::ObjectDetail detail{};
    detail.id = id_str;

    const auto* kc = self->runtime_.store()->kinematics().get(obj_id);
    if (kc) {
      detail.position.latitude  = kc->position.lat;  // radians — standard format
      detail.position.longitude = kc->position.lon;
    }

    const auto* qc = self->runtime_.store()->quality().get(obj_id);
    if (qc) detail.quality = qc->confidence;

    result_arr.push_back(json::parse(tactical_codec::toJson(detail)));
  }

  self->resp_buf_     = result_arr.dump();
  response->data      = self->resp_buf_.data();
  response->size      = static_cast<uint32_t>(self->resp_buf_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// on_tick: assemble entity matches directly via runtime and publish as JSON
// ---------------------------------------------------------------------------
//
// This bypasses the PCL pub/sub path for entity_updates (which fails when a
// socket transport is active, because pcl_executor_publish routes to the
// transport and never calls dispatch_incoming_now for local subscribers).

pcl_status_t StandardBridge::on_tick(double dt) {
  if (!pub_entity_matches_ || interests_.empty()) return PCL_OK;

  for (auto& [interest_id, handle] : interests_) {
    StreamFrame sf = runtime_.assembleStreamFrame(interest_id, handle, dt);
    if (sf.updates.empty() && sf.deletes.empty()) continue;

    pcl_log(nullptr, PCL_LOG_INFO,
            "[StandardBridge] assembleStreamFrame interest=%s updates=%zu deletes=%zu",
            TacticalObjectsCodec::encodeUUID(interest_id).get<std::string>().c_str(),
            sf.updates.size(), sf.deletes.size());

    json arr = json::array();
    for (const auto& upd : sf.updates) {
      if (upd.message_type == STREAM_MSG_ENTITY_DELETE) continue;

      data_model::ObjectMatch obj{};
      obj.id = TacticalObjectsCodec::encodeUUID(upd.entity_id).get<std::string>();
      obj.source = "standard_bridge";
      obj.matching_object_id = obj.id;
      arr.push_back(nlohmann::json::parse(tactical_codec::toJson(obj)));
    }

    if (arr.empty()) continue;

    pcl_log(nullptr, PCL_LOG_INFO,
            "[StandardBridge] publishing %zu standard.entity_matches entries",
            arr.size());

    pub_buf_          = arr.dump();
    pcl_msg_t out     = {};
    out.data          = pub_buf_.data();
    out.size          = static_cast<uint32_t>(pub_buf_.size());
    out.type_name     = TYPE_JSON;
    pcl_port_publish(pub_entity_matches_, &out);
  }

  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Subscriber: standard.object_evidence (JSON) → processObservationBatch
// ---------------------------------------------------------------------------

void StandardBridge::onStandardObjectEvidence(pcl_container_t*,
                                               const pcl_msg_t* msg,
                                               void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  if (!msg->data || msg->size == 0) return;

  data_model::ObjectDetail detail;
  try {
    detail = tactical_codec::fromJson(
        std::string(static_cast<const char*>(msg->data), msg->size),
        static_cast<data_model::ObjectDetail*>(nullptr));
  } catch (...) {
    return;
  }

  Observation obs;
  obs.observation_id  = pyramid::core::uuid::UUIDHelper::generateV4();
  obs.received_at     = 0.0;
  obs.observed_at     = detail.creation_time;
  obs.object_hint_type = ObjectType::Platform;
  obs.affiliation_hint = Affiliation::Unknown;
  obs.confidence       = detail.quality.value_or(0.0);

  obs.position.lat = detail.position.latitude;
  obs.position.lon = detail.position.longitude;
  obs.position.alt = 0.0;

  // Ordinals now match — direct cast from generated StandardIdentity to internal Affiliation.
  obs.affiliation_hint = static_cast<Affiliation>(static_cast<int>(detail.identity));

  // Ordinals now match — direct cast from generated BattleDimension to internal BattleDimension.
  auto dim = static_cast<BattleDimension>(static_cast<int>(detail.dimension));
  switch (dim) {
    case BattleDimension::SeaSurface: obs.source_sidc = "SHSP------*****"; break;
    case BattleDimension::Air:        obs.source_sidc = "SHAP------*****"; break;
    case BattleDimension::Subsurface: obs.source_sidc = "SHUP------*****"; break;
    case BattleDimension::Ground:     obs.source_sidc = "SHGP------*****"; break;
    default: break;
  }

  ObservationBatch batch;
  batch.observations.push_back(obs);
  pcl_log(nullptr, PCL_LOG_INFO,
          "[StandardBridge] processing standard.object_evidence identity=%d dim=%d lat=%f lon=%f",
          static_cast<int>(obs.affiliation_hint), static_cast<int>(dim),
          obs.position.lat, obs.position.lon);
  self->runtime_.processObservationBatch(batch);
}

} // namespace tactical_objects
