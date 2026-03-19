#include <StandardBridge.h>
#include <TacticalObjectsCodec.h>
#include <StreamingCodec.h>
#include <uuid/UUIDHelper.h>

#include <nlohmann/json.hpp>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_log.h>

#include <cstring>
#include <string>
#include <vector>

namespace tactical_objects {

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Topic / service name constants
// ---------------------------------------------------------------------------

static const char* SVC_CREATE_REQUIREMENT     = "object_of_interest.create_requirement";
static const char* SVC_SUBSCRIBE_INTEREST     = "subscribe_interest";

static const char* TOPIC_STD_ENTITY_MATCHES   = "standard.entity_matches";
static const char* TOPIC_STD_EVIDENCE_REQS    = "standard.evidence_requirements";
static const char* TOPIC_STD_OBJECT_EVIDENCE  = "standard.object_evidence";

static const char* TYPE_JSON   = "application/json";

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StandardBridge::StandardBridge(TacticalObjectsRuntime& runtime, pcl_executor_t* exec)
  : pcl::Component("standard_bridge"),
    runtime_(runtime),
    exec_(exec)
{}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

pcl_status_t StandardBridge::on_configure() {
  // Provided standard service
  if (!addService(SVC_CREATE_REQUIREMENT, TYPE_JSON, handleCreateRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }

  // Subscribe to standard input topic from Ada clients
  addSubscriber(TOPIC_STD_OBJECT_EVIDENCE, TYPE_JSON, onStandardObjectEvidence, this);

  // Publishers for standard output topics
  pub_entity_matches_ = addPublisher(TOPIC_STD_ENTITY_MATCHES,  TYPE_JSON);
  pub_evidence_reqs_  = addPublisher(TOPIC_STD_EVIDENCE_REQS,   TYPE_JSON);

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
                                                      void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json req_j;
  try {
    req_j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  // --- Translate standard fields to internal subscribe_interest format ---

  json internal;

  // policy → query_mode
  std::string policy = req_j.value("policy", "DATA_POLICY_OBTAIN");
  if (policy == "DATA_POLICY_OBTAIN") {
    internal["query_mode"] = "active_find";
  } else {
    internal["query_mode"] = "read_current";
  }

  // identity → affiliation (standard string → internal string)
  std::string identity = req_j.value("identity", "");
  if (!identity.empty()) {
    Affiliation a = self->standardIdentityToAffiliation(identity);
    // Map back to internal string for subscribe_interest
    switch (a) {
      case Affiliation::Friendly:     internal["affiliation"] = "Friendly";     break;
      case Affiliation::Hostile:      internal["affiliation"] = "Hostile";      break;
      case Affiliation::Neutral:      internal["affiliation"] = "Neutral";      break;
      case Affiliation::Unknown:      internal["affiliation"] = "Unknown";      break;
      case Affiliation::AssumedFriend:internal["affiliation"] = "AssumedFriend";break;
      case Affiliation::Suspect:      internal["affiliation"] = "Suspect";      break;
      case Affiliation::Joker:        internal["affiliation"] = "Joker";        break;
      case Affiliation::Faker:        internal["affiliation"] = "Faker";        break;
      case Affiliation::Pending:      internal["affiliation"] = "Pending";      break;
    }
  }

  // dimension → battle_dimension (standard string → internal string)
  std::string dimension = req_j.value("dimension", "");
  if (!dimension.empty()) {
    BattleDimension d = self->standardToBattleDim(dimension);
    switch (d) {
      case BattleDimension::Ground:    internal["battle_dimension"] = "Ground";    break;
      case BattleDimension::Air:       internal["battle_dimension"] = "Air";       break;
      case BattleDimension::SeaSurface:internal["battle_dimension"] = "SeaSurface";break;
      case BattleDimension::Subsurface:internal["battle_dimension"] = "Subsurface";break;
      case BattleDimension::Space:     internal["battle_dimension"] = "Space";     break;
      case BattleDimension::SOF:       internal["battle_dimension"] = "SOF";       break;
    }
  }

  // lat/lon radians → degrees for internal bounding box
  if (req_j.contains("min_lat_rad") || req_j.contains("max_lat_rad")) {
    json area;
    area["min_lat"] = self->radToDeg(req_j.value("min_lat_rad", 0.0));
    area["max_lat"] = self->radToDeg(req_j.value("max_lat_rad", 0.0));
    area["min_lon"] = self->radToDeg(req_j.value("min_lon_rad", 0.0));
    area["max_lon"] = self->radToDeg(req_j.value("max_lon_rad", 0.0));
    internal["area"] = area;
    internal["expires_at"] = 9999;
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

  // For ActiveFind: publish evidence_requirements directly (can't rely on PCL pub/sub
  // to route from TacticalObjectsComponent when a socket transport is active).
  if (iresp_j.contains("evidence_requirements") &&
      iresp_j["evidence_requirements"].is_array() &&
      self->pub_evidence_reqs_) {
    for (const auto& ev : iresp_j["evidence_requirements"]) {
      std::string ev_str = ev.dump();
      pcl_msg_t ev_msg   = {};
      ev_msg.data        = ev_str.data();
      ev_msg.size        = static_cast<uint32_t>(ev_str.size());
      ev_msg.type_name   = TYPE_JSON;
      pcl_port_publish(self->pub_evidence_reqs_, &ev_msg);
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

    json arr = json::array();
    for (const auto& upd : sf.updates) {
      if (upd.message_type == STREAM_MSG_ENTITY_DELETE) continue;

      json obj;
      obj["object_id"] = TacticalObjectsCodec::encodeUUID(upd.entity_id).get<std::string>();

      if (upd.affiliation) {
        obj["identity"] = affiliationToStandardIdentity(*upd.affiliation);
      }
      if (upd.mil_class) {
        obj["dimension"] = battleDimToStandard(upd.mil_class->battle_dim);
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

    if (arr.empty()) continue;

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

  std::string str(static_cast<const char*>(msg->data), msg->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return;
  }

  Observation obs;
  obs.observation_id  = pyramid::core::uuid::UUIDHelper::generateV4();
  obs.received_at     = 0.0;
  obs.observed_at     = j.value("observed_at", 0.0);
  obs.object_hint_type = ObjectType::Platform;
  obs.affiliation_hint = Affiliation::Unknown;
  obs.confidence       = j.value("confidence", 0.0);

  // Parse standard position (radians → degrees)
  obs.position.lat = self->radToDeg(j.value("latitude_rad", 0.0));
  obs.position.lon = self->radToDeg(j.value("longitude_rad", 0.0));
  obs.position.alt = 0.0;

  // Parse standard identity
  std::string identity = j.value("identity", "");
  if (!identity.empty()) {
    obs.affiliation_hint = self->standardIdentityToAffiliation(identity);
  }

  // Parse standard dimension → SIDC hint
  std::string dimension = j.value("dimension", "");
  if (dimension == "BATTLE_DIMENSION_SEA_SURFACE") {
    obs.source_sidc = "SHSP------*****";
  } else if (dimension == "BATTLE_DIMENSION_AIR") {
    obs.source_sidc = "SHAP------*****";
  } else if (dimension == "BATTLE_DIMENSION_SUBSURFACE") {
    obs.source_sidc = "SHUP------*****";
  } else if (dimension == "BATTLE_DIMENSION_GROUND") {
    obs.source_sidc = "SHGP------*****";
  }

  ObservationBatch batch;
  batch.observations.push_back(obs);
  self->runtime_.processObservationBatch(batch);
}

// ---------------------------------------------------------------------------
// Enum converters
// ---------------------------------------------------------------------------

std::string StandardBridge::affiliationToStandardIdentity(Affiliation a) {
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

Affiliation StandardBridge::standardIdentityToAffiliation(const std::string& s) {
  if (s == "STANDARD_IDENTITY_FRIENDLY")         return Affiliation::Friendly;
  if (s == "STANDARD_IDENTITY_HOSTILE")           return Affiliation::Hostile;
  if (s == "STANDARD_IDENTITY_NEUTRAL")           return Affiliation::Neutral;
  if (s == "STANDARD_IDENTITY_UNKNOWN")           return Affiliation::Unknown;
  if (s == "STANDARD_IDENTITY_ASSUMED_FRIENDLY")  return Affiliation::AssumedFriend;
  if (s == "STANDARD_IDENTITY_SUSPECT")           return Affiliation::Suspect;
  if (s == "STANDARD_IDENTITY_JOKER")             return Affiliation::Joker;
  if (s == "STANDARD_IDENTITY_FAKER")             return Affiliation::Faker;
  if (s == "STANDARD_IDENTITY_PENDING")           return Affiliation::Pending;
  return Affiliation::Unknown;
}

std::string StandardBridge::battleDimToStandard(BattleDimension d) {
  switch (d) {
    case BattleDimension::Ground:    return "BATTLE_DIMENSION_GROUND";
    case BattleDimension::Air:       return "BATTLE_DIMENSION_AIR";
    case BattleDimension::SeaSurface:return "BATTLE_DIMENSION_SEA_SURFACE";
    case BattleDimension::Subsurface:return "BATTLE_DIMENSION_SUBSURFACE";
    case BattleDimension::Space:     return "BATTLE_DIMENSION_SPACE";
    case BattleDimension::SOF:       return "BATTLE_DIMENSION_SOF";
  }
  return "BATTLE_DIMENSION_GROUND";
}

BattleDimension StandardBridge::standardToBattleDim(const std::string& s) {
  if (s == "BATTLE_DIMENSION_GROUND")      return BattleDimension::Ground;
  if (s == "BATTLE_DIMENSION_AIR")         return BattleDimension::Air;
  if (s == "BATTLE_DIMENSION_SEA_SURFACE") return BattleDimension::SeaSurface;
  if (s == "BATTLE_DIMENSION_SUBSURFACE")  return BattleDimension::Subsurface;
  if (s == "BATTLE_DIMENSION_SPACE")       return BattleDimension::Space;
  if (s == "BATTLE_DIMENSION_SOF")         return BattleDimension::SOF;
  return BattleDimension::Ground;
}

} // namespace tactical_objects
