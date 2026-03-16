#include <TacticalObjectsCodec.h>
#include <uuid/UUIDHelper.h>

namespace tactical_objects {

using json = nlohmann::json;
using pyramid::core::uuid::UUIDHelper;

// --- Enum string maps ---

ObjectType TacticalObjectsCodec::objectTypeFromString(const std::string& s) {
  return stringToObjectType(s);
}

Affiliation TacticalObjectsCodec::affiliationFromString(const std::string& s) {
  return stringToAffiliation(s);
}

BattleDimension TacticalObjectsCodec::battleDimensionFromString(const std::string& s) {
  return stringToBattleDim(s);
}

std::string TacticalObjectsCodec::objectTypeToString(ObjectType t) {
  switch (t) {
    case ObjectType::Platform: return "Platform";
    case ObjectType::Person: return "Person";
    case ObjectType::Equipment: return "Equipment";
    case ObjectType::Unit: return "Unit";
    case ObjectType::Formation: return "Formation";
    case ObjectType::Installation: return "Installation";
    case ObjectType::Feature: return "Feature";
    case ObjectType::Route: return "Route";
    case ObjectType::Point: return "Point";
    case ObjectType::Area: return "Area";
    case ObjectType::Zone: return "Zone";
    default: return "Platform";
  }
}

ObjectType TacticalObjectsCodec::stringToObjectType(const std::string& s) {
  if (s == "Person") return ObjectType::Person;
  if (s == "Equipment") return ObjectType::Equipment;
  if (s == "Unit") return ObjectType::Unit;
  if (s == "Formation") return ObjectType::Formation;
  if (s == "Installation") return ObjectType::Installation;
  if (s == "Feature") return ObjectType::Feature;
  if (s == "Route") return ObjectType::Route;
  if (s == "Point") return ObjectType::Point;
  if (s == "Area") return ObjectType::Area;
  if (s == "Zone") return ObjectType::Zone;
  return ObjectType::Platform;
}

std::string TacticalObjectsCodec::affiliationToString(Affiliation a) {
  switch (a) {
    case Affiliation::Friendly: return "Friendly";
    case Affiliation::Hostile: return "Hostile";
    case Affiliation::Neutral: return "Neutral";
    case Affiliation::Unknown: return "Unknown";
    case Affiliation::AssumedFriend: return "AssumedFriend";
    case Affiliation::Suspect: return "Suspect";
    case Affiliation::Joker: return "Joker";
    case Affiliation::Faker: return "Faker";
    case Affiliation::Pending: return "Pending";
    default: return "Unknown";
  }
}

Affiliation TacticalObjectsCodec::stringToAffiliation(const std::string& s) {
  if (s == "Friendly") return Affiliation::Friendly;
  if (s == "Hostile") return Affiliation::Hostile;
  if (s == "Neutral") return Affiliation::Neutral;
  if (s == "AssumedFriend") return Affiliation::AssumedFriend;
  if (s == "Suspect") return Affiliation::Suspect;
  if (s == "Joker") return Affiliation::Joker;
  if (s == "Faker") return Affiliation::Faker;
  if (s == "Pending") return Affiliation::Pending;
  return Affiliation::Unknown;
}

std::string TacticalObjectsCodec::zoneTypeToString(ZoneType z) {
  switch (z) {
    case ZoneType::AOI: return "AOI";
    case ZoneType::PatrolArea: return "PatrolArea";
    case ZoneType::RestrictedArea: return "RestrictedArea";
    case ZoneType::NoGoArea: return "NoGoArea";
    case ZoneType::KillBox: return "KillBox";
    case ZoneType::PhaseLine: return "PhaseLine";
    case ZoneType::Boundary: return "Boundary";
    case ZoneType::RouteCorridor: return "RouteCorridor";
    case ZoneType::SensorCoverageArea: return "SensorCoverageArea";
    default: return "AOI";
  }
}

ZoneType TacticalObjectsCodec::stringToZoneType(const std::string& s) {
  if (s == "PatrolArea") return ZoneType::PatrolArea;
  if (s == "RestrictedArea") return ZoneType::RestrictedArea;
  if (s == "NoGoArea") return ZoneType::NoGoArea;
  if (s == "KillBox") return ZoneType::KillBox;
  if (s == "PhaseLine") return ZoneType::PhaseLine;
  if (s == "Boundary") return ZoneType::Boundary;
  if (s == "RouteCorridor") return ZoneType::RouteCorridor;
  if (s == "SensorCoverageArea") return ZoneType::SensorCoverageArea;
  return ZoneType::AOI;
}

std::string TacticalObjectsCodec::battleDimToString(BattleDimension b) {
  switch (b) {
    case BattleDimension::Ground: return "Ground";
    case BattleDimension::Air: return "Air";
    case BattleDimension::SeaSurface: return "SeaSurface";
    case BattleDimension::Subsurface: return "Subsurface";
    case BattleDimension::Space: return "Space";
    case BattleDimension::SOF: return "SOF";
    default: return "Ground";
  }
}

BattleDimension TacticalObjectsCodec::stringToBattleDim(const std::string& s) {
  if (s == "Air") return BattleDimension::Air;
  if (s == "SeaSurface") return BattleDimension::SeaSurface;
  if (s == "Subsurface") return BattleDimension::Subsurface;
  if (s == "Space") return BattleDimension::Space;
  if (s == "SOF") return BattleDimension::SOF;
  return BattleDimension::Ground;
}

// --- Helpers ---

json TacticalObjectsCodec::encodeSourceRef(const SourceRef& sr) {
  return json{
    {"source_system", sr.source_system},
    {"source_entity_id", sr.source_entity_id},
    {"source_track_id", sr.source_track_id},
    {"first_seen", sr.first_seen},
    {"last_seen", sr.last_seen}
  };
}

SourceRef TacticalObjectsCodec::decodeSourceRef(const json& j) {
  SourceRef sr;
  sr.source_system = j.value("source_system", "");
  sr.source_entity_id = j.value("source_entity_id", "");
  sr.source_track_id = j.value("source_track_id", "");
  sr.first_seen = j.value("first_seen", 0.0);
  sr.last_seen = j.value("last_seen", 0.0);
  return sr;
}

json TacticalObjectsCodec::encodeMilClassProfile(const MilClassProfile& p) {
  return json{
    {"battle_dim", battleDimToString(p.battle_dim)},
    {"affiliation", affiliationToString(p.affiliation)},
    {"role", p.role},
    {"source_sidc", p.source_sidc},
    {"hq", p.hq},
    {"task_force", p.task_force},
    {"feint_dummy", p.feint_dummy},
    {"installation", p.installation}
  };
}

MilClassProfile TacticalObjectsCodec::decodeMilClassProfile(const json& j) {
  MilClassProfile p;
  p.battle_dim = stringToBattleDim(j.value("battle_dim", "Ground"));
  p.affiliation = stringToAffiliation(j.value("affiliation", "Unknown"));
  p.role = j.value("role", "");
  p.source_sidc = j.value("source_sidc", "");
  p.hq = j.value("hq", false);
  p.task_force = j.value("task_force", false);
  p.feint_dummy = j.value("feint_dummy", false);
  p.installation = j.value("installation", false);
  return p;
}

// --- UUID ---

json TacticalObjectsCodec::encodeUUID(const UUIDKey& key) {
  return json(UUIDHelper::toString(key.uuid));
}

UUIDKey TacticalObjectsCodec::decodeUUID(const json& j) {
  auto result = UUIDHelper::fromString(j.get<std::string>());
  if (result.second) return UUIDKey{result.first};
  return UUIDKey{};
}

// --- Position ---

json TacticalObjectsCodec::encodePosition(const Position& pos) {
  return json{{"lat", pos.lat}, {"lon", pos.lon}, {"alt", pos.alt}};
}

Position TacticalObjectsCodec::decodePosition(const json& j) {
  Position p;
  p.lat = j.value("lat", 0.0);
  p.lon = j.value("lon", 0.0);
  p.alt = j.value("alt", 0.0);
  return p;
}

// --- Observation ---

json TacticalObjectsCodec::encodeObservation(const Observation& obs) {
  json j;
  j["observation_id"] = encodeUUID(UUIDKey{obs.observation_id});
  j["received_at"] = obs.received_at;
  j["observed_at"] = obs.observed_at;
  j["source_ref"] = encodeSourceRef(obs.source_ref);
  j["object_hint_type"] = objectTypeToString(obs.object_hint_type);
  j["position"] = encodePosition(obs.position);
  j["velocity"] = json{{"north", obs.velocity.north},
                        {"east", obs.velocity.east},
                        {"down", obs.velocity.down}};
  j["affiliation_hint"] = affiliationToString(obs.affiliation_hint);
  j["confidence"] = obs.confidence;
  j["uncertainty_radius_m"] = obs.uncertainty_radius_m;
  j["source_sidc"] = obs.source_sidc;
  return j;
}

Observation TacticalObjectsCodec::decodeObservation(const json& j) {
  Observation obs;
  obs.observation_id = decodeUUID(j.at("observation_id")).uuid;
  obs.received_at = j.value("received_at", 0.0);
  obs.observed_at = j.value("observed_at", 0.0);
  if (j.count("source_ref")) obs.source_ref = decodeSourceRef(j.at("source_ref"));
  obs.object_hint_type = stringToObjectType(j.value("object_hint_type", "Platform"));
  if (j.count("position")) obs.position = decodePosition(j.at("position"));
  if (j.count("velocity")) {
    auto& v = j.at("velocity");
    obs.velocity.north = v.value("north", 0.0);
    obs.velocity.east = v.value("east", 0.0);
    obs.velocity.down = v.value("down", 0.0);
  }
  obs.affiliation_hint = stringToAffiliation(j.value("affiliation_hint", "Unknown"));
  obs.confidence = j.value("confidence", 0.0);
  obs.uncertainty_radius_m = j.value("uncertainty_radius_m", 0.0);
  obs.source_sidc = j.value("source_sidc", "");
  return obs;
}

// --- ObservationBatch ---

json TacticalObjectsCodec::encodeObservationBatch(const ObservationBatch& batch) {
  json j = json::array();
  for (auto& obs : batch.observations) {
    j.push_back(encodeObservation(obs));
  }
  return json{{"observations", j}};
}

ObservationBatch TacticalObjectsCodec::decodeObservationBatch(const json& j) {
  ObservationBatch batch;
  for (auto& item : j.at("observations")) {
    batch.observations.push_back(decodeObservation(item));
  }
  return batch;
}

// --- ObjectDefinition ---

json TacticalObjectsCodec::encodeObjectDefinition(const ObjectDefinition& def) {
  json j;
  j["type"] = objectTypeToString(def.type);
  j["position"] = encodePosition(def.position);
  j["velocity"] = json{{"north", def.velocity.north},
                        {"east", def.velocity.east},
                        {"down", def.velocity.down}};
  j["affiliation"] = affiliationToString(def.affiliation);
  j["mil_class"] = encodeMilClassProfile(def.mil_class);
  j["identity_name"] = def.identity_name;
  json srefs = json::array();
  for (auto& sr : def.source_refs) {
    srefs.push_back(encodeSourceRef(sr));
  }
  j["source_refs"] = srefs;
  return j;
}

ObjectDefinition TacticalObjectsCodec::decodeObjectDefinition(const json& j) {
  ObjectDefinition def;
  def.type = stringToObjectType(j.value("type", "Platform"));
  if (j.count("position")) def.position = decodePosition(j.at("position"));
  if (j.count("velocity")) {
    auto& v = j.at("velocity");
    def.velocity.north = v.value("north", 0.0);
    def.velocity.east = v.value("east", 0.0);
    def.velocity.down = v.value("down", 0.0);
  }
  def.affiliation = stringToAffiliation(j.value("affiliation", "Unknown"));
  if (j.count("mil_class")) def.mil_class = decodeMilClassProfile(j.at("mil_class"));
  def.identity_name = j.value("identity_name", "");
  if (j.count("source_refs")) {
    for (auto& item : j.at("source_refs")) {
      def.source_refs.push_back(decodeSourceRef(item));
    }
  }
  return def;
}

json TacticalObjectsCodec::encodeObjectUpdate(const ObjectUpdate& upd) {
  json j;
  if (upd.position.has_value()) j["position"] = encodePosition(*upd.position);
  if (upd.velocity.has_value()) {
    j["velocity"] = json{{"north", upd.velocity->north},
                          {"east", upd.velocity->east},
                          {"down", upd.velocity->down}};
  }
  if (upd.affiliation.has_value()) j["affiliation"] = affiliationToString(*upd.affiliation);
  if (upd.mil_class.has_value()) j["mil_class"] = encodeMilClassProfile(*upd.mil_class);
  if (upd.identity_name.has_value()) j["identity_name"] = *upd.identity_name;
  if (upd.operational_state.has_value()) j["operational_state"] = *upd.operational_state;
  if (upd.behavior_pattern.has_value()) j["behavior_pattern"] = *upd.behavior_pattern;
  return j;
}

ObjectUpdate TacticalObjectsCodec::decodeObjectUpdate(const json& j) {
  ObjectUpdate upd;
  if (j.count("position")) upd.position = decodePosition(j.at("position"));
  if (j.count("velocity")) {
    Velocity v;
    auto& vj = j.at("velocity");
    v.north = vj.value("north", 0.0);
    v.east = vj.value("east", 0.0);
    v.down = vj.value("down", 0.0);
    upd.velocity = v;
  }
  if (j.count("affiliation")) upd.affiliation = stringToAffiliation(j.at("affiliation").get<std::string>());
  if (j.count("mil_class")) upd.mil_class = decodeMilClassProfile(j.at("mil_class"));
  if (j.count("identity_name")) upd.identity_name = j.at("identity_name").get<std::string>();
  if (j.count("operational_state")) upd.operational_state = j.at("operational_state").get<std::string>();
  if (j.count("behavior_pattern")) upd.behavior_pattern = j.at("behavior_pattern").get<std::string>();
  return upd;
}

// --- ZoneDefinition ---

json TacticalObjectsCodec::encodeZoneDefinition(const ZoneDefinition& def) {
  json j;
  j["zone_type"] = zoneTypeToString(def.zone_type);
  j["active_from"] = def.active_from;
  j["active_until"] = def.active_until;
  j["priority"] = def.priority;
  j["owner"] = def.owner;
  j["semantics"] = def.semantics;
  j["center"] = encodePosition(def.geometry.center);
  j["radius_m"] = def.geometry.radius_m;
  json verts = json::array();
  for (auto& v : def.geometry.vertices) {
    verts.push_back(encodePosition(v));
  }
  j["vertices"] = verts;
  return j;
}

ZoneDefinition TacticalObjectsCodec::decodeZoneDefinition(const json& j) {
  ZoneDefinition def;
  def.zone_type = stringToZoneType(j.value("zone_type", "AOI"));
  def.active_from = j.value("active_from", 0.0);
  def.active_until = j.value("active_until", 0.0);
  def.priority = j.value("priority", 0);
  def.owner = j.value("owner", "");
  def.semantics = j.value("semantics", "");
  if (j.count("center")) def.geometry.center = decodePosition(j.at("center"));
  def.geometry.radius_m = j.value("radius_m", 0.0);
  if (j.count("vertices")) {
    for (auto& item : j.at("vertices")) {
      def.geometry.vertices.push_back(decodePosition(item));
    }
  }
  def.geometry.geometry_type = def.zone_type;
  return def;
}

// --- QueryRequest / QueryResponse ---

json TacticalObjectsCodec::encodeQueryRequest(const QueryRequest& req) {
  json j;
  if (req.by_uuid.has_value()) j["by_uuid"] = encodeUUID(*req.by_uuid);
  if (req.by_source_system.has_value()) j["by_source_system"] = *req.by_source_system;
  if (req.by_source_entity_id.has_value()) j["by_source_entity_id"] = *req.by_source_entity_id;
  if (req.by_type.has_value()) j["by_type"] = objectTypeToString(*req.by_type);
  if (req.by_affiliation.has_value()) j["by_affiliation"] = affiliationToString(*req.by_affiliation);
  if (req.max_age_seconds.has_value()) j["max_age_seconds"] = *req.max_age_seconds;
  j["current_time"] = req.current_time;
  if (req.by_region.has_value()) {
    j["region"] = json{
      {"min_lat", req.by_region->min_lat}, {"max_lat", req.by_region->max_lat},
      {"min_lon", req.by_region->min_lon}, {"max_lon", req.by_region->max_lon}
    };
  }
  return j;
}

QueryRequest TacticalObjectsCodec::decodeQueryRequest(const json& j) {
  QueryRequest req;
  if (j.count("by_uuid")) req.by_uuid = decodeUUID(j.at("by_uuid"));
  if (j.count("by_source_system")) req.by_source_system = j.at("by_source_system").get<std::string>();
  if (j.count("by_source_entity_id")) req.by_source_entity_id = j.at("by_source_entity_id").get<std::string>();
  if (j.count("by_type")) req.by_type = stringToObjectType(j.at("by_type").get<std::string>());
  if (j.count("by_affiliation")) req.by_affiliation = stringToAffiliation(j.at("by_affiliation").get<std::string>());
  if (j.count("max_age_seconds")) req.max_age_seconds = j.at("max_age_seconds").get<double>();
  req.current_time = j.value("current_time", 0.0);
  if (j.count("region")) {
    auto& r = j.at("region");
    BoundingBox bb;
    bb.min_lat = r.value("min_lat", 0.0);
    bb.max_lat = r.value("max_lat", 0.0);
    bb.min_lon = r.value("min_lon", 0.0);
    bb.max_lon = r.value("max_lon", 0.0);
    req.by_region = bb;
  }
  return req;
}

json TacticalObjectsCodec::encodeQueryResponse(const QueryResponse& resp) {
  json j;
  json entries = json::array();
  for (auto& e : resp.entries) {
    json entry;
    entry["id"] = encodeUUID(e.id);
    entry["type"] = objectTypeToString(e.record.type);
    entry["version"] = e.record.version;
    entries.push_back(entry);
  }
  j["entries"] = entries;
  j["total"] = resp.total;
  return j;
}

QueryResponse TacticalObjectsCodec::decodeQueryResponse(const json& j) {
  QueryResponse resp;
  resp.total = j.value("total", static_cast<size_t>(0));
  for (auto& item : j.at("entries")) {
    QueryResultEntry e;
    e.id = decodeUUID(item.at("id"));
    e.record.type = stringToObjectType(item.value("type", "Platform"));
    e.record.version = item.value("version", static_cast<uint64_t>(0));
    resp.entries.push_back(e);
  }
  return resp;
}

} // namespace tactical_objects
