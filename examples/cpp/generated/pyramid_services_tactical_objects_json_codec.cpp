// Auto-generated JSON codec implementation
// Namespace: pyramid::services::tactical_objects::json_codec

#include "pyramid_services_tactical_objects_json_codec.hpp"

#include <nlohmann/json.hpp>

namespace pyramid::services::tactical_objects::json_codec {

// ---------------------------------------------------------------------------
// Enum string converters
// ---------------------------------------------------------------------------

std::string toString(StandardIdentity v) {
    switch (v) {
        case StandardIdentity::Unspecified: return "STANDARD_IDENTITY_UNSPECIFIED";
        case StandardIdentity::Unknown: return "STANDARD_IDENTITY_UNKNOWN";
        case StandardIdentity::Friendly: return "STANDARD_IDENTITY_FRIENDLY";
        case StandardIdentity::Hostile: return "STANDARD_IDENTITY_HOSTILE";
        case StandardIdentity::Suspect: return "STANDARD_IDENTITY_SUSPECT";
        case StandardIdentity::Neutral: return "STANDARD_IDENTITY_NEUTRAL";
        case StandardIdentity::Pending: return "STANDARD_IDENTITY_PENDING";
        case StandardIdentity::Joker: return "STANDARD_IDENTITY_JOKER";
        case StandardIdentity::Faker: return "STANDARD_IDENTITY_FAKER";
        case StandardIdentity::AssumedFriendly: return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
    }
    return "STANDARD_IDENTITY_UNSPECIFIED";
}

StandardIdentity standardIdentityFromString(const std::string& s) {
    if (s == "STANDARD_IDENTITY_UNSPECIFIED") return StandardIdentity::Unspecified;
    if (s == "STANDARD_IDENTITY_UNKNOWN") return StandardIdentity::Unknown;
    if (s == "STANDARD_IDENTITY_FRIENDLY") return StandardIdentity::Friendly;
    if (s == "STANDARD_IDENTITY_HOSTILE") return StandardIdentity::Hostile;
    if (s == "STANDARD_IDENTITY_SUSPECT") return StandardIdentity::Suspect;
    if (s == "STANDARD_IDENTITY_NEUTRAL") return StandardIdentity::Neutral;
    if (s == "STANDARD_IDENTITY_PENDING") return StandardIdentity::Pending;
    if (s == "STANDARD_IDENTITY_JOKER") return StandardIdentity::Joker;
    if (s == "STANDARD_IDENTITY_FAKER") return StandardIdentity::Faker;
    if (s == "STANDARD_IDENTITY_ASSUMED_FRIENDLY") return StandardIdentity::AssumedFriendly;
    return StandardIdentity::Unspecified;
}

std::string toString(BattleDimension v) {
    switch (v) {
        case BattleDimension::Unspecified: return "BATTLE_DIMENSION_UNSPECIFIED";
        case BattleDimension::Ground: return "BATTLE_DIMENSION_GROUND";
        case BattleDimension::Subsurface: return "BATTLE_DIMENSION_SUBSURFACE";
        case BattleDimension::SeaSurface: return "BATTLE_DIMENSION_SEA_SURFACE";
        case BattleDimension::Air: return "BATTLE_DIMENSION_AIR";
        case BattleDimension::Unknown: return "BATTLE_DIMENSION_UNKNOWN";
    }
    return "BATTLE_DIMENSION_UNSPECIFIED";
}

BattleDimension battleDimensionFromString(const std::string& s) {
    if (s == "BATTLE_DIMENSION_UNSPECIFIED") return BattleDimension::Unspecified;
    if (s == "BATTLE_DIMENSION_GROUND") return BattleDimension::Ground;
    if (s == "BATTLE_DIMENSION_SUBSURFACE") return BattleDimension::Subsurface;
    if (s == "BATTLE_DIMENSION_SEA_SURFACE") return BattleDimension::SeaSurface;
    if (s == "BATTLE_DIMENSION_AIR") return BattleDimension::Air;
    if (s == "BATTLE_DIMENSION_UNKNOWN") return BattleDimension::Unknown;
    return BattleDimension::Unspecified;
}

std::string toString(DataPolicy v) {
    switch (v) {
        case DataPolicy::Unspecified: return "DATA_POLICY_UNSPECIFIED";
        case DataPolicy::Query: return "DATA_POLICY_QUERY";
        case DataPolicy::Obtain: return "DATA_POLICY_OBTAIN";
    }
    return "DATA_POLICY_UNSPECIFIED";
}

DataPolicy dataPolicyFromString(const std::string& s) {
    if (s == "DATA_POLICY_UNSPECIFIED") return DataPolicy::Unspecified;
    if (s == "DATA_POLICY_QUERY") return DataPolicy::Query;
    if (s == "DATA_POLICY_OBTAIN") return DataPolicy::Obtain;
    return DataPolicy::Unspecified;
}

// ---------------------------------------------------------------------------
// Serialisation (toJson)
// ---------------------------------------------------------------------------

std::string toJson(const CreateRequirementRequest& msg) {
    nlohmann::json obj;
    obj["policy"] = toString(msg.policy);
    obj["identity"] = toString(msg.identity);
    if (msg.dimension != BattleDimension::Unspecified) obj["dimension"] = toString(msg.dimension);
    if (msg.min_lat_rad != 0.0) obj["min_lat_rad"] = msg.min_lat_rad;
    if (msg.max_lat_rad != 0.0) obj["max_lat_rad"] = msg.max_lat_rad;
    if (msg.min_lon_rad != 0.0) obj["min_lon_rad"] = msg.min_lon_rad;
    if (msg.max_lon_rad != 0.0) obj["max_lon_rad"] = msg.max_lon_rad;
    return obj.dump();
}

std::string toJson(const CreateRequirementResponse& msg) {
    nlohmann::json obj;
    if (!msg.interest_id.empty()) obj["interest_id"] = msg.interest_id;
    return obj.dump();
}

std::string toJson(const EntityMatch& msg) {
    nlohmann::json obj;
    obj["object_id"] = msg.object_id;
    obj["identity"] = toString(msg.identity);
    if (msg.dimension != BattleDimension::Unspecified) obj["dimension"] = toString(msg.dimension);
    if (msg.latitude_rad != 0.0) obj["latitude_rad"] = msg.latitude_rad;
    if (msg.longitude_rad != 0.0) obj["longitude_rad"] = msg.longitude_rad;
    if (msg.confidence != 0.0) obj["confidence"] = msg.confidence;
    return obj.dump();
}

std::string toJson(const ObjectEvidence& msg) {
    nlohmann::json obj;
    obj["identity"] = toString(msg.identity);
    obj["dimension"] = toString(msg.dimension);
    obj["latitude_rad"] = msg.latitude_rad;
    obj["longitude_rad"] = msg.longitude_rad;
    obj["confidence"] = msg.confidence;
    if (msg.observed_at != 0.0) obj["observed_at"] = msg.observed_at;
    return obj.dump();
}

std::string toJson(const EvidenceRequirement& msg) {
    nlohmann::json obj;
    if (!msg.id.empty()) obj["id"] = msg.id;
    if (msg.policy != DataPolicy::Unspecified) obj["policy"] = toString(msg.policy);
    if (msg.dimension != BattleDimension::Unspecified) obj["dimension"] = toString(msg.dimension);
    if (msg.min_lat_rad != 0.0) obj["min_lat_rad"] = msg.min_lat_rad;
    if (msg.max_lat_rad != 0.0) obj["max_lat_rad"] = msg.max_lat_rad;
    if (msg.min_lon_rad != 0.0) obj["min_lon_rad"] = msg.min_lon_rad;
    if (msg.max_lon_rad != 0.0) obj["max_lon_rad"] = msg.max_lon_rad;
    return obj.dump();
}

// ---------------------------------------------------------------------------
// Deserialisation (fromJson)
// ---------------------------------------------------------------------------

CreateRequirementRequest createRequirementRequestFromJson(const std::string& s) {
    CreateRequirementRequest result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("policy"))
            result.policy = dataPolicyFromString(j["policy"].get<std::string>());
        if (j.contains("identity"))
            result.identity = standardIdentityFromString(j["identity"].get<std::string>());
        if (j.contains("dimension"))
            result.dimension = battleDimensionFromString(j["dimension"].get<std::string>());
        if (j.contains("min_lat_rad"))
            result.min_lat_rad = j["min_lat_rad"].get<double>();
        if (j.contains("max_lat_rad"))
            result.max_lat_rad = j["max_lat_rad"].get<double>();
        if (j.contains("min_lon_rad"))
            result.min_lon_rad = j["min_lon_rad"].get<double>();
        if (j.contains("max_lon_rad"))
            result.max_lon_rad = j["max_lon_rad"].get<double>();
    } catch (...) {}
    return result;
}

CreateRequirementResponse createRequirementResponseFromJson(const std::string& s) {
    CreateRequirementResponse result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("interest_id"))
            result.interest_id = j["interest_id"].get<std::string>();
    } catch (...) {}
    return result;
}

EntityMatch entityMatchFromJson(const std::string& s) {
    EntityMatch result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("object_id"))
            result.object_id = j["object_id"].get<std::string>();
        if (j.contains("identity"))
            result.identity = standardIdentityFromString(j["identity"].get<std::string>());
        if (j.contains("dimension"))
            result.dimension = battleDimensionFromString(j["dimension"].get<std::string>());
        if (j.contains("latitude_rad"))
            result.latitude_rad = j["latitude_rad"].get<double>();
        if (j.contains("longitude_rad"))
            result.longitude_rad = j["longitude_rad"].get<double>();
        if (j.contains("confidence"))
            result.confidence = j["confidence"].get<double>();
    } catch (...) {}
    return result;
}

ObjectEvidence objectEvidenceFromJson(const std::string& s) {
    ObjectEvidence result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("identity"))
            result.identity = standardIdentityFromString(j["identity"].get<std::string>());
        if (j.contains("dimension"))
            result.dimension = battleDimensionFromString(j["dimension"].get<std::string>());
        if (j.contains("latitude_rad"))
            result.latitude_rad = j["latitude_rad"].get<double>();
        if (j.contains("longitude_rad"))
            result.longitude_rad = j["longitude_rad"].get<double>();
        if (j.contains("confidence"))
            result.confidence = j["confidence"].get<double>();
        if (j.contains("observed_at"))
            result.observed_at = j["observed_at"].get<double>();
    } catch (...) {}
    return result;
}

EvidenceRequirement evidenceRequirementFromJson(const std::string& s) {
    EvidenceRequirement result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("id"))
            result.id = j["id"].get<std::string>();
        if (j.contains("policy"))
            result.policy = dataPolicyFromString(j["policy"].get<std::string>());
        if (j.contains("dimension"))
            result.dimension = battleDimensionFromString(j["dimension"].get<std::string>());
        if (j.contains("min_lat_rad"))
            result.min_lat_rad = j["min_lat_rad"].get<double>();
        if (j.contains("max_lat_rad"))
            result.max_lat_rad = j["max_lat_rad"].get<double>();
        if (j.contains("min_lon_rad"))
            result.min_lon_rad = j["min_lon_rad"].get<double>();
        if (j.contains("max_lon_rad"))
            result.max_lon_rad = j["max_lon_rad"].get<double>();
    } catch (...) {}
    return result;
}

// ---------------------------------------------------------------------------
// Array deserialisation
// ---------------------------------------------------------------------------

EntityMatchArray entityMatchesFromJson(const std::string& s) {
    EntityMatchArray result;
    try {
        auto arr = nlohmann::json::parse(s);
        if (!arr.is_array()) return result;
        result.reserve(arr.size());
        for (const auto& elem : arr) {
            EntityMatch m;
            if (elem.contains("object_id"))
                m.object_id = elem["object_id"].get<std::string>();
            if (elem.contains("identity"))
                m.identity = standardIdentityFromString(elem["identity"].get<std::string>());
            if (elem.contains("dimension"))
                m.dimension = battleDimensionFromString(elem["dimension"].get<std::string>());
            if (elem.contains("latitude_rad"))
                m.latitude_rad = elem["latitude_rad"].get<double>();
            if (elem.contains("longitude_rad"))
                m.longitude_rad = elem["longitude_rad"].get<double>();
            if (elem.contains("confidence"))
                m.confidence = elem["confidence"].get<double>();
            result.push_back(std::move(m));
        }
    } catch (...) {}
    return result;
}

} // namespace pyramid::services::tactical_objects::json_codec
