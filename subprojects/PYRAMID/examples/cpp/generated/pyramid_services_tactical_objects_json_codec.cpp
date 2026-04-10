// Auto-generated JSON codec implementation
// Namespace: pyramid::services::tactical_objects::json_codec

#include "pyramid_services_tactical_objects_json_codec.hpp"

#include <nlohmann/json.hpp>

namespace pyramid::services::tactical_objects::json_codec {

namespace wire_types = pyramid::services::tactical_objects::wire_types;
namespace data_model = pyramid::data_model;

// ---------------------------------------------------------------------------
// Enum string converters
// ---------------------------------------------------------------------------

std::string toString(data_model::StandardIdentity v) {
    switch (v) {
        case data_model::StandardIdentity::Unspecified: return "STANDARD_IDENTITY_UNSPECIFIED";
        case data_model::StandardIdentity::Unknown: return "STANDARD_IDENTITY_UNKNOWN";
        case data_model::StandardIdentity::Friendly: return "STANDARD_IDENTITY_FRIENDLY";
        case data_model::StandardIdentity::Hostile: return "STANDARD_IDENTITY_HOSTILE";
        case data_model::StandardIdentity::Suspect: return "STANDARD_IDENTITY_SUSPECT";
        case data_model::StandardIdentity::Neutral: return "STANDARD_IDENTITY_NEUTRAL";
        case data_model::StandardIdentity::Pending: return "STANDARD_IDENTITY_PENDING";
        case data_model::StandardIdentity::Joker: return "STANDARD_IDENTITY_JOKER";
        case data_model::StandardIdentity::Faker: return "STANDARD_IDENTITY_FAKER";
        case data_model::StandardIdentity::AssumedFriendly: return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
    }
    return "STANDARD_IDENTITY_UNSPECIFIED";
}

data_model::StandardIdentity standardIdentityFromString(const std::string& s) {
    if (s == "STANDARD_IDENTITY_UNSPECIFIED") return data_model::StandardIdentity::Unspecified;
    if (s == "STANDARD_IDENTITY_UNKNOWN") return data_model::StandardIdentity::Unknown;
    if (s == "STANDARD_IDENTITY_FRIENDLY") return data_model::StandardIdentity::Friendly;
    if (s == "STANDARD_IDENTITY_HOSTILE") return data_model::StandardIdentity::Hostile;
    if (s == "STANDARD_IDENTITY_SUSPECT") return data_model::StandardIdentity::Suspect;
    if (s == "STANDARD_IDENTITY_NEUTRAL") return data_model::StandardIdentity::Neutral;
    if (s == "STANDARD_IDENTITY_PENDING") return data_model::StandardIdentity::Pending;
    if (s == "STANDARD_IDENTITY_JOKER") return data_model::StandardIdentity::Joker;
    if (s == "STANDARD_IDENTITY_FAKER") return data_model::StandardIdentity::Faker;
    if (s == "STANDARD_IDENTITY_ASSUMED_FRIENDLY") return data_model::StandardIdentity::AssumedFriendly;
    return data_model::StandardIdentity::Unspecified;
}

std::string toString(data_model::BattleDimension v) {
    switch (v) {
        case data_model::BattleDimension::Unspecified: return "BATTLE_DIMENSION_UNSPECIFIED";
        case data_model::BattleDimension::Ground: return "BATTLE_DIMENSION_GROUND";
        case data_model::BattleDimension::Subsurface: return "BATTLE_DIMENSION_SUBSURFACE";
        case data_model::BattleDimension::SeaSurface: return "BATTLE_DIMENSION_SEA_SURFACE";
        case data_model::BattleDimension::Air: return "BATTLE_DIMENSION_AIR";
        case data_model::BattleDimension::Unknown: return "BATTLE_DIMENSION_UNKNOWN";
    }
    return "BATTLE_DIMENSION_UNSPECIFIED";
}

data_model::BattleDimension battleDimensionFromString(const std::string& s) {
    if (s == "BATTLE_DIMENSION_UNSPECIFIED") return data_model::BattleDimension::Unspecified;
    if (s == "BATTLE_DIMENSION_GROUND") return data_model::BattleDimension::Ground;
    if (s == "BATTLE_DIMENSION_SUBSURFACE") return data_model::BattleDimension::Subsurface;
    if (s == "BATTLE_DIMENSION_SEA_SURFACE") return data_model::BattleDimension::SeaSurface;
    if (s == "BATTLE_DIMENSION_AIR") return data_model::BattleDimension::Air;
    if (s == "BATTLE_DIMENSION_UNKNOWN") return data_model::BattleDimension::Unknown;
    return data_model::BattleDimension::Unspecified;
}

std::string toString(data_model::DataPolicy v) {
    switch (v) {
        case data_model::DataPolicy::Unspecified: return "DATA_POLICY_UNSPECIFIED";
        case data_model::DataPolicy::Query: return "DATA_POLICY_QUERY";
        case data_model::DataPolicy::Obtain: return "DATA_POLICY_OBTAIN";
    }
    return "DATA_POLICY_UNSPECIFIED";
}

data_model::DataPolicy dataPolicyFromString(const std::string& s) {
    if (s == "DATA_POLICY_UNSPECIFIED") return data_model::DataPolicy::Unspecified;
    if (s == "DATA_POLICY_QUERY") return data_model::DataPolicy::Query;
    if (s == "DATA_POLICY_OBTAIN") return data_model::DataPolicy::Obtain;
    return data_model::DataPolicy::Unspecified;
}

// ---------------------------------------------------------------------------
// Serialisation (toJson)
// ---------------------------------------------------------------------------

std::string toJson(const wire_types::EntityMatch& msg) {
    nlohmann::json obj;
    obj["object_id"] = msg.object_id;
    obj["identity"] = toString(msg.identity);
    if (msg.dimension != data_model::BattleDimension::Unspecified) obj["dimension"] = toString(msg.dimension);
    if (msg.latitude_rad != 0.0) obj["latitude_rad"] = msg.latitude_rad;
    if (msg.longitude_rad != 0.0) obj["longitude_rad"] = msg.longitude_rad;
    if (msg.confidence != 0.0) obj["confidence"] = msg.confidence;
    return obj.dump();
}

std::string toJson(const wire_types::ObjectEvidence& msg) {
    nlohmann::json obj;
    obj["identity"] = toString(msg.identity);
    obj["dimension"] = toString(msg.dimension);
    obj["latitude_rad"] = msg.latitude_rad;
    obj["longitude_rad"] = msg.longitude_rad;
    obj["confidence"] = msg.confidence;
    if (msg.observed_at != 0.0) obj["observed_at"] = msg.observed_at;
    return obj.dump();
}

std::string toJson(const wire_types::EvidenceRequirement& msg) {
    nlohmann::json obj;
    if (!msg.id.empty()) obj["id"] = msg.id;
    if (msg.policy != data_model::DataPolicy::Unspecified) obj["policy"] = toString(msg.policy);
    if (msg.dimension != data_model::BattleDimension::Unspecified) obj["dimension"] = toString(msg.dimension);
    if (msg.min_lat_rad != 0.0) obj["min_lat_rad"] = msg.min_lat_rad;
    if (msg.max_lat_rad != 0.0) obj["max_lat_rad"] = msg.max_lat_rad;
    if (msg.min_lon_rad != 0.0) obj["min_lon_rad"] = msg.min_lon_rad;
    if (msg.max_lon_rad != 0.0) obj["max_lon_rad"] = msg.max_lon_rad;
    return obj.dump();
}

// ---------------------------------------------------------------------------
// Deserialisation (fromJson)
// ---------------------------------------------------------------------------

wire_types::EntityMatch entityMatchFromJson(const std::string& s) {
    wire_types::EntityMatch result;
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

wire_types::ObjectEvidence objectEvidenceFromJson(const std::string& s) {
    wire_types::ObjectEvidence result;
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

wire_types::EvidenceRequirement evidenceRequirementFromJson(const std::string& s) {
    wire_types::EvidenceRequirement result;
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

wire_types::EntityMatchArray entityMatchesFromJson(const std::string& s) {
    wire_types::EntityMatchArray result;
    try {
        auto arr = nlohmann::json::parse(s);
        if (!arr.is_array()) return result;
        result.reserve(arr.size());
        for (const auto& elem : arr) {
            wire_types::EntityMatch m;
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
