// Auto-generated data model JSON codec implementation
// Namespace: pyramid::data_model::common

#include "pyramid_data_model_common_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_base_codec.hpp"

namespace pyramid::data_model::common {

std::string toString(Feasibility v) {
    switch (v) {
        case Feasibility::Unspecified: return "FEASIBILITY_UNSPECIFIED";
        case Feasibility::Feasible: return "FEASIBILITY_FEASIBLE";
        case Feasibility::NotFeasible: return "FEASIBILITY_NOT_FEASIBLE";
        case Feasibility::PartiallyFeasible: return "FEASIBILITY_PARTIALLY_FEASIBLE";
        case Feasibility::Pending: return "FEASIBILITY_PENDING";
    }
    return "FEASIBILITY_UNSPECIFIED";
}

Feasibility feasibilityFromString(const std::string& s) {
    if (s == "FEASIBILITY_UNSPECIFIED") return Feasibility::Unspecified;
    if (s == "FEASIBILITY_FEASIBLE") return Feasibility::Feasible;
    if (s == "FEASIBILITY_NOT_FEASIBLE") return Feasibility::NotFeasible;
    if (s == "FEASIBILITY_PARTIALLY_FEASIBLE") return Feasibility::PartiallyFeasible;
    if (s == "FEASIBILITY_PENDING") return Feasibility::Pending;
    return Feasibility::Unspecified;
}

std::string toString(Progress v) {
    switch (v) {
        case Progress::Unspecified: return "PROGRESS_UNSPECIFIED";
        case Progress::NotStarted: return "PROGRESS_NOT_STARTED";
        case Progress::InProgress: return "PROGRESS_IN_PROGRESS";
        case Progress::Completed: return "PROGRESS_COMPLETED";
        case Progress::Cancelled: return "PROGRESS_CANCELLED";
        case Progress::Failed: return "PROGRESS_FAILED";
    }
    return "PROGRESS_UNSPECIFIED";
}

Progress progressFromString(const std::string& s) {
    if (s == "PROGRESS_UNSPECIFIED") return Progress::Unspecified;
    if (s == "PROGRESS_NOT_STARTED") return Progress::NotStarted;
    if (s == "PROGRESS_IN_PROGRESS") return Progress::InProgress;
    if (s == "PROGRESS_COMPLETED") return Progress::Completed;
    if (s == "PROGRESS_CANCELLED") return Progress::Cancelled;
    if (s == "PROGRESS_FAILED") return Progress::Failed;
    return Progress::Unspecified;
}

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

std::string toJson(const GeodeticPosition& msg) {
    nlohmann::json obj;
    obj["latitude"] = msg.latitude;
    obj["longitude"] = msg.longitude;
    return obj.dump();
}

GeodeticPosition fromJson(const std::string& s, GeodeticPosition* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    GeodeticPosition msg;
    if (j.contains("latitude")) msg.latitude = j["latitude"].get<double>();
    if (j.contains("longitude")) msg.longitude = j["longitude"].get<double>();
    return msg;
}

std::string toJson(const PolyArea& msg) {
    nlohmann::json obj;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.points) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["points"] = arr;
    }
    return obj.dump();
}

PolyArea fromJson(const std::string& s, PolyArea* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    PolyArea msg;
    if (j.contains("points")) {
        for (const auto& v : j["points"]) {
            msg.points.push_back(fromJson(v.dump(), static_cast<GeodeticPosition*>(nullptr)));
        }
    }
    return msg;
}

std::string toJson(const Achievement& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["status"] = toString(msg.status);
    if (msg.quality.has_value()) {
        obj["quality"] = msg.quality.value();
    }
    obj["achieveability"] = toString(msg.achieveability);
    return obj.dump();
}

Achievement fromJson(const std::string& s, Achievement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Achievement msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("status")) msg.status = progressFromString(j["status"].get<std::string>());
    if (j.contains("quality")) {
        msg.quality = j["quality"].get<double>();
    }
    if (j.contains("achieveability")) msg.achieveability = feasibilityFromString(j["achieveability"].get<std::string>());
    return msg;
}

std::string toJson(const Requirement& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    return obj.dump();
}

Requirement fromJson(const std::string& s, Requirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Requirement msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<Achievement*>(nullptr));
    return msg;
}

std::string toJson(const Capability& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["availability"] = msg.availability;
    obj["name"] = msg.name;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.contraint) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["contraint"] = arr;
    }
    return obj.dump();
}

Capability fromJson(const std::string& s, Capability* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Capability msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("availability")) msg.availability = j["availability"].get<bool>();
    if (j.contains("name")) msg.name = j["name"].get<std::string>();
    if (j.contains("contraint")) {
        for (const auto& v : j["contraint"]) {
            msg.contraint.push_back(fromJson(v.dump(), static_cast<Contraint*>(nullptr)));
        }
    }
    return msg;
}

std::string toJson(const Entity& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    return obj.dump();
}

Entity fromJson(const std::string& s, Entity* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Entity msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    return msg;
}

std::string toJson(const CircleArea& msg) {
    nlohmann::json obj;
    obj["position"] = nlohmann::json::parse(toJson(msg.position));
    obj["radius"] = msg.radius;
    return obj.dump();
}

CircleArea fromJson(const std::string& s, CircleArea* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    CircleArea msg;
    if (j.contains("position")) msg.position = fromJson(j["position"].dump(), static_cast<GeodeticPosition*>(nullptr));
    if (j.contains("radius")) msg.radius = j["radius"].get<double>();
    return msg;
}

std::string toJson(const Point& msg) {
    nlohmann::json obj;
    obj["position"] = nlohmann::json::parse(toJson(msg.position));
    return obj.dump();
}

Point fromJson(const std::string& s, Point* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Point msg;
    if (j.contains("position")) msg.position = fromJson(j["position"].dump(), static_cast<GeodeticPosition*>(nullptr));
    return msg;
}

std::string toJson(const Contraint& msg) {
    nlohmann::json obj;
    obj["name"] = msg.name;
    obj["value"] = msg.value;
    return obj.dump();
}

Contraint fromJson(const std::string& s, Contraint* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Contraint msg;
    if (j.contains("name")) msg.name = j["name"].get<std::string>();
    if (j.contains("value")) msg.value = j["value"].get<int32_t>();
    return msg;
}

std::string toJson(const Ack& msg) {
    nlohmann::json obj;
    obj["success"] = msg.success;
    return obj.dump();
}

Ack fromJson(const std::string& s, Ack* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Ack msg;
    if (j.contains("success")) msg.success = j["success"].get<bool>();
    return msg;
}

std::string toJson(const Query& msg) {
    nlohmann::json obj;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.id) {
            arr.push_back(v);
        }
        obj["id"] = arr;
    }
    if (msg.one_shot.has_value()) {
        obj["one_shot"] = msg.one_shot.value();
    }
    return obj.dump();
}

Query fromJson(const std::string& s, Query* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Query msg;
    if (j.contains("id")) {
        for (const auto& v : j["id"]) {
            msg.id.push_back(v.get<std::string>());
        }
    }
    if (j.contains("one_shot")) {
        msg.one_shot = j["one_shot"].get<bool>();
    }
    return msg;
}

} // namespace pyramid::data_model::common
