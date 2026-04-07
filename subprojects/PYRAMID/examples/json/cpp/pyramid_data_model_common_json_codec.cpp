// Auto-generated JSON codec — do not edit
#include "pyramid_data_model_common_json_codec.hpp"

#include <nlohmann/json.hpp>

namespace pyramid::data_model::common::json_codec {

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
    obj["latitude"] = nlohmann::json::parse(toJson(msg.latitude));
    obj["longitude"] = nlohmann::json::parse(toJson(msg.longitude));
    return obj.dump();
}

std::string toJson(const PolyArea& msg) {
    nlohmann::json obj;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& e : msg.points) arr.push_back(nlohmann::json::parse(toJson(e)));
        obj["points"] = arr;
    }
    return obj.dump();
}

std::string toJson(const Achievement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = toString(msg.status);
    obj["quality"] = nlohmann::json::parse(toJson(msg.quality));
    obj["achieveability"] = toString(msg.achieveability);
    return obj.dump();
}

std::string toJson(const Requirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    return obj.dump();
}

std::string toJson(const Capability& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["availability"] = msg.availability;
    obj["name"] = msg.name;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& e : msg.contraint) arr.push_back(nlohmann::json::parse(toJson(e)));
        obj["contraint"] = arr;
    }
    return obj.dump();
}

std::string toJson(const Entity& msg) {
    nlohmann::json obj;
    obj["update_time"] = nlohmann::json::parse(toJson(msg.update_time));
    obj["id"] = nlohmann::json::parse(toJson(msg.id));
    obj["source"] = nlohmann::json::parse(toJson(msg.source));
    return obj.dump();
}

std::string toJson(const CircleArea& msg) {
    nlohmann::json obj;
    obj["position"] = nlohmann::json::parse(toJson(msg.position));
    obj["radius"] = nlohmann::json::parse(toJson(msg.radius));
    return obj.dump();
}

std::string toJson(const Point& msg) {
    nlohmann::json obj;
    obj["position"] = nlohmann::json::parse(toJson(msg.position));
    return obj.dump();
}

std::string toJson(const Contraint& msg) {
    nlohmann::json obj;
    obj["name"] = msg.name;
    obj["value"] = msg.value;
    return obj.dump();
}

std::string toJson(const Ack& msg) {
    nlohmann::json obj;
    obj["success"] = msg.success;
    return obj.dump();
}

std::string toJson(const Query& msg) {
    nlohmann::json obj;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& e : msg.id) arr.push_back(nlohmann::json::parse(toJson(e)));
        obj["id"] = arr;
    }
    obj["one_shot"] = msg.one_shot;
    return obj.dump();
}

GeodeticPosition geodeticPositionFromJson(const std::string& s) {
    GeodeticPosition result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("latitude") && j["latitude"].is_object())
            result.latitude = angleFromJson(j["latitude"].dump());
        if (j.contains("longitude") && j["longitude"].is_object())
            result.longitude = angleFromJson(j["longitude"].dump());
    } catch (...) {}
    return result;
}

PolyArea polyAreaFromJson(const std::string& s) {
    PolyArea result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("points") && j["points"].is_array())
            for (const auto& e : j["points"])
                result.points.push_back(geodeticPositionFromJson(e.dump()));
    } catch (...) {}
    return result;
}

Achievement achievementFromJson(const std::string& s) {
    Achievement result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("base") && j["base"].is_object())
            result.base = entityFromJson(j["base"].dump());
        if (j.contains("status"))
            result.status = progressFromString(j["status"].get<std::string>());
        if (j.contains("quality") && j["quality"].is_object())
            result.quality = percentageFromJson(j["quality"].dump());
        if (j.contains("achieveability"))
            result.achieveability = feasibilityFromString(j["achieveability"].get<std::string>());
    } catch (...) {}
    return result;
}

Requirement requirementFromJson(const std::string& s) {
    Requirement result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("base") && j["base"].is_object())
            result.base = entityFromJson(j["base"].dump());
        if (j.contains("status") && j["status"].is_object())
            result.status = achievementFromJson(j["status"].dump());
    } catch (...) {}
    return result;
}

Capability capabilityFromJson(const std::string& s) {
    Capability result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("base") && j["base"].is_object())
            result.base = entityFromJson(j["base"].dump());
        if (j.contains("availability"))
            result.availability = j["availability"].get<bool>();
        if (j.contains("name"))
            result.name = j["name"].get<std::string>();
        if (j.contains("contraint") && j["contraint"].is_array())
            for (const auto& e : j["contraint"])
                result.contraint.push_back(contraintFromJson(e.dump()));
    } catch (...) {}
    return result;
}

Entity entityFromJson(const std::string& s) {
    Entity result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("update_time") && j["update_time"].is_object())
            result.update_time = timestampFromJson(j["update_time"].dump());
        if (j.contains("id") && j["id"].is_object())
            result.id = identifierFromJson(j["id"].dump());
        if (j.contains("source") && j["source"].is_object())
            result.source = identifierFromJson(j["source"].dump());
    } catch (...) {}
    return result;
}

CircleArea circleAreaFromJson(const std::string& s) {
    CircleArea result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("position") && j["position"].is_object())
            result.position = geodeticPositionFromJson(j["position"].dump());
        if (j.contains("radius") && j["radius"].is_object())
            result.radius = lengthFromJson(j["radius"].dump());
    } catch (...) {}
    return result;
}

Point pointFromJson(const std::string& s) {
    Point result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("position") && j["position"].is_object())
            result.position = geodeticPositionFromJson(j["position"].dump());
    } catch (...) {}
    return result;
}

Contraint contraintFromJson(const std::string& s) {
    Contraint result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("name"))
            result.name = j["name"].get<std::string>();
        if (j.contains("value"))
            result.value = j["value"].get<int32_t>();
    } catch (...) {}
    return result;
}

Ack ackFromJson(const std::string& s) {
    Ack result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("success"))
            result.success = j["success"].get<bool>();
    } catch (...) {}
    return result;
}

Query queryFromJson(const std::string& s) {
    Query result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("id") && j["id"].is_array())
            for (const auto& e : j["id"])
                result.id.push_back(identifierFromJson(e.dump()));
        if (j.contains("one_shot"))
            result.one_shot = j["one_shot"].get<bool>();
    } catch (...) {}
    return result;
}

} // namespace pyramid::data_model::common::json_codec
