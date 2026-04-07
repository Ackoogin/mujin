// Auto-generated JSON codec — do not edit
#include "pyramid_data_model_tactical_json_codec.hpp"

#include <nlohmann/json.hpp>

namespace pyramid::data_model::tactical::json_codec {

std::string toString(ObjectSource v) {
    switch (v) {
        case ObjectSource::Unspecified: return "OBJECT_SOURCE_UNSPECIFIED";
        case ObjectSource::Radar: return "OBJECT_SOURCE_RADAR";
        case ObjectSource::Local: return "OBJECT_SOURCE_LOCAL";
    }
    return "OBJECT_SOURCE_UNSPECIFIED";
}

ObjectSource objectSourceFromString(const std::string& s) {
    if (s == "OBJECT_SOURCE_UNSPECIFIED") return ObjectSource::Unspecified;
    if (s == "OBJECT_SOURCE_RADAR") return ObjectSource::Radar;
    if (s == "OBJECT_SOURCE_LOCAL") return ObjectSource::Local;
    return ObjectSource::Unspecified;
}

std::string toJson(const ObjectDetail& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& e : msg.source) arr.push_back(toString(e));
        obj["source"] = arr;
    }
    obj["position"] = nlohmann::json::parse(toJson(msg.position));
    obj["creation_time"] = nlohmann::json::parse(toJson(msg.creation_time));
    obj["quality"] = nlohmann::json::parse(toJson(msg.quality));
    obj["course"] = nlohmann::json::parse(toJson(msg.course));
    obj["speed"] = nlohmann::json::parse(toJson(msg.speed));
    obj["length"] = nlohmann::json::parse(toJson(msg.length));
    obj["identity"] = toString(msg.identity);
    obj["dimension"] = toString(msg.dimension);
    return obj.dump();
}

std::string toJson(const ObjectEvidenceRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["policy"] = toString(msg.policy);
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& e : msg.dimension) arr.push_back(toString(e));
        obj["dimension"] = arr;
    }
    obj["poly_area"] = nlohmann::json::parse(toJson(msg.poly_area));
    obj["circle_area"] = nlohmann::json::parse(toJson(msg.circle_area));
    obj["point"] = nlohmann::json::parse(toJson(msg.point));
    return obj.dump();
}

std::string toJson(const ObjectInterestRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["source"] = toString(msg.source);
    obj["policy"] = toString(msg.policy);
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& e : msg.dimension) arr.push_back(toString(e));
        obj["dimension"] = arr;
    }
    obj["poly_area"] = nlohmann::json::parse(toJson(msg.poly_area));
    obj["circle_area"] = nlohmann::json::parse(toJson(msg.circle_area));
    obj["point"] = nlohmann::json::parse(toJson(msg.point));
    return obj.dump();
}

std::string toJson(const ObjectMatch& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["matching_object_id"] = nlohmann::json::parse(toJson(msg.matching_object_id));
    return obj.dump();
}

ObjectDetail objectDetailFromJson(const std::string& s) {
    ObjectDetail result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("base") && j["base"].is_object())
            result.base = entityFromJson(j["base"].dump());
        if (j.contains("source") && j["source"].is_array())
            for (const auto& e : j["source"])
                result.source.push_back(objectSourceFromString(e.get<std::string>()));
        if (j.contains("position") && j["position"].is_object())
            result.position = geodeticPositionFromJson(j["position"].dump());
        if (j.contains("creation_time") && j["creation_time"].is_object())
            result.creation_time = timestampFromJson(j["creation_time"].dump());
        if (j.contains("quality") && j["quality"].is_object())
            result.quality = percentageFromJson(j["quality"].dump());
        if (j.contains("course") && j["course"].is_object())
            result.course = angleFromJson(j["course"].dump());
        if (j.contains("speed") && j["speed"].is_object())
            result.speed = speedFromJson(j["speed"].dump());
        if (j.contains("length") && j["length"].is_object())
            result.length = lengthFromJson(j["length"].dump());
        if (j.contains("identity"))
            result.identity = standardIdentityFromString(j["identity"].get<std::string>());
        if (j.contains("dimension"))
            result.dimension = battleDimensionFromString(j["dimension"].get<std::string>());
    } catch (...) {}
    return result;
}

ObjectEvidenceRequirement objectEvidenceRequirementFromJson(const std::string& s) {
    ObjectEvidenceRequirement result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("base") && j["base"].is_object())
            result.base = requirementFromJson(j["base"].dump());
        if (j.contains("policy"))
            result.policy = dataPolicyFromString(j["policy"].get<std::string>());
        if (j.contains("dimension") && j["dimension"].is_array())
            for (const auto& e : j["dimension"])
                result.dimension.push_back(battleDimensionFromString(e.get<std::string>()));
        if (j.contains("poly_area") && j["poly_area"].is_object())
            result.poly_area = polyAreaFromJson(j["poly_area"].dump());
        if (j.contains("circle_area") && j["circle_area"].is_object())
            result.circle_area = circleAreaFromJson(j["circle_area"].dump());
        if (j.contains("point") && j["point"].is_object())
            result.point = pointFromJson(j["point"].dump());
    } catch (...) {}
    return result;
}

ObjectInterestRequirement objectInterestRequirementFromJson(const std::string& s) {
    ObjectInterestRequirement result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("base") && j["base"].is_object())
            result.base = requirementFromJson(j["base"].dump());
        if (j.contains("source"))
            result.source = objectSourceFromString(j["source"].get<std::string>());
        if (j.contains("policy"))
            result.policy = dataPolicyFromString(j["policy"].get<std::string>());
        if (j.contains("dimension") && j["dimension"].is_array())
            for (const auto& e : j["dimension"])
                result.dimension.push_back(battleDimensionFromString(e.get<std::string>()));
        if (j.contains("poly_area") && j["poly_area"].is_object())
            result.poly_area = polyAreaFromJson(j["poly_area"].dump());
        if (j.contains("circle_area") && j["circle_area"].is_object())
            result.circle_area = circleAreaFromJson(j["circle_area"].dump());
        if (j.contains("point") && j["point"].is_object())
            result.point = pointFromJson(j["point"].dump());
    } catch (...) {}
    return result;
}

ObjectMatch objectMatchFromJson(const std::string& s) {
    ObjectMatch result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("base") && j["base"].is_object())
            result.base = entityFromJson(j["base"].dump());
        if (j.contains("matching_object_id") && j["matching_object_id"].is_object())
            result.matching_object_id = identifierFromJson(j["matching_object_id"].dump());
    } catch (...) {}
    return result;
}

} // namespace pyramid::data_model::tactical::json_codec
