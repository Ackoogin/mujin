// Auto-generated data model JSON codec implementation
// Namespace: pyramid::domain_model::tactical

#include "pyramid_data_model_tactical_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_base_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"

namespace pyramid::domain_model::tactical {

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
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["entity_source"] = msg.entity_source;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.source) {
            arr.push_back(toString(v));
        }
        obj["source"] = arr;
    }
    obj["position"] = nlohmann::json::parse(toJson(msg.position));
    obj["creation_time"] = msg.creation_time;
    if (msg.quality.has_value()) {
        obj["quality"] = msg.quality.value();
    }
    if (msg.course.has_value()) {
        obj["course"] = msg.course.value();
    }
    if (msg.speed.has_value()) {
        obj["speed"] = msg.speed.value();
    }
    if (msg.length.has_value()) {
        obj["length"] = msg.length.value();
    }
    obj["identity"] = toString(msg.identity);
    obj["dimension"] = toString(msg.dimension);
    return obj.dump();
}

ObjectDetail fromJson(const std::string& s, ObjectDetail* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ObjectDetail msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("entity_source")) msg.entity_source = j["entity_source"].get<std::string>();
    if (j.contains("source")) {
        for (const auto& v : j["source"]) {
            msg.source.push_back(objectSourceFromString(v.get<std::string>()));
        }
    }
    if (j.contains("position")) msg.position = fromJson(j["position"].dump(), static_cast<pyramid::domain_model::common::GeodeticPosition*>(nullptr));
    if (j.contains("creation_time")) msg.creation_time = j["creation_time"].get<double>();
    if (j.contains("quality")) {
        msg.quality = j["quality"].get<double>();
    }
    if (j.contains("course")) {
        msg.course = j["course"].get<double>();
    }
    if (j.contains("speed")) {
        msg.speed = j["speed"].get<double>();
    }
    if (j.contains("length")) {
        msg.length = j["length"].get<double>();
    }
    if (j.contains("identity")) msg.identity = pyramid::domain_model::common::standardIdentityFromString(j["identity"].get<std::string>());
    if (j.contains("dimension")) msg.dimension = pyramid::domain_model::common::battleDimensionFromString(j["dimension"].get<std::string>());
    return msg;
}

std::string toJson(const ObjectEvidenceRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    obj["policy"] = toString(msg.policy);
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.dimension) {
            arr.push_back(pyramid::domain_model::common::toString(v));
        }
        obj["dimension"] = arr;
    }
    if (msg.poly_area.has_value()) {
        obj["poly_area"] = nlohmann::json::parse(toJson(msg.poly_area.value()));
    }
    if (msg.circle_area.has_value()) {
        obj["circle_area"] = nlohmann::json::parse(toJson(msg.circle_area.value()));
    }
    if (msg.point.has_value()) {
        obj["point"] = nlohmann::json::parse(toJson(msg.point.value()));
    }
    return obj.dump();
}

ObjectEvidenceRequirement fromJson(const std::string& s, ObjectEvidenceRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ObjectEvidenceRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    if (j.contains("policy")) msg.policy = pyramid::domain_model::common::dataPolicyFromString(j["policy"].get<std::string>());
    if (j.contains("dimension")) {
        for (const auto& v : j["dimension"]) {
            msg.dimension.push_back(pyramid::domain_model::common::battleDimensionFromString(v.get<std::string>()));
        }
    }
    if (j.contains("poly_area")) {
        msg.poly_area = fromJson(j["poly_area"].dump(), static_cast<pyramid::domain_model::common::PolyArea*>(nullptr));
    }
    if (j.contains("circle_area")) {
        msg.circle_area = fromJson(j["circle_area"].dump(), static_cast<pyramid::domain_model::common::CircleArea*>(nullptr));
    }
    if (j.contains("point")) {
        msg.point = fromJson(j["point"].dump(), static_cast<pyramid::domain_model::common::Point*>(nullptr));
    }
    return msg;
}

std::string toJson(const ObjectInterestRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    if (msg.source.has_value()) {
        obj["source"] = toString(msg.source.value());
    }
    obj["policy"] = toString(msg.policy);
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.dimension) {
            arr.push_back(pyramid::domain_model::common::toString(v));
        }
        obj["dimension"] = arr;
    }
    if (msg.poly_area.has_value()) {
        obj["poly_area"] = nlohmann::json::parse(toJson(msg.poly_area.value()));
    }
    if (msg.circle_area.has_value()) {
        obj["circle_area"] = nlohmann::json::parse(toJson(msg.circle_area.value()));
    }
    if (msg.point.has_value()) {
        obj["point"] = nlohmann::json::parse(toJson(msg.point.value()));
    }
    return obj.dump();
}

ObjectInterestRequirement fromJson(const std::string& s, ObjectInterestRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ObjectInterestRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    if (j.contains("source")) {
        msg.source = objectSourceFromString(j["source"].get<std::string>());
    }
    if (j.contains("policy")) msg.policy = pyramid::domain_model::common::dataPolicyFromString(j["policy"].get<std::string>());
    if (j.contains("dimension")) {
        for (const auto& v : j["dimension"]) {
            msg.dimension.push_back(pyramid::domain_model::common::battleDimensionFromString(v.get<std::string>()));
        }
    }
    if (j.contains("poly_area")) {
        msg.poly_area = fromJson(j["poly_area"].dump(), static_cast<pyramid::domain_model::common::PolyArea*>(nullptr));
    }
    if (j.contains("circle_area")) {
        msg.circle_area = fromJson(j["circle_area"].dump(), static_cast<pyramid::domain_model::common::CircleArea*>(nullptr));
    }
    if (j.contains("point")) {
        msg.point = fromJson(j["point"].dump(), static_cast<pyramid::domain_model::common::Point*>(nullptr));
    }
    return msg;
}

std::string toJson(const ObjectMatch& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["matching_object_id"] = msg.matching_object_id;
    return obj.dump();
}

ObjectMatch fromJson(const std::string& s, ObjectMatch* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ObjectMatch msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("matching_object_id")) msg.matching_object_id = j["matching_object_id"].get<std::string>();
    return msg;
}

} // namespace pyramid::domain_model::tactical
