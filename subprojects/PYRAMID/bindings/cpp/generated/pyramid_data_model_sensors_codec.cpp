// Auto-generated data model JSON codec implementation
// Namespace: pyramid::domain_model::sensors

#include "pyramid_data_model_sensors_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_base_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_radar_codec.hpp"
#include "pyramid_data_model_sensorproducts_codec.hpp"

namespace pyramid::domain_model::sensors {

std::string toString(InterpretationPolicy v) {
    switch (v) {
        case InterpretationPolicy::Unspecified: return "INTERPRETATION_POLICY_UNSPECIFIED";
        case InterpretationPolicy::IgnoreObjects: return "INTERPRETATION_POLICY_IGNORE_OBJECTS";
        case InterpretationPolicy::IncludeObjects: return "INTERPRETATION_POLICY_INCLUDE_OBJECTS";
    }
    return "INTERPRETATION_POLICY_UNSPECIFIED";
}

InterpretationPolicy interpretationPolicyFromString(const std::string& s) {
    if (s == "INTERPRETATION_POLICY_UNSPECIFIED") return InterpretationPolicy::Unspecified;
    if (s == "INTERPRETATION_POLICY_IGNORE_OBJECTS") return InterpretationPolicy::IgnoreObjects;
    if (s == "INTERPRETATION_POLICY_INCLUDE_OBJECTS") return InterpretationPolicy::IncludeObjects;
    return InterpretationPolicy::Unspecified;
}

std::string toString(InterpretationType v) {
    switch (v) {
        case InterpretationType::Unspecified: return "INTERPRETATION_TYPE_UNSPECIFIED";
        case InterpretationType::LocateSeaSurfaceObjects: return "INTERPRETATION_TYPE_LOCATE_SEA_SURFACE_OBJECTS";
    }
    return "INTERPRETATION_TYPE_UNSPECIFIED";
}

InterpretationType interpretationTypeFromString(const std::string& s) {
    if (s == "INTERPRETATION_TYPE_UNSPECIFIED") return InterpretationType::Unspecified;
    if (s == "INTERPRETATION_TYPE_LOCATE_SEA_SURFACE_OBJECTS") return InterpretationType::LocateSeaSurfaceObjects;
    return InterpretationType::Unspecified;
}

std::string toJson(const InterpretationRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    obj["policy"] = toString(msg.policy);
    obj["type"] = toString(msg.type);
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

InterpretationRequirement fromJson(const std::string& s, InterpretationRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    InterpretationRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    if (j.contains("policy")) msg.policy = interpretationPolicyFromString(j["policy"].get<std::string>());
    if (j.contains("type")) msg.type = interpretationTypeFromString(j["type"].get<std::string>());
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

std::string toJson(const ManualTrackRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    obj["position"] = nlohmann::json::parse(toJson(msg.position));
    return obj.dump();
}

ManualTrackRequirement fromJson(const std::string& s, ManualTrackRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ManualTrackRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    if (j.contains("position")) msg.position = fromJson(j["position"].dump(), static_cast<pyramid::domain_model::common::GeodeticPosition*>(nullptr));
    return msg;
}

std::string toJson(const ATIRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    obj["auto_zone"] = nlohmann::json::parse(toJson(msg.auto_zone));
    return obj.dump();
}

ATIRequirement fromJson(const std::string& s, ATIRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ATIRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    if (j.contains("auto_zone")) msg.auto_zone = fromJson(j["auto_zone"].dump(), static_cast<pyramid::domain_model::common::PolyArea*>(nullptr));
    return msg;
}

std::string toJson(const TrackProvisionRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
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

TrackProvisionRequirement fromJson(const std::string& s, TrackProvisionRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    TrackProvisionRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
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

std::string toJson(const ObjectEvidenceProvisionRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
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

ObjectEvidenceProvisionRequirement fromJson(const std::string& s, ObjectEvidenceProvisionRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ObjectEvidenceProvisionRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
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

std::string toJson(const ObjectAquisitionRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
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

ObjectAquisitionRequirement fromJson(const std::string& s, ObjectAquisitionRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ObjectAquisitionRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
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

std::string toJson(const SensorObject& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
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
    return obj.dump();
}

SensorObject fromJson(const std::string& s, SensorObject* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    SensorObject msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
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
    return msg;
}

std::string toJson(const RadarModeChangeRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    obj["mode"] = toString(msg.mode);
    return obj.dump();
}

RadarModeChangeRequirement fromJson(const std::string& s, RadarModeChangeRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    RadarModeChangeRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    if (j.contains("mode")) msg.mode = pyramid::domain_model::radar::radar_Operational_ModeFromString(j["mode"].get<std::string>());
    return msg;
}

} // namespace pyramid::domain_model::sensors
