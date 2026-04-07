// Auto-generated JSON codec — do not edit
#include "pyramid_data_model_base_json_codec.hpp"

#include <nlohmann/json.hpp>

namespace pyramid::data_model::base::json_codec {

std::string toJson(const Angle& msg) {
    nlohmann::json obj;
    obj["radians"] = msg.radians;
    return obj.dump();
}

std::string toJson(const Length& msg) {
    nlohmann::json obj;
    obj["meters"] = msg.meters;
    return obj.dump();
}

std::string toJson(const Timestamp& msg) {
    nlohmann::json obj;
    obj["value"] = nlohmann::json::parse(toJson(msg.value));
    return obj.dump();
}

std::string toJson(const Identifier& msg) {
    nlohmann::json obj;
    obj["value"] = msg.value;
    return obj.dump();
}

std::string toJson(const Speed& msg) {
    nlohmann::json obj;
    obj["meters_per_second"] = msg.meters_per_second;
    return obj.dump();
}

std::string toJson(const Percentage& msg) {
    nlohmann::json obj;
    obj["value"] = msg.value;
    return obj.dump();
}

Angle angleFromJson(const std::string& s) {
    Angle result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("radians"))
            result.radians = j["radians"].get<double>();
    } catch (...) {}
    return result;
}

Length lengthFromJson(const std::string& s) {
    Length result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("meters"))
            result.meters = j["meters"].get<double>();
    } catch (...) {}
    return result;
}

Timestamp timestampFromJson(const std::string& s) {
    Timestamp result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("value") && j["value"].is_object())
            result.value = timestampFromJson(j["value"].dump());
    } catch (...) {}
    return result;
}

Identifier identifierFromJson(const std::string& s) {
    Identifier result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("value"))
            result.value = j["value"].get<std::string>();
    } catch (...) {}
    return result;
}

Speed speedFromJson(const std::string& s) {
    Speed result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("meters_per_second"))
            result.meters_per_second = j["meters_per_second"].get<double>();
    } catch (...) {}
    return result;
}

Percentage percentageFromJson(const std::string& s) {
    Percentage result;
    try {
        auto j = nlohmann::json::parse(s);
        if (j.contains("value"))
            result.value = j["value"].get<double>();
    } catch (...) {}
    return result;
}

} // namespace pyramid::data_model::base::json_codec
