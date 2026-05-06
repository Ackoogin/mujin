// Auto-generated data model JSON codec implementation
// Namespace: pyramid::domain_model::sensorproducts

#include "pyramid_data_model_sensorproducts_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_common_codec.hpp"

namespace pyramid::domain_model::sensorproducts {

std::string toJson(const RadarDisplayProductRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    return obj.dump();
}

RadarDisplayProductRequirement fromJson(const std::string& s, RadarDisplayProductRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    RadarDisplayProductRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    return msg;
}

std::string toJson(const RadarProductRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    return obj.dump();
}

RadarProductRequirement fromJson(const std::string& s, RadarProductRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    RadarProductRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::domain_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::domain_model::common::Achievement*>(nullptr));
    return msg;
}

} // namespace pyramid::domain_model::sensorproducts
