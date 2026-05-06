// Auto-generated data model JSON codec header
// Generated from: pyramid.data_model.sensorproducts.proto by generate_bindings.py (codec)
// Namespace: pyramid::domain_model::sensorproducts
#pragma once

#include "pyramid_data_model_sensorproducts_types.hpp"
#include <string>

namespace pyramid::domain_model::sensorproducts {

// JSON codec
std::string toJson(const RadarDisplayProductRequirement& msg);
RadarDisplayProductRequirement fromJson(const std::string& s, RadarDisplayProductRequirement* /*tag*/ = nullptr);
std::string toJson(const RadarProductRequirement& msg);
RadarProductRequirement fromJson(const std::string& s, RadarProductRequirement* /*tag*/ = nullptr);

} // namespace pyramid::domain_model::sensorproducts
