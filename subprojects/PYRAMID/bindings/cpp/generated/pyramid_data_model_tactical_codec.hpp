// Auto-generated data model JSON codec header
// Generated from: pyramid.data_model.tactical.proto by generate_bindings.py (codec)
// Namespace: pyramid::domain_model::tactical
#pragma once

#include "pyramid_data_model_tactical_types.hpp"
#include <string>

namespace pyramid::domain_model::tactical {

// Enum string converters
std::string toString(ObjectSource v);
ObjectSource objectSourceFromString(const std::string& s);

// JSON codec
std::string toJson(const ObjectDetail& msg);
ObjectDetail fromJson(const std::string& s, ObjectDetail* /*tag*/ = nullptr);
std::string toJson(const ObjectEvidenceRequirement& msg);
ObjectEvidenceRequirement fromJson(const std::string& s, ObjectEvidenceRequirement* /*tag*/ = nullptr);
std::string toJson(const ObjectInterestRequirement& msg);
ObjectInterestRequirement fromJson(const std::string& s, ObjectInterestRequirement* /*tag*/ = nullptr);
std::string toJson(const ObjectMatch& msg);
ObjectMatch fromJson(const std::string& s, ObjectMatch* /*tag*/ = nullptr);

} // namespace pyramid::domain_model::tactical
