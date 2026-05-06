// Auto-generated data model JSON codec header
// Generated from: pyramid.data_model.sensors.proto by generate_bindings.py (codec)
// Namespace: pyramid::domain_model::sensors
#pragma once

#include "pyramid_data_model_sensors_types.hpp"
#include <string>

namespace pyramid::domain_model::sensors {

// Enum string converters
std::string toString(InterpretationPolicy v);
InterpretationPolicy interpretationPolicyFromString(const std::string& s);
std::string toString(InterpretationType v);
InterpretationType interpretationTypeFromString(const std::string& s);

// JSON codec
std::string toJson(const InterpretationRequirement& msg);
InterpretationRequirement fromJson(const std::string& s, InterpretationRequirement* /*tag*/ = nullptr);
std::string toJson(const ManualTrackRequirement& msg);
ManualTrackRequirement fromJson(const std::string& s, ManualTrackRequirement* /*tag*/ = nullptr);
std::string toJson(const ATIRequirement& msg);
ATIRequirement fromJson(const std::string& s, ATIRequirement* /*tag*/ = nullptr);
std::string toJson(const TrackProvisionRequirement& msg);
TrackProvisionRequirement fromJson(const std::string& s, TrackProvisionRequirement* /*tag*/ = nullptr);
std::string toJson(const ObjectEvidenceProvisionRequirement& msg);
ObjectEvidenceProvisionRequirement fromJson(const std::string& s, ObjectEvidenceProvisionRequirement* /*tag*/ = nullptr);
std::string toJson(const ObjectAquisitionRequirement& msg);
ObjectAquisitionRequirement fromJson(const std::string& s, ObjectAquisitionRequirement* /*tag*/ = nullptr);
std::string toJson(const SensorObject& msg);
SensorObject fromJson(const std::string& s, SensorObject* /*tag*/ = nullptr);
std::string toJson(const RadarModeChangeRequirement& msg);
RadarModeChangeRequirement fromJson(const std::string& s, RadarModeChangeRequirement* /*tag*/ = nullptr);

} // namespace pyramid::domain_model::sensors
