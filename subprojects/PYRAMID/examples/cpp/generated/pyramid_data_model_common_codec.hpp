// Auto-generated data model JSON codec header
// Generated from: common.proto by cpp_service_generator.py --codec
// Namespace: pyramid::data_model::common
#pragma once

#include "pyramid_data_model_common_types.hpp"
#include <string>

namespace pyramid::data_model::common {

// Enum string converters
std::string toString(Feasibility v);
Feasibility feasibilityFromString(const std::string& s);
std::string toString(Progress v);
Progress progressFromString(const std::string& s);
std::string toString(StandardIdentity v);
StandardIdentity standardIdentityFromString(const std::string& s);
std::string toString(BattleDimension v);
BattleDimension battleDimensionFromString(const std::string& s);
std::string toString(DataPolicy v);
DataPolicy dataPolicyFromString(const std::string& s);

// JSON codec
std::string toJson(const GeodeticPosition& msg);
GeodeticPosition fromJson(const std::string& s, GeodeticPosition* /*tag*/ = nullptr);
std::string toJson(const PolyArea& msg);
PolyArea fromJson(const std::string& s, PolyArea* /*tag*/ = nullptr);
std::string toJson(const Achievement& msg);
Achievement fromJson(const std::string& s, Achievement* /*tag*/ = nullptr);
std::string toJson(const Requirement& msg);
Requirement fromJson(const std::string& s, Requirement* /*tag*/ = nullptr);
std::string toJson(const Capability& msg);
Capability fromJson(const std::string& s, Capability* /*tag*/ = nullptr);
std::string toJson(const Entity& msg);
Entity fromJson(const std::string& s, Entity* /*tag*/ = nullptr);
std::string toJson(const CircleArea& msg);
CircleArea fromJson(const std::string& s, CircleArea* /*tag*/ = nullptr);
std::string toJson(const Point& msg);
Point fromJson(const std::string& s, Point* /*tag*/ = nullptr);
std::string toJson(const Contraint& msg);
Contraint fromJson(const std::string& s, Contraint* /*tag*/ = nullptr);
std::string toJson(const Ack& msg);
Ack fromJson(const std::string& s, Ack* /*tag*/ = nullptr);
std::string toJson(const Query& msg);
Query fromJson(const std::string& s, Query* /*tag*/ = nullptr);

} // namespace pyramid::data_model::common
