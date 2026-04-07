// Auto-generated JSON codec — do not edit
// Backend: json | Namespace: pyramid::data_model::tactical::json_codec
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace pyramid::data_model::tactical::json_codec {

enum class ObjectSource {
    Unspecified = 0,
    Radar = 1,
    Local = 2,
};

std::string toString(ObjectSource v);
ObjectSource objectSourceFromString(const std::string& s);

struct ObjectDetail {
    Entity base = {};
    std::vector<ObjectSource> source = {};
    GeodeticPosition position = {};
    Timestamp creation_time = {};
    Percentage quality = {};
    Angle course = {};
    Speed speed = {};
    Length length = {};
    StandardIdentity identity = StandardIdentity::Unspecified;
    BattleDimension dimension = BattleDimension::Unspecified;
};

struct ObjectEvidenceRequirement {
    Requirement base = {};
    DataPolicy policy = DataPolicy::Unspecified;
    std::vector<BattleDimension> dimension = {};
    PolyArea poly_area = {};
    CircleArea circle_area = {};
    Point point = {};
};

struct ObjectInterestRequirement {
    Requirement base = {};
    ObjectSource source = ObjectSource::Unspecified;
    DataPolicy policy = DataPolicy::Unspecified;
    std::vector<BattleDimension> dimension = {};
    PolyArea poly_area = {};
    CircleArea circle_area = {};
    Point point = {};
};

struct ObjectMatch {
    Entity base = {};
    Identifier matching_object_id = {};
};

std::string toJson(const ObjectDetail& msg);
ObjectDetail objectDetailFromJson(const std::string& s);
std::string toJson(const ObjectEvidenceRequirement& msg);
ObjectEvidenceRequirement objectEvidenceRequirementFromJson(const std::string& s);
std::string toJson(const ObjectInterestRequirement& msg);
ObjectInterestRequirement objectInterestRequirementFromJson(const std::string& s);
std::string toJson(const ObjectMatch& msg);
ObjectMatch objectMatchFromJson(const std::string& s);

} // namespace pyramid::data_model::tactical::json_codec
