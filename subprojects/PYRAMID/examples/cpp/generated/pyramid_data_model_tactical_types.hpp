// Auto-generated types header
// Generated from: tactical.proto by generate_bindings.py (types)
// Namespace: pyramid::data_model::tactical
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "pyramid_data_model_base_types.hpp"
#include "pyramid_data_model_common_types.hpp"

namespace pyramid::data_model::tactical {


enum class ObjectSource : int {
    Unspecified = 0,
    Radar = 1,
    Local = 2,
};

struct ObjectDetail {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string entity_source = {};  // from Entity  // optional
    std::vector<ObjectSource> source = {};
    pyramid::data_model::common::GeodeticPosition position = {};
    double creation_time = 0.0;
    std::optional<double> quality;  // optional
    std::optional<double> course;  // optional
    std::optional<double> speed;  // optional
    std::optional<double> length;  // optional
    pyramid::data_model::common::StandardIdentity identity = pyramid::data_model::common::StandardIdentity::Unspecified;
    pyramid::data_model::common::BattleDimension dimension = pyramid::data_model::common::BattleDimension::Unspecified;
};

struct ObjectEvidenceRequirement {
    pyramid::data_model::common::Entity base = {};  // from Requirement
    pyramid::data_model::common::Achievement status = {};  // from Requirement
    pyramid::data_model::common::DataPolicy policy = pyramid::data_model::common::DataPolicy::Unspecified;
    std::vector<pyramid::data_model::common::BattleDimension> dimension = {};
    // oneof location
    std::optional<pyramid::data_model::common::PolyArea> poly_area;
    std::optional<pyramid::data_model::common::CircleArea> circle_area;
    std::optional<pyramid::data_model::common::Point> point;
};

struct ObjectInterestRequirement {
    pyramid::data_model::common::Entity base = {};  // from Requirement
    pyramid::data_model::common::Achievement status = {};  // from Requirement
    std::optional<ObjectSource> source;  // optional
    pyramid::data_model::common::DataPolicy policy = pyramid::data_model::common::DataPolicy::Unspecified;
    std::vector<pyramid::data_model::common::BattleDimension> dimension = {};
    // oneof location
    std::optional<pyramid::data_model::common::PolyArea> poly_area;
    std::optional<pyramid::data_model::common::CircleArea> circle_area;
    std::optional<pyramid::data_model::common::Point> point;
};

struct ObjectMatch {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string matching_object_id = {};
};

} // namespace pyramid::data_model::tactical
