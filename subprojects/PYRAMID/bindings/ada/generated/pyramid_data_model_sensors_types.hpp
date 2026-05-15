// Auto-generated types header
// Generated from: pyramid.data_model.sensors.proto by generate_bindings.py (types)
// Namespace: pyramid::domain_model::sensors
#pragma once

#include <cstdint>
#include <tl/optional.hpp>
#include <string>
#include <vector>
#include "pyramid_data_model_base_types.hpp"
#include "pyramid_data_model_common_types.hpp"
#include "pyramid_data_model_radar_types.hpp"
#include "pyramid_data_model_sensorproducts_types.hpp"

namespace pyramid::domain_model::sensors {


enum class InterpretationPolicy : int {
    Unspecified = 0,
    IgnoreObjects = 1,
    IncludeObjects = 2,
};

enum class InterpretationType : int {
    Unspecified = 0,
    LocateSeaSurfaceObjects = 1,
};

struct InterpretationRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
    InterpretationPolicy policy = InterpretationPolicy::Unspecified;
    InterpretationType type = InterpretationType::Unspecified;
    // oneof location
    tl::optional<pyramid::domain_model::common::PolyArea> poly_area;
    tl::optional<pyramid::domain_model::common::CircleArea> circle_area;
    tl::optional<pyramid::domain_model::common::Point> point;
};

struct ManualTrackRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
    pyramid::domain_model::common::GeodeticPosition position = {};
};

struct ATIRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
    pyramid::domain_model::common::PolyArea auto_zone = {};
};

struct TrackProvisionRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
    // oneof location
    tl::optional<pyramid::domain_model::common::PolyArea> poly_area;
    tl::optional<pyramid::domain_model::common::CircleArea> circle_area;
    tl::optional<pyramid::domain_model::common::Point> point;
};

struct ObjectEvidenceProvisionRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
    // oneof location
    tl::optional<pyramid::domain_model::common::PolyArea> poly_area;
    tl::optional<pyramid::domain_model::common::CircleArea> circle_area;
    tl::optional<pyramid::domain_model::common::Point> point;
};

struct ObjectAquisitionRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
    // oneof location
    tl::optional<pyramid::domain_model::common::PolyArea> poly_area;
    tl::optional<pyramid::domain_model::common::CircleArea> circle_area;
    tl::optional<pyramid::domain_model::common::Point> point;
};

struct SensorObject {
    tl::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    pyramid::domain_model::common::GeodeticPosition position = {};
    double creation_time = 0.0;
    tl::optional<double> quality;  // optional
    tl::optional<double> course;  // optional
    tl::optional<double> speed;  // optional
    tl::optional<double> length;  // optional
};

struct RadarModeChangeRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
    pyramid::domain_model::radar::Radar_Operational_Mode mode = pyramid::domain_model::radar::Radar_Operational_Mode::Unspecified;
};

} // namespace pyramid::domain_model::sensors
