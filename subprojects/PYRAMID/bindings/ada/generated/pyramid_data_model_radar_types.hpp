// Auto-generated types header
// Generated from: pyramid.data_model.radar.proto by generate_bindings.py (types)
// Namespace: pyramid::domain_model::radar
#pragma once

#include <cstdint>
#include <tl/optional.hpp>
#include <string>
#include <vector>

namespace pyramid::domain_model::radar {


enum class Radar_Operational_Mode : int {
    Unspecified = 0,
    MaritimeSurveillance = 1,
    ShortRangeAwareness = 2,
    Transponder = 3,
    AMti = 4,
    EnhancedMovingTargetIndicator = 5,
    Weather = 6,
    AirToAirSurveillance = 7,
    SmallTarget = 8,
    Turbulence = 9,
    Isar = 10,
    SpotSar = 11,
    StripSar = 12,
    Hrr = 13,
    PlannedSar = 14,
    GMti = 15,
    MMti = 16,
};

enum class Radar_Operational_State : int {
    Unspecified = 0,
};

} // namespace pyramid::domain_model::radar
