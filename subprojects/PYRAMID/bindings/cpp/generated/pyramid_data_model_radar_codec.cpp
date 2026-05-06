// Auto-generated data model JSON codec implementation
// Namespace: pyramid::domain_model::radar

#include "pyramid_data_model_radar_codec.hpp"

#include <nlohmann/json.hpp>


namespace pyramid::domain_model::radar {

std::string toString(Radar_Operational_Mode v) {
    switch (v) {
        case Radar_Operational_Mode::Unspecified: return "RADAR_OPERATIONAL_MODE_UNSPECIFIED";
        case Radar_Operational_Mode::MaritimeSurveillance: return "RADAR_OPERATIONAL_MODE_MARITIME_SURVEILLANCE";
        case Radar_Operational_Mode::ShortRangeAwareness: return "RADAR_OPERATIONAL_MODE_SHORT_RANGE_AWARENESS";
        case Radar_Operational_Mode::Transponder: return "RADAR_OPERATIONAL_MODE_TRANSPONDER";
        case Radar_Operational_Mode::AMti: return "RADAR_OPERATIONAL_MODE_A_MTI";
        case Radar_Operational_Mode::EnhancedMovingTargetIndicator: return "RADAR_OPERATIONAL_MODE_ENHANCED_MOVING_TARGET_INDICATOR";
        case Radar_Operational_Mode::Weather: return "RADAR_OPERATIONAL_MODE_WEATHER";
        case Radar_Operational_Mode::AirToAirSurveillance: return "RADAR_OPERATIONAL_MODE_AIR_TO_AIR_SURVEILLANCE";
        case Radar_Operational_Mode::SmallTarget: return "RADAR_OPERATIONAL_MODE_SMALL_TARGET";
        case Radar_Operational_Mode::Turbulence: return "RADAR_OPERATIONAL_MODE_TURBULENCE";
        case Radar_Operational_Mode::Isar: return "RADAR_OPERATIONAL_MODE_ISAR";
        case Radar_Operational_Mode::SpotSar: return "RADAR_OPERATIONAL_MODE_SPOT_SAR";
        case Radar_Operational_Mode::StripSar: return "RADAR_OPERATIONAL_MODE_STRIP_SAR";
        case Radar_Operational_Mode::Hrr: return "RADAR_OPERATIONAL_MODE_HRR";
        case Radar_Operational_Mode::PlannedSar: return "RADAR_OPERATIONAL_MODE_PLANNED_SAR";
        case Radar_Operational_Mode::GMti: return "RADAR_OPERATIONAL_MODE_G_MTI";
        case Radar_Operational_Mode::MMti: return "RADAR_OPERATIONAL_MODE_M_MTI";
    }
    return "RADAR_OPERATIONAL_MODE_UNSPECIFIED";
}

Radar_Operational_Mode radar_Operational_ModeFromString(const std::string& s) {
    if (s == "RADAR_OPERATIONAL_MODE_UNSPECIFIED") return Radar_Operational_Mode::Unspecified;
    if (s == "RADAR_OPERATIONAL_MODE_MARITIME_SURVEILLANCE") return Radar_Operational_Mode::MaritimeSurveillance;
    if (s == "RADAR_OPERATIONAL_MODE_SHORT_RANGE_AWARENESS") return Radar_Operational_Mode::ShortRangeAwareness;
    if (s == "RADAR_OPERATIONAL_MODE_TRANSPONDER") return Radar_Operational_Mode::Transponder;
    if (s == "RADAR_OPERATIONAL_MODE_A_MTI") return Radar_Operational_Mode::AMti;
    if (s == "RADAR_OPERATIONAL_MODE_ENHANCED_MOVING_TARGET_INDICATOR") return Radar_Operational_Mode::EnhancedMovingTargetIndicator;
    if (s == "RADAR_OPERATIONAL_MODE_WEATHER") return Radar_Operational_Mode::Weather;
    if (s == "RADAR_OPERATIONAL_MODE_AIR_TO_AIR_SURVEILLANCE") return Radar_Operational_Mode::AirToAirSurveillance;
    if (s == "RADAR_OPERATIONAL_MODE_SMALL_TARGET") return Radar_Operational_Mode::SmallTarget;
    if (s == "RADAR_OPERATIONAL_MODE_TURBULENCE") return Radar_Operational_Mode::Turbulence;
    if (s == "RADAR_OPERATIONAL_MODE_ISAR") return Radar_Operational_Mode::Isar;
    if (s == "RADAR_OPERATIONAL_MODE_SPOT_SAR") return Radar_Operational_Mode::SpotSar;
    if (s == "RADAR_OPERATIONAL_MODE_STRIP_SAR") return Radar_Operational_Mode::StripSar;
    if (s == "RADAR_OPERATIONAL_MODE_HRR") return Radar_Operational_Mode::Hrr;
    if (s == "RADAR_OPERATIONAL_MODE_PLANNED_SAR") return Radar_Operational_Mode::PlannedSar;
    if (s == "RADAR_OPERATIONAL_MODE_G_MTI") return Radar_Operational_Mode::GMti;
    if (s == "RADAR_OPERATIONAL_MODE_M_MTI") return Radar_Operational_Mode::MMti;
    return Radar_Operational_Mode::Unspecified;
}

std::string toString(Radar_Operational_State v) {
    switch (v) {
        case Radar_Operational_State::Unspecified: return "RADAR_OPERATIONAL_STATE_UNSPECIFIED";
    }
    return "RADAR_OPERATIONAL_STATE_UNSPECIFIED";
}

Radar_Operational_State radar_Operational_StateFromString(const std::string& s) {
    if (s == "RADAR_OPERATIONAL_STATE_UNSPECIFIED") return Radar_Operational_State::Unspecified;
    return Radar_Operational_State::Unspecified;
}

} // namespace pyramid::domain_model::radar
