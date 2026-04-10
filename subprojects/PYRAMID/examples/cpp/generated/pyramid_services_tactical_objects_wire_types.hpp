// Auto-generated service wire types header
// Namespace: pyramid::services::tactical_objects::wire_types
// Generated from pim/json_schema.py
#pragma once

#include "pyramid_data_model_types.hpp"

#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::wire_types {

// ---------------------------------------------------------------------------
// Wire message structs
// ---------------------------------------------------------------------------

// element of standard.entity_matches array (proto: ObjectMatch + position/confidence)
struct EntityMatch {
    std::string object_id = {};
    pyramid::data_model::StandardIdentity identity = pyramid::data_model::StandardIdentity::Unspecified;
    pyramid::data_model::BattleDimension dimension = pyramid::data_model::BattleDimension::Unspecified;  // optional
    double latitude_rad = 0.0;  // optional
    double longitude_rad = 0.0;  // optional
    double confidence = 0.0;  // optional
};

// standard.object_evidence publish payload (proto: ObjectDetail)
struct ObjectEvidence {
    pyramid::data_model::StandardIdentity identity = pyramid::data_model::StandardIdentity::Unspecified;
    pyramid::data_model::BattleDimension dimension = pyramid::data_model::BattleDimension::Unspecified;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double confidence = 0.0;
    double observed_at = 0.0;  // Observation timestamp, seconds
};

// standard.evidence_requirements subscribe payload (proto: ObjectEvidenceRequirement)
struct EvidenceRequirement {
    std::string id = {};  // optional
    pyramid::data_model::DataPolicy policy = pyramid::data_model::DataPolicy::Unspecified;  // optional
    pyramid::data_model::BattleDimension dimension = pyramid::data_model::BattleDimension::Unspecified;  // optional
    double min_lat_rad = 0.0;  // optional
    double max_lat_rad = 0.0;  // optional
    double min_lon_rad = 0.0;  // optional
    double max_lon_rad = 0.0;  // optional
};

using EntityMatchArray = std::vector<EntityMatch>;

} // namespace pyramid::services::tactical_objects::wire_types
