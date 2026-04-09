// Auto-generated service wire types header
// Namespace: pyramid::services::tactical_objects::wire_types
// Generated from pim/json_schema.py
#pragma once

#include "pyramid_data_model_types.hpp"

#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::wire_types {

using namespace pyramid::data_model;

// ---------------------------------------------------------------------------
// Wire message structs
// ---------------------------------------------------------------------------

// object_of_interest.create_requirement request (proto: ObjectInterestRequirement)
struct CreateRequirementRequest {
    DataPolicy policy = DataPolicy::Unspecified;
    StandardIdentity identity = StandardIdentity::Unspecified;
    BattleDimension dimension = BattleDimension::Unspecified;  // Optional battle-dimension filter
    double min_lat_rad = 0.0;  // Bounding-box south edge, radians
    double max_lat_rad = 0.0;  // Bounding-box north edge, radians
    double min_lon_rad = 0.0;  // Bounding-box west edge, radians
    double max_lon_rad = 0.0;  // Bounding-box east edge, radians
};

// object_of_interest.create_requirement response (proto: Identifier)
struct CreateRequirementResponse {
    std::string interest_id = {};  // Assigned interest ID; absent when creation failed
};

// element of standard.entity_matches array (proto: ObjectMatch + position/confidence)
struct EntityMatch {
    std::string object_id = {};
    StandardIdentity identity = StandardIdentity::Unspecified;
    BattleDimension dimension = BattleDimension::Unspecified;  // optional
    double latitude_rad = 0.0;  // optional
    double longitude_rad = 0.0;  // optional
    double confidence = 0.0;  // optional
};

// standard.object_evidence publish payload (proto: ObjectDetail)
struct ObjectEvidence {
    StandardIdentity identity = StandardIdentity::Unspecified;
    BattleDimension dimension = BattleDimension::Unspecified;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double confidence = 0.0;
    double observed_at = 0.0;  // Observation timestamp, seconds
};

// standard.evidence_requirements subscribe payload (proto: ObjectEvidenceRequirement)
struct EvidenceRequirement {
    std::string id = {};  // optional
    DataPolicy policy = DataPolicy::Unspecified;  // optional
    BattleDimension dimension = BattleDimension::Unspecified;  // optional
    double min_lat_rad = 0.0;  // optional
    double max_lat_rad = 0.0;  // optional
    double min_lon_rad = 0.0;  // optional
    double max_lon_rad = 0.0;  // optional
};

using EntityMatchArray = std::vector<EntityMatch>;

} // namespace pyramid::services::tactical_objects::wire_types
