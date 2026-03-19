// Auto-generated service types header
// Generated from: proto/pyramid/data_model/ by cpp_service_generator
// Namespace: pyramid::services::tactical_objects
//
// Architecture: component logic > service binding > PCL
//
// C++ types mirroring the standard pyramid data model proto files:
//   proto/pyramid/data_model/base.proto
//   proto/pyramid/data_model/common.proto
//   proto/pyramid/data_model/tactical.proto
#pragma once

#include <string>
#include <vector>
#include <optional>

namespace pyramid::services::tactical_objects {

// -- base.proto: Identifier ------------------------------------------------

using Identifier = std::string;

// -- common.proto: Ack / Query ---------------------------------------------

struct Ack {
    bool success = false;
};

constexpr Ack kAckOk  { true  };
constexpr Ack kAckFail{ false };

struct Query {
    std::vector<Identifier> ids;
    bool one_shot = true;
};

// -- common.proto: BattleDimension -----------------------------------------
// Ordinals match pyramid.data_model.common.BattleDimension proto values.

enum class BattleDimension : int {
    Unspecified = 0,
    Ground      = 1,
    Subsurface  = 2,
    SeaSurface  = 3,
    Air         = 4,
    Unknown     = 5,
};

// -- common.proto: DataPolicy ----------------------------------------------

enum class DataPolicy : int {
    Unspecified = 0,
    Query       = 1,   // query existing data only
    Obtain      = 2,   // actively collect new data
};

// -- common.proto: StandardIdentity ----------------------------------------
// Ordinals match pyramid.data_model.common.StandardIdentity proto values.

enum class StandardIdentity : int {
    Unspecified     = 0,
    Unknown         = 1,
    Friendly        = 2,
    Hostile         = 3,
    Suspect         = 4,
    Neutral         = 5,
    Pending         = 6,
    Joker           = 7,
    Faker           = 8,
    AssumedFriendly = 9,
};

// -- common.proto: GeodeticPosition ----------------------------------------
// Geodetic coordinates (radians), matching proto Angle.radians field.

struct GeodeticPosition {
    double latitude  = 0.0;  // radians
    double longitude = 0.0;  // radians
};

// -- common.proto: Capability ----------------------------------------------

struct Capability {
    Identifier id;
    bool availability = false;
    std::string name;
};

// -- tactical.proto: ObjectSource ------------------------------------------

enum class ObjectSource : int {
    Unspecified = 0,
    Radar       = 1,
    Local       = 2,
};

// -- tactical.proto: ObjectDetail ------------------------------------------
// Maps to pyramid.data_model.tactical.ObjectDetail.

struct ObjectDetail {
    Identifier        id;
    Identifier        source_id;
    GeodeticPosition  position;
    double            quality    = 0.0;
    bool              has_quality = false;
    double            course_rad = 0.0;
    bool              has_course = false;
    double            speed_mps  = 0.0;
    bool              has_speed  = false;
    double            length_m   = 0.0;
    bool              has_length = false;
    StandardIdentity  identity   = StandardIdentity::Unspecified;
    BattleDimension   dimension  = BattleDimension::Unspecified;
    ObjectSource      obj_source = ObjectSource::Unspecified;
};

// -- tactical.proto: ObjectMatch -------------------------------------------
// Reference type — entity ID pair.

struct ObjectMatch {
    Identifier match_id;            // common.Entity.id
    Identifier matching_object_id;  // the correlated tactical object
};

// -- tactical.proto: ObjectInterestRequirement -----------------------------
// Maps to pyramid.data_model.tactical.ObjectInterestRequirement.

struct ObjectInterestRequirement {
    Identifier                    id;
    std::optional<ObjectSource>   source;
    DataPolicy                    policy = DataPolicy::Unspecified;
    GeodeticPosition              center;
    double                        radius_m = 0.0;
    std::vector<BattleDimension>  dimensions;
};

// -- tactical.proto: ObjectEvidenceRequirement -----------------------------
// Maps to pyramid.data_model.tactical.ObjectEvidenceRequirement.

struct ObjectEvidenceRequirement {
    Identifier                    id;
    DataPolicy                    policy = DataPolicy::Unspecified;
    GeodeticPosition              center;
    double                        radius_m = 0.0;
    std::vector<BattleDimension>  dimensions;
};

} // namespace pyramid::services::tactical_objects
