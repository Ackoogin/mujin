// Auto-generated types header
// Generated from: proto/pyramid/data_model by cpp_service_generator.py --types
// Namespace: pyramid::data_model
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace pyramid::data_model {

using Angle = double;
using Length = double;
using Timestamp = double;
using Identifier = std::string;
using Speed = double;
using Percentage = double;

enum class Feasibility : int {
    Unspecified = 0,
    Feasible = 1,
    NotFeasible = 2,
    PartiallyFeasible = 3,
    Pending = 4,
};

enum class Progress : int {
    Unspecified = 0,
    NotStarted = 1,
    InProgress = 2,
    Completed = 3,
    Cancelled = 4,
    Failed = 5,
};

enum class StandardIdentity : int {
    Unspecified = 0,
    Unknown = 1,
    Friendly = 2,
    Hostile = 3,
    Suspect = 4,
    Neutral = 5,
    Pending = 6,
    Joker = 7,
    Faker = 8,
    AssumedFriendly = 9,
};

enum class BattleDimension : int {
    Unspecified = 0,
    Ground = 1,
    Subsurface = 2,
    SeaSurface = 3,
    Air = 4,
    Unknown = 5,
};

enum class DataPolicy : int {
    Unspecified = 0,
    Query = 1,
    Obtain = 2,
};

enum class ObjectSource : int {
    Unspecified = 0,
    Radar = 1,
    Local = 2,
};

struct GeodeticPosition {
    double latitude = 0.0;
    double longitude = 0.0;
};

struct PolyArea {
    std::vector<GeodeticPosition> points = {};
};

struct Entity {
    std::optional<double> update_time;  // optional
    std::string id = {};  // optional
    std::string source = {};  // optional
};

struct Achievement {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    Progress status = Progress::Unspecified;
    std::optional<double> quality;  // optional
    Feasibility achieveability = Feasibility::Unspecified;
};

struct Requirement {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    Achievement status = {};
};

struct Contraint {
    std::string name = {};
    int32_t value = 0;
};

struct Capability {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    bool availability = false;
    std::string name = {};
    std::vector<Contraint> contraint = {};
};

struct CircleArea {
    GeodeticPosition position = {};
    double radius = 0.0;
};

struct Point {
    GeodeticPosition position = {};
};

struct Ack {
    bool success = false;
};
constexpr Ack kAckOk{ true  };
constexpr Ack kAckFail{ false };

struct Query {
    std::vector<std::string> id = {};
    std::optional<bool> one_shot;  // optional
};

struct ObjectDetail {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string entity_source = {};  // from Entity  // optional
    std::vector<ObjectSource> source = {};
    GeodeticPosition position = {};
    double creation_time = 0.0;
    std::optional<double> quality;  // optional
    std::optional<double> course;  // optional
    std::optional<double> speed;  // optional
    std::optional<double> length;  // optional
    StandardIdentity identity = StandardIdentity::Unspecified;
    BattleDimension dimension = BattleDimension::Unspecified;
};

struct ObjectEvidenceRequirement {
    Entity base = {};  // from Requirement
    Achievement status = {};  // from Requirement
    DataPolicy policy = DataPolicy::Unspecified;
    std::vector<BattleDimension> dimension = {};
    // oneof location
    std::optional<PolyArea> poly_area;
    std::optional<CircleArea> circle_area;
    std::optional<Point> point;
};

struct ObjectInterestRequirement {
    Entity base = {};  // from Requirement
    Achievement status = {};  // from Requirement
    std::optional<ObjectSource> source;  // optional
    DataPolicy policy = DataPolicy::Unspecified;
    std::vector<BattleDimension> dimension = {};
    // oneof location
    std::optional<PolyArea> poly_area;
    std::optional<CircleArea> circle_area;
    std::optional<Point> point;
};

struct ObjectMatch {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string matching_object_id = {};
};

} // namespace pyramid::data_model
