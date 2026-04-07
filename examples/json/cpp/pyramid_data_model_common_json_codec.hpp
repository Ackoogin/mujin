// Auto-generated JSON codec — do not edit
// Backend: json | Namespace: pyramid::data_model::common::json_codec
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace pyramid::data_model::common::json_codec {

enum class Feasibility {
    Unspecified = 0,
    Feasible = 1,
    NotFeasible = 2,
    PartiallyFeasible = 3,
    Pending = 4,
};

std::string toString(Feasibility v);
Feasibility feasibilityFromString(const std::string& s);

enum class Progress {
    Unspecified = 0,
    NotStarted = 1,
    InProgress = 2,
    Completed = 3,
    Cancelled = 4,
    Failed = 5,
};

std::string toString(Progress v);
Progress progressFromString(const std::string& s);

enum class StandardIdentity {
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

std::string toString(StandardIdentity v);
StandardIdentity standardIdentityFromString(const std::string& s);

enum class BattleDimension {
    Unspecified = 0,
    Ground = 1,
    Subsurface = 2,
    SeaSurface = 3,
    Air = 4,
    Unknown = 5,
};

std::string toString(BattleDimension v);
BattleDimension battleDimensionFromString(const std::string& s);

enum class DataPolicy {
    Unspecified = 0,
    Query = 1,
    Obtain = 2,
};

std::string toString(DataPolicy v);
DataPolicy dataPolicyFromString(const std::string& s);

struct GeodeticPosition {
    Angle latitude = {};
    Angle longitude = {};
};

struct PolyArea {
    std::vector<GeodeticPosition> points = {};
};

struct Achievement {
    Entity base = {};
    Progress status = Progress::Unspecified;
    Percentage quality = {};
    Feasibility achieveability = Feasibility::Unspecified;
};

struct Requirement {
    Entity base = {};
    Achievement status = {};
};

struct Capability {
    Entity base = {};
    bool availability = false;
    std::string name = {};
    std::vector<Contraint> contraint = {};
};

struct Entity {
    Timestamp update_time = {};
    Identifier id = {};
    Identifier source = {};
};

struct CircleArea {
    GeodeticPosition position = {};
    Length radius = {};
};

struct Point {
    GeodeticPosition position = {};
};

struct Contraint {
    std::string name = {};
    int32_t value = 0;
};

struct Ack {
    bool success = false;
};

struct Query {
    std::vector<Identifier> id = {};
    bool one_shot = false;
};

std::string toJson(const GeodeticPosition& msg);
GeodeticPosition geodeticPositionFromJson(const std::string& s);
std::string toJson(const PolyArea& msg);
PolyArea polyAreaFromJson(const std::string& s);
std::string toJson(const Achievement& msg);
Achievement achievementFromJson(const std::string& s);
std::string toJson(const Requirement& msg);
Requirement requirementFromJson(const std::string& s);
std::string toJson(const Capability& msg);
Capability capabilityFromJson(const std::string& s);
std::string toJson(const Entity& msg);
Entity entityFromJson(const std::string& s);
std::string toJson(const CircleArea& msg);
CircleArea circleAreaFromJson(const std::string& s);
std::string toJson(const Point& msg);
Point pointFromJson(const std::string& s);
std::string toJson(const Contraint& msg);
Contraint contraintFromJson(const std::string& s);
std::string toJson(const Ack& msg);
Ack ackFromJson(const std::string& s);
std::string toJson(const Query& msg);
Query queryFromJson(const std::string& s);

} // namespace pyramid::data_model::common::json_codec
