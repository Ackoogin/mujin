// Auto-generated types header
// Generated from: common.proto by generate_bindings.py (types)
// Namespace: pyramid::data_model::common
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "pyramid_data_model_base_types.hpp"

namespace pyramid::data_model::common {


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

} // namespace pyramid::data_model::common
