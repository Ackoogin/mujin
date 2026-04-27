// Auto-generated types header
// Generated from: autonomy.proto by generate_bindings.py (types)
// Namespace: pyramid::data_model::autonomy
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "pyramid_data_model_base_types.hpp"
#include "pyramid_data_model_common_types.hpp"

namespace pyramid::data_model::autonomy {


enum class FactAuthorityLevel : int {
    Unspecified = 0,
    Believed = 1,
    Confirmed = 2,
};

enum class ExecutionState : int {
    Unspecified = 0,
    Accepted = 1,
    Executing = 2,
    WaitingForComponents = 3,
    Achieved = 4,
    Failed = 5,
    Cancelled = 6,
};

enum class RequirementPlacementOperation : int {
    Unspecified = 0,
    CreateRequirement = 1,
    ReadRequirement = 2,
    UpdateRequirement = 3,
    DeleteRequirement = 4,
    ReadProduct = 5,
    ReadCapability = 6,
};

struct RequirementReference {
    std::string requirement_id = {};
    std::string component_name = {};
    std::string service_name = {};
    std::string type_name = {};
};

struct AgentState {
    std::string agent_id = {};
    std::string agent_type = {};
    bool available = false;
};

struct PlanningPolicy {
    uint32_t max_replans = 0;
    bool enable_replanning = false;
};

struct PlanningGoal {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string name = {};
    // oneof goal
    std::optional<RequirementReference> requirement;
    std::optional<std::string> expression;
};

struct ExecutionPolicy {
    uint32_t max_replans = 0;
    bool enable_replanning = false;
    uint32_t max_concurrent_placements = 0;
};

struct PlanningRequirement {
    pyramid::data_model::common::Entity base = {};  // from Requirement
    pyramid::data_model::common::Achievement status = {};  // from Requirement
    std::vector<RequirementReference> upstream_requirement = {};
    std::vector<PlanningGoal> goal = {};
    PlanningPolicy policy = {};
    std::vector<AgentState> available_agents = {};
};

struct ExecutionRequirement {
    pyramid::data_model::common::Entity base = {};  // from Requirement
    pyramid::data_model::common::Achievement status = {};  // from Requirement
    std::vector<RequirementReference> upstream_requirement = {};
    std::string plan_id = {};
    ExecutionPolicy policy = {};
    std::vector<AgentState> available_agents = {};
    std::string planning_requirement_id = {};  // optional
};

struct WorldFactUpdate {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string entity_source = {};  // from Entity  // optional
    std::string key = {};
    bool value = false;
    std::string source = {};
    FactAuthorityLevel authority = FactAuthorityLevel::Unspecified;
};

struct StateUpdate {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::vector<WorldFactUpdate> fact_update = {};
};

struct Capabilities {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string backend_id = {};
    bool supports_planning_requirements = false;
    bool supports_execution_requirements = false;
    bool supports_approved_plan_execution = false;
    bool supports_replanning = false;
    bool supports_typed_component_requirement_placement = false;
    bool supports_state_update_ingress = false;
};

struct PlannedComponentInteraction {
    std::string target_component = {};
    std::string target_service = {};
    std::string target_type = {};
    RequirementPlacementOperation operation = RequirementPlacementOperation::Unspecified;
};

struct PlanStep {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    uint32_t sequence_number = 0;
    std::string action_name = {};
    std::string signature = {};
    std::vector<PlannedComponentInteraction> interaction = {};
};

struct Plan {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string planning_requirement_id = {};
    std::string backend_id = {};
    uint64_t world_version = 0;
    uint32_t replan_count = 0;
    bool plan_success = false;
    double solve_time_ms = 0.0;
    std::vector<PlanStep> step = {};
    std::string compiled_bt_xml = {};
    std::optional<double> predicted_quality;  // optional
};

struct RequirementPlacement {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string execution_requirement_id = {};
    std::string planning_requirement_id = {};
    std::string plan_id = {};
    std::string plan_step_id = {};
    std::string target_component = {};
    std::string target_service = {};
    std::string target_type = {};
    RequirementPlacementOperation operation = RequirementPlacementOperation::Unspecified;
    std::string target_requirement_id = {};
    std::vector<std::string> related_entity_id = {};
    pyramid::data_model::common::Progress progress = pyramid::data_model::common::Progress::Unspecified;
};

struct ExecutionRun {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string execution_requirement_id = {};
    std::string planning_requirement_id = {};
    std::string plan_id = {};
    ExecutionState state = ExecutionState::Unspecified;
    pyramid::data_model::common::Achievement achievement = {};
    uint32_t replan_count = 0;
    std::vector<RequirementPlacement> outstanding_placement = {};
};

} // namespace pyramid::data_model::autonomy
