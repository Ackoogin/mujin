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

enum class PlanningExecutionMode : int {
    Unspecified = 0,
    PlanAndExecute = 1,
    PlanOnly = 2,
    ExecuteApprovedPlan = 3,
};

enum class PlanningExecutionState : int {
    Unspecified = 0,
    Accepted = 1,
    Planning = 2,
    Executing = 3,
    WaitingForComponents = 4,
    Achieved = 5,
    Failed = 6,
    Cancelled = 7,
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
    uint32_t max_concurrent_placements = 0;
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

struct PlanningExecutionRequirement {
    pyramid::data_model::common::Entity base = {};  // from Requirement
    pyramid::data_model::common::Achievement status = {};  // from Requirement
    std::vector<RequirementReference> upstream_requirement = {};
    std::vector<PlanningGoal> goal = {};
    PlanningPolicy policy = {};
    std::vector<AgentState> available_agents = {};
    PlanningExecutionMode mode = PlanningExecutionMode::Unspecified;
    std::string approved_plan_id = {};  // optional
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
    bool supports_plan_only = false;
    bool supports_plan_and_execute = false;
    bool supports_execute_approved_plan = false;
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
    std::string planning_execution_requirement_id = {};
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
    std::string planning_execution_requirement_id = {};
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
    std::string planning_execution_requirement_id = {};
    std::string plan_id = {};
    PlanningExecutionState state = PlanningExecutionState::Unspecified;
    pyramid::data_model::common::Achievement achievement = {};
    uint32_t replan_count = 0;
    std::vector<RequirementPlacement> outstanding_placement = {};
};

} // namespace pyramid::data_model::autonomy
