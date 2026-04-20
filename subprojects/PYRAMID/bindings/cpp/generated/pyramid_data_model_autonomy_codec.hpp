// Auto-generated data model JSON codec header
// Generated from: autonomy.proto by generate_bindings.py (codec)
// Namespace: pyramid::data_model::autonomy
#pragma once

#include "pyramid_data_model_autonomy_types.hpp"
#include <string>

namespace pyramid::data_model::autonomy {

// Enum string converters
std::string toString(FactAuthorityLevel v);
FactAuthorityLevel factAuthorityLevelFromString(const std::string& s);
std::string toString(PlanningExecutionMode v);
PlanningExecutionMode planningExecutionModeFromString(const std::string& s);
std::string toString(PlanningExecutionState v);
PlanningExecutionState planningExecutionStateFromString(const std::string& s);
std::string toString(RequirementPlacementOperation v);
RequirementPlacementOperation requirementPlacementOperationFromString(const std::string& s);

// JSON codec
std::string toJson(const RequirementReference& msg);
RequirementReference fromJson(const std::string& s, RequirementReference* /*tag*/ = nullptr);
std::string toJson(const AgentState& msg);
AgentState fromJson(const std::string& s, AgentState* /*tag*/ = nullptr);
std::string toJson(const PlanningPolicy& msg);
PlanningPolicy fromJson(const std::string& s, PlanningPolicy* /*tag*/ = nullptr);
std::string toJson(const PlanningGoal& msg);
PlanningGoal fromJson(const std::string& s, PlanningGoal* /*tag*/ = nullptr);
std::string toJson(const PlanningExecutionRequirement& msg);
PlanningExecutionRequirement fromJson(const std::string& s, PlanningExecutionRequirement* /*tag*/ = nullptr);
std::string toJson(const WorldFactUpdate& msg);
WorldFactUpdate fromJson(const std::string& s, WorldFactUpdate* /*tag*/ = nullptr);
std::string toJson(const StateUpdate& msg);
StateUpdate fromJson(const std::string& s, StateUpdate* /*tag*/ = nullptr);
std::string toJson(const Capabilities& msg);
Capabilities fromJson(const std::string& s, Capabilities* /*tag*/ = nullptr);
std::string toJson(const PlannedComponentInteraction& msg);
PlannedComponentInteraction fromJson(const std::string& s, PlannedComponentInteraction* /*tag*/ = nullptr);
std::string toJson(const PlanStep& msg);
PlanStep fromJson(const std::string& s, PlanStep* /*tag*/ = nullptr);
std::string toJson(const Plan& msg);
Plan fromJson(const std::string& s, Plan* /*tag*/ = nullptr);
std::string toJson(const RequirementPlacement& msg);
RequirementPlacement fromJson(const std::string& s, RequirementPlacement* /*tag*/ = nullptr);
std::string toJson(const ExecutionRun& msg);
ExecutionRun fromJson(const std::string& s, ExecutionRun* /*tag*/ = nullptr);

} // namespace pyramid::data_model::autonomy
