#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_autonomy_cabi.h"
#include "pyramid_data_model_base_cabi_marshal.hpp"
#include "pyramid_data_model_common_cabi_marshal.hpp"

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::RequirementReference& in, pyramid_RequirementReference_c* out);
void from_c(const pyramid_RequirementReference_c* in, pyramid::domain_model::RequirementReference& out);

void to_c(const pyramid::domain_model::AgentState& in, pyramid_AgentState_c* out);
void from_c(const pyramid_AgentState_c* in, pyramid::domain_model::AgentState& out);

void to_c(const pyramid::domain_model::PlanningPolicy& in, pyramid_PlanningPolicy_c* out);
void from_c(const pyramid_PlanningPolicy_c* in, pyramid::domain_model::PlanningPolicy& out);

void to_c(const pyramid::domain_model::PlanningGoal& in, pyramid_PlanningGoal_c* out);
void from_c(const pyramid_PlanningGoal_c* in, pyramid::domain_model::PlanningGoal& out);

void to_c(const pyramid::domain_model::ExecutionPolicy& in, pyramid_ExecutionPolicy_c* out);
void from_c(const pyramid_ExecutionPolicy_c* in, pyramid::domain_model::ExecutionPolicy& out);

void to_c(const pyramid::domain_model::PlanningRequirement& in, pyramid_PlanningRequirement_c* out);
void from_c(const pyramid_PlanningRequirement_c* in, pyramid::domain_model::PlanningRequirement& out);

void to_c(const pyramid::domain_model::ExecutionRequirement& in, pyramid_ExecutionRequirement_c* out);
void from_c(const pyramid_ExecutionRequirement_c* in, pyramid::domain_model::ExecutionRequirement& out);

void to_c(const pyramid::domain_model::WorldFactUpdate& in, pyramid_WorldFactUpdate_c* out);
void from_c(const pyramid_WorldFactUpdate_c* in, pyramid::domain_model::WorldFactUpdate& out);

void to_c(const pyramid::domain_model::StateUpdate& in, pyramid_StateUpdate_c* out);
void from_c(const pyramid_StateUpdate_c* in, pyramid::domain_model::StateUpdate& out);

void to_c(const pyramid::domain_model::Capabilities& in, pyramid_Capabilities_c* out);
void from_c(const pyramid_Capabilities_c* in, pyramid::domain_model::Capabilities& out);

void to_c(const pyramid::domain_model::PlannedComponentInteraction& in, pyramid_PlannedComponentInteraction_c* out);
void from_c(const pyramid_PlannedComponentInteraction_c* in, pyramid::domain_model::PlannedComponentInteraction& out);

void to_c(const pyramid::domain_model::PlanStep& in, pyramid_PlanStep_c* out);
void from_c(const pyramid_PlanStep_c* in, pyramid::domain_model::PlanStep& out);

void to_c(const pyramid::domain_model::Plan& in, pyramid_Plan_c* out);
void from_c(const pyramid_Plan_c* in, pyramid::domain_model::Plan& out);

void to_c(const pyramid::domain_model::RequirementPlacement& in, pyramid_RequirementPlacement_c* out);
void from_c(const pyramid_RequirementPlacement_c* in, pyramid::domain_model::RequirementPlacement& out);

void to_c(const pyramid::domain_model::ExecutionRun& in, pyramid_ExecutionRun_c* out);
void from_c(const pyramid_ExecutionRun_c* in, pyramid::domain_model::ExecutionRun& out);

} // namespace pyramid::cabi
