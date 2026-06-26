#ifndef PYRAMID_DATA_MODEL_AUTONOMY_CABI_H
#define PYRAMID_DATA_MODEL_AUTONOMY_CABI_H

#include <stdint.h>
#include "pyramid_datamodel_cabi.h"
#include "pyramid_data_model_base_cabi.h"
#include "pyramid_data_model_common_cabi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pyramid_RequirementReference_c {
  pyramid_str_t requirement_id;
  pyramid_str_t component_name;
  pyramid_str_t service_name;
  pyramid_str_t type_name;
} pyramid_RequirementReference_c;

typedef struct pyramid_AgentState_c {
  pyramid_str_t agent_id;
  pyramid_str_t agent_type;
  uint8_t available;
} pyramid_AgentState_c;

typedef struct pyramid_PlanningPolicy_c {
  uint32_t max_replans;
  uint8_t enable_replanning;
} pyramid_PlanningPolicy_c;

typedef struct pyramid_PlanningGoal_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_str_t name;
  uint8_t has_requirement;
  pyramid_RequirementReference_c requirement;
  uint8_t has_expression;
  pyramid_str_t expression;
} pyramid_PlanningGoal_c;

typedef struct pyramid_ExecutionPolicy_c {
  uint32_t max_replans;
  uint8_t enable_replanning;
  uint32_t max_concurrent_placements;
} pyramid_ExecutionPolicy_c;

typedef struct pyramid_PlanningRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  pyramid_slice_t upstream_requirement;
  pyramid_slice_t goal;
  pyramid_PlanningPolicy_c policy;
  pyramid_slice_t available_agents;
} pyramid_PlanningRequirement_c;

typedef struct pyramid_ExecutionRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  pyramid_slice_t upstream_requirement;
  pyramid_str_t plan_id;
  pyramid_ExecutionPolicy_c policy;
  pyramid_slice_t available_agents;
  pyramid_str_t planning_requirement_id;
} pyramid_ExecutionRequirement_c;

typedef struct pyramid_WorldFactUpdate_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t entity_source;  /* from Entity */
  pyramid_str_t key;
  uint8_t value;
  pyramid_str_t source;
  int32_t authority;
} pyramid_WorldFactUpdate_c;

typedef struct pyramid_StateUpdate_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_slice_t fact_update;
} pyramid_StateUpdate_c;

typedef struct pyramid_Capabilities_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_str_t backend_id;
  uint8_t supports_planning_requirements;
  uint8_t supports_execution_requirements;
  uint8_t supports_approved_plan_execution;
  uint8_t supports_replanning;
  uint8_t supports_typed_component_requirement_placement;
  uint8_t supports_state_update_ingress;
} pyramid_Capabilities_c;

typedef struct pyramid_PlannedComponentInteraction_c {
  pyramid_str_t target_component;
  pyramid_str_t target_service;
  pyramid_str_t target_type;
  int32_t operation;
} pyramid_PlannedComponentInteraction_c;

typedef struct pyramid_PlanStep_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  uint32_t sequence_number;
  pyramid_str_t action_name;
  pyramid_str_t signature;
  pyramid_slice_t interaction;
} pyramid_PlanStep_c;

typedef struct pyramid_Plan_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_str_t planning_requirement_id;
  pyramid_str_t backend_id;
  uint64_t world_version;
  uint32_t replan_count;
  uint8_t plan_success;
  double solve_time_ms;
  pyramid_slice_t step;
  pyramid_str_t compiled_bt_xml;
  uint8_t has_predicted_quality;
  double predicted_quality;
} pyramid_Plan_c;

typedef struct pyramid_RequirementPlacement_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_str_t execution_requirement_id;
  pyramid_str_t planning_requirement_id;
  pyramid_str_t plan_id;
  pyramid_str_t plan_step_id;
  pyramid_str_t target_component;
  pyramid_str_t target_service;
  pyramid_str_t target_type;
  int32_t operation;
  pyramid_str_t target_requirement_id;
  pyramid_slice_t related_entity_id;
  int32_t progress;
} pyramid_RequirementPlacement_c;

typedef struct pyramid_ExecutionRun_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_str_t execution_requirement_id;
  pyramid_str_t planning_requirement_id;
  pyramid_str_t plan_id;
  int32_t state;
  pyramid_Achievement_c achievement;
  uint32_t replan_count;
  pyramid_slice_t outstanding_placement;
} pyramid_ExecutionRun_c;

void pyramid_RequirementReference_c_free(pyramid_RequirementReference_c* value);
void pyramid_AgentState_c_free(pyramid_AgentState_c* value);
void pyramid_PlanningPolicy_c_free(pyramid_PlanningPolicy_c* value);
void pyramid_PlanningGoal_c_free(pyramid_PlanningGoal_c* value);
void pyramid_ExecutionPolicy_c_free(pyramid_ExecutionPolicy_c* value);
void pyramid_PlanningRequirement_c_free(pyramid_PlanningRequirement_c* value);
void pyramid_ExecutionRequirement_c_free(pyramid_ExecutionRequirement_c* value);
void pyramid_WorldFactUpdate_c_free(pyramid_WorldFactUpdate_c* value);
void pyramid_StateUpdate_c_free(pyramid_StateUpdate_c* value);
void pyramid_Capabilities_c_free(pyramid_Capabilities_c* value);
void pyramid_PlannedComponentInteraction_c_free(pyramid_PlannedComponentInteraction_c* value);
void pyramid_PlanStep_c_free(pyramid_PlanStep_c* value);
void pyramid_Plan_c_free(pyramid_Plan_c* value);
void pyramid_RequirementPlacement_c_free(pyramid_RequirementPlacement_c* value);
void pyramid_ExecutionRun_c_free(pyramid_ExecutionRun_c* value);

#ifdef __cplusplus
}
#endif

#endif /* PYRAMID_DATA_MODEL_AUTONOMY_CABI_H */
