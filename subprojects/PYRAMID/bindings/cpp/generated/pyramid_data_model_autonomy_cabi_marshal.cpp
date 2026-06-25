// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_data_model_autonomy_cabi_marshal.hpp"
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

void dup_str(pyramid_str_t& out, const std::string& in) {
  if (in.empty()) {
    out.ptr = nullptr;
    out.len = 0;
    return;
  }
  out.len = static_cast<uint32_t>(in.size());
  out.ptr = static_cast<const char*>(std::malloc(out.len));
  std::memcpy(const_cast<char*>(out.ptr), in.data(), out.len);
}

void free_str(pyramid_str_t& s) {
  if (s.ptr) {
    std::free(const_cast<char*>(s.ptr));
    s.ptr = nullptr;
    s.len = 0;
  }
}

} // namespace

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::RequirementReference& in, pyramid_RequirementReference_c* out) {
  std::memset(out, 0, sizeof(*out));
  dup_str(out->requirement_id, in.requirement_id);
  dup_str(out->component_name, in.component_name);
  dup_str(out->service_name, in.service_name);
  dup_str(out->type_name, in.type_name);
}

void from_c(const pyramid_RequirementReference_c* in, pyramid::domain_model::RequirementReference& out) {
  if (in->requirement_id.ptr && in->requirement_id.len > 0) {
    out.requirement_id.assign(in->requirement_id.ptr, in->requirement_id.len);
  } else {
    out.requirement_id.clear();
  }
  if (in->component_name.ptr && in->component_name.len > 0) {
    out.component_name.assign(in->component_name.ptr, in->component_name.len);
  } else {
    out.component_name.clear();
  }
  if (in->service_name.ptr && in->service_name.len > 0) {
    out.service_name.assign(in->service_name.ptr, in->service_name.len);
  } else {
    out.service_name.clear();
  }
  if (in->type_name.ptr && in->type_name.len > 0) {
    out.type_name.assign(in->type_name.ptr, in->type_name.len);
  } else {
    out.type_name.clear();
  }
}

void _free_RequirementReference(pyramid_RequirementReference_c* value) {
  if (!value) {
    return;
  }
  free_str(value->requirement_id);
  free_str(value->component_name);
  free_str(value->service_name);
  free_str(value->type_name);
}

void to_c(const pyramid::domain_model::AgentState& in, pyramid_AgentState_c* out) {
  std::memset(out, 0, sizeof(*out));
  dup_str(out->agent_id, in.agent_id);
  dup_str(out->agent_type, in.agent_type);
  out->available = in.available ? 1u : 0u;
}

void from_c(const pyramid_AgentState_c* in, pyramid::domain_model::AgentState& out) {
  if (in->agent_id.ptr && in->agent_id.len > 0) {
    out.agent_id.assign(in->agent_id.ptr, in->agent_id.len);
  } else {
    out.agent_id.clear();
  }
  if (in->agent_type.ptr && in->agent_type.len > 0) {
    out.agent_type.assign(in->agent_type.ptr, in->agent_type.len);
  } else {
    out.agent_type.clear();
  }
  out.available = in->available != 0;
}

void _free_AgentState(pyramid_AgentState_c* value) {
  if (!value) {
    return;
  }
  free_str(value->agent_id);
  free_str(value->agent_type);
}

void to_c(const pyramid::domain_model::PlanningPolicy& in, pyramid_PlanningPolicy_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->max_replans = in.max_replans;
  out->enable_replanning = in.enable_replanning ? 1u : 0u;
}

void from_c(const pyramid_PlanningPolicy_c* in, pyramid::domain_model::PlanningPolicy& out) {
  out.max_replans = in->max_replans;
  out.enable_replanning = in->enable_replanning != 0;
}

void _free_PlanningPolicy(pyramid_PlanningPolicy_c* value) {
  if (!value) {
    return;
  }
}

void to_c(const pyramid::domain_model::PlanningGoal& in, pyramid_PlanningGoal_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  dup_str(out->name, in.name);
  out->has_requirement = in.requirement.has_value() ? 1u : 0u;
  if (in.requirement) {
    to_c(*in.requirement, &out->requirement);
  }
  out->has_expression = in.expression.has_value() ? 1u : 0u;
  if (in.expression) {
    dup_str(out->expression, *in.expression);
  }
}

void from_c(const pyramid_PlanningGoal_c* in, pyramid::domain_model::PlanningGoal& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  if (in->name.ptr && in->name.len > 0) {
    out.name.assign(in->name.ptr, in->name.len);
  } else {
    out.name.clear();
  }
  if (in->has_requirement) {
    out.requirement.emplace();
    from_c(&in->requirement, *out.requirement);
  } else {
    out.requirement = tl::nullopt;
  }
  if (in->has_expression) {
    if (in->expression.ptr && in->expression.len > 0) {
      out.expression = std::string(in->expression.ptr, in->expression.len);
    } else {
      out.expression = std::string();
    }
  } else {
    out.expression = tl::nullopt;
  }
}

void _free_PlanningGoal(pyramid_PlanningGoal_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->name);
  if (value->has_requirement) {
    pyramid_RequirementReference_c_free(&value->requirement);
    value->has_requirement = 0;
  }
  free_str(value->expression);
  value->has_expression = 0;
}

void to_c(const pyramid::domain_model::ExecutionPolicy& in, pyramid_ExecutionPolicy_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->max_replans = in.max_replans;
  out->enable_replanning = in.enable_replanning ? 1u : 0u;
  out->max_concurrent_placements = in.max_concurrent_placements;
}

void from_c(const pyramid_ExecutionPolicy_c* in, pyramid::domain_model::ExecutionPolicy& out) {
  out.max_replans = in->max_replans;
  out.enable_replanning = in->enable_replanning != 0;
  out.max_concurrent_placements = in->max_concurrent_placements;
}

void _free_ExecutionPolicy(pyramid_ExecutionPolicy_c* value) {
  if (!value) {
    return;
  }
}

void to_c(const pyramid::domain_model::PlanningRequirement& in, pyramid_PlanningRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  {
    const auto count = in.upstream_requirement.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_RequirementReference_c*>(std::malloc(count * sizeof(pyramid_RequirementReference_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.upstream_requirement[i], &arr[i]);
      }
      out->upstream_requirement.ptr = arr;
      out->upstream_requirement.len = static_cast<uint32_t>(count);
    }
  }
  {
    const auto count = in.goal.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_PlanningGoal_c*>(std::malloc(count * sizeof(pyramid_PlanningGoal_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.goal[i], &arr[i]);
      }
      out->goal.ptr = arr;
      out->goal.len = static_cast<uint32_t>(count);
    }
  }
  to_c(in.policy, &out->policy);
  {
    const auto count = in.available_agents.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_AgentState_c*>(std::malloc(count * sizeof(pyramid_AgentState_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.available_agents[i], &arr[i]);
      }
      out->available_agents.ptr = arr;
      out->available_agents.len = static_cast<uint32_t>(count);
    }
  }
}

void from_c(const pyramid_PlanningRequirement_c* in, pyramid::domain_model::PlanningRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  out.upstream_requirement.clear();
  if (in->upstream_requirement.ptr && in->upstream_requirement.len > 0) {
    const auto* arr = static_cast<const pyramid_RequirementReference_c*>(in->upstream_requirement.ptr);
    out.upstream_requirement.reserve(in->upstream_requirement.len);
    for (uint32_t i = 0; i < in->upstream_requirement.len; ++i) {
      pyramid::domain_model::RequirementReference elem{};
      from_c(&arr[i], elem);
      out.upstream_requirement.push_back(std::move(elem));
    }
  }
  out.goal.clear();
  if (in->goal.ptr && in->goal.len > 0) {
    const auto* arr = static_cast<const pyramid_PlanningGoal_c*>(in->goal.ptr);
    out.goal.reserve(in->goal.len);
    for (uint32_t i = 0; i < in->goal.len; ++i) {
      pyramid::domain_model::PlanningGoal elem{};
      from_c(&arr[i], elem);
      out.goal.push_back(std::move(elem));
    }
  }
  from_c(&in->policy, out.policy);
  out.available_agents.clear();
  if (in->available_agents.ptr && in->available_agents.len > 0) {
    const auto* arr = static_cast<const pyramid_AgentState_c*>(in->available_agents.ptr);
    out.available_agents.reserve(in->available_agents.len);
    for (uint32_t i = 0; i < in->available_agents.len; ++i) {
      pyramid::domain_model::AgentState elem{};
      from_c(&arr[i], elem);
      out.available_agents.push_back(std::move(elem));
    }
  }
}

void _free_PlanningRequirement(pyramid_PlanningRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
  if (value->upstream_requirement.ptr) {
    auto* arr = static_cast<pyramid_RequirementReference_c*>(const_cast<void*>(value->upstream_requirement.ptr));
    for (uint32_t i = 0; i < value->upstream_requirement.len; ++i) {
      pyramid_RequirementReference_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->upstream_requirement.ptr));
    value->upstream_requirement.ptr = nullptr;
    value->upstream_requirement.len = 0;
  }
  if (value->goal.ptr) {
    auto* arr = static_cast<pyramid_PlanningGoal_c*>(const_cast<void*>(value->goal.ptr));
    for (uint32_t i = 0; i < value->goal.len; ++i) {
      pyramid_PlanningGoal_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->goal.ptr));
    value->goal.ptr = nullptr;
    value->goal.len = 0;
  }
  pyramid_PlanningPolicy_c_free(&value->policy);
  if (value->available_agents.ptr) {
    auto* arr = static_cast<pyramid_AgentState_c*>(const_cast<void*>(value->available_agents.ptr));
    for (uint32_t i = 0; i < value->available_agents.len; ++i) {
      pyramid_AgentState_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->available_agents.ptr));
    value->available_agents.ptr = nullptr;
    value->available_agents.len = 0;
  }
}

void to_c(const pyramid::domain_model::ExecutionRequirement& in, pyramid_ExecutionRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  {
    const auto count = in.upstream_requirement.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_RequirementReference_c*>(std::malloc(count * sizeof(pyramid_RequirementReference_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.upstream_requirement[i], &arr[i]);
      }
      out->upstream_requirement.ptr = arr;
      out->upstream_requirement.len = static_cast<uint32_t>(count);
    }
  }
  dup_str(out->plan_id, in.plan_id);
  to_c(in.policy, &out->policy);
  {
    const auto count = in.available_agents.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_AgentState_c*>(std::malloc(count * sizeof(pyramid_AgentState_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.available_agents[i], &arr[i]);
      }
      out->available_agents.ptr = arr;
      out->available_agents.len = static_cast<uint32_t>(count);
    }
  }
  dup_str(out->planning_requirement_id, in.planning_requirement_id);
}

void from_c(const pyramid_ExecutionRequirement_c* in, pyramid::domain_model::ExecutionRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  out.upstream_requirement.clear();
  if (in->upstream_requirement.ptr && in->upstream_requirement.len > 0) {
    const auto* arr = static_cast<const pyramid_RequirementReference_c*>(in->upstream_requirement.ptr);
    out.upstream_requirement.reserve(in->upstream_requirement.len);
    for (uint32_t i = 0; i < in->upstream_requirement.len; ++i) {
      pyramid::domain_model::RequirementReference elem{};
      from_c(&arr[i], elem);
      out.upstream_requirement.push_back(std::move(elem));
    }
  }
  if (in->plan_id.ptr && in->plan_id.len > 0) {
    out.plan_id.assign(in->plan_id.ptr, in->plan_id.len);
  } else {
    out.plan_id.clear();
  }
  from_c(&in->policy, out.policy);
  out.available_agents.clear();
  if (in->available_agents.ptr && in->available_agents.len > 0) {
    const auto* arr = static_cast<const pyramid_AgentState_c*>(in->available_agents.ptr);
    out.available_agents.reserve(in->available_agents.len);
    for (uint32_t i = 0; i < in->available_agents.len; ++i) {
      pyramid::domain_model::AgentState elem{};
      from_c(&arr[i], elem);
      out.available_agents.push_back(std::move(elem));
    }
  }
  if (in->planning_requirement_id.ptr && in->planning_requirement_id.len > 0) {
    out.planning_requirement_id.assign(in->planning_requirement_id.ptr, in->planning_requirement_id.len);
  } else {
    out.planning_requirement_id.clear();
  }
}

void _free_ExecutionRequirement(pyramid_ExecutionRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
  if (value->upstream_requirement.ptr) {
    auto* arr = static_cast<pyramid_RequirementReference_c*>(const_cast<void*>(value->upstream_requirement.ptr));
    for (uint32_t i = 0; i < value->upstream_requirement.len; ++i) {
      pyramid_RequirementReference_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->upstream_requirement.ptr));
    value->upstream_requirement.ptr = nullptr;
    value->upstream_requirement.len = 0;
  }
  free_str(value->plan_id);
  pyramid_ExecutionPolicy_c_free(&value->policy);
  if (value->available_agents.ptr) {
    auto* arr = static_cast<pyramid_AgentState_c*>(const_cast<void*>(value->available_agents.ptr));
    for (uint32_t i = 0; i < value->available_agents.len; ++i) {
      pyramid_AgentState_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->available_agents.ptr));
    value->available_agents.ptr = nullptr;
    value->available_agents.len = 0;
  }
  free_str(value->planning_requirement_id);
}

void to_c(const pyramid::domain_model::WorldFactUpdate& in, pyramid_WorldFactUpdate_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->entity_source, in.entity_source);
  dup_str(out->key, in.key);
  out->value = in.value ? 1u : 0u;
  dup_str(out->source, in.source);
  out->authority = static_cast<int32_t>(in.authority);
}

void from_c(const pyramid_WorldFactUpdate_c* in, pyramid::domain_model::WorldFactUpdate& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->entity_source.ptr && in->entity_source.len > 0) {
    out.entity_source.assign(in->entity_source.ptr, in->entity_source.len);
  } else {
    out.entity_source.clear();
  }
  if (in->key.ptr && in->key.len > 0) {
    out.key.assign(in->key.ptr, in->key.len);
  } else {
    out.key.clear();
  }
  out.value = in->value != 0;
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  out.authority = static_cast<pyramid::domain_model::FactAuthorityLevel>(in->authority);
}

void _free_WorldFactUpdate(pyramid_WorldFactUpdate_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->entity_source);
  free_str(value->key);
  free_str(value->source);
}

void to_c(const pyramid::domain_model::StateUpdate& in, pyramid_StateUpdate_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  {
    const auto count = in.fact_update.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_WorldFactUpdate_c*>(std::malloc(count * sizeof(pyramid_WorldFactUpdate_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.fact_update[i], &arr[i]);
      }
      out->fact_update.ptr = arr;
      out->fact_update.len = static_cast<uint32_t>(count);
    }
  }
}

void from_c(const pyramid_StateUpdate_c* in, pyramid::domain_model::StateUpdate& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  out.fact_update.clear();
  if (in->fact_update.ptr && in->fact_update.len > 0) {
    const auto* arr = static_cast<const pyramid_WorldFactUpdate_c*>(in->fact_update.ptr);
    out.fact_update.reserve(in->fact_update.len);
    for (uint32_t i = 0; i < in->fact_update.len; ++i) {
      pyramid::domain_model::WorldFactUpdate elem{};
      from_c(&arr[i], elem);
      out.fact_update.push_back(std::move(elem));
    }
  }
}

void _free_StateUpdate(pyramid_StateUpdate_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  if (value->fact_update.ptr) {
    auto* arr = static_cast<pyramid_WorldFactUpdate_c*>(const_cast<void*>(value->fact_update.ptr));
    for (uint32_t i = 0; i < value->fact_update.len; ++i) {
      pyramid_WorldFactUpdate_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->fact_update.ptr));
    value->fact_update.ptr = nullptr;
    value->fact_update.len = 0;
  }
}

void to_c(const pyramid::domain_model::Capabilities& in, pyramid_Capabilities_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  dup_str(out->backend_id, in.backend_id);
  out->supports_planning_requirements = in.supports_planning_requirements ? 1u : 0u;
  out->supports_execution_requirements = in.supports_execution_requirements ? 1u : 0u;
  out->supports_approved_plan_execution = in.supports_approved_plan_execution ? 1u : 0u;
  out->supports_replanning = in.supports_replanning ? 1u : 0u;
  out->supports_typed_component_requirement_placement = in.supports_typed_component_requirement_placement ? 1u : 0u;
  out->supports_state_update_ingress = in.supports_state_update_ingress ? 1u : 0u;
}

void from_c(const pyramid_Capabilities_c* in, pyramid::domain_model::Capabilities& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  if (in->backend_id.ptr && in->backend_id.len > 0) {
    out.backend_id.assign(in->backend_id.ptr, in->backend_id.len);
  } else {
    out.backend_id.clear();
  }
  out.supports_planning_requirements = in->supports_planning_requirements != 0;
  out.supports_execution_requirements = in->supports_execution_requirements != 0;
  out.supports_approved_plan_execution = in->supports_approved_plan_execution != 0;
  out.supports_replanning = in->supports_replanning != 0;
  out.supports_typed_component_requirement_placement = in->supports_typed_component_requirement_placement != 0;
  out.supports_state_update_ingress = in->supports_state_update_ingress != 0;
}

void _free_Capabilities(pyramid_Capabilities_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->backend_id);
}

void to_c(const pyramid::domain_model::PlannedComponentInteraction& in, pyramid_PlannedComponentInteraction_c* out) {
  std::memset(out, 0, sizeof(*out));
  dup_str(out->target_component, in.target_component);
  dup_str(out->target_service, in.target_service);
  dup_str(out->target_type, in.target_type);
  out->operation = static_cast<int32_t>(in.operation);
}

void from_c(const pyramid_PlannedComponentInteraction_c* in, pyramid::domain_model::PlannedComponentInteraction& out) {
  if (in->target_component.ptr && in->target_component.len > 0) {
    out.target_component.assign(in->target_component.ptr, in->target_component.len);
  } else {
    out.target_component.clear();
  }
  if (in->target_service.ptr && in->target_service.len > 0) {
    out.target_service.assign(in->target_service.ptr, in->target_service.len);
  } else {
    out.target_service.clear();
  }
  if (in->target_type.ptr && in->target_type.len > 0) {
    out.target_type.assign(in->target_type.ptr, in->target_type.len);
  } else {
    out.target_type.clear();
  }
  out.operation = static_cast<pyramid::domain_model::RequirementPlacementOperation>(in->operation);
}

void _free_PlannedComponentInteraction(pyramid_PlannedComponentInteraction_c* value) {
  if (!value) {
    return;
  }
  free_str(value->target_component);
  free_str(value->target_service);
  free_str(value->target_type);
}

void to_c(const pyramid::domain_model::PlanStep& in, pyramid_PlanStep_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  out->sequence_number = in.sequence_number;
  dup_str(out->action_name, in.action_name);
  dup_str(out->signature, in.signature);
  {
    const auto count = in.interaction.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_PlannedComponentInteraction_c*>(std::malloc(count * sizeof(pyramid_PlannedComponentInteraction_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.interaction[i], &arr[i]);
      }
      out->interaction.ptr = arr;
      out->interaction.len = static_cast<uint32_t>(count);
    }
  }
}

void from_c(const pyramid_PlanStep_c* in, pyramid::domain_model::PlanStep& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  out.sequence_number = in->sequence_number;
  if (in->action_name.ptr && in->action_name.len > 0) {
    out.action_name.assign(in->action_name.ptr, in->action_name.len);
  } else {
    out.action_name.clear();
  }
  if (in->signature.ptr && in->signature.len > 0) {
    out.signature.assign(in->signature.ptr, in->signature.len);
  } else {
    out.signature.clear();
  }
  out.interaction.clear();
  if (in->interaction.ptr && in->interaction.len > 0) {
    const auto* arr = static_cast<const pyramid_PlannedComponentInteraction_c*>(in->interaction.ptr);
    out.interaction.reserve(in->interaction.len);
    for (uint32_t i = 0; i < in->interaction.len; ++i) {
      pyramid::domain_model::PlannedComponentInteraction elem{};
      from_c(&arr[i], elem);
      out.interaction.push_back(std::move(elem));
    }
  }
}

void _free_PlanStep(pyramid_PlanStep_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->action_name);
  free_str(value->signature);
  if (value->interaction.ptr) {
    auto* arr = static_cast<pyramid_PlannedComponentInteraction_c*>(const_cast<void*>(value->interaction.ptr));
    for (uint32_t i = 0; i < value->interaction.len; ++i) {
      pyramid_PlannedComponentInteraction_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->interaction.ptr));
    value->interaction.ptr = nullptr;
    value->interaction.len = 0;
  }
}

void to_c(const pyramid::domain_model::Plan& in, pyramid_Plan_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  dup_str(out->planning_requirement_id, in.planning_requirement_id);
  dup_str(out->backend_id, in.backend_id);
  out->world_version = in.world_version;
  out->replan_count = in.replan_count;
  out->plan_success = in.plan_success ? 1u : 0u;
  out->solve_time_ms = in.solve_time_ms;
  {
    const auto count = in.step.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_PlanStep_c*>(std::malloc(count * sizeof(pyramid_PlanStep_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.step[i], &arr[i]);
      }
      out->step.ptr = arr;
      out->step.len = static_cast<uint32_t>(count);
    }
  }
  dup_str(out->compiled_bt_xml, in.compiled_bt_xml);
  out->has_predicted_quality = in.predicted_quality.has_value() ? 1u : 0u;
  if (in.predicted_quality) {
    out->predicted_quality = *in.predicted_quality;
  }
}

void from_c(const pyramid_Plan_c* in, pyramid::domain_model::Plan& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  if (in->planning_requirement_id.ptr && in->planning_requirement_id.len > 0) {
    out.planning_requirement_id.assign(in->planning_requirement_id.ptr, in->planning_requirement_id.len);
  } else {
    out.planning_requirement_id.clear();
  }
  if (in->backend_id.ptr && in->backend_id.len > 0) {
    out.backend_id.assign(in->backend_id.ptr, in->backend_id.len);
  } else {
    out.backend_id.clear();
  }
  out.world_version = in->world_version;
  out.replan_count = in->replan_count;
  out.plan_success = in->plan_success != 0;
  out.solve_time_ms = in->solve_time_ms;
  out.step.clear();
  if (in->step.ptr && in->step.len > 0) {
    const auto* arr = static_cast<const pyramid_PlanStep_c*>(in->step.ptr);
    out.step.reserve(in->step.len);
    for (uint32_t i = 0; i < in->step.len; ++i) {
      pyramid::domain_model::PlanStep elem{};
      from_c(&arr[i], elem);
      out.step.push_back(std::move(elem));
    }
  }
  if (in->compiled_bt_xml.ptr && in->compiled_bt_xml.len > 0) {
    out.compiled_bt_xml.assign(in->compiled_bt_xml.ptr, in->compiled_bt_xml.len);
  } else {
    out.compiled_bt_xml.clear();
  }
  if (in->has_predicted_quality) {
    out.predicted_quality = in->predicted_quality;
  } else {
    out.predicted_quality = tl::nullopt;
  }
}

void _free_Plan(pyramid_Plan_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->planning_requirement_id);
  free_str(value->backend_id);
  if (value->step.ptr) {
    auto* arr = static_cast<pyramid_PlanStep_c*>(const_cast<void*>(value->step.ptr));
    for (uint32_t i = 0; i < value->step.len; ++i) {
      pyramid_PlanStep_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->step.ptr));
    value->step.ptr = nullptr;
    value->step.len = 0;
  }
  free_str(value->compiled_bt_xml);
}

void to_c(const pyramid::domain_model::RequirementPlacement& in, pyramid_RequirementPlacement_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  dup_str(out->execution_requirement_id, in.execution_requirement_id);
  dup_str(out->planning_requirement_id, in.planning_requirement_id);
  dup_str(out->plan_id, in.plan_id);
  dup_str(out->plan_step_id, in.plan_step_id);
  dup_str(out->target_component, in.target_component);
  dup_str(out->target_service, in.target_service);
  dup_str(out->target_type, in.target_type);
  out->operation = static_cast<int32_t>(in.operation);
  dup_str(out->target_requirement_id, in.target_requirement_id);
  {
    const auto count = in.related_entity_id.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_str_t*>(std::malloc(count * sizeof(pyramid_str_t)));
      for (size_t i = 0; i < count; ++i) {
        dup_str(arr[i], in.related_entity_id[i]);
      }
      out->related_entity_id.ptr = arr;
      out->related_entity_id.len = static_cast<uint32_t>(count);
    }
  }
  out->progress = static_cast<int32_t>(in.progress);
}

void from_c(const pyramid_RequirementPlacement_c* in, pyramid::domain_model::RequirementPlacement& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  if (in->execution_requirement_id.ptr && in->execution_requirement_id.len > 0) {
    out.execution_requirement_id.assign(in->execution_requirement_id.ptr, in->execution_requirement_id.len);
  } else {
    out.execution_requirement_id.clear();
  }
  if (in->planning_requirement_id.ptr && in->planning_requirement_id.len > 0) {
    out.planning_requirement_id.assign(in->planning_requirement_id.ptr, in->planning_requirement_id.len);
  } else {
    out.planning_requirement_id.clear();
  }
  if (in->plan_id.ptr && in->plan_id.len > 0) {
    out.plan_id.assign(in->plan_id.ptr, in->plan_id.len);
  } else {
    out.plan_id.clear();
  }
  if (in->plan_step_id.ptr && in->plan_step_id.len > 0) {
    out.plan_step_id.assign(in->plan_step_id.ptr, in->plan_step_id.len);
  } else {
    out.plan_step_id.clear();
  }
  if (in->target_component.ptr && in->target_component.len > 0) {
    out.target_component.assign(in->target_component.ptr, in->target_component.len);
  } else {
    out.target_component.clear();
  }
  if (in->target_service.ptr && in->target_service.len > 0) {
    out.target_service.assign(in->target_service.ptr, in->target_service.len);
  } else {
    out.target_service.clear();
  }
  if (in->target_type.ptr && in->target_type.len > 0) {
    out.target_type.assign(in->target_type.ptr, in->target_type.len);
  } else {
    out.target_type.clear();
  }
  out.operation = static_cast<pyramid::domain_model::RequirementPlacementOperation>(in->operation);
  if (in->target_requirement_id.ptr && in->target_requirement_id.len > 0) {
    out.target_requirement_id.assign(in->target_requirement_id.ptr, in->target_requirement_id.len);
  } else {
    out.target_requirement_id.clear();
  }
  out.related_entity_id.clear();
  if (in->related_entity_id.ptr && in->related_entity_id.len > 0) {
    const auto* arr = static_cast<const pyramid_str_t*>(in->related_entity_id.ptr);
    out.related_entity_id.reserve(in->related_entity_id.len);
    for (uint32_t i = 0; i < in->related_entity_id.len; ++i) {
      if (arr[i].ptr && arr[i].len > 0) {
        out.related_entity_id.emplace_back(arr[i].ptr, arr[i].len);
      } else {
        out.related_entity_id.emplace_back();
      }
    }
  }
  out.progress = static_cast<pyramid::domain_model::common::Progress>(in->progress);
}

void _free_RequirementPlacement(pyramid_RequirementPlacement_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->execution_requirement_id);
  free_str(value->planning_requirement_id);
  free_str(value->plan_id);
  free_str(value->plan_step_id);
  free_str(value->target_component);
  free_str(value->target_service);
  free_str(value->target_type);
  free_str(value->target_requirement_id);
  if (value->related_entity_id.ptr) {
    auto* arr = static_cast<pyramid_str_t*>(const_cast<void*>(value->related_entity_id.ptr));
    for (uint32_t i = 0; i < value->related_entity_id.len; ++i) {
      free_str(arr[i]);
    }
    std::free(const_cast<void*>(value->related_entity_id.ptr));
    value->related_entity_id.ptr = nullptr;
    value->related_entity_id.len = 0;
  }
}

void to_c(const pyramid::domain_model::ExecutionRun& in, pyramid_ExecutionRun_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  dup_str(out->execution_requirement_id, in.execution_requirement_id);
  dup_str(out->planning_requirement_id, in.planning_requirement_id);
  dup_str(out->plan_id, in.plan_id);
  out->state = static_cast<int32_t>(in.state);
  to_c(in.achievement, &out->achievement);
  out->replan_count = in.replan_count;
  {
    const auto count = in.outstanding_placement.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_RequirementPlacement_c*>(std::malloc(count * sizeof(pyramid_RequirementPlacement_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.outstanding_placement[i], &arr[i]);
      }
      out->outstanding_placement.ptr = arr;
      out->outstanding_placement.len = static_cast<uint32_t>(count);
    }
  }
}

void from_c(const pyramid_ExecutionRun_c* in, pyramid::domain_model::ExecutionRun& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  if (in->execution_requirement_id.ptr && in->execution_requirement_id.len > 0) {
    out.execution_requirement_id.assign(in->execution_requirement_id.ptr, in->execution_requirement_id.len);
  } else {
    out.execution_requirement_id.clear();
  }
  if (in->planning_requirement_id.ptr && in->planning_requirement_id.len > 0) {
    out.planning_requirement_id.assign(in->planning_requirement_id.ptr, in->planning_requirement_id.len);
  } else {
    out.planning_requirement_id.clear();
  }
  if (in->plan_id.ptr && in->plan_id.len > 0) {
    out.plan_id.assign(in->plan_id.ptr, in->plan_id.len);
  } else {
    out.plan_id.clear();
  }
  out.state = static_cast<pyramid::domain_model::ExecutionState>(in->state);
  from_c(&in->achievement, out.achievement);
  out.replan_count = in->replan_count;
  out.outstanding_placement.clear();
  if (in->outstanding_placement.ptr && in->outstanding_placement.len > 0) {
    const auto* arr = static_cast<const pyramid_RequirementPlacement_c*>(in->outstanding_placement.ptr);
    out.outstanding_placement.reserve(in->outstanding_placement.len);
    for (uint32_t i = 0; i < in->outstanding_placement.len; ++i) {
      pyramid::domain_model::RequirementPlacement elem{};
      from_c(&arr[i], elem);
      out.outstanding_placement.push_back(std::move(elem));
    }
  }
}

void _free_ExecutionRun(pyramid_ExecutionRun_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->execution_requirement_id);
  free_str(value->planning_requirement_id);
  free_str(value->plan_id);
  pyramid_Achievement_c_free(&value->achievement);
  if (value->outstanding_placement.ptr) {
    auto* arr = static_cast<pyramid_RequirementPlacement_c*>(const_cast<void*>(value->outstanding_placement.ptr));
    for (uint32_t i = 0; i < value->outstanding_placement.len; ++i) {
      pyramid_RequirementPlacement_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->outstanding_placement.ptr));
    value->outstanding_placement.ptr = nullptr;
    value->outstanding_placement.len = 0;
  }
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_RequirementReference_c_free(pyramid_RequirementReference_c* value) {
  pyramid::cabi::_free_RequirementReference(value);
}

void pyramid_AgentState_c_free(pyramid_AgentState_c* value) {
  pyramid::cabi::_free_AgentState(value);
}

void pyramid_PlanningPolicy_c_free(pyramid_PlanningPolicy_c* value) {
  pyramid::cabi::_free_PlanningPolicy(value);
}

void pyramid_PlanningGoal_c_free(pyramid_PlanningGoal_c* value) {
  pyramid::cabi::_free_PlanningGoal(value);
}

void pyramid_ExecutionPolicy_c_free(pyramid_ExecutionPolicy_c* value) {
  pyramid::cabi::_free_ExecutionPolicy(value);
}

void pyramid_PlanningRequirement_c_free(pyramid_PlanningRequirement_c* value) {
  pyramid::cabi::_free_PlanningRequirement(value);
}

void pyramid_ExecutionRequirement_c_free(pyramid_ExecutionRequirement_c* value) {
  pyramid::cabi::_free_ExecutionRequirement(value);
}

void pyramid_WorldFactUpdate_c_free(pyramid_WorldFactUpdate_c* value) {
  pyramid::cabi::_free_WorldFactUpdate(value);
}

void pyramid_StateUpdate_c_free(pyramid_StateUpdate_c* value) {
  pyramid::cabi::_free_StateUpdate(value);
}

void pyramid_Capabilities_c_free(pyramid_Capabilities_c* value) {
  pyramid::cabi::_free_Capabilities(value);
}

void pyramid_PlannedComponentInteraction_c_free(pyramid_PlannedComponentInteraction_c* value) {
  pyramid::cabi::_free_PlannedComponentInteraction(value);
}

void pyramid_PlanStep_c_free(pyramid_PlanStep_c* value) {
  pyramid::cabi::_free_PlanStep(value);
}

void pyramid_Plan_c_free(pyramid_Plan_c* value) {
  pyramid::cabi::_free_Plan(value);
}

void pyramid_RequirementPlacement_c_free(pyramid_RequirementPlacement_c* value) {
  pyramid::cabi::_free_RequirementPlacement(value);
}

void pyramid_ExecutionRun_c_free(pyramid_ExecutionRun_c* value) {
  pyramid::cabi::_free_ExecutionRun(value);
}

} // extern "C"
