#include "mujin/planner_component.h"
#include "mujin/pddl_parser.h"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace mujin {

PlannerComponent::PlannerComponent()
    : pcl::Component("planner_component") {}

void PlannerComponent::setInProcessWorldModel(const WorldModel* wm) {
  inprocess_wm_ = wm;
}

void PlannerComponent::setQueryStateCallback(QueryStateCallback callback) {
  query_state_callback_ = std::move(callback);
}

PlannerExecutionResult PlannerComponent::solveGoal(
    const std::vector<std::string>& goal_fluents) {
  PlannerExecutionResult execution_result;

  auto local_wm = snapshotWorldModel(goal_fluents);
  auto plan_result = planner_.solve(*local_wm);

  execution_result.success = plan_result.success;
  execution_result.solve_time_ms = plan_result.solve_time_ms;
  execution_result.expanded = plan_result.expanded;
  execution_result.generated = plan_result.generated;
  execution_result.cost = plan_result.cost;

  if (!plan_result.success) {
    execution_result.error_msg = "No plan found";
    return execution_result;
  }

  for (const auto& step : plan_result.steps) {
    const auto& ground_action = local_wm->groundActions().at(step.action_index);
    execution_result.plan_actions.push_back(ground_action.signature);
    execution_result.action_indices.push_back(step.action_index);
  }

  if (compiler_parallel_) {
    execution_result.bt_xml = compiler_.compile(plan_result.steps, *local_wm, registry_);
  } else {
    execution_result.bt_xml =
        compiler_.compileSequential(plan_result.steps, *local_wm, registry_);
  }

  recordAuditEpisode(*local_wm, plan_result, execution_result);
  return execution_result;
}

pcl_status_t PlannerComponent::on_configure() {
  compiler_parallel_ = paramBool("compiler.parallel", false);

  audit_log_.reset();
  if (paramBool("plan_audit.enabled", true)) {
    audit_log_.emplace(paramStr("plan_audit.path", "plan_audit.jsonl"));
  }

  return PCL_OK;
}

pcl_status_t PlannerComponent::on_activate() {
  return PCL_OK;
}

pcl_status_t PlannerComponent::on_deactivate() {
  return PCL_OK;
}

pcl_status_t PlannerComponent::on_cleanup() {
  if (audit_log_) {
    audit_log_->flush();
  }
  audit_log_.reset();
  return PCL_OK;
}

pcl_status_t PlannerComponent::on_shutdown() {
  if (audit_log_) {
    audit_log_->flush();
  }
  return PCL_OK;
}

std::unique_ptr<WorldModel> PlannerComponent::snapshotWorldModel(
    const std::vector<std::string>& goal_fluents) const {
  if (inprocess_wm_ != nullptr) {
    auto wm = std::make_unique<WorldModel>(*inprocess_wm_);
    wm->setAuditCallback({});
    wm->setGoal(goal_fluents);
    return wm;
  }

  if (!query_state_callback_) {
    throw std::runtime_error("No world state source configured");
  }

  const auto domain_file = paramStr("domain.pddl_file", "");
  const auto problem_file = paramStr("domain.problem_file", "");
  if (domain_file.empty() || problem_file.empty()) {
    throw std::runtime_error(
        "domain.pddl_file / domain.problem_file must be set for distributed planning");
  }

  auto wm = std::make_unique<WorldModel>();
  PddlParser::parse(domain_file, problem_file, *wm);
  for (unsigned i = 0; i < wm->numFluents(); ++i) {
    wm->setFact(i, false, "planner_snapshot_reset");
  }

  const auto snapshot = query_state_callback_();
  if (!snapshot.success) {
    throw std::runtime_error("World state query failed");
  }

  for (const auto& fact : snapshot.facts) {
    try {
      wm->setFact(fact.key, fact.value, "snapshot");
    } catch (...) {
    }
  }

  wm->setGoal(goal_fluents);
  return wm;
}

void PlannerComponent::recordAuditEpisode(
    const WorldModel& local_wm,
    const PlanResult& plan_result,
    const PlannerExecutionResult& execution_result) {
  if (!audit_log_) {
    return;
  }

  PlanAuditLog::Episode episode;
  episode.ts_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  episode.solver = "BRFS";
  episode.solve_time_ms = plan_result.solve_time_ms;
  episode.success = plan_result.success;
  episode.expanded = plan_result.expanded;
  episode.generated = plan_result.generated;
  episode.cost = plan_result.cost;
  episode.bt_xml = execution_result.bt_xml;

  for (unsigned i = 0; i < local_wm.numFluents(); ++i) {
    if (local_wm.getFact(i)) {
      episode.init_facts.push_back(local_wm.fluentName(i));
    }
  }
  for (auto goal_id : local_wm.goalFluentIds()) {
    episode.goal_fluents.push_back(local_wm.fluentName(goal_id));
  }
  for (const auto& action : execution_result.plan_actions) {
    episode.plan_actions.push_back(action);
  }

  audit_log_->recordEpisode(episode);
  audit_log_->flush();
}

}  // namespace mujin
