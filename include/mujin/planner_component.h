#pragma once

#include "mujin/action_registry.h"
#include "mujin/plan_audit_log.h"
#include "mujin/plan_compiler.h"
#include "mujin/planner.h"
#include "mujin/world_model.h"
#include "mujin/world_model_component.h"
#include "pcl/component.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mujin {

struct PlannerExecutionResult {
  bool success = false;
  std::vector<std::string> plan_actions;
  std::vector<unsigned> action_indices;
  std::string bt_xml;
  double solve_time_ms = 0.0;
  unsigned expanded = 0;
  unsigned generated = 0;
  float cost = 0.0f;
  std::string error_msg;
};

/// \brief PCL-backed planner component with ROS-agnostic planning logic.
class PlannerComponent : public pcl::Component {
public:
  using QueryStateCallback = std::function<WorldStateSnapshot()>;

  PlannerComponent();

  /// \brief Configure direct in-process state access.
  void setInProcessWorldModel(const WorldModel* wm);

  /// \brief Configure distributed state access through a callback.
  void setQueryStateCallback(QueryStateCallback callback);

  /// \brief Register or inspect PDDL-to-BT action mappings.
  ActionRegistry& actionRegistry() { return registry_; }

  /// \brief Register or inspect PDDL-to-BT action mappings.
  const ActionRegistry& actionRegistry() const { return registry_; }

  /// \brief Solve a goal and compile the resulting plan to BT XML.
  PlannerExecutionResult solveGoal(const std::vector<std::string>& goal_fluents);

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;
  pcl_status_t on_deactivate() override;
  pcl_status_t on_cleanup() override;
  pcl_status_t on_shutdown() override;

private:
  std::unique_ptr<WorldModel> snapshotWorldModel(
      const std::vector<std::string>& goal_fluents) const;
  void recordAuditEpisode(const WorldModel& local_wm,
                          const PlanResult& plan_result,
                          const PlannerExecutionResult& execution_result);

  Planner planner_;
  PlanCompiler compiler_;
  ActionRegistry registry_;
  std::optional<PlanAuditLog> audit_log_;
  const WorldModel* inprocess_wm_ = nullptr;
  QueryStateCallback query_state_callback_;
  bool compiler_parallel_ = false;
};

}  // namespace mujin
