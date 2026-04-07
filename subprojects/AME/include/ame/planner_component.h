#pragma once

#include <ame/action_registry.h>
#include <ame/plan_audit_log.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/world_model.h>
#include <ame/world_model_component.h>
#include <pcl/component.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ame {

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

  /// \brief Access the underlying planner for in-process hierarchical planning.
  Planner& planner() { return planner_; }

  /// \brief Access the plan compiler for in-process hierarchical planning.
  PlanCompiler& compiler() { return compiler_; }

  /// \brief Access the plan audit log (may be nullptr if disabled).
  PlanAuditLog* planAuditLog() {
    return audit_log_.has_value() ? &audit_log_.value() : nullptr;
  }

  /// \brief Load a domain from PDDL strings at runtime.
  /// Stores the parsed domain template for use in distributed-mode snapshots,
  /// replacing the need to read domain files from disk.
  /// Returns {success, error_msg, num_fluents, num_ground_actions}.
  struct LoadDomainResult {
    bool success = false;
    std::string error_msg;
    unsigned num_fluents = 0;
    unsigned num_ground_actions = 0;
  };
  LoadDomainResult loadDomainFromStrings(const std::string& domain_id,
                                         const std::string& domain_pddl,
                                         const std::string& problem_pddl);

  /// \brief Returns the currently loaded domain ID (empty if loaded from files).
  const std::string& domainId() const { return loaded_domain_id_; }

  /// \brief Returns true if a domain has been loaded (from files or strings).
  bool hasDomain() const { return loaded_domain_template_ != nullptr; }

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

  // Domain loaded at runtime via loadDomainFromStrings()
  std::string loaded_domain_id_;
  std::unique_ptr<WorldModel> loaded_domain_template_;
};

}  // namespace ame
