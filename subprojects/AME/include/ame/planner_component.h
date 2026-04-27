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

/// \brief PCL-backed planner component.
///
/// Ports created during on_configure():
///   pub  "bt_xml"      (ame/BTXML)      -- compiled BT XML after successful planning
///   svc  "load_domain" (ame/LoadDomain) -- load PDDL domain from strings at runtime
///   svc  "plan"        (ame/Plan)       -- synchronous planning request/response
///
/// Planning runs synchronously on the executor thread (no std::thread).
///
/// Parameters:
///   domain.pddl_file     (string, "")
///   domain.problem_file  (string, "")
///   plan_audit.enabled   (bool,   true)
///   plan_audit.path      (string, "plan_audit.jsonl")
///   compiler.parallel    (bool,   false)
///   action_registry.<n>  (string) -- maps PDDL action name to BT node type
class PlannerComponent : public pcl::Component {
public:
  using QueryStateCallback = std::function<WorldStateSnapshot()>;

  PlannerComponent();

  /// \brief Configure direct in-process state access.
  void setInProcessWorldModel(const WorldModel* wm);

  /// \brief Configure distributed state access through a callback.
  void setQueryStateCallback(QueryStateCallback callback);

  ActionRegistry& actionRegistry() { return registry_; }
  const ActionRegistry& actionRegistry() const { return registry_; }

  Planner& planner() { return planner_; }
  PlanCompiler& compiler() { return compiler_; }

  PlanAuditLog* planAuditLog() {
    return audit_log_.has_value() ? &audit_log_.value() : nullptr;
  }

  struct LoadDomainResult {
    bool success = false;
    std::string error_msg;
    unsigned num_fluents = 0;
    unsigned num_ground_actions = 0;
  };
  LoadDomainResult loadDomainFromStrings(const std::string& domain_id,
                                         const std::string& domain_pddl,
                                         const std::string& problem_pddl);

  const std::string& domainId() const { return loaded_domain_id_; }
  bool hasDomain() const { return loaded_domain_template_ != nullptr; }

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

  std::string loaded_domain_id_;
  std::unique_ptr<WorldModel> loaded_domain_template_;

  // PCL ports (valid after on_configure)
  pcl_port_t* pub_bt_xml_ = nullptr;

  // Per-service response buffers (serialised on executor thread)
  std::string resp_buf_load_domain_;
  std::string resp_buf_plan_;

  // -- Static PCL service callbacks ----------------------------------------

  static pcl_status_t handleLoadDomainCb(pcl_container_t*,
                                          const pcl_msg_t* req,
                                          pcl_msg_t* resp,
                                          pcl_svc_context_t*,
                                          void* ud);

  static pcl_status_t handlePlanCb(pcl_container_t*,
                                    const pcl_msg_t* req,
                                    pcl_msg_t* resp,
                                    pcl_svc_context_t*,
                                    void* ud);
};

}  // namespace ame
