#pragma once

#include <ame/action_registry.h>
#include <ame/goal_allocator.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/world_model.h>
#include <pcl/component.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ame {

/// \brief Result of dispatching a plan to an agent.
struct AgentDispatchResult {
    std::string agent_id;
    bool success = false;
    std::string bt_xml;
    std::vector<std::string> plan_actions;
    double solve_time_ms = 0.0;
    std::string error_message;
};

/// \brief PCL component for coordinating multi-agent plan dispatch.
///
/// Uses a fixed agent roster declared via the "agent_ids" parameter
/// (comma-separated list, e.g. "uav1,uav2").  Ports are created in
/// on_configure() and are immutable thereafter.
///
/// Ports created during on_configure():
///   pub  "{id}/executor/bt_xml"   (ame/BTXML)  — one per agent in roster
///   sub  "{id}/executor/status"   (ame/Status) — one per agent in roster
///   svc  "dispatch_goals"         (ame/DispatchGoals)
///
/// Parameters:
///   agent_ids  (string, "") — comma-separated agent IDs (e.g. "uav1,uav2")
class AgentDispatcher : public pcl::Component {
public:
    AgentDispatcher();

    /// \brief Set the world model for planning.
    void setWorldModel(WorldModel* wm) { wm_ = wm; }

    /// \brief Set the planner for solving agent sub-goals.
    void setPlanner(Planner* planner) { planner_ = planner; }

    /// \brief Set the plan compiler for generating BT XML.
    void setPlanCompiler(PlanCompiler* compiler) { compiler_ = compiler; }

    /// \brief Set the action registry for BT compilation.
    void setActionRegistry(ActionRegistry* registry) { registry_ = registry; }

    /// \brief Allocate and dispatch goals to available agents.
    std::vector<AgentDispatchResult> dispatchGoals(
        const std::vector<std::string>& goals);

    /// \brief Dispatch a specific set of goals to a specific agent.
    AgentDispatchResult dispatchToAgent(
        const std::string& agent_id,
        const std::vector<std::string>& goals);

    /// \brief Query the current status of all dispatched agents.
    std::map<std::string, std::string> queryAllStatus() const;

    /// \brief Check if all dispatched agents have completed.
    bool allAgentsComplete() const;

    /// \brief Check if all dispatched agents succeeded.
    bool allAgentsSucceeded() const;

    const std::vector<std::string>& dispatchedAgents() const {
        return dispatched_agents_;
    }

protected:
    pcl_status_t on_configure() override;
    pcl_status_t on_activate() override;
    pcl_status_t on_deactivate() override;
    pcl_status_t on_cleanup() override;
    pcl_status_t on_shutdown() override;

private:
    static std::vector<std::string> parseAgentIds(const std::string& param);

    WorldModel*    wm_       = nullptr;
    Planner*       planner_  = nullptr;
    PlanCompiler*  compiler_ = nullptr;
    ActionRegistry* registry_ = nullptr;

    GoalAllocator allocator_;

    std::vector<std::string> dispatched_agents_;

    // PCL ports — one publisher and one status cache per agent in the roster
    std::map<std::string, pcl_port_t*> agent_bt_pubs_;
    std::map<std::string, std::string> agent_statuses_;

    // Per-service response buffer (serialised on executor thread)
    std::string resp_buf_dispatch_;

    // -- Static PCL callbacks --------------------------------------------

    static void onAgentStatusCb(pcl_container_t*, const pcl_msg_t* msg, void* ud);

    // user_data for onAgentStatusCb carries the agent id along with `this`
    struct StatusCbCtx { AgentDispatcher* self; std::string agent_id; };
    std::vector<std::unique_ptr<StatusCbCtx>> status_cb_ctxs_;

    static pcl_status_t handleDispatchGoalsCb(pcl_container_t*,
                                               const pcl_msg_t* req,
                                               pcl_msg_t* resp,
                                               pcl_svc_context_t*,
                                               void* ud);
};

}  // namespace ame
