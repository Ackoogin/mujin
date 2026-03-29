#pragma once

#include <ame/action_registry.h>
#include <ame/goal_allocator.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/world_model.h>
#include <pcl/component.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ame {

/// \brief Result of dispatching a plan to an agent.
struct AgentDispatchResult {
    std::string agent_id;
    bool success = false;
    std::string bt_xml;           ///< Compiled BT XML for the agent
    std::vector<std::string> plan_actions;  ///< Action signatures in plan
    double solve_time_ms = 0.0;
    std::string error_message;
};

/// \brief Callback signature for sending BT XML to a remote agent executor.
/// Transport layer (ROS2, etc.) implements this to publish to agent topics.
using AgentBTSender = std::function<bool(const std::string& agent_id,
                                          const std::string& bt_xml)>;

/// \brief Callback signature for querying agent execution status.
/// Returns: "IDLE", "RUNNING", "SUCCESS", "FAILURE", or empty on error.
using AgentStatusQuery = std::function<std::string(const std::string& agent_id)>;

/// \brief PCL component for coordinating multi-agent plan dispatch.
///
/// Business logic for leader-delegation pattern:
/// 1. Allocate goals to available agents
/// 2. Plan for each agent's goals
/// 3. Compile agent-scoped BT XML
/// 4. Dispatch via transport-agnostic callbacks
/// 5. Monitor agent status for completion
///
/// ROS2 layer provides AgentBTSender/AgentStatusQuery implementations
/// that publish to /{agent_id}/executor/bt_xml topics.
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

    /// \brief Set the transport callback for sending BT XML to agents.
    void setBTSender(AgentBTSender sender) { bt_sender_ = std::move(sender); }

    /// \brief Set the transport callback for querying agent status.
    void setStatusQuery(AgentStatusQuery query) { status_query_ = std::move(query); }

    /// \brief Allocate and dispatch goals to available agents.
    /// \param goals Mission goals to distribute
    /// \return Results for each agent dispatch attempt
    std::vector<AgentDispatchResult> dispatchGoals(
        const std::vector<std::string>& goals);

    /// \brief Dispatch a specific set of goals to a specific agent.
    /// \param agent_id Target agent
    /// \param goals Goals for this agent
    /// \return Dispatch result
    AgentDispatchResult dispatchToAgent(
        const std::string& agent_id,
        const std::vector<std::string>& goals);

    /// \brief Query the current status of all dispatched agents.
    /// \return Map of agent_id -> status string
    std::map<std::string, std::string> queryAllStatus() const;

    /// \brief Check if all dispatched agents have completed.
    /// \return true if all agents report SUCCESS or FAILURE
    bool allAgentsComplete() const;

    /// \brief Check if all dispatched agents succeeded.
    /// \return true if all agents report SUCCESS
    bool allAgentsSucceeded() const;

    /// \brief Get IDs of agents currently dispatched.
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
    WorldModel* wm_ = nullptr;
    Planner* planner_ = nullptr;
    PlanCompiler* compiler_ = nullptr;
    ActionRegistry* registry_ = nullptr;

    GoalAllocator allocator_;
    AgentBTSender bt_sender_;
    AgentStatusQuery status_query_;

    std::vector<std::string> dispatched_agents_;
};

}  // namespace ame
