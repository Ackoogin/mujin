#include <ame/agent_dispatcher.h>

namespace ame {

AgentDispatcher::AgentDispatcher() : pcl::Component("agent_dispatcher") {}

pcl_status_t AgentDispatcher::on_configure() {
    return PCL_OK;
}

pcl_status_t AgentDispatcher::on_activate() {
    return PCL_OK;
}

pcl_status_t AgentDispatcher::on_deactivate() {
    dispatched_agents_.clear();
    return PCL_OK;
}

pcl_status_t AgentDispatcher::on_cleanup() {
    dispatched_agents_.clear();
    return PCL_OK;
}

pcl_status_t AgentDispatcher::on_shutdown() {
    dispatched_agents_.clear();
    return PCL_OK;
}

std::vector<AgentDispatchResult> AgentDispatcher::dispatchGoals(
    const std::vector<std::string>& goals) {
    
    std::vector<AgentDispatchResult> results;
    
    if (!wm_ || !planner_ || !compiler_ || !registry_) {
        return results;
    }
    
    // Allocate goals to available agents
    auto assignments = allocator_.allocate(goals, *wm_);
    
    for (const auto& assignment : assignments) {
        auto result = dispatchToAgent(assignment.agent_id, assignment.goals);
        results.push_back(std::move(result));
    }
    
    return results;
}

AgentDispatchResult AgentDispatcher::dispatchToAgent(
    const std::string& agent_id,
    const std::vector<std::string>& goals) {
    
    AgentDispatchResult result;
    result.agent_id = agent_id;
    
    if (!wm_ || !planner_ || !compiler_ || !registry_) {
        result.error_message = "Missing required components";
        return result;
    }
    
    // Verify agent exists and is available
    AgentInfo* agent = wm_->getAgent(agent_id);
    if (!agent) {
        result.error_message = "Agent not found: " + agent_id;
        return result;
    }
    
    if (!agent->available) {
        result.error_message = "Agent not available: " + agent_id;
        return result;
    }
    
    // Mark agent as busy
    agent->available = false;
    
    // Plan for agent's goals
    wm_->setGoal(goals);
    auto plan_result = planner_->solve(*wm_);
    
    if (!plan_result.success) {
        agent->available = true;
        result.error_message = "Planning failed for agent: " + agent_id;
        return result;
    }
    
    result.solve_time_ms = plan_result.solve_time_ms;
    
    // Extract plan actions
    for (const auto& step : plan_result.steps) {
        result.plan_actions.push_back(
            wm_->groundActions()[step.action_index].signature);
    }
    
    // Compile BT XML with agent context
    result.bt_xml = compiler_->compile(plan_result.steps, *wm_, *registry_, agent_id);
    
    // Dispatch via transport callback
    if (bt_sender_) {
        result.success = bt_sender_(agent_id, result.bt_xml);
        if (result.success) {
            dispatched_agents_.push_back(agent_id);
        } else {
            agent->available = true;
            result.error_message = "Failed to send BT to agent: " + agent_id;
        }
    } else {
        // No sender configured - just mark success for in-process use
        result.success = true;
        dispatched_agents_.push_back(agent_id);
    }
    
    return result;
}

std::map<std::string, std::string> AgentDispatcher::queryAllStatus() const {
    std::map<std::string, std::string> statuses;
    
    if (!status_query_) {
        return statuses;
    }
    
    for (const auto& agent_id : dispatched_agents_) {
        statuses[agent_id] = status_query_(agent_id);
    }
    
    return statuses;
}

bool AgentDispatcher::allAgentsComplete() const {
    if (dispatched_agents_.empty()) {
        return true;
    }
    
    if (!status_query_) {
        return false;
    }
    
    for (const auto& agent_id : dispatched_agents_) {
        std::string status = status_query_(agent_id);
        if (status != "SUCCESS" && status != "FAILURE") {
            return false;
        }
    }
    
    return true;
}

bool AgentDispatcher::allAgentsSucceeded() const {
    if (dispatched_agents_.empty()) {
        return true;
    }
    
    if (!status_query_) {
        return false;
    }
    
    for (const auto& agent_id : dispatched_agents_) {
        std::string status = status_query_(agent_id);
        if (status != "SUCCESS") {
            return false;
        }
    }
    
    return true;
}

}  // namespace ame
