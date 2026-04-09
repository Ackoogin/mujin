#include <ame/agent_dispatcher.h>
#include <ame/pcl_msg_json.h>

#include <sstream>

namespace ame {

AgentDispatcher::AgentDispatcher() : pcl::Component("agent_dispatcher") {}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

pcl_status_t AgentDispatcher::on_configure() {
    agent_bt_pubs_.clear();
    agent_statuses_.clear();
    status_cb_ctxs_.clear();

    // Parse agent roster from "agent_ids" parameter (comma-separated)
    const auto ids_str = paramStr("agent_ids", "");
    const auto agent_ids = parseAgentIds(ids_str);

    for (const auto& id : agent_ids) {
        // Publisher for sending BT XML to this agent
        const std::string bt_topic     = id + "/executor/bt_xml";
        const std::string status_topic = id + "/executor/status";

        agent_bt_pubs_[id] = addPublisher(bt_topic.c_str(), "ame/BTXML");
        agent_statuses_[id] = "";

        // Subscriber for receiving status from this agent.
        // Pass a context struct so the callback knows which agent it came from.
        auto ctx = std::make_unique<StatusCbCtx>();
        ctx->self     = this;
        ctx->agent_id = id;
        addSubscriber(status_topic.c_str(), "ame/Status",
                      onAgentStatusCb, ctx.get());
        status_cb_ctxs_.push_back(std::move(ctx));
    }

    addService("dispatch_goals", "ame/DispatchGoals",
               handleDispatchGoalsCb, this);

    return PCL_OK;
}

pcl_status_t AgentDispatcher::on_activate() { return PCL_OK; }

pcl_status_t AgentDispatcher::on_deactivate() {
    dispatched_agents_.clear();
    return PCL_OK;
}

pcl_status_t AgentDispatcher::on_cleanup() {
    dispatched_agents_.clear();
    agent_bt_pubs_.clear();
    agent_statuses_.clear();
    status_cb_ctxs_.clear();
    return PCL_OK;
}

pcl_status_t AgentDispatcher::on_shutdown() {
    dispatched_agents_.clear();
    return PCL_OK;
}

// ---------------------------------------------------------------------------
// Business logic
// ---------------------------------------------------------------------------

std::vector<AgentDispatchResult> AgentDispatcher::dispatchGoals(
    const std::vector<std::string>& goals) {
    std::vector<AgentDispatchResult> results;
    if (!wm_ || !planner_ || !compiler_ || !registry_) return results;

    auto assignments = allocator_.allocate(goals, *wm_);
    for (const auto& assignment : assignments) {
        results.push_back(dispatchToAgent(assignment.agent_id, assignment.goals));
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

    AgentInfo* agent = wm_->getAgent(agent_id);
    if (!agent) {
        result.error_message = "Agent not found: " + agent_id;
        return result;
    }
    if (!agent->available) {
        result.error_message = "Agent not available: " + agent_id;
        return result;
    }

    agent->available = false;

    wm_->setGoal(goals);
    auto plan_result = planner_->solve(*wm_);
    if (!plan_result.success) {
        agent->available = true;
        result.error_message = "Planning failed for agent: " + agent_id;
        return result;
    }

    result.solve_time_ms = plan_result.solve_time_ms;
    for (const auto& step : plan_result.steps) {
        result.plan_actions.push_back(
            wm_->groundActions()[step.action_index].signature);
    }
    result.bt_xml = compiler_->compile(plan_result.steps, *wm_, *registry_, agent_id);

    // Dispatch via PCL port
    auto it = agent_bt_pubs_.find(agent_id);
    if (it != agent_bt_pubs_.end() && it->second) {
        pcl_msg_t msg;
        msg.data      = result.bt_xml.c_str();
        msg.size      = static_cast<uint32_t>(result.bt_xml.size());
        msg.type_name = "ame/BTXML";
        const auto rc = pcl_port_publish(it->second, &msg);
        result.success = (rc == PCL_OK);
        if (result.success) {
            dispatched_agents_.push_back(agent_id);
        } else {
            agent->available = true;
            result.error_message = "Failed to publish BT to agent: " + agent_id;
        }
    } else {
        // Port not in roster (in-process / test mode without a real port)
        result.success = true;
        dispatched_agents_.push_back(agent_id);
    }

    return result;
}

std::map<std::string, std::string> AgentDispatcher::queryAllStatus() const {
    std::map<std::string, std::string> statuses;
    for (const auto& id : dispatched_agents_) {
        auto it = agent_statuses_.find(id);
        statuses[id] = (it != agent_statuses_.end()) ? it->second : "";
    }
    return statuses;
}

bool AgentDispatcher::allAgentsComplete() const {
    if (dispatched_agents_.empty()) return true;
    for (const auto& id : dispatched_agents_) {
        auto it = agent_statuses_.find(id);
        const std::string& s = (it != agent_statuses_.end()) ? it->second : "";
        if (s != "SUCCESS" && s != "FAILURE") return false;
    }
    return true;
}

bool AgentDispatcher::allAgentsSucceeded() const {
    if (dispatched_agents_.empty()) return true;
    for (const auto& id : dispatched_agents_) {
        auto it = agent_statuses_.find(id);
        const std::string& s = (it != agent_statuses_.end()) ? it->second : "";
        if (s != "SUCCESS") return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

std::vector<std::string> AgentDispatcher::parseAgentIds(const std::string& param) {
    std::vector<std::string> ids;
    if (param.empty()) return ids;
    std::istringstream ss(param);
    std::string token;
    while (std::getline(ss, token, ',')) {
        // Trim whitespace
        size_t start = token.find_first_not_of(" \t");
        size_t end   = token.find_last_not_of(" \t");
        if (start != std::string::npos) {
            ids.push_back(token.substr(start, end - start + 1));
        }
    }
    return ids;
}

// ---------------------------------------------------------------------------
// Static PCL callbacks
// ---------------------------------------------------------------------------

void AgentDispatcher::onAgentStatusCb(pcl_container_t*,
                                       const pcl_msg_t* msg,
                                       void* ud) {
    auto* ctx    = static_cast<StatusCbCtx*>(ud);
    auto  status = ame_msg_to_string(msg);
    ctx->self->agent_statuses_[ctx->agent_id] = status;
}

pcl_status_t AgentDispatcher::handleDispatchGoalsCb(pcl_container_t*,
                                                     const pcl_msg_t* req,
                                                     pcl_msg_t* resp,
                                                     pcl_svc_context_t*,
                                                     void* ud) {
    auto* self = static_cast<AgentDispatcher*>(ud);
    auto  dreq = ame_unpack_dispatch_goals_request(req);

    DispatchGoalsResponse dresp;
    if (dreq.goal_fluents.empty()) {
        dresp.success   = false;
        dresp.error_msg = "No goal fluents provided";
    } else {
        try {
            auto results = self->dispatchGoals(dreq.goal_fluents);
            dresp.success = true;
            for (const auto& r : results) {
                if (r.success) {
                    dresp.dispatched_agents.push_back(r.agent_id);
                }
            }
            if (dresp.dispatched_agents.empty()) {
                dresp.success   = false;
                dresp.error_msg = "No agents were successfully dispatched";
            }
        } catch (const std::exception& e) {
            dresp.success   = false;
            dresp.error_msg = e.what();
        }
    }

    self->resp_buf_dispatch_ = ame_pack_dispatch_goals_response(dresp);
    ame_make_pcl_msg(self->resp_buf_dispatch_, "ame/DispatchGoals_Response", *resp);
    return PCL_OK;
}

}  // namespace ame
