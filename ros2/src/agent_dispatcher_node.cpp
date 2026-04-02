#include <ame_ros2/agent_dispatcher_node.hpp>

namespace ame_ros2 {

AgentDispatcherNode::AgentDispatcherNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("agent_dispatcher_node", options) {}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_configure(const rclcpp_lifecycle::State&) {
    declare_parameter("world_model_node", std::string("world_model_node"));

    // Set up transport callbacks
    component_.setBTSender(
        [this](const std::string& agent_id, const std::string& bt_xml) {
            return sendBTToAgent(agent_id, bt_xml);
        });

    component_.setStatusQuery(
        [this](const std::string& agent_id) {
            return queryAgentStatus(agent_id);
        });

    // Inject in-process dependencies if available
    if (inprocess_wm_) {
        component_.setWorldModel(inprocess_wm_);
    }
    if (inprocess_planner_) {
        component_.setPlanner(inprocess_planner_);
    }
    if (inprocess_compiler_) {
        component_.setPlanCompiler(inprocess_compiler_);
    }
    if (inprocess_registry_) {
        component_.setActionRegistry(inprocess_registry_);
    }

    if (component_.configure() != PCL_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to configure agent dispatcher component");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "AgentDispatcherNode configured");
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_activate(const rclcpp_lifecycle::State&) {
    if (component_.activate() != PCL_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to activate agent dispatcher component");
        return CallbackReturn::FAILURE;
    }

    srv_dispatch_ = create_service<ame_ros2::srv::DispatchGoals>(
        "~/dispatch_goals",
        [this](std::shared_ptr<ame_ros2::srv::DispatchGoals::Request> req,
               std::shared_ptr<ame_ros2::srv::DispatchGoals::Response> res) {
            handleDispatchGoals(req, res);
        });

    RCLCPP_INFO(get_logger(), "AgentDispatcherNode activated");
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_deactivate(const rclcpp_lifecycle::State&) {
    srv_dispatch_.reset();
    agent_bt_pubs_.clear();
    agent_status_subs_.clear();
    agent_statuses_.clear();
    component_.deactivate();
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_cleanup(const rclcpp_lifecycle::State&) {
    component_.cleanup();
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_shutdown(const rclcpp_lifecycle::State&) {
    component_.shutdown();
    return CallbackReturn::SUCCESS;
}

void AgentDispatcherNode::setInProcessDependencies(
    ame::WorldModel* wm,
    ame::Planner* planner,
    ame::PlanCompiler* compiler,
    ame::ActionRegistry* registry) {
    inprocess_wm_ = wm;
    inprocess_planner_ = planner;
    inprocess_compiler_ = compiler;
    inprocess_registry_ = registry;
}

void AgentDispatcherNode::ensureAgentPublisher(const std::string& agent_id) {
    if (agent_bt_pubs_.find(agent_id) != agent_bt_pubs_.end()) {
        return;
    }

    std::string topic = "/" + agent_id + "/executor/bt_xml";
    auto qos = rclcpp::QoS(10).reliable();
    agent_bt_pubs_[agent_id] = create_publisher<std_msgs::msg::String>(topic, qos);

    RCLCPP_INFO(get_logger(), "Created publisher for agent '%s' on %s",
                agent_id.c_str(), topic.c_str());
}

void AgentDispatcherNode::ensureAgentStatusSubscription(const std::string& agent_id) {
    if (agent_status_subs_.find(agent_id) != agent_status_subs_.end()) {
        return;
    }

    std::string topic = "/" + agent_id + "/executor/status";
    auto qos = rclcpp::QoS(1).reliable().transient_local();

    agent_status_subs_[agent_id] = create_subscription<std_msgs::msg::String>(
        topic, qos,
        [this, agent_id](const std_msgs::msg::String::SharedPtr msg) {
            agent_statuses_[agent_id] = msg->data;
            RCLCPP_DEBUG(get_logger(), "Agent '%s' status: %s",
                         agent_id.c_str(), msg->data.c_str());
        });

    // Initialize status as unknown
    agent_statuses_[agent_id] = "";

    RCLCPP_INFO(get_logger(), "Subscribed to agent '%s' status on %s",
                agent_id.c_str(), topic.c_str());
}

bool AgentDispatcherNode::sendBTToAgent(const std::string& agent_id,
                                         const std::string& bt_xml) {
    ensureAgentPublisher(agent_id);
    ensureAgentStatusSubscription(agent_id);

    auto pub_it = agent_bt_pubs_.find(agent_id);
    if (pub_it == agent_bt_pubs_.end()) {
        RCLCPP_ERROR(get_logger(), "No publisher for agent '%s'", agent_id.c_str());
        return false;
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = bt_xml;
    pub_it->second->publish(std::move(msg));

    RCLCPP_INFO(get_logger(), "Dispatched BT to agent '%s' (%zu bytes)",
                agent_id.c_str(), bt_xml.size());
    return true;
}

std::string AgentDispatcherNode::queryAgentStatus(const std::string& agent_id) {
    auto it = agent_statuses_.find(agent_id);
    if (it == agent_statuses_.end()) {
        return "";
    }
    return it->second;
}

void AgentDispatcherNode::handleDispatchGoals(
    std::shared_ptr<ame_ros2::srv::DispatchGoals::Request> req,
    std::shared_ptr<ame_ros2::srv::DispatchGoals::Response> res) {
    RCLCPP_INFO(get_logger(), "dispatch_goals called (%zu goals, %zu agents)",
                req->goal_fluents.size(), req->agent_ids.size());

    if (req->goal_fluents.empty()) {
        res->success = false;
        res->error_msg = "No goal fluents provided";
        return;
    }

    try {
        auto results = component_.dispatchGoals(req->goal_fluents);

        res->success = true;
        for (const auto& result : results) {
            if (result.success) {
                res->dispatched_agents.push_back(result.agent_id);
            } else {
                RCLCPP_WARN(get_logger(), "Failed to dispatch to agent '%s': %s",
                            result.agent_id.c_str(), result.error_message.c_str());
            }
        }

        if (res->dispatched_agents.empty()) {
            res->success = false;
            res->error_msg = "No agents were successfully dispatched";
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Dispatch failed: %s", e.what());
        res->success = false;
        res->error_msg = e.what();
    }
}

}  // namespace ame_ros2
