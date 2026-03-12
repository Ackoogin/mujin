#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "ame_ros2/srv/get_fact.hpp"
#include "ame_ros2/srv/set_fact.hpp"

namespace ame_ros2 {

/// BT ConditionNode that calls WorldModelNode/get_fact service.
/// Used in distributed mode instead of ame::CheckWorldPredicate.
/// Ports: "predicate" (string), "expected" (bool, default true)
/// Blackboard key: "get_fact_client" (rclcpp::Client<GetFact>*)
class RosCheckWorldPredicate : public BT::ConditionNode {
public:
    RosCheckWorldPredicate(const std::string& name,
                           const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/// BT SyncActionNode that calls WorldModelNode/set_fact service.
/// Used in distributed mode instead of ame::SetWorldPredicate.
/// Ports: "predicate" (string), "value" (bool, default true)
/// Blackboard key: "set_fact_client" (rclcpp::Client<SetFact>*)
class RosSetWorldPredicate : public BT::SyncActionNode {
public:
    RosSetWorldPredicate(const std::string& name,
                         const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace ame_ros2
