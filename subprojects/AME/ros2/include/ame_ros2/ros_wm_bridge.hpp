#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

#include "ame/world_state_access.h"
#include "ame_ros2/srv/get_fact.hpp"
#include "ame_ros2/srv/set_fact.hpp"

namespace ame_ros2 {

/// \brief BT ConditionNode that calls WorldModelNode/get_fact.
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

/// \brief World-state access backed by WorldModelNode GetFact and SetFact
/// service clients.
class RosWorldStateAccess : public ame::IWorldStateAccess {
public:
  RosWorldStateAccess(
      rclcpp::Client<ame_ros2::srv::GetFact>::SharedPtr get_fact_client,
      rclcpp::Client<ame_ros2::srv::SetFact>::SharedPtr set_fact_client);

  bool getFact(const std::string& key) override;
  bool setFact(const std::string& key,
               bool value,
               const std::string& source) override;

private:
  rclcpp::Client<ame_ros2::srv::GetFact>::SharedPtr get_fact_client_;
  rclcpp::Client<ame_ros2::srv::SetFact>::SharedPtr set_fact_client_;
};

} // namespace ame_ros2
