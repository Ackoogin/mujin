#pragma once

#include <behaviortree_cpp/condition_node.h>

#include <string>

namespace ame {

/// \brief Succeeds when every grounded fact in the goals port is true.
class GoalReached : public BT::ConditionNode {
public:
  GoalReached(const std::string& name,
              const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace ame
