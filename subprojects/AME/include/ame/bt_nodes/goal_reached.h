#pragma once

#include <behaviortree_cpp/condition_node.h>

#include <string>

namespace ame {

/// \brief Succeeds when every grounded fact in the goals port is true.
///
/// Like a planned action, it accepts an ame_required_authority port. Set to
/// "confirmed" it treats a goal as reached only on observed facts, so the goal
/// guard and the actions below it can be held to the same standard of evidence.
class GoalReached : public BT::ConditionNode {
public:
  GoalReached(const std::string& name,
              const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace ame
