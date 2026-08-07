#pragma once

#include "ame/world_state_access.h"

#include <behaviortree_cpp/decorator_node.h>

#include <string>
#include <vector>

namespace ame {

/// \brief Applies the planned-action contract to a registered XML subtree.
class PlannedAction : public BT::DecoratorNode {
public:
  PlannedAction(const std::string& name,
                const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  std::vector<std::string> factList(const std::string& port_name) const;
  IWorldStateAccess* worldStateAccess();
  bool preconditionsMet();
  bool commitEffects();

  bool started_ = false;
  bool reactive_ = false;
  LocalWorldStateAccess local_world_state_access_{nullptr};
};

}  // namespace ame
