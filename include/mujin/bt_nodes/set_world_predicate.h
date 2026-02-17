#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace mujin {

class WorldModel;

class SetWorldPredicate : public BT::SyncActionNode {
public:
    SetWorldPredicate(const std::string& name,
                      const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace mujin
