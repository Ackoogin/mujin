#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace mujin {

class WorldModel;

class CheckWorldPredicate : public BT::ConditionNode {
public:
    CheckWorldPredicate(const std::string& name,
                        const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace mujin
