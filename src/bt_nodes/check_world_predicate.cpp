#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/world_model.h"

namespace mujin {

CheckWorldPredicate::CheckWorldPredicate(const std::string& name,
                                         const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckWorldPredicate::providedPorts() {
    return {
        BT::InputPort<std::string>("predicate"),
        BT::InputPort<bool>("expected", true, "expected truth value"),
    };
}

BT::NodeStatus CheckWorldPredicate::tick() {
    auto pred = getInput<std::string>("predicate");
    if (!pred) return BT::NodeStatus::FAILURE;

    bool expected = true;
    getInput("expected", expected);

    auto* wm = config().blackboard->get<WorldModel*>("world_model");
    bool actual = wm->getFact(pred.value());

    return (actual == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace mujin
