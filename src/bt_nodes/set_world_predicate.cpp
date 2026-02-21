#include "mujin/bt_nodes/set_world_predicate.h"
#include "mujin/world_model.h"

namespace mujin {

SetWorldPredicate::SetWorldPredicate(const std::string& name,
                                     const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList SetWorldPredicate::providedPorts() {
    return {
        BT::InputPort<std::string>("predicate"),
        BT::InputPort<bool>("value", true, "value to set"),
    };
}

BT::NodeStatus SetWorldPredicate::tick() {
    auto pred = getInput<std::string>("predicate");
    if (!pred) return BT::NodeStatus::FAILURE;

    bool value = true;
    getInput("value", value);

    auto* wm = config().blackboard->get<WorldModel*>("world_model");
    wm->setFact(pred.value(), value, std::string("SetWorldPredicate:") + name());

    return BT::NodeStatus::SUCCESS;
}

} // namespace mujin
