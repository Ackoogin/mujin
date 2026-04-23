#include "ame/bt_nodes/check_world_predicate.h"
#include "ame/world_model.h"

namespace ame {

CheckWorldPredicate::CheckWorldPredicate(const std::string& name,
                                         const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckWorldPredicate::providedPorts() {
    return {
        BT::InputPort<std::string>("predicate"),
        BT::InputPort<bool>("expected", true, "expected truth value"),
        BT::InputPort<std::string>("required_authority", "any",
            "authority required for the fact: 'any' (default) or 'confirmed'"),
    };
}

BT::NodeStatus CheckWorldPredicate::tick() {
    auto pred = getInput<std::string>("predicate");
    if (!pred) return BT::NodeStatus::FAILURE;

    bool expected = true;
    getInput("expected", expected);

    std::string required_authority = "any";
    getInput("required_authority", required_authority);

    auto* wm = config().blackboard->get<WorldModel*>("world_model");
    if (!wm) return BT::NodeStatus::FAILURE;

    if (required_authority == "confirmed") {
        auto meta = wm->getFactMetadata(pred.value());
        if (meta.authority != FactAuthority::CONFIRMED)
            return BT::NodeStatus::FAILURE;
    }

    bool actual = wm->getFact(pred.value());
    return (actual == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace ame
