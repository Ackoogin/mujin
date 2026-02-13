/**
 * mujin_test_app — Basic integration test for LAPKT + BehaviorTree.CPP
 *
 * Verifies that both libraries compile, link, and function correctly:
 *   1. LAPKT:  Create a STRIPS_Problem, add fluents and an action, verify
 *   2. BT.CPP: Register a custom action node, build a tree from XML, tick it
 */

#include <cstdlib>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// BehaviorTree.CPP v4
// ---------------------------------------------------------------------------
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// ---------------------------------------------------------------------------
// LAPKT core (STRIPS model)
// ---------------------------------------------------------------------------
#include "strips_prob.hxx"
#include "fluent.hxx"
#include "action.hxx"
#include "strips_state.hxx"

// ===== BT.CPP demo nodes ====================================================

/// A trivial synchronous action node that always succeeds.
class SayHello : public BT::SyncActionNode {
public:
    SayHello(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        std::cout << "  [BT] SayHello ticked — returning SUCCESS\n";
        return BT::NodeStatus::SUCCESS;
    }
};

/// A condition node that checks a blackboard predicate (string key → bool).
class CheckPredicate : public BT::ConditionNode {
public:
    CheckPredicate(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("predicate") };
    }

    BT::NodeStatus tick() override {
        auto pred = getInput<std::string>("predicate");
        if (!pred) {
            return BT::NodeStatus::FAILURE;
        }
        // Look up on the blackboard
        auto val = config().blackboard->get<bool>(pred.value());
        std::cout << "  [BT] CheckPredicate(\"" << pred.value()
                  << "\") = " << (val ? "true" : "false") << "\n";
        return val ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// ===== Test functions ========================================================

bool test_lapkt() {
    std::cout << "\n=== LAPKT STRIPS Test ===\n";

    aptk::STRIPS_Problem prob;

    // --- Add fluents (propositions) ---
    aptk::STRIPS_Problem::add_fluent(prob, "(at robot base)");
    aptk::STRIPS_Problem::add_fluent(prob, "(at robot sector_a)");
    aptk::STRIPS_Problem::add_fluent(prob, "(searched sector_a)");

    std::cout << "  Fluents registered: " << prob.num_fluents() << "\n";
    if (prob.num_fluents() != 3) {
        std::cerr << "  FAIL: expected 3 fluents\n";
        return false;
    }

    // --- Add an action: move(robot, base, sector_a) ---
    aptk::Fluent_Vec pre  = { 0 };      // precondition: at(robot, base)
    aptk::Fluent_Vec add  = { 1 };      // add effect:   at(robot, sector_a)
    aptk::Fluent_Vec del  = { 0 };      // del effect:   not at(robot, base)
    aptk::Conditional_Effect_Vec ceffs;  // no conditional effects

    aptk::STRIPS_Problem::add_action(
        prob,
        "move(robot,base,sector_a)",
        pre, add, del, ceffs
    );

    std::cout << "  Actions registered: " << prob.num_actions() << "\n";
    if (prob.num_actions() != 1) {
        std::cerr << "  FAIL: expected 1 action\n";
        return false;
    }

    // --- Create initial state: at(robot, base) ---
    aptk::Fluent_Vec init_fluents = { 0 };
    aptk::STRIPS_Problem::set_init(prob, init_fluents);

    aptk::Fluent_Vec goal_fluents = { 1, 2 };  // at(robot, sector_a) AND searched(sector_a)
    aptk::STRIPS_Problem::set_goal(prob, goal_fluents);

    std::cout << "  Init state set with " << init_fluents.size() << " fluent(s)\n";
    std::cout << "  Goal state set with " << goal_fluents.size() << " fluent(s)\n";
    std::cout << "  PASS: LAPKT STRIPS_Problem created successfully\n";
    return true;
}

bool test_behaviortree() {
    std::cout << "\n=== BehaviorTree.CPP v4 Test ===\n";

    BT::BehaviorTreeFactory factory;

    // Register custom node types
    factory.registerNodeType<SayHello>("SayHello");
    factory.registerNodeType<CheckPredicate>("CheckPredicate");

    // Define a simple tree: Sequence → [CheckPredicate, SayHello]
    static const char* xml_tree = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <CheckPredicate predicate="robot_ready"/>
                    <SayHello/>
                </Sequence>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_tree);

    // Set blackboard value so the condition passes
    tree.rootBlackboard()->set("robot_ready", true);

    std::cout << "  Tree created with " << tree.subtrees.size() << " subtree(s)\n";
    std::cout << "  Ticking tree...\n";

    auto status = tree.tickOnce();

    std::cout << "  Tree status: "
              << BT::toStr(status) << "\n";

    if (status != BT::NodeStatus::SUCCESS) {
        std::cerr << "  FAIL: expected SUCCESS\n";
        return false;
    }

    std::cout << "  PASS: BehaviorTree.CPP working correctly\n";
    return true;
}

// ===== Main ==================================================================

int main() {
    std::cout << "============================================\n";
    std::cout << " MUJIN Integration Test\n";
    std::cout << " LAPKT (PDDL) + BehaviorTree.CPP (BT v4)\n";
    std::cout << "============================================\n";

    bool lapkt_ok = test_lapkt();
    bool bt_ok    = test_behaviortree();

    std::cout << "\n============================================\n";
    std::cout << " Results:\n";
    std::cout << "   LAPKT STRIPS:      " << (lapkt_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "   BehaviorTree.CPP:  " << (bt_ok    ? "PASS" : "FAIL") << "\n";
    std::cout << "============================================\n";

    return (lapkt_ok && bt_ok) ? EXIT_SUCCESS : EXIT_FAILURE;
}
