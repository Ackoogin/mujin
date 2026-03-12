#pragma once

#include <ame/action_registry.h>
#include <ame/world_model.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

class StubMoveAction : public BT::SyncActionNode {
public:
  StubMoveAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
            BT::InputPort<std::string>("param2")};
  }
};

class StubSearchAction : public BT::SyncActionNode {
public:
  StubSearchAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1")};
  }
};

class StubClassifyAction : public BT::SyncActionNode {
public:
  StubClassifyAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1")};
  }
};

inline ame::WorldModel buildUavWorldModel() {
  ame::WorldModel wm;
  auto& ts = wm.typeSystem();
  ts.addType("object");
  ts.addType("location", "object");
  ts.addType("sector", "location");
  ts.addType("robot", "object");

  wm.addObject("uav1", "robot");
  wm.addObject("base", "location");
  wm.addObject("sector_a", "sector");

  wm.registerPredicate("at", {"robot", "location"});
  wm.registerPredicate("searched", {"sector"});
  wm.registerPredicate("classified", {"sector"});

  wm.registerAction("move",
                    {"?r", "?from", "?to"},
                    {"robot", "location", "location"},
                    {"(at ?r ?from)"},
                    {"(at ?r ?to)"},
                    {"(at ?r ?from)"});
  wm.registerAction("search",
                    {"?r", "?s"},
                    {"robot", "sector"},
                    {"(at ?r ?s)"},
                    {"(searched ?s)"},
                    {});
  wm.registerAction("classify",
                    {"?r", "?s"},
                    {"robot", "sector"},
                    {"(at ?r ?s)", "(searched ?s)"},
                    {"(classified ?s)"},
                    {});
  wm.setFact("(at uav1 base)", true, "test");
  return wm;
}

inline void registerUavActions(ame::ActionRegistry& registry) {
  registry.registerAction("move", "StubMoveAction");
  registry.registerAction("search", "StubSearchAction");
  registry.registerAction("classify", "StubClassifyAction");
}

inline void registerUavStubNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<StubMoveAction>("StubMoveAction");
  factory.registerNodeType<StubSearchAction>("StubSearchAction");
  factory.registerNodeType<StubClassifyAction>("StubClassifyAction");
}
