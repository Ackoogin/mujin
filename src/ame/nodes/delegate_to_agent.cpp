#include <ame/action_registry.h>
#include <ame/bt_nodes/delegate_to_agent.h>
#include <ame/plan_audit_log.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/world_model.h>

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <sstream>
#include <stdexcept>

namespace ame {

DelegateToAgent::DelegateToAgent(const std::string& name,
                                 const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList DelegateToAgent::providedPorts() {
  return {
      BT::InputPort<std::string>("agent_id", "ID of agent to delegate to"),
      BT::InputPort<std::string>(
          "agent_goals",
          "Semicolon-separated goal fluents for this agent"),
  };
}

BT::NodeStatus DelegateToAgent::onStart() {
  auto agent_id_input = getInput<std::string>("agent_id");
  if (!agent_id_input) {
    return BT::NodeStatus::FAILURE;
  }
  agent_id_ = agent_id_input.value();

  auto encoded_goals = getInput<std::string>("agent_goals");
  if (!encoded_goals) {
    return BT::NodeStatus::FAILURE;
  }

  auto goals = parseGoals(encoded_goals.value());
  if (goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // Get required blackboard entries
  WorldModel* wm = nullptr;
  Planner* planner = nullptr;
  PlanCompiler* compiler = nullptr;
  ActionRegistry* registry = nullptr;
  BT::BehaviorTreeFactory* factory_ptr = nullptr;

  try {
    wm = config().blackboard->get<WorldModel*>("world_model");
    planner = config().blackboard->get<Planner*>("planner");
    compiler = config().blackboard->get<PlanCompiler*>("plan_compiler");
    registry = config().blackboard->get<ActionRegistry*>("action_registry");
    factory_ptr = config().blackboard->get<BT::BehaviorTreeFactory*>("bt_factory");
  } catch (const std::exception&) {
    return BT::NodeStatus::FAILURE;
  }

  if (!wm || !planner || !compiler || !registry || !factory_ptr) {
    return BT::NodeStatus::FAILURE;
  }

  // Verify agent exists and is available
  AgentInfo* agent = wm->getAgent(agent_id_);
  if (!agent) {
    return BT::NodeStatus::FAILURE;
  }

  if (!agent->available) {
    return BT::NodeStatus::FAILURE;
  }

  // Mark agent as busy
  agent->available = false;

  // Set agent context in blackboard for action nodes
  config().blackboard->set("current_agent_id", agent_id_);

  // Plan for this agent's goals
  wm->setGoal(goals);
  auto result = planner->solve(*wm);

  if (!result.success || result.steps.empty()) {
    agent->available = true;  // Restore availability on failure
    return BT::NodeStatus::FAILURE;
  }

  // Compile plan to BT, passing agent context
  std::string bt_xml = compiler->compile(result.steps, *wm, *registry, agent_id_);

  // Record audit if available
  PlanAuditLog* audit = nullptr;
  try {
    audit = config().blackboard->get<PlanAuditLog*>("plan_audit_log");
  } catch (const std::exception&) {}

  if (audit) {
    PlanAuditLog::Episode ep;
    ep.phase_name = "delegate:" + agent_id_;
    ep.ts_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    ep.solver = "BRFS";
    ep.solve_time_ms = result.solve_time_ms;
    ep.success = true;
    ep.expanded = result.expanded;
    ep.generated = result.generated;
    ep.cost = result.cost;
    ep.goal_fluents = goals;
    ep.bt_xml = bt_xml;

    for (const auto& step : result.steps) {
      ep.plan_actions.push_back(wm->groundActions()[step.action_index].signature);
    }

    for (unsigned i = 0; i < wm->numFluents(); ++i) {
      if (wm->getFact(i)) {
        ep.init_facts.push_back(wm->fluentName(i));
      }
    }

    try {
      ep.parent_episode_id =
          config().blackboard->get<uint64_t>("parent_episode_id");
    } catch (const std::exception&) {
      ep.parent_episode_id = 0;
    }

    episode_id_ = audit->recordEpisode(std::move(ep));
  }

  // Build subtree
  try {
    auto bb = config().blackboard;
    if (episode_id_ != 0) {
      bb->set("parent_episode_id", episode_id_);
    }
    sub_tree_ =
        std::make_unique<BT::Tree>(factory_ptr->createTreeFromText(bt_xml, bb));
  } catch (const std::exception&) {
    AgentInfo* ag = wm->getAgent(agent_id_);
    if (ag) ag->available = true;
    return BT::NodeStatus::FAILURE;
  }

  return onRunning();
}

BT::NodeStatus DelegateToAgent::onRunning() {
  if (!sub_tree_) {
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus status = sub_tree_->tickOnce();
  if (status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  // Delegation complete - restore agent availability
  WorldModel* wm = nullptr;
  try {
    wm = config().blackboard->get<WorldModel*>("world_model");
  } catch (const std::exception&) {}

  if (wm) {
    AgentInfo* agent = wm->getAgent(agent_id_);
    if (agent) {
      agent->available = true;
    }
  }

  sub_tree_.reset();
  return status;
}

void DelegateToAgent::onHalted() {
  if (sub_tree_) {
    sub_tree_->haltTree();
    sub_tree_.reset();
  }

  // Restore agent availability on halt
  WorldModel* wm = nullptr;
  try {
    wm = config().blackboard->get<WorldModel*>("world_model");
  } catch (const std::exception&) {}

  if (wm) {
    AgentInfo* agent = wm->getAgent(agent_id_);
    if (agent) {
      agent->available = true;
    }
  }
}

std::vector<std::string> DelegateToAgent::parseGoals(const std::string& encoded) {
  std::vector<std::string> goals;
  std::istringstream stream(encoded);
  std::string token;
  while (std::getline(stream, token, ';')) {
    size_t start = token.find_first_not_of(" \t");
    size_t end = token.find_last_not_of(" \t");
    if (start != std::string::npos) {
      goals.push_back(token.substr(start, end - start + 1));
    }
  }
  return goals;
}

} // namespace ame
