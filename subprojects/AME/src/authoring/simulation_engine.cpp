#include "simulation_engine.h"

#include "pddl_validator.h"
#include "relation_index.h"

#include <ame/action_registry.h>
#include <ame/bt_logger.h>
#include <ame/bt_nodes/goal_reached.h>
#include <ame/bt_nodes/planned_action_node.h>
#include <ame/fact_authority.h>
#include <ame/plan_compiler.h>
#include <ame/plan_audit_log.h>
#include <ame/planner.h>
#include <ame/world_model.h>
#include <ame/wm_audit_log.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <chrono>
#include <exception>
#include <functional>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

/// The node type every planned action is built as during a simulated run.
constexpr const char* kStandInNodeType = "SimulatedAction";

/// How many action parameters a stand-in node can carry. The plan compiler
/// writes one param0..paramN attribute per grounded argument, and a node only
/// accepts attributes it declares as ports, so this is the widest action a run
/// can build. A project with a wider action is reported before the run starts
/// rather than failing inside the tree builder.
constexpr int kMaxActionParams = 10;

/// \brief Everything the stand-in nodes of one run share.
struct SimulationContext {
  std::unordered_map<std::string, SimulationSettings> settings;
  RunFaultSet faults;
  std::unordered_map<std::string, unsigned> actionAttempts;
  std::mt19937 rng;
};

/// The schema name inside a grounded signature: "move(uav1,base)" -> "move".
std::string schemaNameOf(const std::string& signature) {
  const auto paren = signature.find('(');
  std::string name =
      (paren == std::string::npos) ? signature : signature.substr(0, paren);
  // A disjunctive-precondition action is registered once per alternative under
  // a "#k"-suffixed name; all of them come from the one authored action.
  const auto hash = name.rfind('#');
  if (hash != std::string::npos && hash + 1 < name.size()) {
    const bool allDigits =
        std::all_of(name.begin() + static_cast<long>(hash) + 1, name.end(),
                    [](unsigned char c) { return std::isdigit(c) != 0; });
    if (allDigits) {
      name = name.substr(0, hash);
    }
  }
  return name;
}

std::string factKey(const std::string& predicateName,
                    const std::vector<std::string>& objectNames) {
  std::ostringstream key;
  key << '(' << predicateName;
  for (const auto& objectName : objectNames) {
    key << ' ' << objectName;
  }
  key << ')';
  return key.str();
}

std::vector<std::string> planSignatures(const ame::PlanResult& plan,
                                        const ame::WorldModel& worldModel) {
  std::vector<std::string> signatures;
  const auto& actions = worldModel.groundActions();
  signatures.reserve(plan.steps.size());
  for (const auto& step : plan.steps) {
    signatures.push_back(step.action_index < actions.size()
                             ? actions[step.action_index].signature
                             : "action #" + std::to_string(step.action_index));
  }
  return signatures;
}

/// \brief The stand-in that every action of a simulated run is built as.
///
/// It inherits the whole planned-action contract from `ame::PlannedActionNode`,
/// so it checks the same grounded preconditions a deployed node would check and
/// records the same declared effects. What it does not do is any real work: it
/// waits for the number of ticks the project configured for that action, and
/// then reports the outcome the project configured.
class StandInAction : public ame::PlannedActionNode {
public:
  StandInAction(const std::string& name,
                const BT::NodeConfiguration& config,
                SimulationContext* context)
      : PlannedActionNode(name, config), m_context(context) {}

  static BT::PortsList providedPorts() {
    BT::PortsList ports;
    for (int i = 0; i < kMaxActionParams; ++i) {
      ports.insert(BT::InputPort<std::string>("param" + std::to_string(i), "",
                                              "Grounded action argument"));
    }
    return withBasePorts(std::move(ports));
  }

protected:
  BT::NodeStatus onActionStart() override {
    const SimulationSettings settings = settingsForThisAction();
    m_configuredTicks = settings.ticks < 1 ? 1 : settings.ticks;
    m_completedTicks = 0;
    m_succeeds = settings.succeeds;
    if (m_succeeds && settings.failureChance > 0.0 && m_context != nullptr) {
      std::uniform_real_distribution<double> draw(0.0, 1.0);
      m_succeeds = draw(m_context->rng) >= settings.failureChance;
    }
    if (m_context != nullptr) {
      const std::string action_name = schemaNameOf(name());
      const unsigned attempt = ++m_context->actionAttempts[action_name];
      const bool forced = std::any_of(
          m_context->faults.actionFailures.begin(),
          m_context->faults.actionFailures.end(),
          [&](const ForcedActionFailure& failure) {
            return failure.actionName == action_name &&
                   failure.attempt == attempt;
          });
      if (forced) {
        m_succeeds = false;
      }
    }
    return advance();
  }

  BT::NodeStatus onActionRunning() override { return advance(); }

  void onActionHalted() override { m_completedTicks = 0; }

private:
  SimulationSettings settingsForThisAction() const {
    if (m_context == nullptr) {
      return SimulationSettings{};
    }
    const auto it = m_context->settings.find(schemaNameOf(name()));
    if (it == m_context->settings.end()) {
      return SimulationSettings{};
    }
    return it->second;
  }

  BT::NodeStatus advance() {
    ++m_completedTicks;
    if (m_completedTicks < m_configuredTicks) {
      return BT::NodeStatus::RUNNING;
    }
    return m_succeeds ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  SimulationContext* m_context = nullptr;
  int m_configuredTicks = 1;
  int m_completedTicks = 0;
  bool m_succeeds = true;
};

const ScenarioDef* findScenario(const ProjectModel& model,
                                const std::string& scenarioName) {
  const auto it = std::find_if(model.scenarios.begin(), model.scenarios.end(),
                               [&scenarioName](const ScenarioDef& scenario) {
                                 return scenario.name == scenarioName;
                               });
  return it == model.scenarios.end() ? nullptr : &(*it);
}

RunNodeStatus statusFromBt(BT::NodeStatus status) {
  switch (status) {
  case BT::NodeStatus::RUNNING:
    return RunNodeStatus::Happening;
  case BT::NodeStatus::SUCCESS:
    return RunNodeStatus::Finished;
  case BT::NodeStatus::FAILURE:
    return RunNodeStatus::WentWrong;
  default:
    return RunNodeStatus::Waiting;
  }
}

}  // namespace

/// \brief The parts of a loaded run that only exist while it is loaded.
struct SimulationEngine::Run {
  ame::WorldModel worldModel;
  ame::PlanResult plan;
  SimulationContext context;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;
  std::unique_ptr<ame::AmeBTLogger> btLogger;
  ame::WmAuditLog wmAudit;
  ame::PlanAuditLog planAudit;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers;
  std::unordered_map<uint16_t, size_t> nodeIndexByUid;
  // Which action step each entry of the node list belongs to, for the entries
  // that are actions at all.
  std::unordered_map<size_t, size_t> stepIndexByNodeIndex;
  std::vector<unsigned> goalIds;
  std::unordered_set<size_t> appliedFactFaults;
};

const char* runPhaseName(RunPhase phase) {
  switch (phase) {
  case RunPhase::NotStarted:
    return "not started";
  case RunPhase::Running:
    return "running";
  case RunPhase::Held:
    return "held";
  case RunPhase::Completed:
    return "completed";
  case RunPhase::Failed:
    return "went wrong";
  case RunPhase::Error:
    return "could not run";
  }
  return "could not run";
}

const char* runNodeStatusName(RunNodeStatus status) {
  switch (status) {
  case RunNodeStatus::Waiting:
    return "waiting";
  case RunNodeStatus::Happening:
    return "happening now";
  case RunNodeStatus::Finished:
    return "finished";
  case RunNodeStatus::WentWrong:
    return "went wrong";
  }
  return "waiting";
}

SimulationEngine::SimulationEngine() = default;
SimulationEngine::~SimulationEngine() = default;

bool SimulationEngine::start(const ProjectModel& model,
                             const std::string& scenarioName) {
  m_model = model;
  m_scenarioName = scenarioName;
  return prepare();
}

void SimulationEngine::setFaults(RunFaultSet faults) {
  for (auto& failure : faults.actionFailures) {
    failure.attempt = std::max(1U, failure.attempt);
  }
  for (auto& change : faults.factChanges) {
    change.tick = std::max(1U, change.tick);
  }
  m_faults = std::move(faults);
}

bool SimulationEngine::startAgain() {
  if (m_scenarioName.empty()) {
    m_error = "no scenario has been run yet";
    m_phase = RunPhase::Error;
    return false;
  }
  return prepare();
}

void SimulationEngine::clearRunState() {
  m_run.reset();
  m_nodes.clear();
  m_actionSteps.clear();
  m_factChanges.clear();
  m_faultExplanations.clear();
  m_replans.clear();
  m_btAuditEvents.clear();
  m_wmAuditEvents.clear();
  m_planAuditEvents.clear();
  m_factNames.clear();
  m_startingFacts.clear();
  m_compiledXml.clear();
  m_error.clear();
  m_tick = 0;
  m_tickAccumulator = 0.0;
  m_phase = RunPhase::NotStarted;
}

bool SimulationEngine::prepare() {
  clearRunState();

  const ScenarioDef* scenario = findScenario(m_model, m_scenarioName);
  if (scenario == nullptr) {
    m_error = "there is no scenario called '" + m_scenarioName + "'";
    m_phase = RunPhase::Error;
    return false;
  }

  for (const auto& action : m_model.actions) {
    if (static_cast<int>(action.params.size()) > kMaxActionParams) {
      m_error = "the action '" + action.name + "' has more than " +
                std::to_string(kMaxActionParams) +
                " parameters, which a simulated run cannot build";
      m_phase = RunPhase::Error;
      return false;
    }
  }

  auto run = std::make_unique<Run>();

  const ValidationReport validation = PddlValidator::validateAndBuildWorldModel(
      m_model, m_scenarioName, run->worldModel);
  if (!validation.ok) {
    m_error = validation.errors.empty()
                  ? "the model could not be read"
                  : validation.errors.front().message;
    m_phase = RunPhase::Error;
    return false;
  }

  run->worldModel.setAuditCallback(
      [this, run_ptr = run.get()](uint64_t version, uint64_t timestamp,
                                  const std::string& fact, bool value,
                                  const std::string& source) {
        run_ptr->wmAudit.onFactChange(version, timestamp, fact, value, source);
        nlohmann::json event = {
            {"wm_version", version}, {"ts_us", timestamp}, {"fact", fact},
            {"value", value}, {"source", source}};
        m_wmAuditEvents.push_back({m_tick, event.dump()});
        if (m_tick != 0U) {
          m_factChanges.push_back(RunFactChange{m_tick, fact, value, source});
        }
      });

  // The scenario says what is actually true when the mission starts, so its
  // facts are recorded as observed. Everything a stand-in action goes on to
  // record is merely predicted. An action that the domain says needs observed
  // evidence therefore behaves in a run the way it would in the field: it will
  // not proceed on a fact that only a plan effect produced.
  for (const auto& fact : scenario->initialState) {
    const std::string key = factKey(fact.predicateName, fact.objectNames);
    run->worldModel.setFact(key, true,
                            "scenario", ame::FactAuthority::CONFIRMED);
    m_startingFacts.push_back(key);
  }

  m_factNames.reserve(run->worldModel.numFluents());
  for (unsigned id = 0; id < run->worldModel.numFluents(); ++id) {
    m_factNames.push_back(run->worldModel.fluentName(id));
  }

  try {
    ame::Planner planner;
    run->plan = planner.solve(run->worldModel);
  } catch (const std::exception& ex) {
    m_error = std::string("the planner could not run: ") + ex.what();
    m_phase = RunPhase::Error;
    return false;
  }

  if (!run->plan.success) {
    m_error = "there is no plan for this scenario, so there is nothing to run";
    m_phase = RunPhase::Error;
    return false;
  }

  for (const auto& action : m_model.actions) {
    run->context.settings[action.name] = action.simulation;
  }
  run->context.faults = m_faults;
  run->context.rng.seed(m_model.simulationSeed);
  run->factory.registerNodeType<StandInAction>(kStandInNodeType,
                                               &run->context);
  run->factory.registerNodeType<ame::GoalReached>("GoalReached");
  run->goalIds = run->worldModel.goalFluentIds();
  m_run = std::move(run);
  if (!buildTreeForPlan(m_run->plan, true)) {
    return false;
  }
  buildFaultExplanations();

  m_phase = RunPhase::Running;
  return true;
}

bool SimulationEngine::buildTreeForPlan(const ame::PlanResult& plan,
                                        bool firstPlan) {
  if (m_run == nullptr) {
    return false;
  }

  ame::ActionRegistry registry;
  for (const auto& action : m_model.actions) {
    // Keeping the authored reactive flag makes a fact lost while an action is
    // running fail at the same point that it would in the runtime.
    registry.registerAction(action.name, kStandInNodeType,
                            action.btBinding.reactive);
  }

  try {
    ame::PlanCompiler compiler;
    compiler.setStubUnregisteredActions(true);
    m_compiledXml = compiler.compile(plan.steps, m_run->worldModel, registry);
  } catch (const std::exception& ex) {
    m_error = std::string("the plan could not be compiled: ") + ex.what();
    m_phase = RunPhase::Error;
    return false;
  }

  m_run->btLogger.reset();
  m_run->subscribers.clear();
  m_run->tree.reset();
  m_run->nodeIndexByUid.clear();
  m_run->stepIndexByNodeIndex.clear();
  m_nodes.clear();
  if (firstPlan) {
    m_actionSteps.clear();
  }

  try {
    m_run->tree = std::make_unique<BT::Tree>(
        m_run->factory.createTreeFromText(m_compiledXml));
  } catch (const std::exception& ex) {
    m_error = std::string("the compiled tree could not be built: ") + ex.what();
    m_phase = RunPhase::Error;
    return false;
  }
  m_run->tree->rootBlackboard()->set("world_model", &m_run->worldModel);
  m_run->plan = plan;

  // Only the current tree is drawn, but its action rows are appended to the
  // run timeline. That keeps the abandoned work visible beside what replaced
  // it without asking the drawing code to own execution history.
  const std::function<void(BT::TreeNode*, int)> walk =
      [&](BT::TreeNode* node, int depth) {
        if (node == nullptr) {
          return;
        }
        const size_t node_index = m_nodes.size();
        RunNode entry;
        entry.name = node->name();
        entry.nodeType = node->registrationName();
        entry.depth = depth;
        entry.isAction = entry.nodeType == kStandInNodeType;
        m_nodes.push_back(std::move(entry));
        m_run->nodeIndexByUid[node->UID()] = node_index;

        if (m_nodes[node_index].isAction) {
          RunActionStep step;
          step.signature = node->name();
          step.actionName = schemaNameOf(step.signature);
          const auto settings = m_run->context.settings.find(step.actionName);
          if (settings != m_run->context.settings.end()) {
            step.configuredTicks = std::max(1, settings->second.ticks);
          }
          m_run->stepIndexByNodeIndex[node_index] = m_actionSteps.size();
          m_actionSteps.push_back(std::move(step));
        }

        m_run->subscribers.push_back(node->subscribeToStatusChange(
            [this, node_index](BT::TimePoint, const BT::TreeNode&,
                               BT::NodeStatus, BT::NodeStatus status) {
              recordStatusChange(node_index, static_cast<int>(status));
            }));

        if (auto* control = dynamic_cast<BT::ControlNode*>(node)) {
          for (auto* child : control->children()) {
            walk(child, depth + 1);
          }
        } else if (auto* decorator = dynamic_cast<BT::DecoratorNode*>(node)) {
          walk(decorator->child(), depth + 1);
        }
      };
  walk(m_run->tree->rootNode(), 0);

  m_run->btLogger = std::make_unique<ame::AmeBTLogger>(
      *m_run->tree, "MissionPlan", &m_run->worldModel);
  m_run->btLogger->addCallbackSink([this](const std::string& json) {
    m_btAuditEvents.push_back({m_tick, json});
  });

  recordPlanningEpisode(plan, m_compiledXml);
  return true;
}

void SimulationEngine::recordPlanningEpisode(const ame::PlanResult& plan,
                                             const std::string& btXml) {
  if (m_run == nullptr) {
    return;
  }
  ame::PlanAuditLog::Episode episode;
  episode.ts_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
  for (unsigned id = 0; id < m_run->worldModel.numFluents(); ++id) {
    if (m_run->worldModel.getFact(id)) {
      episode.init_facts.push_back(m_run->worldModel.fluentName(id));
    }
  }
  for (const unsigned id : m_run->worldModel.goalFluentIds()) {
    episode.goal_fluents.push_back(m_run->worldModel.fluentName(id));
  }
  episode.solver = "BRFS";
  episode.solve_time_ms = plan.solve_time_ms;
  episode.success = plan.success;
  episode.expanded = plan.expanded;
  episode.generated = plan.generated;
  episode.cost = plan.cost;
  episode.plan_actions = planSignatures(plan, m_run->worldModel);
  episode.bt_xml = btXml;
  const uint64_t episode_id = m_run->planAudit.recordEpisode(episode);
  nlohmann::json plan_json = {
      {"episode_id", episode_id},
      {"parent_episode_id", 0},
      {"ts_us", episode.ts_us},
      {"solver", episode.solver},
      {"solve_time_ms", episode.solve_time_ms},
      {"success", episode.success},
      {"expanded", episode.expanded},
      {"generated", episode.generated},
      {"cost", episode.cost},
      {"init_facts", episode.init_facts},
      {"goal_fluents", episode.goal_fluents},
      {"plan_actions", episode.plan_actions},
      {"bt_xml", episode.bt_xml},
      {"heuristic_source", episode.heuristic_source},
      {"goal_source", episode.goal_source},
      {"repair_source", episode.repair_source}};
  m_planAuditEvents.push_back({m_tick, plan_json.dump()});
}

void SimulationEngine::applyFactFaultsForCurrentTick() {
  if (m_run == nullptr) {
    return;
  }
  for (size_t i = 0; i < m_faults.factChanges.size(); ++i) {
    const ScheduledFactChange& change = m_faults.factChanges[i];
    if (change.tick != m_tick ||
        m_run->appliedFactFaults.find(i) != m_run->appliedFactFaults.end()) {
      continue;
    }

    const std::string target = factKey(change.fact.predicateName,
                                       change.fact.objectNames);
    const std::string source = m_faults.name.empty()
                                   ? "injected fact change"
                                   : "injected fault: " + m_faults.name;
    try {
      if (change.value) {
        for (const StateGroupDef& group : m_model.stateGroups) {
          if (std::find(group.predicateNames.begin(), group.predicateNames.end(),
                        change.fact.predicateName) ==
              group.predicateNames.end()) {
            continue;
          }
          for (const std::string& alternative : group.predicateNames) {
            if (alternative == change.fact.predicateName) {
              continue;
            }
            const std::string alternative_key =
                factKey(alternative, change.fact.objectNames);
            try {
              m_run->worldModel.setFact(alternative_key, false, source,
                                        ame::FactAuthority::CONFIRMED);
            } catch (const std::exception&) {
              // A state group may contain predicates with a different shape.
              // Only grounded alternatives for the same objects are cleared.
            }
          }
        }
      }
      m_run->worldModel.setFact(target, change.value, source,
                                ame::FactAuthority::CONFIRMED);
      m_run->appliedFactFaults.insert(i);
    } catch (const std::exception& ex) {
      m_error = std::string("the injected fact could not be set: ") + ex.what();
      m_phase = RunPhase::Error;
      return;
    }
  }
}

void SimulationEngine::buildFaultExplanations() {
  m_faultExplanations.clear();
  const RelationIndex index(m_model);

  for (const ForcedActionFailure& failure : m_faults.actionFailures) {
    FaultEffectExplanation explanation;
    explanation.faultName = m_faults.name;
    explanation.affectedAction = failure.actionName;
    explanation.summary = "The " + failure.actionName + " action will fail on attempt " +
                          std::to_string(failure.attempt) +
                          " and will work normally afterwards.";
    const int action_index = index.actionIndex(failure.actionName);
    if (action_index >= 0) {
      const ActionRelations& relations =
          index.action(static_cast<size_t>(action_index));
      for (const size_t responder : relations.mayBeEnabledBy) {
        explanation.respondingActions.push_back(m_model.actions[responder].name);
      }
    }
    m_faultExplanations.push_back(std::move(explanation));
  }

  for (const ScheduledFactChange& change : m_faults.factChanges) {
    FaultEffectExplanation explanation;
    explanation.faultName = m_faults.name;
    std::string affected_predicate = change.fact.predicateName;
    if (change.value) {
      for (const StateGroupDef& group : m_model.stateGroups) {
        if (std::find(group.predicateNames.begin(), group.predicateNames.end(),
                      change.fact.predicateName) != group.predicateNames.end()) {
          for (const std::string& alternative : group.predicateNames) {
            if (alternative != change.fact.predicateName) {
              affected_predicate = alternative;
              break;
            }
          }
        }
      }
    }
    // A no-fault copy gives the same answer for sequential and parallel trees.
    // Stop immediately before the injected tick, because the question is what
    // is already under way when the outside event arrives.
    SimulationEngine baseline;
    if (baseline.start(m_model, m_scenarioName)) {
      while (baseline.tick() + 1U < change.tick && baseline.stepOnce()) {
      }
      const std::vector<std::string> happening = baseline.happeningNow();
      if (!happening.empty()) {
        explanation.affectedAction = happening.front();
      }
    }
    if (change.value && !explanation.affectedAction.empty() && m_run != nullptr) {
      const auto ground_action = std::find_if(
          m_run->worldModel.groundActions().begin(),
          m_run->worldModel.groundActions().end(),
          [&explanation](const ame::GroundAction& action) {
            return action.signature == explanation.affectedAction;
          });
      if (ground_action != m_run->worldModel.groundActions().end()) {
        for (const StateGroupDef& group : m_model.stateGroups) {
          if (std::find(group.predicateNames.begin(), group.predicateNames.end(),
                        change.fact.predicateName) == group.predicateNames.end()) {
            continue;
          }
          for (const unsigned precondition : ground_action->preconditions) {
            const std::string& required =
                m_run->worldModel.fluentName(precondition);
            for (const std::string& alternative : group.predicateNames) {
              if (alternative != change.fact.predicateName &&
                  required == factKey(alternative, change.fact.objectNames)) {
                affected_predicate = alternative;
              }
            }
          }
        }
      }
    }
    explanation.lostPrecondition =
        factKey(affected_predicate, change.fact.objectNames);
    explanation.summary = "At tick " + std::to_string(change.tick) + ", " +
                          factKey(change.fact.predicateName,
                                  change.fact.objectNames) +
                          " will be made " +
                          (change.value ? "true" : "false") + ".";
    if (!explanation.affectedAction.empty()) {
      explanation.summary += " The run expects " + explanation.affectedAction +
                             " to be part-way through.";
    }

    std::unordered_set<std::string> responders;
    const auto add_responders = [&](const std::string& predicate_name) {
      const int predicate_index = index.predicateIndex(predicate_name);
      if (predicate_index < 0) {
        return;
      }
      const PredicateRelations& relations =
          index.predicate(static_cast<size_t>(predicate_index));
      const auto add = [&](const std::vector<PredicateActionRelation>& entries) {
        for (const auto& entry : entries) {
          responders.insert(m_model.actions[entry.actionIndex].name);
        }
      };
      add(relations.requiredBy);
      add(relations.madeTrueBy);
      add(relations.madeFalseBy);
    };
    add_responders(change.fact.predicateName);
    add_responders(affected_predicate);
    explanation.respondingActions.assign(responders.begin(), responders.end());
    std::sort(explanation.respondingActions.begin(),
              explanation.respondingActions.end());
    m_faultExplanations.push_back(std::move(explanation));
  }
}

FailureExplanation SimulationEngine::explainCurrentFailure() const {
  FailureExplanation best;
  if (m_run == nullptr) {
    return best;
  }
  const ScenarioDef* original = findScenario(m_model, m_scenarioName);
  if (original == nullptr) {
    return best;
  }

  ScenarioDef current = *original;
  current.initialState.clear();
  for (unsigned id = 0; id < m_run->worldModel.numFluents(); ++id) {
    if (!m_run->worldModel.getFact(id)) {
      continue;
    }
    const std::string& key = m_run->worldModel.fluentName(id);
    if (key.size() < 2U || key.front() != '(' || key.back() != ')') {
      continue;
    }
    std::istringstream words(key.substr(1U, key.size() - 2U));
    FactRef fact;
    words >> fact.predicateName;
    std::string object;
    while (words >> object) {
      fact.objectNames.push_back(object);
    }
    current.initialState.push_back(std::move(fact));
  }

  const RelationIndex index(m_model);
  for (size_t goal_index = 0; goal_index < current.goals.size(); ++goal_index) {
    const std::string key = factKey(current.goals[goal_index].predicateName,
                                    current.goals[goal_index].objectNames);
    if (m_run->worldModel.getFact(key)) {
      continue;
    }
    FailureExplanation candidate =
        FailureExplainer::explain(m_model, index, current, goal_index);
    if (candidate.rows.size() >= best.rows.size()) {
      best = std::move(candidate);
    }
  }
  return best;
}

bool SimulationEngine::replanAfterFailure() {
  if (m_run == nullptr) {
    return false;
  }
  ReplanEvent event;
  event.tick = m_tick;
  event.abandonedPlan = planSignatures(m_run->plan, m_run->worldModel);
  std::string failed_action = "a step";
  for (auto it = m_actionSteps.rbegin(); it != m_actionSteps.rend(); ++it) {
    if (it->status == RunNodeStatus::WentWrong && it->endTick == m_tick) {
      failed_action = it->signature;
      break;
    }
  }
  event.reason = "At tick " + std::to_string(m_tick) + ", " + failed_action +
                 " failed. The mission replanned from the world as it stood.";

  constexpr size_t kMaxReplansPerRun = 10U;
  if (m_replans.size() + 1U >= kMaxReplansPerRun) {
    event.reason += " The run stopped after ten replans without making progress.";
    m_replans.push_back(std::move(event));
    m_error = m_replans.back().reason;
    m_phase = RunPhase::Failed;
    return false;
  }

  ame::PlanResult replacement;
  try {
    ame::Planner planner;
    replacement = planner.solve(m_run->worldModel);
  } catch (const std::exception& ex) {
    event.reason += " The planner could not run: " + std::string(ex.what());
    m_replans.push_back(std::move(event));
    m_error = m_replans.back().reason;
    m_phase = RunPhase::Error;
    return false;
  }

  event.replacementFound = replacement.success;
  if (!replacement.success) {
    recordPlanningEpisode(replacement, "");
    event.failureExplanation = explainCurrentFailure();
    event.reason += " No replacement plan exists.";
    m_replans.push_back(std::move(event));
    m_error = m_replans.back().reason;
    m_phase = RunPhase::Failed;
    return false;
  }

  event.replacementPlan = planSignatures(replacement, m_run->worldModel);
  m_replans.push_back(event);
  if (!buildTreeForPlan(replacement, false)) {
    return false;
  }
  m_phase = RunPhase::Running;
  return true;
}

void SimulationEngine::recordStatusChange(size_t nodeIndex, int btStatus) {
  const auto status = static_cast<BT::NodeStatus>(btStatus);
  // A tree resets its nodes to IDLE when a branch is halted or the run ends.
  // Showing that would wipe the picture of what happened just as the user comes
  // to read it, so the last thing a node actually did is what stays on screen.
  if (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::SKIPPED) {
    return;
  }
  if (nodeIndex >= m_nodes.size()) {
    return;
  }

  const RunNodeStatus runStatus = statusFromBt(status);
  m_nodes[nodeIndex].status = runStatus;

  if (!m_nodes[nodeIndex].isAction || m_run == nullptr) {
    return;
  }

  // Steps are found by the node's position in the tree rather than by its name,
  // because a plan may use the same grounded action twice and the two would
  // then share a name.
  const auto stepIt = m_run->stepIndexByNodeIndex.find(nodeIndex);
  if (stepIt == m_run->stepIndexByNodeIndex.end() ||
      stepIt->second >= m_actionSteps.size()) {
    return;
  }

  RunActionStep& step = m_actionSteps[stepIt->second];
  if (runStatus == RunNodeStatus::Happening) {
    if (step.status != RunNodeStatus::Happening) {
      step.startTick = m_tick;
    }
  } else {
    step.endTick = m_tick;
    if (step.startTick == 0) {
      step.startTick = m_tick;
    }
  }
  step.status = runStatus;
}

void SimulationEngine::stop() {
  if (m_run == nullptr || isFinished()) {
    return;
  }
  m_run->tree->haltTree();
  m_phase = RunPhase::Failed;
  if (m_error.empty()) {
    m_error = "the run was stopped before the mission finished";
  }
}

void SimulationEngine::hold() {
  if (m_phase == RunPhase::Running) {
    m_phase = RunPhase::Held;
    m_tickAccumulator = 0.0;
  }
}

void SimulationEngine::resume() {
  if (m_phase == RunPhase::Held) {
    m_phase = RunPhase::Running;
    m_tickAccumulator = 0.0;
  }
}

bool SimulationEngine::isFinished() const {
  return m_phase == RunPhase::Completed || m_phase == RunPhase::Failed ||
         m_phase == RunPhase::Error;
}

bool SimulationEngine::stepOnce() {
  if (m_run == nullptr || m_run->tree == nullptr || isFinished()) {
    return false;
  }

  if (m_tick >= m_tickLimit) {
    m_error = "the run passed its limit of " + std::to_string(m_tickLimit) +
              " ticks and was abandoned";
    m_phase = RunPhase::Error;
    return false;
  }

  ++m_tick;
  applyFactFaultsForCurrentTick();
  if (m_phase == RunPhase::Error) {
    return false;
  }
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  try {
    status = m_run->tree->tickExactlyOnce();
  } catch (const std::exception& ex) {
    m_error = std::string("the run stopped: ") + ex.what();
    m_phase = RunPhase::Error;
    return false;
  }

  if (status == BT::NodeStatus::SUCCESS) {
    m_phase = RunPhase::Completed;
    return false;
  }
  if (status == BT::NodeStatus::FAILURE) {
    return replanAfterFailure();
  }
  return true;
}

bool SimulationEngine::runToCompletion() {
  while (stepOnce()) {
    // stepOnce() reports when the run has somewhere left to go.
  }
  return m_phase == RunPhase::Completed;
}

void SimulationEngine::advance(double elapsedSeconds) {
  if (m_phase != RunPhase::Running || elapsedSeconds <= 0.0) {
    return;
  }

  m_tickAccumulator += elapsedSeconds * m_ticksPerSecond;
  // A frame that took a long time, because the window was hidden or the machine
  // was busy, must not make the run leap forward past what anyone can watch.
  const int kMaxTicksPerCall = 100;
  int ticked = 0;
  while (m_tickAccumulator >= 1.0 && ticked < kMaxTicksPerCall) {
    m_tickAccumulator -= 1.0;
    ++ticked;
    if (!stepOnce()) {
      m_tickAccumulator = 0.0;
      return;
    }
  }
  if (ticked >= kMaxTicksPerCall) {
    m_tickAccumulator = 0.0;
  }
}

void SimulationEngine::setTicksPerSecond(double ticksPerSecond) {
  m_ticksPerSecond = ticksPerSecond < 0.1 ? 0.1 : ticksPerSecond;
}

void SimulationEngine::setTickLimit(unsigned ticks) {
  m_tickLimit = ticks == 0 ? 1 : ticks;
}

std::vector<RunGoal> SimulationEngine::goals() const {
  std::vector<RunGoal> goals;
  if (m_run == nullptr) {
    return goals;
  }
  goals.reserve(m_run->goalIds.size());
  for (const unsigned id : m_run->goalIds) {
    goals.push_back(
        RunGoal{m_run->worldModel.fluentName(id), m_run->worldModel.getFact(id)});
  }
  return goals;
}

size_t SimulationEngine::goalsMetCount() const {
  const std::vector<RunGoal> current = goals();
  return static_cast<size_t>(
      std::count_if(current.begin(), current.end(),
                    [](const RunGoal& goal) { return goal.met; }));
}

std::vector<std::string> SimulationEngine::happeningNow() const {
  std::vector<std::string> running;
  for (const auto& step : m_actionSteps) {
    if (step.status == RunNodeStatus::Happening) {
      running.push_back(step.signature);
    }
  }
  return running;
}

size_t SimulationEngine::actionsFinishedCount() const {
  return static_cast<size_t>(
      std::count_if(m_actionSteps.begin(), m_actionSteps.end(),
                    [](const RunActionStep& step) {
                      return step.status == RunNodeStatus::Finished;
                    }));
}

size_t SimulationEngine::actionsRunCount() const {
  return static_cast<size_t>(
      std::count_if(m_actionSteps.begin(), m_actionSteps.end(),
                    [](const RunActionStep& step) {
                      return step.status != RunNodeStatus::Waiting;
                    }));
}

std::vector<std::string> SimulationEngine::actionsRun() const {
  std::vector<std::string> actions;
  for (const RunActionStep& step : m_actionSteps) {
    if (step.status != RunNodeStatus::Waiting) {
      actions.push_back(step.actionName);
    }
  }
  return actions;
}

RunState SimulationEngine::stateAtTick(unsigned requestedTick) const {
  RunState state;
  state.tick = std::min(requestedTick, m_tick);
  state.facts.reserve(m_factNames.size());

  for (const std::string& fact : m_factNames) {
    const bool startsTrue =
        std::find(m_startingFacts.begin(), m_startingFacts.end(), fact) !=
        m_startingFacts.end();
    state.facts.push_back(RunFactState{fact, startsTrue, 0, false});
  }

  for (const RunFactChange& change : m_factChanges) {
    if (change.tick > state.tick) {
      break;
    }
    const auto found = std::find_if(
        state.facts.begin(), state.facts.end(),
        [&change](const RunFactState& fact) { return fact.fact == change.fact; });
    if (found != state.facts.end()) {
      found->value = change.value;
      found->lastChangedTick = change.tick;
      found->changedDuringRun = true;
    } else {
      state.facts.push_back(
          RunFactState{change.fact, change.value, change.tick, true});
    }
  }

  state.actionSteps = m_actionSteps;
  for (RunActionStep& step : state.actionSteps) {
    const RunNodeStatus finalStatus = step.status;
    if (step.startTick == 0 || state.tick < step.startTick) {
      step.status = RunNodeStatus::Waiting;
      continue;
    }
    if (step.endTick == 0 || state.tick < step.endTick) {
      step.status = RunNodeStatus::Happening;
      continue;
    }
    step.status = finalStatus;
  }
  return state;
}

std::string SimulationEngine::summaryLine() const {
  if (m_run == nullptr) {
    return m_error.empty() ? "No run loaded" : m_error;
  }
  std::ostringstream line;
  line << "tick " << m_tick << " · " << actionsFinishedCount() << " of "
       << m_actionSteps.size() << " actions done · " << goalsMetCount()
       << " of " << goals().size() << " goals met · " << replanCount()
       << " replans";
  return line.str();
}

std::string SimulationEngine::toJson() const {
  nlohmann::json json;
  json["scenario"] = m_scenarioName;
  // A run and a recording of a real mission are read the same way and shown on
  // the same screens, so the data itself has to say which it is. Anything
  // quoting a run has this to quote.
  json["simulated"] = true;
  json["phase"] = runPhaseName(m_phase);
  json["error"] = m_error;
  json["ticks"] = m_tick;
  json["seed"] = m_model.simulationSeed;
  json["faultName"] = m_faults.name;
  json["faults"] = m_faults;
  json["replanCount"] = replanCount();
  json["actionCount"] = m_actionSteps.size();
  json["actionsRun"] = actionsRunCount();
  json["actionsFinished"] = actionsFinishedCount();

  const std::vector<RunGoal> currentGoals = goals();
  json["goalCount"] = currentGoals.size();
  json["goalsMet"] = goalsMetCount();
  json["goals"] = nlohmann::json::array();
  for (const auto& goal : currentGoals) {
    json["goals"].push_back({{"fact", goal.fact}, {"met", goal.met}});
  }

  json["actions"] = nlohmann::json::array();
  for (const auto& step : m_actionSteps) {
    json["actions"].push_back({
        {"signature", step.signature},
        {"action", step.actionName},
        {"status", runNodeStatusName(step.status)},
        {"startTick", step.startTick},
        {"endTick", step.endTick},
        {"configuredTicks", step.configuredTicks},
    });
  }

  json["factChanges"] = nlohmann::json::array();
  for (const auto& change : m_factChanges) {
    json["factChanges"].push_back({
        {"tick", change.tick},
        {"fact", change.fact},
        {"value", change.value},
        {"source", change.source},
    });
  }

  json["replans"] = nlohmann::json::array();
  for (const ReplanEvent& replan : m_replans) {
    nlohmann::json explanation = nlohmann::json::array();
    for (const FailureExplanationRow& row : replan.failureExplanation.rows) {
      explanation.push_back(row.text);
    }
    json["replans"].push_back({
        {"tick", replan.tick},
        {"reason", replan.reason},
        {"abandonedPlan", replan.abandonedPlan},
        {"replacementPlan", replan.replacementPlan},
        {"replacementFound", replan.replacementFound},
        {"failureExplanation", explanation},
    });
  }

  return json.dump(2);
}
