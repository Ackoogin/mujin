#include <gtest/gtest.h>

#include "project_model.h"
#include "simulation_engine.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <utility>

namespace {

/// A two-step mission: the vehicle moves to a sector and searches it.
ProjectModel makeSearchModel() {
  ProjectModel model;
  model.projectName = "uav-search";
  model.types.push_back({"location", "object"});
  model.types.push_back({"sector", "location"});
  model.types.push_back({"robot", "object"});

  model.predicates.push_back({"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back({"searched", {{"?s", "sector"}}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  move.preconditions.push_back({"at", {"?r", "?from"}});
  move.addEffects.push_back({"at", {"?r", "?to"}});
  move.delEffects.push_back({"at", {"?r", "?from"}});
  model.actions.push_back(move);

  ActionDef search;
  search.name = "search";
  search.params = {{"?r", "robot"}, {"?s", "sector"}};
  search.preconditions.push_back({"at", {"?r", "?s"}});
  search.addEffects.push_back({"searched", {"?s"}});
  model.actions.push_back(search);

  model.objects.push_back({"uav1", "robot"});
  model.objects.push_back({"base", "location"});
  model.objects.push_back({"sector_a", "sector"});

  ScenarioDef scenario;
  scenario.name = "nominal";
  scenario.initialState.push_back({"at", {"uav1", "base"}});
  scenario.goals.push_back({"searched", {"sector_a"}});
  model.scenarios.push_back(std::move(scenario));

  return model;
}

ProjectModel makeRouteModel(bool includeBackup) {
  ProjectModel model;
  model.projectName = "route-recovery";
  model.types.push_back({"robot", "object"});
  model.predicates.push_back({"ready", {{"?r", "robot"}}, 0.0F, 0.0F});
  model.predicates.push_back(
      {"primary-available", {{"?r", "robot"}}, 0.0F, 0.0F});
  model.predicates.push_back(
      {"backup-available", {{"?r", "robot"}}, 0.0F, 0.0F});
  model.predicates.push_back({"done", {{"?r", "robot"}}, 0.0F, 0.0F});

  ActionDef primary;
  primary.name = "use-primary";
  primary.params = {{"?r", "robot"}};
  primary.preconditions = {{"ready", {"?r"}},
                           {"primary-available", {"?r"}}};
  primary.addEffects = {{"done", {"?r"}}};
  primary.btBinding.reactive = true;
  model.actions.push_back(primary);

  if (includeBackup) {
    ActionDef backup;
    backup.name = "use-backup";
    backup.params = {{"?r", "robot"}};
    backup.preconditions = {{"ready", {"?r"}},
                            {"backup-available", {"?r"}}};
    backup.addEffects = {{"done", {"?r"}}};
    backup.btBinding.reactive = true;
    model.actions.push_back(backup);
  }

  model.objects.push_back({"uav1", "robot"});
  ScenarioDef scenario;
  scenario.name = "route";
  scenario.initialState = {{"ready", {"uav1"}},
                           {"primary-available", {"uav1"}},
                           {"backup-available", {"uav1"}}};
  scenario.goals = {{"done", {"uav1"}}};
  model.scenarios.push_back(std::move(scenario));
  return model;
}

ActionDef* findAction(ProjectModel& model, const std::string& name) {
  const auto it = std::find_if(model.actions.begin(), model.actions.end(),
                               [&name](const ActionDef& action) {
                                 return action.name == name;
                               });
  return it == model.actions.end() ? nullptr : &(*it);
}

const RunActionStep* findStep(const SimulationEngine& engine,
                              const std::string& actionName) {
  const auto& steps = engine.actionSteps();
  const auto it = std::find_if(steps.begin(), steps.end(),
                               [&actionName](const RunActionStep& step) {
                                 return step.actionName == actionName;
                               });
  return it == steps.end() ? nullptr : &(*it);
}

bool recordedChange(const SimulationEngine& engine,
                    const std::string& fact,
                    bool value) {
  const auto& changes = engine.factChanges();
  return std::any_of(changes.begin(), changes.end(),
                     [&fact, value](const RunFactChange& change) {
                       return change.fact == fact && change.value == value;
                     });
}

const RunFactState* findFact(const RunState& state, const std::string& name) {
  const auto it = std::find_if(
      state.facts.begin(), state.facts.end(),
      [&name](const RunFactState& fact) { return fact.fact == name; });
  return it == state.facts.end() ? nullptr : &(*it);
}

const RunActionStep* findStateStep(const RunState& state,
                                  const std::string& actionName) {
  const auto it = std::find_if(
      state.actionSteps.begin(), state.actionSteps.end(),
      [&actionName](const RunActionStep& step) {
        return step.actionName == actionName;
      });
  return it == state.actionSteps.end() ? nullptr : &(*it);
}

} // namespace

TEST(SimulationEngine, RunsToTheGoalWithNothingConfigured) {
  const ProjectModel model = makeSearchModel();
  SimulationEngine engine;

  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  EXPECT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  EXPECT_EQ(engine.phase(), RunPhase::Completed);
  EXPECT_EQ(engine.actionSteps().size(), 2U);
  EXPECT_EQ(engine.actionsFinishedCount(), 2U);
  EXPECT_EQ(engine.goalsMetCount(), 1U);
  EXPECT_TRUE(engine.errorMessage().empty());
}

TEST(SimulationEngine, WithNothingConfiguredTheStepsStillHappenInOrder) {
  const ProjectModel model = makeSearchModel();
  SimulationEngine engine;

  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  const RunActionStep* move = findStep(engine, "move");
  const RunActionStep* search = findStep(engine, "search");
  ASSERT_NE(move, nullptr);
  ASSERT_NE(search, nullptr);

  // The default duration exists so that a run can be watched: the two steps
  // must not begin and end together on the first tick.
  EXPECT_GT(move->endTick, 1U);
  EXPECT_GT(search->endTick, move->endTick);
  EXPECT_GE(search->startTick, move->endTick);
  EXPECT_GT(engine.tick(), 1U);
}

TEST(SimulationEngine, TreeHasOneNodePerMissionStep) {
  const ProjectModel model = makeSearchModel();
  SimulationEngine engine;

  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();

  size_t actionNodes = 0;
  for (const auto& node : engine.nodes()) {
    if (node.isAction) {
      ++actionNodes;
    }
    // The fact-level helper nodes the compiler used to wrap actions in are
    // gone; a run that saw one would be drawing a tree no reviewer can read.
    EXPECT_NE(node.nodeType, "CheckWorldPredicate");
    EXPECT_NE(node.nodeType, "SetWorldPredicate");
  }
  EXPECT_EQ(actionNodes, 2U);
  EXPECT_EQ(actionNodes, engine.actionSteps().size());
}

TEST(SimulationEngine, ActionsTakeTheTicksTheyAreGiven) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 3;
  findAction(model, "search")->simulation.ticks = 2;

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  const RunActionStep* move = findStep(engine, "move");
  const RunActionStep* search = findStep(engine, "search");
  ASSERT_NE(move, nullptr);
  ASSERT_NE(search, nullptr);

  EXPECT_EQ(move->startTick, 1U);
  EXPECT_EQ(move->endTick, 3U);
  EXPECT_EQ(search->endTick, 4U);
  EXPECT_EQ(engine.tick(), 4U);
}

TEST(SimulationEngine, OneStepAtATimeAdvancesOneTick) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 4;

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();

  EXPECT_EQ(engine.tick(), 0U);
  EXPECT_TRUE(engine.stepOnce());
  EXPECT_EQ(engine.tick(), 1U);
  EXPECT_EQ(engine.phase(), RunPhase::Running);

  const RunActionStep* move = findStep(engine, "move");
  ASSERT_NE(move, nullptr);
  EXPECT_EQ(move->status, RunNodeStatus::Happening);
  EXPECT_EQ(engine.happeningNow().size(), 1U);
}

TEST(SimulationEngine, HoldStopsTheRunAdvancing) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 10;

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  engine.setTicksPerSecond(4.0);

  engine.advance(0.5);
  const unsigned afterFirstAdvance = engine.tick();
  EXPECT_EQ(afterFirstAdvance, 2U);

  engine.hold();
  engine.advance(1.0);
  EXPECT_EQ(engine.tick(), afterFirstAdvance);

  engine.resume();
  engine.advance(0.25);
  EXPECT_EQ(engine.tick(), afterFirstAdvance + 1U);
}

TEST(SimulationEngine, AnActionSetToFailStopsTheMission) {
  ProjectModel model = makeSearchModel();
  findAction(model, "search")->simulation.succeeds = false;

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  EXPECT_FALSE(engine.runToCompletion());

  EXPECT_EQ(engine.phase(), RunPhase::Failed);
  const RunActionStep* search = findStep(engine, "search");
  ASSERT_NE(search, nullptr);
  EXPECT_EQ(search->status, RunNodeStatus::WentWrong);
  EXPECT_EQ(engine.goalsMetCount(), 0U);
}

TEST(SimulationEngine, FactChangesAreRecordedWithTheirTick) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 2;

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  EXPECT_TRUE(recordedChange(engine, "(at uav1 sector_a)", true));
  EXPECT_TRUE(recordedChange(engine, "(at uav1 base)", false));
  EXPECT_TRUE(recordedChange(engine, "(searched sector_a)", true));

  unsigned previousTick = 0;
  for (const auto& change : engine.factChanges()) {
    EXPECT_GE(change.tick, previousTick);
    previousTick = change.tick;
  }
  EXPECT_GE(previousTick, 1U);
}

TEST(SimulationEngine, StateAtTickZeroHasTheScenariosStartingFacts) {
  const ProjectModel model = makeSearchModel();
  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();

  const RunState state = engine.stateAtTick(0);
  EXPECT_EQ(state.tick, 0U);
  const RunFactState* atBase = findFact(state, "(at uav1 base)");
  const RunFactState* atSector = findFact(state, "(at uav1 sector_a)");
  const RunFactState* searched = findFact(state, "(searched sector_a)");
  ASSERT_NE(atBase, nullptr);
  ASSERT_NE(atSector, nullptr);
  ASSERT_NE(searched, nullptr);
  EXPECT_TRUE(atBase->value);
  EXPECT_FALSE(atBase->changedDuringRun);
  EXPECT_FALSE(atSector->value);
  EXPECT_FALSE(searched->value);
}

TEST(SimulationEngine, StateAtFinalTickMatchesTheLiveEndState) {
  const ProjectModel model = makeSearchModel();
  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  const RunState state = engine.stateAtTick(engine.tick());
  ASSERT_NE(findFact(state, "(at uav1 base)"), nullptr);
  ASSERT_NE(findFact(state, "(at uav1 sector_a)"), nullptr);
  ASSERT_NE(findFact(state, "(searched sector_a)"), nullptr);
  EXPECT_FALSE(findFact(state, "(at uav1 base)")->value);
  EXPECT_TRUE(findFact(state, "(at uav1 sector_a)")->value);
  EXPECT_TRUE(findFact(state, "(searched sector_a)")->value);
  EXPECT_EQ(findFact(state, "(searched sector_a)")->lastChangedTick,
            engine.tick());
}

TEST(SimulationEngine, StateAtTickReconstructsActionProgress) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 3;
  findAction(model, "search")->simulation.ticks = 3;
  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  const RunActionStep* liveMove = findStep(engine, "move");
  ASSERT_NE(liveMove, nullptr);
  ASSERT_LT(liveMove->startTick, liveMove->endTick);

  const RunState during = engine.stateAtTick(liveMove->startTick + 1U);
  const RunActionStep* duringMove = findStateStep(during, "move");
  ASSERT_NE(duringMove, nullptr);
  EXPECT_EQ(duringMove->status, RunNodeStatus::Happening);

  const RunState after = engine.stateAtTick(liveMove->endTick + 1U);
  const RunActionStep* afterMove = findStateStep(after, "move");
  ASSERT_NE(afterMove, nullptr);
  EXPECT_EQ(afterMove->status, RunNodeStatus::Finished);
}

TEST(SimulationEngine, StateAtTickClampsRequestsBeyondTheEnd) {
  const ProjectModel model = makeSearchModel();
  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  const RunState atEnd = engine.stateAtTick(engine.tick());
  const RunState beyondEnd = engine.stateAtTick(engine.tick() + 1000U);
  EXPECT_EQ(beyondEnd.tick, engine.tick());
  ASSERT_EQ(beyondEnd.facts.size(), atEnd.facts.size());
  for (size_t i = 0; i < atEnd.facts.size(); ++i) {
    EXPECT_EQ(beyondEnd.facts[i].fact, atEnd.facts[i].fact);
    EXPECT_EQ(beyondEnd.facts[i].value, atEnd.facts[i].value);
    EXPECT_EQ(beyondEnd.facts[i].lastChangedTick,
              atEnd.facts[i].lastChangedTick);
  }
}

TEST(SimulationEngine, StartAgainRepeatsTheSameRun) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 3;

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();
  const std::string firstRun = engine.toJson();

  ASSERT_TRUE(engine.startAgain()) << engine.errorMessage();
  EXPECT_EQ(engine.tick(), 0U);
  EXPECT_TRUE(engine.factChanges().empty());
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  EXPECT_EQ(engine.toJson(), firstRun);
}

TEST(SimulationEngine, ARandomFailureRepeatsUnderTheSameSeed) {
  ProjectModel model = makeSearchModel();
  findAction(model, "search")->simulation.failureChance = 0.5;

  std::string firstOutcome;
  for (int attempt = 0; attempt < 3; ++attempt) {
    SimulationEngine engine;
    ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
    engine.runToCompletion();
    const std::string outcome = runPhaseName(engine.phase());
    if (attempt == 0) {
      firstOutcome = outcome;
    }
    EXPECT_EQ(outcome, firstOutcome);
  }
}

TEST(SimulationEngine, AnActionNeedingObservedStateWillNotRunOnAPrediction) {
  ProjectModel model = makeSearchModel();
  // The domain now says that being somewhere only counts once it has been
  // observed. The scenario observes where the vehicle starts, but nothing
  // observes where the move predicts it ends up, so the search cannot start.
  for (auto& predicate : model.predicates) {
    if (predicate.name == "at") {
      predicate.confirmed = true;
    }
  }

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  EXPECT_FALSE(engine.runToCompletion());

  EXPECT_EQ(engine.phase(), RunPhase::Failed);
  const RunActionStep* move = findStep(engine, "move");
  const RunActionStep* search = findStep(engine, "search");
  ASSERT_NE(move, nullptr);
  ASSERT_NE(search, nullptr);
  EXPECT_EQ(move->status, RunNodeStatus::Finished);
  EXPECT_EQ(search->status, RunNodeStatus::WentWrong);
}

TEST(SimulationEngine, AScenarioThatCannotBePlannedSaysSoInPlainWords) {
  ProjectModel model = makeSearchModel();
  model.scenarios[0].initialState.clear();

  SimulationEngine engine;
  EXPECT_FALSE(engine.start(model, "nominal"));
  EXPECT_EQ(engine.phase(), RunPhase::Error);
  EXPECT_NE(engine.errorMessage().find("no plan"), std::string::npos);
}

TEST(SimulationEngine, AnUnknownScenarioIsNamedInTheError) {
  const ProjectModel model = makeSearchModel();

  SimulationEngine engine;
  EXPECT_FALSE(engine.start(model, "night-run"));
  EXPECT_EQ(engine.phase(), RunPhase::Error);
  EXPECT_NE(engine.errorMessage().find("night-run"), std::string::npos);
}

TEST(SimulationEngine, TheTickLimitEndsARunThatWillNotFinish) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 50;

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  engine.setTickLimit(5);
  EXPECT_FALSE(engine.runToCompletion());

  EXPECT_EQ(engine.phase(), RunPhase::Error);
  EXPECT_EQ(engine.tick(), 5U);
  EXPECT_NE(engine.errorMessage().find("limit"), std::string::npos);
}

TEST(SimulationEngine, TheJsonReportNamesEveryActionAndGoal) {
  const ProjectModel model = makeSearchModel();

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  const nlohmann::json report = nlohmann::json::parse(engine.toJson());
  EXPECT_EQ(report["scenario"], "nominal");
  EXPECT_EQ(report["phase"], "completed");
  EXPECT_EQ(report["actions"].size(), 2U);
  EXPECT_EQ(report["goals"].size(), 1U);
  EXPECT_EQ(report["goals"][0]["met"], true);
  EXPECT_FALSE(report["factChanges"].empty());
}

TEST(SimulationEngine, SimulationSettingsSurviveSaveAndLoad) {
  ProjectModel model = makeSearchModel();
  findAction(model, "move")->simulation.ticks = 6;
  findAction(model, "search")->simulation.succeeds = false;
  findAction(model, "search")->simulation.failureChance = 0.25;
  model.simulationSeed = 99U;

  const nlohmann::json json = model;
  const ProjectModel reloaded = json.get<ProjectModel>();

  ProjectModel copy = reloaded;
  EXPECT_EQ(findAction(copy, "move")->simulation.ticks, 6);
  EXPECT_FALSE(findAction(copy, "search")->simulation.succeeds);
  EXPECT_DOUBLE_EQ(findAction(copy, "search")->simulation.failureChance, 0.25);
  EXPECT_EQ(reloaded.simulationSeed, 99U);
}

TEST(SimulationEngine, AProjectSavedBeforeRunsExistedTakesTheDefaults) {
  ProjectModel model = makeSearchModel();
  nlohmann::json json = model;
  for (auto& action : json["actions"]) {
    action.erase("simulation");
  }
  json.erase("simulationSeed");

  const ProjectModel reloaded = json.get<ProjectModel>();
  ProjectModel copy = reloaded;
  EXPECT_EQ(findAction(copy, "move")->simulation.ticks,
            SimulationSettings{}.ticks);
  EXPECT_TRUE(findAction(copy, "move")->simulation.succeeds);
  EXPECT_EQ(reloaded.simulationSeed, ProjectModel{}.simulationSeed);

  SimulationEngine engine;
  ASSERT_TRUE(engine.start(reloaded, "nominal")) << engine.errorMessage();
  EXPECT_TRUE(engine.runToCompletion()) << engine.errorMessage();
}

TEST(SimulationEngine, ForcedActionFailureAppliesOnlyToChosenAttempt) {
  ProjectModel model = makeSearchModel();
  RunFaultSet faults;
  faults.name = "search-fails-once";
  faults.actionFailures.push_back({"search", 1U});

  SimulationEngine engine;
  engine.setFaults(faults);
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  EXPECT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  EXPECT_EQ(engine.replanCount(), 1U);
  EXPECT_EQ(engine.phase(), RunPhase::Completed);
  const std::vector<std::string> actions_run = engine.actionsRun();
  EXPECT_EQ(std::count(actions_run.begin(), actions_run.end(), "search"), 2);
  ASSERT_FALSE(engine.replans().empty());
  EXPECT_TRUE(engine.replans().front().replacementFound);
}

TEST(SimulationEngine, SetsAChosenFactAtTheChosenTick) {
  ProjectModel model = makeSearchModel();
  RunFaultSet faults;
  faults.name = "left-base-early";
  faults.factChanges.push_back({{"at", {"uav1", "base"}}, false, 1U});

  SimulationEngine engine;
  engine.setFaults(faults);
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  engine.stepOnce();

  const auto found = std::find_if(
      engine.factChanges().begin(), engine.factChanges().end(),
      [](const RunFactChange& change) {
        return change.tick == 1U && change.fact == "(at uav1 base)" &&
               !change.value && change.source.find("left-base-early") !=
                                    std::string::npos;
      });
  EXPECT_NE(found, engine.factChanges().end());
}

TEST(SimulationEngine, SettingLifecycleStateClearsItsAlternatives) {
  ProjectModel model = makeSearchModel();
  model.predicates.push_back(
      {"available", {{"?r", "robot"}}, 0.0F, 0.0F});
  model.predicates.push_back(
      {"unavailable", {{"?r", "robot"}}, 0.0F, 0.0F});
  model.stateGroups.push_back(
      {"availability", "robot", {"available", "unavailable"}});
  model.scenarios[0].initialState.push_back({"available", {"uav1"}});
  RunFaultSet faults;
  faults.factChanges.push_back(
      {{"unavailable", {"uav1"}}, true, 1U});

  SimulationEngine engine;
  engine.setFaults(faults);
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  engine.stepOnce();

  const RunState state = engine.stateAtTick(1U);
  ASSERT_NE(findFact(state, "(available uav1)"), nullptr);
  ASSERT_NE(findFact(state, "(unavailable uav1)"), nullptr);
  EXPECT_FALSE(findFact(state, "(available uav1)")->value);
  EXPECT_TRUE(findFact(state, "(unavailable uav1)")->value);
}

TEST(SimulationEngine, FactLossReplansAndRecoversByADifferentRoute) {
  ProjectModel model = makeRouteModel(true);
  RunFaultSet faults;
  faults.name = "primary-lost";
  faults.factChanges.push_back(
      {{"primary-available", {"uav1"}}, false, 2U});

  SimulationEngine engine;
  engine.setFaults(faults);
  ASSERT_TRUE(engine.start(model, "route")) << engine.errorMessage();
  ASSERT_FALSE(engine.faultExplanations().empty());
  EXPECT_NE(engine.faultExplanations().front().summary.find("part-way"),
            std::string::npos);
  EXPECT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  EXPECT_EQ(engine.replanCount(), 1U);
  const std::vector<std::string> actions_run = engine.actionsRun();
  EXPECT_TRUE(std::find(actions_run.begin(), actions_run.end(),
                        "use-primary") != actions_run.end());
  EXPECT_TRUE(std::find(actions_run.begin(), actions_run.end(),
                        "use-backup") != actions_run.end());
  ASSERT_EQ(engine.replans().size(), 1U);
  EXPECT_NE(engine.replans()[0].reason.find("replanned"), std::string::npos);
  EXPECT_TRUE(std::any_of(engine.replans()[0].replacementPlan.begin(),
                          engine.replans()[0].replacementPlan.end(),
                          [](const std::string& action) {
                            return action.find("use-backup") != std::string::npos;
                          }));
}

TEST(SimulationEngine, NoPlanAfterFailureUsesFailureExplainer) {
  ProjectModel model = makeRouteModel(false);
  RunFaultSet faults;
  faults.name = "only-route-lost";
  faults.factChanges.push_back(
      {{"primary-available", {"uav1"}}, false, 2U});

  SimulationEngine engine;
  engine.setFaults(faults);
  ASSERT_TRUE(engine.start(model, "route")) << engine.errorMessage();
  EXPECT_FALSE(engine.runToCompletion());

  EXPECT_EQ(engine.phase(), RunPhase::Failed);
  ASSERT_EQ(engine.replans().size(), 1U);
  EXPECT_FALSE(engine.replans()[0].replacementFound);
  EXPECT_TRUE(engine.replans()[0].failureExplanation.available);
  EXPECT_EQ(engine.replans()[0].failureExplanation.blockingFact.predicateName,
            "primary-available");
}
