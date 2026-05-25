#include <gtest/gtest.h>

#include "project_model.h"
#include "scenario_runner.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <utility>

namespace {

ProjectModel makeUavSearchModel() {
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

  return model;
}

void addSearchScenario(ProjectModel& model, ScenarioExpectation expectation) {
  ScenarioDef scenario;
  scenario.name = "nominal";
  scenario.initialState.push_back({"at", {"uav1", "base"}});
  scenario.goals.push_back({"searched", {"sector_a"}});
  scenario.expectation = std::move(expectation);
  model.scenarios.push_back(std::move(scenario));
}

} // namespace

TEST(ScenarioRunner, EmptyModelGivesEmptyReport) {
  const ScenarioBatchReport report = ScenarioRunner::runAll(ProjectModel{});

  EXPECT_TRUE(report.results.empty());
  EXPECT_EQ(report.passCount, 0U);
  EXPECT_EQ(report.failCount, 0U);
  EXPECT_EQ(report.errorCount, 0U);
}

TEST(ScenarioRunner, PassWhenExpectationMet) {
  ProjectModel model = makeUavSearchModel();
  addSearchScenario(model, ScenarioExpectation{});

  const ScenarioBatchReport report = ScenarioRunner::runAll(model);

  ASSERT_EQ(report.results.size(), 1U);
  EXPECT_EQ(report.passCount, 1U);
  EXPECT_EQ(report.results[0].outcome, ScenarioOutcome::Pass);
  EXPECT_TRUE(report.results[0].planSucceeded);
  EXPECT_TRUE(report.results[0].reason.empty());
}

TEST(ScenarioRunner, FailWhenPlanShorterThanMin) {
  ProjectModel model = makeUavSearchModel();
  ScenarioExpectation expectation;
  expectation.minPlanSteps = 99;
  addSearchScenario(model, expectation);

  const ScenarioBatchReport report = ScenarioRunner::runAll(model);

  ASSERT_EQ(report.results.size(), 1U);
  EXPECT_EQ(report.failCount, 1U);
  EXPECT_EQ(report.results[0].outcome, ScenarioOutcome::Fail);
  EXPECT_NE(report.results[0].reason.find("plan too short"), std::string::npos);
}

TEST(ScenarioRunner, FailWhenExpectedActionMissing) {
  ProjectModel model = makeUavSearchModel();
  ScenarioExpectation expectation;
  expectation.expectedActions = {"nonexistent"};
  addSearchScenario(model, expectation);

  const ScenarioBatchReport report = ScenarioRunner::runAll(model);

  ASSERT_EQ(report.results.size(), 1U);
  EXPECT_EQ(report.failCount, 1U);
  EXPECT_EQ(report.results[0].outcome, ScenarioOutcome::Fail);
  EXPECT_NE(report.results[0].reason.find("expected action 'nonexistent' not used"),
            std::string::npos);
}

TEST(ScenarioRunner, FailWhenForbiddenActionUsed) {
  ProjectModel model = makeUavSearchModel();
  ScenarioExpectation expectation;
  expectation.forbiddenActions = {"move"};
  addSearchScenario(model, expectation);

  const ScenarioBatchReport report = ScenarioRunner::runAll(model);

  ASSERT_EQ(report.results.size(), 1U);
  EXPECT_EQ(report.failCount, 1U);
  EXPECT_EQ(report.results[0].outcome, ScenarioOutcome::Fail);
  EXPECT_TRUE(std::find(report.results[0].usedActionSchemas.begin(),
                        report.results[0].usedActionSchemas.end(),
                        "move") != report.results[0].usedActionSchemas.end());
  EXPECT_NE(report.results[0].reason.find("forbidden action 'move' used"),
            std::string::npos);
}

TEST(ScenarioRunner, JsonRoundTripStructure) {
  ScenarioBatchReport report;
  ScenarioRunResult pass;
  pass.scenarioName = "nominal";
  pass.outcome = ScenarioOutcome::Pass;
  pass.planStepCount = 2;
  pass.solveTimeMs = 1.25;
  pass.planSucceeded = true;
  pass.usedActionSchemas = {"move", "search"};
  report.results.push_back(pass);

  ScenarioRunResult fail;
  fail.scenarioName = "blocked";
  fail.outcome = ScenarioOutcome::Fail;
  fail.reason = "expected success but no plan found";
  fail.planStepCount = 0;
  fail.solveTimeMs = 0.5;
  fail.planSucceeded = false;
  report.results.push_back(fail);
  report.passCount = 1;
  report.failCount = 1;

  const nlohmann::json json = nlohmann::json::parse(ScenarioRunner::toJson(report));

  EXPECT_EQ(json.at("passCount").get<size_t>(), 1U);
  EXPECT_EQ(json.at("failCount").get<size_t>(), 1U);
  EXPECT_EQ(json.at("errorCount").get<size_t>(), 0U);
  ASSERT_EQ(json.at("results").size(), 2U);
  EXPECT_EQ(json.at("results").at(0).at("scenarioName").get<std::string>(),
            "nominal");
  EXPECT_EQ(json.at("results").at(0).at("outcome").get<std::string>(), "Pass");
  EXPECT_EQ(json.at("results").at(0).at("planStepCount").get<size_t>(), 2U);
  EXPECT_TRUE(json.at("results").at(0).at("planSucceeded").get<bool>());
  EXPECT_EQ(json.at("results").at(0).at("usedActionSchemas").size(), 2U);
  EXPECT_EQ(json.at("results").at(1).at("outcome").get<std::string>(), "Fail");
  EXPECT_EQ(json.at("results").at(1).at("reason").get<std::string>(),
            "expected success but no plan found");
}
