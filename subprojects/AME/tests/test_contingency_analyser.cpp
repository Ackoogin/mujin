#include <gtest/gtest.h>

#include "contingency_analyser.h"
#include "project_model.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace {

ProjectModel makeContingencyModel(size_t locationCount) {
  ProjectModel model;
  model.projectName = "contingency-test";
  model.types.push_back({"robot", "object"});
  model.types.push_back({"location", "object"});

  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back(
      {"visited", {{"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back(
      {"comms_ok", {{"?l", "location"}}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  move.preconditions.push_back({"at", {"?r", "?from"}});
  move.preconditions.push_back({"comms_ok", {"?to"}});
  move.addEffects.push_back({"at", {"?r", "?to"}});
  move.addEffects.push_back({"visited", {"?to"}});
  move.delEffects.push_back({"at", {"?r", "?from"}});
  model.actions.push_back(std::move(move));

  model.objects.push_back({"uav1", "robot"});
  for (size_t i = 0; i < locationCount; ++i) {
    model.objects.push_back({"loc" + std::to_string(i), "location"});
  }

  ScenarioDef scenario;
  scenario.name = "baseline";
  scenario.initialState.push_back({"at", {"uav1", "loc0"}});
  scenario.goals.push_back({"visited", {"loc" + std::to_string(locationCount - 1U)}});
  model.scenarios.push_back(std::move(scenario));

  return model;
}

bool contains(const std::vector<std::string>& values, const std::string& value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

} // namespace

TEST(ContingencyAnalyser, IdentifiesContextPredicate) {
  const ProjectModel model = makeContingencyModel(1U);

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline");

  ASSERT_TRUE(report.ok) << report.error;
  EXPECT_TRUE(contains(report.contextPredicates, "comms_ok"));
  EXPECT_FALSE(contains(report.contextPredicates, "at"));
  EXPECT_FALSE(contains(report.contextPredicates, "visited"));
}

TEST(ContingencyAnalyser, EnumeratesAllSubsets) {
  const ProjectModel model = makeContingencyModel(3U);

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline");

  ASSERT_TRUE(report.ok) << report.error;
  ASSERT_EQ(report.contextFluents.size(), 3U);
  EXPECT_EQ(report.results.size(), 8U);
}

TEST(ContingencyAnalyser, CapsAtMaxFluents) {
  const ProjectModel model = makeContingencyModel(10U);

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline", 4U);

  EXPECT_FALSE(report.ok);
  EXPECT_NE(report.error.find("too many context fluents (10), max is 4"),
            std::string::npos);
}

TEST(ContingencyAnalyser, FeasibleContextCounted) {
  const ProjectModel model = makeContingencyModel(3U);

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline");

  ASSERT_TRUE(report.ok) << report.error;
  ASSERT_EQ(report.results.size(), 8U);
  const ContingencyContext& allTrue = report.results.back();
  EXPECT_TRUE(allTrue.errorMessage.empty());
  EXPECT_TRUE(allTrue.planFound);
  EXPECT_GT(report.feasibleCount, 0U);
  EXPECT_EQ(report.feasibleCount + report.infeasibleCount + report.errorCount,
            report.results.size());
}
