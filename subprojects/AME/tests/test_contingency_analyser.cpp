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

TEST(ContingencyAnalyser, RefusesADomainTooLargeToEnumerateAndSaysWhatToDo) {
  const ProjectModel model = makeContingencyModel(10U);

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline", 4U);

  EXPECT_FALSE(report.ok);
  // The refusal names the numbers and says what would narrow it, because
  // "too many" on its own leaves the reader with nothing to do.
  EXPECT_NE(report.error.find("10"), std::string::npos) << report.error;
  EXPECT_NE(report.error.find("at most 4"), std::string::npos) << report.error;
  EXPECT_NE(report.error.find("represent a contingency"), std::string::npos)
      << report.error;
}

TEST(ContingencyAnalyser, TheResultSaysHowMuchOfTheSpaceWasCovered) {
  const ProjectModel model = makeContingencyModel(3U);

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline");

  ASSERT_TRUE(report.ok) << report.error;
  EXPECT_EQ(report.combinationsChecked, 8U);
  EXPECT_EQ(report.plannerCalls + report.answeredByReasoning,
            report.combinationsChecked);
  EXPECT_FALSE(report.declaredByUser) << "this scenario declares nothing";
  EXPECT_NE(report.coverageSentence().find("all 8"), std::string::npos)
      << report.coverageSentence();
}

TEST(ContingencyAnalyser, ADeclaredContingencyNarrowsWhatIsVaried) {
  // Two things the plan cannot change: whether comms are up at each location,
  // and whether the weather allows flying at all. Only one of them is the
  // contingency this scenario is about.
  ProjectModel model = makeContingencyModel(3U);
  model.predicates.push_back({"weather_ok", {}, 0.0F, 0.0F});
  model.actions[0].preconditions.push_back({"weather_ok", {}});

  const ContingencyReport everything =
      ContingencyAnalyser::analyse(model, "baseline");
  ASSERT_TRUE(everything.ok) << everything.error;
  EXPECT_EQ(everything.contextPredicates.size(), 2U);
  EXPECT_EQ(everything.combinationsChecked, 16U)
      << "three places with comms, and the weather";

  model.scenarios[0].contingency.contingencyPredicates = {"weather_ok"};
  const ContingencyReport declared =
      ContingencyAnalyser::analyse(model, "baseline");

  ASSERT_TRUE(declared.ok) << declared.error;
  EXPECT_TRUE(declared.declaredByUser);
  EXPECT_EQ(declared.contextPredicates.size(), 1U);
  EXPECT_EQ(declared.combinationsChecked, 2U) << "only the weather is varied";
  EXPECT_NE(declared.coverageSentence().find("this scenario declares"),
            std::string::npos)
      << declared.coverageSentence();
}

TEST(ContingencyAnalyser, ADeclarationThatMatchesNothingIsRefusedNotReportedAsChecked) {
  ProjectModel model = makeContingencyModel(3U);
  // "visited" is made true by move, so it is not a fact the plan cannot
  // change, and declaring it as the contingency varies nothing at all.
  model.scenarios[0].contingency.contingencyPredicates = {"visited"};

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline");

  // Reporting "checked all 1 ways" here would put a claim of complete coverage
  // into the assurance report for something that was never varied.
  EXPECT_FALSE(report.ok);
  EXPECT_NE(report.error.find("visited"), std::string::npos) << report.error;
  EXPECT_NE(report.error.find("Nothing was checked"), std::string::npos)
      << report.error;
}

TEST(ContingencyAnalyser, ADeclaredSafeStateIsWhatMustBeReachable) {
  ProjectModel model = makeContingencyModel(3U);
  // Getting back to where it started counts as safe, whatever else happened.
  model.scenarios[0].contingency.safeState.push_back({"at", {"uav1", "loc0"}});

  const ContingencyReport report =
      ContingencyAnalyser::analyse(model, "baseline");

  ASSERT_TRUE(report.ok) << report.error;
  EXPECT_TRUE(report.declaredByUser);
  // Standing still already satisfies it, so every context reaches it, which
  // the goal the scenario was written with does not.
  EXPECT_EQ(report.feasibleCount, report.results.size());
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
