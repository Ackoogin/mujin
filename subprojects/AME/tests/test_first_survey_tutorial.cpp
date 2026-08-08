// =========================================================================
// The worked example the authoring tool user guide teaches from.
//
// `doc/guides/authoring_tool_user_guide.md` walks a reader who has never seen
// PDDL through building `domains/first_survey/first-survey.ameproj.json`, and
// prints the plan, the PDDL and the scenario results the tool produces from
// it. A change that alters any of those makes the guide wrong, which is what
// these tests are here to catch: they check the project the guide ships, the
// exported PDDL files beside it, and the three scenarios the guide describes.
//
// The pipeline is the one the interface uses, without the interface:
//   ProjectModel::load           — open the saved project
//   StructuralValidator::check   — the checks Validate Now runs first
//   PddlValidator::validate...   — generate PDDL, parse it, ground it
//   ame::Planner::solve          — Check Feasibility
//   ScenarioRunner::runAll       — Run All Scenarios
// =========================================================================

#include <gtest/gtest.h>

#include "pddl_generator.h"
#include "pddl_validator.h"
#include "project_model.h"
#include "scenario_runner.h"
#include "structural_validator.h"

#include <ame/planner.h>
#include <ame/world_model.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

#ifndef AME_DOMAINS_DIR
#error "AME_DOMAINS_DIR must be defined by CMake"
#endif

namespace {

const char* const kProjectFile = "first-survey.ameproj.json";
const char* const kNominalScenario = "survey-sector-a";

std::string tutorialPath(const std::string& leaf) {
  return std::string(AME_DOMAINS_DIR) + "/first_survey/" + leaf;
}

std::string readFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return {};
  }
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

ProjectModel loadTutorialProject() {
  ProjectModel model;
  EXPECT_TRUE(model.load(tutorialPath(kProjectFile)))
      << "could not open " << tutorialPath(kProjectFile);
  return model;
}

const ScenarioRunResult* findResult(const ScenarioBatchReport& report,
                                    const std::string& name) {
  const auto found = std::find_if(report.results.begin(), report.results.end(),
                                  [&name](const ScenarioRunResult& result) {
                                    return result.scenarioName == name;
                                  });
  return found == report.results.end() ? nullptr : &*found;
}

}  // namespace

TEST(FirstSurveyTutorial, ProjectOpensWithTheShapeTheGuideDescribes) {
  const ProjectModel model = loadTutorialProject();

  EXPECT_EQ(model.projectName, "first-survey");
  // Three types, six facts, five actions, four things, three scenarios. The
  // guide names every one of them, so a change here is a change to the guide.
  EXPECT_EQ(model.types.size(), 3U);
  EXPECT_EQ(model.predicates.size(), 6U);
  EXPECT_EQ(model.actions.size(), 5U);
  EXPECT_EQ(model.objects.size(), 4U);
  EXPECT_EQ(model.scenarios.size(), 3U);
  ASSERT_EQ(model.stateGroups.size(), 1U);
  EXPECT_EQ(model.stateGroups.front().name, "flight state");
}

TEST(FirstSurveyTutorial, NothingIsStructurallyWrongWithIt) {
  const ProjectModel model = loadTutorialProject();

  const StructuralReport structural = StructuralValidator::check(model);
  EXPECT_EQ(structural.errorCount, 0U)
      << "first structural error: "
      << (structural.issues.empty() ? "n/a" : structural.issues.front().message);

  const ValidationReport report = PddlValidator::validate(model, kNominalScenario);
  EXPECT_TRUE(report.ok)
      << "generated PDDL did not validate: "
      << (report.errors.empty() ? "no message" : report.errors.front().message);
}

TEST(FirstSurveyTutorial, ExportedPddlMatchesWhatTheToolGenerates) {
  // The .pddl files beside the project are what the guide prints, and are what
  // a reader compares their own export against. They are generated files, so
  // they have to stay identical to what the generator produces today.
  const ProjectModel model = loadTutorialProject();

  const std::string domainOnDisk = readFile(tutorialPath("domain.pddl"));
  ASSERT_FALSE(domainOnDisk.empty()) << "could not read domain.pddl";
  EXPECT_EQ(domainOnDisk, PddlGenerator::generateDomain(model));

  for (const ScenarioDef& scenario : model.scenarios) {
    const std::string leaf = "problem_" + scenario.name + ".pddl";
    const std::string problemOnDisk = readFile(tutorialPath(leaf));
    ASSERT_FALSE(problemOnDisk.empty()) << "could not read " << leaf;
    EXPECT_EQ(problemOnDisk, PddlGenerator::generateProblem(model, scenario.name))
        << leaf << " is not what the generator writes today";
  }
}

TEST(FirstSurveyTutorial, TheNominalMissionPlansInSixSteps) {
  const ProjectModel model = loadTutorialProject();

  ame::WorldModel worldModel;
  const ValidationReport report =
      PddlValidator::validateAndBuildWorldModel(model, kNominalScenario, worldModel);
  ASSERT_TRUE(report.ok)
      << "validation failed: "
      << (report.errors.empty() ? "no message" : report.errors.front().message);

  const ame::Planner planner;
  const ame::PlanResult plan = planner.solve(worldModel);
  ASSERT_TRUE(plan.success) << "no plan was found for " << kNominalScenario;
  // take-off, fly out, survey, report, fly home, land: the plan the guide
  // shows on the Plan tab.
  EXPECT_EQ(plan.steps.size(), 6U);
}

TEST(FirstSurveyTutorial, EveryScenarioDoesWhatTheProjectExpects) {
  const ProjectModel model = loadTutorialProject();

  const ScenarioBatchReport report = ScenarioRunner::runAll(model);
  EXPECT_EQ(report.passCount, model.scenarios.size());
  EXPECT_EQ(report.failCount, 0U);
  EXPECT_EQ(report.errorCount, 0U);

  const ScenarioRunResult* nominal = findResult(report, kNominalScenario);
  ASSERT_NE(nominal, nullptr);
  EXPECT_TRUE(nominal->goalReached);
  EXPECT_EQ(nominal->replanCount, 0U);

  // The scenario whose saved fault makes the first survey attempt fail is the
  // one the guide uses to show replanning, so it has to actually replan.
  const ScenarioRunResult* faulted = findResult(report, "survey-both-sectors");
  ASSERT_NE(faulted, nullptr);
  EXPECT_TRUE(faulted->goalReached);
  EXPECT_GE(faulted->replanCount, 1U);

  // The scenario with no comms is expected to have no plan at all, which is
  // how the guide introduces the failure explanation.
  const ScenarioRunResult* noComms = findResult(report, "comms-lost-before-take-off");
  ASSERT_NE(noComms, nullptr);
  EXPECT_FALSE(noComms->planSucceeded);
  EXPECT_FALSE(noComms->goalReached);
}
