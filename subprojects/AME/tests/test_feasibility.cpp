#include <gtest/gtest.h>

#include "pddl_validator.h"
#include "project_model.h"

#include <ame/planner.h>
#include <ame/world_model.h>

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
  model.predicates.push_back({"classified", {{"?s", "sector"}}, 0.0F, 0.0F});

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

  return model;
}

void addObjects(ProjectModel& model) {
  model.objects.push_back({"uav1", "robot"});
  model.objects.push_back({"base", "location"});
  model.objects.push_back({"sector_a", "sector"});
}

} // namespace

TEST(Feasibility, FeasibleTrivialGoal) {
  ProjectModel model = makeUavSearchModel();
  addObjects(model);

  ScenarioDef scenario;
  scenario.name = "nominal";
  scenario.initialState.push_back({"at", {"uav1", "base"}});
  scenario.goals.push_back({"at", {"uav1", "base"}});
  model.scenarios.push_back(std::move(scenario));

  ame::WorldModel wm;
  const ValidationReport report =
      PddlValidator::validateAndBuildWorldModel(model, "nominal", wm);
  ASSERT_TRUE(report.ok);

  const ame::PlanResult result = ame::Planner{}.solve(wm);
  EXPECT_TRUE(result.success);
}

TEST(Feasibility, InfeasibleUnreachableGoal) {
  ProjectModel model = makeUavSearchModel();
  addObjects(model);
  model.actions.erase(model.actions.begin() + 1);

  ScenarioDef scenario;
  scenario.name = "blocked";
  scenario.initialState.push_back({"at", {"uav1", "base"}});
  scenario.goals.push_back({"searched", {"sector_a"}});
  model.scenarios.push_back(std::move(scenario));

  ame::WorldModel wm;
  const ValidationReport report =
      PddlValidator::validateAndBuildWorldModel(model, "blocked", wm);
  ASSERT_TRUE(report.ok);

  const ame::PlanResult result = ame::Planner{}.solve(wm);
  EXPECT_FALSE(result.success);
}
