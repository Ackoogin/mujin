#include <gtest/gtest.h>

#include "project_model.h"
#include "structural_validator.h"

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

  ActionDef classify;
  classify.name = "classify";
  classify.params = {{"?r", "robot"}, {"?s", "sector"}};
  classify.preconditions.push_back({"at", {"?r", "?s"}});
  classify.preconditions.push_back({"searched", {"?s"}});
  classify.addEffects.push_back({"classified", {"?s"}});
  model.actions.push_back(classify);

  model.objects.push_back({"uav1", "robot"});
  model.objects.push_back({"base", "location"});
  model.objects.push_back({"sector_a", "sector"});

  ScenarioDef scenario;
  scenario.name = "baseline";
  scenario.initialState.push_back({"at", {"uav1", "base"}});
  scenario.goals.push_back({"classified", {"sector_a"}});
  model.scenarios.push_back(scenario);

  return model;
}

} // namespace

TEST(StructuralValidator, EmptyModelHasNoIssues) {
  const ProjectModel model;

  const StructuralReport report = StructuralValidator::check(model);

  EXPECT_EQ(report.errorCount, 0U);
  EXPECT_EQ(report.warningCount, 0U);
  EXPECT_TRUE(report.issues.empty());
}

TEST(StructuralValidator, EmptyPredicateNameIsError) {
  ProjectModel model;
  model.predicates.push_back({"", {}, 0.0F, 0.0F});

  const StructuralReport report = StructuralValidator::check(model);

  EXPECT_GE(report.errorCount, 1U);
  EXPECT_TRUE(report.hasErrors());
}

TEST(StructuralValidator, DuplicatePredicateNamesAreError) {
  ProjectModel model;
  model.predicates.push_back({"at", {}, 0.0F, 0.0F});
  model.predicates.push_back({"at", {}, 0.0F, 0.0F});

  const StructuralReport report = StructuralValidator::check(model);

  EXPECT_GE(report.errorCount, 1U);
}

TEST(StructuralValidator, UndeclaredTypeInActionParamIsError) {
  ProjectModel model;
  ActionDef action;
  action.name = "move";
  action.params.push_back({"?r", "robot"});
  model.actions.push_back(action);

  const StructuralReport report = StructuralValidator::check(model);

  EXPECT_GE(report.errorCount, 1U);
}

TEST(StructuralValidator, PredicateRefToMissingPredicateInActionIsError) {
  ProjectModel model;
  ActionDef action;
  action.name = "move";
  action.preconditions.push_back({"at", {"?r", "?l"}});
  model.actions.push_back(action);

  const StructuralReport report = StructuralValidator::check(model);

  EXPECT_GE(report.errorCount, 1U);
}

TEST(StructuralValidator, ActionWithZeroEffectsIsWarning) {
  ProjectModel model;
  ActionDef action;
  action.name = "inspect";
  model.actions.push_back(action);

  const StructuralReport report = StructuralValidator::check(model);

  EXPECT_EQ(report.errorCount, 0U);
  EXPECT_GE(report.warningCount, 1U);
}

TEST(StructuralValidator, ValidUavSearchHasZeroErrors) {
  const ProjectModel model = makeUavSearchModel();

  const StructuralReport report = StructuralValidator::check(model);

  EXPECT_EQ(report.errorCount, 0U);
}
