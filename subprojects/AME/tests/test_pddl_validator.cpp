#include <gtest/gtest.h>

#include "pddl_validator.h"
#include "project_model.h"

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

  return model;
}

} // namespace

TEST(PddlValidator, ValidatesUavSearchDomainOk) {
  const ProjectModel model = makeUavSearchModel();
  const ValidationReport report = PddlValidator::validate(model, "");
  EXPECT_TRUE(report.ok);
  EXPECT_TRUE(report.errors.empty());
}

TEST(PddlValidator, EmptyDomainNameStillProducesSomething) {
  ProjectModel model = makeUavSearchModel();
  model.projectName.clear();

  const ValidationReport report = PddlValidator::validate(model, "");
  EXPECT_TRUE(report.ok);
}

TEST(PddlValidator, ReportsErrorWhenPredicateNameEmpty) {
  ProjectModel model = makeUavSearchModel();
  model.predicates[0].name.clear();

  const ValidationReport report = PddlValidator::validate(model, "");
  EXPECT_FALSE(report.ok);
  EXPECT_GE(report.errors.size(), 1U);
}
