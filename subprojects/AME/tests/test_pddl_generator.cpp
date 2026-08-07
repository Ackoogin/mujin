#include <gtest/gtest.h>

#include "ame/pddl_parser.h"
#include "ame/world_model.h"
#include "pddl_generator.h"

#include <string>

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
  model.objects.push_back({"sector_b", "sector"});

  ScenarioDef scenario;
  scenario.name = "uav-search-1";
  scenario.initialState.push_back({"at", {"uav1", "base"}});
  scenario.goals.push_back({"searched", {"sector_a"}});
  scenario.goals.push_back({"classified", {"sector_a"}});
  model.scenarios.push_back(scenario);

  return model;
}

void expectContains(const std::string& text, const std::string& needle) {
  EXPECT_NE(text.find(needle), std::string::npos) << "missing: " << needle;
}

} // namespace

TEST(PddlGenerator, GenerateUavSearchDomain) {
  const ProjectModel model = makeUavSearchModel();
  const std::string domain = PddlGenerator::generateDomain(model);

  expectContains(domain, "(define (domain uav-search)");
  expectContains(domain, "(:requirements :strips :typing)");
  expectContains(domain, "(:types");
  expectContains(domain, "location - object");
  expectContains(domain, "sector - location");
  expectContains(domain, "robot - object");
  expectContains(domain, "(at ?r - robot ?l - location)");
  expectContains(domain, "(:action move");
  expectContains(domain, ":parameters (?r - robot ?from - location ?to - location)");
  expectContains(domain, ":precondition (at ?r ?from)");
  expectContains(domain, "(not (at ?r ?from))");
}

TEST(PddlGenerator, GenerateUavSearchProblem) {
  const ProjectModel model = makeUavSearchModel();
  const std::string problem = PddlGenerator::generateProblem(model, "uav-search-1");

  expectContains(problem, "(define (problem uav-search-1)");
  expectContains(problem, "(:domain uav-search)");
  expectContains(problem, "(:objects");
  expectContains(problem, "uav1 - robot");
  expectContains(problem, "sector_a - sector");
  expectContains(problem, "(:init");
  expectContains(problem, "(at uav1 base)");
  expectContains(problem, "(:goal");
  expectContains(problem, "(searched sector_a)");
  expectContains(problem, "(classified sector_a)");
}

TEST(PddlGenerator, GenerateProblemDeclaresLiteralActionArguments) {
  ProjectModel model;
  model.projectName = "literal-args";
  model.types.push_back({"mission", "object"});
  model.types.push_back({"priority", "object"});
  model.predicates.push_back({"mission-active", {{"?m", "mission"}}, 0.0F, 0.0F});
  model.predicates.push_back({"has-priority",
                              {{"?m", "mission"}, {"?p", "priority"}},
                              0.0F,
                              0.0F});
  model.predicates.push_back({"mission-complete", {{"?m", "mission"}}, 0.0F, 0.0F});

  ActionDef complete;
  complete.name = "complete-critical";
  complete.params = {{"?m", "mission"}};
  complete.preconditions.push_back({"mission-active", {"?m"}});
  complete.preconditions.push_back({"has-priority", {"?m", "critical"}});
  complete.addEffects.push_back({"mission-complete", {"?m"}});
  model.actions.push_back(complete);

  model.objects.push_back({"mission-1", "mission"});
  ScenarioDef scenario;
  scenario.name = "nominal";
  scenario.initialState.push_back({"mission-active", {"mission-1"}});
  scenario.initialState.push_back({"has-priority", {"mission-1", "critical"}});
  scenario.goals.push_back({"mission-complete", {"mission-1"}});
  model.scenarios.push_back(scenario);

  const std::string problem = PddlGenerator::generateProblem(model, "nominal");
  expectContains(problem, "mission-1 - mission");
  expectContains(problem, "critical - priority");

  ame::WorldModel wm;
  EXPECT_NO_THROW(ame::PddlParser::parseFromString(
      PddlGenerator::generateDomain(model), problem, wm));
  EXPECT_GT(wm.numGroundActions(), 0U);
}

TEST(PddlGenerator, RoundTripsThroughPddlParser) {
  const ProjectModel model = makeUavSearchModel();
  const std::string domain = PddlGenerator::generateDomain(model);
  const std::string problem = PddlGenerator::generateProblem(model, "uav-search-1");

  ame::WorldModel wm;
  EXPECT_NO_THROW(ame::PddlParser::parseFromString(domain, problem, wm));
  EXPECT_EQ(wm.numFluents(), 7U);
  EXPECT_EQ(wm.numGroundActions(), 13U);
  EXPECT_TRUE(wm.getFact("(at uav1 base)"));
}

// The declaration is only worth writing if the core parser picks it up again,
// because that is what makes the plan compiler treat the fact as one an action
// has to observe rather than one it may assume.
TEST(PddlGenerator, ConfirmedPredicatesReachTheWorldModel) {
  ProjectModel model = makeUavSearchModel();
  for (auto& predicate : model.predicates) {
    if (predicate.name == "searched") {
      predicate.confirmed = true;
    }
  }

  const std::string domain = PddlGenerator::generateDomain(model);
  expectContains(domain, "(:confirmed-predicates searched)");

  ame::WorldModel wm;
  ASSERT_NO_THROW(ame::PddlParser::parseFromString(
      domain, PddlGenerator::generateProblem(model, "uav-search-1"), wm));
  EXPECT_TRUE(wm.isConfirmedPredicate("searched"));
  EXPECT_FALSE(wm.isConfirmedPredicate("at"));
}
