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
