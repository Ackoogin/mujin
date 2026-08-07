#include <gtest/gtest.h>

#include "assurance_report.h"
#include "project_model.h"

#include <string>

namespace {

/// A mission with one thing the world decides, one action bound to something
/// that could execute it, and one that is bound to nothing.
ProjectModel makeModel() {
  ProjectModel model;
  model.projectName = "evidence-test";
  model.types = {{"location", "object"}, {"robot", "object"}};
  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back({"searched", {{"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back({"comms-available", {}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  move.preconditions = {{"at", {"?r", "?from"}}, {"comms-available", {}}};
  move.addEffects = {{"at", {"?r", "?to"}}};
  move.delEffects = {{"at", {"?r", "?from"}}};
  move.btBinding.nodeType = "MoveToLocation";
  model.actions.push_back(move);

  ActionDef search;
  search.name = "search";
  search.params = {{"?r", "robot"}, {"?l", "location"}};
  search.preconditions = {{"at", {"?r", "?l"}}};
  search.addEffects = {{"searched", {"?l"}}};
  model.actions.push_back(search);  // nothing bound to it

  model.objects = {{"uav1", "robot"}, {"base", "location"},
                   {"sector-a", "location"}};

  ScenarioDef nominal;
  nominal.name = "nominal";
  nominal.initialState = {{"at", {"uav1", "base"}}, {"comms-available", {}}};
  nominal.goals = {{"searched", {"sector-a"}}};
  model.scenarios.push_back(nominal);
  return model;
}

bool mentions(const std::string& report, const std::string& text) {
  return report.find(text) != std::string::npos;
}

}  // namespace

TEST(AssuranceReport, ItNamesTheModelAndWhatIsInIt) {
  const std::string report = AssuranceReport::generate(makeModel());

  EXPECT_TRUE(mentions(report, "evidence-test"));
  EXPECT_TRUE(mentions(report, "What the model contains"));
  EXPECT_TRUE(mentions(report, "| Actions | 2 |"));
  EXPECT_TRUE(mentions(report, "| Scenarios | 1 |"));
}

TEST(AssuranceReport, ItNamesTheFactsNothingBringsAbout) {
  const std::string report = AssuranceReport::generate(makeModel());

  EXPECT_TRUE(mentions(report, "Facts that nothing in the mission brings about"));
  EXPECT_TRUE(mentions(report, "comms-available"));
  // And says who relies on it, which is what makes it worth reading.
  EXPECT_TRUE(mentions(report, "| comms-available | move |"));
  // A fact an action does make true is not listed there.
  EXPECT_FALSE(mentions(report, "| searched | "));
}

TEST(AssuranceReport, ItSaysWhichActionsHaveNothingToCarryThemOut) {
  const std::string report = AssuranceReport::generate(makeModel());

  EXPECT_TRUE(mentions(report, "| move | MoveToLocation |"));
  EXPECT_TRUE(mentions(report, "| search | **nothing** |"));
  EXPECT_TRUE(mentions(report, "1 of 2 actions have nothing bound to them"));
}

TEST(AssuranceReport, ItReportsHowEveryScenarioBehaved) {
  const std::string report = AssuranceReport::generate(makeModel());

  EXPECT_TRUE(mentions(report, "Scenarios, and how they behaved"));
  EXPECT_TRUE(mentions(report, "1 of 1 scenarios behaved"));
  EXPECT_TRUE(mentions(report, "| nominal | as expected | reached |"));
  EXPECT_TRUE(mentions(report, "repeated from seed"));
}

TEST(AssuranceReport, ItSaysPlainlyWhenNoContingencyHasBeenChecked) {
  const std::string report = AssuranceReport::generate(makeModel());

  // The acceptance for this item is that a reviewer can tell what has not been
  // checked. An unchecked contingency is the most important instance of that.
  EXPECT_TRUE(mentions(report, "no contingency has been checked"));
  EXPECT_TRUE(mentions(report, "carries no evidence that a safe state stays "
                               "reachable"));
}

TEST(AssuranceReport, ADeclaredContingencyIsCheckedAndReported) {
  ProjectModel model = makeModel();
  model.scenarios[0].contingency.contingencyPredicates = {"comms-available"};
  model.scenarios[0].contingency.safeState = {{"at", {"uav1", "base"}}};

  const std::string report = AssuranceReport::generate(model);

  EXPECT_TRUE(mentions(report, "Contingencies, and what stayed reachable"));
  EXPECT_TRUE(mentions(report, "### nominal"));
  EXPECT_TRUE(mentions(report, "A safe state was reachable in"));
  EXPECT_FALSE(mentions(report, "no contingency has been checked"));
}

TEST(AssuranceReport, AContingencyWithNoWayBackIsCalledOut) {
  ProjectModel model = makeModel();
  // Moving needs comms, and there is no other way to get anywhere, so with
  // comms down there is no way back to base from the sector.
  model.scenarios[0].initialState = {{"at", {"uav1", "sector-a"}},
                                     {"comms-available", {}}};
  model.scenarios[0].contingency.contingencyPredicates = {"comms-available"};
  model.scenarios[0].contingency.safeState = {{"at", {"uav1", "base"}}};

  const std::string report = AssuranceReport::generate(model);

  EXPECT_TRUE(mentions(report, "leave no way back to a safe state"));
}

TEST(AssuranceReport, ItAlwaysSaysWhatItDoesNotCover) {
  const std::string report = AssuranceReport::generate(makeModel());

  EXPECT_TRUE(mentions(report, "What this report does not tell you"));
  EXPECT_TRUE(mentions(report, "Nothing here is evidence about the field"));
  EXPECT_TRUE(mentions(report,
                       "A scenario that behaved as expected is not a scenario "
                       "that is\n  right"));
}

TEST(AssuranceReport, AModelWithNoScenariosSaysNothingHasBeenRun) {
  ProjectModel model = makeModel();
  model.scenarios.clear();

  const std::string report = AssuranceReport::generate(model);

  EXPECT_TRUE(mentions(report, "no scenarios, so nothing has been run at all"));
}

TEST(AssuranceReport, AFactThatMustBeObservedIsListed) {
  ProjectModel model = makeModel();
  model.predicates[2].confirmed = true;  // comms-available

  const std::string report = AssuranceReport::generate(model);

  EXPECT_TRUE(mentions(report, "only once observed"));
  EXPECT_TRUE(mentions(report, "- comms-available"));
}

TEST(AssuranceReport, AModelThatDoesNotHoldTogetherSaysSoFirst) {
  ProjectModel model = makeModel();
  model.actions[0].preconditions.push_back({"refuelled", {"?r"}});

  const std::string report = AssuranceReport::generate(model);

  EXPECT_TRUE(mentions(report, "structural error"));
  EXPECT_TRUE(mentions(report, "Nothing below should be relied on"));
}

TEST(AssuranceReport, TheDateIsIncludedWhenOneIsGiven) {
  const std::string report =
      AssuranceReport::generate(makeModel(), "2026-08-07");

  EXPECT_TRUE(mentions(report, "on 2026-08-07"));
}
