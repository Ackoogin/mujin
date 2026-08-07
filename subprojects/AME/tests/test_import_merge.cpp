#include <gtest/gtest.h>

#include "import_merge.h"
#include "project_model.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>

namespace {

ProjectModel makeCurrent() {
  ProjectModel model;
  model.projectName = "current";
  model.types = {{"location", "object"}, {"robot", "object"}};
  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 10.0F, 20.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  move.preconditions = {{"at", {"?r", "?from"}}};
  move.addEffects = {{"at", {"?r", "?to"}}};
  move.delEffects = {{"at", {"?r", "?from"}}};
  move.posX = 100.0F;
  move.posY = 200.0F;
  move.btBinding.nodeType = "MoveToLocation";
  move.simulation.ticks = 7;
  model.actions.push_back(move);

  model.objects = {{"uav1", "robot"}, {"base", "location"}};
  return model;
}

/// A second domain: it knows "move" differently, and brings new things too.
ProjectModel makeIncoming() {
  ProjectModel model;
  model.projectName = "incoming";
  model.types = {{"location", "object"}, {"sector", "location"}};
  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back({"searched", {{"?s", "sector"}}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?to", "location"}};  // different shape
  move.addEffects = {{"at", {"?r", "?to"}}};
  model.actions.push_back(move);

  ActionDef search;
  search.name = "search";
  search.params = {{"?r", "robot"}, {"?s", "sector"}};
  search.addEffects = {{"searched", {"?s"}}};
  model.actions.push_back(search);

  model.objects = {{"uav1", "robot"}, {"sector-a", "sector"}};
  return model;
}

const MergeItem* find(const MergePlan& plan, const std::string& name) {
  const auto it = std::find_if(plan.items.begin(), plan.items.end(),
                               [&name](const MergeItem& item) {
                                 return item.name == name;
                               });
  return it == plan.items.end() ? nullptr : &(*it);
}

}  // namespace

TEST(ImportMerge, ThePlanSaysWhatWouldBeAddedAndWhatWouldBeOverwritten) {
  const MergePlan plan = ImportMerge::plan(makeCurrent(), makeIncoming());

  const MergeItem* move = find(plan, "move");
  ASSERT_NE(move, nullptr);
  EXPECT_EQ(move->disposition, MergeDisposition::Replaced);
  EXPECT_FALSE(move->whatWouldBeLost.empty())
      << "the user is told what replacing it costs";

  const MergeItem* search = find(plan, "search");
  ASSERT_NE(search, nullptr);
  EXPECT_EQ(search->disposition, MergeDisposition::Added);

  const MergeItem* at = find(plan, "at");
  ASSERT_NE(at, nullptr);
  EXPECT_EQ(at->disposition, MergeDisposition::Unchanged)
      << "the same fact by the same name is not a change";

  EXPECT_TRUE(plan.anythingWouldBeOverwritten());
  const std::vector<std::string> overwritten = plan.wouldOverwrite();
  EXPECT_NE(std::find(overwritten.begin(), overwritten.end(), "move"),
            overwritten.end());
}

TEST(ImportMerge, RefusingToReplaceKeepsTheProjectsOwnWork) {
  const ProjectModel current = makeCurrent();
  MergeChoices choices;  // replace nothing
  const ProjectModel merged =
      ImportMerge::apply(current, makeIncoming(), choices);

  const auto move = std::find_if(merged.actions.begin(), merged.actions.end(),
                                 [](const ActionDef& action) {
                                   return action.name == "move";
                                 });
  ASSERT_NE(move, merged.actions.end());
  EXPECT_EQ(move->params.size(), 3U) << "the project's own move is untouched";
  EXPECT_EQ(move->preconditions.size(), 1U);

  // What was new still arrives: adding costs nothing, so it is not a choice.
  EXPECT_EQ(std::count_if(merged.actions.begin(), merged.actions.end(),
                          [](const ActionDef& action) {
                            return action.name == "search";
                          }),
            1);
  EXPECT_EQ(std::count_if(merged.predicates.begin(), merged.predicates.end(),
                          [](const PredicateDef& fact) {
                            return fact.name == "searched";
                          }),
            1);
}

TEST(ImportMerge, AgreeingToReplaceKeepsWhatTheImportCannotKnow) {
  MergeChoices choices;
  choices.replaceActions = true;
  const ProjectModel merged =
      ImportMerge::apply(makeCurrent(), makeIncoming(), choices);

  const auto move = std::find_if(merged.actions.begin(), merged.actions.end(),
                                 [](const ActionDef& action) {
                                   return action.name == "move";
                                 });
  ASSERT_NE(move, merged.actions.end());
  EXPECT_EQ(move->params.size(), 2U) << "the imported shape won";
  // The PDDL says nothing about any of these, so replacing it must not lose
  // them: they are this project's work, not the file's.
  EXPECT_EQ(move->btBinding.nodeType, "MoveToLocation");
  EXPECT_EQ(move->simulation.ticks, 7);
  EXPECT_FLOAT_EQ(move->posX, 100.0F);
  EXPECT_FLOAT_EQ(move->posY, 200.0F);
}

TEST(ImportMerge, NothingIsLostFromTheProjectThatTheImportDoesNotMention) {
  ProjectModel current = makeCurrent();
  ScenarioDef scenario;
  scenario.name = "nominal";
  scenario.initialState = {{"at", {"uav1", "base"}}};
  current.scenarios.push_back(scenario);

  MergeChoices choices;
  choices.replaceActions = true;
  choices.replaceFacts = true;
  choices.replaceTypes = true;
  choices.replaceObjects = true;
  const ProjectModel merged =
      ImportMerge::apply(current, makeIncoming(), choices);

  ASSERT_EQ(merged.scenarios.size(), 1U) << "scenarios are the user's, and stay";
  EXPECT_EQ(merged.scenarios[0].name, "nominal");
  EXPECT_EQ(merged.projectName, "current") << "the project keeps its name";
}

TEST(ImportMerge, ImportedElementsAreLaidOutByWhatTheyHaveToDoWithEachOther) {
  ProjectModel model = makeIncoming();
  ImportMerge::layoutByRelationships(model);

  // Two actions, so two columns: they do not sit on top of each other.
  ASSERT_EQ(model.actions.size(), 2U);
  EXPECT_NE(model.actions[0].posX, model.actions[1].posX);

  // A fact an action uses sits below that action rather than in a row of its
  // own somewhere else.
  const auto searched =
      std::find_if(model.predicates.begin(), model.predicates.end(),
                   [](const PredicateDef& fact) {
                     return fact.name == "searched";
                   });
  ASSERT_NE(searched, model.predicates.end());
  const ActionDef& search = model.actions[1];
  EXPECT_GT(searched->posY, search.posY) << "below the action that makes it true";
  EXPECT_NEAR(searched->posX, search.posX, 100.0F) << "and beside it";
}

// ---------------------------------------------------------------------------
// Named groups and saved views live in the project file
// ---------------------------------------------------------------------------

TEST(ProjectFormat, GroupsAndSavedViewsSurviveSaveAndLoad) {
  ProjectModel model = makeCurrent();
  PresentationGroup group;
  group.name = "communications loss";
  group.predicateNames = {"at"};
  group.actionNames = {"move"};
  group.collapsed = true;
  model.presentationGroups.push_back(group);

  SavedView view;
  view.name = "how the vehicle moves";
  view.focusAction = "move";
  view.depth = 2;
  view.relationshipFilter = 5U;
  view.viewMode = 1;
  model.savedViews.push_back(view);

  const nlohmann::json json = model;
  const ProjectModel reloaded = json.get<ProjectModel>();

  ASSERT_EQ(reloaded.presentationGroups.size(), 1U);
  EXPECT_EQ(reloaded.presentationGroups[0].name, "communications loss");
  EXPECT_TRUE(reloaded.presentationGroups[0].collapsed);
  EXPECT_EQ(reloaded.presentationGroups[0].actionNames.size(), 1U);

  ASSERT_EQ(reloaded.savedViews.size(), 1U);
  EXPECT_EQ(reloaded.savedViews[0].name, "how the vehicle moves");
  EXPECT_EQ(reloaded.savedViews[0].focusAction, "move");
  EXPECT_EQ(reloaded.savedViews[0].depth, 2);
  EXPECT_EQ(reloaded.savedViews[0].relationshipFilter, 5U);
}

TEST(ProjectFormat, AProjectSavedBeforeGroupsExistedLoadsWithNone) {
  nlohmann::json json = makeCurrent();
  json.erase("presentationGroups");
  json.erase("savedViews");

  const ProjectModel reloaded = json.get<ProjectModel>();

  EXPECT_TRUE(reloaded.presentationGroups.empty());
  EXPECT_TRUE(reloaded.savedViews.empty());
}

TEST(ProjectFormat, ADeclaredContingencySurvivesSaveAndLoad) {
  ProjectModel model = makeCurrent();
  ScenarioDef scenario;
  scenario.name = "comms lost";
  scenario.contingency.contingencyPredicates = {"comms-available"};
  scenario.contingency.safeState.push_back({"at", {"uav1", "base"}});
  model.scenarios.push_back(scenario);

  const nlohmann::json json = model;
  const ProjectModel reloaded = json.get<ProjectModel>();

  ASSERT_EQ(reloaded.scenarios.size(), 1U);
  const ContingencyDeclaration& declaration = reloaded.scenarios[0].contingency;
  EXPECT_FALSE(declaration.isEmpty());
  ASSERT_EQ(declaration.contingencyPredicates.size(), 1U);
  EXPECT_EQ(declaration.contingencyPredicates[0], "comms-available");
  ASSERT_EQ(declaration.safeState.size(), 1U);
  EXPECT_EQ(declaration.safeState[0].predicateName, "at");
}
