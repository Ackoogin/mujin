#include <gtest/gtest.h>

#include "command_stack.h"
#include "model_edits.h"
#include "project_model.h"
#include "recent_projects.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

namespace fs = std::filesystem;

/// A project where one type is used by everything that can use a type.
ProjectModel makeModel() {
  ProjectModel model;
  model.projectName = "edits";
  model.types = {{"location", "object"},
                 {"sector", "location"},
                 {"robot", "object"}};
  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back({"searched", {{"?s", "sector"}}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  move.preconditions = {{"at", {"?r", "?from"}}};
  move.addEffects = {{"at", {"?r", "?to"}}};
  move.delEffects = {{"at", {"?r", "?from"}}};
  model.actions.push_back(move);

  model.objects = {{"uav1", "robot"},
                   {"base", "location"},
                   {"sector-a", "sector"}};
  model.stateGroups.push_back({"where it is", "location", {"at"}});
  return model;
}

fs::path temporaryPath(const std::string& name) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return fs::temp_directory_path() /
         ("ame-edits-" + name + "-" + std::to_string(stamp) + ".json");
}

}  // namespace

TEST(ModelEdits, RenamingATypeReachesEverythingThatNamesIt) {
  ProjectModel model = makeModel();

  ASSERT_TRUE(ModelEdits::renameType(model, "location", "place"));

  EXPECT_EQ(model.types[0].name, "place");
  EXPECT_EQ(model.types[1].parent, "place") << "the child type follows";
  EXPECT_EQ(model.predicates[0].params[1].type, "place") << "the fact follows";
  EXPECT_EQ(model.actions[0].params[1].type, "place") << "the action follows";
  EXPECT_EQ(model.actions[0].params[2].type, "place");
  EXPECT_EQ(model.objects[1].type, "place") << "the object follows";
  EXPECT_EQ(model.stateGroups[0].type, "place") << "the group follows";
}

TEST(ModelEdits, RenamingATypeIsRefusedWhenItWouldClash) {
  ProjectModel model = makeModel();

  EXPECT_FALSE(ModelEdits::renameType(model, "location", "robot"));
  EXPECT_NE(ModelEdits::whyTypeCannotBeRenamed(model, "location", "robot")
                .find("already called"),
            std::string::npos);
  EXPECT_FALSE(ModelEdits::renameType(model, "location", ""));
  EXPECT_FALSE(ModelEdits::renameType(model, "nothing-like-this", "place"));
  EXPECT_EQ(model.types[0].name, "location") << "nothing changed";
}

TEST(ModelEdits, RenamingATypeIsOneUndoableStep) {
  ProjectModel model = makeModel();
  CommandStack stack;

  stack.execute(model, "Rename type", [](ProjectModel& m) {
    ModelEdits::renameType(m, "location", "place");
  });
  ASSERT_EQ(model.objects[1].type, "place");

  ASSERT_TRUE(stack.undo(model));
  EXPECT_EQ(model.types[0].name, "location");
  EXPECT_EQ(model.objects[1].type, "location");
  EXPECT_EQ(model.actions[0].params[1].type, "location");
}

TEST(ModelEdits, ParametersCanBeReorderedAndConditionsFollow) {
  ProjectModel model = makeModel();

  ASSERT_TRUE(ModelEdits::moveActionParameter(model, 0, 2, false));
  EXPECT_EQ(model.actions[0].params[1].name, "?to");
  EXPECT_EQ(model.actions[0].params[2].name, "?from");

  // The conditions name their parameters, so they are untouched by the move.
  EXPECT_EQ(model.actions[0].preconditions[0].argNames[1], "?from");
  EXPECT_EQ(model.actions[0].addEffects[0].argNames[1], "?to");
}

TEST(ModelEdits, AParameterCannotBeMovedOffEitherEnd) {
  ProjectModel model = makeModel();

  EXPECT_FALSE(ModelEdits::moveActionParameter(model, 0, 0, false));
  EXPECT_FALSE(ModelEdits::moveActionParameter(model, 0, 2, true));
  EXPECT_FALSE(ModelEdits::moveActionParameter(model, 9, 0, true));
  EXPECT_EQ(model.actions[0].params[0].name, "?r") << "nothing moved";
}

TEST(ModelEdits, PastingGivesTheCopyANameNothingElseUses) {
  ProjectModel model = makeModel();
  ElementClipboard clipboard;

  ASSERT_TRUE(ModelEdits::copyAction(model, 0, clipboard));
  EXPECT_EQ(clipboard.description(), "Paste 'move'");

  const std::string first = ModelEdits::paste(model, clipboard);
  EXPECT_EQ(first, "move copy");
  const std::string second = ModelEdits::paste(model, clipboard);
  EXPECT_EQ(second, "move copy 2");
  EXPECT_EQ(model.actions.size(), 3U);

  // The copy keeps the work that went into the original.
  EXPECT_EQ(model.actions[1].params.size(), model.actions[0].params.size());
  EXPECT_EQ(model.actions[1].preconditions.size(),
            model.actions[0].preconditions.size());
}

TEST(ModelEdits, PastingAFactWorksTheSameWay) {
  ProjectModel model = makeModel();
  ElementClipboard clipboard;

  ASSERT_TRUE(ModelEdits::copyFact(model, 1, clipboard));
  const std::string pasted = ModelEdits::paste(model, clipboard);

  EXPECT_EQ(pasted, "searched copy");
  EXPECT_EQ(model.predicates.size(), 3U);
  EXPECT_EQ(model.predicates.back().params.size(), 1U);
}

TEST(ModelEdits, AnEmptyClipboardPastesNothing) {
  ProjectModel model = makeModel();
  const ElementClipboard empty;

  EXPECT_FALSE(empty.holdsSomething());
  EXPECT_TRUE(ModelEdits::paste(model, empty).empty());
  EXPECT_EQ(model.actions.size(), 1U);
}

// ---------------------------------------------------------------------------
// Typing folds into one undoable step
// ---------------------------------------------------------------------------

TEST(CommandStackCoalescing, ARunOfKeystrokesIsOneUndoableStep) {
  ProjectModel model = makeModel();
  CommandStack stack;
  const std::string original = model.actions[0].name;

  // What typing "flyto" one letter at a time looks like to the stack.
  const std::string typed = "flyto";
  for (size_t length = 1; length <= typed.size(); ++length) {
    const std::string sofar = typed.substr(0, length);
    stack.executeCoalescing(model, "Rename the action", "action:0:name",
                            [sofar](ProjectModel& m) {
                              m.actions[0].name = sofar;
                            });
  }
  EXPECT_EQ(model.actions[0].name, "flyto");
  EXPECT_EQ(stack.undoDepth(), 1U) << "five keystrokes, one step";

  ASSERT_TRUE(stack.undo(model));
  EXPECT_EQ(model.actions[0].name, original)
      << "undo puts back the name before the typing started";
}

TEST(CommandStackCoalescing, TypingInAnotherFieldStartsANewStep) {
  ProjectModel model = makeModel();
  CommandStack stack;

  stack.executeCoalescing(model, "Rename the action", "action:0:name",
                          [](ProjectModel& m) { m.actions[0].name = "fly"; });
  stack.executeCoalescing(model, "Rename the fact", "fact:0:name",
                          [](ProjectModel& m) { m.predicates[0].name = "near"; });

  EXPECT_EQ(stack.undoDepth(), 2U);
  ASSERT_TRUE(stack.undo(model));
  EXPECT_EQ(model.predicates[0].name, "at");
  EXPECT_EQ(model.actions[0].name, "fly") << "the earlier field is untouched";
}

TEST(CommandStackCoalescing, ComingBackToAFieldAfterUndoStartsAgain) {
  ProjectModel model = makeModel();
  CommandStack stack;

  stack.executeCoalescing(model, "Rename the action", "action:0:name",
                          [](ProjectModel& m) { m.actions[0].name = "fly"; });
  ASSERT_TRUE(stack.undo(model));
  stack.executeCoalescing(model, "Rename the action", "action:0:name",
                          [](ProjectModel& m) { m.actions[0].name = "land"; });

  EXPECT_EQ(model.actions[0].name, "land");
  EXPECT_EQ(stack.undoDepth(), 1U);
  ASSERT_TRUE(stack.undo(model));
  EXPECT_EQ(model.actions[0].name, "move") << "back to where it started";
}

TEST(CommandStackCoalescing, TheEditCountMovesWhereTheUndoDepthDoesNot) {
  ProjectModel model = makeModel();
  CommandStack stack;

  const size_t before = stack.editCount();
  stack.executeCoalescing(model, "Rename the action", "action:0:name",
                          [](ProjectModel& m) { m.actions[0].name = "f"; });
  stack.executeCoalescing(model, "Rename the action", "action:0:name",
                          [](ProjectModel& m) { m.actions[0].name = "fl"; });

  // Two keystrokes fold into one undoable step, but both are edits, and
  // anything asking "has this changed since I last looked" must see both.
  EXPECT_EQ(stack.undoDepth(), 1U);
  EXPECT_EQ(stack.editCount(), before + 2U);

  const size_t beforeUndo = stack.editCount();
  ASSERT_TRUE(stack.undo(model));
  EXPECT_GT(stack.editCount(), beforeUndo) << "undoing is a change too";
}

TEST(CommandStackCoalescing, TheEditCountKeepsRisingPastTheUndoLimit) {
  ProjectModel model = makeModel();
  CommandStack stack(4);  // a deliberately tiny limit

  for (int i = 0; i < 10; ++i) {
    stack.execute(model, "Add a fact", [i](ProjectModel& m) {
      PredicateDef fact;
      fact.name = "fact" + std::to_string(i);
      m.predicates.push_back(fact);
    });
  }

  // The stack only remembers four, but ten edits happened, and a project with
  // ten unsaved edits must not look unchanged.
  EXPECT_EQ(stack.undoDepth(), 4U);
  EXPECT_EQ(stack.editCount(), 10U);
}

TEST(CommandStackCoalescing, TheMenuCanSayWhatWouldBeUndone) {
  ProjectModel model = makeModel();
  CommandStack stack;

  stack.execute(model, "Rename type", [](ProjectModel& m) {
    ModelEdits::renameType(m, "location", "place");
  });
  EXPECT_EQ(stack.topUndoLabel(), "Rename type");
}

// ---------------------------------------------------------------------------
// Recent projects and the recovery copy
// ---------------------------------------------------------------------------

TEST(RecentProjects, TheMostRecentComesFirstAndNothingRepeats) {
  const fs::path settings = temporaryPath("recent");
  const fs::path first = temporaryPath("one");
  const fs::path second = temporaryPath("two");
  ASSERT_TRUE(makeModel().save(first.string()));
  ASSERT_TRUE(makeModel().save(second.string()));

  ASSERT_TRUE(RecentProjects::remember(settings.string(), first.string()));
  ASSERT_TRUE(RecentProjects::remember(settings.string(), second.string()));
  ASSERT_TRUE(RecentProjects::remember(settings.string(), first.string()));

  const std::vector<std::string> recent =
      RecentProjects::load(settings.string());
  ASSERT_EQ(recent.size(), 2U) << "opening one twice does not list it twice";
  EXPECT_EQ(recent[0], first.string());
  EXPECT_EQ(recent[1], second.string());

  fs::remove(settings);
  fs::remove(first);
  fs::remove(second);
}

TEST(RecentProjects, AProjectThatHasGoneIsNotOffered) {
  const fs::path settings = temporaryPath("recent-missing");
  const fs::path project = temporaryPath("gone");
  ASSERT_TRUE(makeModel().save(project.string()));
  ASSERT_TRUE(RecentProjects::remember(settings.string(), project.string()));
  fs::remove(project);

  EXPECT_TRUE(RecentProjects::load(settings.string()).empty());
  fs::remove(settings);
}

TEST(RecentProjects, AnUnreadableSettingsFileIsNotWorthStoppingOver) {
  const fs::path settings = temporaryPath("recent-broken");
  std::ofstream(settings.string()) << "this is not JSON at all";

  EXPECT_TRUE(RecentProjects::load(settings.string()).empty());
  fs::remove(settings);
}

TEST(RecentProjects, TheRecoveryCopySitsBesideTheProject) {
  EXPECT_EQ(RecentProjects::recoveryPathFor("/work/mission.ameproj.json"),
            "/work/mission.ameproj.json.recovery");
  EXPECT_TRUE(RecentProjects::recoveryPathFor("").empty());
}
