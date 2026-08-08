// Tests for named groups of facts and actions on the whole-domain canvas.
//
// The rules and the resulting picture live in a library rather than in the
// drawing code precisely so that they can be checked without a window, which is
// what this file does. Nothing here opens a canvas.

#include "presentation_groups.h"
#include "project_model.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

/// A small domain with two facts and two actions, wired so that every kind of
/// line appears at least once: "move" needs (at-base), makes (at-sector) true
/// and (at-base) false; "search" needs (at-sector).
ProjectModel sampleModel() {
  ProjectModel model;
  model.types.push_back({"vehicle", "object"});

  PredicateDef atBase;
  atBase.name = "at-base";
  atBase.params.push_back({"v", "vehicle"});
  PredicateDef atSector;
  atSector.name = "at-sector";
  atSector.params.push_back({"v", "vehicle"});
  model.predicates.push_back(atBase);
  model.predicates.push_back(atSector);

  ActionDef move;
  move.name = "move";
  move.params.push_back({"v", "vehicle"});
  move.preconditions.push_back({"at-base", {"v"}});
  move.addEffects.push_back({"at-sector", {"v"}});
  move.delEffects.push_back({"at-base", {"v"}});

  ActionDef search;
  search.name = "search";
  search.params.push_back({"v", "vehicle"});
  search.preconditions.push_back({"at-sector", {"v"}});

  model.actions.push_back(move);
  model.actions.push_back(search);
  return model;
}

size_t countNodes(const CanvasLayout& layout, CanvasNodeKind kind) {
  size_t total = 0;
  for (const CanvasNode& node : layout.nodes) {
    if (node.kind == kind) {
      ++total;
    }
  }
  return total;
}

}  // namespace

// ---- The rules for making a group ------------------------------------------

TEST(PresentationGroups, MakesAGroupFromFactsAndActions) {
  ProjectModel model = sampleModel();
  EXPECT_TRUE(PresentationGroups::create(model, "at the base", {"at-base"},
                                         {"move"}));
  ASSERT_EQ(model.presentationGroups.size(), 1U);
  EXPECT_EQ(model.presentationGroups[0].name, "at the base");
  EXPECT_FALSE(model.presentationGroups[0].collapsed);
}

TEST(PresentationGroups, RefusesAGroupWithNoName) {
  const ProjectModel model = sampleModel();
  EXPECT_EQ(PresentationGroups::whyGroupCannotBeMade(model, "", {"at-base"}, {}),
            "a group needs a name");
}

TEST(PresentationGroups, RefusesAGroupHoldingNothing) {
  const ProjectModel model = sampleModel();
  EXPECT_EQ(PresentationGroups::whyGroupCannotBeMade(model, "empty", {}, {}),
            "a group has to hold at least one fact or action");
}

TEST(PresentationGroups, RefusesASecondGroupWithTheSameName) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "sector work", {"at-sector"}, {}));
  EXPECT_EQ(PresentationGroups::whyGroupCannotBeMade(model, "sector work", {},
                                                     {"search"}),
            "another group is already called 'sector work'");
  EXPECT_FALSE(
      PresentationGroups::create(model, "sector work", {}, {"search"}));
}

TEST(PresentationGroups, RefusesSomethingThatIsNotInTheProject) {
  const ProjectModel model = sampleModel();
  EXPECT_EQ(PresentationGroups::whyGroupCannotBeMade(model, "made up",
                                                     {"at-orbit"}, {}),
            "there is no fact called 'at-orbit' in this project");
  EXPECT_EQ(PresentationGroups::whyGroupCannotBeMade(model, "made up", {},
                                                     {"teleport"}),
            "there is no action called 'teleport' in this project");
}

// An element in two groups would make collapsing ambiguous: two boxes would
// each claim to stand for the same thing. The refusal says why, so the user
// learns the rule rather than wondering where the option went.
TEST(PresentationGroups, RefusesSomethingAlreadyInAnotherGroup) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {"move"}));

  const std::string factReason =
      PresentationGroups::whyGroupCannotBeMade(model, "second", {"at-base"}, {});
  EXPECT_NE(factReason.find("'at-base' is already in the group 'base'"),
            std::string::npos);
  EXPECT_NE(factReason.find("can only be in one group"), std::string::npos);

  const std::string actionReason =
      PresentationGroups::whyGroupCannotBeMade(model, "second", {}, {"move"});
  EXPECT_NE(actionReason.find("'move' is already in the group 'base'"),
            std::string::npos);

  EXPECT_FALSE(PresentationGroups::create(model, "second", {"at-base"}, {}));
  EXPECT_EQ(model.presentationGroups.size(), 1U);
}

TEST(PresentationGroups, LetsAGroupKeepItsOwnContentsWhenBeingChanged) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {"move"}));
  // Checking the same group against itself must not report a clash with itself.
  EXPECT_EQ(PresentationGroups::whyGroupCannotBeMade(model, "base", {"at-base"},
                                                     {"move"}, 0),
            "");
}

// ---- Renaming, removing and membership --------------------------------------

TEST(PresentationGroups, RenamesAGroupAndRefusesAClash) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {}));
  ASSERT_TRUE(PresentationGroups::create(model, "sector", {"at-sector"}, {}));

  EXPECT_EQ(PresentationGroups::whyGroupCannotBeRenamed(model, 0, "sector"),
            "another group is already called 'sector'");
  EXPECT_FALSE(PresentationGroups::rename(model, 0, "sector"));
  EXPECT_EQ(PresentationGroups::whyGroupCannotBeRenamed(model, 0, "base"),
            "that is the name it already has");

  EXPECT_TRUE(PresentationGroups::rename(model, 0, "at the base"));
  EXPECT_EQ(model.presentationGroups[0].name, "at the base");
}

TEST(PresentationGroups, RemovingAGroupLeavesTheModelAlone) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {"move"}));
  ASSERT_TRUE(PresentationGroups::remove(model, 0));
  EXPECT_TRUE(model.presentationGroups.empty());
  EXPECT_EQ(model.predicates.size(), 2U);
  EXPECT_EQ(model.actions.size(), 2U);
}

TEST(PresentationGroups, AddsAndRemovesMembers) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {}));

  EXPECT_TRUE(PresentationGroups::addMember(model, 0, "move", true));
  EXPECT_EQ(PresentationGroups::groupHoldingAction(model, "move"), 0);
  // Something already in this group, and something that does not exist.
  EXPECT_FALSE(PresentationGroups::addMember(model, 0, "move", true));
  EXPECT_FALSE(PresentationGroups::addMember(model, 0, "teleport", true));

  EXPECT_TRUE(PresentationGroups::removeMember(model, 0, "move", true));
  EXPECT_EQ(PresentationGroups::groupHoldingAction(model, "move"), -1);
}

TEST(PresentationGroups, RemovingTheLastMemberRemovesTheGroup) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {}));
  EXPECT_TRUE(PresentationGroups::removeMember(model, 0, "at-base", false));
  EXPECT_TRUE(model.presentationGroups.empty());
}

TEST(PresentationGroups, DescribesWhatAGroupHolds) {
  PresentationGroup group;
  EXPECT_EQ(PresentationGroups::describeContents(group), "nothing");
  group.predicateNames.push_back("at-base");
  EXPECT_EQ(PresentationGroups::describeContents(group), "1 fact");
  group.predicateNames.push_back("at-sector");
  group.actionNames.push_back("move");
  EXPECT_EQ(PresentationGroups::describeContents(group), "2 facts and 1 action");
}

// ---- Membership follows the project ----------------------------------------

// Membership is stored by name, so a fact that is deleted, or renamed, stops
// being a member. Pruning is what keeps a group from naming something that is
// no longer there.
TEST(PresentationGroups, PruningDropsMembersThatNoLongerExist) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(
      PresentationGroups::create(model, "base", {"at-base"}, {"move", "search"}));

  model.actions.erase(model.actions.begin());  // delete "move"
  EXPECT_EQ(PresentationGroups::prune(model), 1U);
  ASSERT_EQ(model.presentationGroups.size(), 1U);
  EXPECT_EQ(model.presentationGroups[0].actionNames.size(), 1U);
  EXPECT_EQ(model.presentationGroups[0].actionNames[0], "search");
}

TEST(PresentationGroups, PruningRemovesAGroupLeftHoldingNothing) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {}));
  model.predicates.erase(model.predicates.begin());  // delete "at-base"
  EXPECT_EQ(PresentationGroups::prune(model), 1U);
  EXPECT_TRUE(model.presentationGroups.empty());
}

TEST(PresentationGroups, PruningLeavesAHealthyProjectAlone) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {"move"}));
  EXPECT_EQ(PresentationGroups::prune(model), 0U);
  EXPECT_EQ(model.presentationGroups.size(), 1U);
}

// ---- The picture the canvas draws -------------------------------------------

TEST(PresentationGroupsLayout, DrawsEverythingWhenThereAreNoGroups) {
  const ProjectModel model = sampleModel();
  const CanvasLayout layout = PresentationGroups::computeLayout(model);

  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Fact), 2U);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Action), 2U);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::CollapsedGroup), 0U);
  EXPECT_TRUE(layout.boxes.empty());
  // Two requirements, one fact made true and one made false.
  EXPECT_EQ(layout.links.size(), 4U);
}

TEST(PresentationGroupsLayout, AnOpenGroupIsABoxAroundMembersStillDrawn) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {"move"}));

  const CanvasLayout layout = PresentationGroups::computeLayout(model);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Fact), 2U);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Action), 2U);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::CollapsedGroup), 0U);
  EXPECT_EQ(layout.links.size(), 4U);

  ASSERT_EQ(layout.boxes.size(), 1U);
  EXPECT_EQ(layout.boxes[0].groupIndex, 0U);
  EXPECT_EQ(layout.boxes[0].memberNodes.size(), 2U);
}

TEST(PresentationGroupsLayout, CollapsingHidesMembersBehindOneBox) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "base", {"at-base"}, {"move"}));
  ASSERT_TRUE(PresentationGroups::setCollapsed(model, 0, true));

  const CanvasLayout layout = PresentationGroups::computeLayout(model);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::CollapsedGroup), 1U);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Fact), 1U);   // at-sector
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Action), 1U);  // search
  EXPECT_TRUE(layout.boxes.empty());

  EXPECT_TRUE(layout.factIsHidden(0));       // at-base
  EXPECT_FALSE(layout.factIsHidden(1));      // at-sector
  EXPECT_TRUE(layout.actionIsHidden(0));     // move
  EXPECT_FALSE(layout.actionIsHidden(1));    // search

  // Two of the four lines had both ends inside the group and are gone: the one
  // saying "move" needs (at-base), and the one saying "move" makes (at-base)
  // false. What is left is "move" makes (at-sector) true, which now leaves the
  // group's box, and "search" needs (at-sector), which the group never touched.
  ASSERT_EQ(layout.links.size(), 2U);
  const int groupNode = 0;  // collapsed groups are added first
  ASSERT_EQ(layout.nodes[0].kind, CanvasNodeKind::CollapsedGroup);
  EXPECT_EQ(layout.nodes[0].label, "base");
  size_t touchingTheGroup = 0;
  for (const CanvasLink& link : layout.links) {
    if (static_cast<int>(link.factNode) == groupNode ||
        static_cast<int>(link.actionNode) == groupNode) {
      ++touchingTheGroup;
      // A line reaching a collapsed group cannot name one of the action's
      // condition or outcome rows, because the box has a single connection
      // point instead of a row for each.
      EXPECT_EQ(link.slot, -1);
      EXPECT_EQ(link.kind, CanvasLinkKind::MakesTrue);
    }
  }
  EXPECT_EQ(touchingTheGroup, 1U);
}

// A collapsed group has one connection point, so several lines that used to
// reach different members of it become one line. Here "move" both makes
// (at-sector) true and, once (at-sector) is a requirement of "search", the two
// separate lines to the collapsed group would otherwise be drawn on top of one
// another.
TEST(PresentationGroupsLayout, LinesThatWouldOverlapAreDrawnOnce) {
  ProjectModel model = sampleModel();
  // A second action that needs the same fact twice would draw two lines to two
  // different condition rows. Collapse the fact's group and both lines end on
  // the same connection point, so only one is drawn.
  ActionDef guard;
  guard.name = "guard";
  guard.params.push_back({"v", "vehicle"});
  guard.preconditions.push_back({"at-sector", {"v"}});
  guard.preconditions.push_back({"at-sector", {"v"}});
  model.actions.push_back(guard);

  const CanvasLayout open = PresentationGroups::computeLayout(model);
  ASSERT_TRUE(PresentationGroups::create(model, "guarding", {}, {"guard"}));
  ASSERT_TRUE(PresentationGroups::setCollapsed(model, 0, true));
  const CanvasLayout collapsed = PresentationGroups::computeLayout(model);

  EXPECT_EQ(open.links.size(), collapsed.links.size() + 1U);
}

TEST(PresentationGroupsLayout, IgnoresMembersThatNameNothing) {
  ProjectModel model = sampleModel();
  PresentationGroup stale;
  stale.name = "stale";
  stale.predicateNames.push_back("at-orbit");
  stale.collapsed = true;
  model.presentationGroups.push_back(stale);

  // The layout must not fall over on a project that has not been pruned; the
  // group simply stands for nothing and nothing is hidden.
  const CanvasLayout layout = PresentationGroups::computeLayout(model);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Fact), 2U);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::Action), 2U);
  EXPECT_EQ(layout.links.size(), 4U);
}

// ---- Groups survive being saved and reopened --------------------------------

TEST(PresentationGroups, SurvivesSaveAndLoadWithItsCollapsedState) {
  ProjectModel model = sampleModel();
  ASSERT_TRUE(PresentationGroups::create(model, "at the base", {"at-base"},
                                         {"move"}));
  ASSERT_TRUE(PresentationGroups::setCollapsed(model, 0, true));

  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "ame_presentation_groups.json";
  ASSERT_TRUE(model.save(path.string()));

  ProjectModel reloaded;
  ASSERT_TRUE(reloaded.load(path.string()));
  ASSERT_EQ(reloaded.presentationGroups.size(), 1U);
  EXPECT_EQ(reloaded.presentationGroups[0].name, "at the base");
  EXPECT_TRUE(reloaded.presentationGroups[0].collapsed);
  EXPECT_EQ(reloaded.presentationGroups[0].predicateNames.size(), 1U);
  EXPECT_EQ(reloaded.presentationGroups[0].actionNames.size(), 1U);

  const CanvasLayout layout = PresentationGroups::computeLayout(reloaded);
  EXPECT_EQ(countNodes(layout, CanvasNodeKind::CollapsedGroup), 1U);

  std::filesystem::remove(path);
}
