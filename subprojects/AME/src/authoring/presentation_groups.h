#pragma once

#include "project_model.h"

#include <cstddef>
#include <string>
#include <vector>

/// \file
/// \brief Named groups of facts and actions, and the picture the whole-domain
/// canvas draws once they are taken into account.
///
/// A presentation group is a named set of facts and actions that the canvas
/// draws as one labelled box, and which the user can collapse to a single node.
/// It is a way of talking about a domain in a review: the generated PDDL knows
/// nothing about groups, and adding, changing or removing one cannot change
/// what the planner does.
///
/// The rules and the resulting picture live here rather than in the drawing
/// code, for the same reason the fact chooser's rules do: here they are one set
/// of functions with tests around them, and the drawing code has only to draw
/// what it is given.

/// \brief What one box on the whole-domain canvas stands for.
enum class CanvasNodeKind {
  Fact,
  Action,
  /// A group that has been collapsed, standing in for everything inside it.
  CollapsedGroup,
};

/// \brief One box the whole-domain canvas should draw.
struct CanvasNode {
  CanvasNodeKind kind = CanvasNodeKind::Fact;
  /// Index into the project's facts, actions or groups, according to `kind`.
  size_t index = 0;
  /// What the box says. For a collapsed group this is the group's name.
  std::string label;
};

/// \brief What one line on the canvas means.
enum class CanvasLinkKind {
  /// The fact has to be true before the action can happen.
  Requires,
  /// The fact has to be false before the action can happen.
  RequiresFalse,
  /// The fact is one of several ways the action's condition may be met.
  AcceptsAlternative,
  /// The action makes the fact true.
  MakesTrue,
  /// The action makes the fact false.
  MakesFalse,
};

/// \brief One line the whole-domain canvas should draw.
///
/// The line always runs between a fact and an action, or between whichever
/// collapsed groups those two ended up inside. Which way round it is drawn
/// follows from `kind`: a condition runs from the fact to the action, and an
/// outcome runs from the action to the fact.
struct CanvasLink {
  /// Index into CanvasLayout::nodes for the fact end of the line.
  size_t factNode = 0;
  /// Index into CanvasLayout::nodes for the action end of the line.
  size_t actionNode = 0;
  CanvasLinkKind kind = CanvasLinkKind::Requires;
  /// Which of the action's conditions or outcomes the line comes from, so that
  /// it can be drawn against the right row of the action's box. It is -1 when
  /// the action end is a collapsed group, because a collapsed group has one
  /// connection point rather than a row for every condition and outcome.
  int slot = -1;
};

/// \brief A group that is open, drawn as a labelled box behind its members.
struct CanvasGroupBox {
  /// Index into the project's presentation groups.
  size_t groupIndex = 0;
  /// Indices into CanvasLayout::nodes for the boxes drawn inside this one.
  std::vector<size_t> memberNodes;
};

/// \brief Everything the whole-domain canvas needs to draw, groups included.
struct CanvasLayout {
  std::vector<CanvasNode> nodes;
  std::vector<CanvasLink> links;
  std::vector<CanvasGroupBox> boxes;

  /// \brief Which box a fact is drawn as, or -1 when a collapsed group hides it.
  int nodeForFact(size_t factIndex) const;
  /// \brief Which box an action is drawn as, or -1 when a collapsed group hides it.
  int nodeForAction(size_t actionIndex) const;
  /// \brief Whether a collapsed group is standing in for this fact.
  bool factIsHidden(size_t factIndex) const { return nodeForFact(factIndex) < 0; }
  /// \brief Whether a collapsed group is standing in for this action.
  bool actionIsHidden(size_t actionIndex) const {
    return nodeForAction(actionIndex) < 0;
  }
};

/// \brief Making, changing and drawing named groups of facts and actions.
class PresentationGroups {
public:
  /// \brief Why a new group with these contents would be refused, in words the
  /// screen can show, or an empty string when it would be accepted.
  ///
  /// \param ignoreGroupIndex A group to leave out of the checks, so that a
  /// group can be changed without clashing with the name it already has. Pass
  /// -1 when creating a new group.
  static std::string whyGroupCannotBeMade(
      const ProjectModel& model,
      const std::string& name,
      const std::vector<std::string>& factNames,
      const std::vector<std::string>& actionNames,
      int ignoreGroupIndex = -1);

  /// \brief Why these facts and actions could not be grouped, whatever the
  /// group were called, or an empty string when they could be.
  ///
  /// This is the half of the check a screen can make before the user has typed
  /// a name, so that the button offering to group them can say why it is not
  /// available.
  static std::string whyContentsCannotBeGrouped(
      const ProjectModel& model,
      const std::vector<std::string>& factNames,
      const std::vector<std::string>& actionNames,
      int ignoreGroupIndex = -1);

  /// \brief Add a group holding these facts and actions.
  /// \return False when whyGroupCannotBeMade would have given a reason.
  static bool create(ProjectModel& model,
                     const std::string& name,
                     std::vector<std::string> factNames,
                     std::vector<std::string> actionNames);

  /// \brief Why renaming a group would be refused, or an empty string.
  static std::string whyGroupCannotBeRenamed(const ProjectModel& model,
                                             size_t groupIndex,
                                             const std::string& newName);

  /// \brief Give a group a different name.
  static bool rename(ProjectModel& model,
                     size_t groupIndex,
                     const std::string& newName);

  /// \brief Remove a group. The facts and actions it held are untouched: only
  /// the way they are drawn changes.
  static bool remove(ProjectModel& model, size_t groupIndex);

  /// \brief Collapse a group to one box, or open it again.
  static bool setCollapsed(ProjectModel& model, size_t groupIndex, bool collapsed);

  /// \brief Put a fact or an action into a group it is not already in.
  static bool addMember(ProjectModel& model,
                        size_t groupIndex,
                        const std::string& name,
                        bool isAction);

  /// \brief Take a fact or an action out of a group. A group left holding
  /// nothing is removed, because an empty box says nothing.
  static bool removeMember(ProjectModel& model,
                           size_t groupIndex,
                           const std::string& name,
                           bool isAction);

  /// \brief Which group holds this fact, or -1 when none does.
  static int groupHoldingFact(const ProjectModel& model, const std::string& name);

  /// \brief Which group holds this action, or -1 when none does.
  static int groupHoldingAction(const ProjectModel& model, const std::string& name);

  /// \brief Drop members that no longer name anything in the project, and
  /// remove any group left holding nothing.
  ///
  /// Membership is stored by name, so a fact or action that is deleted, or
  /// renamed, stops being a member. Call this after the project changes shape.
  /// \return How many members were dropped.
  static size_t prune(ProjectModel& model);

  /// \brief What a group holds, in words, such as "3 facts and 2 actions".
  static std::string describeContents(const PresentationGroup& group);

  /// \brief Work out what the whole-domain canvas should draw.
  ///
  /// Facts and actions inside a collapsed group are left out, and one box is
  /// added for the group itself. Lines that reached a hidden member reach the
  /// group's box instead, lines that end up saying the same thing twice are
  /// drawn once, and a line with both ends inside the same collapsed group is
  /// not drawn at all, because it would start and finish on the same box.
  ///
  /// Members that name nothing in the project are ignored, so a layout can be
  /// computed from a project that has not been pruned.
  static CanvasLayout computeLayout(const ProjectModel& model);
};
