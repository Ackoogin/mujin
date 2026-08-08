#include "presentation_groups.h"

#include "relation_index.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace {

bool containsName(const std::vector<std::string>& names,
                  const std::string& name) {
  return std::find(names.begin(), names.end(), name) != names.end();
}

bool factExists(const ProjectModel& model, const std::string& name) {
  return std::any_of(model.predicates.begin(), model.predicates.end(),
                     [&name](const PredicateDef& fact) {
                       return fact.name == name;
                     });
}

bool actionExists(const ProjectModel& model, const std::string& name) {
  return std::any_of(model.actions.begin(), model.actions.end(),
                     [&name](const ActionDef& action) {
                       return action.name == name;
                     });
}

bool groupIndexValid(const ProjectModel& model, size_t groupIndex) {
  return groupIndex < model.presentationGroups.size();
}

/// The same sentence for a fact and for an action, because the rule is the
/// same one and the user should recognise it the second time they meet it.
std::string alreadyInAGroup(const std::string& name,
                            const std::string& groupName) {
  return "'" + name + "' is already in the group '" + groupName +
         "'. Something can only be in one group, because a collapsed group "
         "stands for exactly the things inside it";
}

/// A count with the right singular or plural word, such as "3 facts".
std::string counted(size_t count, const char* singular, const char* plural) {
  return std::to_string(count) + " " + (count == 1 ? singular : plural);
}

}  // namespace

int CanvasLayout::nodeForFact(size_t factIndex) const {
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].kind == CanvasNodeKind::Fact && nodes[i].index == factIndex) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int CanvasLayout::nodeForAction(size_t actionIndex) const {
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].kind == CanvasNodeKind::Action &&
        nodes[i].index == actionIndex) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string PresentationGroups::whyGroupCannotBeMade(
    const ProjectModel& model,
    const std::string& name,
    const std::vector<std::string>& factNames,
    const std::vector<std::string>& actionNames,
    int ignoreGroupIndex) {
  if (name.empty()) {
    return "a group needs a name";
  }
  for (size_t i = 0; i < model.presentationGroups.size(); ++i) {
    if (static_cast<int>(i) == ignoreGroupIndex) {
      continue;
    }
    if (model.presentationGroups[i].name == name) {
      return "another group is already called '" + name + "'";
    }
  }
  return whyContentsCannotBeGrouped(model, factNames, actionNames,
                                    ignoreGroupIndex);
}

std::string PresentationGroups::whyContentsCannotBeGrouped(
    const ProjectModel& model,
    const std::vector<std::string>& factNames,
    const std::vector<std::string>& actionNames,
    int ignoreGroupIndex) {
  if (factNames.empty() && actionNames.empty()) {
    return "a group has to hold at least one fact or action";
  }

  for (const std::string& factName : factNames) {
    if (!factExists(model, factName)) {
      return "there is no fact called '" + factName + "' in this project";
    }
    const int holder = groupHoldingFact(model, factName);
    if (holder >= 0 && holder != ignoreGroupIndex) {
      return alreadyInAGroup(
          factName, model.presentationGroups[static_cast<size_t>(holder)].name);
    }
  }
  for (const std::string& actionName : actionNames) {
    if (!actionExists(model, actionName)) {
      return "there is no action called '" + actionName + "' in this project";
    }
    const int holder = groupHoldingAction(model, actionName);
    if (holder >= 0 && holder != ignoreGroupIndex) {
      return alreadyInAGroup(
          actionName,
          model.presentationGroups[static_cast<size_t>(holder)].name);
    }
  }
  return "";
}

bool PresentationGroups::create(ProjectModel& model,
                                const std::string& name,
                                std::vector<std::string> factNames,
                                std::vector<std::string> actionNames) {
  if (!whyGroupCannotBeMade(model, name, factNames, actionNames).empty()) {
    return false;
  }
  PresentationGroup group;
  group.name = name;
  group.predicateNames = std::move(factNames);
  group.actionNames = std::move(actionNames);
  group.collapsed = false;
  model.presentationGroups.push_back(std::move(group));
  return true;
}

std::string PresentationGroups::whyGroupCannotBeRenamed(
    const ProjectModel& model,
    size_t groupIndex,
    const std::string& newName) {
  if (!groupIndexValid(model, groupIndex)) {
    return "that group no longer exists";
  }
  if (newName.empty()) {
    return "a group needs a name";
  }
  if (newName == model.presentationGroups[groupIndex].name) {
    return "that is the name it already has";
  }
  for (size_t i = 0; i < model.presentationGroups.size(); ++i) {
    if (i != groupIndex && model.presentationGroups[i].name == newName) {
      return "another group is already called '" + newName + "'";
    }
  }
  return "";
}

bool PresentationGroups::rename(ProjectModel& model,
                                size_t groupIndex,
                                const std::string& newName) {
  if (!whyGroupCannotBeRenamed(model, groupIndex, newName).empty()) {
    return false;
  }
  model.presentationGroups[groupIndex].name = newName;
  return true;
}

bool PresentationGroups::remove(ProjectModel& model, size_t groupIndex) {
  if (!groupIndexValid(model, groupIndex)) {
    return false;
  }
  model.presentationGroups.erase(model.presentationGroups.begin() +
                                 static_cast<std::ptrdiff_t>(groupIndex));
  return true;
}

bool PresentationGroups::setCollapsed(ProjectModel& model,
                                      size_t groupIndex,
                                      bool collapsed) {
  if (!groupIndexValid(model, groupIndex)) {
    return false;
  }
  model.presentationGroups[groupIndex].collapsed = collapsed;
  return true;
}

bool PresentationGroups::addMember(ProjectModel& model,
                                   size_t groupIndex,
                                   const std::string& name,
                                   bool isAction) {
  if (!groupIndexValid(model, groupIndex)) {
    return false;
  }
  if (isAction ? !actionExists(model, name) : !factExists(model, name)) {
    return false;
  }
  const int holder = isAction ? groupHoldingAction(model, name)
                              : groupHoldingFact(model, name);
  if (holder >= 0) {
    return false;
  }
  PresentationGroup& group = model.presentationGroups[groupIndex];
  if (isAction) {
    group.actionNames.push_back(name);
  } else {
    group.predicateNames.push_back(name);
  }
  return true;
}

bool PresentationGroups::removeMember(ProjectModel& model,
                                      size_t groupIndex,
                                      const std::string& name,
                                      bool isAction) {
  if (!groupIndexValid(model, groupIndex)) {
    return false;
  }
  PresentationGroup& group = model.presentationGroups[groupIndex];
  std::vector<std::string>& names =
      isAction ? group.actionNames : group.predicateNames;
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    return false;
  }
  names.erase(it);
  if (group.predicateNames.empty() && group.actionNames.empty()) {
    remove(model, groupIndex);
  }
  return true;
}

int PresentationGroups::groupHoldingFact(const ProjectModel& model,
                                         const std::string& name) {
  for (size_t i = 0; i < model.presentationGroups.size(); ++i) {
    if (containsName(model.presentationGroups[i].predicateNames, name)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int PresentationGroups::groupHoldingAction(const ProjectModel& model,
                                           const std::string& name) {
  for (size_t i = 0; i < model.presentationGroups.size(); ++i) {
    if (containsName(model.presentationGroups[i].actionNames, name)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

size_t PresentationGroups::prune(ProjectModel& model) {
  size_t dropped = 0;
  for (PresentationGroup& group : model.presentationGroups) {
    const auto factEnd =
        std::remove_if(group.predicateNames.begin(), group.predicateNames.end(),
                       [&model](const std::string& name) {
                         return !factExists(model, name);
                       });
    dropped += static_cast<size_t>(
        std::distance(factEnd, group.predicateNames.end()));
    group.predicateNames.erase(factEnd, group.predicateNames.end());

    const auto actionEnd =
        std::remove_if(group.actionNames.begin(), group.actionNames.end(),
                       [&model](const std::string& name) {
                         return !actionExists(model, name);
                       });
    dropped +=
        static_cast<size_t>(std::distance(actionEnd, group.actionNames.end()));
    group.actionNames.erase(actionEnd, group.actionNames.end());
  }
  model.presentationGroups.erase(
      std::remove_if(model.presentationGroups.begin(),
                     model.presentationGroups.end(),
                     [](const PresentationGroup& group) {
                       return group.predicateNames.empty() &&
                              group.actionNames.empty();
                     }),
      model.presentationGroups.end());
  return dropped;
}

std::string PresentationGroups::describeContents(
    const PresentationGroup& group) {
  const size_t facts = group.predicateNames.size();
  const size_t actions = group.actionNames.size();
  if (facts > 0 && actions > 0) {
    return counted(facts, "fact", "facts") + " and " +
           counted(actions, "action", "actions");
  }
  if (facts > 0) {
    return counted(facts, "fact", "facts");
  }
  if (actions > 0) {
    return counted(actions, "action", "actions");
  }
  return "nothing";
}

CanvasLayout PresentationGroups::computeLayout(const ProjectModel& model) {
  CanvasLayout layout;

  // Which collapsed group, if any, is standing in for each fact and action.
  // Anything in an open group is still drawn as itself, inside the group's box.
  std::vector<int> collapsedGroupOfFact(model.predicates.size(), -1);
  std::vector<int> collapsedGroupOfAction(model.actions.size(), -1);
  const RelationIndex relations(model);

  for (size_t g = 0; g < model.presentationGroups.size(); ++g) {
    const PresentationGroup& group = model.presentationGroups[g];
    if (!group.collapsed) {
      continue;
    }
    for (const std::string& name : group.predicateNames) {
      const int index = relations.predicateIndex(name);
      if (index >= 0) {
        collapsedGroupOfFact[static_cast<size_t>(index)] = static_cast<int>(g);
      }
    }
    for (const std::string& name : group.actionNames) {
      const int index = relations.actionIndex(name);
      if (index >= 0) {
        collapsedGroupOfAction[static_cast<size_t>(index)] = static_cast<int>(g);
      }
    }
  }

  // One box per collapsed group, added first so that the group's box exists
  // before anything tries to draw a line to it.
  std::vector<int> nodeOfCollapsedGroup(model.presentationGroups.size(), -1);
  for (size_t g = 0; g < model.presentationGroups.size(); ++g) {
    if (!model.presentationGroups[g].collapsed) {
      continue;
    }
    nodeOfCollapsedGroup[g] = static_cast<int>(layout.nodes.size());
    layout.nodes.push_back(
        {CanvasNodeKind::CollapsedGroup, g, model.presentationGroups[g].name});
  }

  for (size_t f = 0; f < model.predicates.size(); ++f) {
    if (collapsedGroupOfFact[f] < 0) {
      layout.nodes.push_back(
          {CanvasNodeKind::Fact, f, model.predicates[f].name});
    }
  }
  for (size_t a = 0; a < model.actions.size(); ++a) {
    if (collapsedGroupOfAction[a] < 0) {
      layout.nodes.push_back(
          {CanvasNodeKind::Action, a, model.actions[a].name});
    }
  }

  // An open group is a labelled box drawn behind the members still on screen.
  for (size_t g = 0; g < model.presentationGroups.size(); ++g) {
    const PresentationGroup& group = model.presentationGroups[g];
    if (group.collapsed) {
      continue;
    }
    CanvasGroupBox box;
    box.groupIndex = g;
    for (const std::string& name : group.predicateNames) {
      const int index = relations.predicateIndex(name);
      if (index < 0) {
        continue;
      }
      const int node = layout.nodeForFact(static_cast<size_t>(index));
      if (node >= 0) {
        box.memberNodes.push_back(static_cast<size_t>(node));
      }
    }
    for (const std::string& name : group.actionNames) {
      const int index = relations.actionIndex(name);
      if (index < 0) {
        continue;
      }
      const int node = layout.nodeForAction(static_cast<size_t>(index));
      if (node >= 0) {
        box.memberNodes.push_back(static_cast<size_t>(node));
      }
    }
    if (!box.memberNodes.empty()) {
      layout.boxes.push_back(std::move(box));
    }
  }

  // The lines. Each one runs between a fact and an action, or between whatever
  // is standing in for them. A line whose two ends land on the same box says
  // nothing and is dropped; two lines that end up saying the same thing are
  // drawn once.
  const auto endpointForFact = [&](size_t factIndex) {
    const int group = collapsedGroupOfFact[factIndex];
    return group >= 0 ? nodeOfCollapsedGroup[static_cast<size_t>(group)]
                      : layout.nodeForFact(factIndex);
  };
  const auto endpointForAction = [&](size_t actionIndex) {
    const int group = collapsedGroupOfAction[actionIndex];
    return group >= 0 ? nodeOfCollapsedGroup[static_cast<size_t>(group)]
                      : layout.nodeForAction(actionIndex);
  };

  const auto addLink = [&](size_t factIndex, size_t actionIndex,
                           CanvasLinkKind kind, int slot) {
    const int factNode = endpointForFact(factIndex);
    const int actionNode = endpointForAction(actionIndex);
    if (factNode < 0 || actionNode < 0 || factNode == actionNode) {
      return;
    }
    CanvasLink link;
    link.factNode = static_cast<size_t>(factNode);
    link.actionNode = static_cast<size_t>(actionNode);
    link.kind = kind;
    // A collapsed group has one connection point, so a line reaching it cannot
    // name a condition or outcome row. Dropping the slot is also what makes
    // several lines from the same group collapse into one.
    link.slot = collapsedGroupOfAction[actionIndex] >= 0 ? -1 : slot;
    const bool alreadyDrawn =
        std::any_of(layout.links.begin(), layout.links.end(),
                    [&link](const CanvasLink& existing) {
                      return existing.factNode == link.factNode &&
                             existing.actionNode == link.actionNode &&
                             existing.kind == link.kind &&
                             existing.slot == link.slot;
                    });
    if (!alreadyDrawn) {
      layout.links.push_back(link);
    }
  };

  for (size_t f = 0; f < model.predicates.size(); ++f) {
    const PredicateRelations& factRelations = relations.predicate(f);
    for (const auto& relation : factRelations.requiredBy) {
      addLink(f, relation.actionIndex, CanvasLinkKind::Requires,
              static_cast<int>(relation.referenceIndex));
    }
    for (const auto& relation : factRelations.requiredFalseBy) {
      addLink(f, relation.actionIndex, CanvasLinkKind::RequiresFalse,
              static_cast<int>(relation.referenceIndex));
    }
    for (const auto& relation : factRelations.acceptedAsAlternativeBy) {
      addLink(f, relation.actionIndex, CanvasLinkKind::AcceptsAlternative,
              static_cast<int>(relation.referenceIndex));
    }
    for (const auto& relation : factRelations.madeTrueBy) {
      addLink(f, relation.actionIndex, CanvasLinkKind::MakesTrue,
              static_cast<int>(relation.referenceIndex));
    }
    for (const auto& relation : factRelations.madeFalseBy) {
      // The canvas gives an action one row per outcome, the facts it makes
      // true first, so an outcome that makes a fact false sits after them.
      const size_t slot = model.actions[relation.actionIndex].addEffects.size() +
                          relation.referenceIndex;
      addLink(f, relation.actionIndex, CanvasLinkKind::MakesFalse,
              static_cast<int>(slot));
    }
  }

  return layout;
}
