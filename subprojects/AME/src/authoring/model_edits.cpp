#include "model_edits.h"

#include <algorithm>
#include <string>
#include <vector>

namespace {

bool typeExists(const ProjectModel& model, const std::string& name) {
  return std::any_of(model.types.begin(), model.types.end(),
                     [&name](const TypeDef& type) { return type.name == name; });
}

bool nameTaken(const ProjectModel& model,
               const std::string& name,
               bool forAction) {
  if (forAction) {
    return std::any_of(model.actions.begin(), model.actions.end(),
                       [&name](const ActionDef& action) {
                         return action.name == name;
                       });
  }
  return std::any_of(model.predicates.begin(), model.predicates.end(),
                     [&name](const PredicateDef& predicate) {
                       return predicate.name == name;
                     });
}

void renameTypeInParameter(Parameter& parameter,
                           const std::string& old_name,
                           const std::string& new_name) {
  if (parameter.type == old_name) {
    parameter.type = new_name;
  }
  for (std::string& type : parameter.eitherTypes) {
    if (type == old_name) {
      type = new_name;
    }
  }
}

void renameTypeInCondition(ConditionExpression& condition,
                           const std::string& old_name,
                           const std::string& new_name) {
  for (Parameter& variable : condition.variables) {
    renameTypeInParameter(variable, old_name, new_name);
  }
  for (ConditionExpression& child : condition.children) {
    renameTypeInCondition(child, old_name, new_name);
  }
}

}  // namespace

std::string ElementClipboard::description() const {
  switch (kind) {
  case Kind::Fact:
    return "Paste '" + fact.name + "'";
  case Kind::Action:
    return "Paste '" + action.name + "'";
  case Kind::Nothing:
    break;
  }
  return "Nothing to paste";
}

std::string ModelEdits::whyTypeCannotBeRenamed(const ProjectModel& model,
                                               const std::string& oldName,
                                               const std::string& newName) {
  if (oldName.empty() || !typeExists(model, oldName)) {
    return "there is no type called '" + oldName + "'";
  }
  if (newName.empty()) {
    return "a type needs a name";
  }
  if (newName == oldName) {
    return "that is the name it already has";
  }
  if (typeExists(model, newName)) {
    return "another type is already called '" + newName + "'";
  }
  return "";
}

bool ModelEdits::renameType(ProjectModel& model,
                            const std::string& oldName,
                            const std::string& newName) {
  if (!whyTypeCannotBeRenamed(model, oldName, newName).empty()) {
    return false;
  }

  for (TypeDef& type : model.types) {
    if (type.name == oldName) {
      type.name = newName;
    }
    // A type whose parent was renamed keeps the same parent under its new name.
    if (type.parent == oldName) {
      type.parent = newName;
    }
  }
  for (PredicateDef& predicate : model.predicates) {
    for (Parameter& parameter : predicate.params) {
      renameTypeInParameter(parameter, oldName, newName);
    }
  }
  for (ActionDef& action : model.actions) {
    for (Parameter& parameter : action.params) {
      renameTypeInParameter(parameter, oldName, newName);
    }
    if (action.hasConditionExpression) {
      renameTypeInCondition(action.conditionExpression, oldName, newName);
    }
  }
  for (ObjectDef& object : model.objects) {
    if (object.type == oldName) {
      object.type = newName;
    }
  }
  for (ObjectDef& constant : model.constants) {
    if (constant.type == oldName) {
      constant.type = newName;
    }
  }
  for (StateGroupDef& group : model.stateGroups) {
    if (group.type == oldName) {
      group.type = newName;
    }
  }
  return true;
}

bool ModelEdits::moveActionParameter(ProjectModel& model,
                                     size_t actionIndex,
                                     size_t parameterIndex,
                                     bool later) {
  if (actionIndex >= model.actions.size()) {
    return false;
  }
  std::vector<Parameter>& parameters = model.actions[actionIndex].params;
  if (parameterIndex >= parameters.size()) {
    return false;
  }
  const size_t target = later ? parameterIndex + 1U : parameterIndex - 1U;
  if (later ? target >= parameters.size() : parameterIndex == 0U) {
    return false;
  }
  std::swap(parameters[parameterIndex], parameters[target]);
  return true;
}

std::string ModelEdits::unusedName(const ProjectModel& model,
                                   const std::string& wanted,
                                   bool forAction) {
  if (!nameTaken(model, wanted, forAction)) {
    return wanted;
  }
  const std::string base = wanted + " copy";
  if (!nameTaken(model, base, forAction)) {
    return base;
  }
  for (int suffix = 2; suffix < 1000; ++suffix) {
    const std::string candidate = base + " " + std::to_string(suffix);
    if (!nameTaken(model, candidate, forAction)) {
      return candidate;
    }
  }
  return base;
}

bool ModelEdits::copyFact(const ProjectModel& model,
                          size_t factIndex,
                          ElementClipboard& clipboard) {
  if (factIndex >= model.predicates.size()) {
    return false;
  }
  clipboard.kind = ElementClipboard::Kind::Fact;
  clipboard.fact = model.predicates[factIndex];
  return true;
}

bool ModelEdits::copyAction(const ProjectModel& model,
                            size_t actionIndex,
                            ElementClipboard& clipboard) {
  if (actionIndex >= model.actions.size()) {
    return false;
  }
  clipboard.kind = ElementClipboard::Kind::Action;
  clipboard.action = model.actions[actionIndex];
  return true;
}

std::string ModelEdits::paste(ProjectModel& model,
                              const ElementClipboard& clipboard) {
  // A pasted element lands at the origin rather than on top of the one it came
  // from, because two nodes in the same place look like one.
  switch (clipboard.kind) {
  case ElementClipboard::Kind::Fact: {
    PredicateDef pasted = clipboard.fact;
    pasted.name = unusedName(model, pasted.name, false);
    pasted.posX = 0.0F;
    pasted.posY = 0.0F;
    model.predicates.push_back(pasted);
    return pasted.name;
  }
  case ElementClipboard::Kind::Action: {
    ActionDef pasted = clipboard.action;
    pasted.name = unusedName(model, pasted.name, true);
    pasted.posX = 0.0F;
    pasted.posY = 0.0F;
    model.actions.push_back(pasted);
    return pasted.name;
  }
  case ElementClipboard::Kind::Nothing:
    break;
  }
  return "";
}
