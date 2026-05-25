#include "structural_validator.h"

#include "project_model.h"

#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

void addIssue(StructuralReport& report,
              Severity severity,
              std::string message,
              std::string predicateName = {},
              std::string actionName = {},
              std::string typeName = {}) {
  report.issues.push_back({
    severity,
    std::move(message),
    std::move(predicateName),
    std::move(actionName),
    std::move(typeName)
  });
  if (severity == Severity::Error) {
    ++report.errorCount;
  } else {
    ++report.warningCount;
  }
}

std::unordered_set<std::string> collectTypeNames(const ProjectModel& model) {
  std::unordered_set<std::string> names;
  for (const auto& type : model.types) {
    names.insert(type.name);
  }
  return names;
}

std::unordered_set<std::string> collectPredicateNames(const ProjectModel& model) {
  std::unordered_set<std::string> names;
  for (const auto& predicate : model.predicates) {
    names.insert(predicate.name);
  }
  return names;
}

void checkDuplicateNames(StructuralReport& report,
                         const std::unordered_map<std::string, size_t>& counts,
                         const char* elementName,
                         int elementKind) {
  for (const auto& entry : counts) {
    if (entry.second <= 1U) {
      continue;
    }

    const std::string shownName = entry.first.empty() ? "(empty)" : entry.first;
    if (elementKind == 0) {
      addIssue(report,
               Severity::Error,
               std::string("Duplicate ") + elementName + " name '" + shownName + "'",
               entry.first);
    } else if (elementKind == 1) {
      addIssue(report,
               Severity::Error,
               std::string("Duplicate ") + elementName + " name '" + shownName + "'",
               {},
               entry.first);
    } else {
      addIssue(report,
               Severity::Error,
               std::string("Duplicate ") + elementName + " name '" + shownName + "'",
               {},
               {},
               entry.first);
    }
  }
}

void checkActionPredicateRefs(StructuralReport& report,
                              const ActionDef& action,
                              const std::unordered_set<std::string>& predicateNames,
                              const std::vector<EffectRef>& refs,
                              const char* sectionName) {
  for (const auto& ref : refs) {
    if (predicateNames.find(ref.predicateName) == predicateNames.end()) {
      addIssue(report,
               Severity::Error,
               "Action '" + action.name + "' " + sectionName +
                   " references missing predicate '" + ref.predicateName + "'",
               ref.predicateName,
               action.name);
    }
  }
}

void collectRefs(const std::vector<EffectRef>& refs, std::set<std::string>& names) {
  for (const auto& ref : refs) {
    names.insert(ref.predicateName);
  }
}

} // namespace

StructuralReport StructuralValidator::check(const ProjectModel& model) {
  StructuralReport report;

  std::unordered_map<std::string, size_t> predicateCounts;
  std::unordered_map<std::string, size_t> actionCounts;
  std::unordered_map<std::string, size_t> typeCounts;

  for (const auto& predicate : model.predicates) {
    ++predicateCounts[predicate.name];
    if (predicate.name.empty()) {
      addIssue(report, Severity::Error, "Predicate has empty name");
    }
  }
  for (const auto& action : model.actions) {
    ++actionCounts[action.name];
    if (action.name.empty()) {
      addIssue(report, Severity::Error, "Action has empty name");
    }
  }
  for (const auto& type : model.types) {
    ++typeCounts[type.name];
    if (type.name.empty()) {
      addIssue(report, Severity::Error, "Type has empty name");
    }
  }

  checkDuplicateNames(report, predicateCounts, "predicate", 0);
  checkDuplicateNames(report, actionCounts, "action", 1);
  checkDuplicateNames(report, typeCounts, "type", 2);

  const std::unordered_set<std::string> typeNames = collectTypeNames(model);
  const std::unordered_set<std::string> predicateNames = collectPredicateNames(model);

  for (const auto& predicate : model.predicates) {
    for (const auto& param : predicate.params) {
      if (!param.type.empty() && typeNames.find(param.type) == typeNames.end()) {
        addIssue(report,
                 Severity::Error,
                 "Predicate '" + predicate.name +
                     "' parameter '" + param.name +
                     "' uses undeclared type '" + param.type + "'",
                 predicate.name);
      }
    }
  }

  for (const auto& action : model.actions) {
    for (const auto& param : action.params) {
      if (!param.type.empty() && typeNames.find(param.type) == typeNames.end()) {
        addIssue(report,
                 Severity::Error,
                 "Action '" + action.name +
                     "' parameter '" + param.name +
                     "' uses undeclared type '" + param.type + "'",
                 {},
                 action.name);
      }
    }

    checkActionPredicateRefs(report,
                             action,
                             predicateNames,
                             action.preconditions,
                             "precondition");
    checkActionPredicateRefs(report,
                             action,
                             predicateNames,
                             action.addEffects,
                             "add-effect");
    checkActionPredicateRefs(report,
                             action,
                             predicateNames,
                             action.delEffects,
                             "del-effect");
  }

  for (const auto& object : model.objects) {
    if (typeNames.find(object.type) == typeNames.end()) {
      addIssue(report,
               Severity::Error,
               "Object '" + object.name + "' uses undeclared type '" +
                   object.type + "'",
               {},
               {},
               object.type);
    }
  }

  std::set<std::string> actionReferencedPredicates;
  std::set<std::string> effectPredicates;
  for (const auto& action : model.actions) {
    collectRefs(action.preconditions, actionReferencedPredicates);
    collectRefs(action.addEffects, actionReferencedPredicates);
    collectRefs(action.delEffects, actionReferencedPredicates);
    collectRefs(action.addEffects, effectPredicates);
    collectRefs(action.delEffects, effectPredicates);

    if (action.params.empty()) {
      addIssue(report,
               Severity::Warning,
               "Action '" + action.name + "' has zero parameters",
               {},
               action.name);
    }
    if (action.addEffects.empty() && action.delEffects.empty()) {
      addIssue(report,
               Severity::Warning,
               "Action '" + action.name + "' has zero effects",
               {},
               action.name);
    }
  }

  std::set<std::string> goalPredicates;
  for (const auto& scenario : model.scenarios) {
    for (const auto& goal : scenario.goals) {
      goalPredicates.insert(goal.predicateName);
    }
  }

  for (const auto& predicate : model.predicates) {
    if (predicate.params.empty() &&
        actionReferencedPredicates.find(predicate.name) == actionReferencedPredicates.end()) {
      addIssue(report,
               Severity::Warning,
               "Predicate '" + predicate.name +
                   "' has zero parameters and is not referenced by any action",
               predicate.name);
    }
    if (goalPredicates.find(predicate.name) != goalPredicates.end() &&
        effectPredicates.find(predicate.name) == effectPredicates.end()) {
      addIssue(report,
               Severity::Warning,
               "Predicate '" + predicate.name +
                   "' is a goal target but is never produced by an effect",
               predicate.name);
    }
  }

  std::unordered_map<std::string, std::vector<std::string>> children;
  std::unordered_map<std::string, std::string> parents;
  for (const auto& type : model.types) {
    if (!type.parent.empty()) {
      children[type.parent].push_back(type.name);
    }
    parents[type.name] = type.parent;
  }

  std::unordered_set<std::string> typesWithObjectsInSubtree;
  for (const auto& object : model.objects) {
    std::string current = object.type;
    std::unordered_set<std::string> visited;
    while (!current.empty() && visited.insert(current).second) {
      typesWithObjectsInSubtree.insert(current);
      const auto parentIt = parents.find(current);
      if (parentIt == parents.end()) {
        break;
      }
      current = parentIt->second;
    }
  }

  for (const auto& type : model.types) {
    const auto childIt = children.find(type.name);
    if (childIt == children.end() || childIt->second.empty()) {
      continue;
    }
    if (typesWithObjectsInSubtree.find(type.name) == typesWithObjectsInSubtree.end()) {
      addIssue(report,
               Severity::Warning,
               "Type '" + type.name +
                   "' has children but no objects in its subtree",
               {},
               {},
               type.name);
    }
  }

  return report;
}
