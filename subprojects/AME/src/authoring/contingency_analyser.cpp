#include "contingency_analyser.h"

#include "pddl_validator.h"
#include "project_model.h"

#include <ame/planner.h>
#include <ame/world_model.h>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

bool containsName(const std::vector<std::string>& names,
                  const std::string& name) {
  return std::find(names.begin(), names.end(), name) != names.end();
}

std::string formatFluent(const std::string& predicateName,
                         const std::vector<std::string>& objectNames) {
  std::ostringstream out;
  out << "(" << predicateName;
  for (const auto& objectName : objectNames) {
    out << " " << objectName;
  }
  out << ")";
  return out.str();
}

std::string formatFact(const FactRef& fact) {
  return formatFluent(fact.predicateName, fact.objectNames);
}

std::vector<std::string> validationErrorMessages(const ValidationReport& report) {
  std::vector<std::string> messages;
  for (const auto& error : report.errors) {
    messages.push_back(error.message);
  }
  return messages;
}

std::string firstValidationError(const ValidationReport& report) {
  const std::vector<std::string> messages = validationErrorMessages(report);
  if (messages.empty()) {
    return "validation failed";
  }
  return messages.front();
}

const ScenarioDef* findScenario(const ProjectModel& model,
                                const std::string& scenarioName) {
  const auto it = std::find_if(model.scenarios.begin(), model.scenarios.end(),
                               [&scenarioName](const ScenarioDef& scenario) {
                                 return scenario.name == scenarioName;
                               });
  if (it == model.scenarios.end()) {
    return nullptr;
  }
  return &(*it);
}

ScenarioDef* findScenario(ProjectModel& model, const std::string& scenarioName) {
  const auto it = std::find_if(model.scenarios.begin(), model.scenarios.end(),
                               [&scenarioName](const ScenarioDef& scenario) {
                                 return scenario.name == scenarioName;
                               });
  if (it == model.scenarios.end()) {
    return nullptr;
  }
  return &(*it);
}

std::vector<std::string> identifyContextPredicates(const ProjectModel& model) {
  std::set<std::string> effectPredicates;
  std::set<std::string> preconditionPredicates;

  for (const auto& action : model.actions) {
    for (const auto& effect : action.addEffects) {
      effectPredicates.insert(effect.predicateName);
    }
    for (const auto& effect : action.delEffects) {
      effectPredicates.insert(effect.predicateName);
    }
    for (const auto& precondition : action.preconditions) {
      preconditionPredicates.insert(precondition.predicateName);
    }
  }

  std::vector<std::string> contextPredicates;
  for (const auto& predicate : model.predicates) {
    if (preconditionPredicates.find(predicate.name) != preconditionPredicates.end() &&
        effectPredicates.find(predicate.name) == effectPredicates.end() &&
        !containsName(contextPredicates, predicate.name)) {
      contextPredicates.push_back(predicate.name);
    }
  }

  return contextPredicates;
}

std::vector<const ObjectDef*> matchingObjects(const ProjectModel& model,
                                              const Parameter& parameter) {
  std::vector<const ObjectDef*> objects;
  for (const auto& object : model.objects) {
    if (parameter.type.empty() || parameter.type == "object" ||
        object.type == parameter.type) {
      objects.push_back(&object);
    }
  }
  return objects;
}

void enumeratePredicateFluents(const ProjectModel& model,
                               const PredicateDef& predicate,
                               size_t paramIdx,
                               std::vector<std::string>& args,
                               std::vector<std::string>& fluents) {
  if (paramIdx == predicate.params.size()) {
    fluents.push_back(formatFluent(predicate.name, args));
    return;
  }

  const std::vector<const ObjectDef*> objects =
      matchingObjects(model, predicate.params[paramIdx]);
  for (const ObjectDef* object : objects) {
    args.push_back(object->name);
    enumeratePredicateFluents(model, predicate, paramIdx + 1U, args, fluents);
    args.pop_back();
  }
}

std::vector<std::string> enumerateContextFluents(
    const ProjectModel& model,
    const std::vector<std::string>& contextPredicates) {
  std::vector<std::string> fluents;
  for (const auto& predicate : model.predicates) {
    if (!containsName(contextPredicates, predicate.name)) {
      continue;
    }
    if (predicate.params.empty()) {
      continue;
    }

    std::vector<std::string> args;
    enumeratePredicateFluents(model, predicate, 0U, args, fluents);
  }
  return fluents;
}

FactRef parseGroundFluent(const std::string& fluent) {
  FactRef fact;
  if (fluent.size() < 2U || fluent.front() != '(' || fluent.back() != ')') {
    return fact;
  }

  std::istringstream input(fluent.substr(1U, fluent.size() - 2U));
  input >> fact.predicateName;
  std::string objectName;
  while (input >> objectName) {
    fact.objectNames.push_back(objectName);
  }
  return fact;
}

void addUniqueFact(std::vector<FactRef>& facts, FactRef fact) {
  const std::string key = formatFact(fact);
  const bool exists =
      std::any_of(facts.begin(), facts.end(), [&key](const FactRef& existing) {
        return formatFact(existing) == key;
      });
  if (!exists) {
    facts.push_back(std::move(fact));
  }
}

} // namespace

ContingencyReport ContingencyAnalyser::analyse(const ProjectModel& model,
                                               const std::string& scenarioName,
                                               size_t maxFluents) {
  ContingencyReport report;

  if (findScenario(model, scenarioName) == nullptr) {
    report.error = "scenario not found: " + scenarioName;
    return report;
  }

  report.contextPredicates = identifyContextPredicates(model);
  report.contextFluents =
      enumerateContextFluents(model, report.contextPredicates);

  if (report.contextFluents.size() > maxFluents) {
    report.error = "too many context fluents (" +
                   std::to_string(report.contextFluents.size()) +
                   "), max is " + std::to_string(maxFluents);
    return report;
  }
  if (report.contextFluents.size() >= 64U) {
    report.error = "too many context fluents (" +
                   std::to_string(report.contextFluents.size()) +
                   "), max is 63";
    return report;
  }

  const size_t totalAssignments = size_t{1} << report.contextFluents.size();
  report.results.reserve(totalAssignments);

  for (size_t mask = 0; mask < totalAssignments; ++mask) {
    ContingencyContext context;
    ProjectModel modifiedModel = model;
    ScenarioDef* scenario = findScenario(modifiedModel, scenarioName);
    if (scenario == nullptr) {
      context.errorMessage = "scenario not found: " + scenarioName;
      ++report.errorCount;
      report.results.push_back(std::move(context));
      continue;
    }

    for (size_t i = 0; i < report.contextFluents.size(); ++i) {
      if ((mask & (size_t{1} << i)) == 0U) {
        continue;
      }
      const std::string& fluent = report.contextFluents[i];
      context.trueFluents.push_back(fluent);
      addUniqueFact(scenario->initialState, parseGroundFluent(fluent));
    }

    ame::WorldModel wm;
    const ValidationReport validation =
        PddlValidator::validateAndBuildWorldModel(modifiedModel, scenarioName, wm);
    if (!validation.ok) {
      context.errorMessage = "parse failed: " + firstValidationError(validation);
      ++report.errorCount;
      report.results.push_back(std::move(context));
      continue;
    }

    try {
      const ame::PlanResult result = ame::Planner{}.solve(wm);
      context.planFound = result.success;
      context.planSteps = result.steps.size();
      if (result.success) {
        ++report.feasibleCount;
      } else {
        ++report.infeasibleCount;
      }
    } catch (const std::exception& ex) {
      context.errorMessage = std::string("planner threw: ") + ex.what();
      ++report.errorCount;
    } catch (...) {
      context.errorMessage = "planner threw: unknown exception";
      ++report.errorCount;
    }

    report.results.push_back(std::move(context));
  }

  report.ok = true;
  return report;
}
