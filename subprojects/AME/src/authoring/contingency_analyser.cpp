#include "contingency_analyser.h"

#include "authoring_utils.h"
#include "pddl_generator.h"
#include "pddl_validator.h"
#include "project_model.h"

#include <ame/contingency_search.h>
#include <ame/pddl_parser.h>
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
  const std::string key = authoring::formatFactRef(fact);
  const bool exists =
      std::any_of(facts.begin(), facts.end(), [&key](const FactRef& existing) {
        return authoring::formatFactRef(existing) == key;
      });
  if (!exists) {
    facts.push_back(std::move(fact));
  }
}

} // namespace

std::string ContingencyReport::coverageSentence() const {
  if (!ok) {
    return error;
  }
  std::string sentence =
      "Checked all " + std::to_string(combinationsChecked) +
      " ways the context could be: " + std::to_string(plannerCalls) +
      " were planned for, and " + std::to_string(answeredByReasoning) +
      " followed from those without planning.";
  if (pruningRefused) {
    sentence +=
        " Nothing could be carried between them, because this domain has "
        "conditions about facts being false, so every one was planned for.";
  }
  sentence += declaredByUser
                  ? " The facts varied and the safe state are the ones this "
                    "scenario declares."
                  : " Nothing was declared, so the facts varied and the safe "
                    "state were both worked out from the model.";
  return sentence;
}

ContingencyReport ContingencyAnalyser::analyse(const ProjectModel& model,
                                               const std::string& scenarioName,
                                               size_t maxFluents) {
  ContingencyReport report;

  const ScenarioDef* scenario = findScenario(model, scenarioName);
  if (scenario == nullptr) {
    report.error = "scenario not found: " + scenarioName;
    return report;
  }

  // The same search the contingency verifier runs from the command line, and
  // the same one the assurance report quotes. See ame/contingency_search.h.
  const std::string domain_pddl = PddlGenerator::generateDomain(model);
  const std::string problem_pddl =
      PddlGenerator::generateProblem(model, scenarioName);

  ame::ContingencySearchOptions options;
  options.only_predicates = scenario->contingency.contingencyPredicates;
  if (!scenario->contingency.safeState.empty()) {
    std::vector<std::string> safe;
    for (const FactRef& fact : scenario->contingency.safeState) {
      safe.push_back(formatFluent(fact.predicateName, fact.objectNames));
    }
    options.goal_options.push_back(std::move(safe));
  }
  report.declaredByUser = !scenario->contingency.isEmpty();

  ame::ContingencySearchReport search;
  try {
    // Count the facts before running, so a domain far too large to enumerate
    // is refused with a number rather than by taking all afternoon.
    ame::WorldModel counting_wm;
    ame::PddlParser::parseFromString(domain_pddl, problem_pddl, counting_wm);
    size_t factCount =
        ame::ContingencySearch::identifyContextFacts(counting_wm).size();
    if (!options.only_predicates.empty()) {
      size_t declared = 0;
      for (const auto& fact :
           ame::ContingencySearch::identifyContextFacts(counting_wm)) {
        const std::string predicate =
            fact.short_name.substr(0, fact.short_name.find(' '));
        if (std::find(options.only_predicates.begin(),
                      options.only_predicates.end(),
                      predicate) != options.only_predicates.end()) {
          ++declared;
        }
      }
      factCount = declared;
    }
    if (factCount > maxFluents) {
      report.error = "there are " + std::to_string(factCount) +
                     " facts the plan cannot change, and this checks at most " +
                     std::to_string(maxFluents) +
                     ". Say which of them represent a contingency to narrow it.";
      return report;
    }

    search = ame::ContingencySearch::run(domain_pddl, problem_pddl, options);
  } catch (const std::exception& ex) {
    report.error = std::string("the model could not be read: ") + ex.what();
    return report;
  }

  for (const ame::ContextFact& fact : search.context_facts) {
    report.contextFluents.push_back(fact.fluent_name);
    const std::string predicate =
        fact.short_name.substr(0, fact.short_name.find(' '));
    if (!containsName(report.contextPredicates, predicate)) {
      report.contextPredicates.push_back(predicate);
    }
  }

  report.results.reserve(search.cases.size());
  for (const ame::ContingencyCase& item : search.cases) {
    ContingencyContext context;
    for (size_t i = 0; i < search.context_facts.size(); ++i) {
      if (((item.combination >> i) & 1u) != 0u) {
        context.trueFluents.push_back(search.context_facts[i].fluent_name);
      }
    }
    context.planFound = item.reachable();
    context.planSteps = item.plan_length;
    if (context.planFound) {
      ++report.feasibleCount;
    } else {
      ++report.infeasibleCount;
    }
    report.results.push_back(std::move(context));
  }

  report.combinationsChecked = search.cases.size();
  report.plannerCalls = search.solver_calls;
  report.answeredByReasoning =
      static_cast<size_t>(search.implied_reachable + search.implied_unreachable);
  report.pruningUsed = search.pruning_used;
  report.pruningRefused = search.pruning_unsound_for_domain;
  report.ok = true;
  return report;
}
