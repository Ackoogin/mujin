#include "pddl_generator.h"

#include "authoring_utils.h"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string domainName(const ProjectModel& model) {
  return authoring::slugify(model.projectName);
}

std::string parameterName(const std::string& name) {
  if (!name.empty() && name.front() == '?') {
    return name;
  }
  return "?" + name;
}

void emitParameters(std::ostringstream& out, const std::vector<Parameter>& params) {
  for (size_t i = 0; i < params.size(); ++i) {
    if (i > 0U) {
      out << " ";
    }
    out << parameterName(params[i].name);
    if (!params[i].type.empty()) {
      out << " - " << params[i].type;
    }
  }
}

void emitFact(std::ostringstream& out, const EffectRef& ref) {
  out << "(" << ref.predicateName;
  for (const auto& arg : ref.argNames) {
    out << " " << arg;
  }
  out << ")";
}

void emitFact(std::ostringstream& out, const FactRef& ref) {
  out << "(" << ref.predicateName;
  for (const auto& arg : ref.objectNames) {
    out << " " << arg;
  }
  out << ")";
}

bool isParameterRef(const std::string& arg) {
  return !arg.empty() && arg.front() == '?';
}

bool hasObjectNamed(const std::vector<ObjectDef>& objects,
                    const std::string& name) {
  return std::any_of(objects.begin(), objects.end(),
                     [&name](const ObjectDef& object) {
                       return object.name == name;
                     });
}

const PredicateDef* findPredicate(const ProjectModel& model,
                                  const std::string& name) {
  const auto it = std::find_if(model.predicates.begin(), model.predicates.end(),
                               [&name](const PredicateDef& predicate) {
                                 return predicate.name == name;
                               });
  if (it == model.predicates.end()) {
    return nullptr;
  }
  return &(*it);
}

void appendImplicitLiteralObjects(const ProjectModel& model,
                                  const std::string& predicateName,
                                  const std::vector<std::string>& args,
                                  std::vector<ObjectDef>& objects) {
  const PredicateDef* predicate = findPredicate(model, predicateName);
  if (predicate == nullptr) {
    return;
  }

  const size_t count = std::min(args.size(), predicate->params.size());
  for (size_t i = 0; i < count; ++i) {
    if (isParameterRef(args[i]) || hasObjectNamed(objects, args[i])) {
      continue;
    }
    objects.push_back({args[i], predicate->params[i].type});
  }
}

std::vector<ObjectDef> problemObjectsWithImplicitLiterals(
    const ProjectModel& model,
    const ScenarioDef& scenario) {
  std::vector<ObjectDef> objects = model.objects;

  for (const auto& action : model.actions) {
    for (const auto& ref : action.preconditions) {
      appendImplicitLiteralObjects(model, ref.predicateName, ref.argNames, objects);
    }
    for (const auto& ref : action.addEffects) {
      appendImplicitLiteralObjects(model, ref.predicateName, ref.argNames, objects);
    }
    for (const auto& ref : action.delEffects) {
      appendImplicitLiteralObjects(model, ref.predicateName, ref.argNames, objects);
    }
  }

  for (const auto& fact : scenario.initialState) {
    appendImplicitLiteralObjects(model,
                                 fact.predicateName,
                                 fact.objectNames,
                                 objects);
  }
  for (const auto& fact : scenario.goals) {
    appendImplicitLiteralObjects(model,
                                 fact.predicateName,
                                 fact.objectNames,
                                 objects);
  }

  return objects;
}

void emitFactList(std::ostringstream& out,
                  const std::vector<EffectRef>& facts,
                  const std::string& indent) {
  if (facts.empty()) {
    out << "(and)";
    return;
  }

  if (facts.size() == 1U) {
    emitFact(out, facts.front());
    return;
  }

  out << "(and\n";
  for (const auto& fact : facts) {
    out << indent;
    emitFact(out, fact);
    out << "\n";
  }
  const std::string closeIndent(indent.size() >= 2U ? indent.size() - 2U : 0U, ' ');
  out << closeIndent << ")";
}

void emitEffects(std::ostringstream& out, const ActionDef& action) {
  const size_t effectCount = action.addEffects.size() + action.delEffects.size();
  if (effectCount == 0U) {
    out << "(and)";
    return;
  }

  if (effectCount == 1U) {
    if (!action.addEffects.empty()) {
      emitFact(out, action.addEffects.front());
    } else {
      out << "(not ";
      emitFact(out, action.delEffects.front());
      out << ")";
    }
    return;
  }

  out << "(and\n";
  for (const auto& effect : action.addEffects) {
    out << "      ";
    emitFact(out, effect);
    out << "\n";
  }
  for (const auto& effect : action.delEffects) {
    out << "      (not ";
    emitFact(out, effect);
    out << ")\n";
  }
  out << "    )";
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

} // namespace

std::string PddlGenerator::generateDomain(const ProjectModel& model) {
  std::ostringstream out;
  out << "(define (domain " << domainName(model) << ")\n";
  out << "  (:requirements :strips :typing)\n\n";

  out << "  (:types\n";
  for (const auto& type : model.types) {
    out << "    " << type.name;
    if (!type.parent.empty()) {
      out << " - " << type.parent;
    }
    out << "\n";
  }
  out << "  )\n\n";

  out << "  (:predicates\n";
  for (const auto& predicate : model.predicates) {
    out << "    (" << predicate.name;
    for (const auto& param : predicate.params) {
      out << " " << parameterName(param.name);
      if (!param.type.empty()) {
        out << " - " << param.type;
      }
    }
    out << ")\n";
  }
  out << "  )\n";

  // Facts the model says must be observed rather than predicted. The plan
  // compiler reads this section and routes preconditions on these predicates
  // into a planned action's confirmed-precondition port, so leaving it out
  // would quietly let an action proceed on predicted state.
  const bool anyConfirmed =
      std::any_of(model.predicates.begin(), model.predicates.end(),
                  [](const PredicateDef& predicate) {
                    return predicate.confirmed;
                  });
  if (anyConfirmed) {
    out << "\n  (:confirmed-predicates";
    for (const auto& predicate : model.predicates) {
      if (predicate.confirmed) {
        out << " " << predicate.name;
      }
    }
    out << ")\n";
  }

  for (const auto& action : model.actions) {
    out << "\n";
    out << "  (:action " << action.name << "\n";
    out << "    :parameters (";
    emitParameters(out, action.params);
    out << ")\n";
    if (!action.preconditions.empty()) {
      out << "    :precondition ";
      emitFactList(out, action.preconditions, "      ");
      out << "\n";
    }
    out << "    :effect ";
    emitEffects(out, action);
    out << "\n";
    out << "  )\n";
  }

  out << ")\n";
  return out.str();
}

std::string PddlGenerator::generateProblem(const ProjectModel& model,
                                           const std::string& scenarioName) {
  const std::string slug = domainName(model);
  const ScenarioDef* scenario = findScenario(model, scenarioName);

  std::ostringstream out;
  out << "(define (problem " << slug << "-1)\n";
  out << "  (:domain " << slug << ")";
  if (scenario == nullptr) {
    out << "\n)\n";
    return out.str();
  }
  out << "\n\n";

  out << "  (:objects\n";
  const std::vector<ObjectDef> objects =
      problemObjectsWithImplicitLiterals(model, *scenario);
  for (const auto& object : objects) {
    out << "    " << object.name;
    if (!object.type.empty()) {
      out << " - " << object.type;
    }
    out << "\n";
  }
  out << "  )\n\n";

  out << "  (:init\n";
  for (const auto& fact : scenario->initialState) {
    out << "    ";
    emitFact(out, fact);
    out << "\n";
  }
  out << "  )\n\n";

  out << "  (:goal ";
  if (scenario->goals.size() == 1U) {
    emitFact(out, scenario->goals.front());
    out << ")\n";
  } else {
    out << "(and\n";
    for (const auto& goal : scenario->goals) {
      out << "    ";
      emitFact(out, goal);
      out << "\n";
    }
    out << "  ))\n";
  }

  out << ")\n";
  return out.str();
}
