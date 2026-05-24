#include "pddl_generator.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string slugify(const std::string& text) {
  std::string out;
  bool lastWasDash = false;

  for (unsigned char ch : text) {
    const char lower = static_cast<char>(std::tolower(ch));
    const bool isSlugChar =
        (lower >= 'a' && lower <= 'z') || (lower >= '0' && lower <= '9') ||
        lower == '-';
    if (isSlugChar) {
      if (lower == '-') {
        if (!out.empty() && !lastWasDash) {
          out.push_back(lower);
        }
        lastWasDash = true;
      } else {
        out.push_back(lower);
        lastWasDash = false;
      }
    } else if (!out.empty() && !lastWasDash) {
      out.push_back('-');
      lastWasDash = true;
    }
  }

  while (!out.empty() && out.back() == '-') {
    out.pop_back();
  }

  if (out.empty()) {
    return "untitled";
  }
  return out;
}

std::string domainName(const ProjectModel& model) {
  return slugify(model.projectName);
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
  for (const auto& object : model.objects) {
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
