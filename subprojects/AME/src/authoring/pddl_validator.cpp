#include "pddl_validator.h"

#include "pddl_generator.h"
#include "project_model.h"

#include <ame/pddl_parser.h>
#include <ame/world_model.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <utility>

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

bool hasScenario(const ProjectModel& model, const std::string& scenarioName) {
  return std::any_of(model.scenarios.begin(), model.scenarios.end(),
                     [&scenarioName](const ScenarioDef& scenario) {
                       return scenario.name == scenarioName;
                     });
}

std::string emptyProblemForDomain(const ProjectModel& model) {
  return "(define (problem __validate__) (:domain " +
         slugify(model.projectName) + "))\n";
}

void addNameMatches(const ProjectModel& model,
                    const std::string& message,
                    ValidationError& error) {
  for (const auto& predicate : model.predicates) {
    if (!predicate.name.empty() &&
        message.find(predicate.name) != std::string::npos) {
      error.predicateNames.push_back(predicate.name);
    }
  }
  for (const auto& action : model.actions) {
    if (!action.name.empty() && message.find(action.name) != std::string::npos) {
      error.actionNames.push_back(action.name);
    }
  }
}

void addAuthoringPrecheckErrors(const ProjectModel& model,
                                ValidationReport& report) {
  for (const auto& predicate : model.predicates) {
    if (predicate.name.empty()) {
      ValidationError error;
      error.message = "PDDL validation error: predicate name is empty";
      report.errors.push_back(std::move(error));
    }
  }

  for (const auto& action : model.actions) {
    if (action.name.empty()) {
      ValidationError error;
      error.message = "PDDL validation error: action name is empty";
      report.errors.push_back(std::move(error));
    }
  }
}

bool fluentMatchesPredicate(const std::string& fluentName,
                            const std::string& predicateName) {
  const std::string exact = "(" + predicateName + ")";
  const std::string prefix = "(" + predicateName + " ";
  return fluentName == exact || fluentName.rfind(prefix, 0) == 0;
}

GroundingReport buildGroundingReport(const ProjectModel& model,
                                     const ame::WorldModel& wm) {
  GroundingReport grounding;
  grounding.valid = true;
  grounding.totalFluents = wm.numFluents();
  grounding.totalGroundActions = wm.numGroundActions();

  for (const auto& predicate : model.predicates) {
    GroundingStat stat;
    stat.elementName = predicate.name;
    for (unsigned i = 0; i < wm.numFluents(); ++i) {
      if (fluentMatchesPredicate(wm.fluentName(i), predicate.name)) {
        ++stat.count;
      }
    }
    if (stat.count == 0U) {
      grounding.warnings.push_back(
          "Predicate '" + predicate.name +
          "' has no ground instances (no matching objects)");
    }
    grounding.predicateStats.push_back(std::move(stat));
  }

  for (size_t actionIdx = 0; actionIdx < model.actions.size(); ++actionIdx) {
    const auto& action = model.actions[actionIdx];
    GroundingStat stat;
    stat.elementName = action.name;
    for (const auto& groundAction : wm.groundActions()) {
      if (groundAction.schema_index == static_cast<unsigned>(actionIdx)) {
        ++stat.count;
      }
    }
    if (stat.count == 0U) {
      grounding.warnings.push_back("Action schema '" + action.name +
                                   "' has no ground actions");
    }
    grounding.actionStats.push_back(std::move(stat));
  }

  return grounding;
}

ValidationReport validateImpl(const ProjectModel& model,
                              const std::string& scenarioName,
                              ame::WorldModel* outWm) {
  ValidationReport report;
  addAuthoringPrecheckErrors(model, report);

  const std::string domain = PddlGenerator::generateDomain(model);
  const std::string problem =
      (!scenarioName.empty() && hasScenario(model, scenarioName))
          ? PddlGenerator::generateProblem(model, scenarioName)
          : emptyProblemForDomain(model);

  try {
    ame::WorldModel parsedWm;
    ame::PddlParser::parseFromString(domain, problem, parsedWm);
    report.grounding = buildGroundingReport(model, parsedWm);
    if (outWm != nullptr && report.errors.empty()) {
      *outWm = parsedWm;
    }
  } catch (const std::runtime_error& e) {
    ValidationError error;
    error.message = e.what();
    addNameMatches(model, error.message, error);
    report.errors.push_back(std::move(error));
  }

  report.ok = report.errors.empty();
  return report;
}

} // namespace

ValidationReport PddlValidator::validate(const ProjectModel& model,
                                         const std::string& scenarioName) {
  return validateImpl(model, scenarioName, nullptr);
}

ValidationReport PddlValidator::validateAndBuildWorldModel(
    const ProjectModel& model,
    const std::string& scenarioName,
    ame::WorldModel& wm) {
  return validateImpl(model, scenarioName, &wm);
}
