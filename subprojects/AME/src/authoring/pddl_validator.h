#pragma once

#include <string>
#include <vector>

struct ProjectModel;

struct ValidationError {
  std::string message;
  std::vector<std::string> predicateNames;
  std::vector<std::string> actionNames;
};

struct ValidationReport {
  bool ok = false;
  std::vector<ValidationError> errors;
};

class PddlValidator {
public:
  /// \brief Generate and parse PDDL for the domain and optional scenario problem.
  static ValidationReport validate(const ProjectModel& model,
                                   const std::string& scenarioName = "");
};
