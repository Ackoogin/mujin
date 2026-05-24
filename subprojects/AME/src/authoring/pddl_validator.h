#pragma once

#include <string>
#include <vector>

struct ProjectModel;

struct ValidationError {
  std::string message;
  std::vector<std::string> predicateNames;
  std::vector<std::string> actionNames;
};

struct GroundingStat {
  std::string elementName;
  unsigned count = 0;
};

struct GroundingReport {
  bool valid = false;
  unsigned totalFluents = 0;
  unsigned totalGroundActions = 0;
  std::vector<GroundingStat> predicateStats;
  std::vector<GroundingStat> actionStats;
  std::vector<std::string> warnings;
};

struct ValidationReport {
  bool ok = false;
  std::vector<ValidationError> errors;
  GroundingReport grounding;
};

class PddlValidator {
public:
  /// \brief Generate and parse PDDL for the domain and optional scenario problem.
  static ValidationReport validate(const ProjectModel& model,
                                   const std::string& scenarioName = "");
};
