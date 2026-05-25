#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct ProjectModel;

struct ContingencyContext {
  std::vector<std::string> trueFluents;
  bool planFound = false;
  size_t planSteps = 0;
  std::string errorMessage;
};

struct ContingencyReport {
  bool ok = false;
  std::string error;
  std::vector<std::string> contextPredicates;
  std::vector<std::string> contextFluents;
  std::vector<ContingencyContext> results;
  size_t feasibleCount = 0;
  size_t infeasibleCount = 0;
  size_t errorCount = 0;
};

class ContingencyAnalyser {
public:
  /// \brief Enumerate context fluent assignments and solve each resulting scenario.
  static ContingencyReport analyse(const ProjectModel& model,
                                   const std::string& scenarioName,
                                   size_t maxFluents = 8);
};
