#pragma once

#include <cstddef>
#include <string>
#include <vector>

struct ProjectModel;

enum class ScenarioOutcome { Pass, Fail, Error };

struct ScenarioRunResult {
  std::string scenarioName;
  ScenarioOutcome outcome = ScenarioOutcome::Error;
  std::string reason;
  size_t planStepCount = 0;
  double solveTimeMs = 0.0;
  bool planSucceeded = false;
  std::vector<std::string> usedActionSchemas;
};

struct ScenarioBatchReport {
  std::vector<ScenarioRunResult> results;
  size_t passCount = 0;
  size_t failCount = 0;
  size_t errorCount = 0;
};

class ScenarioRunner {
public:
  /// \brief Run every scenario through validation and planning, then compare
  /// against the scenario expectation.
  static ScenarioBatchReport runAll(const ProjectModel& model);

  /// \brief Serialise a scenario batch report to pretty-printed JSON.
  static std::string toJson(const ScenarioBatchReport& report);
};
