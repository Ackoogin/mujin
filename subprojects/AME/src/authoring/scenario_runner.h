#pragma once

#include "project_model.h"

#include <cstddef>
#include <string>
#include <vector>

enum class ScenarioOutcome { Pass, Fail, Error };

struct ScenarioRunResult {
  std::string scenarioName;
  ScenarioOutcome outcome = ScenarioOutcome::Error;
  std::string reason;
  size_t planStepCount = 0;
  double solveTimeMs = 0.0;
  bool planSucceeded = false;
  std::vector<std::string> usedActionSchemas;
  bool executionAttempted = false;
  bool goalReached = false;
  size_t runActionCount = 0;
  size_t replanCount = 0;
  unsigned simulationSeed = 0;
  std::string faultName;
  std::vector<std::string> runActionSchemas;
};

struct ScenarioBatchReport {
  std::vector<ScenarioRunResult> results;
  size_t passCount = 0;
  size_t failCount = 0;
  size_t errorCount = 0;
  unsigned simulationSeed = 0;
  bool stopped = false;
};

class ScenarioRunner {
public:
  /// \brief Begin a batch and leave its first scenario waiting to run.
  void start(const ProjectModel& model);

  /// \brief Simulate one waiting scenario.
  /// \return True while another scenario remains.
  bool step();

  /// \brief Stop after the scenario that most recently finished.
  void stop();

  bool isRunning() const { return running_; }
  bool wasStopped() const { return report_.stopped; }
  size_t completedCount() const { return next_scenario_; }
  size_t totalCount() const { return model_.scenarios.size(); }
  std::string currentScenarioName() const;
  const ScenarioBatchReport& report() const { return report_; }

  /// \brief Run every scenario through planning and simulated execution.
  static ScenarioBatchReport runAll(const ProjectModel& model);

  /// \brief Serialise a scenario batch report to pretty-printed JSON.
  static std::string toJson(const ScenarioBatchReport& report);

private:
  ScenarioRunResult runOne(const ScenarioDef& scenario) const;

  ProjectModel model_;
  ScenarioBatchReport report_;
  size_t next_scenario_ = 0;
  bool running_ = false;
};
