#include "mission_commands.h"

#include "assurance_report.h"
#include "project_model.h"
#include "run_record.h"
#include "scenario_runner.h"
#include "simulation_engine.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

namespace {

const char* outcomeWord(ScenarioOutcome outcome) {
  switch (outcome) {
  case ScenarioOutcome::Pass:
    return "as expected";
  case ScenarioOutcome::Fail:
    return "not as expected";
  case ScenarioOutcome::Error:
    break;
  }
  return "could not be run";
}

std::string describe(const ScenarioRunResult& result) {
  std::ostringstream line;
  line << result.scenarioName << ": " << outcomeWord(result.outcome)
       << ", goal=" << (result.goalReached ? "reached" : "not reached")
       << ", actions=" << result.runActionCount
       << ", replans=" << result.replanCount
       << ", seed=" << result.simulationSeed;
  if (!result.reason.empty()) {
    line << " - " << result.reason;
  }
  return line.str();
}

std::string describe(const ScenarioBatchReport& report) {
  std::ostringstream out;
  for (size_t i = 0; i < report.results.size(); ++i) {
    if (i != 0) {
      out << '\n';
    }
    out << describe(report.results[i]);
  }
  if (report.results.size() > 1) {
    out << '\n'
        << report.passCount << " as expected, " << report.failCount
        << " not as expected, " << report.errorCount << " could not be run";
  }
  return out.str();
}

/// Open a project, and reduce it to one scenario when one is named.
bool openProject(const std::string& projectPath,
                 const std::string& scenarioName,
                 ProjectModel& model,
                 MissionCommandResult& result) {
  if (!model.load(projectPath)) {
    result.message = "Could not open the project file: " + projectPath;
    return false;
  }
  if (model.scenarios.empty()) {
    result.message = "The project has no scenarios to run.";
    return false;
  }
  if (scenarioName.empty()) {
    return true;
  }

  const auto found = std::find_if(model.scenarios.begin(), model.scenarios.end(),
                                  [&scenarioName](const ScenarioDef& scenario) {
                                    return scenario.name == scenarioName;
                                  });
  if (found == model.scenarios.end()) {
    result.message = "The project has no scenario called " + scenarioName + ".";
    return false;
  }
  model.scenarios = {*found};
  return true;
}

}  // namespace

MissionCommandResult MissionCommands::runScenario(
    const std::string& projectPath, const std::string& scenarioName) {
  MissionCommandResult result;
  ProjectModel model;
  if (scenarioName.empty()) {
    result.message = "A scenario is needed: name one, or use the batch command.";
    return result;
  }
  if (!openProject(projectPath, scenarioName, model, result)) {
    return result;
  }

  const ScenarioBatchReport report = ScenarioRunner::runAll(model);
  result.ran = true;
  result.message = describe(report);
  result.reportJson = ScenarioRunner::toJson(report);
  result.verdict = report.failCount == 0 && report.errorCount == 0;
  return result;
}

MissionCommandResult MissionCommands::runBatch(const std::string& projectPath) {
  MissionCommandResult result;
  ProjectModel model;
  if (!openProject(projectPath, "", model, result)) {
    return result;
  }

  const ScenarioBatchReport report = ScenarioRunner::runAll(model);
  result.ran = true;
  result.message = describe(report);
  result.reportJson = ScenarioRunner::toJson(report);
  result.verdict = report.failCount == 0 && report.errorCount == 0;
  return result;
}

MissionCommandResult MissionCommands::recordScenario(
    const std::string& projectPath,
    const std::string& scenarioName,
    const std::string& outFolder) {
  MissionCommandResult result;
  if (scenarioName.empty()) {
    result.message =
        "A scenario is needed: one folder holds one run, so name the run.";
    return result;
  }
  if (outFolder.empty()) {
    result.message = "A folder is needed to write the run into.";
    return result;
  }

  ProjectModel model;
  if (!openProject(projectPath, scenarioName, model, result)) {
    return result;
  }

  const ScenarioDef& scenario = model.scenarios.front();
  SimulationEngine simulation;
  simulation.setFaults(scenario.expectation.runFault);
  if (!simulation.start(model, scenario.name)) {
    result.message = "Could not record the run: " + simulation.errorMessage();
    return result;
  }
  simulation.runToCompletion();

  const RecordedRun recorded = RecordedRun::fromSimulation(model, simulation);
  if (!recorded.save(outFolder)) {
    result.message = "Could not write the recorded run to " + outFolder;
    return result;
  }

  result.ran = true;
  // A recording is a record of what happened, so it is written whether the
  // mission worked or not. The verdict still reports whether it did.
  result.verdict = simulation.phase() == RunPhase::Completed;
  std::ostringstream message;
  message << "Recorded " << scenario.name << " in " << outFolder << ": "
          << runPhaseName(simulation.phase()) << " after " << simulation.tick()
          << " ticks";
  result.message = message.str();
  result.reportJson = simulation.toJson();
  return result;
}

MissionCommandResult MissionCommands::assuranceEvidence(
    const std::string& projectPath) {
  MissionCommandResult result;
  ProjectModel model;
  if (!openProject(projectPath, "", model, result)) {
    return result;
  }

  const ScenarioBatchReport batch = ScenarioRunner::runAll(model);
  result.ran = true;
  result.verdict = batch.failCount == 0 && batch.errorCount == 0;
  result.reportJson = AssuranceReport::generate(model);
  std::ostringstream message;
  message << "Assurance evidence for " << model.projectName << ": "
          << batch.passCount << " of " << batch.results.size()
          << " scenarios as expected";
  result.message = message.str();
  return result;
}

bool MissionCommands::writeReport(const std::string& path,
                                  const std::string& json,
                                  std::string& error) {
  std::ofstream out(path);
  if (!out) {
    error = "Could not write the report to " + path;
    return false;
  }
  out << json << '\n';
  if (!out) {
    error = "Could not finish writing the report to " + path;
    return false;
  }
  return true;
}
