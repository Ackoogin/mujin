#include "scenario_runner.h"

#include "pddl_validator.h"
#include "simulation_engine.h"

#include <ame/planner.h>
#include <ame/world_model.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

namespace {

const char* outcomeName(ScenarioOutcome outcome) {
  switch (outcome) {
  case ScenarioOutcome::Pass:
    return "Pass";
  case ScenarioOutcome::Fail:
    return "Fail";
  case ScenarioOutcome::Error:
    return "Error";
  }
  return "Error";
}

bool contains(const std::vector<std::string>& values,
              const std::string& needle) {
  return std::find(values.begin(), values.end(), needle) != values.end();
}

void appendReason(std::vector<std::string>& reasons, std::string reason) {
  reasons.push_back(std::move(reason));
}

std::string joinReasons(const std::vector<std::string>& reasons) {
  std::ostringstream text;
  for (size_t i = 0; i < reasons.size(); ++i) {
    if (i != 0U) {
      text << ' ';
    }
    text << reasons[i];
    if (!reasons[i].empty() && reasons[i].back() != '.') {
      text << '.';
    }
  }
  return text.str();
}

void addUsedActionSchemas(ScenarioRunResult& result,
                          const ProjectModel& model,
                          const ame::PlanResult& plan,
                          const ame::WorldModel& world_model) {
  const std::vector<ame::GroundAction>& ground_actions =
      world_model.groundActions();
  for (const auto& step : plan.steps) {
    if (step.action_index >= ground_actions.size()) {
      continue;
    }
    const unsigned schema_index =
        ground_actions[step.action_index].schema_index;
    if (schema_index >= model.actions.size()) {
      continue;
    }
    const std::string& action_name = model.actions[schema_index].name;
    if (!contains(result.usedActionSchemas, action_name)) {
      result.usedActionSchemas.push_back(action_name);
    }
  }
}

void evaluatePlanning(const ScenarioRunResult& result,
                      const ScenarioExpectation& expectation,
                      std::vector<std::string>& reasons) {
  if (expectation.shouldSucceed && !result.planSucceeded) {
    appendReason(reasons, "Expected planning to succeed, but no plan was found");
    return;
  }
  if (!expectation.shouldSucceed && result.planSucceeded) {
    appendReason(reasons, "Expected no plan, but the planner found one");
  }
  if (!result.planSucceeded) {
    return;
  }
  if (expectation.minPlanSteps > 0 &&
      result.planStepCount < static_cast<size_t>(expectation.minPlanSteps)) {
    appendReason(reasons, "The plan had " +
                              std::to_string(result.planStepCount) +
                              " steps; at least " +
                              std::to_string(expectation.minPlanSteps) +
                              " were expected");
  }
  if (expectation.maxPlanSteps > 0 &&
      result.planStepCount > static_cast<size_t>(expectation.maxPlanSteps)) {
    appendReason(reasons, "The plan had " +
                              std::to_string(result.planStepCount) +
                              " steps; no more than " +
                              std::to_string(expectation.maxPlanSteps) +
                              " were expected");
  }
  for (const std::string& expected : expectation.expectedActions) {
    if (!contains(result.usedActionSchemas, expected)) {
      appendReason(reasons, "Expected planning action '" + expected +
                                "' was not used");
    }
  }
  for (const std::string& forbidden : expectation.forbiddenActions) {
    if (contains(result.usedActionSchemas, forbidden)) {
      appendReason(reasons, "Forbidden planning action '" + forbidden +
                                "' was used");
    }
  }
}

void evaluateExecution(const ScenarioRunResult& result,
                       const ScenarioExpectation& expectation,
                       std::vector<std::string>& reasons) {
  if (!result.executionAttempted) {
    return;
  }
  if (result.goalReached != expectation.shouldReachGoal) {
    appendReason(reasons,
                 expectation.shouldReachGoal
                     ? "The run did not reach its goal"
                     : "The run reached its goal, but it was expected not to");
  }
  if (expectation.minRunActions > 0 &&
      result.runActionCount < static_cast<size_t>(expectation.minRunActions)) {
    appendReason(reasons, "The run used " +
                              std::to_string(result.runActionCount) +
                              " actions; at least " +
                              std::to_string(expectation.minRunActions) +
                              " were expected");
  }
  if (expectation.maxRunActions > 0 &&
      result.runActionCount > static_cast<size_t>(expectation.maxRunActions)) {
    appendReason(reasons, "The run used " +
                              std::to_string(result.runActionCount) +
                              " actions; no more than " +
                              std::to_string(expectation.maxRunActions) +
                              " were expected");
  }
  for (const std::string& required : expectation.requiredRunActions) {
    if (!contains(result.runActionSchemas, required)) {
      appendReason(reasons, "Required run action '" + required +
                                "' did not appear");
    }
  }
  for (const std::string& forbidden : expectation.forbiddenRunActions) {
    if (contains(result.runActionSchemas, forbidden)) {
      appendReason(reasons, "Run action '" + forbidden +
                                "' appeared, but the scenario forbids it");
    }
  }
  if (expectation.maxReplans >= 0 &&
      result.replanCount > static_cast<size_t>(expectation.maxReplans)) {
    appendReason(reasons, "The run replanned " +
                              std::to_string(result.replanCount) +
                              " times; no more than " +
                              std::to_string(expectation.maxReplans) +
                              " were expected");
  }
}

void tally(ScenarioBatchReport& report, ScenarioOutcome outcome) {
  switch (outcome) {
  case ScenarioOutcome::Pass:
    ++report.passCount;
    break;
  case ScenarioOutcome::Fail:
    ++report.failCount;
    break;
  case ScenarioOutcome::Error:
    ++report.errorCount;
    break;
  }
}

} // namespace

void ScenarioRunner::start(const ProjectModel& model) {
  model_ = model;
  report_ = ScenarioBatchReport{};
  report_.simulationSeed = model.simulationSeed;
  next_scenario_ = 0;
  running_ = !model_.scenarios.empty();
}

bool ScenarioRunner::step() {
  if (!running_ || next_scenario_ >= model_.scenarios.size()) {
    running_ = false;
    return false;
  }
  ScenarioRunResult result = runOne(model_.scenarios[next_scenario_]);
  tally(report_, result.outcome);
  report_.results.push_back(std::move(result));
  ++next_scenario_;
  running_ = next_scenario_ < model_.scenarios.size();
  return running_;
}

void ScenarioRunner::stop() {
  if (running_) {
    report_.stopped = true;
  }
  running_ = false;
}

std::string ScenarioRunner::currentScenarioName() const {
  return next_scenario_ < model_.scenarios.size()
             ? model_.scenarios[next_scenario_].name
             : std::string{};
}

ScenarioRunResult ScenarioRunner::runOne(const ScenarioDef& scenario) const {
  ScenarioRunResult result;
  result.scenarioName = scenario.name;
  result.simulationSeed = model_.simulationSeed;
  result.faultName = scenario.expectation.runFault.name;
  std::vector<std::string> reasons;

  ame::WorldModel world_model;
  const ValidationReport validation =
      PddlValidator::validateAndBuildWorldModel(model_, scenario.name,
                                                world_model);
  if (!validation.ok) {
    result.outcome = ScenarioOutcome::Error;
    result.reason = validation.errors.empty()
                        ? "The scenario could not be parsed."
                        : "The scenario could not be parsed: " +
                              validation.errors.front().message;
    return result;
  }

  try {
    ame::Planner planner;
    const ame::PlanResult plan = planner.solve(world_model);
    result.planSucceeded = plan.success;
    result.planStepCount = plan.steps.size();
    result.solveTimeMs = plan.solve_time_ms;
    addUsedActionSchemas(result, model_, plan, world_model);
  } catch (const std::exception& ex) {
    result.outcome = ScenarioOutcome::Error;
    result.reason = std::string("The planner could not run: ") + ex.what();
    return result;
  }

  evaluatePlanning(result, scenario.expectation, reasons);
  if (result.planSucceeded) {
    SimulationEngine simulation;
    simulation.setFaults(scenario.expectation.runFault);
    if (!simulation.start(model_, scenario.name)) {
      result.outcome = ScenarioOutcome::Error;
      result.reason = "The simulation could not start: " +
                      simulation.errorMessage();
      return result;
    }
    result.executionAttempted = true;
    simulation.runToCompletion();
    if (simulation.phase() == RunPhase::Error) {
      result.outcome = ScenarioOutcome::Error;
      result.reason = "The simulation could not finish: " +
                      simulation.errorMessage();
      return result;
    }
    result.goalReached = simulation.phase() == RunPhase::Completed &&
                         simulation.goalsMetCount() == simulation.goals().size();
    result.runActionCount = simulation.actionsRunCount();
    result.replanCount = simulation.replanCount();
    result.runActionSchemas = simulation.actionsRun();
    evaluateExecution(result, scenario.expectation, reasons);
  }

  if (reasons.empty()) {
    result.outcome = ScenarioOutcome::Pass;
    result.reason = result.executionAttempted
                        ? "Planning and execution matched the scenario expectations."
                        : "Planning matched the scenario expectations; there was no plan to simulate.";
  } else {
    result.outcome = ScenarioOutcome::Fail;
    result.reason = joinReasons(reasons);
  }
  return result;
}

ScenarioBatchReport ScenarioRunner::runAll(const ProjectModel& model) {
  ScenarioRunner runner;
  runner.start(model);
  while (runner.isRunning()) {
    runner.step();
  }
  return runner.report();
}

std::string ScenarioRunner::toJson(const ScenarioBatchReport& report) {
  nlohmann::json json;
  json["simulated"] = true;
  json["simulationSeed"] = report.simulationSeed;
  json["stopped"] = report.stopped;
  json["passCount"] = report.passCount;
  json["failCount"] = report.failCount;
  json["errorCount"] = report.errorCount;
  json["results"] = nlohmann::json::array();

  for (const auto& result : report.results) {
    json["results"].push_back({
        {"scenarioName", result.scenarioName},
        {"outcome", outcomeName(result.outcome)},
        {"reason", result.reason},
        {"planStepCount", result.planStepCount},
        {"solveTimeMs", result.solveTimeMs},
        {"planSucceeded", result.planSucceeded},
        {"usedActionSchemas", result.usedActionSchemas},
        {"executionAttempted", result.executionAttempted},
        {"goalReached", result.goalReached},
        {"runActionCount", result.runActionCount},
        {"runActionSchemas", result.runActionSchemas},
        {"replanCount", result.replanCount},
        {"simulationSeed", result.simulationSeed},
        {"faultName", result.faultName},
    });
  }

  return json.dump(2);
}
