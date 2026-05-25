#include "scenario_runner.h"

#include "pddl_validator.h"
#include "project_model.h"

#include <ame/planner.h>
#include <ame/world_model.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <exception>
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

bool contains(const std::vector<std::string>& values, const std::string& needle) {
  return std::find(values.begin(), values.end(), needle) != values.end();
}

void addUsedActionSchemas(ScenarioRunResult& result,
                          const ProjectModel& model,
                          const ame::PlanResult& plan,
                          const ame::WorldModel& wm) {
  const std::vector<ame::GroundAction>& groundActions = wm.groundActions();
  for (const auto& step : plan.steps) {
    if (step.action_index >= groundActions.size()) {
      continue;
    }

    const unsigned schemaIdx = groundActions[step.action_index].schema_index;
    if (schemaIdx >= model.actions.size()) {
      continue;
    }

    const std::string& actionName = model.actions[schemaIdx].name;
    if (!contains(result.usedActionSchemas, actionName)) {
      result.usedActionSchemas.push_back(actionName);
    }
  }
}

void evaluateExpectation(ScenarioRunResult& result,
                         const ScenarioExpectation& expectation) {
  if (expectation.shouldSucceed && !result.planSucceeded) {
    result.outcome = ScenarioOutcome::Fail;
    result.reason = "expected success but no plan found";
    return;
  }

  if (!expectation.shouldSucceed && result.planSucceeded) {
    result.outcome = ScenarioOutcome::Fail;
    result.reason = "expected infeasible but planner found a plan";
    return;
  }

  if (result.planSucceeded && expectation.minPlanSteps > 0 &&
      result.planStepCount < static_cast<size_t>(expectation.minPlanSteps)) {
    result.outcome = ScenarioOutcome::Fail;
    result.reason = "plan too short (" + std::to_string(result.planStepCount) +
                    " steps, expected >= " +
                    std::to_string(expectation.minPlanSteps) + ")";
    return;
  }

  if (result.planSucceeded && expectation.maxPlanSteps > 0 &&
      result.planStepCount > static_cast<size_t>(expectation.maxPlanSteps)) {
    result.outcome = ScenarioOutcome::Fail;
    result.reason = "plan too long (" + std::to_string(result.planStepCount) +
                    " steps, expected <= " +
                    std::to_string(expectation.maxPlanSteps) + ")";
    return;
  }

  for (const auto& expected : expectation.expectedActions) {
    if (!contains(result.usedActionSchemas, expected)) {
      result.outcome = ScenarioOutcome::Fail;
      result.reason = "expected action '" + expected + "' not used";
      return;
    }
  }

  for (const auto& forbidden : expectation.forbiddenActions) {
    if (contains(result.usedActionSchemas, forbidden)) {
      result.outcome = ScenarioOutcome::Fail;
      result.reason = "forbidden action '" + forbidden + "' used";
      return;
    }
  }

  result.outcome = ScenarioOutcome::Pass;
  result.reason.clear();
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

ScenarioBatchReport ScenarioRunner::runAll(const ProjectModel& model) {
  ScenarioBatchReport report;

  for (const auto& scenario : model.scenarios) {
    ScenarioRunResult result;
    result.scenarioName = scenario.name;

    ame::WorldModel wm;
    const ValidationReport validation =
        PddlValidator::validateAndBuildWorldModel(model, scenario.name, wm);
    if (!validation.ok) {
      result.outcome = ScenarioOutcome::Error;
      if (!validation.errors.empty()) {
        result.reason = "parse failed: " + validation.errors.front().message;
      } else {
        result.reason = "parse failed";
      }
      tally(report, result.outcome);
      report.results.push_back(std::move(result));
      continue;
    }

    try {
      ame::Planner planner;
      const ame::PlanResult plan = planner.solve(wm);
      result.planSucceeded = plan.success;
      result.planStepCount = plan.steps.size();
      result.solveTimeMs = plan.solve_time_ms;
      addUsedActionSchemas(result, model, plan, wm);
      evaluateExpectation(result, scenario.expectation);
    } catch (const std::exception& ex) {
      result.outcome = ScenarioOutcome::Error;
      result.reason = std::string("planner threw: ") + ex.what();
    } catch (...) {
      result.outcome = ScenarioOutcome::Error;
      result.reason = "planner threw: unknown exception";
    }

    tally(report, result.outcome);
    report.results.push_back(std::move(result));
  }

  return report;
}

std::string ScenarioRunner::toJson(const ScenarioBatchReport& report) {
  nlohmann::json json;
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
    });
  }

  return json.dump(2);
}
