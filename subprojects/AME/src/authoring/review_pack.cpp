#include "review_pack.h"

#include "assurance_report.h"

#include "fact_action_matrix.h"
#include "pddl_generator.h"
#include "relation_index.h"
#include "run_record.h"
#include "scenario_runner.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {

namespace fs = std::filesystem;

std::string slug(std::string value) {
  std::string result;
  bool separator = false;
  for (const unsigned char character : value) {
    if (std::isalnum(character) != 0) {
      result.push_back(static_cast<char>(std::tolower(character)));
      separator = false;
    } else if (!result.empty() && !separator) {
      result.push_back('-');
      separator = true;
    }
  }
  while (!result.empty() && result.back() == '-') {
    result.pop_back();
  }
  return result.empty() ? "untitled" : result;
}

std::string datedFolderName() {
  const std::time_t now = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
  std::tm local{};
#if defined(_WIN32)
  localtime_s(&local, &now);
#else
  localtime_r(&now, &local);
#endif
  std::ostringstream name;
  name << "ame-review-" << std::put_time(&local, "%Y%m%d-%H%M%S");
  return name.str();
}

bool writeText(const fs::path& path, const std::string& text) {
  std::ofstream output(path);
  output << text;
  return static_cast<bool>(output);
}

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

std::string scenarioTable(const ScenarioBatchReport& report) {
  std::ostringstream markdown;
  markdown << "# Scenario results\n\n"
           << "These results come from simulated execution.\n\n"
           << "| Scenario | Result | Goal reached | Actions run | Replans | Reason |\n"
           << "|---|---|---:|---:|---:|---|\n";
  for (const ScenarioRunResult& result : report.results) {
    markdown << "| " << result.scenarioName << " | "
             << outcomeName(result.outcome) << " | "
             << (result.goalReached ? "yes" : "no") << " | "
             << result.runActionCount << " | " << result.replanCount << " | "
             << result.reason << " |\n";
  }
  return markdown.str();
}

std::string domainSummary(const ProjectModel& model,
                          const RelationIndex& index) {
  std::ostringstream markdown;
  markdown << "# Domain summary\n\n";
  const auto section = [&markdown](const char* title, const auto& values,
                                   const auto& name) {
    markdown << "## " << title << " (" << values.size() << ")\n\n";
    if (values.empty()) {
      markdown << "None.\n\n";
      return;
    }
    for (const auto& value : values) {
      markdown << "- " << name(value) << '\n';
    }
    markdown << '\n';
  };
  section("Types", model.types,
          [](const TypeDef& value) { return value.name + " — child of " + value.parent; });
  section("Objects", model.objects,
          [](const ObjectDef& value) { return value.name + " — " + value.type; });
  section("Facts", model.predicates,
          [](const PredicateDef& value) { return value.name; });
  section("Actions", model.actions,
          [](const ActionDef& value) { return value.name; });

  markdown << "## Facts nothing produces\n\n";
  const std::vector<size_t> unproduced = index.factsNoActionMakesTrue();
  if (unproduced.empty()) {
    markdown << "Every fact can be made true by at least one action.\n";
  } else {
    for (const size_t predicate_index : unproduced) {
      if (predicate_index < model.predicates.size()) {
        markdown << "- " << model.predicates[predicate_index].name << '\n';
      }
    }
  }
  return markdown.str();
}

}  // namespace

ReviewPackResult ReviewPackExporter::write(
    const ProjectModel& model,
    const std::string& destination,
    const SimulationEngine* current_run) {
  ReviewPackResult result;
  try {
    fs::path folder = fs::path(destination) / datedFolderName();
    unsigned suffix = 2U;
    while (fs::exists(folder)) {
      folder = fs::path(destination) /
               (datedFolderName() + "-" + std::to_string(suffix++));
    }
    fs::create_directories(folder);
    result.folder = folder.string();

    const std::string project_slug = slug(model.projectName);
    const std::string domain_name = "01-domain-" + project_slug + ".pddl";
    if (!writeText(folder / domain_name, PddlGenerator::generateDomain(model))) {
      throw std::runtime_error("the domain PDDL could not be written");
    }

    std::vector<std::string> problem_names;
    for (size_t i = 0; i < model.scenarios.size(); ++i) {
      const std::string name = "02-scenario-" + std::to_string(i + 1U) + "-" +
                               slug(model.scenarios[i].name) + "-problem.pddl";
      if (!writeText(folder / name,
                     PddlGenerator::generateProblem(model,
                                                    model.scenarios[i].name))) {
        throw std::runtime_error("a scenario problem file could not be written");
      }
      problem_names.push_back(name);
    }

    const RelationIndex index(model);
    const FactActionMatrix matrix(model, index);
    const std::string matrix_csv = "03-fact-by-action-matrix.csv";
    const std::string matrix_markdown = "03-fact-by-action-matrix.md";
    if (!writeText(folder / matrix_csv, matrix.toCsv(model)) ||
        !writeText(folder / matrix_markdown, matrix.toMarkdown(model))) {
      throw std::runtime_error("the fact-by-action matrix could not be written");
    }

    const ScenarioBatchReport scenario_report = ScenarioRunner::runAll(model);
    const std::string scenario_name = "04-scenario-results.md";
    if (!writeText(folder / scenario_name, scenarioTable(scenario_report))) {
      throw std::runtime_error("the scenario results could not be written");
    }

    RecordedRun recorded;
    SimulationEngine generated_run;
    if (current_run != nullptr && current_run->isLoaded()) {
      recorded = RecordedRun::fromSimulation(model, *current_run);
    } else {
      if (model.scenarios.empty()) {
        throw std::runtime_error("the project has no scenario to record");
      }
      const ScenarioDef& scenario = model.scenarios.front();
      generated_run.setFaults(scenario.expectation.runFault);
      if (!generated_run.start(model, scenario.name)) {
        throw std::runtime_error("the recorded run could not start: " +
                                 generated_run.errorMessage());
      }
      generated_run.runToCompletion();
      recorded = RecordedRun::fromSimulation(model, generated_run);
    }
    const std::string run_name =
        "05-recorded-run-" + slug(recorded.manifest().scenario);
    if (!recorded.save((folder / run_name).string())) {
      throw std::runtime_error("the recorded run could not be written");
    }

    const std::string summary_name = "06-domain-summary.md";
    if (!writeText(folder / summary_name, domainSummary(model, index))) {
      throw std::runtime_error("the domain summary could not be written");
    }

    // The assurance report goes in the pack too: it is the one file that says
    // what has and has not been checked, which is the question a reviewer asks
    // first.
    const std::string assurance_name = "07-assurance-evidence.md";
    if (!writeText(folder / assurance_name, AssuranceReport::generate(model))) {
      throw std::runtime_error("the assurance report could not be written");
    }

    std::ostringstream index_text;
    index_text << "# AME review pack\n\n"
               << "This folder was generated from project **" << model.projectName
               << "**. The recorded run and scenario results are simulations.\n\n"
               << "- `" << domain_name << "` is the generated PDDL domain.\n";
    for (const std::string& name : problem_names) {
      index_text << "- `" << name << "` is one scenario's generated PDDL problem.\n";
    }
    index_text << "- `" << matrix_csv
               << "` is the fact-by-action matrix for spreadsheet software.\n"
               << "- `" << matrix_markdown
               << "` is the same matrix for a document review.\n"
               << "- `" << scenario_name
               << "` lists every scenario result and its reason.\n"
               << "- `" << run_name
               << "/` contains a replayable run and its provenance manifest.\n"
               << "- `" << summary_name
               << "` lists the domain contents and facts nothing produces.\n"
               << "- `" << assurance_name
               << "` is the assurance evidence: what has been checked about "
                  "this model, and what has not.\n";
    if (!writeText(folder / "00-index.md", index_text.str())) {
      throw std::runtime_error("the review pack index could not be written");
    }
    result.success = true;
  } catch (const std::exception& exception) {
    result.error = exception.what();
  }
  return result;
}

