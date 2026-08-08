#include <gtest/gtest.h>

#include "project_model.h"
#include "review_pack.h"
#include "run_record.h"
#include "simulation_engine.h"
#include "test_shell_command.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>

namespace {

namespace fs = std::filesystem;

ProjectModel makeModel() {
  ProjectModel model;
  model.projectName = "recorded-search";
  model.types = {{"location", "object"}, {"robot", "object"}};
  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back(
      {"searched", {{"?l", "location"}}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"},
                 {"?to", "location"}};
  move.preconditions = {{"at", {"?r", "?from"}}};
  move.addEffects = {{"at", {"?r", "?to"}}};
  move.delEffects = {{"at", {"?r", "?from"}}};
  move.simulation.ticks = 2;
  model.actions.push_back(move);

  ActionDef search;
  search.name = "search";
  search.params = {{"?r", "robot"}, {"?l", "location"}};
  search.preconditions = {{"at", {"?r", "?l"}}};
  search.addEffects = {{"searched", {"?l"}}};
  search.simulation.ticks = 2;
  model.actions.push_back(search);

  model.objects = {{"uav1", "robot"}, {"base", "location"},
                   {"sector-a", "location"}};
  ScenarioDef nominal;
  nominal.name = "nominal";
  nominal.initialState = {{"at", {"uav1", "base"}}};
  nominal.goals = {{"searched", {"sector-a"}}};
  model.scenarios.push_back(nominal);
  ScenarioDef second = nominal;
  second.name = "second-scenario";
  model.scenarios.push_back(second);
  return model;
}

fs::path temporaryPath(const std::string& name) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return fs::temp_directory_path() /
         ("ame-authoring-" + name + "-" + std::to_string(stamp));
}

nlohmann::json firstJsonLine(const fs::path& path) {
  std::ifstream input(path);
  std::string line;
  std::getline(input, line);
  return nlohmann::json::parse(line);
}

std::vector<nlohmann::json> jsonLines(const fs::path& path) {
  std::vector<nlohmann::json> result;
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty()) {
      result.push_back(nlohmann::json::parse(line));
    }
  }
  return result;
}

using ame_test::shellQuote;

bool containsAction(const std::vector<std::string>& actions,
                    const std::string& name) {
  return std::any_of(actions.begin(), actions.end(),
                     [&name](const std::string& action) {
                       return action.find(name + "(") == 0U;
                     });
}

}  // namespace

TEST(RunRecord, RoundTripUsesTheDevEnvSchemas) {
  const ProjectModel model = makeModel();
  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal")) << engine.errorMessage();
  ASSERT_TRUE(engine.runToCompletion()) << engine.errorMessage();

  const fs::path folder = temporaryPath("round-trip");
  const RecordedRun captured = RecordedRun::fromSimulation(model, engine);
  ASSERT_TRUE(captured.save(folder.string()));

  const nlohmann::json manifest =
      nlohmann::json::parse(std::ifstream(folder / "run.json"));
  EXPECT_TRUE(manifest.at("simulated").get<bool>());
  EXPECT_EQ(manifest.at("project").get<std::string>(), model.projectName);
  EXPECT_EQ(manifest.at("scenario").get<std::string>(), "nominal");
  EXPECT_EQ(manifest.at("seed").get<unsigned>(), model.simulationSeed);
  EXPECT_TRUE(manifest.contains("faults"));
  EXPECT_EQ(manifest.at("timeBasis").get<std::string>(),
            "simulated_tick_time");
  EXPECT_DOUBLE_EQ(manifest.at("tickPeriodSeconds").get<double>(), 0.25);

  const nlohmann::json bt = firstJsonLine(folder / "ame_bt_events.jsonl");
  for (const char* field : {"ts_us", "node", "type", "prev", "status",
                            "tree_id", "wm_version"}) {
    EXPECT_TRUE(bt.contains(field)) << field;
  }
  EXPECT_TRUE(bt.at("ts_us").is_number_unsigned());
  EXPECT_TRUE(bt.at("node").is_string());
  EXPECT_TRUE(bt.at("wm_version").is_number_unsigned());

  const nlohmann::json wm = firstJsonLine(folder / "ame_wm_audit.jsonl");
  for (const char* field : {"wm_version", "ts_us", "fact", "value",
                            "source"}) {
    EXPECT_TRUE(wm.contains(field)) << field;
  }
  EXPECT_TRUE(wm.at("fact").is_string());
  EXPECT_TRUE(wm.at("value").is_boolean());

  const nlohmann::json plan =
      firstJsonLine(folder / "ame_plan_audit.jsonl");
  for (const char* field : {"ts_us", "init_facts", "goal_fluents", "solver",
                            "solve_time_ms", "success", "expanded",
                            "generated", "cost", "plan_actions", "bt_xml"}) {
    EXPECT_TRUE(plan.contains(field)) << field;
  }
  EXPECT_TRUE(plan.at("init_facts").is_array());
  EXPECT_TRUE(plan.at("goal_fluents").is_array());
  EXPECT_TRUE(plan.at("plan_actions").is_array());

  std::map<unsigned, uint64_t> timestamp_by_tick;
  uint64_t minimum_timestamp = std::numeric_limits<uint64_t>::max();
  uint64_t maximum_timestamp = 0U;
  const auto check_timestamps = [&](const fs::path& path) {
    for (const nlohmann::json& event : jsonLines(path)) {
      const unsigned tick = event.at("tick").get<unsigned>();
      const uint64_t timestamp = event.at("ts_us").get<uint64_t>();
      const auto inserted = timestamp_by_tick.emplace(tick, timestamp);
      if (!inserted.second) {
        EXPECT_EQ(inserted.first->second, timestamp);
      }
      minimum_timestamp = std::min(minimum_timestamp, timestamp);
      maximum_timestamp = std::max(maximum_timestamp, timestamp);
    }
  };
  check_timestamps(folder / "ame_bt_events.jsonl");
  check_timestamps(folder / "ame_wm_audit.jsonl");
  check_timestamps(folder / "ame_plan_audit.jsonl");
  EXPECT_EQ(maximum_timestamp - minimum_timestamp,
            static_cast<uint64_t>(engine.tick()) * 250000U);

  std::map<unsigned, std::vector<std::string>> expected_bt_order;
  for (const RunJsonEvent& event : engine.btAuditEvents()) {
    const nlohmann::json json = nlohmann::json::parse(event.json);
    expected_bt_order[event.tick].push_back(
        json.at("node").get<std::string>() + "\n" +
        json.at("status").get<std::string>());
  }
  std::map<unsigned, std::vector<std::string>> saved_bt_order;
  for (const nlohmann::json& event :
       jsonLines(folder / "ame_bt_events.jsonl")) {
    saved_bt_order[event.at("tick").get<unsigned>()].push_back(
        event.at("node").get<std::string>() + "\n" +
        event.at("status").get<std::string>());
  }
  EXPECT_EQ(saved_bt_order, expected_bt_order);

  RecordedRun replay;
  ASSERT_TRUE(replay.load(folder.string())) << replay.errorMessage();
  EXPECT_TRUE(replay.simulated());
  EXPECT_EQ(replay.manifest().scenario, "nominal");
  EXPECT_EQ(replay.tick(), engine.tick());
  EXPECT_FALSE(replay.actionSteps().empty());
  EXPECT_FALSE(replay.stateAtTick(replay.tick()).facts.empty());
  EXPECT_FALSE(replay.compiledXml().empty());
  fs::remove_all(folder);
}

TEST(RunRecordCli, WritesFolderThatRecordedRunLoads) {
  const fs::path root = temporaryPath("cli-record");
  fs::create_directories(root);
  const fs::path project_path = root / "project.ameproj.json";
  const fs::path record_path = root / "recorded-run";
  const fs::path report_path = root / "report.json";
  ASSERT_TRUE(makeModel().save(project_path.string()));

  // Recording from a script is ame_mission_cli's job: the graphical tool takes
  // no options but --self-test, so driving it here would open a window and hang.
  const std::string command = ame_test::shellCommand(
      shellQuote(AME_MISSION_CLI_PATH) + " record " + shellQuote(project_path) +
      " --scenario nominal --json " + shellQuote(report_path) + " --out " +
      shellQuote(record_path) + ame_test::discardOutput());
  ASSERT_EQ(std::system(command.c_str()), 0);

  RecordedRun recorded;
  ASSERT_TRUE(recorded.load(record_path.string())) << recorded.errorMessage();
  EXPECT_TRUE(recorded.simulated());
  EXPECT_EQ(recorded.manifest().scenario, "nominal");
  EXPECT_EQ(recorded.manifest().timeBasis, "simulated_tick_time");
  EXPECT_DOUBLE_EQ(recorded.manifest().tickPeriodSeconds, 0.25);
  EXPECT_GT(recorded.tick(), 0U);
  EXPECT_FALSE(recorded.actionSteps().empty());
  fs::remove_all(root);
}

TEST(RunRecord, RuntimeFolderWithoutManifestIsShownAsReal) {
  const ProjectModel model = makeModel();
  SimulationEngine engine;
  ASSERT_TRUE(engine.start(model, "nominal"));
  engine.runToCompletion();
  const fs::path folder = temporaryPath("real-marker");
  ASSERT_TRUE(RecordedRun::fromSimulation(model, engine).save(folder.string()));
  fs::remove(folder / "run.json");

  RecordedRun replay;
  ASSERT_TRUE(replay.load(folder.string())) << replay.errorMessage();
  EXPECT_FALSE(replay.simulated());
  fs::remove_all(folder);
}

TEST(RunComparison, FindsFirstTickAndNamesFaultedRunActions) {
  const ProjectModel model = makeModel();
  SimulationEngine nominal;
  ASSERT_TRUE(nominal.start(model, "nominal"));
  ASSERT_TRUE(nominal.runToCompletion());

  SimulationEngine faulted;
  RunFaultSet fault;
  fault.name = "search-fails-once";
  fault.actionFailures.push_back({"search", 1U});
  faulted.setFaults(fault);
  ASSERT_TRUE(faulted.start(model, "nominal"));
  ASSERT_TRUE(faulted.runToCompletion()) << faulted.errorMessage();
  EXPECT_EQ(faulted.planAuditEvents().size(), 2U);

  const RunComparison comparison = compareRuns(
      RecordedRun::fromSimulation(model, nominal),
      RecordedRun::fromSimulation(model, faulted));
  EXPECT_TRUE(comparison.treesDiffer);
  EXPECT_GT(comparison.firstDifferentTick, 0U);
  EXPECT_NE(comparison.summary.find("same until tick"), std::string::npos);
  EXPECT_TRUE(containsAction(comparison.actionsOnlyInSecond, "search"));
}

TEST(ReviewPack, WritesEveryPromisedFile) {
  const ProjectModel model = makeModel();
  const fs::path destination = temporaryPath("review-pack");
  fs::create_directories(destination);

  const ReviewPackResult result =
      ReviewPackExporter::write(model, destination.string());
  ASSERT_TRUE(result.success) << result.error;
  const fs::path folder(result.folder);
  EXPECT_TRUE(fs::is_regular_file(folder / "00-index.md"));
  EXPECT_TRUE(fs::is_regular_file(folder / "01-domain-recorded-search.pddl"));
  EXPECT_TRUE(fs::is_regular_file(
      folder / "02-scenario-1-nominal-problem.pddl"));
  EXPECT_TRUE(fs::is_regular_file(
      folder / "02-scenario-2-second-scenario-problem.pddl"));
  EXPECT_TRUE(fs::is_regular_file(folder / "03-fact-by-action-matrix.csv"));
  EXPECT_TRUE(fs::is_regular_file(folder / "03-fact-by-action-matrix.md"));
  EXPECT_TRUE(fs::is_regular_file(folder / "04-scenario-results.md"));
  EXPECT_TRUE(fs::is_regular_file(folder / "06-domain-summary.md"));

  const fs::path run = folder / "05-recorded-run-nominal";
  EXPECT_TRUE(fs::is_regular_file(run / "run.json"));
  EXPECT_TRUE(fs::is_regular_file(run / "ame_bt_events.jsonl"));
  EXPECT_TRUE(fs::is_regular_file(run / "ame_wm_audit.jsonl"));
  EXPECT_TRUE(fs::is_regular_file(run / "ame_plan_audit.jsonl"));

  // The stream is closed before the folder is deleted: Windows refuses to
  // delete a file that is still open.
  std::string summary;
  {
    std::ifstream summary_input(folder / "06-domain-summary.md");
    summary.assign((std::istreambuf_iterator<char>(summary_input)),
                   std::istreambuf_iterator<char>());
  }
  EXPECT_NE(summary.find("Types"), std::string::npos);
  EXPECT_NE(summary.find("Objects"), std::string::npos);
  EXPECT_NE(summary.find("Facts nothing produces"), std::string::npos);
  fs::remove_all(destination);
}
