#include <gtest/gtest.h>

#include "mission_commands.h"
#include "project_model.h"
#include "run_record.h"
#include "test_shell_command.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

namespace fs = std::filesystem;

/// A project with one scenario that works and one that cannot be planned.
ProjectModel makeProject() {
  ProjectModel model;
  model.projectName = "cli-search";
  model.types = {{"location", "object"}, {"robot", "object"}};
  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back({"searched", {{"?l", "location"}}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  move.preconditions = {{"at", {"?r", "?from"}}};
  move.addEffects = {{"at", {"?r", "?to"}}};
  move.delEffects = {{"at", {"?r", "?from"}}};
  model.actions.push_back(move);

  ActionDef search;
  search.name = "search";
  search.params = {{"?r", "robot"}, {"?l", "location"}};
  search.preconditions = {{"at", {"?r", "?l"}}};
  search.addEffects = {{"searched", {"?l"}}};
  model.actions.push_back(search);

  model.objects = {{"uav1", "robot"}, {"base", "location"},
                   {"sector-a", "location"}};

  ScenarioDef nominal;
  nominal.name = "nominal";
  nominal.initialState = {{"at", {"uav1", "base"}}};
  nominal.goals = {{"searched", {"sector-a"}}};
  model.scenarios.push_back(nominal);

  // Nothing says where the vehicle starts, so no plan exists for this one.
  ScenarioDef impossible;
  impossible.name = "impossible";
  impossible.goals = {{"searched", {"sector-a"}}};
  model.scenarios.push_back(impossible);

  return model;
}

fs::path temporaryPath(const std::string& name) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return fs::temp_directory_path() /
         ("ame-cli-" + name + "-" + std::to_string(stamp));
}

/// Save a project and return its path, so the commands open it as a user would.
fs::path saveProject(const ProjectModel& model, const std::string& name) {
  const fs::path path = temporaryPath(name).string() + ".ameproj.json";
  EXPECT_TRUE(model.save(path.string()));
  return path;
}

}  // namespace

TEST(MissionCommands, RunningAScenarioThatBehavesGivesAGoodVerdict) {
  ProjectModel model = makeProject();
  model.scenarios.pop_back();  // keep only the one that works
  const fs::path project = saveProject(model, "run-good");

  const MissionCommandResult result =
      MissionCommands::runScenario(project.string(), "nominal");

  EXPECT_TRUE(result.ran);
  EXPECT_TRUE(result.verdict);
  EXPECT_NE(result.message.find("nominal"), std::string::npos);
  EXPECT_NE(result.message.find("goal=reached"), std::string::npos);
  EXPECT_FALSE(result.reportJson.empty());
  fs::remove(project);
}

TEST(MissionCommands, RunningAScenarioThatCannotBePlannedGivesABadVerdict) {
  const fs::path project = saveProject(makeProject(), "run-bad");

  const MissionCommandResult result =
      MissionCommands::runScenario(project.string(), "impossible");

  // The command itself worked; the mission did not.
  EXPECT_TRUE(result.ran);
  EXPECT_FALSE(result.verdict);
  fs::remove(project);
}

TEST(MissionCommands, TheBatchReportsEveryScenario) {
  const fs::path project = saveProject(makeProject(), "batch");

  const MissionCommandResult result =
      MissionCommands::runBatch(project.string());

  EXPECT_TRUE(result.ran);
  EXPECT_FALSE(result.verdict) << "one scenario cannot be planned";
  EXPECT_NE(result.message.find("nominal"), std::string::npos);
  EXPECT_NE(result.message.find("impossible"), std::string::npos);

  const nlohmann::json report = nlohmann::json::parse(result.reportJson);
  EXPECT_EQ(report["results"].size(), 2U);
  fs::remove(project);
}

TEST(MissionCommands, ARecordedRunLoadsBackFromItsFolder) {
  ProjectModel model = makeProject();
  model.scenarios.pop_back();
  const fs::path project = saveProject(model, "record");
  const fs::path folder = temporaryPath("record-out");

  const MissionCommandResult result = MissionCommands::recordScenario(
      project.string(), "nominal", folder.string());

  EXPECT_TRUE(result.ran);
  EXPECT_TRUE(result.verdict);
  EXPECT_NE(result.message.find("Recorded"), std::string::npos);

  RecordedRun replay;
  ASSERT_TRUE(replay.load(folder.string())) << replay.errorMessage();
  EXPECT_TRUE(replay.simulated());
  EXPECT_GT(replay.tick(), 0U);

  fs::remove_all(folder);
  fs::remove(project);
}

TEST(MissionCommands, RecordingNeedsAScenarioAndAFolder) {
  const fs::path project = saveProject(makeProject(), "record-args");

  const MissionCommandResult noScenario =
      MissionCommands::recordScenario(project.string(), "", "/tmp/unused");
  EXPECT_FALSE(noScenario.ran);
  EXPECT_FALSE(noScenario.message.empty());

  const MissionCommandResult noFolder =
      MissionCommands::recordScenario(project.string(), "nominal", "");
  EXPECT_FALSE(noFolder.ran);
  EXPECT_FALSE(noFolder.message.empty());
  fs::remove(project);
}

TEST(MissionCommands, AProjectThatCannotBeOpenedIsReportedNotCrashed) {
  const MissionCommandResult result =
      MissionCommands::runScenario("/no/such/project.ameproj.json", "nominal");

  EXPECT_FALSE(result.ran);
  EXPECT_FALSE(result.verdict);
  EXPECT_NE(result.message.find("Could not open"), std::string::npos);
}

TEST(MissionCommands, AScenarioTheProjectDoesNotHaveIsNamedInTheMessage) {
  const fs::path project = saveProject(makeProject(), "missing-scenario");

  const MissionCommandResult result =
      MissionCommands::runScenario(project.string(), "night-run");

  EXPECT_FALSE(result.ran);
  EXPECT_NE(result.message.find("night-run"), std::string::npos);
  fs::remove(project);
}

TEST(MissionCommands, AReportIsWrittenWhereItIsAskedFor) {
  ProjectModel model = makeProject();
  model.scenarios.pop_back();
  const fs::path project = saveProject(model, "report");
  const fs::path reportPath = temporaryPath("report").string() + ".json";

  const MissionCommandResult result =
      MissionCommands::runScenario(project.string(), "nominal");
  std::string error;
  ASSERT_TRUE(MissionCommands::writeReport(reportPath.string(),
                                           result.reportJson, error))
      << error;

  // The stream is closed before the file is deleted: Windows refuses to
  // delete a file that is still open.
  nlohmann::json written;
  {
    std::ifstream in(reportPath);
    ASSERT_TRUE(in.good());
    written = nlohmann::json::parse(in);
  }
  EXPECT_EQ(written["results"][0]["scenarioName"], "nominal");

  fs::remove(reportPath);
  fs::remove(project);
}

TEST(MissionCommands, AReportPathThatCannotBeWrittenSaysSo) {
  std::string error;
  EXPECT_FALSE(MissionCommands::writeReport("/no/such/folder/report.json",
                                            "{}", error));
  EXPECT_FALSE(error.empty());
}

// The command-line binary is exercised as well as the library behind it,
// because the exit code is the part a build agent reads and only the
// executable produces it.
#if defined(AME_MISSION_CLI_PATH)
TEST(MissionCli, ExitCodeIsTheVerdict) {
  ProjectModel model = makeProject();
  model.scenarios.pop_back();
  const fs::path project = saveProject(model, "exit-good");
  const std::string cli = ame_test::shellQuote(AME_MISSION_CLI_PATH);
  const std::string good = ame_test::shellCommand(
      cli + " run " + ame_test::shellQuote(project) +
      " --scenario nominal" + ame_test::discardOutput());
  EXPECT_EQ(std::system(good.c_str()), 0);
  fs::remove(project);

  const fs::path both = saveProject(makeProject(), "exit-bad");
  const std::string bad = ame_test::shellCommand(
      cli + " run " + ame_test::shellQuote(both) +
      " --scenario impossible" + ame_test::discardOutput());
  EXPECT_NE(std::system(bad.c_str()), 0);

  const std::string unknown = ame_test::shellCommand(
      cli + " frobnicate " + ame_test::shellQuote(both) +
      ame_test::discardOutput());
  EXPECT_NE(std::system(unknown.c_str()), 0);

  const std::string help =
      ame_test::shellCommand(cli + " --help" + ame_test::discardOutput());
  EXPECT_EQ(std::system(help.c_str()), 0);
  fs::remove(both);
}
#endif
