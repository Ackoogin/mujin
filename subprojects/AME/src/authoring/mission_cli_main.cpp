// =========================================================================
// ame_mission_cli — run a mission model without a window
// =========================================================================
// Simulates a project's scenarios from a script: run one, record one to a
// folder of replay files, or run the whole set against its expectations.
//
// The conventions match contingency_verifier, which is the other command-line
// tool in this project: the file being examined is a positional argument,
// --json names the machine-readable report, --help explains itself, and the
// exit code is the verdict rather than merely a sign that the process ran.
// Somebody scripting these tools should have to learn that once.
//
// Usage:
//   ame_mission_cli run    <project.ameproj.json> --scenario <name> [--json <file>]
//   ame_mission_cli record <project.ameproj.json> --scenario <name> --out <folder>
//   ame_mission_cli batch  <project.ameproj.json> [--json <file>]
// =========================================================================

#include "mission_commands.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace {

void printUsage(const char* program) {
  std::printf(
      "ame_mission_cli — run a mission model without a window\n"
      "\n"
      "Usage:\n"
      "  %s run    <project.ameproj.json> --scenario <name> [--json <file>]\n"
      "  %s record <project.ameproj.json> --scenario <name> --out <folder>\n"
      "  %s batch  <project.ameproj.json> [--json <file>]\n"
      "\n"
      "Commands:\n"
      "  run       Simulate one scenario and report what happened.\n"
      "  record    Simulate one scenario and write a folder of replay files,\n"
      "            which the AME DevEnv and this project's authoring tool both\n"
      "            open.\n"
      "  batch     Simulate every scenario and check each against the planning\n"
      "            and execution expectations the project records for it.\n"
      "  evidence  Write the assurance evidence for the model: what has been\n"
      "            checked about it, and what has not. The report is Markdown,\n"
      "            so --json names a .md file.\n"
      "\n"
      "Options:\n"
      "  --scenario <name>  Which scenario to run. Required by run and record.\n"
      "  --json <file>      Write the machine-readable report to this file.\n"
      "                     Without it, no report is written.\n"
      "  --out <folder>     Where record writes the run. Required by record.\n"
      "  -h, --help         Show this help.\n"
      "\n"
      "Exit code: 0 when the mission behaved as the project expects, 1 when it\n"
      "did not or the command could not be carried out.\n"
      "\n"
      "Every run this tool produces is a simulation: the planner, the world\n"
      "model and the compiled behaviour tree are the real ones, but each action\n"
      "is a stand-in. A run is evidence about a mission model, never about how\n"
      "the system will behave in the field.\n",
      program, program, program, program);
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string command = argv[1];
  if (command == "--help" || command == "-h") {
    printUsage(argv[0]);
    return 0;
  }
  if (command != "run" && command != "record" && command != "batch" &&
      command != "evidence") {
    std::printf("Unknown command: %s\n\n", command.c_str());
    printUsage(argv[0]);
    return 1;
  }

  std::string projectPath;
  std::string scenarioName;
  std::string jsonPath;
  std::string outFolder;

  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scenario" && i + 1 < argc) {
      scenarioName = argv[++i];
    } else if (arg == "--json" && i + 1 < argc) {
      jsonPath = argv[++i];
    } else if (arg == "--out" && i + 1 < argc) {
      outFolder = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else if (!arg.empty() && arg[0] != '-') {
      if (projectPath.empty()) {
        projectPath = arg;
      } else {
        std::printf("Unexpected argument: %s\n\n", arg.c_str());
        printUsage(argv[0]);
        return 1;
      }
    } else {
      std::printf("Unknown option: %s\n\n", arg.c_str());
      printUsage(argv[0]);
      return 1;
    }
  }

  if (projectPath.empty()) {
    std::printf("A project file is needed.\n\n");
    printUsage(argv[0]);
    return 1;
  }

  MissionCommandResult result;
  if (command == "run") {
    result = MissionCommands::runScenario(projectPath, scenarioName);
  } else if (command == "record") {
    result = MissionCommands::recordScenario(projectPath, scenarioName,
                                             outFolder);
  } else if (command == "evidence") {
    result = MissionCommands::assuranceEvidence(projectPath);
  } else {
    result = MissionCommands::runBatch(projectPath);
  }

  if (!result.message.empty()) {
    std::printf("%s\n", result.message.c_str());
  }
  if (!result.ran) {
    return 1;
  }

  // The report is written only when it is asked for, as the contingency
  // verifier does. Standard output carries the summary a person reads, and the
  // planner writes progress lines of its own there, so it is no place for a
  // document something else has to parse.
  if (!jsonPath.empty() && !result.reportJson.empty()) {
    std::string error;
    if (!MissionCommands::writeReport(jsonPath, result.reportJson, error)) {
      std::printf("%s\n", error.c_str());
      return 1;
    }
  }

  std::fflush(stdout);
  return result.verdict ? 0 : 1;
}
