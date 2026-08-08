#pragma once

#include <string>

/// \brief What a command did, and what it concluded.
///
/// `ran` says the command itself worked: the project opened, the scenario
/// existed, the folder could be written. `verdict` says the answer was a good
/// one: the scenarios behaved as they were expected to. They are separate
/// because a command that runs perfectly and reports a mission that does not
/// work has done its job, and the two failures need different responses from
/// whoever is reading.
struct MissionCommandResult {
  bool ran = false;
  bool verdict = false;
  std::string message;      // for a person, one or more lines, no trailing newline
  std::string reportJson;   // for a machine, empty when the command produces none
};

/// \brief The work behind the mission command line.
///
/// These live in the library rather than in the executable so that the command
/// line and the test suite exercise the same code, and so that neither needs a
/// window to run. Nothing here prints: the caller decides where the message and
/// the report go.
class MissionCommands {
public:
  /// \brief Simulate one scenario and report what happened.
  static MissionCommandResult runScenario(const std::string& projectPath,
                                          const std::string& scenarioName);

  /// \brief Simulate every scenario against its expectations.
  static MissionCommandResult runBatch(const std::string& projectPath);

  /// \brief Simulate one scenario and write its recorded run to a folder.
  static MissionCommandResult recordScenario(const std::string& projectPath,
                                             const std::string& scenarioName,
                                             const std::string& outFolder);

  /// \brief Generate the assurance evidence for a project.
  ///
  /// The verdict is good when nothing the evidence covers is outstanding: the
  /// scenarios behaved, and no declared contingency leaves the mission with no
  /// way back to a safe state.
  static MissionCommandResult assuranceEvidence(const std::string& projectPath);

  /// \brief Write a report to a file.
  /// \return True when it was written; the reason is left in `error` if not.
  static bool writeReport(const std::string& path,
                          const std::string& json,
                          std::string& error);
};
