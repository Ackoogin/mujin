#pragma once

#include "project_model.h"
#include "simulation_engine.h"

#include <map>
#include <string>
#include <vector>

/// \brief Describes where a recorded run came from.
struct RunManifest {
  int version = 1;
  std::string project;
  std::string scenario;
  unsigned seed = 0;
  RunFaultSet faults;
  bool simulated = false;
  /// Names whether timestamps are measured or derived from simulation ticks.
  std::string timeBasis;
  /// Seconds represented by one simulation tick, or zero for a real run.
  double tickPeriodSeconds = 0.0;
};

/// \brief A run loaded from the three runtime-compatible observability files.
///
/// The loader does not need a project. It can therefore open recordings made
/// by a deployed runtime as well as recordings made by the authoring tool.
class RecordedRun {
public:
  /// \brief Capture the current simulation without saving it first.
  static RecordedRun fromSimulation(const ProjectModel& model,
                                    const SimulationEngine& simulation);

  /// \brief Load a folder containing the three DevEnv replay files.
  bool load(const std::string& folder);

  /// \brief Write the manifest and the three DevEnv replay files.
  bool save(const std::string& folder) const;

  bool loaded() const { return loaded_; }
  const std::string& errorMessage() const { return error_; }
  const RunManifest& manifest() const { return manifest_; }
  bool simulated() const { return manifest_.simulated; }
  unsigned tick() const { return max_tick_; }
  const std::string& folder() const { return folder_; }

  RunState stateAtTick(unsigned tick) const;
  std::vector<RunGoal> goalsAtTick(unsigned tick) const;
  std::map<std::string, std::string> treeStateAtTick(unsigned tick) const;
  const std::vector<RunNode>& nodes() const { return nodes_; }
  const std::vector<RunActionStep>& actionSteps() const { return action_steps_; }
  const std::vector<RunFactChange>& factChanges() const { return fact_changes_; }
  const std::string& compiledXml() const { return compiled_xml_; }
  std::string summaryLine() const;

private:
  struct BtEvent {
    uint64_t tsUs = 0;
    unsigned tick = 0;
    std::string node;
    std::string type;
    std::string previous;
    std::string status;
    std::string treeId;
    uint64_t wmVersion = 0;
  };
  struct WmEvent {
    uint64_t wmVersion = 0;
    uint64_t tsUs = 0;
    unsigned tick = 0;
    std::string fact;
    bool value = false;
    std::string source;
  };
  struct PlanEvent {
    uint64_t tsUs = 0;
    unsigned tick = 0;
    std::vector<std::string> initFacts;
    std::vector<std::string> goals;
    std::vector<std::string> actions;
    std::string btXml;
    std::string json;
  };

  bool parseEvents(const std::vector<RunJsonEvent>& bt,
                   const std::vector<RunJsonEvent>& wm,
                   const std::vector<RunJsonEvent>& plan);
  void addMissingInitialWorldEvents();
  void useSimulatedTickTimestamps();
  void deriveViews();

  RunManifest manifest_;
  std::vector<BtEvent> bt_events_;
  std::vector<WmEvent> wm_events_;
  std::vector<PlanEvent> plan_events_;
  std::vector<RunNode> nodes_;
  std::vector<RunActionStep> action_steps_;
  std::vector<RunFactChange> fact_changes_;
  std::vector<std::string> initial_facts_;
  std::vector<std::string> goal_facts_;
  std::string compiled_xml_;
  std::string folder_;
  std::string error_;
  unsigned max_tick_ = 0;
  bool loaded_ = false;
};

/// \brief The differences a reviewer needs when comparing two runs.
struct RunComparison {
  bool treesDiffer = false;
  unsigned firstDifferentTick = 0;
  std::vector<std::string> actionsOnlyInFirst;
  std::vector<std::string> actionsOnlyInSecond;
  std::vector<std::string> endFactDifferences;
  std::string summary;
};

/// \brief Compare two captured or loaded runs.
RunComparison compareRuns(const RecordedRun& first,
                          const RecordedRun& second);
