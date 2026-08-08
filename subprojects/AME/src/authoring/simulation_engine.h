#pragma once

#include "failure_explainer.h"
#include "project_model.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace ame {
struct PlanResult;
}

/// \brief How far a simulated run has got.
///
/// `Error` means the run could not be set up or could not continue: the
/// scenario would not validate, no plan exists, the compiled tree would not
/// build, or the run passed its tick limit. `Failed` means the mission itself
/// went wrong, which is a result rather than a defect.
enum class RunPhase { NotStarted, Running, Held, Completed, Failed, Error };

/// \brief Where one part of the running tree has got to.
///
/// The four values are the four the rest of the tool already colours: waiting,
/// happening now, finished, and went wrong.
enum class RunNodeStatus { Waiting, Happening, Finished, WentWrong };

/// \brief One node of the compiled tree, in the order the tree draws it.
struct RunNode {
  std::string name;
  std::string nodeType;
  int depth = 0;
  bool isAction = false;
  RunNodeStatus status = RunNodeStatus::Waiting;
};

/// \brief One of the plan's actions, and when the run reached it.
struct RunActionStep {
  std::string signature;
  std::string actionName;
  unsigned startTick = 0;
  unsigned endTick = 0;
  int configuredTicks = 1;
  RunNodeStatus status = RunNodeStatus::Waiting;
};

/// \brief One change a run made to a world fact, with the tick it happened on.
struct RunFactChange {
  unsigned tick = 0;
  std::string fact;
  bool value = false;
  std::string source;
};

/// \brief One world-model fact as it appeared at a selected run tick.
struct RunFactState {
  std::string fact;
  bool value = false;
  unsigned lastChangedTick = 0;
  bool changedDuringRun = false;
};

/// \brief Reconstructed facts and action progress at one run tick.
///
/// This is rebuilt from the scenario's starting facts, the recorded fact
/// changes, and the recorded action spans. It is not a stored snapshot.
struct RunState {
  unsigned tick = 0;
  std::vector<RunFactState> facts;
  std::vector<RunActionStep> actionSteps;
};

/// \brief One of the scenario's goals, and whether it is true yet.
struct RunGoal {
  std::string fact;
  bool met = false;
};

/// \brief What one configured fault is expected to affect before a run starts.
struct FaultEffectExplanation {
  std::string faultName;
  std::string summary;
  std::string affectedAction;
  std::string lostPrecondition;
  std::vector<std::string> respondingActions;
};

/// \brief The plans on either side of one failed execution step.
struct ReplanEvent {
  unsigned tick = 0;
  std::string reason;
  std::vector<std::string> abandonedPlan;
  std::vector<std::string> replacementPlan;
  bool replacementFound = false;
  FailureExplanation failureExplanation;
};

/// \brief One core observability JSON event and the simulation tick that made it.
///
/// The JSON remains in the runtime schema. The tick is stored separately so a
/// recorded run can add it as an optional field without changing the core
/// logger or asking a replay viewer to infer simulation ticks from wall time.
struct RunJsonEvent {
  unsigned tick = 0;
  std::string json;
};

/// \brief Runs a mission model inside the tool, against a simulated world.
///
/// The run uses the project's real generated PDDL, the real world model, the
/// real planner and the real plan compiler. Only the action nodes are stand-ins:
/// every action in the compiled tree is built as a node that waits for the
/// number of ticks the project configured, then succeeds or fails as configured
/// and records the action's declared effects as believed facts. A run is
/// therefore evidence about the mission model, never about field behaviour.
///
/// The engine holds no user-interface state and does not draw anything, so the
/// same runs can be driven from the test suite and from the command line.
class SimulationEngine {
public:
  SimulationEngine();
  ~SimulationEngine();
  SimulationEngine(const SimulationEngine&) = delete;
  SimulationEngine& operator=(const SimulationEngine&) = delete;

  /// \brief Plan, compile and load a scenario, leaving the run ready at tick 0.
  /// \return True when the run is ready; false leaves errorMessage() set.
  bool start(const ProjectModel& model, const std::string& scenarioName);

  /// \brief Replace the run-local faults used by the next start or Reset.
  void setFaults(RunFaultSet faults);
  const RunFaultSet& faults() const { return m_faults; }

  /// \brief Plan, compile and load the same scenario again from the beginning.
  bool startAgain();

  /// \brief End the run where it stands, keeping what it did on screen.
  void stop();

  /// \brief Stop ticking without ending the run.
  void hold();

  /// \brief Resume ticking a held run.
  void resume();

  /// \brief Tick the tree exactly once.
  /// \return True while the run still has somewhere to go.
  bool stepOnce();

  /// \brief Tick until the run finishes or passes its tick limit.
  /// \return True when the mission reached its goals.
  bool runToCompletion();

  /// \brief Tick at the configured speed for the time that has passed.
  ///
  /// Called once per frame with the frame's elapsed seconds. A held or finished
  /// run ignores it.
  void advance(double elapsedSeconds);

  void setTicksPerSecond(double ticksPerSecond);
  double ticksPerSecond() const { return m_ticksPerSecond; }

  /// \brief Cap on the ticks one run may take before it is abandoned.
  void setTickLimit(unsigned ticks);
  unsigned tickLimit() const { return m_tickLimit; }

  RunPhase phase() const { return m_phase; }
  bool isFinished() const;
  bool isLoaded() const { return m_run != nullptr; }
  const std::string& errorMessage() const { return m_error; }
  const std::string& scenarioName() const { return m_scenarioName; }
  unsigned tick() const { return m_tick; }
  unsigned seed() const { return m_model.simulationSeed; }
  size_t replanCount() const { return m_replans.size(); }

  const std::vector<RunNode>& nodes() const { return m_nodes; }
  const std::vector<RunActionStep>& actionSteps() const { return m_actionSteps; }
  const std::vector<RunFactChange>& factChanges() const { return m_factChanges; }
  const std::string& compiledXml() const { return m_compiledXml; }
  const std::vector<FaultEffectExplanation>& faultExplanations() const {
    return m_faultExplanations;
  }
  const std::vector<ReplanEvent>& replans() const { return m_replans; }
  const std::vector<RunJsonEvent>& btAuditEvents() const {
    return m_btAuditEvents;
  }
  const std::vector<RunJsonEvent>& wmAuditEvents() const {
    return m_wmAuditEvents;
  }
  const std::vector<RunJsonEvent>& planAuditEvents() const {
    return m_planAuditEvents;
  }

  /// \brief Reconstruct how the run looked at a tick without changing it.
  ///
  /// Requests after the live tick are clamped to the live tick. An action is
  /// happening throughout its recorded start-to-finish span, then takes its
  /// recorded final status at its finish tick and on later ticks.
  RunState stateAtTick(unsigned tick) const;

  /// \brief The scenario's goals, with whether each one is true right now.
  std::vector<RunGoal> goals() const;
  size_t goalsMetCount() const;

  /// \brief Actions that are running on this tick, by their grounded signature.
  std::vector<std::string> happeningNow() const;
  size_t actionsFinishedCount() const;
  size_t actionsRunCount() const;
  std::vector<std::string> actionsRun() const;

  /// \brief One line naming what the run has done so far, for the status bar.
  std::string summaryLine() const;

  /// \brief The whole run as JSON, for the command-line runner and for tests.
  std::string toJson() const;

private:
  struct Run;

  bool prepare();
  bool buildTreeForPlan(const ame::PlanResult& plan, bool firstPlan);
  void recordPlanningEpisode(const ame::PlanResult& plan,
                             const std::string& btXml);
  bool replanAfterFailure();
  void applyFactFaultsForCurrentTick();
  void buildFaultExplanations();
  FailureExplanation explainCurrentFailure() const;
  void clearRunState();
  void recordStatusChange(size_t nodeIndex, int btStatus);

  std::unique_ptr<Run> m_run;
  ProjectModel m_model;
  std::string m_scenarioName;
  std::string m_error;
  std::string m_compiledXml;
  std::vector<RunNode> m_nodes;
  std::vector<RunActionStep> m_actionSteps;
  std::vector<RunFactChange> m_factChanges;
  RunFaultSet m_faults;
  std::vector<FaultEffectExplanation> m_faultExplanations;
  std::vector<ReplanEvent> m_replans;
  std::vector<RunJsonEvent> m_btAuditEvents;
  std::vector<RunJsonEvent> m_wmAuditEvents;
  std::vector<RunJsonEvent> m_planAuditEvents;
  std::vector<std::string> m_factNames;
  std::vector<std::string> m_startingFacts;
  RunPhase m_phase = RunPhase::NotStarted;
  unsigned m_tick = 0;
  unsigned m_tickLimit = 2000;
  double m_ticksPerSecond = 4.0;
  double m_tickAccumulator = 0.0;
};

/// \brief The word the user interface and the reports use for a phase.
const char* runPhaseName(RunPhase phase);

/// \brief The word the user interface and the reports use for a node status.
const char* runNodeStatusName(RunNodeStatus status);
