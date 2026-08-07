#pragma once

#include "bt_graph_panel.h"
#include "command_stack.h"
#include "contingency_analyser.h"
#include "domain_graph_panel.h"
#include "fact_action_matrix.h"
#include "failure_explainer.h"
#include "lifecycle_model.h"
#include "pddl_validator.h"
#include "plan_graph_panel.h"
#include "import_merge.h"
#include "model_edits.h"
#include "problem_list.h"
#include "recent_projects.h"
#include "scenario_runner.h"
#include "simulation_engine.h"
#include "structural_validator.h"
#include "relation_index.h"
#include "review_pack.h"
#include "run_record.h"
#include "type_hierarchy_panel.h"

#include <ame/planner.h>

#include <array>
#include <string>
#include <vector>

/// \brief Main application shell for the AME graphical authoring tool.
class AppShell {
public:
  AppShell();

  std::string projectName;
  std::string validationState;
  std::string lastOperation;
  bool wantsQuit = false;

  void renderMenuBar();
  void renderPanels();
  void renderStatusBar();

  /// Workflow-ordered tab labels. Stable list — self-tests assert against it.
  static const std::vector<std::string>& tabLabels();

  // --- Test interface (used by --self-test mode only) ---
  void selfTestNew();
  void selfTestAddPredicate(const std::string& name);
  void selfTestAddPredicateParam(int predicateIdx,
                                 const std::string& name,
                                 const std::string& type);
  void selfTestAddType(const std::string& name, const std::string& parent = "object");
  void selfTestAddAction(const std::string& name);
  void selfTestAddActionParam(int actionIdx,
                              const std::string& name,
                              const std::string& type);
  void selfTestAddActionPrecondition(int actionIdx,
                                     const std::string& predName,
                                     std::vector<std::string> argNames);
  void selfTestAddActionAddEffect(int actionIdx,
                                  const std::string& predName,
                                  std::vector<std::string> argNames);
  void selfTestAddActionDelEffect(int actionIdx,
                                  const std::string& predName,
                                  std::vector<std::string> argNames);
  void selfTestSetActionBtBinding(int actionIdx,
                                  std::string nodeType,
                                  std::string subtreeXml,
                                  bool reactive);
  void selfTestAddObject(const std::string& name, const std::string& type);
  void selfTestAddScenario(const std::string& name);
  void selfTestAddInitialFact(int scenarioIdx,
                              const std::string& predicateName,
                              std::vector<std::string> objectNames);
  void selfTestAddGoal(int scenarioIdx,
                       const std::string& predicateName,
                       std::vector<std::string> objectNames);
  void selfTestSetScenarioExpectation(int scenarioIdx,
                                      bool shouldSucceed,
                                      int minSteps,
                                      int maxSteps,
                                      std::vector<std::string> expected,
                                      std::vector<std::string> forbidden);
  const ScenarioExpectation& selfTestScenarioExpectation(int scenarioIdx) const;
  void selfTestRunFeasibility(const std::string& scenarioName);
  void selfTestRunAllScenarios();
  void selfTestStartBatch();
  void selfTestStopBatch();
  bool selfTestBatchRunning() const { return m_batchRunner.isRunning(); }
  size_t selfTestBatchCompletedCount() const {
    return m_batchRunner.completedCount();
  }
  void selfTestRunContingencyAnalysis();
  bool selfTestUndo();
  bool selfTestRedo();
  void selfTestValidate();
  void selfTestCorruptPredicateName(int idx);
  void selfTestRemoveAllObjects();
  void selfTestPlanAndPreview();
  void selfTestSetSelectedPlanStep(int idx);
  bool selfTestImportDomain(const std::string& pddl);
  bool selfTestImportProblem(const std::string& pddl,
                             const std::string& scenarioName);
  size_t selfTestUndoDepth() const;
  const BtBinding& selfTestActionBtBinding(int actionIdx) const;
  const ProjectModel& selfTestModel() const { return m_model; }
  const ValidationReport& selfTestValidation() const { return m_lastValidation; }
  const ame::PlanResult& selfTestLastPlan() const { return m_lastPlan; }
  const ScenarioBatchReport& selfTestBatchReport() const { return m_lastBatchReport; }
  const ContingencyReport& selfTestContingencyReport() const {
    return m_lastContingencyReport;
  }
  const PlanGraphPanel& selfTestPlanGraph() const { return m_planGraph; }
  size_t selfTestPlanGraphStepCount() const { return m_planGraph.stepCount(); }
  size_t selfTestBtNodeCount() const { return m_btGraph.nodeCount(); }
  bool selfTestBtHasActionContract() const {
    return m_btGraph.hasActionContract();
  }
  const StructuralReport& selfTestStructuralReport() const { return m_structuralReport; }
  RelationIndex selfTestRelationIndex() const { return RelationIndex(m_model); }
  size_t selfTestNeighbourhoodNodeCount(int depth = 1) const;
  std::string selfTestMatrixCsv() const;
  size_t selfTestLifecycleTransitionCount() const;
  void selfTestAddStateGroup(std::string name,
                             std::string type,
                             std::vector<std::string> predicates);
  const FailureExplanation& selfTestFailureExplanation() const {
    return m_lastFailureExplanation;
  }
  bool selfTestSelectionBack();
  bool selfTestSelectionForward();
  /// Starts a simulated run of the named scenario, as the Run tab does.
  bool selfTestStartRun(const std::string& scenarioName);
  bool selfTestStepRun();
  bool selfTestRunToCompletion();
  const SimulationEngine& selfTestSimulation() const { return m_simulation; }
  /// Loads a saved project file, as File > Open does. Used by the offscreen
  /// self-test to exercise the views against a real domain rather than the
  /// small model the rest of the self-test builds by hand.
  bool selfTestLoadProject(const std::string& path);
  /// Selects the predicate with the most relationships, which is the case most
  /// likely to expose layout and identity problems in the neighbourhood view.
  bool selfTestFocusBusiestPredicate();
  float selfTestWidestNeighbourItemWidth() const {
    return m_domainGraph.widestNeighbourItemWidth();
  }
  const std::vector<std::string>& selfTestNeighbourItemIds() const {
    return m_domainGraph.neighbourItemIds();
  }
  void selfTestSetDomainView(int view);
  void selfTestShowPlanTab() { m_requestedTab = "Plan"; }
  void selfTestShowRunTab() { m_requestedTab = "Run"; }
  void selfTestShowPddlTab() { m_requestedTab = "PDDL"; }
  bool selfTestDomainViewRendered(int view) const;
  bool selfTestGuidedEditorRendered() const { return m_guidedEditorRendered; }
  bool selfTestRunTimelineRendered() const { return m_runTimelineRendered; }
  bool selfTestRunFactsRendered() const { return m_runFactsRendered; }
  bool selfTestProblemListRendered() const { return m_problemListRendered; }
  /// Saves the current picture under a name, as the Save this view button does.
  void selfTestSaveCurrentView(const std::string& name);
  bool selfTestOpenSavedView(const std::string& name);
  bool selfTestHasUnsavedChanges() const { return m_unsavedChanges; }
  bool selfTestRecoveryCopyWritten() const { return m_recoveryWritten; }
  const std::vector<std::string>& selfTestRecentProjects() const {
    return m_recentProjects;
  }
  std::string selfTestUndoLabel() const { return m_commandStack.topUndoLabel(); }
  void selfTestCopySelection() { copySelection(); }
  void selfTestPasteClipboard() { pasteClipboard(); }
  /// Clicks the first problem that names an element, as a reader would.
  bool selfTestClickFirstProblem();
  void selfTestSetRunFaults(RunFaultSet faults) {
    m_runFaults = std::move(faults);
    m_simulation.setFaults(m_runFaults);
  }
  bool selfTestRunFaultPanelRendered() const {
    return m_runFaultPanelRendered;
  }
  bool selfTestReplanComparisonRendered() const {
    return m_replanComparisonRendered;
  }
  bool selfTestBatchProgressRendered() const {
    return m_batchProgressRendered;
  }
  void selfTestReplayCurrentRun();
  void selfTestCloseReplay() { m_replay = RecordedRun{}; }
  void selfTestCompareCurrentRunWithItself();
  bool selfTestReplayRendered() const { return m_replayRendered; }
  bool selfTestRunComparisonRendered() const {
    return m_runComparisonRendered;
  }

private:
  void renderDomainTab();
  void renderPddlTab();
  void renderPlanTab();
  void renderRunTab();
  void revealProblemTarget(const ProblemEntry& problem);
  bool openProjectFrom(const std::string& path);
  void noteProjectOpened(const std::string& path);
  void noteProjectSaved(const std::string& path);
  void writeRecoveryCopy();
  void applySavedView(const SavedView& view);
  void copySelection();
  void pasteClipboard();
  void renderReplayRunTab();
  void renderRunComparison();
  void startRun();
  void previewRunFaults();
  void renderSelectedElementEditor();
  void renderRelationsPanel();
  void renderMatrixPanel();
  void renderLifecyclePanel();
  void clearDerivedResults();
  void resetScenarioEditorState();
  void runValidation();
  void runFeasibilityCheck();
  void runAllScenarios();
  void runContingencyAnalysis();
  void runPlanAndPreview();
  void compileAndShowBt();

  CommandStack m_commandStack;
  DomainGraphPanel m_domainGraph;
  PlanGraphPanel m_planGraph;
  BtGraphPanel m_btGraph;
  ValidationReport m_lastValidation;
  StructuralReport m_structuralReport;
  ScenarioBatchReport m_lastBatchReport;
  ScenarioRunner m_batchRunner;
  ContingencyReport m_lastContingencyReport;
  FailureExplanation m_lastFailureExplanation;
  ame::PlanResult m_lastPlan;
  SimulationEngine m_simulation;
  RecordedRun m_replay;
  RecordedRun m_comparisonFirst;
  RecordedRun m_comparisonSecond;
  RunComparison m_runComparison;
  ProjectModel m_model;
  TypeHierarchyPanel m_typeHierarchy;
  std::vector<std::string> m_lastPlanStepLabels;
  std::string m_lastPlanScenarioName;
  std::string m_validationScenario;
  std::string m_requestedTab;
  int m_selectedScenarioIdx = -1;
  bool m_autoValidateOnSave = true;
  bool m_showRawDiagnostics = false;
  std::string m_projectPath;
  std::string m_settingsPath;
  std::vector<std::string> m_recentProjects;
  ElementClipboard m_clipboard;
  ProjectModel m_incomingModel;
  MergePlan m_mergePlan;
  MergeChoices m_mergeChoices;
  bool m_showMergeDialog = false;
  bool m_showSaveViewDialog = false;
  char m_saveViewNameInput[64] = {};
  bool m_unsavedChanges = false;
  bool m_recoveryWritten = false;
  bool m_askBeforeQuitting = false;
  double m_secondsSinceRecoveryCopy = 0.0;
  size_t m_lastSeenUndoDepth = 0;
  bool m_problemListRendered = false;
  bool m_hasLastPlan = false;
  bool showAboutModal = false;
  int m_domainViewMode = 0;
  int m_neighbourDepth = 1;
  uint32_t m_neighbourFilter = ShowEverything;
  std::array<bool, 5> m_domainViewsRendered{};
  bool m_guidedEditorRendered = false;
  bool m_runTimelineRendered = false;
  bool m_runFactsRendered = false;
  bool m_runFaultPanelRendered = false;
  bool m_replanComparisonRendered = false;
  bool m_batchProgressRendered = false;
  bool m_replayRendered = false;
  bool m_runComparisonRendered = false;
  bool m_runViewingHistory = false;
  unsigned m_runViewTick = 0;
  size_t m_runDisplayedReplans = 0;
  char m_runFactFilter[128] = {};
  RunFaultSet m_runFaults;
  std::vector<std::string> m_runFactChoices;
  std::string m_runFaultChoiceScenario;
  int m_runFaultActionIdx = 0;
  int m_runFaultAttempt = 1;
  int m_runFaultFactIdx = 0;
  int m_runFaultTick = 1;
  bool m_runFaultFactValue = true;
  char m_runFaultName[64] = {};

  char m_scenarioNameInput[64] = {};
  char m_renameScenarioNameInput[64] = {};
  int m_renameScenarioIdx = -2;
  int m_initPredIdx = 0;
  int m_goalPredIdx = 0;
  std::vector<std::string> m_initChosenObjects;
  std::vector<std::string> m_goalChosenObjects;
  char m_initArgsInput[128] = {};
  char m_goalArgsInput[128] = {};
  char m_stateGroupNameInput[64] = {};
  char m_stateGroupTypeInput[64] = {};
  char m_stateGroupPredicatesInput[256] = {};
  std::array<char, 65536> m_pddlEditorBuffer{};
  bool m_pddlEditorInitialised = false;
  bool m_pddlEditorDirty = false;
  std::string m_renameScenarioSource;

  // Palette quick-add state (WI-5.2)
  bool m_paletteQuickAddOpen = false;
  int  m_paletteQuickAddKind = 0;             // 0 = Predicate, 1 = Action
  char m_paletteQuickAddName[64] = {};

public:
  // Palette test helpers
  void selfTestSelectPredicateFromPalette(int idx) { m_domainGraph.setSelectedPredicate(idx); }
  void selfTestSelectActionFromPalette(int idx) { m_domainGraph.setSelectedAction(idx); }
  int  selfTestSelectedPredicateIndex() const { return m_domainGraph.selectedPredicateIndex(); }
  int  selfTestSelectedActionIndex() const { return m_domainGraph.selectedActionIndex(); }
};
