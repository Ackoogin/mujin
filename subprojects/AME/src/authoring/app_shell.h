#pragma once

#include "bt_graph_panel.h"
#include "command_stack.h"
#include "contingency_analyser.h"
#include "domain_graph_panel.h"
#include "pddl_validator.h"
#include "plan_graph_panel.h"
#include "scenario_runner.h"
#include "structural_validator.h"
#include "type_hierarchy_panel.h"

#include <ame/planner.h>

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
  void selfTestRunContingencyAnalysis();
  bool selfTestAddCausalLink(int fromAction,
                             int fromAddEffectIdx,
                             int toAction,
                             int toPreconditionIdx);
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
  const StructuralReport& selfTestStructuralReport() const { return m_structuralReport; }

private:
  void renderDomainTab();
  void renderPddlTab();
  void renderPlanTab();
  void renderBtTab();
  void renderSelectedElementEditor();
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
  ContingencyReport m_lastContingencyReport;
  ame::PlanResult m_lastPlan;
  ProjectModel m_model;
  TypeHierarchyPanel m_typeHierarchy;
  std::vector<std::string> m_lastPlanStepLabels;
  std::string m_lastPlanScenarioName;
  std::string m_validationScenario;
  std::string m_requestedTab;
  int m_selectedScenarioIdx = -1;
  bool m_autoValidateOnSave = true;
  bool m_hasLastPlan = false;
  bool showAboutModal = false;
};
