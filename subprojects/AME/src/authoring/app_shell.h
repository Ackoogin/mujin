#pragma once

#include "command_stack.h"
#include "domain_graph_panel.h"
#include "type_hierarchy_panel.h"

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
  bool selfTestAddCausalLink(int fromAction,
                             int fromAddEffectIdx,
                             int toAction,
                             int toPreconditionIdx);
  bool selfTestUndo();
  bool selfTestRedo();
  size_t selfTestUndoDepth() const;
  const ProjectModel& selfTestModel() const { return m_model; }

private:
  void renderDomainTab();
  void renderPddlTab();
  void renderPlanTab();
  void renderBtTab();
  void renderSelectedElementEditor();

  CommandStack m_commandStack;
  DomainGraphPanel m_domainGraph;
  ProjectModel m_model;
  TypeHierarchyPanel m_typeHierarchy;
  bool showAboutModal = false;
};
