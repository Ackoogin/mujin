#pragma once

#include "domain_graph_panel.h"
#include "type_hierarchy_panel.h"

#include <string>

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

  // --- Test interface (used by --self-test mode only) ---
  void selfTestNew();
  void selfTestAddPredicate(const std::string& name);
  void selfTestAddType(const std::string& name, const std::string& parent = "object");
  const ProjectModel& selfTestModel() const { return m_model; }

private:
  DomainGraphPanel m_domainGraph;
  ProjectModel m_model;
  TypeHierarchyPanel m_typeHierarchy;
  bool showAboutModal = false;
};
