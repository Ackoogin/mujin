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

private:
  DomainGraphPanel m_domainGraph;
  ProjectModel m_model;
  TypeHierarchyPanel m_typeHierarchy;
  bool showAboutModal = false;
};
