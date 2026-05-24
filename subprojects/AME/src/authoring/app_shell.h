#pragma once

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
  bool showAboutModal = false;
};
