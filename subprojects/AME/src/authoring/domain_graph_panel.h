#pragma once

#include <imgui_node_editor.h>
#include "command_stack.h"
#include "project_model.h"

#include <string>
#include <vector>

namespace ed = ax::NodeEditor;

class DomainGraphPanel {
public:
  DomainGraphPanel();
  ~DomainGraphPanel();

  void render(ProjectModel& model, CommandStack& stack);
  void setHighlightedElements(std::vector<std::string> predicateNames,
                              std::vector<std::string> actionNames);
  void setStructuralHighlights(std::vector<std::string> errPreds,
                               std::vector<std::string> errActs,
                               std::vector<std::string> warnPreds,
                               std::vector<std::string> warnActs);
  int selectedPredicateIndex() const { return m_selectedPredIdx; }
  int selectedActionIndex() const { return m_selectedActionIdx; }

private:
  ed::EditorContext* m_context = nullptr;
  int m_selectedPredIdx = -1;
  int m_selectedActionIdx = -1;
  bool m_openAddPredicatePopup = false;
  bool m_openAddActionPopup = false;
  char m_newPredName[64] = {};
  char m_newActionName[64] = {};
  std::vector<std::string> m_highlightedPredicates;
  std::vector<std::string> m_highlightedActions;
  std::vector<std::string> m_structuralErrorPredicates;
  std::vector<std::string> m_structuralErrorActions;
  std::vector<std::string> m_structuralWarningPredicates;
  std::vector<std::string> m_structuralWarningActions;
};
