#pragma once

#include <imgui_node_editor.h>
#include "project_model.h"

namespace ed = ax::NodeEditor;

class DomainGraphPanel {
public:
  DomainGraphPanel();
  ~DomainGraphPanel();

  void render(ProjectModel& model);
  int selectedPredicateIndex() const { return m_selectedPredIdx; }

private:
  ed::EditorContext* m_context = nullptr;
  int m_selectedPredIdx = -1;
  bool m_openAddPredicatePopup = false;
  char m_newPredName[64] = {};
};
