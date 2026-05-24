#pragma once

#include <imgui_node_editor.h>

namespace ed = ax::NodeEditor;

class DomainGraphPanel {
public:
  DomainGraphPanel();
  ~DomainGraphPanel();

  void render();

private:
  ed::EditorContext* m_context = nullptr;
};
