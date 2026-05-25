#pragma once

#include <imgui_node_editor.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ed = ax::NodeEditor;

/// \brief Read-only view of a compiled BehaviorTree XML document.
class BtGraphPanel {
public:
  BtGraphPanel();
  ~BtGraphPanel();

  /// \brief Push a new BT XML string, or an empty string to clear.
  void setXml(const std::string& xml);

  /// \brief Render inside the caller's ImGui::BeginChild scope.
  void render();

  /// \brief Last parse error, empty when the XML was accepted.
  const std::string& lastError() const { return m_lastError; }

  /// \brief Number of BT nodes currently displayed.
  size_t nodeCount() const;

  /// \brief Index of the currently selected BT node, or -1 if none.
  int selectedNodeIndex() const { return m_selectedNode; }

private:
  struct BtNode {
    std::string kind;
    std::string label;
    std::vector<std::pair<std::string, std::string>> attributes;
    std::vector<int> children;
    int parent = -1;
    int depth = 0;
    int siblingIdx = 0;
  };

  std::vector<BtNode> m_nodes;
  std::string m_lastError;
  ed::EditorContext* m_context = nullptr;
  int m_selectedNode = -1;
  bool m_layoutDone = false;
};
