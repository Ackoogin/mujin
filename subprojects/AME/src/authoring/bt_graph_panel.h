#pragma once

#include "simulation_engine.h"

#include <imgui_node_editor.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ed = ax::NodeEditor;

/// \brief The authored action contract carried by one compiled tree node.
struct BtNodeDetail {
  std::string kind;
  std::string label;
  std::string action;
  std::vector<std::string> preconditions;
  std::vector<std::string> confirmedPreconditions;
  std::vector<std::string> negativePreconditions;
  std::vector<std::string> addEffects;
  std::vector<std::string> deleteEffects;
  bool isAction = false;
};

/// \brief Read-only view of a compiled BehaviorTree XML document.
class BtGraphPanel {
public:
  BtGraphPanel();
  ~BtGraphPanel();

  /// \brief Push a new BT XML string, or an empty string to clear.
  void setXml(const std::string& xml);

  /// \brief Colour the tree by how far a run has got.
  ///
  /// Steps are matched to nodes by the grounded signature the compiler writes
  /// as the node's name, so the two never have to agree on node ordering. A
  /// node that holds others takes its colour from them: it is happening while
  /// any of them is, finished once all of them are.
  void setRunProgress(const std::vector<RunActionStep>& steps);

  /// \brief Draw the tree without any run colouring.
  void clearRunProgress();

  /// \brief Render inside the caller's ImGui::BeginChild scope.
  void render();

  /// \brief Last parse error, empty when the XML was accepted.
  const std::string& lastError() const { return m_lastError; }

  /// \brief Number of BT nodes in the parsed tree, including collapsed nodes.
  size_t nodeCount() const;

  /// \brief Collapse every BT node that has children.
  void collapseAll();

  /// \brief Expand every BT node.
  void expandAll();

  /// \brief Index of the currently selected BT node, or -1 if none.
  int selectedNodeIndex() const { return m_selectedNode; }

  /// \brief Read the action contract parsed from a tree node's XML attributes.
  bool nodeDetail(int nodeIndex, BtNodeDetail& detail) const;

  /// \brief Whether any parsed node carries an authored action contract.
  bool hasActionContract() const;

private:
  struct BtNode {
    std::string kind;
    std::string label;
    std::vector<std::pair<std::string, std::string>> attributes;
    std::vector<int> children;
    int parent = -1;
    int depth = 0;
    int siblingIdx = 0;
    bool collapsed = false;
    bool visible = true;
  };

  std::vector<BtNode> m_nodes;
  std::string m_lastError;
  std::unordered_map<std::string, RunNodeStatus> m_runStatusByName;
  bool m_hasRunProgress = false;
  ed::EditorContext* m_context = nullptr;
  int m_selectedNode = -1;
  bool m_layoutDone = false;

  void updateVisibility();
  void applyLayout();
  int selectedNodeFromEditor() const;
  /// \brief The run status to draw a node with, or nothing when a node is not
  /// part of a run and holds nothing that is.
  bool runStatusOf(int nodeIndex, RunNodeStatus& status) const;
};
