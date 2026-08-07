#pragma once

#include <imgui_node_editor.h>
#include "command_stack.h"
#include "neighbourhood_model.h"
#include "project_model.h"
#include "relation_index.h"

#include <string>
#include <vector>

namespace ed = ax::NodeEditor;

class DomainGraphPanel {
public:
  DomainGraphPanel();
  ~DomainGraphPanel();

  void render(ProjectModel& model, CommandStack& stack);
  void renderFocused(const ProjectModel& model,
                     const RelationIndex& index,
                     int depth,
                     uint32_t relationshipFilter,
                     size_t neighbourCap = 20);
  void setHighlightedElements(std::vector<std::string> predicateNames,
                              std::vector<std::string> actionNames);
  void setStructuralHighlights(std::vector<std::string> errPreds,
                               std::vector<std::string> errActs,
                               std::vector<std::string> warnPreds,
                               std::vector<std::string> warnActs);
  int selectedPredicateIndex() const { return m_selectedPredIdx; }
  int selectedActionIndex() const { return m_selectedActionIdx; }

  /// Width in pixels of the widest clickable name drawn by the last
  /// neighbourhood render. These items must stay as wide as their text; if one
  /// stretches to the width of the window it draws a highlight bar across the
  /// screen. Used by the offscreen self-test to catch that.
  float widestNeighbourItemWidth() const { return m_widestNeighbourItem; }

  /// The identity string given to each clickable name in the last
  /// neighbourhood render. These must all differ. Dear ImGui only notices two
  /// items sharing an identity while the mouse happens to be over one of them,
  /// which a headless run never does, so the self-test checks this list instead
  /// of relying on the library to report the problem.
  const std::vector<std::string>& neighbourItemIds() const {
    return m_neighbourItemIds;
  }

  /// Straight links are the default here. The three columns sit in a row, so a
  /// straight line between two of them is easy to follow; a curve leaves the
  /// node sideways and can pass under a neighbour on its way across.
  /// \brief Bring every node back into view on the next frame.
  void requestFitToContents() { m_fitToContents = true; }

  void setStraightLinks(bool straight) { m_straightLinks = straight; }
  bool straightLinks() const { return m_straightLinks; }

  /// Programmatic selection used by the palette / cross-view highlights.
  /// The next ed::GetSelectedNodes call inside render() will overwrite these
  /// when the user clicks the canvas — palette selection is one-shot.
  void setSelectedPredicate(int idx);
  void setSelectedAction(int idx);
  bool canGoBack() const { return m_historyPosition > 0; }
  bool canGoForward() const {
    return m_historyPosition + 1 < static_cast<int>(m_history.size());
  }
  void goBack();
  void goForward();
  const std::vector<DomainElementRef>& selectionHistory() const { return m_history; }
  int historyPosition() const { return m_historyPosition; }

private:
  ed::EditorContext* m_context = nullptr;
  int m_selectedPredIdx = -1;
  int m_selectedActionIdx = -1;
  bool m_openAddPredicatePopup = false;
  bool m_openAddActionPopup = false;
  bool m_openAddTypePopup = false;
  char m_newTypeName[64] = {};
  char m_newPredName[64] = {};
  char m_newActionName[64] = {};
  std::vector<std::string> m_highlightedPredicates;
  std::vector<std::string> m_highlightedActions;
  std::vector<std::string> m_structuralErrorPredicates;
  std::vector<std::string> m_structuralErrorActions;
  std::vector<std::string> m_structuralWarningPredicates;
  std::vector<std::string> m_structuralWarningActions;
  std::vector<DomainElementRef> m_history;
  int m_historyPosition = -1;
  bool m_morePopupOpen = false;
  /// Matches imgui-node-editor's own default link strength.
  static constexpr float kCurvedLinkStrength = 100.0F;

  bool m_straightLinks = true;
  bool m_fitToContents = false;
  float m_widestNeighbourItem = 0.0F;
  std::vector<std::string> m_neighbourItemIds;

  void select(DomainElementRef element, bool addToHistory);
};
