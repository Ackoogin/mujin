#include "domain_graph_panel.h"

#include "authoring_utils.h"
#include "imgui.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

static ed::PinId actionPreconditionPinId(int actionIdx, int slotIdx) {
  return 4000 + actionIdx * 100 + slotIdx;
}

static ed::PinId actionEffectPinId(int actionIdx, int slotIdx) {
  return 5000 + actionIdx * 100 + slotIdx;
}

static bool containsName(const std::vector<std::string>& names,
                         const std::string& name) {
  return std::find(names.begin(), names.end(), name) != names.end();
}

DomainGraphPanel::DomainGraphPanel() {
  m_context = ed::CreateEditor(nullptr);
}

DomainGraphPanel::~DomainGraphPanel() {
  ed::DestroyEditor(m_context);
}

void DomainGraphPanel::select(DomainElementRef element, bool addToHistory) {
  if (element.kind == DomainElementKind::Predicate) {
    m_selectedPredIdx = static_cast<int>(element.index);
    m_selectedActionIdx = -1;
  } else if (element.kind == DomainElementKind::Action) {
    m_selectedActionIdx = static_cast<int>(element.index);
    m_selectedPredIdx = -1;
  } else {
    return;
  }

  if (!addToHistory) {
    return;
  }
  if (m_historyPosition >= 0 &&
      m_historyPosition < static_cast<int>(m_history.size())) {
    const DomainElementRef& current = m_history[static_cast<size_t>(m_historyPosition)];
    if (current.kind == element.kind && current.index == element.index) {
      return;
    }
  }
  if (m_historyPosition + 1 < static_cast<int>(m_history.size())) {
    m_history.erase(m_history.begin() + m_historyPosition + 1, m_history.end());
  }
  m_history.push_back(element);
  m_historyPosition = static_cast<int>(m_history.size()) - 1;
}

void DomainGraphPanel::setSelectedPredicate(int idx) {
  if (idx >= 0) {
    select({DomainElementKind::Predicate, static_cast<size_t>(idx)}, true);
  }
}

void DomainGraphPanel::setSelectedAction(int idx) {
  if (idx >= 0) {
    select({DomainElementKind::Action, static_cast<size_t>(idx)}, true);
  }
}

void DomainGraphPanel::goBack() {
  if (canGoBack()) {
    --m_historyPosition;
    select(m_history[static_cast<size_t>(m_historyPosition)], false);
  }
}

void DomainGraphPanel::goForward() {
  if (canGoForward()) {
    ++m_historyPosition;
    select(m_history[static_cast<size_t>(m_historyPosition)], false);
  }
}

void DomainGraphPanel::setHighlightedElements(std::vector<std::string> predicateNames,
                                              std::vector<std::string> actionNames) {
  m_highlightedPredicates = std::move(predicateNames);
  m_highlightedActions = std::move(actionNames);
}

void DomainGraphPanel::setStructuralHighlights(std::vector<std::string> errPreds,
                                               std::vector<std::string> errActs,
                                               std::vector<std::string> warnPreds,
                                               std::vector<std::string> warnActs) {
  m_structuralErrorPredicates = std::move(errPreds);
  m_structuralErrorActions = std::move(errActs);
  m_structuralWarningPredicates = std::move(warnPreds);
  m_structuralWarningActions = std::move(warnActs);
}

void DomainGraphPanel::render(ProjectModel& model, CommandStack& stack) {
  ed::SetCurrentEditor(m_context);
  ed::Begin("DomainGraphCanvas");
  const float zoom = ed::GetCurrentZoom();
  const bool showDetails = zoom >= 0.75F;
  const bool showCounts = zoom >= 0.45F;
  const RelationIndex relation_index(model);

  // ---- Predicate nodes (green) ----------------------------------------
  ed::PushStyleColor(ed::StyleColor_NodeBg,     ImVec4(0.05f, 0.28f, 0.10f, 1.0f));
  ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.18f, 0.65f, 0.25f, 0.9f));

  for (int i = 0; i < static_cast<int>(model.predicates.size()); ++i) {
    PredicateDef& pred = model.predicates[i];
    const ed::NodeId nodeId = 1000 + i;
    const ed::PinId outputPinId = 2000 + i;
    const ed::PinId inputPinId = 2100 + i;

    // Place unpositioned nodes in a row
    if (pred.posX == 0.0f && pred.posY == 0.0f) {
      ed::SetNodePosition(nodeId, ImVec2(60.0f + static_cast<float>(i) * 230.0f, 60.0f));
    }

    const bool errorHighlighted =
        containsName(m_highlightedPredicates, pred.name) ||
        containsName(m_structuralErrorPredicates, pred.name);
    const bool warningHighlighted =
        !errorHighlighted &&
        containsName(m_structuralWarningPredicates, pred.name);
    if (errorHighlighted) {
      ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(1.0f, 0.2f, 0.2f, 1.0f));
    } else if (warningHighlighted) {
      ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
    }
    ed::BeginNode(nodeId);

    if (showDetails) {
      ImGui::TextColored(ImVec4(0.25f, 0.90f, 0.40f, 1.0f), "[Fact]");
    }
    ImGui::Text("%s", pred.name.empty() ? "(unnamed)" : pred.name.c_str());
    if (showDetails) {
      for (const auto& p : pred.params) {
        ImGui::Text("  %s: %s", p.name.c_str(), p.type.c_str());
      }
    } else if (showCounts) {
      const PredicateRelations& relations = relation_index.predicate(static_cast<size_t>(i));
      ImGui::TextDisabled("%zu links", relations.requiredBy.size() +
                          relations.madeTrueBy.size() + relations.madeFalseBy.size());
    }

    ed::BeginPin(inputPinId, ed::PinKind::Input);
    ImGui::TextUnformatted(" ");
    ed::EndPin();
    ImGui::SameLine();
    ed::BeginPin(outputPinId, ed::PinKind::Output);
    ImGui::TextUnformatted(" ");
    ed::EndPin();

    ed::EndNode();
    if (errorHighlighted || warningHighlighted) {
      ed::PopStyleColor();
    }
  }

  ed::PopStyleColor(2);

  // ---- Action schema nodes (blue) -------------------------------------
  ed::PushStyleColor(ed::StyleColor_NodeBg,     ImVec4(0.04f, 0.16f, 0.30f, 1.0f));
  ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.18f, 0.65f, 0.95f, 0.9f));

  for (int i = 0; i < static_cast<int>(model.actions.size()); ++i) {
    ActionDef& action = model.actions[i];
    const ed::NodeId nodeId = 3000 + i;

    if (action.posX == 0.0f && action.posY == 0.0f) {
      ed::SetNodePosition(nodeId, ImVec2(80.0f + static_cast<float>(i) * 260.0f, 300.0f));
    }

    const bool errorHighlighted =
        containsName(m_highlightedActions, action.name) ||
        containsName(m_structuralErrorActions, action.name);
    const bool warningHighlighted =
        !errorHighlighted &&
        containsName(m_structuralWarningActions, action.name);
    if (errorHighlighted) {
      ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(1.0f, 0.2f, 0.2f, 1.0f));
    } else if (warningHighlighted) {
      ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
    }
    ed::BeginNode(nodeId);

    if (showDetails) {
      ImGui::TextColored(ImVec4(0.30f, 0.85f, 1.0f, 1.0f), "[Action]");
    }
    ImGui::Text("%s", action.name.empty() ? "(unnamed)" : action.name.c_str());
    if (showDetails) {
      for (const auto& p : action.params) {
        ImGui::Text("  %s - %s", p.name.c_str(), p.type.c_str());
      }
    } else if (showCounts) {
      ImGui::TextDisabled("%zu conditions, %zu outcomes",
                          action.preconditions.size(),
                          action.addEffects.size() + action.delEffects.size());
    }

    if (showDetails && !action.preconditions.empty()) {
      // No ImGui::Separator here — it would stretch across the full canvas
      // width inside an ed::BeginNode block. The TextDisabled label alone
      // is enough to demarcate the section.
      ImGui::Spacing();
      ImGui::TextDisabled("Preconditions");
    }
    for (int pi = 0; pi < static_cast<int>(action.preconditions.size()); ++pi) {
      const ed::PinId pinId = actionPreconditionPinId(i, pi);
      ed::BeginPin(pinId, ed::PinKind::Input);
      const std::string label = showDetails
          ? authoring::formatEffectRef(action.preconditions[pi]) : " ";
      ImGui::TextColored(ImVec4(0.88F, 0.69F, 0.32F, 1.0F), "%s", label.c_str());
      ed::EndPin();
    }

    if (showDetails && (!action.addEffects.empty() || !action.delEffects.empty())) {
      ImGui::Spacing();
      ImGui::TextDisabled("Effects");
    }
    for (int ai = 0; ai < static_cast<int>(action.addEffects.size()); ++ai) {
      const ed::PinId pinId = actionEffectPinId(i, ai);
      ed::BeginPin(pinId, ed::PinKind::Output);
      const std::string label = showDetails
          ? "+ " + authoring::formatEffectRef(action.addEffects[ai]) : " ";
      ImGui::TextColored(ImVec4(0.32F, 0.84F, 0.60F, 1.0F), "%s", label.c_str());
      ed::EndPin();
    }
    for (int di = 0; di < static_cast<int>(action.delEffects.size()); ++di) {
      const int slotIdx = static_cast<int>(action.addEffects.size()) + di;
      const ed::PinId pinId = actionEffectPinId(i, slotIdx);
      ed::BeginPin(pinId, ed::PinKind::Output);
      const std::string label = showDetails
          ? "- " + authoring::formatEffectRef(action.delEffects[di]) : " ";
      ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F), "%s", label.c_str());
      ed::EndPin();
    }

    ed::EndNode();
    if (errorHighlighted || warningHighlighted) {
      ed::PopStyleColor();
    }
  }

  ed::PopStyleColor(2);

  int link_id = 6000;
  for (size_t predicate_index = 0; predicate_index < model.predicates.size();
       ++predicate_index) {
    const PredicateRelations& relations = relation_index.predicate(predicate_index);
    for (const auto& relation : relations.requiredBy) {
      ed::Link(link_id++, 2000 + static_cast<int>(predicate_index),
               actionPreconditionPinId(static_cast<int>(relation.actionIndex),
                                       static_cast<int>(relation.referenceIndex)),
               ImVec4(0.88F, 0.69F, 0.32F, 1.0F));
    }
    for (const auto& relation : relations.madeTrueBy) {
      ed::Link(link_id++,
               actionEffectPinId(static_cast<int>(relation.actionIndex),
                                 static_cast<int>(relation.referenceIndex)),
               2100 + static_cast<int>(predicate_index),
               ImVec4(0.32F, 0.84F, 0.60F, 1.0F));
    }
    for (const auto& relation : relations.madeFalseBy) {
      const ActionDef& action = model.actions[relation.actionIndex];
      const int slot = static_cast<int>(action.addEffects.size() + relation.referenceIndex);
      ed::Link(link_id++,
               actionEffectPinId(static_cast<int>(relation.actionIndex), slot),
               2100 + static_cast<int>(predicate_index),
               ImVec4(0.95F, 0.51F, 0.42F, 1.0F));
    }
  }

  if (m_fitToContents) {
    ed::NavigateToContent();
    m_fitToContents = false;
  }

  // ---- Right-click context menu on empty canvas (must be inside Begin/End)
  ed::Suspend();
  if (ed::ShowBackgroundContextMenu()) {
    ImGui::OpenPopup("##CanvasCtx");
  }
  if (ImGui::BeginPopup("##CanvasCtx")) {
    if (ImGui::MenuItem("Add Predicate")) {
      m_openAddPredicatePopup = true;
      m_newPredName[0] = '\0';
    }
    if (ImGui::MenuItem("Add Action")) {
      m_openAddActionPopup = true;
      m_newActionName[0] = '\0';
    }
    if (ImGui::MenuItem("Add Type")) {
      m_openAddTypePopup = true;
      m_newTypeName[0] = '\0';
    }
    ImGui::EndPopup();
  }
  ed::Resume();

  ed::End();

  // ---- Track selection (after End so selection state is finalised) ----
  // Only overwrite when the canvas reports an active selection; otherwise
  // preserve whatever the palette / cross-view caller set via setSelectedXxx.
  {
    const int total = ed::GetSelectedObjectCount();
    if (total > 0) {
      m_selectedPredIdx = -1;
      m_selectedActionIdx = -1;
      std::vector<ed::NodeId> sel(static_cast<size_t>(total));
      const int cnt = ed::GetSelectedNodes(sel.data(), total);
      for (int i = 0; i < cnt; ++i) {
        const int id = static_cast<int>(sel[static_cast<size_t>(i)].Get());
        if (id >= 1000 && id < 2000) {
          setSelectedPredicate(id - 1000);
        } else if (id >= 3000 && id < 4000) {
          setSelectedAction(id - 3000);
        }
      }
    }
  }

  // ---- Persist node positions back to model ----------------------------
  for (int i = 0; i < static_cast<int>(model.predicates.size()); ++i) {
    const ImVec2 pos = ed::GetNodePosition(1000 + i);
    model.predicates[i].posX = pos.x;
    model.predicates[i].posY = pos.y;
  }
  for (int i = 0; i < static_cast<int>(model.actions.size()); ++i) {
    const ImVec2 pos = ed::GetNodePosition(3000 + i);
    model.actions[i].posX = pos.x;
    model.actions[i].posY = pos.y;
  }

  // ---- Add Predicate modal --------------------------------------------
  if (m_openAddPredicatePopup) {
    ImGui::OpenPopup("Add Predicate##modal");
    m_openAddPredicatePopup = false;
  }
  if (ImGui::BeginPopupModal("Add Predicate##modal", nullptr,
                              ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::InputText("Name##newpred", m_newPredName, sizeof(m_newPredName));
    if (ImGui::Button("Add") && m_newPredName[0] != '\0') {
      const std::string name = m_newPredName;
      stack.execute(model, "Add predicate", [name](ProjectModel& m) {
        PredicateDef p;
        p.name = name;
        m.predicates.push_back(p);
      });
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // ---- Add Action modal -----------------------------------------------
  if (m_openAddActionPopup) {
    ImGui::OpenPopup("Add Action##modal");
    m_openAddActionPopup = false;
  }
  if (ImGui::BeginPopupModal("Add Action##modal", nullptr,
                              ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::InputText("Name##newaction", m_newActionName, sizeof(m_newActionName));
    if (ImGui::Button("Add") && m_newActionName[0] != '\0') {
      const std::string name = m_newActionName;
      stack.execute(model, "Add action", [name](ProjectModel& m) {
        ActionDef action;
        action.name = name;
        m.actions.push_back(action);
      });
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // ---- Add Type modal --------------------------------------------------
  if (m_openAddTypePopup) {
    ImGui::OpenPopup("Add Type##modal");
    m_openAddTypePopup = false;
  }
  if (ImGui::BeginPopupModal("Add Type##modal", nullptr,
                              ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::InputText("Name##newtype", m_newTypeName, sizeof(m_newTypeName));
    ImGui::TextDisabled("It will be a kind of object; change its parent in the "
                        "sidebar.");
    if (ImGui::Button("Add") && m_newTypeName[0] != '\0') {
      const std::string name = m_newTypeName;
      stack.execute(model, "Add type", [name](ProjectModel& m) {
        m.types.push_back({name, "object"});
      });
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  ed::SetCurrentEditor(nullptr);
}

void DomainGraphPanel::renderFocused(const ProjectModel& model,
                                     const RelationIndex& index,
                                     int depth,
                                     uint32_t relationshipFilter,
                                     size_t neighbourCap) {
  m_widestNeighbourItem = 0.0F;
  m_neighbourItemIds.clear();
  DomainElementRef focus;
  if (m_selectedPredIdx >= 0 &&
      m_selectedPredIdx < static_cast<int>(model.predicates.size())) {
    focus = {DomainElementKind::Predicate, static_cast<size_t>(m_selectedPredIdx)};
  } else if (m_selectedActionIdx >= 0 &&
             m_selectedActionIdx < static_cast<int>(model.actions.size())) {
    focus = {DomainElementKind::Action, static_cast<size_t>(m_selectedActionIdx)};
  } else if (!model.predicates.empty()) {
    setSelectedPredicate(0);
    focus = {DomainElementKind::Predicate, 0};
  } else if (!model.actions.empty()) {
    setSelectedAction(0);
    focus = {DomainElementKind::Action, 0};
  } else {
    ImGui::TextDisabled("Add a fact or action to explore its neighbourhood.");
    return;
  }

  const NeighbourhoodModel neighbourhood(model, index, focus, depth,
                                         relationshipFilter, neighbourCap);
  ImGui::TextDisabled("CHANGES IT");
  ImGui::SameLine(360.0F);
  ImGui::TextDisabled("IN FOCUS");
  ImGui::SameLine(690.0F);
  ImGui::TextDisabled("NEEDS IT");

  DomainElementRef clicked;
  bool has_clicked = false;
  ed::SetCurrentEditor(m_context);
  ed::Begin("FocusedNeighbourhoodCanvas");
  // A link is drawn as a curve whose bend is set by the "strength" of the pins
  // at each end. At zero the curve's control points sit on the endpoints and
  // the link is drawn as a straight line. The value is read when a pin is
  // declared rather than when the link is drawn, so it has to be pushed around
  // the nodes below, not around the links further down.
  //
  // The library has these two shapes only. There is no right-angled or routed
  // link style to choose from.
  ed::PushStyleVar(ed::StyleVar_LinkStrength,
                   m_straightLinks ? 0.0F : kCurvedLinkStrength);
  for (const NeighbourNode& node : neighbourhood.nodes()) {
    const ed::NodeId node_id = 100000 + node.id;
    ed::SetNodePosition(node_id, ImVec2(node.x, node.y));
    const bool focus_node = node.column == NeighbourColumn::InFocus;
    if (focus_node) {
      ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.0F, 0.9F, 1.0F, 1.0F));
    }
    ed::BeginNode(node_id);
    // Both pins sit on the row that holds the name, one at each end, so a link
    // meets the node on the side it comes from. Drawing the input above the
    // name and the output below it, as this did originally, put every link
    // ending at a corner: they all converged on the top-left of the node in
    // focus and left the others from the bottom-left, which made the lines look
    // unrelated to the boxes they joined.
    ed::BeginPin(200000 + node.id * 2, ed::PinKind::Input);
    ImGui::TextUnformatted(" ");
    ed::EndPin();
    ImGui::SameLine();
    std::string label;
    if (node.element.kind == DomainElementKind::Predicate) {
      label = model.predicates[node.element.index].name;
    } else if (node.element.kind == DomainElementKind::Action) {
      label = model.actions[node.element.index].name;
    } else {
      label = "+ " + std::to_string(node.hiddenCount) + " more";
    }
    // Dear ImGui gives a clickable item its identity from its label text, and
    // it sizes a Selectable to the full width available unless told otherwise.
    // Neither default suits this view.
    //
    // One action often appears in more than one column, because an action can
    // require a fact, make it true and make it false all at once. Those are
    // separate nodes showing the same name, so the label on its own is not a
    // unique identity and Dear ImGui reports the repeats as conflicting items.
    // The text after "##" is not drawn, so appending the node number keeps the
    // visible label while making each item distinct.
    //
    // The width matters because a node here sits on a canvas rather than inside
    // a panel, so "all the width available" is the width of the whole window.
    // A full-width Selectable would draw its highlight across the screen. The
    // size is therefore fixed to the text it contains.
    const std::string item_id = label + "##neighbour" + std::to_string(node.id);
    m_neighbourItemIds.push_back(item_id);
    const ImVec2 label_size = ImGui::CalcTextSize(label.c_str());
    const bool node_clicked =
        ImGui::Selectable(item_id.c_str(), focus_node, ImGuiSelectableFlags_None,
                          ImVec2(label_size.x, label_size.y));
    // Recorded so the offscreen self-test can prove these items stay the width
    // of their text. A regression here is very visible but easy to miss in a
    // screenshot, because it looks like a highlight rather than a broken node.
    m_widestNeighbourItem =
        std::max(m_widestNeighbourItem, ImGui::GetItemRectSize().x);
    if (node_clicked) {
      if (node.element.kind == DomainElementKind::More) {
        m_morePopupOpen = true;
      } else {
        clicked = node.element;
        has_clicked = true;
      }
    }
    // A node is as wide as its widest line, which is usually the reason
    // underneath rather than the name. Placing the outgoing pin straight after
    // the name would leave it short of the node's right-hand side, and the link
    // would then start inside the node and cross its own border on the way out.
    // Padding the name out to the node's width puts the pin on the edge, so a
    // link begins where the node visibly ends.
    const float reason_width =
        node.reason.empty() ? 0.0F : ImGui::CalcTextSize(node.reason.c_str()).x;
    if (reason_width > label_size.x) {
      ImGui::SameLine(0.0F, 0.0F);
      ImGui::Dummy(ImVec2(reason_width - label_size.x, 1.0F));
    }
    ImGui::SameLine(0.0F, 0.0F);
    ed::BeginPin(200001 + node.id * 2, ed::PinKind::Output);
    ImGui::TextUnformatted(" ");
    ed::EndPin();
    if (!node.reason.empty()) {
      ImGui::TextDisabled("%s", node.reason.c_str());
    }
    ed::EndNode();
    if (focus_node) {
      ed::PopStyleColor();
    }
  }
  ed::PopStyleVar();
  for (size_t i = 0; i < neighbourhood.edges().size(); ++i) {
    const NeighbourEdge& edge = neighbourhood.edges()[i];
    ImVec4 colour(0.88F, 0.69F, 0.32F, 1.0F);
    if (edge.kind == PredicateRelationKind::MakesTrue) {
      colour = ImVec4(0.32F, 0.84F, 0.60F, 1.0F);
    } else if (edge.kind == PredicateRelationKind::MakesFalse) {
      colour = ImVec4(0.95F, 0.51F, 0.42F, 1.0F);
    }
    ed::Link(300000 + static_cast<int>(i),
             200001 + edge.fromNode * 2,
             200000 + edge.toNode * 2, colour, 1.6F);
  }
  ed::End();
  ed::SetCurrentEditor(nullptr);

  if (has_clicked) {
    select(clicked, true);
  }
  if (m_morePopupOpen) {
    ImGui::OpenPopup("More neighbours##focused");
    m_morePopupOpen = false;
  }
  if (ImGui::BeginPopup("More neighbours##focused")) {
    const NeighbourhoodModel all(model, index, focus, depth,
                                 relationshipFilter, 1000U);
    for (const NeighbourNode& node : all.nodes()) {
      if (node.element.kind == DomainElementKind::More || node.id == 0) {
        continue;
      }
      const std::string label = node.element.kind == DomainElementKind::Predicate
          ? model.predicates[node.element.index].name
          : model.actions[node.element.index].name;
      ImGui::PushID(node.id);
      if (ImGui::Selectable(label.c_str())) {
        select(node.element, true);
        ImGui::CloseCurrentPopup();
      }
      ImGui::PopID();
    }
    ImGui::EndPopup();
  }
}
