#include "domain_graph_panel.h"

#include "authoring_utils.h"
#include "imgui.h"
#include "presentation_groups.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

// Connection points are numbered so that no two can ever come out the same.
// The strides are deliberately generous: an earlier scheme gave each action a
// hundred numbers starting at 4000 for its conditions and 5000 for its
// outcomes, which meant the eleventh action's first condition and the first
// action's first outcome were both 5000, and the canvas drew lines between the
// wrong boxes on any domain with eleven or more actions. The shipped sample
// project has seventeen.
static constexpr int kSlotsPerAction = 1000;

static ed::PinId factOutputPinId(int factIdx) {
  return 200000 + factIdx;
}

static ed::PinId factInputPinId(int factIdx) {
  return 300000 + factIdx;
}

static ed::PinId actionPreconditionPinId(int actionIdx, int slotIdx) {
  return 400000 + actionIdx * kSlotsPerAction + slotIdx;
}

static ed::PinId actionEffectPinId(int actionIdx, int slotIdx) {
  return 600000 + actionIdx * kSlotsPerAction + slotIdx;
}

// A collapsed group stands in for everything inside it, so it has one
// connection point in each direction rather than a row for every condition and
// outcome the actions inside it have.
static ed::PinId groupInputPinId(int groupIdx) {
  return 800000 + groupIdx;
}

static ed::PinId groupOutputPinId(int groupIdx) {
  return 900000 + groupIdx;
}

static ed::NodeId openGroupNodeId(int groupIdx) {
  return 9000 + groupIdx;
}

static ed::NodeId collapsedGroupNodeId(int groupIdx) {
  return 10000 + groupIdx;
}

/// How big a box is assumed to be before it has been drawn once and the node
/// editor can report its real size.
static constexpr float kAssumedNodeWidth = 180.0F;
static constexpr float kAssumedNodeHeight = 70.0F;
/// Space left between a group's box and the boxes inside it, and the height of
/// the strip along the top of the box that carries the group's name.
static constexpr float kGroupPadding = 18.0F;
static constexpr float kGroupTitleHeight = 30.0F;

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

void DomainGraphPanel::drawGroupBoxes(const ProjectModel& model,
                                      const CanvasLayout& layout) {
  if (layout.boxes.empty()) {
    return;
  }
  // Group boxes are drawn before the boxes they hold, so that they sit behind
  // them rather than over them.
  ed::PushStyleColor(ed::StyleColor_NodeBg, ImVec4(0.10F, 0.10F, 0.14F, 0.55F));
  ed::PushStyleColor(ed::StyleColor_NodeBorder,
                     ImVec4(0.62F, 0.55F, 0.85F, 0.85F));

  for (const CanvasGroupBox& box : layout.boxes) {
    const PresentationGroup& group = model.presentationGroups[box.groupIndex];

    // The box wraps whatever it holds. Nothing about its position or size is
    // stored, because a box the user had to keep dragging back over its own
    // contents would be work the model never reads.
    float minX = 0.0F;
    float minY = 0.0F;
    float maxX = 0.0F;
    float maxY = 0.0F;
    bool first = true;
    for (const size_t memberNode : box.memberNodes) {
      const CanvasNode& member = layout.nodes[memberNode];
      const bool isFact = member.kind == CanvasNodeKind::Fact;
      const ed::NodeId memberId =
          isFact ? 1000 + static_cast<int>(member.index)
                 : 3000 + static_cast<int>(member.index);
      const float x = isFact ? model.predicates[member.index].posX
                             : model.actions[member.index].posX;
      const float y = isFact ? model.predicates[member.index].posY
                             : model.actions[member.index].posY;
      // A box that has not been drawn yet reports no size, so assume one until
      // the next frame can measure it.
      ImVec2 size = ed::GetNodeSize(memberId);
      if (size.x <= 0.0F || size.y <= 0.0F) {
        size = ImVec2(kAssumedNodeWidth, kAssumedNodeHeight);
      }
      if (first) {
        minX = x;
        minY = y;
        maxX = x + size.x;
        maxY = y + size.y;
        first = false;
      } else {
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x + size.x);
        maxY = std::max(maxY, y + size.y);
      }
    }
    if (first) {
      continue;  // nothing left to wrap
    }

    const ed::NodeId groupNodeId =
        openGroupNodeId(static_cast<int>(box.groupIndex));
    const ImVec2 position(minX - kGroupPadding,
                          minY - kGroupPadding - kGroupTitleHeight);
    const ImVec2 size(maxX - minX + kGroupPadding * 2.0F,
                      maxY - minY + kGroupPadding * 2.0F + kGroupTitleHeight);
    ed::SetNodePosition(groupNodeId, position);
    ed::SetGroupSize(groupNodeId, size);

    ed::BeginNode(groupNodeId);
    ImGui::TextColored(ImVec4(0.78F, 0.72F, 0.98F, 1.0F), "%s",
                       group.name.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("(%s)",
                        PresentationGroups::describeContents(group).c_str());
    ed::Group(size);
    ed::EndNode();
  }

  ed::PopStyleColor(2);
}

void DomainGraphPanel::drawCollapsedGroups(ProjectModel& model,
                                           const CanvasLayout& layout,
                                           CommandStack& stack) {
  ed::PushStyleColor(ed::StyleColor_NodeBg, ImVec4(0.16F, 0.12F, 0.26F, 1.0F));
  ed::PushStyleColor(ed::StyleColor_NodeBorder,
                     ImVec4(0.62F, 0.55F, 0.85F, 0.9F));

  for (const CanvasNode& node : layout.nodes) {
    if (node.kind != CanvasNodeKind::CollapsedGroup) {
      continue;
    }
    const PresentationGroup& group = model.presentationGroups[node.index];
    const ed::NodeId nodeId = collapsedGroupNodeId(static_cast<int>(node.index));

    // The first time a group is drawn closed it appears in the middle of what
    // it holds, so that it turns up where the user was looking rather than at
    // the corner of the canvas. After that the editor remembers where it was
    // dragged to.
    if (std::find(m_positionedGroups.begin(), m_positionedGroups.end(),
                  group.name) == m_positionedGroups.end()) {
      float sumX = 0.0F;
      float sumY = 0.0F;
      float count = 0.0F;
      for (const std::string& name : group.predicateNames) {
        for (const PredicateDef& fact : model.predicates) {
          if (fact.name == name) {
            sumX += fact.posX;
            sumY += fact.posY;
            count += 1.0F;
          }
        }
      }
      for (const std::string& name : group.actionNames) {
        for (const ActionDef& action : model.actions) {
          if (action.name == name) {
            sumX += action.posX;
            sumY += action.posY;
            count += 1.0F;
          }
        }
      }
      if (count > 0.0F) {
        ed::SetNodePosition(nodeId, ImVec2(sumX / count, sumY / count));
      }
      m_positionedGroups.push_back(group.name);
    }

    ed::BeginNode(nodeId);
    ImGui::TextColored(ImVec4(0.78F, 0.72F, 0.98F, 1.0F), "[Group]");
    ImGui::Text("%s", group.name.c_str());
    ImGui::TextDisabled("%s",
                        PresentationGroups::describeContents(group).c_str());

    ed::BeginPin(groupInputPinId(static_cast<int>(node.index)),
                 ed::PinKind::Input);
    ImGui::TextUnformatted(" ");
    ed::EndPin();
    ImGui::SameLine();
    ed::BeginPin(groupOutputPinId(static_cast<int>(node.index)),
                 ed::PinKind::Output);
    ImGui::TextUnformatted(" ");
    ed::EndPin();

    ImGui::PushID(static_cast<int>(node.index));
    if (ImGui::SmallButton("Open")) {
      m_requestedGroupToOpen = static_cast<int>(node.index);
    }
    ImGui::PopID();
    ed::EndNode();
  }

  ed::PopStyleColor(2);

  // Applied after the loop, because changing the project while walking the
  // picture computed from it would leave the two disagreeing for a frame.
  if (m_requestedGroupToOpen >= 0) {
    const int groupIdx = m_requestedGroupToOpen;
    m_requestedGroupToOpen = -1;
    if (groupIdx < static_cast<int>(model.presentationGroups.size())) {
      const std::string label =
          "Open the group '" + model.presentationGroups[groupIdx].name + "'";
      stack.execute(model, label, [groupIdx](ProjectModel& m) {
        PresentationGroups::setCollapsed(m, static_cast<size_t>(groupIdx), false);
      });
    }
  }
}

void DomainGraphPanel::render(ProjectModel& model, CommandStack& stack) {
  ed::SetCurrentEditor(m_context);
  ed::Begin("DomainGraphCanvas");
  const float zoom = ed::GetCurrentZoom();
  const bool showDetails = zoom >= 0.75F;
  const bool showCounts = zoom >= 0.45F;
  const RelationIndex relation_index(model);

  // What the canvas should draw once named groups are taken into account:
  // which boxes are hidden inside a collapsed group, which group boxes to draw,
  // and where every line runs. Working that out is not a drawing job, so it
  // happens in PresentationGroups where it has tests around it.
  const CanvasLayout layout = PresentationGroups::computeLayout(model);
  m_collapsedGroupCount = 0;
  for (const CanvasNode& node : layout.nodes) {
    if (node.kind == CanvasNodeKind::CollapsedGroup) {
      ++m_collapsedGroupCount;
    }
  }
  m_visibleNodeCount = layout.nodes.size();

  drawGroupBoxes(model, layout);

  // ---- Predicate nodes (green) ----------------------------------------
  ed::PushStyleColor(ed::StyleColor_NodeBg,     ImVec4(0.05f, 0.28f, 0.10f, 1.0f));
  ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.18f, 0.65f, 0.25f, 0.9f));

  for (int i = 0; i < static_cast<int>(model.predicates.size()); ++i) {
    if (layout.factIsHidden(static_cast<size_t>(i))) {
      continue;  // a collapsed group is standing in for it
    }
    PredicateDef& pred = model.predicates[i];
    const ed::NodeId nodeId = 1000 + i;
    const ed::PinId outputPinId = factOutputPinId(i);
    const ed::PinId inputPinId = factInputPinId(i);

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
    if (layout.actionIsHidden(static_cast<size_t>(i))) {
      continue;  // a collapsed group is standing in for it
    }
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

  drawCollapsedGroups(model, layout, stack);

  // ---- Lines, taken straight from the computed picture ------------------
  int link_id = 6000;
  for (const CanvasLink& link : layout.links) {
    const CanvasNode& fact_node = layout.nodes[link.factNode];
    const CanvasNode& action_node = layout.nodes[link.actionNode];
    const bool fact_end_is_group =
        fact_node.kind == CanvasNodeKind::CollapsedGroup;
    const bool action_end_is_group =
        action_node.kind == CanvasNodeKind::CollapsedGroup;

    const ed::PinId fact_out =
        fact_end_is_group ? groupOutputPinId(static_cast<int>(fact_node.index))
                          : factOutputPinId(static_cast<int>(fact_node.index));
    const ed::PinId fact_in =
        fact_end_is_group ? groupInputPinId(static_cast<int>(fact_node.index))
                          : factInputPinId(static_cast<int>(fact_node.index));
    const ed::PinId action_in =
        action_end_is_group
            ? groupInputPinId(static_cast<int>(action_node.index))
            : actionPreconditionPinId(static_cast<int>(action_node.index),
                                      link.slot);
    const ed::PinId action_out =
        action_end_is_group
            ? groupOutputPinId(static_cast<int>(action_node.index))
            : actionEffectPinId(static_cast<int>(action_node.index), link.slot);

    switch (link.kind) {
    case CanvasLinkKind::Requires:
      ed::Link(link_id++, fact_out, action_in,
               ImVec4(0.88F, 0.69F, 0.32F, 1.0F));
      break;
    case CanvasLinkKind::MakesTrue:
      ed::Link(link_id++, action_out, fact_in,
               ImVec4(0.32F, 0.84F, 0.60F, 1.0F));
      break;
    case CanvasLinkKind::MakesFalse:
      ed::Link(link_id++, action_out, fact_in,
               ImVec4(0.95F, 0.51F, 0.42F, 1.0F));
      break;
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
      m_selectedGroupIdx = -1;
      m_selection = CanvasSelection{};
      std::vector<ed::NodeId> sel(static_cast<size_t>(total));
      const int cnt = ed::GetSelectedNodes(sel.data(), total);
      for (int i = 0; i < cnt; ++i) {
        const int id = static_cast<int>(sel[static_cast<size_t>(i)].Get());
        if (id >= 1000 && id < 2000) {
          const int factIdx = id - 1000;
          setSelectedPredicate(factIdx);
          if (factIdx < static_cast<int>(model.predicates.size())) {
            m_selection.factNames.push_back(
                model.predicates[static_cast<size_t>(factIdx)].name);
          }
        } else if (id >= 3000 && id < 4000) {
          const int actionIdx = id - 3000;
          setSelectedAction(actionIdx);
          if (actionIdx < static_cast<int>(model.actions.size())) {
            m_selection.actionNames.push_back(
                model.actions[static_cast<size_t>(actionIdx)].name);
          }
        } else if (id >= 10000 &&
                   id < 10000 + static_cast<int>(model.presentationGroups.size())) {
          m_selectedGroupIdx = id - 10000;
        }
      }
    }
  }

  // ---- Persist node positions back to model ----------------------------
  // Only boxes that were drawn this frame have a position to read. Asking for
  // the position of one hidden inside a collapsed group would write a zero back
  // to the project and lose where the user had put it.
  for (int i = 0; i < static_cast<int>(model.predicates.size()); ++i) {
    if (layout.factIsHidden(static_cast<size_t>(i))) {
      continue;
    }
    const ImVec2 pos = ed::GetNodePosition(1000 + i);
    model.predicates[i].posX = pos.x;
    model.predicates[i].posY = pos.y;
  }
  for (int i = 0; i < static_cast<int>(model.actions.size()); ++i) {
    if (layout.actionIsHidden(static_cast<size_t>(i))) {
      continue;
    }
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
    const std::string wanted = m_newTypeName;
    const bool taken =
        std::any_of(model.types.begin(), model.types.end(),
                    [&wanted](const TypeDef& type) {
                      return type.name == wanted;
                    });
    if (taken) {
      ImGui::TextDisabled("There is already a type called %s.", wanted.c_str());
    }
    if (taken) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Add") && !wanted.empty() && !taken) {
      stack.execute(model, "Add type", [wanted](ProjectModel& m) {
        m.types.push_back({wanted, "object"});
      });
      ImGui::CloseCurrentPopup();
    }
    if (taken) {
      ImGui::EndDisabled();
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
