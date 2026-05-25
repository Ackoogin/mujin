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

static bool decodePreconditionPin(ed::PinId pinId, int& actionIdx, int& slotIdx) {
  const int id = static_cast<int>(pinId.Get());
  if (id < 4000 || id >= 5000) {
    return false;
  }
  const int localId = id - 4000;
  actionIdx = localId / 100;
  slotIdx = localId % 100;
  return true;
}

static bool decodeEffectPin(ed::PinId pinId, int& actionIdx, int& slotIdx) {
  const int id = static_cast<int>(pinId.Get());
  if (id < 5000 || id >= 6000) {
    return false;
  }
  const int localId = id - 5000;
  actionIdx = localId / 100;
  slotIdx = localId % 100;
  return true;
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

  // ---- Predicate nodes (green) ----------------------------------------
  ed::PushStyleColor(ed::StyleColor_NodeBg,     ImVec4(0.05f, 0.28f, 0.10f, 1.0f));
  ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.18f, 0.65f, 0.25f, 0.9f));

  for (int i = 0; i < static_cast<int>(model.predicates.size()); ++i) {
    PredicateDef& pred = model.predicates[i];
    const ed::NodeId nodeId = 1000 + i;
    const ed::PinId  pinId  = 2000 + i;

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

    ImGui::TextColored(ImVec4(0.25f, 0.90f, 0.40f, 1.0f), "[Predicate]");
    ImGui::Text("%s", pred.name.empty() ? "(unnamed)" : pred.name.c_str());
    for (const auto& p : pred.params) {
      ImGui::Text("  %s: %s", p.name.c_str(), p.type.c_str());
    }

    // One output pin representing "this predicate is available"
    ed::BeginPin(pinId, ed::PinKind::Output);
    ImGui::Text("o");
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

    ImGui::TextColored(ImVec4(0.30f, 0.85f, 1.0f, 1.0f), "[Action]");
    ImGui::Text("%s", action.name.empty() ? "(unnamed)" : action.name.c_str());
    for (const auto& p : action.params) {
      ImGui::Text("  %s - %s", p.name.c_str(), p.type.c_str());
    }

    if (!action.preconditions.empty()) {
      // No ImGui::Separator here — it would stretch across the full canvas
      // width inside an ed::BeginNode block. The TextDisabled label alone
      // is enough to demarcate the section.
      ImGui::Spacing();
      ImGui::TextDisabled("Preconditions");
    }
    for (int pi = 0; pi < static_cast<int>(action.preconditions.size()); ++pi) {
      const ed::PinId pinId = actionPreconditionPinId(i, pi);
      ed::BeginPin(pinId, ed::PinKind::Input);
      const std::string label =
          authoring::formatEffectRef(action.preconditions[pi]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }

    if (!action.addEffects.empty() || !action.delEffects.empty()) {
      ImGui::Spacing();
      ImGui::TextDisabled("Effects");
    }
    for (int ai = 0; ai < static_cast<int>(action.addEffects.size()); ++ai) {
      const ed::PinId pinId = actionEffectPinId(i, ai);
      ed::BeginPin(pinId, ed::PinKind::Output);
      const std::string label =
          "+" + authoring::formatEffectRef(action.addEffects[ai]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }
    for (int di = 0; di < static_cast<int>(action.delEffects.size()); ++di) {
      const int slotIdx = static_cast<int>(action.addEffects.size()) + di;
      const ed::PinId pinId = actionEffectPinId(i, slotIdx);
      ed::BeginPin(pinId, ed::PinKind::Output);
      const std::string label =
          "-" + authoring::formatEffectRef(action.delEffects[di]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }

    ed::EndNode();
    if (errorHighlighted || warningHighlighted) {
      ed::PopStyleColor();
    }
  }

  ed::PopStyleColor(2);

  for (int i = 0; i < static_cast<int>(model.causalLinks.size()); ++i) {
    const CausalLink& link = model.causalLinks[static_cast<size_t>(i)];
    const ed::PinId effectPin = actionEffectPinId(link.fromAction, link.fromAddEffectIdx);
    const ed::PinId precPin = actionPreconditionPinId(link.toAction, link.toPreconditionIdx);
    ed::Link(6000 + i, effectPin, precPin, ImVec4(0.0f, 1.0f, 1.0f, 1.0f));
  }

  if (ed::BeginCreate()) {
    ed::PinId startPinId = 0;
    ed::PinId endPinId = 0;
    if (ed::QueryNewLink(&startPinId, &endPinId)) {
      int startEffectAction = -1;
      int startEffectSlot = -1;
      int endEffectAction = -1;
      int endEffectSlot = -1;
      int startPreAction = -1;
      int startPreSlot = -1;
      int endPreAction = -1;
      int endPreSlot = -1;
      const bool startIsEffect = decodeEffectPin(startPinId, startEffectAction, startEffectSlot);
      const bool endIsEffect = decodeEffectPin(endPinId, endEffectAction, endEffectSlot);
      const bool startIsPrecondition =
          decodePreconditionPin(startPinId, startPreAction, startPreSlot);
      const bool endIsPrecondition = decodePreconditionPin(endPinId, endPreAction, endPreSlot);

      CausalLink candidate;
      bool hasCandidate = false;
      if (startIsEffect && endIsPrecondition) {
        candidate = {startEffectAction, startEffectSlot, endPreAction, endPreSlot};
        hasCandidate = true;
      } else if (endIsEffect && startIsPrecondition) {
        candidate = {endEffectAction, endEffectSlot, startPreAction, startPreSlot};
        hasCandidate = true;
      }

      const bool compatible = hasCandidate && causalLinkCompatible(model, candidate);
      if (compatible) {
        if (ed::AcceptNewItem(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), 2.0f)) {
          stack.execute(model, "Add causal link", [candidate](ProjectModel& m) {
            m.causalLinks.push_back(candidate);
          });
        }
      } else if (startPinId && endPinId) {
        ed::RejectNewItem(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f);
      }
    }
    ed::EndCreate();
  }

  if (ed::BeginDelete()) {
    ed::LinkId deletedLinkId = 0;
    while (ed::QueryDeletedLink(&deletedLinkId)) {
      const int linkIdx = static_cast<int>(deletedLinkId.Get()) - 6000;
      if (linkIdx >= 0 && linkIdx < static_cast<int>(model.causalLinks.size()) &&
          ed::AcceptDeletedItem()) {
        stack.execute(model, "Delete causal link", [linkIdx](ProjectModel& m) {
          m.causalLinks.erase(m.causalLinks.begin() + linkIdx);
        });
      }
    }
    ed::EndDelete();
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
    ImGui::MenuItem("Add Type");    // placeholder
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
          m_selectedPredIdx = id - 1000;
        } else if (id >= 3000 && id < 4000) {
          m_selectedActionIdx = id - 3000;
          m_selectedPredIdx = -1;
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

  ed::SetCurrentEditor(nullptr);
}
