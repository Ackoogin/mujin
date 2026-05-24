#include "domain_graph_panel.h"

#include "imgui.h"

#include <sstream>
#include <string>
#include <vector>

static std::string formatRef(const EffectRef& r) {
  std::ostringstream out;
  out << "(" << r.predicateName;
  for (const auto& arg : r.argNames) {
    out << " " << arg;
  }
  out << ")";
  return out.str();
}

DomainGraphPanel::DomainGraphPanel() {
  m_context = ed::CreateEditor(nullptr);
}

DomainGraphPanel::~DomainGraphPanel() {
  ed::DestroyEditor(m_context);
}

void DomainGraphPanel::render(ProjectModel& model) {
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

    ed::BeginNode(nodeId);

    ImGui::TextColored(ImVec4(0.30f, 0.85f, 1.0f, 1.0f), "[Action]");
    ImGui::Text("%s", action.name.empty() ? "(unnamed)" : action.name.c_str());
    for (const auto& p : action.params) {
      ImGui::Text("  %s - %s", p.name.c_str(), p.type.c_str());
    }

    if (!action.preconditions.empty()) {
      ImGui::Separator();
      ImGui::TextDisabled("Preconditions");
    }
    for (int pi = 0; pi < static_cast<int>(action.preconditions.size()); ++pi) {
      // Action input pins: 4000 + actionIdx * 100 + slotIdx.
      const ed::PinId pinId = 4000 + i * 100 + pi;
      ed::BeginPin(pinId, ed::PinKind::Input);
      const std::string label = formatRef(action.preconditions[pi]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }

    if (!action.addEffects.empty() || !action.delEffects.empty()) {
      ImGui::Separator();
      ImGui::TextDisabled("Effects");
    }
    for (int ai = 0; ai < static_cast<int>(action.addEffects.size()); ++ai) {
      // Action effect pins: 5000 + actionIdx * 100 + combined add/del slotIdx.
      const ed::PinId pinId = 5000 + i * 100 + ai;
      ed::BeginPin(pinId, ed::PinKind::Output);
      const std::string label = "+" + formatRef(action.addEffects[ai]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }
    for (int di = 0; di < static_cast<int>(action.delEffects.size()); ++di) {
      const int slotIdx = static_cast<int>(action.addEffects.size()) + di;
      const ed::PinId pinId = 5000 + i * 100 + slotIdx;
      ed::BeginPin(pinId, ed::PinKind::Output);
      const std::string label = "-" + formatRef(action.delEffects[di]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }

    ed::EndNode();
  }

  ed::PopStyleColor(2);

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
  m_selectedPredIdx = -1;
  m_selectedActionIdx = -1;
  {
    const int total = ed::GetSelectedObjectCount();
    if (total > 0) {
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
      PredicateDef p;
      p.name = m_newPredName;
      model.predicates.push_back(p);
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
      ActionDef action;
      action.name = m_newActionName;
      model.actions.push_back(action);
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
