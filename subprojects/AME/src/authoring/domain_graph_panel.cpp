#include "domain_graph_panel.h"

#include "imgui.h"

#include <vector>

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
    ImGui::MenuItem("Add Action");  // placeholder (WI-1.4)
    ImGui::MenuItem("Add Type");    // placeholder
    ImGui::EndPopup();
  }
  ed::Resume();

  ed::End();

  // ---- Track selection (after End so selection state is finalised) ----
  m_selectedPredIdx = -1;
  {
    const int total = ed::GetSelectedObjectCount();
    if (total > 0) {
      std::vector<ed::NodeId> sel(static_cast<size_t>(total));
      const int cnt = ed::GetSelectedNodes(sel.data(), total);
      for (int i = 0; i < cnt; ++i) {
        const int id = static_cast<int>(sel[static_cast<size_t>(i)].Get());
        if (id >= 1000 && id < 2000) {
          m_selectedPredIdx = id - 1000;
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

  ed::SetCurrentEditor(nullptr);
}
