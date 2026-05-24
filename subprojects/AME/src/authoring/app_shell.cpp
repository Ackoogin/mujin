#include "app_shell.h"

#include "imgui.h"

AppShell::AppShell()
  : projectName("[No Project]"),
    validationState("Not validated"),
    lastOperation("Ready") {
}

void AppShell::renderMenuBar() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("New")) {
        projectName = "[Untitled]";
        m_model.clear();
        m_model.projectName = projectName;
        validationState = "Not validated";
        lastOperation = "New project";
      }
      if (ImGui::MenuItem("Open...")) {
        lastOperation = "TODO: open dialog";
      }
      if (ImGui::MenuItem("Save")) {
        lastOperation = "TODO: save";
      }
      if (ImGui::MenuItem("Save As...")) {
        lastOperation = "TODO: save-as dialog";
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Exit")) {
        wantsQuit = true;
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit")) {
      ImGui::BeginDisabled();
      ImGui::MenuItem("Undo");
      ImGui::MenuItem("Redo");
      ImGui::EndDisabled();
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Validate")) {
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Help")) {
      if (ImGui::MenuItem("About")) {
        showAboutModal = true;
      }
      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }
}

void AppShell::renderPanels() {
  ImGui::DockSpaceOverViewport();

  ImGui::Begin("Domain Graph", nullptr);
  m_domainGraph.render(m_model);
  ImGui::End();

  ImGui::Begin("Properties", nullptr);
  m_typeHierarchy.render(m_model);

  // Selected predicate editor
  const int selPred = m_domainGraph.selectedPredicateIndex();
  if (selPred >= 0 && selPred < static_cast<int>(m_model.predicates.size())) {
    PredicateDef& pred = m_model.predicates[selPred];
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.25f, 0.90f, 0.40f, 1.0f),
                       "Predicate: %s", pred.name.c_str());
    if (ImGui::BeginTable("##predparams", 2,
                           ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Param");
      ImGui::TableSetupColumn("Type");
      ImGui::TableHeadersRow();
      for (int pi = 0; pi < static_cast<int>(pred.params.size()); ++pi) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(pred.params[pi].name.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(pred.params[pi].type.c_str());
      }
      ImGui::EndTable();
    }
    static char s_pname[32] = {};
    static char s_ptype[32] = {};
    ImGui::InputText("Param##ppn", s_pname, sizeof(s_pname));
    ImGui::InputText("Type##ppt",  s_ptype, sizeof(s_ptype));
    if (ImGui::Button("Add Param") && s_pname[0] != '\0' && s_ptype[0] != '\0') {
      pred.params.push_back({s_pname, s_ptype});
      s_pname[0] = s_ptype[0] = '\0';
    }
  }

  ImGui::End();

  ImGui::Begin("PDDL Preview", nullptr);
  ImGui::TextDisabled("PDDL Preview panel");
  ImGui::End();

  ImGui::Begin("Validation Output", nullptr);
  ImGui::TextDisabled("Validation Output panel");
  ImGui::End();

  ImGui::Begin("Plan View", nullptr);
  ImGui::TextDisabled("Plan View panel");
  ImGui::End();

  ImGui::Begin("BT View", nullptr);
  ImGui::TextDisabled("BT View panel");
  ImGui::End();

  if (showAboutModal) {
    ImGui::OpenPopup("About##modal");
    showAboutModal = false;
  }

  if (ImGui::BeginPopupModal("About##modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("AME Authoring Tool");
    ImGui::Text("Version: 0.1.0-dev");
    if (ImGui::Button("OK")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void AppShell::renderStatusBar() {
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  constexpr float kStatusBarHeight = 22.0F;
  const ImGuiWindowFlags flags =
    ImGuiWindowFlags_NoDecoration |
    ImGuiWindowFlags_NoInputs |
    ImGuiWindowFlags_NoNav |
    ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_NoSavedSettings |
    ImGuiWindowFlags_NoBringToFrontOnFocus |
    ImGuiWindowFlags_NoScrollbar;

  ImGui::SetNextWindowPos(
    ImVec2(viewport->Pos.x, viewport->Pos.y + viewport->Size.y - kStatusBarHeight));
  ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kStatusBarHeight));
  ImGui::SetNextWindowBgAlpha(0.85F);

  ImGui::Begin("##StatusBar", nullptr, flags);
  ImGui::Text("%s  |  %s  |  %s",
              projectName.c_str(),
              validationState.c_str(),
              lastOperation.c_str());
  ImGui::End();
}
