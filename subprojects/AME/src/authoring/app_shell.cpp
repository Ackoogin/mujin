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
  m_domainGraph.render();
  ImGui::End();

  ImGui::Begin("Properties", nullptr);
  ImGui::TextDisabled("Properties panel");
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
