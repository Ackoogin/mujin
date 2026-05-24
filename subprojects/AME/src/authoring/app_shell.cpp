#include "app_shell.h"

#include "imgui.h"

#include <cstdio>
#include <sstream>
#include <string>
#include <utility>
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

static std::string formatArgList(const std::vector<std::string>& args) {
  std::ostringstream out;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0U) {
      out << " ";
    }
    out << args[i];
  }
  return out.str();
}

static std::vector<std::string> parseArgList(const char* text) {
  std::vector<std::string> args;
  std::istringstream input(text);
  std::string arg;
  while (input >> arg) {
    args.push_back(arg);
  }
  return args;
}

static void renderActionRefSection(const char* title,
                                   const char* tableId,
                                   const char* addButtonLabel,
                                   std::vector<EffectRef>& refs,
                                   const ProjectModel& model,
                                   int& selectedPredicate,
                                   char* argBuffer,
                                   size_t argBufferSize) {
  ImGui::TextUnformatted(title);
  if (ImGui::BeginTable(tableId, 3,
                         ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Predicate");
    ImGui::TableSetupColumn("Args");
    ImGui::TableSetupColumn("");
    ImGui::TableHeadersRow();
    for (int ri = 0; ri < static_cast<int>(refs.size()); ++ri) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(refs[ri].predicateName.c_str());
      ImGui::TableSetColumnIndex(1);
      const std::string args = formatArgList(refs[ri].argNames);
      ImGui::TextUnformatted(args.c_str());
      ImGui::TableSetColumnIndex(2);
      ImGui::PushID(ri);
      if (ImGui::SmallButton("Remove")) {
        refs.erase(refs.begin() + ri);
        ImGui::PopID();
        --ri;
        continue;
      }
      ImGui::PopID();
    }
    ImGui::EndTable();
  }

  ImGui::PushID(tableId);
  if (model.predicates.empty()) {
    ImGui::TextDisabled("No predicates available");
    ImGui::PopID();
    return;
  }

  if (selectedPredicate < 0 ||
      selectedPredicate >= static_cast<int>(model.predicates.size())) {
    selectedPredicate = 0;
  }

  const char* preview = model.predicates[static_cast<size_t>(selectedPredicate)].name.c_str();
  if (ImGui::BeginCombo("Predicate", preview)) {
    for (int pi = 0; pi < static_cast<int>(model.predicates.size()); ++pi) {
      const bool selected = (pi == selectedPredicate);
      if (ImGui::Selectable(model.predicates[static_cast<size_t>(pi)].name.c_str(), selected)) {
        selectedPredicate = pi;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  ImGui::InputText("Args", argBuffer, argBufferSize);
  if (ImGui::Button(addButtonLabel)) {
    refs.push_back({
      model.predicates[static_cast<size_t>(selectedPredicate)].name,
      parseArgList(argBuffer)
    });
    argBuffer[0] = '\0';
  }
  ImGui::PopID();
}

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

  // Selected action editor
  const int selAction = m_domainGraph.selectedActionIndex();
  if (selAction >= 0 && selAction < static_cast<int>(m_model.actions.size())) {
    ActionDef& action = m_model.actions[selAction];
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.30f, 0.85f, 1.0f, 1.0f),
                       "Action: %s", action.name.c_str());
    if (ImGui::BeginTable("##actionparams", 2,
                           ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Param name");
      ImGui::TableSetupColumn("Type");
      ImGui::TableHeadersRow();
      for (int pi = 0; pi < static_cast<int>(action.params.size()); ++pi) {
        ImGui::TableNextRow();
        ImGui::PushID(pi);

        char name[64] = {};
        char type[64] = {};
        std::snprintf(name, sizeof(name), "%s", action.params[pi].name.c_str());
        std::snprintf(type, sizeof(type), "%s", action.params[pi].type.c_str());

        ImGui::TableSetColumnIndex(0);
        if (ImGui::InputText("##name", name, sizeof(name))) {
          action.params[pi].name = name;
        }
        ImGui::TableSetColumnIndex(1);
        if (ImGui::InputText("##type", type, sizeof(type))) {
          action.params[pi].type = type;
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }

    static char s_aname[32] = {};
    static char s_atype[32] = {};
    ImGui::InputText("Param##apn", s_aname, sizeof(s_aname));
    ImGui::InputText("Type##apt",  s_atype, sizeof(s_atype));
    if (ImGui::Button("Add Param##action") &&
        s_aname[0] != '\0' && s_atype[0] != '\0') {
      action.params.push_back({s_aname, s_atype});
      s_aname[0] = s_atype[0] = '\0';
    }

    static int s_prePredIdx = 0;
    static int s_addPredIdx = 0;
    static int s_delPredIdx = 0;
    static char s_preArgs[128] = {};
    static char s_addArgs[128] = {};
    static char s_delArgs[128] = {};

    ImGui::Separator();
    renderActionRefSection("Preconditions", "##preconditions", "Add Precondition",
                           action.preconditions, m_model, s_prePredIdx,
                           s_preArgs, sizeof(s_preArgs));
    ImGui::Separator();
    renderActionRefSection("Add-effects", "##addeffects", "Add Add-effect",
                           action.addEffects, m_model, s_addPredIdx,
                           s_addArgs, sizeof(s_addArgs));
    ImGui::Separator();
    renderActionRefSection("Del-effects", "##deleffects", "Add Del-effect",
                           action.delEffects, m_model, s_delPredIdx,
                           s_delArgs, sizeof(s_delArgs));

    if (!action.preconditions.empty() ||
        !action.addEffects.empty() ||
        !action.delEffects.empty()) {
      ImGui::Separator();
      ImGui::TextDisabled("Templates");
      for (const auto& ref : action.preconditions) {
        const std::string label = formatRef(ref);
        ImGui::Text("%s", label.c_str());
      }
      for (const auto& ref : action.addEffects) {
        const std::string label = "+" + formatRef(ref);
        ImGui::Text("%s", label.c_str());
      }
      for (const auto& ref : action.delEffects) {
        const std::string label = "-" + formatRef(ref);
        ImGui::Text("%s", label.c_str());
      }
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

void AppShell::selfTestNew() {
  projectName     = "[Untitled]";
  validationState = "Not validated";
  lastOperation   = "New project";
  m_model.clear();
  m_model.projectName = projectName;
}

void AppShell::selfTestAddPredicate(const std::string& name) {
  PredicateDef p;
  p.name = name;
  m_model.predicates.push_back(p);
  lastOperation = "Added predicate: " + name;
}

void AppShell::selfTestAddType(const std::string& name, const std::string& parent) {
  m_model.types.push_back({name, parent});
  lastOperation = "Added type: " + name;
}

void AppShell::selfTestAddAction(const std::string& name) {
  ActionDef action;
  action.name = name;
  m_model.actions.push_back(action);
  lastOperation = "Added action: " + name;
}

void AppShell::selfTestAddActionParam(int actionIdx,
                                      const std::string& name,
                                      const std::string& type) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_model.actions[static_cast<size_t>(actionIdx)].params.push_back({name, type});
}

void AppShell::selfTestAddActionPrecondition(int actionIdx,
                                             const std::string& predName,
                                             std::vector<std::string> argNames) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_model.actions[static_cast<size_t>(actionIdx)].preconditions.push_back({
    predName,
    std::move(argNames)
  });
}

void AppShell::selfTestAddActionAddEffect(int actionIdx,
                                          const std::string& predName,
                                          std::vector<std::string> argNames) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_model.actions[static_cast<size_t>(actionIdx)].addEffects.push_back({
    predName,
    std::move(argNames)
  });
}

void AppShell::selfTestAddActionDelEffect(int actionIdx,
                                          const std::string& predName,
                                          std::vector<std::string> argNames) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_model.actions[static_cast<size_t>(actionIdx)].delEffects.push_back({
    predName,
    std::move(argNames)
  });
}

bool AppShell::selfTestAddCausalLink(int fromAction,
                                     int fromAddEffectIdx,
                                     int toAction,
                                     int toPreconditionIdx) {
  CausalLink link;
  link.fromAction = fromAction;
  link.fromAddEffectIdx = fromAddEffectIdx;
  link.toAction = toAction;
  link.toPreconditionIdx = toPreconditionIdx;
  if (!causalLinkCompatible(m_model, link)) {
    return false;
  }
  m_model.causalLinks.push_back(link);
  return true;
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
