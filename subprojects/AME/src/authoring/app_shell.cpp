#include "app_shell.h"

#include "imgui.h"
#include "pddl_generator.h"

#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

static std::string slugifyForFilename(const std::string& text) {
  std::string out;
  bool lastWasDash = false;

  for (unsigned char ch : text) {
    const char lower = static_cast<char>(std::tolower(ch));
    const bool isSlugChar =
        (lower >= 'a' && lower <= 'z') || (lower >= '0' && lower <= '9') ||
        lower == '-';
    if (isSlugChar) {
      if (lower == '-') {
        if (!out.empty() && !lastWasDash) {
          out.push_back(lower);
        }
        lastWasDash = true;
      } else {
        out.push_back(lower);
        lastWasDash = false;
      }
    } else if (!out.empty() && !lastWasDash) {
      out.push_back('-');
      lastWasDash = true;
    }
  }

  while (!out.empty() && out.back() == '-') {
    out.pop_back();
  }

  if (out.empty()) {
    return "untitled";
  }
  return out;
}

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

static bool isPddlKeywordLine(const std::string& line) {
  const size_t start = line.find_first_not_of(" \t");
  if (start == std::string::npos) {
    return false;
  }

  const std::string text = line.substr(start);
  return text.rfind("(define", 0) == 0 ||
         text.rfind("(:", 0) == 0 ||
         text.rfind(":requirements", 0) == 0 ||
         text.rfind(":parameters", 0) == 0 ||
         text.rfind(":precondition", 0) == 0 ||
         text.rfind(":effect", 0) == 0 ||
         text.rfind("(:goal", 0) == 0;
}

static void renderPddlText(const std::string& pddl) {
  const ImVec4 keywordColor(0.0F, 1.0F, 1.0F, 1.0F);
  std::istringstream input(pddl);
  std::string line;
  while (std::getline(input, line)) {
    if (isPddlKeywordLine(line)) {
      ImGui::TextColored(keywordColor, "%s", line.c_str());
    } else {
      ImGui::TextUnformatted(line.c_str());
    }
  }
}

static void renderActionRefSection(const char* title,
                                   const char* tableId,
                                   const char* addButtonLabel,
                                   const char* removeCommandLabel,
                                   const char* addCommandLabel,
                                   std::vector<EffectRef>& refs,
                                   ProjectModel& model,
                                   CommandStack& stack,
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
        const int removeIdx = ri;
        stack.execute(model, removeCommandLabel, [&](ProjectModel&) {
          refs.erase(refs.begin() + removeIdx);
        });
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
    const EffectRef ref{
      model.predicates[static_cast<size_t>(selectedPredicate)].name,
      parseArgList(argBuffer)
    };
    stack.execute(model, addCommandLabel, [ref, &refs](ProjectModel&) {
      refs.push_back(ref);
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
        m_commandStack.clear();
        projectName = "[Untitled]";
        m_model.clear();
        m_model.projectName = projectName;
        m_lastValidation = ValidationReport{};
        m_domainGraph.setHighlightedElements({}, {});
        validationState = "Not validated";
        lastOperation = "New project";
      }
      if (ImGui::MenuItem("Open...")) {
        lastOperation = "TODO: open dialog";
      }
      if (ImGui::MenuItem("Save")) {
        lastOperation = "TODO: save";
        if (m_autoValidateOnSave) {
          runValidation();
        }
      }
      if (ImGui::MenuItem("Save As...")) {
        lastOperation = "TODO: save-as dialog";
        if (m_autoValidateOnSave) {
          runValidation();
        }
      }
      if (ImGui::MenuItem("Export Domain PDDL...")) {
        const std::string slug = slugifyForFilename(m_model.projectName);
        const std::string path = "./" + slug + "-domain.pddl";
        std::ofstream file(path);
        file << PddlGenerator::generateDomain(m_model);
        lastOperation = file.good() ? "Wrote " + path : "Failed to write " + path;
      }
      if (ImGui::MenuItem("Export Problem PDDL...")) {
        if (m_model.scenarios.empty()) {
          lastOperation = "No scenarios to export";
        } else {
          const std::string slug = slugifyForFilename(m_model.projectName);
          const std::string scenarioSlug =
              slugifyForFilename(m_model.scenarios.front().name);
          const std::string path =
              "./" + slug + "-problem-" + scenarioSlug + ".pddl";
          std::ofstream file(path);
          file << PddlGenerator::generateProblem(m_model, m_model.scenarios.front().name);
          lastOperation = file.good() ? "Wrote " + path : "Failed to write " + path;
        }
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Exit")) {
        wantsQuit = true;
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::MenuItem("Undo", "Ctrl+Z", false, m_commandStack.canUndo())) {
        m_commandStack.undo(m_model);
      }
      if (ImGui::MenuItem("Redo", "Ctrl+Y", false, m_commandStack.canRedo())) {
        m_commandStack.redo(m_model);
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
      ImGui::MenuItem("Auto-validate on Save", nullptr, &m_autoValidateOnSave);
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Validate")) {
      if (ImGui::MenuItem("Validate Now")) {
        runValidation();
      }
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

const std::vector<std::string>& AppShell::tabLabels() {
  // Workflow order: author the domain -> generate PDDL -> see the plan -> see the BT.
  static const std::vector<std::string> labels = {"Domain", "PDDL", "Plan", "BT"};
  return labels;
}

void AppShell::renderPanels() {
  ImGuiIO& io = ImGui::GetIO();
  if (io.KeyCtrl && !io.WantTextInput) {
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false) && m_commandStack.canUndo()) {
      m_commandStack.undo(m_model);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Y, false) && m_commandStack.canRedo()) {
      m_commandStack.redo(m_model);
    }
  }

  // Single full-viewport host window holds the workflow tab bar. The menu bar
  // and the 22px status bar overlay carve out the top and bottom margins.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float menuH = ImGui::GetFrameHeight();
  constexpr float kStatusBarHeight = 22.0F;
  ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, vp->Pos.y + menuH));
  ImGui::SetNextWindowSize(ImVec2(vp->Size.x,
                                  vp->Size.y - menuH - kStatusBarHeight));
  const ImGuiWindowFlags hostFlags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_NoNavFocus;
  ImGui::Begin("##MainHost", nullptr, hostFlags);

  if (ImGui::BeginTabBar("##MainTabs", ImGuiTabBarFlags_None)) {
    if (ImGui::BeginTabItem(tabLabels()[0].c_str())) {
      renderDomainTab();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(tabLabels()[1].c_str())) {
      renderPddlTab();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(tabLabels()[2].c_str())) {
      renderPlanTab();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(tabLabels()[3].c_str())) {
      renderBtTab();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

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

void AppShell::renderDomainTab() {
  // Left sidebar: type hierarchy + selected-element editor.
  ImGui::BeginChild("##DomainSidebar", ImVec2(360.0F, 0.0F),
                    ImGuiChildFlags_Border);
  m_typeHierarchy.render(m_model, m_commandStack);
  renderSelectedElementEditor();
  ImGui::EndChild();

  ImGui::SameLine();

  // Right area: node-editor canvas.
  ImGui::BeginChild("##DomainCanvas", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border);
  m_domainGraph.render(m_model, m_commandStack);
  ImGui::EndChild();
}

void AppShell::renderPddlTab() {
  const float halfH = ImGui::GetContentRegionAvail().y * 0.5F - 4.0F;
  ImGui::BeginChild("##PddlPreview", ImVec2(0.0F, halfH),
                    ImGuiChildFlags_Border);
  const std::string pddl = PddlGenerator::generateDomain(m_model);
  renderPddlText(pddl);
  ImGui::EndChild();
  ImGui::BeginChild("##ValidationOutput", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border);
  if (m_lastValidation.ok) {
    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "Validation passed");
  } else if (m_lastValidation.errors.empty()) {
    ImGui::TextDisabled("Click Validate > Validate Now");
  } else {
    for (const auto& err : m_lastValidation.errors) {
      ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", err.message.c_str());
      for (const auto& pn : err.predicateNames) {
        ImGui::BulletText("predicate: %s", pn.c_str());
      }
      for (const auto& an : err.actionNames) {
        ImGui::BulletText("action: %s", an.c_str());
      }
    }
  }
  ImGui::EndChild();
}

void AppShell::renderPlanTab() {
  ImGui::TextDisabled("Plan View");
}

void AppShell::renderBtTab() {
  ImGui::TextDisabled("BT View");
}

void AppShell::renderSelectedElementEditor() {
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
      const std::string paramName = s_pname;
      const std::string paramType = s_ptype;
      m_commandStack.execute(m_model, "Add predicate parameter",
                             [selPred, paramName, paramType](ProjectModel& model) {
        model.predicates[static_cast<size_t>(selPred)].params.push_back({paramName, paramType});
      });
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
        // TODO: Route text-field edits through CommandStack in a later WI.
        if (ImGui::InputText("##name", name, sizeof(name))) {
          action.params[pi].name = name;
        }
        ImGui::TableSetColumnIndex(1);
        // TODO: Route text-field edits through CommandStack in a later WI.
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
      const std::string paramName = s_aname;
      const std::string paramType = s_atype;
      m_commandStack.execute(m_model, "Add action parameter",
                             [selAction, paramName, paramType](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].params.push_back({paramName, paramType});
      });
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
                           "Remove precondition", "Add precondition",
                           action.preconditions, m_model, m_commandStack, s_prePredIdx,
                           s_preArgs, sizeof(s_preArgs));
    ImGui::Separator();
    renderActionRefSection("Add-effects", "##addeffects", "Add Add-effect",
                           "Remove add-effect", "Add add-effect",
                           action.addEffects, m_model, m_commandStack, s_addPredIdx,
                           s_addArgs, sizeof(s_addArgs));
    ImGui::Separator();
    renderActionRefSection("Del-effects", "##deleffects", "Add Del-effect",
                           "Remove del-effect", "Add del-effect",
                           action.delEffects, m_model, m_commandStack, s_delPredIdx,
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
}

void AppShell::selfTestNew() {
  m_commandStack.clear();
  projectName     = "[Untitled]";
  validationState = "Not validated";
  lastOperation   = "New project";
  m_lastValidation = ValidationReport{};
  m_domainGraph.setHighlightedElements({}, {});
  m_model.clear();
  m_model.projectName = projectName;
}

void AppShell::selfTestAddPredicate(const std::string& name) {
  m_commandStack.execute(m_model, "Add predicate", [name](ProjectModel& model) {
    PredicateDef p;
    p.name = name;
    model.predicates.push_back(p);
  });
  lastOperation = "Added predicate: " + name;
}

void AppShell::selfTestAddPredicateParam(int predicateIdx,
                                         const std::string& name,
                                         const std::string& type) {
  if (predicateIdx < 0 || predicateIdx >= static_cast<int>(m_model.predicates.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Add predicate parameter",
                         [predicateIdx, name, type](ProjectModel& model) {
    model.predicates[static_cast<size_t>(predicateIdx)].params.push_back({name, type});
  });
}

void AppShell::selfTestAddType(const std::string& name, const std::string& parent) {
  m_commandStack.execute(m_model, "Add type", [name, parent](ProjectModel& model) {
    model.types.push_back({name, parent});
  });
  lastOperation = "Added type: " + name;
}

void AppShell::selfTestAddAction(const std::string& name) {
  m_commandStack.execute(m_model, "Add action", [name](ProjectModel& model) {
    ActionDef action;
    action.name = name;
    model.actions.push_back(action);
  });
  lastOperation = "Added action: " + name;
}

void AppShell::selfTestAddActionParam(int actionIdx,
                                      const std::string& name,
                                      const std::string& type) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Add action parameter",
                         [actionIdx, name, type](ProjectModel& model) {
    model.actions[static_cast<size_t>(actionIdx)].params.push_back({name, type});
  });
}

void AppShell::selfTestAddActionPrecondition(int actionIdx,
                                             const std::string& predName,
                                             std::vector<std::string> argNames) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Add precondition", [actionIdx, predName, argNames](
                                                      ProjectModel& model) {
    model.actions[static_cast<size_t>(actionIdx)].preconditions.push_back({
      predName,
      argNames
    });
  });
}

void AppShell::selfTestAddActionAddEffect(int actionIdx,
                                          const std::string& predName,
                                          std::vector<std::string> argNames) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Add add-effect", [actionIdx, predName, argNames](
                                                  ProjectModel& model) {
    model.actions[static_cast<size_t>(actionIdx)].addEffects.push_back({
      predName,
      argNames
    });
  });
}

void AppShell::selfTestAddActionDelEffect(int actionIdx,
                                          const std::string& predName,
                                          std::vector<std::string> argNames) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Add del-effect", [actionIdx, predName, argNames](
                                                  ProjectModel& model) {
    model.actions[static_cast<size_t>(actionIdx)].delEffects.push_back({
      predName,
      argNames
    });
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
  m_commandStack.execute(m_model, "Add causal link", [link](ProjectModel& model) {
    model.causalLinks.push_back(link);
  });
  return true;
}

bool AppShell::selfTestUndo() {
  return m_commandStack.undo(m_model);
}

bool AppShell::selfTestRedo() {
  return m_commandStack.redo(m_model);
}

void AppShell::selfTestValidate() {
  runValidation();
}

void AppShell::selfTestCorruptPredicateName(int idx) {
  if (idx < 0 || idx >= static_cast<int>(m_model.predicates.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Corrupt for test", [idx](ProjectModel& model) {
    model.predicates[static_cast<size_t>(idx)].name.clear();
  });
}

size_t AppShell::selfTestUndoDepth() const {
  return m_commandStack.undoDepth();
}

void AppShell::runValidation() {
  m_lastValidation = PddlValidator::validate(m_model, m_validationScenario);

  if (m_lastValidation.ok) {
    validationState = "Valid";
    m_domainGraph.setHighlightedElements({}, {});
  } else {
    validationState = std::to_string(m_lastValidation.errors.size()) + " error(s)";
    std::vector<std::string> preds;
    std::vector<std::string> acts;
    for (const auto& e : m_lastValidation.errors) {
      preds.insert(preds.end(), e.predicateNames.begin(), e.predicateNames.end());
      acts.insert(acts.end(), e.actionNames.begin(), e.actionNames.end());
    }
    m_domainGraph.setHighlightedElements(std::move(preds), std::move(acts));
  }

  lastOperation = "Validated PDDL";
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
