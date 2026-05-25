#include "app_shell.h"

#include "authoring_utils.h"
#include "imgui.h"
#include "pddl_generator.h"
#include "pddl_importer.h"

#include <ame/action_registry.h>
#include <ame/plan_compiler.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Forward declarations for helpers defined later in this file.
static void SectionAccent();
static void StatusPill(const char* text, ImVec4 borderColor, ImVec4 textColor);

static void renderPlanFluentList(const char* title,
                                 const std::vector<unsigned>& fluentIds,
                                 const PlanGraphPanel& graph) {
  ImGui::TextDisabled("%s", title);
  if (fluentIds.empty()) {
    ImGui::TextDisabled("  (none)");
    return;
  }
  for (const unsigned fluentId : fluentIds) {
    const std::string label = graph.fluentLabel(fluentId);
    ImGui::BulletText("%s", label.c_str());
  }
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

static bool readTextFile(const std::string& path, std::string& out) {
  std::ifstream file(path);
  if (!file.good()) {
    return false;
  }

  out.assign(std::istreambuf_iterator<char>(file),
             std::istreambuf_iterator<char>());
  return file.good() || file.eof();
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
      const std::string args = authoring::formatArgList(refs[ri].argNames);
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

static void renderFactSection(const char* title,
                              const char* tableId,
                              const char* addButtonLabel,
                              const char* removeCommandLabel,
                              const char* addCommandLabel,
                              std::vector<FactRef>& facts,
                              ProjectModel& model,
                              CommandStack& stack,
                              int& selectedPredicate,
                              char* argBuffer,
                              size_t argBufferSize) {
  ImGui::TextUnformatted(title);
  if (ImGui::BeginTable(tableId, 2,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Fact");
    ImGui::TableSetupColumn("");
    ImGui::TableHeadersRow();
    for (int fi = 0; fi < static_cast<int>(facts.size()); ++fi) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      const std::string label =
          authoring::formatFactRef(facts[static_cast<size_t>(fi)]);
      ImGui::TextUnformatted(label.c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::PushID(fi);
      if (ImGui::SmallButton("Remove")) {
        const int removeIdx = fi;
        stack.execute(model, removeCommandLabel, [&](ProjectModel&) {
          facts.erase(facts.begin() + removeIdx);
        });
        ImGui::PopID();
        --fi;
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

  const char* preview =
      model.predicates[static_cast<size_t>(selectedPredicate)].name.c_str();
  if (ImGui::BeginCombo("Predicate", preview)) {
    for (int pi = 0; pi < static_cast<int>(model.predicates.size()); ++pi) {
      const bool selected = (pi == selectedPredicate);
      if (ImGui::Selectable(model.predicates[static_cast<size_t>(pi)].name.c_str(),
                            selected)) {
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
    const FactRef fact{
      model.predicates[static_cast<size_t>(selectedPredicate)].name,
      parseArgList(argBuffer)
    };
    stack.execute(model, addCommandLabel, [fact, &facts](ProjectModel&) {
      facts.push_back(fact);
    });
    argBuffer[0] = '\0';
  }
  ImGui::PopID();
}

static bool scenarioNameTaken(const ProjectModel& model,
                              const std::string& name,
                              int exceptIdx = -1) {
  for (int si = 0; si < static_cast<int>(model.scenarios.size()); ++si) {
    if (si != exceptIdx &&
        model.scenarios[static_cast<size_t>(si)].name == name) {
      return true;
    }
  }
  return false;
}

static std::string uniqueScenarioCopyName(const ProjectModel& model,
                                          const std::string& originalName) {
  const std::string baseName = originalName + " (copy)";
  if (!scenarioNameTaken(model, baseName)) {
    return baseName;
  }

  for (int copyNumber = 2; copyNumber < 1000; ++copyNumber) {
    const std::string candidate =
        originalName + " (copy " + std::to_string(copyNumber) + ")";
    if (!scenarioNameTaken(model, candidate)) {
      return candidate;
    }
  }

  return originalName + " (copy)";
}

static std::vector<std::string>& expectationActionList(ProjectModel& model,
                                                       int scenarioIdx,
                                                       bool forbidden) {
  ScenarioExpectation& expectation =
      model.scenarios[static_cast<size_t>(scenarioIdx)].expectation;
  return forbidden ? expectation.forbiddenActions : expectation.expectedActions;
}

static void renderExpectationActionSection(const char* title,
                                           const char* tableId,
                                           const char* removeCommandLabel,
                                           const char* addCommandLabel,
                                           std::vector<std::string>& actionNames,
                                           ProjectModel& model,
                                           CommandStack& stack,
                                           int scenarioIdx,
                                           bool forbidden,
                                           int& selectedAction) {
  ImGui::TextUnformatted(title);
  if (ImGui::BeginTable(tableId, 2,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Action schema");
    ImGui::TableSetupColumn("");
    ImGui::TableHeadersRow();
    for (int ai = 0; ai < static_cast<int>(actionNames.size()); ++ai) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(actionNames[static_cast<size_t>(ai)].c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::PushID(ai);
      if (ImGui::SmallButton("Remove")) {
        const int removeIdx = ai;
        stack.execute(model, removeCommandLabel,
                      [scenarioIdx, forbidden, removeIdx](ProjectModel& target) {
          std::vector<std::string>& names =
              expectationActionList(target, scenarioIdx, forbidden);
          if (removeIdx >= 0 && removeIdx < static_cast<int>(names.size())) {
            names.erase(names.begin() + removeIdx);
          }
        });
        ImGui::PopID();
        --ai;
        continue;
      }
      ImGui::PopID();
    }
    ImGui::EndTable();
  }

  ImGui::PushID(tableId);
  if (model.actions.empty()) {
    ImGui::TextDisabled("No action schemas available");
    ImGui::PopID();
    return;
  }

  if (selectedAction < 0 ||
      selectedAction >= static_cast<int>(model.actions.size())) {
    selectedAction = 0;
  }

  const char* preview =
      model.actions[static_cast<size_t>(selectedAction)].name.c_str();
  ImGui::SetNextItemWidth(180.0F);
  if (ImGui::BeginCombo("Action", preview)) {
    for (int ai = 0; ai < static_cast<int>(model.actions.size()); ++ai) {
      const bool selected = (ai == selectedAction);
      if (ImGui::Selectable(model.actions[static_cast<size_t>(ai)].name.c_str(),
                            selected)) {
        selectedAction = ai;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Add")) {
    const std::string actionName =
        model.actions[static_cast<size_t>(selectedAction)].name;
    const bool alreadyListed =
        std::find(actionNames.begin(), actionNames.end(), actionName) !=
        actionNames.end();
    if (!actionName.empty() && !alreadyListed) {
      stack.execute(model, addCommandLabel,
                    [scenarioIdx, forbidden, actionName](ProjectModel& target) {
        std::vector<std::string>& names =
            expectationActionList(target, scenarioIdx, forbidden);
        if (std::find(names.begin(), names.end(), actionName) == names.end()) {
          names.push_back(actionName);
        }
      });
    }
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
        m_selectedScenarioIdx = -1;
        clearDerivedResults();
        resetScenarioEditorState();
        lastOperation = "New project";
      }
      if (ImGui::MenuItem("Open...")) {
        // File dialog deferred; read from a conventional project path.
        const std::string path = authoring::projectFilePath(m_model.projectName);
        ProjectModel loaded;
        if (loaded.load(path)) {
          m_commandStack.clear();
          m_model = std::move(loaded);
          projectName = m_model.projectName;
          m_selectedScenarioIdx = -1;
          clearDerivedResults();
          resetScenarioEditorState();
          lastOperation = "Loaded " + path;
        } else {
          lastOperation = "Failed to load " + path;
        }
      }
      if (ImGui::MenuItem("Save")) {
        const std::string path = authoring::projectFilePath(m_model.projectName);
        lastOperation = m_model.save(path) ? "Saved " + path
                                            : "Failed to save " + path;
        if (m_autoValidateOnSave) {
          runValidation();
        }
      }
      if (ImGui::MenuItem("Save As...")) {
        // No native file picker yet; uses the same path as Save.
        const std::string path = authoring::projectFilePath(m_model.projectName);
        lastOperation = m_model.save(path) ? "Saved as " + path
                                            : "Failed to save as " + path;
        if (m_autoValidateOnSave) {
          runValidation();
        }
      }
      if (ImGui::MenuItem("Import PDDL Domain...")) {
        const std::string path = authoring::importDomainPath();
        std::string pddl;
        if (!readTextFile(path, pddl)) {
          lastOperation = "Failed to read " + path;
        } else {
          const PddlImportResult import = PddlImporter::importDomain(pddl);
          if (!import.ok) {
            lastOperation = "Import failed: " + import.error;
          } else {
            m_commandStack.clear();
            m_model = import.model;
            projectName = m_model.projectName;
            m_selectedScenarioIdx = -1;
            clearDerivedResults();
            resetScenarioEditorState();
            lastOperation = "Imported domain: " + projectName;
          }
        }
      }
      if (ImGui::MenuItem("Import PDDL Problem...")) {
        const std::string path = authoring::importProblemPath();
        std::string pddl;
        if (!readTextFile(path, pddl)) {
          lastOperation = "Failed to read " + path;
        } else {
          const PddlImportResult import = PddlImporter::importProblem(m_model, pddl);
          if (!import.ok) {
            lastOperation = "Import failed: " + import.error;
          } else {
            m_model = import.model;
            clearDerivedResults();
            resetScenarioEditorState();
            m_selectedScenarioIdx =
                static_cast<int>(m_model.scenarios.size()) - 1;
            const std::string scenarioName =
                m_model.scenarios.empty() ? "" : m_model.scenarios.back().name;
            lastOperation = "Imported problem: " + scenarioName;
          }
        }
      }
      if (ImGui::MenuItem("Export Domain PDDL...")) {
        m_structuralReport = StructuralValidator::check(m_model);
        if (m_structuralReport.hasErrors()) {
          lastOperation =
              "Refusing: " + std::to_string(m_structuralReport.errorCount) +
              " structural error(s)";
        } else {
          const std::string path = authoring::domainPddlPath(m_model.projectName);
          std::ofstream file(path);
          file << PddlGenerator::generateDomain(m_model);
          lastOperation = file.good() ? "Wrote " + path : "Failed to write " + path;
        }
      }
      if (ImGui::MenuItem("Export Problem PDDL...")) {
        m_structuralReport = StructuralValidator::check(m_model);
        if (m_structuralReport.hasErrors()) {
          lastOperation =
              "Refusing: " + std::to_string(m_structuralReport.errorCount) +
              " structural error(s)";
        } else if (m_model.scenarios.empty()) {
          lastOperation = "No scenarios to export";
        } else {
          const std::string path =
              authoring::problemPddlPath(m_model.projectName,
                                         m_model.scenarios.front().name);
          std::ofstream file(path);
          file << PddlGenerator::generateProblem(m_model, m_model.scenarios.front().name);
          lastOperation = file.good() ? "Wrote " + path : "Failed to write " + path;
        }
      }
      if (ImGui::MenuItem("Export Regression Report...")) {
        if (m_lastBatchReport.results.empty()) {
          lastOperation = "No batch report to export";
        } else {
          const std::string path =
              authoring::regressionReportPath(m_model.projectName);
          std::ofstream file(path);
          file << ScenarioRunner::toJson(m_lastBatchReport);
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
      if (ImGui::MenuItem("Check Feasibility")) {
        runFeasibilityCheck();
      }
      if (ImGui::MenuItem("Run All Scenarios")) {
        runAllScenarios();
      }
      if (ImGui::MenuItem("Run Contingency Analysis")) {
        runContingencyAnalysis();
      }
      if (ImGui::MenuItem("Plan & Preview")) {
        runPlanAndPreview();
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

  // HUD-chrome: cyan hairline immediately below the menu bar.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float menuH = ImGui::GetFrameHeight();
  ImDrawList* fg = ImGui::GetForegroundDrawList();
  fg->AddLine(ImVec2(vp->Pos.x, vp->Pos.y + menuH - 0.5F),
              ImVec2(vp->Pos.x + vp->Size.x, vp->Pos.y + menuH - 0.5F),
              ImGui::GetColorU32(ImVec4(0.0F, 0.85F, 1.0F, 0.55F)),
              1.0F);
}

const std::vector<std::string>& AppShell::tabLabels() {
  // Workflow order: author the domain -> generate PDDL -> see the plan -> see the BT.
  static const std::vector<std::string> labels = {"Domain", "PDDL", "Plan", "BT"};
  return labels;
}

void AppShell::renderPanels() {
  m_structuralReport = StructuralValidator::check(m_model);
  std::vector<std::string> errPreds;
  std::vector<std::string> errActs;
  std::vector<std::string> warnPreds;
  std::vector<std::string> warnActs;
  for (const auto& issue : m_structuralReport.issues) {
    if (issue.severity == Severity::Error) {
      if (!issue.predicateName.empty()) {
        errPreds.push_back(issue.predicateName);
      }
      if (!issue.actionName.empty()) {
        errActs.push_back(issue.actionName);
      }
    } else {
      if (!issue.predicateName.empty()) {
        warnPreds.push_back(issue.predicateName);
      }
      if (!issue.actionName.empty()) {
        warnActs.push_back(issue.actionName);
      }
    }
  }
  const std::string selectedPlanAction = m_planGraph.selectedActionSchemaName();
  if (!selectedPlanAction.empty()) {
    warnActs.push_back(selectedPlanAction);
  }
  if (m_lastContingencyReport.ok) {
    warnPreds.insert(warnPreds.end(),
                     m_lastContingencyReport.contextPredicates.begin(),
                     m_lastContingencyReport.contextPredicates.end());
  }
  m_domainGraph.setStructuralHighlights(std::move(errPreds),
                                        std::move(errActs),
                                        std::move(warnPreds),
                                        std::move(warnActs));

  ImGuiIO& io = ImGui::GetIO();
  if (io.KeyCtrl && !io.WantTextInput) {
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false) && m_commandStack.canUndo()) {
      m_commandStack.undo(m_model);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Y, false) && m_commandStack.canRedo()) {
      m_commandStack.redo(m_model);
    }
    // Ctrl+D: duplicate selected predicate or action
    if (ImGui::IsKeyPressed(ImGuiKey_D, false)) {
      const int selPred = m_domainGraph.selectedPredicateIndex();
      const int selAct  = m_domainGraph.selectedActionIndex();
      if (selPred >= 0 && selPred < static_cast<int>(m_model.predicates.size())) {
        const PredicateDef src = m_model.predicates[static_cast<size_t>(selPred)];
        m_commandStack.execute(m_model, "Duplicate predicate", [src](ProjectModel& m) {
          PredicateDef d = src; d.name = src.name + "_copy"; d.posX = 0; d.posY = 0;
          m.predicates.push_back(d);
        });
      } else if (selAct >= 0 && selAct < static_cast<int>(m_model.actions.size())) {
        const ActionDef src = m_model.actions[static_cast<size_t>(selAct)];
        m_commandStack.execute(m_model, "Duplicate action", [src](ProjectModel& m) {
          ActionDef d = src; d.name = src.name + "_copy"; d.posX = 0; d.posY = 0;
          m.actions.push_back(d);
        });
      }
    }
  }
  // Function-key shortcuts (no Ctrl needed)
  if (!io.WantTextInput) {
    if (ImGui::IsKeyPressed(ImGuiKey_F5, false)) {
      runPlanAndPreview();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F6, false)) {
      runValidation();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Tab, false) && io.KeyCtrl) {
      // Ctrl+Tab cycles through workflow tabs
      const auto& labels = tabLabels();
      size_t currentIdx = 0;
      for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == m_requestedTab) { currentIdx = i; break; }
      }
      m_requestedTab = labels[(currentIdx + 1) % labels.size()];
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete, false)) {
      // Delete: remove selected predicate or action
      const int selPred = m_domainGraph.selectedPredicateIndex();
      const int selAct  = m_domainGraph.selectedActionIndex();
      if (selPred >= 0 && selPred < static_cast<int>(m_model.predicates.size())) {
        m_commandStack.execute(m_model, "Delete predicate", [selPred](ProjectModel& m) {
          m.predicates.erase(m.predicates.begin() + selPred);
        });
      } else if (selAct >= 0 && selAct < static_cast<int>(m_model.actions.size())) {
        m_commandStack.execute(m_model, "Delete action", [selAct](ProjectModel& m) {
          m.actions.erase(m.actions.begin() + selAct);
        });
      }
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
    struct TabDescriptor {
      const char* label;
      void (AppShell::*render)();
    };
    static constexpr std::array<TabDescriptor, 4> kTabs = {{
        {"Domain", &AppShell::renderDomainTab},
        {"PDDL", &AppShell::renderPddlTab},
        {"Plan", &AppShell::renderPlanTab},
        {"BT", &AppShell::renderBtTab},
    }};

    for (const TabDescriptor& tab : kTabs) {
      ImGuiTabItemFlags tabFlags = ImGuiTabItemFlags_None;
      if (m_requestedTab == tab.label) {
        tabFlags |= ImGuiTabItemFlags_SetSelected;
      }
      if (ImGui::BeginTabItem(tab.label, nullptr, tabFlags)) {
        (this->*tab.render)();
        ImGui::EndTabItem();
      }
    }
    m_requestedTab.clear();
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

static bool containsCaseInsensitive(const std::string& haystack, const char* needle) {
  if (needle == nullptr || needle[0] == '\0') { return true; }
  std::string h = haystack;
  std::string n = needle;
  std::transform(h.begin(), h.end(), h.begin(), [](unsigned char c){ return std::tolower(c); });
  std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c){ return std::tolower(c); });
  return h.find(n) != std::string::npos;
}

static std::string formatPredicateSignature(const PredicateDef& p) {
  std::string sig = p.name + "(";
  for (size_t i = 0; i < p.params.size(); ++i) {
    if (i > 0) { sig += ", "; }
    sig += p.params[i].type.empty() ? p.params[i].name : p.params[i].type;
  }
  sig += ")";
  return sig;
}

static std::string formatActionSignature(const ActionDef& a) {
  std::string sig = a.name + "(";
  for (size_t i = 0; i < a.params.size(); ++i) {
    if (i > 0) { sig += ", "; }
    sig += a.params[i].type.empty() ? a.params[i].name : a.params[i].type;
  }
  sig += ")";
  return sig;
}

void AppShell::renderDomainTab() {
  // Left sidebar: palette + type hierarchy + selected-element editor.
  ImGui::BeginChild("##DomainSidebar", ImVec2(420.0F, 0.0F),
                    ImGuiChildFlags_Border);

  // ---- Palette ----------------------------------------------------------
  SectionAccent();
  if (ImGui::CollapsingHeader("Palette", ImGuiTreeNodeFlags_DefaultOpen)) {
    static char s_paletteFilter[64] = {};
    ImGui::InputText("Filter##palette", s_paletteFilter, sizeof(s_paletteFilter));
    if (ImGui::SmallButton("Quick Add##palette")) {
      m_paletteQuickAddOpen = true;
      m_paletteQuickAddName[0] = '\0';
    }

    ImGui::Indent();
    ImGui::TextDisabled("Predicates");
    for (int i = 0; i < static_cast<int>(m_model.predicates.size()); ++i) {
      const auto& p = m_model.predicates[static_cast<size_t>(i)];
      if (!containsCaseInsensitive(p.name, s_paletteFilter)) { continue; }
      const std::string label = formatPredicateSignature(p);
      ImGui::PushID(10000 + i);
      if (ImGui::Selectable(label.c_str(), m_domainGraph.selectedPredicateIndex() == i)) {
        m_domainGraph.setSelectedPredicate(i);
      }
      ImGui::PopID();
    }
    ImGui::TextDisabled("Actions");
    for (int i = 0; i < static_cast<int>(m_model.actions.size()); ++i) {
      const auto& a = m_model.actions[static_cast<size_t>(i)];
      if (!containsCaseInsensitive(a.name, s_paletteFilter)) { continue; }
      const std::string label = formatActionSignature(a);
      ImGui::PushID(20000 + i);
      if (ImGui::Selectable(label.c_str(), m_domainGraph.selectedActionIndex() == i)) {
        m_domainGraph.setSelectedAction(i);
      }
      ImGui::PopID();
    }
    ImGui::Unindent();
  }

  // Quick-add modal
  if (m_paletteQuickAddOpen) {
    ImGui::OpenPopup("Quick Add##palette-modal");
    m_paletteQuickAddOpen = false;
  }
  if (ImGui::BeginPopupModal("Quick Add##palette-modal", nullptr,
                              ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::InputText("Name##qa", m_paletteQuickAddName, sizeof(m_paletteQuickAddName));
    ImGui::RadioButton("Predicate", &m_paletteQuickAddKind, 0); ImGui::SameLine();
    ImGui::RadioButton("Action",    &m_paletteQuickAddKind, 1);
    const bool canAdd = m_paletteQuickAddName[0] != '\0';
    if (ImGui::Button("Add") && canAdd) {
      const std::string nm = m_paletteQuickAddName;
      if (m_paletteQuickAddKind == 0) {
        m_commandStack.execute(m_model, "Quick add predicate", [nm](ProjectModel& m) {
          PredicateDef p; p.name = nm; m.predicates.push_back(p);
        });
      } else {
        m_commandStack.execute(m_model, "Quick add action", [nm](ProjectModel& m) {
          ActionDef a; a.name = nm; m.actions.push_back(a);
        });
      }
      m_paletteQuickAddName[0] = '\0';
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) { ImGui::CloseCurrentPopup(); }
    ImGui::EndPopup();
  }

  ImGui::Separator();
  m_typeHierarchy.render(m_model, m_commandStack);
  renderSelectedElementEditor();
  ImGui::Separator();
  SectionAccent();
  if (ImGui::CollapsingHeader("Scenarios", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (m_selectedScenarioIdx < 0 ||
        m_selectedScenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
      m_selectedScenarioIdx = -1;
    }

    const char* preview = "(none)";
    if (m_selectedScenarioIdx >= 0) {
      preview = m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)].name.c_str();
    }

    // Combo gets the full row so its preview text isn't clipped.
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##scenarioCombo", preview)) {
      for (int si = 0; si < static_cast<int>(m_model.scenarios.size()); ++si) {
        const bool selected = (si == m_selectedScenarioIdx);
        if (ImGui::Selectable(m_model.scenarios[static_cast<size_t>(si)].name.c_str(),
                              selected)) {
          m_selectedScenarioIdx = si;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      if (m_model.scenarios.empty()) {
        ImGui::TextDisabled("(none)");
      }
      ImGui::EndCombo();
    }

    // Duplicate / Delete on their own row below the combo so labels fit.
    bool hasSelection = (m_selectedScenarioIdx >= 0);
    const bool scenarioButtonsDisabled = !hasSelection;
    if (scenarioButtonsDisabled) {
      ImGui::BeginDisabled();
    }
    if (ImGui::SmallButton("Duplicate") && hasSelection) {
      const int duplicateIdx = m_selectedScenarioIdx;
      const std::string copyName = uniqueScenarioCopyName(
          m_model, m_model.scenarios[static_cast<size_t>(duplicateIdx)].name);
      m_commandStack.execute(m_model, "Duplicate scenario",
                             [duplicateIdx, copyName](ProjectModel& model) {
        ScenarioDef scenario = model.scenarios[static_cast<size_t>(duplicateIdx)];
        scenario.name = copyName;
        model.scenarios.push_back(std::move(scenario));
      });
      m_selectedScenarioIdx = static_cast<int>(m_model.scenarios.size()) - 1;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Delete") && hasSelection) {
      const int deleteIdx = m_selectedScenarioIdx;
      m_commandStack.execute(m_model, "Delete scenario", [deleteIdx](ProjectModel& model) {
        model.scenarios.erase(model.scenarios.begin() + deleteIdx);
      });
      if (m_model.scenarios.empty()) {
        m_selectedScenarioIdx = -1;
      } else if (m_selectedScenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
        m_selectedScenarioIdx = static_cast<int>(m_model.scenarios.size()) - 1;
      }
      hasSelection = (m_selectedScenarioIdx >= 0);
    }
    if (scenarioButtonsDisabled) {
      ImGui::EndDisabled();
    }

    ImGui::InputText("Name##scenario",
                     m_scenarioNameInput,
                     sizeof(m_scenarioNameInput));
    ImGui::SameLine();
    if (ImGui::Button("Add Scenario")) {
      const std::string name = m_scenarioNameInput;
      const bool taken = std::any_of(
          m_model.scenarios.begin(), m_model.scenarios.end(),
          [&name](const ScenarioDef& scenario) {
            return scenario.name == name;
          });
      if (!name.empty() && !taken) {
        m_commandStack.execute(m_model, "Add scenario", [name](ProjectModel& model) {
          ScenarioDef scenario;
          scenario.name = name;
          model.scenarios.push_back(std::move(scenario));
        });
        m_selectedScenarioIdx = static_cast<int>(m_model.scenarios.size()) - 1;
        m_scenarioNameInput[0] = '\0';
      }
    }

    if (hasSelection) {
      const std::string currentName =
          m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)].name;
      if (m_renameScenarioIdx != m_selectedScenarioIdx ||
          m_renameScenarioSource != currentName) {
        std::snprintf(m_renameScenarioNameInput,
                      sizeof(m_renameScenarioNameInput),
                      "%s",
                      currentName.c_str());
        m_renameScenarioIdx = m_selectedScenarioIdx;
        m_renameScenarioSource = currentName;
      }
    } else {
      m_renameScenarioNameInput[0] = '\0';
      m_renameScenarioIdx = -2;
      m_renameScenarioSource.clear();
    }

    if (!hasSelection) {
      ImGui::BeginDisabled();
    }
    ImGui::InputText("Rename to##scenario",
                     m_renameScenarioNameInput,
                     sizeof(m_renameScenarioNameInput));
    ImGui::SameLine();
    if (ImGui::Button("Rename") && hasSelection) {
      const int renameIdx = m_selectedScenarioIdx;
      const std::string newName = m_renameScenarioNameInput;
      if (!newName.empty() &&
          !scenarioNameTaken(m_model, newName, renameIdx)) {
        m_commandStack.execute(m_model, "Rename scenario",
                               [renameIdx, newName](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(renameIdx)].name = newName;
        });
        m_renameScenarioSource = newName;
      }
    }
    if (!hasSelection) {
      ImGui::EndDisabled();
    }

    if (m_selectedScenarioIdx >= 0 &&
        m_selectedScenarioIdx < static_cast<int>(m_model.scenarios.size())) {
      ScenarioDef& scenario =
          m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)];

      ImGui::Separator();
      renderFactSection("Initial state", "##initialfacts", "Add Fact",
                        "Remove initial fact", "Add initial fact",
                        scenario.initialState, m_model, m_commandStack,
                        m_initPredIdx, m_initArgsInput, sizeof(m_initArgsInput));
      ImGui::Separator();
      renderFactSection("Goals", "##goalfacts", "Add Fact##goal",
                        "Remove goal fact", "Add goal fact",
                        scenario.goals, m_model, m_commandStack,
                        m_goalPredIdx, m_goalArgsInput, sizeof(m_goalArgsInput));
      if (ImGui::CollapsingHeader("Expected outcome")) {
        ScenarioExpectation& expectation = scenario.expectation;
        bool shouldSucceed = expectation.shouldSucceed;
        if (ImGui::Checkbox("Should succeed (expect feasible)",
                            &shouldSucceed)) {
          const int scenarioIdx = m_selectedScenarioIdx;
          m_commandStack.execute(m_model, "Set scenario expected success",
                                 [scenarioIdx, shouldSucceed](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(scenarioIdx)]
                .expectation.shouldSucceed = shouldSucceed;
          });
        }

        int minPlanSteps = expectation.minPlanSteps;
        if (ImGui::InputInt("Min plan steps", &minPlanSteps)) {
          const int scenarioIdx = m_selectedScenarioIdx;
          minPlanSteps = std::max(0, minPlanSteps);
          m_commandStack.execute(m_model, "Set scenario min plan steps",
                                 [scenarioIdx, minPlanSteps](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(scenarioIdx)]
                .expectation.minPlanSteps = minPlanSteps;
          });
        }

        int maxPlanSteps = expectation.maxPlanSteps;
        if (ImGui::InputInt("Max plan steps", &maxPlanSteps)) {
          const int scenarioIdx = m_selectedScenarioIdx;
          maxPlanSteps = std::max(0, maxPlanSteps);
          m_commandStack.execute(m_model, "Set scenario max plan steps",
                                 [scenarioIdx, maxPlanSteps](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(scenarioIdx)]
                .expectation.maxPlanSteps = maxPlanSteps;
          });
        }
        ImGui::TextDisabled("Use 0 for no bound.");

        static int s_expectedActionIdx = 0;
        static int s_forbiddenActionIdx = 0;
        ImGui::Separator();
        renderExpectationActionSection("Expected actions",
                                       "##expectedactions",
                                       "Remove expected action",
                                       "Add expected action",
                                       expectation.expectedActions,
                                       m_model,
                                       m_commandStack,
                                       m_selectedScenarioIdx,
                                       false,
                                       s_expectedActionIdx);
        ImGui::Separator();
        renderExpectationActionSection("Forbidden actions",
                                       "##forbiddenactions",
                                       "Remove forbidden action",
                                       "Add forbidden action",
                                       expectation.forbiddenActions,
                                       m_model,
                                       m_commandStack,
                                       m_selectedScenarioIdx,
                                       true,
                                       s_forbiddenActionIdx);
      }
    }
  }
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
  if (m_lastValidation.grounding.valid) {
    ImGui::Separator();
    ImGui::TextDisabled("Grounding Report");
    ImGui::Text("Total fluents:        %u",
                m_lastValidation.grounding.totalFluents);
    ImGui::Text("Total ground actions: %u",
                m_lastValidation.grounding.totalGroundActions);
    for (const auto& warning : m_lastValidation.grounding.warnings) {
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f),
                         "WARNING: %s", warning.c_str());
    }

    if (ImGui::BeginTable("##predcounts", 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Predicate");
      ImGui::TableSetupColumn("Ground instances");
      ImGui::TableHeadersRow();
      for (const auto& stat : m_lastValidation.grounding.predicateStats) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(stat.elementName.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", stat.count);
      }
      ImGui::EndTable();
    }

    if (ImGui::BeginTable("##actioncounts", 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Action schema");
      ImGui::TableSetupColumn("Ground actions");
      ImGui::TableHeadersRow();
      for (const auto& stat : m_lastValidation.grounding.actionStats) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(stat.elementName.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", stat.count);
      }
      ImGui::EndTable();
    }
  }
  ImGui::Separator();
  ImGui::TextDisabled("Structural Issues");
  if (m_structuralReport.issues.empty()) {
    ImGui::TextColored(ImVec4(0.2F, 0.9F, 0.3F, 1.0F), "No structural issues");
  } else {
    for (const auto& issue : m_structuralReport.issues) {
      if (issue.severity == Severity::Error) {
        ImGui::TextColored(ImVec4(1.0F, 0.4F, 0.4F, 1.0F),
                           "ERR: %s",
                           issue.message.c_str());
      } else {
        ImGui::TextColored(ImVec4(1.0F, 0.8F, 0.2F, 1.0F),
                           "WARN: %s",
                           issue.message.c_str());
      }
    }
  }
  if (!m_lastBatchReport.results.empty()) {
    ImGui::Separator();
    ImGui::TextDisabled("Scenario regression: %zu pass, %zu fail, %zu error",
                        m_lastBatchReport.passCount,
                        m_lastBatchReport.failCount,
                        m_lastBatchReport.errorCount);
    if (ImGui::BeginTable("##regressionResults", 5,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Scenario");
      ImGui::TableSetupColumn("Outcome");
      ImGui::TableSetupColumn("Steps");
      ImGui::TableSetupColumn("Time (ms)");
      ImGui::TableSetupColumn("Notes");
      ImGui::TableHeadersRow();
      for (const auto& r : m_lastBatchReport.results) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(r.scenarioName.c_str());
        ImGui::TableSetColumnIndex(1);
        ImVec4 c = (r.outcome == ScenarioOutcome::Pass)
                       ? ImVec4(0.3F, 0.9F, 0.3F, 1.0F)
                       : (r.outcome == ScenarioOutcome::Fail)
                             ? ImVec4(1.0F, 0.5F, 0.3F, 1.0F)
                             : ImVec4(1.0F, 0.3F, 0.3F, 1.0F);
        const char* label = (r.outcome == ScenarioOutcome::Pass)
                                ? "PASS"
                                : (r.outcome == ScenarioOutcome::Fail)
                                      ? "FAIL"
                                      : "ERROR";
        ImGui::TextColored(c, "%s", label);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%zu", r.planStepCount);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.2f", r.solveTimeMs);
        ImGui::TableSetColumnIndex(4);
        ImGui::TextWrapped("%s", r.reason.c_str());
      }
      ImGui::EndTable();
    }
  }
  if (!m_lastContingencyReport.contextFluents.empty()) {
    const ImVec4 red(1.0F, 0.3F, 0.3F, 1.0F);
    const ImVec4 green(0.3F, 0.9F, 0.3F, 1.0F);
    const ImVec4 orange(1.0F, 0.6F, 0.2F, 1.0F);
    ImGui::Separator();
    ImGui::TextDisabled(
        "Contingency analysis: %zu feasible / %zu infeasible / %zu error  (%zu context fluents)",
        m_lastContingencyReport.feasibleCount,
        m_lastContingencyReport.infeasibleCount,
        m_lastContingencyReport.errorCount,
        m_lastContingencyReport.contextFluents.size());
    if (!m_lastContingencyReport.error.empty()) {
      ImGui::TextColored(red, "Error: %s", m_lastContingencyReport.error.c_str());
    }
    if (ImGui::BeginTable("##contingency", 3,
                          ImGuiTableFlags_Borders |
                              ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY,
                          ImVec2(0.0F, 200.0F))) {
      ImGui::TableSetupColumn("Context (true fluents)");
      ImGui::TableSetupColumn("Outcome");
      ImGui::TableSetupColumn("Plan steps");
      ImGui::TableHeadersRow();
      for (const auto& ctx : m_lastContingencyReport.results) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        std::string contextStr;
        for (const auto& fluent : ctx.trueFluents) {
          contextStr += fluent + " ";
        }
        ImGui::TextWrapped("%s",
                           contextStr.empty() ? "(none)" : contextStr.c_str());
        ImGui::TableSetColumnIndex(1);
        if (!ctx.errorMessage.empty()) {
          ImGui::TextColored(red, "ERROR");
        } else if (ctx.planFound) {
          ImGui::TextColored(green, "OK");
        } else {
          ImGui::TextColored(orange, "NO PLAN");
        }
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%zu", ctx.planSteps);
      }
      ImGui::EndTable();
    }
  }
  ImGui::EndChild();
}

void AppShell::renderPlanTab() {
  if (!m_hasLastPlan) {
    ImGui::TextDisabled("Run Validate > Check Feasibility to populate.");
    return;
  }

  if (m_lastPlan.success) {
    ImGui::BeginChild("##PlanStats", ImVec2(0.0F, 60.0F),
                      ImGuiChildFlags_Border);
    ImGui::TextColored(ImVec4(0.2F, 0.9F, 0.3F, 1.0F),
                       "Plan found for scenario: %s",
                       m_lastPlanScenarioName.c_str());
    ImGui::Text("Steps: %zu  Cost: %.2f  Expanded: %u  Generated: %u  Time: %.2f ms",
                m_lastPlan.steps.size(), m_lastPlan.cost, m_lastPlan.expanded,
                m_lastPlan.generated, m_lastPlan.solve_time_ms);
    ImGui::EndChild();

    const int selectedStep = m_planGraph.selectedStepIndex();
    const bool showDetails =
        selectedStep >= 0 &&
        selectedStep < static_cast<int>(m_planGraph.stepCount());
    const float detailsWidth = showDetails ? 340.0F : 0.0F;
    const float canvasWidth =
        showDetails ? std::max(120.0F,
                               ImGui::GetContentRegionAvail().x - detailsWidth - 8.0F)
                    : 0.0F;

    ImGui::BeginChild("##PlanCanvas", ImVec2(canvasWidth, 0.0F),
                      ImGuiChildFlags_Border);
    m_planGraph.render();
    ImGui::EndChild();

    if (showDetails) {
      ImGui::SameLine();
      ImGui::BeginChild("##PlanStepDetails", ImVec2(detailsWidth, 0.0F),
                        ImGuiChildFlags_Border);
      const size_t stepIdx = static_cast<size_t>(selectedStep);
      ImGui::Text("Step %d", selectedStep + 1);
      ImGui::TextWrapped("%s", m_planGraph.stepLabel(stepIdx).c_str());
      ImGui::Separator();
      renderPlanFluentList("Preconditions",
                           m_planGraph.stepPreconditions(stepIdx),
                           m_planGraph);
      ImGui::Separator();
      renderPlanFluentList("Add effects",
                           m_planGraph.stepAddEffects(stepIdx),
                           m_planGraph);
      ImGui::Separator();
      renderPlanFluentList("Delete effects",
                           m_planGraph.stepDelEffects(stepIdx),
                           m_planGraph);
      ImGui::EndChild();
    }
    return;
  }

  if (!m_lastPlan.error_msg.empty()) {
    ImGui::TextColored(ImVec4(1.0F, 0.4F, 0.4F, 1.0F),
                       "Infeasible: %s",
                       m_lastPlan.error_msg.c_str());
  } else {
    ImGui::TextColored(ImVec4(1.0F, 0.4F, 0.4F, 1.0F),
                       "Infeasible: no plan exists");
  }
}

void AppShell::renderBtTab() {
  if (!m_btGraph.lastError().empty()) {
    ImGui::TextColored(ImVec4(1.0F, 0.35F, 0.35F, 1.0F),
                       "Parse error: %s",
                       m_btGraph.lastError().c_str());
  }
  ImGui::BeginChild("##BtCanvas", ImVec2(0.0F, 0.0F), ImGuiChildFlags_Border);
  m_btGraph.render();
  ImGui::EndChild();
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
        if (ImGui::InputText("##name", name, sizeof(name))) {
          const int paramIdx = pi;
          const std::string newName = name;
          m_commandStack.execute(m_model, "Rename action parameter",
                                 [selAction, paramIdx, newName](ProjectModel& model) {
            model.actions[static_cast<size_t>(selAction)]
                .params[static_cast<size_t>(paramIdx)].name = newName;
          });
        }
        ImGui::TableSetColumnIndex(1);
        if (ImGui::InputText("##type", type, sizeof(type))) {
          const int paramIdx = pi;
          const std::string newType = type;
          m_commandStack.execute(m_model, "Retype action parameter",
                                 [selAction, paramIdx, newType](ProjectModel& model) {
            model.actions[static_cast<size_t>(selAction)]
                .params[static_cast<size_t>(paramIdx)].type = newType;
          });
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
        const std::string label = authoring::formatEffectRef(ref);
        ImGui::Text("%s", label.c_str());
      }
      for (const auto& ref : action.addEffects) {
        const std::string label = "+" + authoring::formatEffectRef(ref);
        ImGui::Text("%s", label.c_str());
      }
      for (const auto& ref : action.delEffects) {
        const std::string label = "-" + authoring::formatEffectRef(ref);
        ImGui::Text("%s", label.c_str());
      }
    }

    ImGui::Separator();
    ImGui::TextDisabled("BT binding");
    static int s_lastBtSelAction = -1;
    static std::string s_lastBtNodeType;
    static std::string s_lastBtSubtreeXml;
    static char s_nodeType[64] = {};
    static char s_subtreeXml[1024] = {};
    if (selAction != s_lastBtSelAction ||
        action.btBinding.nodeType != s_lastBtNodeType ||
        action.btBinding.subtreeXml != s_lastBtSubtreeXml) {
      std::snprintf(s_nodeType,
                    sizeof(s_nodeType),
                    "%s",
                    action.btBinding.nodeType.c_str());
      std::snprintf(s_subtreeXml,
                    sizeof(s_subtreeXml),
                    "%s",
                    action.btBinding.subtreeXml.c_str());
      s_lastBtSelAction = selAction;
      s_lastBtNodeType = action.btBinding.nodeType;
      s_lastBtSubtreeXml = action.btBinding.subtreeXml;
    }

    if (ImGui::InputText("Node type##bt", s_nodeType, sizeof(s_nodeType))) {
      const std::string nodeType = s_nodeType;
      m_commandStack.execute(m_model, "Set BT node type",
                             [selAction, nodeType](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].btBinding.nodeType =
            nodeType;
      });
      s_lastBtNodeType = nodeType;
    }
    bool reactive = action.btBinding.reactive;
    if (ImGui::Checkbox("Reactive##bt", &reactive)) {
      m_commandStack.execute(m_model, "Set BT reactive binding",
                             [selAction, reactive](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].btBinding.reactive =
            reactive;
      });
    }
    if (ImGui::InputTextMultiline("Subtree XML##bt",
                                  s_subtreeXml,
                                  sizeof(s_subtreeXml),
                                  ImVec2(-FLT_MIN, 120.0F))) {
      const std::string subtreeXml = s_subtreeXml;
      m_commandStack.execute(m_model, "Set BT subtree XML",
                             [selAction, subtreeXml](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].btBinding.subtreeXml =
            subtreeXml;
      });
      s_lastBtSubtreeXml = subtreeXml;
    }
    if (!action.btBinding.subtreeXml.empty() && !action.params.empty()) {
      ImGui::TextDisabled("Resolved preview (first grounding):");
      std::vector<std::string> sampleArgs;
      for (size_t i = 0; i < action.params.size(); ++i) {
        sampleArgs.push_back("obj" + std::to_string(i + 1));
      }
      std::string resolved = action.btBinding.subtreeXml;
      for (size_t i = 0; i < sampleArgs.size(); ++i) {
        const std::string placeholder = "{param" + std::to_string(i) + "}";
        size_t pos = 0;
        while ((pos = resolved.find(placeholder, pos)) != std::string::npos) {
          resolved.replace(pos, placeholder.size(), sampleArgs[i]);
          pos += sampleArgs[i].size();
        }
      }
      ImGui::TextWrapped("%s", resolved.c_str());
    }
  }
}

void AppShell::clearDerivedResults() {
  m_lastValidation = ValidationReport{};
  m_structuralReport = StructuralReport{};
  m_lastBatchReport = ScenarioBatchReport{};
  m_lastContingencyReport = ContingencyReport{};
  m_lastPlan = ame::PlanResult{};
  m_lastPlanScenarioName.clear();
  m_lastPlanStepLabels.clear();
  m_requestedTab.clear();
  m_planGraph.clear();
  m_btGraph.setXml("");
  m_hasLastPlan = false;
  m_domainGraph.setHighlightedElements({}, {});
  m_domainGraph.setStructuralHighlights({}, {}, {}, {});
  validationState = "Not validated";
}

void AppShell::resetScenarioEditorState() {
  m_scenarioNameInput[0] = '\0';
  m_renameScenarioNameInput[0] = '\0';
  m_renameScenarioIdx = -2;
  m_renameScenarioSource.clear();
  m_initPredIdx = 0;
  m_goalPredIdx = 0;
  m_initArgsInput[0] = '\0';
  m_goalArgsInput[0] = '\0';
}

void AppShell::selfTestNew() {
  m_commandStack.clear();
  projectName     = "[Untitled]";
  lastOperation   = "New project";
  m_selectedScenarioIdx = -1;
  clearDerivedResults();
  resetScenarioEditorState();
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

void AppShell::selfTestSetActionBtBinding(int actionIdx,
                                          std::string nodeType,
                                          std::string subtreeXml,
                                          bool reactive) {
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return;
  }
  m_commandStack.execute(m_model,
                         "Set action BT binding",
                         [actionIdx, nodeType, subtreeXml, reactive](ProjectModel& model) {
    BtBinding& binding =
        model.actions[static_cast<size_t>(actionIdx)].btBinding;
    binding.nodeType = nodeType;
    binding.subtreeXml = subtreeXml;
    binding.reactive = reactive;
  });
}

void AppShell::selfTestAddObject(const std::string& name, const std::string& type) {
  m_commandStack.execute(m_model, "Add object", [name, type](ProjectModel& model) {
    model.objects.push_back({name, type});
  });
}

void AppShell::selfTestAddScenario(const std::string& name) {
  m_commandStack.execute(m_model, "Add scenario", [name](ProjectModel& model) {
    ScenarioDef scenario;
    scenario.name = name;
    model.scenarios.push_back(std::move(scenario));
  });
  m_selectedScenarioIdx = static_cast<int>(m_model.scenarios.size()) - 1;
}

void AppShell::selfTestAddInitialFact(int scenarioIdx,
                                      const std::string& predicateName,
                                      std::vector<std::string> objectNames) {
  if (scenarioIdx < 0 || scenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Add initial fact",
                         [scenarioIdx, predicateName, objectNames](ProjectModel& model) {
    model.scenarios[static_cast<size_t>(scenarioIdx)].initialState.push_back({
      predicateName,
      objectNames
    });
  });
}

void AppShell::selfTestAddGoal(int scenarioIdx,
                               const std::string& predicateName,
                               std::vector<std::string> objectNames) {
  if (scenarioIdx < 0 || scenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    return;
  }
  m_commandStack.execute(m_model, "Add goal",
                         [scenarioIdx, predicateName, objectNames](ProjectModel& model) {
    model.scenarios[static_cast<size_t>(scenarioIdx)].goals.push_back({
      predicateName,
      objectNames
    });
  });
}

void AppShell::selfTestSetScenarioExpectation(int scenarioIdx,
                                              bool shouldSucceed,
                                              int minSteps,
                                              int maxSteps,
                                              std::vector<std::string> expected,
                                              std::vector<std::string> forbidden) {
  if (scenarioIdx < 0 || scenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    return;
  }
  minSteps = std::max(0, minSteps);
  maxSteps = std::max(0, maxSteps);
  m_commandStack.execute(m_model, "Set scenario expectation",
                         [scenarioIdx,
                          shouldSucceed,
                          minSteps,
                          maxSteps,
                          expected,
                          forbidden](ProjectModel& model) {
    ScenarioExpectation& expectation =
        model.scenarios[static_cast<size_t>(scenarioIdx)].expectation;
    expectation.shouldSucceed = shouldSucceed;
    expectation.minPlanSteps = minSteps;
    expectation.maxPlanSteps = maxSteps;
    expectation.expectedActions = expected;
    expectation.forbiddenActions = forbidden;
  });
}

const ScenarioExpectation& AppShell::selfTestScenarioExpectation(int scenarioIdx) const {
  static const ScenarioExpectation kDefaultExpectation;
  if (scenarioIdx < 0 || scenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    return kDefaultExpectation;
  }
  return m_model.scenarios[static_cast<size_t>(scenarioIdx)].expectation;
}

void AppShell::selfTestRunFeasibility(const std::string& scenarioName) {
  const auto it = std::find_if(m_model.scenarios.begin(), m_model.scenarios.end(),
                               [&scenarioName](const ScenarioDef& scenario) {
                                 return scenario.name == scenarioName;
                               });
  if (it == m_model.scenarios.end()) {
    m_selectedScenarioIdx = -1;
  } else {
    m_selectedScenarioIdx =
        static_cast<int>(std::distance(m_model.scenarios.begin(), it));
  }
  runFeasibilityCheck();
}

void AppShell::selfTestRunAllScenarios() {
  runAllScenarios();
}

void AppShell::selfTestRunContingencyAnalysis() {
  runContingencyAnalysis();
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

void AppShell::selfTestRemoveAllObjects() {
  m_commandStack.execute(m_model, "Remove all objects", [](ProjectModel& model) {
    model.objects.clear();
  });
}

void AppShell::selfTestPlanAndPreview() {
  runPlanAndPreview();
}

void AppShell::selfTestSetSelectedPlanStep(int idx) {
  m_planGraph.setSelectedStepForTest(idx);
}

bool AppShell::selfTestImportDomain(const std::string& pddl) {
  const PddlImportResult import = PddlImporter::importDomain(pddl);
  if (!import.ok) {
    lastOperation = "Import failed: " + import.error;
    return false;
  }

  m_commandStack.clear();
  m_model = import.model;
  projectName = m_model.projectName;
  lastOperation = "Imported domain: " + projectName;
  m_selectedScenarioIdx = -1;
  clearDerivedResults();
  resetScenarioEditorState();
  return true;
}

bool AppShell::selfTestImportProblem(const std::string& pddl,
                                     const std::string& scenarioName) {
  const PddlImportResult import =
      PddlImporter::importProblem(m_model, pddl, scenarioName);
  if (!import.ok) {
    lastOperation = "Import failed: " + import.error;
    return false;
  }

  m_model = import.model;
  m_selectedScenarioIdx = static_cast<int>(m_model.scenarios.size()) - 1;
  const std::string importedScenario =
      m_model.scenarios.empty() ? "" : m_model.scenarios.back().name;
  lastOperation = "Imported problem: " + importedScenario;
  clearDerivedResults();
  resetScenarioEditorState();
  return true;
}

size_t AppShell::selfTestUndoDepth() const {
  return m_commandStack.undoDepth();
}

const BtBinding& AppShell::selfTestActionBtBinding(int actionIdx) const {
  static const BtBinding kDefaultBinding;
  if (actionIdx < 0 || actionIdx >= static_cast<int>(m_model.actions.size())) {
    return kDefaultBinding;
  }
  return m_model.actions[static_cast<size_t>(actionIdx)].btBinding;
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

void AppShell::runFeasibilityCheck() {
  m_structuralReport = StructuralValidator::check(m_model);
  if (m_structuralReport.hasErrors()) {
    lastOperation = "Cannot plan: fix structural errors first";
    m_btGraph.setXml("");
    return;
  }

  if (m_selectedScenarioIdx < 0 ||
      m_selectedScenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    lastOperation = "No scenario selected";
    m_btGraph.setXml("");
    return;
  }

  const std::string scenarioName =
      m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)].name;
  ame::WorldModel wm;
  m_lastValidation =
      PddlValidator::validateAndBuildWorldModel(m_model, scenarioName, wm);
  if (!m_lastValidation.ok) {
    validationState = "Parse failed - cannot plan";
    m_lastPlan = ame::PlanResult{};
    m_lastPlan.error_msg = "parse failed - cannot plan";
    m_lastPlanScenarioName = scenarioName;
    m_lastPlanStepLabels.clear();
    m_planGraph.clear();
    m_btGraph.setXml("");
    m_hasLastPlan = true;
    return;
  }

  ame::Planner planner;
  m_lastPlan = planner.solve(wm);
  m_lastPlanScenarioName = scenarioName;
  m_lastPlanStepLabels.clear();
  for (const auto& step : m_lastPlan.steps) {
    if (step.action_index < wm.groundActions().size()) {
      m_lastPlanStepLabels.push_back(wm.groundActions()[step.action_index].signature);
    } else {
      m_lastPlanStepLabels.push_back("action #" + std::to_string(step.action_index));
    }
  }
  m_hasLastPlan = true;

  if (m_lastPlan.success) {
    m_planGraph.setPlan(m_lastPlan, wm, scenarioName);
    validationState =
        "Feasible (" + std::to_string(m_lastPlan.steps.size()) + " steps)";
  } else {
    m_planGraph.clear();
    const std::string error =
        m_lastPlan.error_msg.empty() ? "no plan exists" : m_lastPlan.error_msg;
    validationState = "Infeasible: " + error;
  }
  lastOperation = "Checked feasibility: " + scenarioName;
  compileAndShowBt();
}

void AppShell::runAllScenarios() {
  m_lastBatchReport = ScenarioRunner::runAll(m_model);
  const size_t total =
      m_lastBatchReport.passCount + m_lastBatchReport.failCount +
      m_lastBatchReport.errorCount;
  validationState = "Scenarios: " + std::to_string(m_lastBatchReport.passCount) +
                    "/" + std::to_string(total) + " passed";
  lastOperation = "Ran " + std::to_string(m_lastBatchReport.results.size()) +
                  " scenarios";
}

void AppShell::runContingencyAnalysis() {
  if (m_selectedScenarioIdx < 0 ||
      m_selectedScenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    lastOperation = "No scenario selected";
    return;
  }

  const std::string scenarioName =
      m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)].name;
  m_lastContingencyReport =
      ContingencyAnalyser::analyse(m_model, scenarioName);
  validationState = "Contingency: " +
                    std::to_string(m_lastContingencyReport.feasibleCount) +
                    " feasible, " +
                    std::to_string(m_lastContingencyReport.infeasibleCount) +
                    " infeasible, " +
                    std::to_string(m_lastContingencyReport.errorCount) +
                    " error";
  lastOperation = "Ran contingency analysis on: " + scenarioName;
}

void AppShell::runPlanAndPreview() {
  runFeasibilityCheck();
  m_requestedTab = "Plan";
  const std::string successOperation = "Checked feasibility: " + m_lastPlanScenarioName;
  if (m_hasLastPlan && m_lastPlan.success && lastOperation == successOperation) {
    lastOperation = "Planned & previewed: " + m_lastPlanScenarioName;
  }
}

void AppShell::compileAndShowBt() {
  if (!m_lastPlan.success) {
    lastOperation = "No plan to compile";
    m_btGraph.setXml("");
    return;
  }

  ame::WorldModel wm;
  const ValidationReport report =
      PddlValidator::validateAndBuildWorldModel(m_model, m_lastPlanScenarioName, wm);
  if (!report.ok) {
    m_btGraph.setXml("");
    lastOperation = "BT compile failed: scenario validation failed";
    return;
  }

  try {
    ame::ActionRegistry registry;
    for (const auto& act : m_model.actions) {
      if (!act.btBinding.subtreeXml.empty()) {
        registry.registerActionSubTree(act.name,
                                       act.btBinding.subtreeXml,
                                       act.btBinding.reactive);
      } else if (!act.btBinding.nodeType.empty()) {
        registry.registerAction(act.name,
                                act.btBinding.nodeType,
                                act.btBinding.reactive);
      }
    }
    ame::PlanCompiler compiler;
    const std::string xml = compiler.compile(m_lastPlan.steps, wm, registry);
    m_btGraph.setXml(xml);
    if (!m_btGraph.lastError().empty()) {
      lastOperation = "BT parse failed: " + m_btGraph.lastError();
    }
  } catch (const std::exception& ex) {
    m_btGraph.setXml("");
    lastOperation = std::string("BT compile failed: ") + ex.what();
  }
}

// Draw a small framed "HUD pill" — filled background tinted from the border
// colour, 1px outline in `borderColor`, text inside in `textColor`. Advances
// the ImGui cursor horizontally (call before the next pill, no SameLine needed).
// Draws a 3px cyan vertical bar overlaid on the upcoming line — call
// just before a CollapsingHeader to give it the HUD section-marker look.
// Does NOT touch indent state, so it composes safely.
static void SectionAccent() {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float h = ImGui::GetFrameHeight();
  dl->AddRectFilled(ImVec2(p.x - 4.0F, p.y),
                    ImVec2(p.x - 1.0F, p.y + h),
                    ImGui::GetColorU32(ImVec4(0.0F, 0.85F, 1.0F, 1.0F)));
}

static void StatusPill(const char* text, ImVec4 borderColor, ImVec4 textColor) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 padding(8.0F, 2.0F);
  const ImVec2 textSize = ImGui::CalcTextSize(text);
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const ImVec2 rectMax(pos.x + textSize.x + padding.x * 2.0F,
                       pos.y + textSize.y + padding.y * 2.0F);
  const ImVec4 bg(borderColor.x * 0.15F, borderColor.y * 0.15F,
                  borderColor.z * 0.15F, 1.0F);
  dl->AddRectFilled(pos, rectMax, ImGui::GetColorU32(bg));
  dl->AddRect(pos, rectMax, ImGui::GetColorU32(borderColor), 0.0F, 0, 1.0F);
  ImGui::SetCursorScreenPos(ImVec2(pos.x + padding.x, pos.y + padding.y));
  ImGui::TextColored(textColor, "%s", text);
  ImGui::SetCursorScreenPos(ImVec2(rectMax.x + 6.0F, pos.y));
}

void AppShell::renderStatusBar() {
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  constexpr float kStatusBarHeight = 26.0F;  // slightly taller for pill padding
  const ImGuiWindowFlags flags =
    ImGuiWindowFlags_NoDecoration |
    ImGuiWindowFlags_NoInputs |
    ImGuiWindowFlags_NoNav |
    ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_NoSavedSettings |
    ImGuiWindowFlags_NoBringToFrontOnFocus |
    ImGuiWindowFlags_NoScrollbar;

  // Cyan hairline immediately above the status bar (mirrors the menu-bar line).
  ImDrawList* fg = ImGui::GetForegroundDrawList();
  const float topY = viewport->Pos.y + viewport->Size.y - kStatusBarHeight;
  fg->AddLine(ImVec2(viewport->Pos.x, topY - 0.5F),
              ImVec2(viewport->Pos.x + viewport->Size.x, topY - 0.5F),
              ImGui::GetColorU32(ImVec4(0.0F, 0.85F, 1.0F, 0.55F)),
              1.0F);

  ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x, topY));
  ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kStatusBarHeight));
  ImGui::SetNextWindowBgAlpha(0.85F);

  ImGui::Begin("##StatusBar", nullptr, flags);
  // Pull cursor up a couple px so pills sit nicely centered in the strip.
  ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2.0F);

  const ImVec4 cyan(0.0F, 0.85F, 1.0F, 1.0F);
  const ImVec4 ok(0.2F, 0.9F, 0.4F, 1.0F);
  const ImVec4 warn(1.0F, 0.7F, 0.2F, 1.0F);
  const ImVec4 err(1.0F, 0.35F, 0.35F, 1.0F);
  const ImVec4 dim(0.55F, 0.7F, 0.8F, 1.0F);

  // Project name pill (always cyan)
  StatusPill(projectName.c_str(), cyan, cyan);

  // Validation state pill — colour reflects outcome
  ImVec4 vColor = dim;
  if (validationState == "Valid" ||
      validationState.rfind("Feasible", 0U) == 0U) {
    vColor = ok;
  } else if (validationState.find("error") != std::string::npos ||
             validationState.find("Infeasible") != std::string::npos) {
    vColor = err;
  }
  StatusPill(validationState.c_str(), vColor, vColor);

  // Structural issue count pill (only when non-zero)
  const size_t structuralIssueCount =
      m_structuralReport.errorCount + m_structuralReport.warningCount;
  if (structuralIssueCount > 0U) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%zu issue%s",
                  structuralIssueCount,
                  structuralIssueCount == 1U ? "" : "s");
    const ImVec4 issueColor = m_structuralReport.hasErrors() ? err : warn;
    StatusPill(buf, issueColor, issueColor);
  }

  // Last operation pill (dim — informational)
  StatusPill(lastOperation.c_str(), dim, dim);

  ImGui::End();
}
