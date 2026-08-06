#include "app_shell.h"

#include "authoring_utils.h"
#include "guided_editor_model.h"
#include "imgui.h"
#include "pddl_generator.h"
#include "pddl_importer.h"

#include <tinyfiledialogs.h>

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

// ---------------------------------------------------------------------------
// Native file-dialog helpers (tinyfiledialogs). Return empty string if the
// user cancels. The const-char* return from tinyfd points to its own static
// buffer; copy immediately into std::string so it survives.
// ---------------------------------------------------------------------------
static std::string pickOpenFile(const char* title,
                                const std::string& defaultPath,
                                const char* filterPattern,
                                const char* filterDesc) {
  const char* patterns[1] = { filterPattern };
  const char* p = tinyfd_openFileDialog(title, defaultPath.c_str(),
                                        1, patterns, filterDesc, 0);
  return p != nullptr ? std::string(p) : std::string();
}

static std::string pickSaveFile(const char* title,
                                const std::string& defaultPath,
                                const char* filterPattern,
                                const char* filterDesc) {
  const char* patterns[1] = { filterPattern };
  const char* p = tinyfd_saveFileDialog(title, defaultPath.c_str(),
                                        1, patterns, filterDesc);
  return p != nullptr ? std::string(p) : std::string();
}

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

static const PredicateDef* predicateByName(const ProjectModel& model,
                                           const std::string& name) {
  const auto found = std::find_if(model.predicates.begin(), model.predicates.end(),
                                  [&name](const PredicateDef& predicate) {
                                    return predicate.name == name;
                                  });
  return found == model.predicates.end() ? nullptr : &*found;
}

static std::vector<EffectRef>& actionReferences(ActionDef& action,
                                                PredicateRelationKind kind) {
  if (kind == PredicateRelationKind::Requires) {
    return action.preconditions;
  }
  if (kind == PredicateRelationKind::MakesTrue) {
    return action.addEffects;
  }
  return action.delEffects;
}

static const std::vector<EffectRef>& actionReferences(const ActionDef& action,
                                                      PredicateRelationKind kind) {
  if (kind == PredicateRelationKind::Requires) {
    return action.preconditions;
  }
  if (kind == PredicateRelationKind::MakesTrue) {
    return action.addEffects;
  }
  return action.delEffects;
}

static bool predicateHasLegalArguments(const ProjectModel& model,
                                       const ActionDef& action,
                                       const PredicateDef& predicate) {
  const auto choices = guidedPredicateChoices(model, action);
  const auto found = std::find_if(
      choices.begin(), choices.end(), [&predicate](const GuidedEditorChoice& choice) {
        return choice.name == predicate.name;
      });
  return found != choices.end() && found->legal;
}

static EffectRef defaultReference(const ProjectModel& model,
                                  const ActionDef& action,
                                  const PredicateDef& predicate) {
  return makeGuidedReference(model, action, predicate);
}

static void renderGuidedReferenceGroup(const char* visibleTitle,
                                       const char* tableId,
                                       const char* addLabel,
                                       const char* rowVerb,
                                       const char* outcome,
                                       int actionIndex,
                                       PredicateRelationKind kind,
                                       ProjectModel& model,
                                       CommandStack& stack) {
  ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F), "%s", visibleTitle);
  ActionDef& action = model.actions[static_cast<size_t>(actionIndex)];
  std::vector<EffectRef>& refs = actionReferences(action, kind);
  for (int reference_index = 0;
       reference_index < static_cast<int>(refs.size());
       ++reference_index) {
    ImGui::PushID(reference_index);
    EffectRef& reference = refs[static_cast<size_t>(reference_index)];
    const PredicateDef* predicate = predicateByName(model, reference.predicateName);

    if (!reference.argNames.empty()) {
      ImGui::SetNextItemWidth(90.0F);
      if (ImGui::BeginCombo("##subject", reference.argNames.front().c_str())) {
        const std::string expected =
            predicate != nullptr && !predicate->params.empty()
                ? predicate->params.front().type : std::string{};
        for (const Parameter& parameter : action.params) {
          const bool legal = guidedTypeCompatible(model, parameter.type, expected);
          if (ImGui::Selectable(parameter.name.c_str(),
                                parameter.name == reference.argNames.front(),
                                legal ? ImGuiSelectableFlags_None
                                      : ImGuiSelectableFlags_Disabled)) {
            const std::string argument = parameter.name;
            stack.execute(model, "Choose guided fact name",
                          [actionIndex, reference_index, kind, argument](ProjectModel& target) {
              actionReferences(target.actions[static_cast<size_t>(actionIndex)], kind)
                  [static_cast<size_t>(reference_index)].argNames[0] = argument;
            });
          }
          if (!legal) {
            ImGui::SameLine();
            ImGui::TextDisabled("needs a %s", expected.c_str());
          }
        }
        ImGui::EndCombo();
      }
      ImGui::SameLine();
    }
    ImGui::TextUnformatted(rowVerb);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(145.0F);
    if (ImGui::BeginCombo("##predicate", reference.predicateName.c_str())) {
      for (const PredicateDef& candidate : model.predicates) {
        const bool legal = predicateHasLegalArguments(model, action, candidate);
        if (ImGui::Selectable(candidate.name.c_str(),
                              candidate.name == reference.predicateName,
                              legal ? ImGuiSelectableFlags_None
                                    : ImGuiSelectableFlags_Disabled)) {
          const EffectRef replacement = defaultReference(model, action, candidate);
          stack.execute(model, "Choose guided fact",
                        [actionIndex, reference_index, kind, replacement](ProjectModel& target) {
            actionReferences(target.actions[static_cast<size_t>(actionIndex)], kind)
                [static_cast<size_t>(reference_index)] = replacement;
          });
        }
        if (!legal) {
          ImGui::SameLine();
          const std::string applies_to = candidate.params.empty()
              ? "no object" : candidate.params.front().type;
          ImGui::TextDisabled("applies to a %s", applies_to.c_str());
        }
      }
      ImGui::EndCombo();
    }

    predicate = predicateByName(model, reference.predicateName);
    for (size_t argument_index = 1;
         argument_index < reference.argNames.size();
         ++argument_index) {
      ImGui::SameLine();
      ImGui::SetNextItemWidth(90.0F);
      const std::string expected =
          predicate != nullptr && argument_index < predicate->params.size()
              ? predicate->params[argument_index].type : std::string{};
      const std::string combo_id = "##arg" + std::to_string(argument_index);
      if (ImGui::BeginCombo(combo_id.c_str(), reference.argNames[argument_index].c_str())) {
        for (const Parameter& parameter : action.params) {
          const bool legal = guidedTypeCompatible(model, parameter.type, expected);
          if (ImGui::Selectable(parameter.name.c_str(),
                                parameter.name == reference.argNames[argument_index],
                                legal ? ImGuiSelectableFlags_None
                                      : ImGuiSelectableFlags_Disabled)) {
            const std::string argument = parameter.name;
            stack.execute(model, "Choose guided fact name",
                          [actionIndex, reference_index, argument_index,
                           kind, argument](ProjectModel& target) {
              actionReferences(target.actions[static_cast<size_t>(actionIndex)], kind)
                  [static_cast<size_t>(reference_index)].argNames[argument_index] = argument;
            });
          }
          if (!legal) {
            ImGui::SameLine();
            ImGui::TextDisabled("needs a %s", expected.c_str());
          }
        }
        ImGui::EndCombo();
      }
    }
    if (outcome != nullptr && outcome[0] != '\0') {
      ImGui::SameLine();
      const ImVec4 colour = kind == PredicateRelationKind::MakesTrue
          ? ImVec4(0.32F, 0.84F, 0.60F, 1.0F)
          : ImVec4(0.95F, 0.51F, 0.42F, 1.0F);
      ImGui::TextColored(colour, "%s", outcome);
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Remove")) {
      stack.execute(model, "Remove guided fact",
                    [actionIndex, reference_index, kind](ProjectModel& target) {
        std::vector<EffectRef>& target_refs =
            actionReferences(target.actions[static_cast<size_t>(actionIndex)], kind);
        target_refs.erase(target_refs.begin() + reference_index);
      });
      ImGui::PopID();
      --reference_index;
      continue;
    }
    ImGui::PopID();
  }

  ImGui::PushID(tableId);
  static int selected_predicates[3] = {0, 0, 0};
  const int kind_index = static_cast<int>(kind);
  int& selected_predicate = selected_predicates[kind_index];
  if (!model.predicates.empty()) {
    selected_predicate = std::max(0, std::min(
        selected_predicate, static_cast<int>(model.predicates.size()) - 1));
    const PredicateDef& selected = model.predicates[static_cast<size_t>(selected_predicate)];
    ImGui::SetNextItemWidth(145.0F);
    if (ImGui::BeginCombo("##new-guided-fact", selected.name.c_str())) {
      for (int i = 0; i < static_cast<int>(model.predicates.size()); ++i) {
        const PredicateDef& candidate = model.predicates[static_cast<size_t>(i)];
        const bool legal = predicateHasLegalArguments(model, action, candidate);
        if (ImGui::Selectable(candidate.name.c_str(), i == selected_predicate,
                              legal ? ImGuiSelectableFlags_None
                                    : ImGuiSelectableFlags_Disabled)) {
          selected_predicate = i;
        }
        if (!legal) {
          ImGui::SameLine();
          ImGui::TextDisabled("applies to a %s",
                              candidate.params.empty()
                                  ? "fact with no object"
                                  : candidate.params.front().type.c_str());
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    const bool legal = predicateHasLegalArguments(model, action, selected);
    if (!legal) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button(addLabel)) {
      const EffectRef reference = defaultReference(model, action, selected);
      stack.execute(model, "Add guided fact",
                    [actionIndex, kind, reference](ProjectModel& target) {
        actionReferences(target.actions[static_cast<size_t>(actionIndex)], kind)
            .push_back(reference);
      });
    }
    if (!legal) {
      ImGui::EndDisabled();
    }
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
        const std::string defaultPath =
            authoring::projectFilePath(m_model.projectName);
        const std::string path = pickOpenFile("Open AME project",
                                              defaultPath,
                                              "*.ameproj.json",
                                              "AME project (*.ameproj.json)");
        if (path.empty()) {
          lastOperation = "Open cancelled";
        } else {
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
      }
      if (ImGui::MenuItem("Save")) {
        const std::string defaultPath =
            authoring::projectFilePath(m_model.projectName);
        const std::string path = pickSaveFile("Save AME project",
                                              defaultPath,
                                              "*.ameproj.json",
                                              "AME project (*.ameproj.json)");
        if (path.empty()) {
          lastOperation = "Save cancelled";
        } else {
          lastOperation = m_model.save(path) ? "Saved " + path
                                              : "Failed to save " + path;
          if (m_autoValidateOnSave) {
            runValidation();
          }
        }
      }
      if (ImGui::MenuItem("Save As...")) {
        const std::string defaultPath =
            authoring::projectFilePath(m_model.projectName);
        const std::string path = pickSaveFile("Save AME project as",
                                              defaultPath,
                                              "*.ameproj.json",
                                              "AME project (*.ameproj.json)");
        if (path.empty()) {
          lastOperation = "Save As cancelled";
        } else {
          lastOperation = m_model.save(path) ? "Saved as " + path
                                              : "Failed to save as " + path;
          if (m_autoValidateOnSave) {
            runValidation();
          }
        }
      }
      if (ImGui::MenuItem("Import PDDL Domain...")) {
        const std::string path = pickOpenFile("Import PDDL domain",
                                              authoring::importDomainPath(),
                                              "*.pddl",
                                              "PDDL domain (*.pddl)");
        if (path.empty()) {
          lastOperation = "Import cancelled";
        } else {
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
      }
      if (ImGui::MenuItem("Import PDDL Problem...")) {
        const std::string path = pickOpenFile("Import PDDL problem",
                                              authoring::importProblemPath(),
                                              "*.pddl",
                                              "PDDL problem (*.pddl)");
        if (path.empty()) {
          lastOperation = "Import cancelled";
        } else {
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
      }
      if (ImGui::MenuItem("Export Domain PDDL...")) {
        m_structuralReport = StructuralValidator::check(m_model);
        if (m_structuralReport.hasErrors()) {
          lastOperation =
              "Refusing: " + std::to_string(m_structuralReport.errorCount) +
              " structural error(s)";
        } else {
          const std::string path = pickSaveFile("Export PDDL domain",
                                                authoring::domainPddlPath(m_model.projectName),
                                                "*.pddl",
                                                "PDDL domain (*.pddl)");
          if (path.empty()) {
            lastOperation = "Export cancelled";
          } else {
            std::ofstream file(path);
            file << PddlGenerator::generateDomain(m_model);
            lastOperation = file.good() ? "Wrote " + path : "Failed to write " + path;
          }
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
          const std::string defaultPath =
              authoring::problemPddlPath(m_model.projectName,
                                         m_model.scenarios.front().name);
          const std::string path = pickSaveFile("Export PDDL problem",
                                                defaultPath,
                                                "*.pddl",
                                                "PDDL problem (*.pddl)");
          if (path.empty()) {
            lastOperation = "Export cancelled";
          } else {
            std::ofstream file(path);
            file << PddlGenerator::generateProblem(m_model, m_model.scenarios.front().name);
            lastOperation = file.good() ? "Wrote " + path : "Failed to write " + path;
          }
        }
      }
      if (ImGui::MenuItem("Export Regression Report...")) {
        if (m_lastBatchReport.results.empty()) {
          lastOperation = "No batch report to export";
        } else {
          const std::string defaultPath =
              authoring::regressionReportPath(m_model.projectName);
          const std::string path = pickSaveFile("Export regression report",
                                                defaultPath,
                                                "*.json",
                                                "JSON report (*.json)");
          if (path.empty()) {
            lastOperation = "Export cancelled";
          } else {
            std::ofstream file(path);
            file << ScenarioRunner::toJson(m_lastBatchReport);
            lastOperation = file.good() ? "Wrote " + path : "Failed to write " + path;
          }
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
  // and the status-bar overlay carve out the top and bottom margins. The
  // height must match kStatusBarHeight in renderStatusBar() — keep them in
  // sync via the same constant.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float menuH = ImGui::GetFrameHeight();
  constexpr float kStatusBarHeight = 26.0F;
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
  ImGui::BeginChild("##DomainSidebar", ImVec2(520.0F, 0.0F),
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

  // Right area: the five complementary navigation and review views.
  ImGui::BeginChild("##DomainCanvas", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border);
  if (m_domainGraph.canGoBack()) {
    if (ImGui::Button("< back")) {
      m_domainGraph.goBack();
    }
  } else {
    ImGui::BeginDisabled();
    ImGui::Button("< back");
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  if (m_domainGraph.canGoForward()) {
    if (ImGui::Button("forward >")) {
      m_domainGraph.goForward();
    }
  } else {
    ImGui::BeginDisabled();
    ImGui::Button("forward >");
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  static constexpr const char* kDomainViews[] = {
      "Neighbourhood", "Relations", "Matrix", "Lifecycles", "Whole domain"};
  ImGui::SetNextItemWidth(150.0F);
  ImGui::Combo("View##domain", &m_domainViewMode, kDomainViews,
               static_cast<int>(std::size(kDomainViews)));
  ImGui::Separator();

  const RelationIndex relation_index(m_model);
  if (m_domainViewMode == 0) {
    m_domainViewsRendered[0] = true;
    ImGui::SetNextItemWidth(90.0F);
    ImGui::SliderInt("steps", &m_neighbourDepth, 1, 2);
    ImGui::SameLine();
    bool show_requires = (m_neighbourFilter & ShowRequires) != 0U;
    bool show_true = (m_neighbourFilter & ShowMakesTrue) != 0U;
    bool show_false = (m_neighbourFilter & ShowMakesFalse) != 0U;
    if (ImGui::Checkbox("needs it", &show_requires)) {
      m_neighbourFilter = show_requires
          ? m_neighbourFilter | ShowRequires
          : m_neighbourFilter & ~ShowRequires;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("makes true", &show_true)) {
      m_neighbourFilter = show_true
          ? m_neighbourFilter | ShowMakesTrue
          : m_neighbourFilter & ~ShowMakesTrue;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("makes false", &show_false)) {
      m_neighbourFilter = show_false
          ? m_neighbourFilter | ShowMakesFalse
          : m_neighbourFilter & ~ShowMakesFalse;
    }
    m_domainGraph.renderFocused(m_model, relation_index, m_neighbourDepth,
                                m_neighbourFilter, 20U);
  } else if (m_domainViewMode == 1) {
    m_domainViewsRendered[1] = true;
    renderRelationsPanel();
  } else if (m_domainViewMode == 2) {
    m_domainViewsRendered[2] = true;
    renderMatrixPanel();
  } else if (m_domainViewMode == 3) {
    m_domainViewsRendered[3] = true;
    renderLifecyclePanel();
  } else {
    m_domainViewsRendered[4] = true;
    ImGui::TextDisabled("Derived relationships are read-only. Zoom out for compact nodes.");
    m_domainGraph.render(m_model, m_commandStack);
  }
  ImGui::EndChild();
}

void AppShell::renderRelationsPanel() {
  const RelationIndex index(m_model);
  const ImVec4 amber(0.88F, 0.69F, 0.32F, 1.0F);
  const ImVec4 green(0.32F, 0.84F, 0.60F, 1.0F);
  const ImVec4 red(0.95F, 0.51F, 0.42F, 1.0F);

  ImGui::TextDisabled("Selected");
  const auto& history = m_domainGraph.selectionHistory();
  for (size_t i = 0; i < history.size(); ++i) {
    if (i > 0) {
      ImGui::SameLine();
      ImGui::TextDisabled(">");
      ImGui::SameLine();
    }
    const DomainElementRef& element = history[i];
    const std::string label = element.kind == DomainElementKind::Predicate
        ? (element.index < m_model.predicates.size()
               ? m_model.predicates[element.index].name : "?")
        : (element.index < m_model.actions.size()
               ? m_model.actions[element.index].name : "?");
    ImGui::TextColored(static_cast<int>(i) == m_domainGraph.historyPosition()
                           ? ImVec4(0.0F, 0.9F, 1.0F, 1.0F)
                           : ImVec4(0.39F, 0.51F, 0.59F, 1.0F),
                       "%s", label.c_str());
  }
  ImGui::Separator();

  const int selected_predicate = m_domainGraph.selectedPredicateIndex();
  if (selected_predicate >= 0 &&
      selected_predicate < static_cast<int>(m_model.predicates.size())) {
    const PredicateDef& predicate =
        m_model.predicates[static_cast<size_t>(selected_predicate)];
    const PredicateRelations& relations =
        index.predicate(static_cast<size_t>(selected_predicate));
    ImGui::Text("fact  %s", formatPredicateSignature(predicate).c_str());

    const auto render_action_list = [&](const char* title,
                                        const std::vector<PredicateActionRelation>& entries,
                                        ImVec4 colour,
                                        const char* badge) {
      ImGui::TextColored(colour, "%s  %zu", title, entries.size());
      for (const auto& entry : entries) {
        const ActionDef& action = m_model.actions[entry.actionIndex];
        ImGui::PushID(static_cast<int>(entry.actionIndex * 10U + entry.referenceIndex));
        ImGui::TextColored(colour, "%s", badge);
        ImGui::SameLine();
        if (ImGui::Selectable(action.name.c_str())) {
          m_domainGraph.setSelectedAction(static_cast<int>(entry.actionIndex));
        }
        ImGui::SameLine();
        std::string reason;
        for (const EffectRef& condition : action.preconditions) {
          if (condition.predicateName != predicate.name) {
            reason = "also needs " + condition.predicateName;
            break;
          }
        }
        if (reason.empty() && entry.kind == PredicateRelationKind::MakesFalse &&
            std::any_of(relations.requiredBy.begin(), relations.requiredBy.end(),
                        [&](const PredicateActionRelation& required) {
                          return required.actionIndex == entry.actionIndex;
                        })) {
          reason = "also appears above";
        }
        ImGui::TextDisabled("%s", reason.c_str());
        ImGui::PopID();
      }
    };
    render_action_list("Actions that need this to be true", relations.requiredBy,
                       amber, "R");
    render_action_list("Actions that make it true", relations.madeTrueBy,
                       green, "+");
    render_action_list("Actions that make it false", relations.madeFalseBy,
                       red, "-");

    ImGui::Separator();
    ImGui::TextDisabled("Where it starts out");
    for (const ScenarioDef& scenario : m_model.scenarios) {
      const size_t count = static_cast<size_t>(std::count_if(
          scenario.initialState.begin(), scenario.initialState.end(),
          [&](const FactRef& fact) { return fact.predicateName == predicate.name; }));
      ImGui::Text("%s", scenario.name.c_str());
      ImGui::SameLine();
      ImGui::TextDisabled("%zu facts", count);
    }
    ImGui::TextDisabled("Same shape");
    for (size_t i = 0; i < m_model.predicates.size(); ++i) {
      if (static_cast<int>(i) == selected_predicate) {
        continue;
      }
      const PredicateDef& other = m_model.predicates[i];
      const bool same_shape = other.params.size() == predicate.params.size() &&
          std::equal(other.params.begin(), other.params.end(), predicate.params.begin(),
                     [](const Parameter& first, const Parameter& second) {
                       return first.type == second.type;
                     });
      if (same_shape) {
        ImGui::PushID(static_cast<int>(30000 + i));
        if (ImGui::SmallButton(other.name.c_str())) {
          m_domainGraph.setSelectedPredicate(static_cast<int>(i));
        }
        ImGui::PopID();
        ImGui::SameLine();
      }
    }
    ImGui::NewLine();
    if (ImGui::Button("Neighbourhood view")) {
      m_domainViewMode = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Lifecycle of type")) {
      m_domainViewMode = 3;
    }
    ImGui::SameLine();
    if (ImGui::Button("Row in matrix")) {
      m_domainViewMode = 2;
    }
  } else {
    const int selected_action = m_domainGraph.selectedActionIndex();
    if (selected_action < 0 ||
        selected_action >= static_cast<int>(m_model.actions.size())) {
      ImGui::TextDisabled("Select a fact or action to inspect its relations.");
    } else {
      const ActionDef& action = m_model.actions[static_cast<size_t>(selected_action)];
      const ActionRelations& relations = index.action(static_cast<size_t>(selected_action));
      ImGui::Text("action  %s", formatActionSignature(action).c_str());
      const auto predicate_buttons = [&](const char* title,
                                         const std::vector<ActionPredicateRelation>& entries,
                                         ImVec4 colour) {
        ImGui::TextColored(colour, "%s  %zu", title, entries.size());
        for (const auto& entry : entries) {
          ImGui::PushID(static_cast<int>(40000 + entry.predicateIndex * 10U +
                                        entry.referenceIndex));
          if (ImGui::Selectable(m_model.predicates[entry.predicateIndex].name.c_str())) {
            m_domainGraph.setSelectedPredicate(static_cast<int>(entry.predicateIndex));
          }
          ImGui::PopID();
        }
      };
      predicate_buttons("It needs", relations.requires, amber);
      predicate_buttons("It makes true", relations.makesTrue, green);
      predicate_buttons("It makes false", relations.makesFalse, red);
      ImGui::TextColored(green, "Actions it may enable  %zu", relations.mayEnable.size());
      for (const size_t action_index : relations.mayEnable) {
        ImGui::PushID(static_cast<int>(50000 + action_index));
        if (ImGui::Selectable(m_model.actions[action_index].name.c_str())) {
          m_domainGraph.setSelectedAction(static_cast<int>(action_index));
        }
        ImGui::PopID();
      }
      ImGui::TextColored(amber, "Actions that may enable it  %zu",
                         relations.mayBeEnabledBy.size());
      for (const size_t action_index : relations.mayBeEnabledBy) {
        ImGui::PushID(static_cast<int>(60000 + action_index));
        if (ImGui::Selectable(m_model.actions[action_index].name.c_str())) {
          m_domainGraph.setSelectedAction(static_cast<int>(action_index));
        }
        ImGui::PopID();
      }
    }
  }
  ImGui::Separator();
  ImGui::TextDisabled("relation index: %zu predicates, %zu actions, %zu links",
                      index.predicateCount(), index.actionCount(), index.linkCount());
}

void AppShell::renderMatrixPanel() {
  const RelationIndex index(m_model);
  const FactActionMatrix matrix(m_model, index);
  ImGui::Text("%s - %zu facts x %zu actions", m_model.projectName.c_str(),
              m_model.predicates.size(), m_model.actions.size());
  ImGui::SameLine();
  if (ImGui::Button("Export CSV")) {
    const std::string path = pickSaveFile("Export fact-by-action matrix",
                                          m_model.projectName + "_matrix.csv",
                                          "*.csv", "CSV matrix (*.csv)");
    if (!path.empty()) {
      lastOperation = matrix.exportCsv(m_model, path)
          ? "Wrote " + path : "Failed to write " + path;
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Export Markdown")) {
    const std::string path = pickSaveFile("Export fact-by-action matrix",
                                          m_model.projectName + "_matrix.md",
                                          "*.md", "Markdown table (*.md)");
    if (!path.empty()) {
      lastOperation = matrix.exportMarkdown(m_model, path)
          ? "Wrote " + path : "Failed to write " + path;
    }
  }

  if (ImGui::BeginTable("##fact-action-matrix",
                        static_cast<int>(m_model.actions.size() + 1U),
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY)) {
    ImGui::TableSetupColumn("fact \\ action", ImGuiTableColumnFlags_WidthFixed, 150.0F);
    for (const ActionDef& action : m_model.actions) {
      ImGui::TableSetupColumn(action.name.c_str(), ImGuiTableColumnFlags_WidthFixed, 95.0F);
    }
    ImGui::TableHeadersRow();
    for (size_t predicate_index = 0;
         predicate_index < m_model.predicates.size(); ++predicate_index) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      if (ImGui::Selectable(m_model.predicates[predicate_index].name.c_str())) {
        m_domainGraph.setSelectedPredicate(static_cast<int>(predicate_index));
      }
      for (size_t action_index = 0; action_index < m_model.actions.size(); ++action_index) {
        ImGui::TableSetColumnIndex(static_cast<int>(action_index + 1U));
        const FactActionCell& cell = matrix.cell(predicate_index, action_index);
        if (cell.requires) {
          ImGui::TextColored(ImVec4(0.88F, 0.69F, 0.32F, 1.0F), "R");
          ImGui::SameLine();
        }
        if (cell.makesTrue) {
          ImGui::TextColored(ImVec4(0.32F, 0.84F, 0.60F, 1.0F), "+");
          ImGui::SameLine();
        }
        if (cell.makesFalse) {
          ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F), "-");
        }
      }
    }
    ImGui::EndTable();
  }
  ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F),
                     "%zu facts no action ever makes true",
                     index.factsNoActionMakesTrue().size());
}

void AppShell::renderLifecyclePanel() {
  ImGui::TextDisabled("Groupings are declared by hand; automatic suggestion is a later step.");
  ImGui::InputText("Grouping name", m_stateGroupNameInput,
                   sizeof(m_stateGroupNameInput));
  ImGui::InputText("Object type", m_stateGroupTypeInput,
                   sizeof(m_stateGroupTypeInput));
  ImGui::InputText("Alternative fact names", m_stateGroupPredicatesInput,
                   sizeof(m_stateGroupPredicatesInput));
  if (ImGui::Button("Add state grouping") &&
      m_stateGroupNameInput[0] != '\0' && m_stateGroupTypeInput[0] != '\0') {
    const StateGroupDef group{m_stateGroupNameInput, m_stateGroupTypeInput,
                              parseArgList(m_stateGroupPredicatesInput)};
    m_commandStack.execute(m_model, "Add lifecycle grouping",
                           [group](ProjectModel& model) {
      model.stateGroups.push_back(group);
    });
    m_stateGroupNameInput[0] = '\0';
    m_stateGroupTypeInput[0] = '\0';
    m_stateGroupPredicatesInput[0] = '\0';
  }
  ImGui::Separator();
  const RelationIndex index(m_model);
  const LifecycleModel lifecycles(m_model, index);
  if (lifecycles.diagrams().empty()) {
    ImGui::TextDisabled("No lifecycle groupings have been declared.");
  }
  for (const LifecycleDiagram& diagram : lifecycles.diagrams()) {
    const StateGroupDef& group = m_model.stateGroups[diagram.groupIndex];
    ImGui::PushID(static_cast<int>(diagram.groupIndex));
    ImGui::TextColored(ImVec4(0.0F, 0.9F, 1.0F, 1.0F), "%s", group.type.c_str());
    ImGui::SameLine();
    ImGui::Text("grouped by: %s", group.name.c_str());
    for (const size_t predicate_index : diagram.predicateIndices) {
      ImGui::Button(m_model.predicates[predicate_index].name.c_str(), ImVec2(150.0F, 0.0F));
      ImGui::SameLine();
    }
    ImGui::NewLine();
    if (diagram.transitions.empty()) {
      ImGui::TextDisabled("No action changes one declared state into another.");
    } else {
      for (const LifecycleTransition& transition : diagram.transitions) {
        ImGui::Text("%s  -- %s -->  %s",
                    m_model.predicates[transition.fromPredicateIndex].name.c_str(),
                    m_model.actions[transition.actionIndex].name.c_str(),
                    m_model.predicates[transition.toPredicateIndex].name.c_str());
      }
    }
    if (ImGui::SmallButton("Remove grouping")) {
      const size_t remove_index = diagram.groupIndex;
      m_commandStack.execute(m_model, "Remove lifecycle grouping",
                             [remove_index](ProjectModel& model) {
        model.stateGroups.erase(model.stateGroups.begin() +
                                static_cast<std::ptrdiff_t>(remove_index));
      });
      ImGui::PopID();
      break;
    }
    ImGui::Separator();
    ImGui::PopID();
  }
}

void AppShell::renderPddlTab() {
  const float halfH = ImGui::GetContentRegionAvail().y * 0.5F - 4.0F;
  ImGui::BeginChild("##PddlPreview", ImVec2(0.0F, halfH),
                    ImGuiChildFlags_Border);
  const std::string pddl = PddlGenerator::generateDomain(m_model);
  const auto load_generated = [&]() {
    const size_t length = std::min(pddl.size(), m_pddlEditorBuffer.size() - 1U);
    std::copy_n(pddl.data(), length, m_pddlEditorBuffer.data());
    m_pddlEditorBuffer[length] = '\0';
    m_pddlEditorInitialised = true;
    m_pddlEditorDirty = false;
  };
  if (!m_pddlEditorInitialised || !m_pddlEditorDirty) {
    load_generated();
  }
  if (ImGui::Button("Reload generated PDDL")) {
    load_generated();
  }
  ImGui::SameLine();
  if (ImGui::Button("Apply edited PDDL to project")) {
    const PddlImportResult imported =
        PddlImporter::importDomain(m_pddlEditorBuffer.data());
    if (imported.ok) {
      ProjectModel replacement = imported.model;
      replacement.objects = m_model.objects;
      replacement.scenarios = m_model.scenarios;
      replacement.stateGroups = m_model.stateGroups;
      m_commandStack.execute(m_model, "Apply edited PDDL",
                             [replacement](ProjectModel& model) {
        model = replacement;
      });
      projectName = m_model.projectName;
      clearDerivedResults();
      m_requestedTab = "PDDL";
      lastOperation = "Applied edited PDDL";
    } else {
      lastOperation = "PDDL edit failed: " + imported.error;
    }
  }
  if (ImGui::InputTextMultiline("##editable-pddl", m_pddlEditorBuffer.data(),
                                m_pddlEditorBuffer.size(), ImVec2(-FLT_MIN, -FLT_MIN),
                                ImGuiInputTextFlags_AllowTabInput)) {
    m_pddlEditorDirty = true;
  }
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

  ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F),
                     "Scenario: %s - no plan", m_lastPlanScenarioName.c_str());
  if (!m_lastFailureExplanation.available) {
    ImGui::TextWrapped("The tool could not identify one unreachable fact. %s",
                       m_lastPlan.error_msg.c_str());
    return;
  }

  ImGui::BeginChild("##failure-explanation", ImVec2(-270.0F, 0.0F),
                    ImGuiChildFlags_Border);
  ImGui::TextDisabled("the goal it could not reach");
  const std::string failed_goal =
      FailureExplainer::formatFact(m_lastFailureExplanation.failedGoal);
  ImGui::TextColored(ImVec4(0.0F, 0.9F, 1.0F, 1.0F), "%s", failed_goal.c_str());
  ImGui::Separator();
  ImGui::TextDisabled("Why");
  for (const FailureExplanationRow& row : m_lastFailureExplanation.rows) {
    ImVec4 colour(0.88F, 0.69F, 0.32F, 1.0F);
    const char* badge = "R";
    if (row.kind == FailureExplanationKind::Producer) {
      colour = ImVec4(0.32F, 0.84F, 0.60F, 1.0F);
      badge = "+";
    } else if (row.kind == FailureExplanationKind::RemovedAndRestored) {
      colour = ImVec4(0.95F, 0.51F, 0.42F, 1.0F);
      badge = "-";
    } else if (row.kind == FailureExplanationKind::Conclusion) {
      colour = ImVec4(0.95F, 0.51F, 0.42F, 1.0F);
      badge = ">";
    }
    ImGui::TextColored(colour, "%s", badge);
    ImGui::SameLine();
    ImGui::TextWrapped("%s", row.text.c_str());
  }

  ImGui::Separator();
  ImGui::TextDisabled("What would fix it");
  const std::string blocking_fact =
      FailureExplainer::formatFact(m_lastFailureExplanation.blockingFact);
  if (ImGui::Button(("Add " + blocking_fact +
                     " to this scenario's starting facts").c_str())) {
    const auto found = std::find_if(
        m_model.scenarios.begin(), m_model.scenarios.end(),
        [&](const ScenarioDef& scenario) { return scenario.name == m_lastPlanScenarioName; });
    if (found != m_model.scenarios.end()) {
      const size_t scenario_index = static_cast<size_t>(
          std::distance(m_model.scenarios.begin(), found));
      const FactRef fact = m_lastFailureExplanation.blockingFact;
      m_commandStack.execute(m_model, "Add suggested starting fact",
                             [scenario_index, fact](ProjectModel& model) {
        model.scenarios[scenario_index].initialState.push_back(fact);
      });
    }
  }
  if (ImGui::Button(("Add an action that restores " +
                     m_lastFailureExplanation.blockingFact.predicateName).c_str())) {
    const std::string predicate_name =
        m_lastFailureExplanation.blockingFact.predicateName;
    ActionDef action;
    action.name = "restore-" + predicate_name;
    const PredicateDef* predicate = predicateByName(m_model, predicate_name);
    if (predicate != nullptr) {
      action.params = predicate->params;
      EffectRef effect;
      effect.predicateName = predicate_name;
      for (const Parameter& parameter : predicate->params) {
        effect.argNames.push_back(parameter.name);
      }
      action.addEffects.push_back(std::move(effect));
    }
    m_commandStack.execute(m_model, "Add suggested restoring action",
                           [action](ProjectModel& model) {
      model.actions.push_back(action);
    });
    m_domainGraph.setSelectedAction(static_cast<int>(m_model.actions.size()) - 1);
    m_requestedTab = "Domain";
  }
  const auto jump_row = std::find_if(
      m_lastFailureExplanation.rows.begin(), m_lastFailureExplanation.rows.end(),
      [](const FailureExplanationRow& row) {
        return row.kind == FailureExplanationKind::RemovedAndRestored;
      });
  if (jump_row != m_lastFailureExplanation.rows.end() &&
      ImGui::Button(("Show me " + jump_row->actionName).c_str())) {
    const int action_index = RelationIndex(m_model).actionIndex(jump_row->actionName);
    if (action_index >= 0) {
      m_domainGraph.setSelectedAction(action_index);
      m_requestedTab = "Domain";
    }
  }

  ImGui::Separator();
  ImGui::TextDisabled("If this is intended");
  if (ImGui::Button("Mark this scenario expected-to-fail")) {
    const auto found = std::find_if(
        m_model.scenarios.begin(), m_model.scenarios.end(),
        [&](const ScenarioDef& scenario) { return scenario.name == m_lastPlanScenarioName; });
    if (found != m_model.scenarios.end()) {
      const size_t scenario_index = static_cast<size_t>(
          std::distance(m_model.scenarios.begin(), found));
      m_commandStack.execute(m_model, "Mark scenario expected to fail",
                             [scenario_index](ProjectModel& model) {
        model.scenarios[scenario_index].expectation.shouldSucceed = false;
      });
    }
  }
  ImGui::EndChild();
  ImGui::SameLine();
  ImGui::BeginChild("##unproduced-facts", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border);
  const RelationIndex index(m_model);
  ImGui::Text("Facts no action produces  %zu",
              index.factsNoActionMakesTrue().size());
  for (const size_t predicate_index : index.factsNoActionMakesTrue()) {
    ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F), "-");
    ImGui::SameLine();
    if (ImGui::Selectable(m_model.predicates[predicate_index].name.c_str())) {
      m_domainGraph.setSelectedPredicate(static_cast<int>(predicate_index));
      m_requestedTab = "Domain";
    }
  }
  ImGui::EndChild();
}

void AppShell::renderBtTab() {
  if (!m_btGraph.lastError().empty()) {
    ImGui::TextColored(ImVec4(1.0F, 0.35F, 0.35F, 1.0F),
                       "Parse error: %s",
                       m_btGraph.lastError().c_str());
  }
  if (m_btGraph.nodeCount() > 0U) {
    if (ImGui::Button("Collapse all")) {
      m_btGraph.collapseAll();
    }
    ImGui::SameLine();
    if (ImGui::Button("Expand all")) {
      m_btGraph.expandAll();
    }
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

  // Selected action editor uses the reviewed guided sentence wording.
  const int selAction = m_domainGraph.selectedActionIndex();
  if (selAction >= 0 && selAction < static_cast<int>(m_model.actions.size())) {
    m_guidedEditorRendered = true;
    ActionDef& action = m_model.actions[selAction];
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.30F, 0.85F, 1.0F, 1.0F),
                       "Editing an action");
    ImGui::Text("Action");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.0F, 0.9F, 1.0F, 1.0F), "%s", action.name.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("- described as an action in %s", m_model.projectName.c_str());

    ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F), "It involves");
    for (int parameter_index = 0;
         parameter_index < static_cast<int>(action.params.size());
         ++parameter_index) {
      Parameter& parameter = action.params[static_cast<size_t>(parameter_index)];
      ImGui::PushID(parameter_index);
      ImGui::TextUnformatted("a");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(120.0F);
      if (ImGui::BeginCombo("##involves-type", parameter.type.c_str())) {
        for (const TypeDef& type : m_model.types) {
          if (ImGui::Selectable(type.name.c_str(), type.name == parameter.type)) {
            const std::string selected_type = type.name;
            m_commandStack.execute(m_model, "Choose involved type",
                                   [selAction, parameter_index,
                                    selected_type](ProjectModel& model) {
              model.actions[static_cast<size_t>(selAction)]
                  .params[static_cast<size_t>(parameter_index)].type = selected_type;
            });
          }
        }
        ImGui::EndCombo();
      }
      ImGui::SameLine();
      ImGui::TextUnformatted("called");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(90.0F);
      if (ImGui::BeginCombo("##involves-name", parameter.name.c_str())) {
        for (const Parameter& candidate : action.params) {
          ImGui::Selectable(candidate.name.c_str(), candidate.name == parameter.name,
                            ImGuiSelectableFlags_Disabled);
        }
        ImGui::EndCombo();
      }
      ImGui::PopID();
    }
    static char s_aname[32] = {};
    static int s_new_type = 0;
    if (!m_model.types.empty()) {
      s_new_type = std::max(0, std::min(
          s_new_type, static_cast<int>(m_model.types.size()) - 1));
      ImGui::InputText("called##new-involves", s_aname, sizeof(s_aname));
      ImGui::SameLine();
      ImGui::SetNextItemWidth(120.0F);
      if (ImGui::BeginCombo("##new-involves-type",
                            m_model.types[static_cast<size_t>(s_new_type)].name.c_str())) {
        for (int i = 0; i < static_cast<int>(m_model.types.size()); ++i) {
          if (ImGui::Selectable(m_model.types[static_cast<size_t>(i)].name.c_str(),
                                i == s_new_type)) {
            s_new_type = i;
          }
        }
        ImGui::EndCombo();
      }
      ImGui::SameLine();
      if (ImGui::Button("+ add something else it involves") && s_aname[0] != '\0') {
        const std::string name = s_aname;
        const std::string type = m_model.types[static_cast<size_t>(s_new_type)].name;
        m_commandStack.execute(m_model, "Add involved name",
                               [selAction, name, type](ProjectModel& model) {
          model.actions[static_cast<size_t>(selAction)].params.push_back({name, type});
        });
        s_aname[0] = '\0';
      }
    }

    ImGui::Separator();
    renderGuidedReferenceGroup("Before it can happen", "##before-guided",
                               "+ add a condition", "must be", "",
                               selAction, PredicateRelationKind::Requires,
                               m_model, m_commandStack);
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F), "Afterwards");
    renderGuidedReferenceGroup("", "##after-true-guided",
                               "+ add an outcome", "is", "becomes true",
                               selAction, PredicateRelationKind::MakesTrue,
                               m_model, m_commandStack);
    renderGuidedReferenceGroup("", "##after-false-guided",
                               "+ add an outcome", "is", "becomes false",
                               selAction, PredicateRelationKind::MakesFalse,
                               m_model, m_commandStack);

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F), "When it runs");
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

    if (ImGui::InputText("behaviour tree node##bt", s_nodeType, sizeof(s_nodeType))) {
      const std::string nodeType = s_nodeType;
      m_commandStack.execute(m_model, "Set BT node type",
                             [selAction, nodeType](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].btBinding.nodeType =
            nodeType;
      });
      s_lastBtNodeType = nodeType;
    }
    bool reactive = action.btBinding.reactive;
    if (ImGui::Checkbox("reactive##bt", &reactive)) {
      m_commandStack.execute(m_model, "Set BT reactive binding",
                             [selAction, reactive](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].btBinding.reactive =
            reactive;
      });
    }
    if (ImGui::InputTextMultiline("subtree XML##bt",
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

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F), "Reads as");
    const ProjectModel selected_model = [&]() {
      ProjectModel copy = m_model;
      copy.actions = {action};
      return copy;
    }();
    renderPddlText(PddlGenerator::generateDomain(selected_model));
    ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F), "Checks");
    const StructuralReport checks = StructuralValidator::check(m_model);
    if (checks.hasErrors()) {
      ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F),
                         "! Fix %zu structural checks", checks.errorCount);
    } else {
      ImGui::TextColored(ImVec4(0.32F, 0.84F, 0.60F, 1.0F),
                         "Every name has a type");
      ImGui::TextColored(ImVec4(0.32F, 0.84F, 0.60F, 1.0F),
                         "Parses as STRIPS");
      ImGui::TextColored(ImVec4(0.32F, 0.84F, 0.60F, 1.0F),
                         "Grounds against %zu objects", m_model.objects.size());
    }
  }
}

void AppShell::clearDerivedResults() {
  m_lastValidation = ValidationReport{};
  m_structuralReport = StructuralReport{};
  m_lastBatchReport = ScenarioBatchReport{};
  m_lastContingencyReport = ContingencyReport{};
  m_lastPlan = ame::PlanResult{};
  m_lastFailureExplanation = FailureExplanation{};
  m_pddlEditorInitialised = false;
  m_pddlEditorDirty = false;
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

size_t AppShell::selfTestNeighbourhoodNodeCount(int depth) const {
  DomainElementRef focus;
  if (m_domainGraph.selectedPredicateIndex() >= 0) {
    focus = {DomainElementKind::Predicate,
             static_cast<size_t>(m_domainGraph.selectedPredicateIndex())};
  } else if (m_domainGraph.selectedActionIndex() >= 0) {
    focus = {DomainElementKind::Action,
             static_cast<size_t>(m_domainGraph.selectedActionIndex())};
  } else if (!m_model.predicates.empty()) {
    focus = {DomainElementKind::Predicate, 0};
  } else {
    return 0U;
  }
  return NeighbourhoodModel(m_model, RelationIndex(m_model), focus, depth).nodes().size();
}

std::string AppShell::selfTestMatrixCsv() const {
  const RelationIndex index(m_model);
  return FactActionMatrix(m_model, index).toCsv(m_model);
}

size_t AppShell::selfTestLifecycleTransitionCount() const {
  const RelationIndex index(m_model);
  const LifecycleModel lifecycles(m_model, index);
  size_t count = 0;
  for (const LifecycleDiagram& diagram : lifecycles.diagrams()) {
    count += diagram.transitions.size();
  }
  return count;
}

void AppShell::selfTestAddStateGroup(std::string name,
                                     std::string type,
                                     std::vector<std::string> predicates) {
  m_commandStack.execute(m_model, "Add lifecycle grouping",
                         [name = std::move(name), type = std::move(type),
                          predicates = std::move(predicates)](ProjectModel& model) {
    model.stateGroups.push_back({name, type, predicates});
  });
}

bool AppShell::selfTestSelectionBack() {
  if (!m_domainGraph.canGoBack()) {
    return false;
  }
  m_domainGraph.goBack();
  return true;
}

bool AppShell::selfTestSelectionForward() {
  if (!m_domainGraph.canGoForward()) {
    return false;
  }
  m_domainGraph.goForward();
  return true;
}

void AppShell::selfTestSetDomainView(int view) {
  m_domainViewMode = std::max(0, std::min(view, 4));
  m_requestedTab = "Domain";
}

bool AppShell::selfTestDomainViewRendered(int view) const {
  return view >= 0 && view < static_cast<int>(m_domainViewsRendered.size()) &&
         m_domainViewsRendered[static_cast<size_t>(view)];
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
    m_lastFailureExplanation = FailureExplanation{};
    m_planGraph.setPlan(m_lastPlan, wm, scenarioName);
    validationState =
        "Feasible (" + std::to_string(m_lastPlan.steps.size()) + " steps)";
  } else {
    m_planGraph.clear();
    const ScenarioDef& scenario =
        m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)];
    const RelationIndex relation_index(m_model);
    m_lastFailureExplanation = FailureExplanation{};
    for (size_t goal_index = 0; goal_index < scenario.goals.size(); ++goal_index) {
      FailureExplanation candidate = FailureExplainer::explain(
          m_model, relation_index, scenario, goal_index);
      if (!candidate.blockingFact.predicateName.empty() &&
          candidate.rows.size() > m_lastFailureExplanation.rows.size()) {
        m_lastFailureExplanation = std::move(candidate);
      }
    }
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
