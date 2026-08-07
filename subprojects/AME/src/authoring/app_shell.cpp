#include "app_shell.h"

#include "assurance_report.h"
#include "authoring_utils.h"
#include "fact_chooser.h"
#include "problem_list.h"
#include "guided_editor_model.h"
#include "imgui.h"
#include "pddl_generator.h"
#include "pddl_importer.h"
#include "review_pack.h"
#include "run_record.h"

#include <tinyfiledialogs.h>

#include <filesystem>

#include <ame/action_registry.h>
#include <ame/plan_compiler.h>
#include <ame/world_model.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cctype>
#include <cmath>
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

static std::string pickFolder(const char* title,
                              const std::string& defaultPath) {
  const char* path = tinyfd_selectFolderDialog(title, defaultPath.c_str());
  return path != nullptr ? std::string(path) : std::string();
}

// Forward declarations for helpers defined later in this file.
static void SectionAccent();
static void StatusPill(const char* text, ImVec4 borderColor, ImVec4 textColor);

// Vertical padding inside the status bar, above and below the row of pills.
static constexpr float kStatusBarPaddingY = 3.0F;
// Vertical padding inside each pill, above and below its text.
static constexpr float kStatusPillPaddingY = 2.0F;

/// Height of the status bar strip along the bottom of the window.
///
/// Worked out from the current text height rather than fixed, because a fixed
/// height has to be guessed and the guess was too small: the pills were taller
/// than the strip and their lower halves were cut off by the bottom of the
/// window. Both the status bar and the host window above it, which has to leave
/// room for it, take the height from here so the two cannot disagree.
static float statusBarHeight() {
  return ImGui::GetTextLineHeight() + (kStatusPillPaddingY * 2.0F) +
         (kStatusBarPaddingY * 2.0F);
}

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

static FactRef parseGroundedFact(const std::string& key) {
  FactRef fact;
  if (key.size() < 2U || key.front() != '(' || key.back() != ')') {
    return fact;
  }
  std::istringstream words(key.substr(1U, key.size() - 2U));
  words >> fact.predicateName;
  std::string object;
  while (words >> object) {
    fact.objectNames.push_back(object);
  }
  return fact;
}

static std::vector<std::string> groundedFactsForScenario(
    const ProjectModel& model,
    const std::string& scenarioName) {
  ame::WorldModel world_model;
  const ValidationReport validation =
      PddlValidator::validateAndBuildWorldModel(model, scenarioName,
                                                world_model);
  std::vector<std::string> facts;
  if (!validation.ok) {
    return facts;
  }
  facts.reserve(world_model.numFluents());
  for (unsigned id = 0; id < world_model.numFluents(); ++id) {
    facts.push_back(world_model.fluentName(id));
  }
  const RelationIndex index(model);
  std::vector<std::string> outside_predicates;
  for (const size_t predicate_index : index.factsNoActionMakesTrue()) {
    if (predicate_index < model.predicates.size()) {
      outside_predicates.push_back(model.predicates[predicate_index].name);
    }
  }
  // Facts no action produces represent things that happen to the mission.
  // Put them first because they are the usual controls for contingency paths.
  std::stable_sort(facts.begin(), facts.end(),
                   [&outside_predicates](const std::string& first,
                                         const std::string& second) {
    const FactRef first_fact = parseGroundedFact(first);
    const FactRef second_fact = parseGroundedFact(second);
    const bool first_outside =
        std::find(outside_predicates.begin(), outside_predicates.end(),
                  first_fact.predicateName) != outside_predicates.end();
    const bool second_outside =
        std::find(outside_predicates.begin(), outside_predicates.end(),
                  second_fact.predicateName) != outside_predicates.end();
    return first_outside && !second_outside;
  });
  return facts;
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

static void renderReadOnlyTextBox(const char* id,
                                  const std::string& text,
                                  const ImVec2& size) {
  std::vector<char> buffer(text.begin(), text.end());
  buffer.push_back('\0');
  ImGui::InputTextMultiline(id,
                            buffer.data(),
                            buffer.size(),
                            size,
                            ImGuiInputTextFlags_ReadOnly |
                                ImGuiInputTextFlags_AllowTabInput);
}

// True for the lines that open a PDDL section, which are drawn in the accent
// colour so a reader can find the shape of the generated text at a glance.
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
         text.rfind(":effect", 0) == 0;
}

// Draws generated PDDL as read-only text. Used by the guided editor's "Reads
// as" preview, where the user is shown the PDDL their sentence produced.
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

static ImVec2 remainingPanelSize() {
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 1.0F);
  size.y = std::max(size.y, 1.0F);
  return size;
}

static std::string buildValidationOutputText(
    const ValidationReport& validation,
    const StructuralReport& structural,
    const ScenarioBatchReport& batch,
    const ContingencyReport& contingency,
    const std::string& validationState,
    const std::string& lastOperation) {
  std::ostringstream output;
  output << "Validation state: " << validationState << '\n';
  output << "Last operation: " << lastOperation << "\n\n";

  if (validation.ok) {
    output << "Validation passed\n";
  } else if (validation.errors.empty()) {
    output << "Click Validate > Validate Now\n";
  } else {
    output << "Validation errors:\n";
    for (const auto& err : validation.errors) {
      output << "ERR: " << err.message << '\n';
      for (const auto& pn : err.predicateNames) {
        output << "  predicate: " << pn << '\n';
      }
      for (const auto& an : err.actionNames) {
        output << "  action: " << an << '\n';
      }
    }
  }

  if (validation.grounding.valid) {
    output << "\nGrounding report:\n";
    output << "Total fluents: " << validation.grounding.totalFluents << '\n';
    output << "Total ground actions: "
           << validation.grounding.totalGroundActions << '\n';
    for (const auto& warning : validation.grounding.warnings) {
      output << "WARNING: " << warning << '\n';
    }

    output << "\nPredicate ground instances:\n";
    for (const auto& stat : validation.grounding.predicateStats) {
      output << "  " << stat.elementName << ": " << stat.count << '\n';
    }

    output << "\nAction ground instances:\n";
    for (const auto& stat : validation.grounding.actionStats) {
      output << "  " << stat.elementName << ": " << stat.count << '\n';
    }
  }

  output << "\nStructural issues:\n";
  if (structural.issues.empty()) {
    output << "No structural issues\n";
  } else {
    for (const auto& issue : structural.issues) {
      output << (issue.severity == Severity::Error ? "ERR: " : "WARN: ")
             << issue.message << '\n';
    }
  }

  if (!batch.results.empty()) {
    output << "\nScenario simulation: " << batch.passCount << " as expected, "
           << batch.failCount << " fail, " << batch.errorCount << " error\n";
    for (const auto& result : batch.results) {
      output << result.scenarioName << ": ";
      switch (result.outcome) {
      case ScenarioOutcome::Pass:
        output << "AS EXPECTED";
        break;
      case ScenarioOutcome::Fail:
        output << "FAIL";
        break;
      case ScenarioOutcome::Error:
        output << "ERROR";
        break;
      }
      output << ", plan_steps=" << result.planStepCount
             << ", goal_reached=" << (result.goalReached ? "yes" : "no")
             << ", run_actions=" << result.runActionCount
             << ", replans=" << result.replanCount
             << ", seed=" << result.simulationSeed
             << ", time_ms=" << result.solveTimeMs;
      if (!result.reason.empty()) {
        output << ", notes=" << result.reason;
      }
      output << '\n';
    }
  }

  if (!contingency.contextFluents.empty()) {
    output << "\nContingency analysis: " << contingency.feasibleCount
           << " feasible / " << contingency.infeasibleCount
           << " infeasible / " << contingency.errorCount
           << " error (" << contingency.contextFluents.size()
           << " context fluents)\n";
    if (!contingency.error.empty()) {
      output << "ERROR: " << contingency.error << '\n';
    }
    for (const auto& ctx : contingency.results) {
      output << "Context:";
      if (ctx.trueFluents.empty()) {
        output << " (none)";
      } else {
        for (const auto& fluent : ctx.trueFluents) {
          output << ' ' << fluent;
        }
      }
      output << " -> ";
      if (!ctx.errorMessage.empty()) {
        output << "ERROR: " << ctx.errorMessage;
      } else if (ctx.planFound) {
        output << "OK";
      } else {
        output << "NO PLAN";
      }
      output << ", steps=" << ctx.planSteps << '\n';
    }
  }

  return output.str();
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
    ImGui::TextDisabled("No facts to choose from yet");
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
  // Everything this function draws is scoped to the group it is drawing.
  //
  // The three groups an action shows -- what must be true beforehand, what
  // becomes true afterwards, and what becomes false afterwards -- are all drawn
  // by this same function, and each numbers its rows from zero. The controls in
  // a row are named for their position in the sentence rather than for the fact
  // they carry, so without this the first row of all three groups asked Dear
  // ImGui for the same identities. An action that requires a fact, makes it
  // true and makes it false, which is common, therefore produced three
  // conflicting items, and typing in one row could change another.
  ImGui::PushID(tableId);
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
                              std::vector<std::string>& chosenObjects,
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
    ImGui::TextDisabled("No facts to choose from yet");
    ImGui::PopID();
    return;
  }

  if (selectedPredicate < 0 ||
      selectedPredicate >= static_cast<int>(model.predicates.size())) {
    selectedPredicate = 0;
  }

  const char* preview =
      model.predicates[static_cast<size_t>(selectedPredicate)].name.c_str();
  if (ImGui::BeginCombo("Fact", preview)) {
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
  // One chooser per thing the fact involves, each holding only objects of the
  // right type. Objects of the wrong type stay in the list, greyed, with the
  // reason beside them, so the rule is learned rather than hidden.
  const std::string factName =
      model.predicates[static_cast<size_t>(selectedPredicate)].name;
  const size_t arity = FactChooser::arity(model, factName);
  chosenObjects.resize(arity);

  for (size_t position = 0; position < arity; ++position) {
    ImGui::PushID(static_cast<int>(position));
    const std::vector<FactChoice> objects =
        FactChooser::objectsFor(model, factName, position);
    const std::string label =
        model.predicates[static_cast<size_t>(selectedPredicate)]
            .params[position]
            .name;
    const char* preview = chosenObjects[position].empty()
                              ? "choose"
                              : chosenObjects[position].c_str();
    if (ImGui::BeginCombo(label.c_str(), preview)) {
      for (const FactChoice& choice : objects) {
        if (!choice.allowed) {
          ImGui::BeginDisabled();
          ImGui::Selectable(choice.name.c_str(), false);
          ImGui::EndDisabled();
          if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("%s", choice.reason.c_str());
          }
          continue;
        }
        if (ImGui::Selectable(choice.name.c_str(),
                              choice.name == chosenObjects[position])) {
          chosenObjects[position] = choice.name;
        }
      }
      ImGui::EndCombo();
    }
    ImGui::PopID();
  }

  FactRef chosen;
  chosen.predicateName = factName;
  chosen.objectNames = chosenObjects;
  const std::string chosenProblem = FactChooser::whyNotValid(model, chosen);
  const bool chosenOk = chosenProblem.empty();

  if (!chosenOk) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button(addButtonLabel) && chosenOk) {
    stack.execute(model, addCommandLabel, [chosen, &facts](ProjectModel&) {
      facts.push_back(chosen);
    });
    for (std::string& object : chosenObjects) {
      object.clear();
    }
  }
  if (!chosenOk) {
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("%s", chosenProblem.c_str());
  }

  // The typed path stays for users who prefer it, held to the same rules.
  if (ImGui::TreeNode("Type it instead")) {
    ImGui::InputText("Fact", argBuffer, argBufferSize);
    const FactRef typed = FactChooser::parse(argBuffer);
    const std::string typedProblem =
        typed.predicateName.empty() ? "nothing typed yet"
                                    : FactChooser::whyNotValid(model, typed);
    const bool typedOk = typedProblem.empty();
    if (!typedOk) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Add what I typed") && typedOk) {
      stack.execute(model, addCommandLabel, [typed, &facts](ProjectModel&) {
        facts.push_back(typed);
      });
      argBuffer[0] = '\0';
    }
    if (!typedOk) {
      ImGui::EndDisabled();
      ImGui::SameLine();
      ImGui::TextDisabled("%s", typedProblem.c_str());
    }
    ImGui::TreePop();
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
                                                       int listKind) {
  ScenarioExpectation& expectation =
      model.scenarios[static_cast<size_t>(scenarioIdx)].expectation;
  if (listKind == 1) {
    return expectation.forbiddenActions;
  }
  if (listKind == 2) {
    return expectation.requiredRunActions;
  }
  if (listKind == 3) {
    return expectation.forbiddenRunActions;
  }
  return expectation.expectedActions;
}

static void renderExpectationActionSection(const char* title,
                                           const char* tableId,
                                           const char* removeCommandLabel,
                                           const char* addCommandLabel,
                                           std::vector<std::string>& actionNames,
                                           ProjectModel& model,
                                           CommandStack& stack,
                                           int scenarioIdx,
                                           int listKind,
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
                      [scenarioIdx, listKind, removeIdx](ProjectModel& target) {
          std::vector<std::string>& names =
              expectationActionList(target, scenarioIdx, listKind);
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
                    [scenarioIdx, listKind, actionName](ProjectModel& target) {
        std::vector<std::string>& names =
            expectationActionList(target, scenarioIdx, listKind);
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
  m_settingsPath = RecentProjects::defaultSettingsPath();
  m_recentProjects = RecentProjects::load(m_settingsPath);
}

/// A label reads better after "Undo" when it does not start with a capital.
static std::string lowerFirst(std::string text) {
  if (!text.empty()) {
    text[0] = static_cast<char>(
        std::tolower(static_cast<unsigned char>(text[0])));
  }
  return text;
}

void AppShell::selfTestSaveCurrentView(const std::string& name) {
  SavedView view;
  view.name = name;
  const int selectedAction = m_domainGraph.selectedActionIndex();
  const int selectedFact = m_domainGraph.selectedPredicateIndex();
  if (selectedAction >= 0 &&
      selectedAction < static_cast<int>(m_model.actions.size())) {
    view.focusAction = m_model.actions[static_cast<size_t>(selectedAction)].name;
  } else if (selectedFact >= 0 &&
             selectedFact < static_cast<int>(m_model.predicates.size())) {
    view.focusPredicate =
        m_model.predicates[static_cast<size_t>(selectedFact)].name;
  }
  view.depth = m_neighbourDepth;
  view.relationshipFilter = m_neighbourFilter;
  view.viewMode = m_domainViewMode;
  m_commandStack.execute(m_model, "Save a view", [view](ProjectModel& model) {
    model.savedViews.push_back(view);
  });
}

bool AppShell::selfTestOpenSavedView(const std::string& name) {
  const auto it = std::find_if(m_model.savedViews.begin(),
                               m_model.savedViews.end(),
                               [&name](const SavedView& view) {
                                 return view.name == name;
                               });
  if (it == m_model.savedViews.end()) {
    return false;
  }
  applySavedView(*it);
  return true;
}

void AppShell::applySavedView(const SavedView& view) {
  m_domainViewMode = view.viewMode;
  m_neighbourDepth = view.depth < 1 ? 1 : view.depth;
  m_neighbourFilter = view.relationshipFilter;

  if (!view.focusAction.empty()) {
    const auto it = std::find_if(m_model.actions.begin(), m_model.actions.end(),
                                 [&view](const ActionDef& action) {
                                   return action.name == view.focusAction;
                                 });
    if (it != m_model.actions.end()) {
      m_domainGraph.setSelectedAction(
          static_cast<int>(std::distance(m_model.actions.begin(), it)));
      m_domainGraph.setSelectedPredicate(-1);
    }
  } else if (!view.focusPredicate.empty()) {
    const auto it =
        std::find_if(m_model.predicates.begin(), m_model.predicates.end(),
                     [&view](const PredicateDef& predicate) {
                       return predicate.name == view.focusPredicate;
                     });
    if (it != m_model.predicates.end()) {
      m_domainGraph.setSelectedPredicate(
          static_cast<int>(std::distance(m_model.predicates.begin(), it)));
      m_domainGraph.setSelectedAction(-1);
    }
  }
  m_requestedTab = "Domain";
  lastOperation = "Opened the view '" + view.name + "'";
}

void AppShell::copySelection() {
  const int selectedAction = m_domainGraph.selectedActionIndex();
  if (selectedAction >= 0 &&
      ModelEdits::copyAction(m_model, static_cast<size_t>(selectedAction),
                             m_clipboard)) {
    lastOperation = "Copied " + m_clipboard.action.name;
    return;
  }
  const int selectedFact = m_domainGraph.selectedPredicateIndex();
  if (selectedFact >= 0 &&
      ModelEdits::copyFact(m_model, static_cast<size_t>(selectedFact),
                           m_clipboard)) {
    lastOperation = "Copied " + m_clipboard.fact.name;
    return;
  }
  lastOperation = "Nothing selected to copy";
}

void AppShell::pasteClipboard() {
  if (!m_clipboard.holdsSomething()) {
    lastOperation = "Nothing to paste";
    return;
  }
  const ElementClipboard clipboard = m_clipboard;
  std::string pastedName;
  m_commandStack.execute(m_model, "Paste",
                         [clipboard, &pastedName](ProjectModel& model) {
    pastedName = ModelEdits::paste(model, clipboard);
  });
  m_unsavedChanges = true;
  lastOperation = pastedName.empty() ? "Nothing to paste"
                                     : "Pasted " + pastedName;
}

bool AppShell::openProjectFrom(const std::string& path) {
  ProjectModel loaded;
  if (!loaded.load(path)) {
    lastOperation = "Failed to load " + path;
    return false;
  }
  m_commandStack.clear();
  m_model = std::move(loaded);
  projectName = m_model.projectName;
  m_selectedScenarioIdx = -1;
  clearDerivedResults();
  resetScenarioEditorState();
  noteProjectOpened(path);
  lastOperation = "Loaded " + path;
  return true;
}

void AppShell::noteProjectOpened(const std::string& path) {
  m_projectPath = path;
  m_unsavedChanges = false;
  // Without this the next frame sees a count it has not seen before and marks
  // an untouched project as changed.
  m_lastSeenEditCount = m_commandStack.editCount();
  RecentProjects::remember(m_settingsPath, path);
  m_recentProjects = RecentProjects::load(m_settingsPath);
}

void AppShell::noteProjectSaved(const std::string& path) {
  noteProjectOpened(path);
  // The work is on disk under its own name, so the recovery copy beside it has
  // nothing left to recover.
  const std::string recovery = RecentProjects::recoveryPathFor(path);
  if (!recovery.empty()) {
    std::error_code ignored;
    std::filesystem::remove(recovery, ignored);
  }
}

void AppShell::writeRecoveryCopy() {
  // A copy beside the project, rewritten as the work changes, so that a tool
  // that stops unexpectedly has not taken the day's edits with it. It is not a
  // save: the project file itself is untouched until the user asks.
  const std::string recovery = RecentProjects::recoveryPathFor(m_projectPath);
  if (recovery.empty() || !m_unsavedChanges) {
    return;
  }
  m_model.save(recovery);
  m_recoveryWritten = true;
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
            noteProjectOpened(path);
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
          if (m_model.save(path)) {
            noteProjectSaved(path);
            lastOperation = "Saved " + path;
          } else {
            lastOperation = "Failed to save " + path;
          }
          if (m_autoValidateOnSave) {
            runValidation();
          }
        }
      }
      // The projects this user opened last, so a path never has to be typed.
      if (ImGui::BeginMenu("Recent projects", !m_recentProjects.empty())) {
        for (const std::string& recent : m_recentProjects) {
          if (ImGui::MenuItem(recent.c_str())) {
            openProjectFrom(recent);
          }
        }
        ImGui::EndMenu();
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
            } else if (m_model.predicates.empty() && m_model.actions.empty()) {
              // Nothing to merge with, so nothing to ask about.
              m_commandStack.clear();
              m_model = import.model;
              ImportMerge::layoutByRelationships(m_model);
              projectName = m_model.projectName;
              m_selectedScenarioIdx = -1;
              clearDerivedResults();
              resetScenarioEditorState();
              lastOperation = "Imported domain: " + projectName;
            } else {
              // Something is already here, so show what the import would do
              // before doing any of it.
              m_incomingModel = import.model;
              m_mergePlan = ImportMerge::plan(m_model, m_incomingModel);
              m_mergeChoices = MergeChoices{};
              m_showMergeDialog = true;
              lastOperation = "Reviewing what this import would change";
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
      if (ImGui::MenuItem("Export every scenario's problem file...")) {
        m_structuralReport = StructuralValidator::check(m_model);
        if (m_structuralReport.hasErrors()) {
          lastOperation =
              "Refusing: " + std::to_string(m_structuralReport.errorCount) +
              " structural error(s)";
        } else if (m_model.scenarios.empty()) {
          lastOperation = "No scenarios to export";
        } else {
          // One file per scenario, named after it, into the folder the user
          // picks. Exporting only the first was never what anybody wanted; it
          // was what the code did.
          const std::string anchor = pickSaveFile(
              "Choose a folder and name for the problem files",
              authoring::problemPddlPath(m_model.projectName,
                                         m_model.scenarios.front().name),
              "*.pddl", "PDDL problem (*.pddl)");
          if (anchor.empty()) {
            lastOperation = "Export cancelled";
          } else {
            const std::filesystem::path folder =
                std::filesystem::path(anchor).parent_path();
            size_t written = 0;
            for (const ScenarioDef& scenario : m_model.scenarios) {
              const std::filesystem::path path =
                  folder / (m_model.projectName + "-" + scenario.name + ".pddl");
              std::ofstream file(path);
              file << PddlGenerator::generateProblem(m_model, scenario.name);
              if (file.good()) {
                ++written;
              }
            }
            lastOperation = "Wrote " + std::to_string(written) + " of " +
                            std::to_string(m_model.scenarios.size()) +
                            " problem files into " + folder.string();
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
      if (ImGui::MenuItem("Save Current Run...", nullptr, false,
                          m_simulation.isLoaded())) {
        const std::string folder = pickFolder("Save the recorded run in this folder",
                                              ".");
        if (folder.empty()) {
          lastOperation = "Save run cancelled";
        } else {
          const RecordedRun run =
              RecordedRun::fromSimulation(m_model, m_simulation);
          lastOperation = run.save(folder) ? "Saved recorded run: " + folder
                                           : "Could not save the recorded run";
        }
      }
      if (ImGui::MenuItem("Open Recorded Run...")) {
        const std::string folder = pickFolder("Open a recorded run", ".");
        if (!folder.empty()) {
          RecordedRun replay;
          if (replay.load(folder)) {
            m_replay = std::move(replay);
            m_btGraph.setXml(m_replay.compiledXml());
            m_requestedTab = "Run";
            m_runViewingHistory = false;
            lastOperation = "Opened recorded run: " + folder;
          } else {
            lastOperation = "Could not open recorded run: " +
                            replay.errorMessage();
          }
        }
      }
      if (ImGui::MenuItem("Compare Current Run with Recorded Run...", nullptr,
                          false, m_simulation.isLoaded())) {
        const std::string folder = pickFolder("Choose the recorded run to compare",
                                              ".");
        if (!folder.empty()) {
          RecordedRun saved;
          if (saved.load(folder)) {
            m_comparisonFirst =
                RecordedRun::fromSimulation(m_model, m_simulation);
            m_comparisonSecond = std::move(saved);
            m_runComparison =
                compareRuns(m_comparisonFirst, m_comparisonSecond);
            m_requestedTab = "Run";
            lastOperation = m_runComparison.summary;
          } else {
            lastOperation = "Could not compare: " + saved.errorMessage();
          }
        }
      }
      if (ImGui::MenuItem("Compare Two Recorded Runs...")) {
        const std::string first_folder = pickFolder("Choose the first recorded run",
                                                    ".");
        const std::string second_folder = first_folder.empty()
                                              ? std::string()
                                              : pickFolder(
                                                    "Choose the second recorded run",
                                                    ".");
        if (!second_folder.empty()) {
          RecordedRun first;
          RecordedRun second;
          if (first.load(first_folder) && second.load(second_folder)) {
            m_comparisonFirst = std::move(first);
            m_comparisonSecond = std::move(second);
            m_runComparison =
                compareRuns(m_comparisonFirst, m_comparisonSecond);
            m_requestedTab = "Run";
            lastOperation = m_runComparison.summary;
          } else {
            lastOperation = "Could not load both recorded runs";
          }
        }
      }
      if (ImGui::MenuItem("Export Assurance Evidence...")) {
        const std::string path = pickSaveFile(
            "Export assurance evidence",
            authoring::projectFilePath(m_model.projectName) + "-assurance.md",
            "*.md", "Markdown (*.md)");
        if (path.empty()) {
          lastOperation = "Export cancelled";
        } else {
          std::ofstream file(path);
          file << AssuranceReport::generate(m_model);
          lastOperation = file.good() ? "Wrote " + path
                                      : "Failed to write " + path;
        }
      }
      if (ImGui::MenuItem("Export Review Pack...")) {
        const std::string folder = pickFolder("Export the review pack here", ".");
        if (!folder.empty()) {
          const ReviewPackResult review = ReviewPackExporter::write(
              m_model, folder, m_simulation.isLoaded() ? &m_simulation : nullptr);
          lastOperation = review.success ? "Wrote review pack: " + review.folder
                                         : "Could not write review pack: " +
                                               review.error;
        }
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Exit")) {
        if (m_unsavedChanges) {
          m_askBeforeQuitting = true;
        } else {
          wantsQuit = true;
        }
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit")) {
      // The menu says what would be undone, so the user does not have to
      // remember what they last did.
      const std::string undoLabel =
          m_commandStack.canUndo()
              ? "Undo " + lowerFirst(m_commandStack.topUndoLabel())
              : std::string("Undo");
      const std::string redoLabel =
          m_commandStack.canRedo()
              ? "Redo " + lowerFirst(m_commandStack.topRedoLabel())
              : std::string("Redo");
      if (ImGui::MenuItem(undoLabel.c_str(), "Ctrl+Z",
                          false, m_commandStack.canUndo())) {
        m_commandStack.undo(m_model);
        m_unsavedChanges = true;
      }
      if (ImGui::MenuItem(redoLabel.c_str(), "Ctrl+Y",
                          false, m_commandStack.canRedo())) {
        m_commandStack.redo(m_model);
        m_unsavedChanges = true;
      }
      ImGui::Separator();
      const int selectedFact = m_domainGraph.selectedPredicateIndex();
      const int selectedAction = m_domainGraph.selectedActionIndex();
      const bool canCopy = selectedFact >= 0 || selectedAction >= 0;
      if (ImGui::MenuItem("Copy", "Ctrl+C", false, canCopy)) {
        copySelection();
      }
      if (ImGui::MenuItem(m_clipboard.description().c_str(), "Ctrl+V", false,
                          m_clipboard.holdsSomething())) {
        pasteClipboard();
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
  // Workflow order: author the domain -> generate PDDL -> see the plan -> run
  // it. The compiled behaviour tree has no tab of its own: it is drawn on the
  // Run tab, which is where it either sits waiting or lights up as it runs.
  // Two tabs drawing the same tree taught users that the tree you compile and
  // the tree that runs are different things, and they are not.
  static const std::vector<std::string> labels = {"Domain", "PDDL", "Plan",
                                                  "Run"};
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
  // A run keeps going while the user is looking at another tab, so it is
  // advanced here rather than while the Run tab is being drawn.
  m_simulation.advance(static_cast<double>(io.DeltaTime));

  // A recovery copy every half minute of changed work. Often enough that
  // little is lost, rarely enough that it is never in the way.
  // Any edit means the project has moved on since it was last written, which
  // is what the quit prompt and the recovery copy both key off. The count is
  // used rather than the undo depth, which stops rising once the stack is full
  // and does not move at all when keystrokes fold into the step before them.
  if (m_commandStack.editCount() != m_lastSeenEditCount) {
    m_lastSeenEditCount = m_commandStack.editCount();
    m_unsavedChanges = true;
  }

  m_secondsSinceRecoveryCopy += static_cast<double>(io.DeltaTime);
  if (m_secondsSinceRecoveryCopy > 30.0) {
    m_secondsSinceRecoveryCopy = 0.0;
    writeRecoveryCopy();
  }

  if (m_batchRunner.isRunning()) {
    m_batchProgressRendered = true;
    // One scenario runs per frame. This keeps project and planner ownership on
    // the UI thread, while still returning control to Dear ImGui between the
    // longer simulations so progress can be drawn and Stop can be pressed.
    const std::string scenario_name = m_batchRunner.currentScenarioName();
    m_batchRunner.step();
    m_lastBatchReport = m_batchRunner.report();
    const size_t total = m_batchRunner.totalCount();
    validationState = "Scenarios: " +
                      std::to_string(m_batchRunner.completedCount()) + "/" +
                      std::to_string(total) + " simulated";
    lastOperation = "Simulated scenario: " + scenario_name;
    if (!m_batchRunner.isRunning()) {
      validationState = "Scenarios: " +
                        std::to_string(m_lastBatchReport.passCount) + "/" +
                        std::to_string(total) + " as expected";
      lastOperation = "Finished simulating " + std::to_string(total) +
                      " scenarios";
    }
  }

  if (io.KeyCtrl && !io.WantTextInput) {
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false) && m_commandStack.canUndo()) {
      m_commandStack.undo(m_model);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Y, false) && m_commandStack.canRedo()) {
      m_commandStack.redo(m_model);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_C, false)) {
      copySelection();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_V, false)) {
      pasteClipboard();
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
    // F and A are unmodified keys, so they must not fire while a modifier is
    // held: Ctrl+A is "select all" everywhere else, and would otherwise open
    // the quick-add box.
    // F: bring the whole picture back into view, for a canvas that has been
    // panned somewhere the user cannot find their way back from.
    if (!io.KeyCtrl && !io.KeyAlt && !io.KeySuper &&
        ImGui::IsKeyPressed(ImGuiKey_F, false)) {
      m_domainGraph.requestFitToContents();
      lastOperation = "Fitted the canvas to what is on it";
    }
    // A: add something without reaching for the palette.
    if (!io.KeyCtrl && !io.KeyAlt && !io.KeySuper &&
        !ImGui::IsAnyItemActive() &&
        ImGui::IsKeyPressed(ImGuiKey_A, false)) {
      m_paletteQuickAddOpen = true;
      m_paletteQuickAddName[0] = '\0';
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
  // and the status-bar overlay carve out the top and bottom margins.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float menuH = ImGui::GetFrameHeight();
  const float statusH = statusBarHeight();
  ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, vp->Pos.y + menuH));
  ImGui::SetNextWindowSize(ImVec2(vp->Size.x, vp->Size.y - menuH - statusH));
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
        {"Run", &AppShell::renderRunTab},
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

  if (m_showSaveViewDialog) {
    ImGui::OpenPopup("Save this view##modal");
    m_showSaveViewDialog = false;
  }
  if (ImGui::BeginPopupModal("Save this view##modal", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("Give this picture a name to come back to.");
    ImGui::InputText("Name##saveview", m_saveViewNameInput,
                     sizeof(m_saveViewNameInput));
    const std::string name = m_saveViewNameInput;
    if (name.empty()) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Save") && !name.empty()) {
      SavedView view;
      view.name = name;
      const int selectedFact = m_domainGraph.selectedPredicateIndex();
      const int selectedAction = m_domainGraph.selectedActionIndex();
      if (selectedAction >= 0 &&
          selectedAction < static_cast<int>(m_model.actions.size())) {
        view.focusAction =
            m_model.actions[static_cast<size_t>(selectedAction)].name;
      } else if (selectedFact >= 0 &&
                 selectedFact < static_cast<int>(m_model.predicates.size())) {
        view.focusPredicate =
            m_model.predicates[static_cast<size_t>(selectedFact)].name;
      }
      view.depth = m_neighbourDepth;
      view.relationshipFilter = m_neighbourFilter;
      view.viewMode = m_domainViewMode;
      m_commandStack.execute(m_model, "Save a view",
                             [view](ProjectModel& model) {
        const auto existing =
            std::find_if(model.savedViews.begin(), model.savedViews.end(),
                         [&view](const SavedView& saved) {
                           return saved.name == view.name;
                         });
        if (existing == model.savedViews.end()) {
          model.savedViews.push_back(view);
        } else {
          *existing = view;
        }
      });
      lastOperation = "Saved the view '" + name + "'";
      ImGui::CloseCurrentPopup();
    }
    if (name.empty()) {
      ImGui::EndDisabled();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (m_showMergeDialog) {
    ImGui::OpenPopup("What this import would do##modal");
    m_showMergeDialog = false;
  }
  if (ImGui::BeginPopupModal("What this import would do##modal", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("%zu would be added, %zu would be overwritten, %zu are already "
                "the same.",
                m_mergePlan.countAdded(), m_mergePlan.countReplaced(),
                m_mergePlan.countUnchanged());
    ImGui::Separator();

    if (ImGui::BeginChild("##MergeItems", ImVec2(560.0F, 240.0F))) {
      for (const MergeItem& item : m_mergePlan.items) {
        if (item.disposition == MergeDisposition::Unchanged) {
          continue;
        }
        const bool replacing = item.disposition == MergeDisposition::Replaced;
        ImGui::TextColored(replacing ? ImVec4(0.88F, 0.69F, 0.32F, 1.0F)
                                     : ImVec4(0.32F, 0.84F, 0.60F, 1.0F),
                           replacing ? "overwrites" : "adds");
        ImGui::SameLine(110.0F);
        ImGui::TextUnformatted(item.name.c_str());
        if (replacing && !item.whatWouldBeLost.empty()) {
          ImGui::SameLine();
          ImGui::TextDisabled("losing %s", item.whatWouldBeLost.c_str());
        }
      }
    }
    ImGui::EndChild();

    ImGui::Separator();
    ImGui::TextDisabled("Anything new is always added. Choose what may be "
                        "overwritten:");
    ImGui::Checkbox("types", &m_mergeChoices.replaceTypes);
    ImGui::SameLine();
    ImGui::Checkbox("facts", &m_mergeChoices.replaceFacts);
    ImGui::SameLine();
    ImGui::Checkbox("actions", &m_mergeChoices.replaceActions);
    ImGui::SameLine();
    ImGui::Checkbox("objects", &m_mergeChoices.replaceObjects);

    if (ImGui::Button("Import")) {
      const ProjectModel incoming = m_incomingModel;
      const MergeChoices choices = m_mergeChoices;
      m_commandStack.execute(m_model, "Import a domain",
                             [incoming, choices](ProjectModel& model) {
        model = ImportMerge::apply(model, incoming, choices);
      });
      clearDerivedResults();
      resetScenarioEditorState();
      lastOperation = "Imported, keeping what was not replaced";
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      lastOperation = "Import cancelled";
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (m_askBeforeQuitting) {
    ImGui::OpenPopup("Unsaved work##modal");
    m_askBeforeQuitting = false;
  }
  if (ImGui::BeginPopupModal("Unsaved work##modal", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("This project has changes that are not saved.");
    if (!m_projectPath.empty()) {
      ImGui::TextDisabled("%s", m_projectPath.c_str());
    }
    if (ImGui::Button("Save and close")) {
      if (!m_projectPath.empty() && m_model.save(m_projectPath)) {
        noteProjectSaved(m_projectPath);
        wantsQuit = true;
      } else {
        lastOperation = "Could not save; nothing was closed";
      }
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Close without saving")) {
      wantsQuit = true;
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Keep working")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

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
                        m_initPredIdx, m_initChosenObjects, m_initArgsInput,
                        sizeof(m_initArgsInput));
      ImGui::Separator();
      renderFactSection("Goals", "##goalfacts", "Add Fact##goal",
                        "Remove goal fact", "Add goal fact",
                        scenario.goals, m_model, m_commandStack,
                        m_goalPredIdx, m_goalChosenObjects, m_goalArgsInput,
                        sizeof(m_goalArgsInput));
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

        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F),
                           "Expected simulated execution");
        bool shouldReachGoal = expectation.shouldReachGoal;
        if (ImGui::Checkbox("Run should reach the goal", &shouldReachGoal)) {
          const int scenarioIdx = m_selectedScenarioIdx;
          m_commandStack.execute(m_model, "Set expected run outcome",
                                 [scenarioIdx, shouldReachGoal](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(scenarioIdx)]
                .expectation.shouldReachGoal = shouldReachGoal;
          });
        }
        int minRunActions = expectation.minRunActions;
        if (ImGui::InputInt("Min actions run", &minRunActions)) {
          const int scenarioIdx = m_selectedScenarioIdx;
          minRunActions = std::max(0, minRunActions);
          m_commandStack.execute(m_model, "Set minimum actions run",
                                 [scenarioIdx, minRunActions](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(scenarioIdx)]
                .expectation.minRunActions = minRunActions;
          });
        }
        int maxRunActions = expectation.maxRunActions;
        if (ImGui::InputInt("Max actions run", &maxRunActions)) {
          const int scenarioIdx = m_selectedScenarioIdx;
          maxRunActions = std::max(0, maxRunActions);
          m_commandStack.execute(m_model, "Set maximum actions run",
                                 [scenarioIdx, maxRunActions](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(scenarioIdx)]
                .expectation.maxRunActions = maxRunActions;
          });
        }
        int maxReplans = expectation.maxReplans;
        if (ImGui::InputInt("Max replans", &maxReplans)) {
          const int scenarioIdx = m_selectedScenarioIdx;
          maxReplans = std::max(-1, maxReplans);
          m_commandStack.execute(m_model, "Set maximum replans",
                                 [scenarioIdx, maxReplans](ProjectModel& model) {
            model.scenarios[static_cast<size_t>(scenarioIdx)]
                .expectation.maxReplans = maxReplans;
          });
        }
        ImGui::TextDisabled("Use 0 for no action bound and -1 for no replan bound.");
        static int s_requiredRunActionIdx = 0;
        static int s_forbiddenRunActionIdx = 0;
        renderExpectationActionSection("Actions that must run",
                                       "##requiredrunactions",
                                       "Remove required run action",
                                       "Add required run action",
                                       expectation.requiredRunActions,
                                       m_model, m_commandStack,
                                       m_selectedScenarioIdx, 2,
                                       s_requiredRunActionIdx);
        renderExpectationActionSection("Actions that must not run",
                                       "##forbiddenrunactions",
                                       "Remove forbidden run action",
                                       "Add forbidden run action",
                                       expectation.forbiddenRunActions,
                                       m_model, m_commandStack,
                                       m_selectedScenarioIdx, 3,
                                       s_forbiddenRunActionIdx);
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
  // Saved views: a picture somebody wants to come back to, or show to somebody
  // else. Stored in the project, so it survives reopening and travels with it.
  if (ImGui::Button("Save this view")) {
    m_saveViewNameInput[0] = '\0';
    m_showSaveViewDialog = true;
  }
  if (!m_model.savedViews.empty()) {
    ImGui::SameLine();
    ImGui::SetNextItemWidth(220.0F);
    if (ImGui::BeginCombo("Saved views", "open a saved view")) {
      for (size_t i = 0; i < m_model.savedViews.size(); ++i) {
        const SavedView& view = m_model.savedViews[i];
        if (ImGui::Selectable(view.name.c_str())) {
          applySavedView(view);
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("%s, %d step%s out",
                            view.focusAction.empty()
                                ? (view.focusPredicate.empty()
                                       ? "no focus"
                                       : view.focusPredicate.c_str())
                                : view.focusAction.c_str(),
                            view.depth, view.depth == 1 ? "" : "s");
        }
      }
      ImGui::EndCombo();
    }
  }

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
    ImGui::SameLine();
    bool straight_links = m_domainGraph.straightLinks();
    if (ImGui::Checkbox("straight lines", &straight_links)) {
      m_domainGraph.setStraightLinks(straight_links);
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Draw links as straight lines between nodes rather than as curves.");
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

    // The role is part of the identity deliberately. One action can require a
    // fact, make it true and make it false, so the same action appears in all
    // three lists below. Keying only on the action would then give several
    // items the same identity, which Dear ImGui reports as an error and which
    // makes clicking one of them select the wrong row.
    const auto render_action_list = [&](const char* title,
                                        const std::vector<PredicateActionRelation>& entries,
                                        ImVec4 colour,
                                        const char* badge,
                                        int role_salt) {
      ImGui::TextColored(colour, "%s  %zu", title, entries.size());
      for (const auto& entry : entries) {
        const ActionDef& action = m_model.actions[entry.actionIndex];
        ImGui::PushID(role_salt +
                      static_cast<int>(entry.actionIndex * 10U + entry.referenceIndex));
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
                       amber, "R", 100000);
    render_action_list("Actions that make it true", relations.madeTrueBy,
                       green, "+", 200000);
    render_action_list("Actions that make it false", relations.madeFalseBy,
                       red, "-", 300000);

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
      // Salted per role for the same reason as the action lists above: a
      // predicate can be needed, made true and made false by one action.
      const auto predicate_buttons = [&](const char* title,
                                         const std::vector<ActionPredicateRelation>& entries,
                                         ImVec4 colour,
                                         int role_salt) {
        ImGui::TextColored(colour, "%s  %zu", title, entries.size());
        for (const auto& entry : entries) {
          ImGui::PushID(role_salt + static_cast<int>(entry.predicateIndex * 10U +
                                        entry.referenceIndex));
          if (ImGui::Selectable(m_model.predicates[entry.predicateIndex].name.c_str())) {
            m_domainGraph.setSelectedPredicate(static_cast<int>(entry.predicateIndex));
          }
          ImGui::PopID();
        }
      };
      predicate_buttons("It needs", relations.requires, amber, 400000);
      predicate_buttons("It makes true", relations.makesTrue, green, 500000);
      predicate_buttons("It makes false", relations.makesFalse, red, 600000);
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
  if (m_batchRunner.isRunning()) {
    m_batchProgressRendered = true;
    ImGui::Text("Simulating scenario %zu of %zu: %s",
                m_batchRunner.completedCount() + 1U,
                m_batchRunner.totalCount(),
                m_batchRunner.currentScenarioName().c_str());
    ImGui::SameLine();
    if (ImGui::Button("Stop batch")) {
      m_batchRunner.stop();
      m_lastBatchReport = m_batchRunner.report();
      lastOperation = "Stopped scenario simulation after " +
                      std::to_string(m_batchRunner.completedCount()) +
                      " scenarios";
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.88F, 0.69F, 0.32F, 1.0F), "SIMULATED");
  } else if (!m_lastBatchReport.results.empty()) {
    ImGui::Text("%zu of %zu scenarios were as expected",
                m_lastBatchReport.passCount,
                m_lastBatchReport.results.size());
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.88F, 0.69F, 0.32F, 1.0F), "SIMULATED");
  }
  const float halfH = ImGui::GetContentRegionAvail().y * 0.5F - 4.0F;
  ImGui::BeginChild("##PddlPreview", ImVec2(0.0F, halfH),
                    ImGuiChildFlags_Border);
  const std::string pddl = PddlGenerator::generateDomain(m_model);
  if (ImGui::Button("Copy domain PDDL")) {
    ImGui::SetClipboardText(pddl.c_str());
    lastOperation = "Copied domain PDDL";
  }
  ImGui::SameLine();
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
  const std::string validationOutput =
      buildValidationOutputText(m_lastValidation,
                                m_structuralReport,
                                m_lastBatchReport,
                                m_lastContingencyReport,
                                validationState,
                                lastOperation);
  if (ImGui::Button("Copy diagnostics")) {
    ImGui::SetClipboardText(validationOutput.c_str());
    lastOperation = "Copied diagnostics";
  }
  ImGui::SameLine();
  ImGui::Checkbox("Show the raw text instead", &m_showRawDiagnostics);
  ImGui::Separator();

  if (m_showRawDiagnostics) {
    renderReadOnlyTextBox("##ValidationDiagnosticsOutput",
                          validationOutput,
                          remainingPanelSize());
    ImGui::EndChild();
    return;
  }

  // Every problem is a row that selects and reveals what it is about, so a
  // reader works down the list rather than hunting for the thing it names.
  const std::vector<ProblemEntry> problems =
      ProblemList::build(m_model, m_structuralReport, m_lastValidation);
  m_problemListRendered = true;
  if (problems.empty()) {
    ImGui::TextColored(ImVec4(0.32F, 0.84F, 0.60F, 1.0F),
                       "Nothing is wrong with this project.");
    ImGui::EndChild();
    return;
  }

  ImGui::BeginChild("##ProblemRows", remainingPanelSize());
  for (size_t i = 0; i < problems.size(); ++i) {
    const ProblemEntry& problem = problems[i];
    ImGui::PushID(static_cast<int>(i));
    ImGui::TextColored(problem.isError ? ImVec4(0.95F, 0.51F, 0.42F, 1.0F)
                                       : ImVec4(0.88F, 0.69F, 0.32F, 1.0F),
                       problem.isError ? "problem" : "worth a look");
    ImGui::SameLine(110.0F);
    if (ImGui::Selectable(problem.sentence.c_str())) {
      revealProblemTarget(problem);
    }
    if (problem.canReveal() && ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Click to open %s", problem.targetName.c_str());
    }
    if (!problem.detail.empty() && ImGui::TreeNode("What the reader said")) {
      ImGui::TextWrapped("%s", problem.detail.c_str());
      ImGui::TreePop();
    }
    ImGui::PopID();
  }
  ImGui::EndChild();
  ImGui::EndChild();
}

void AppShell::revealProblemTarget(const ProblemEntry& problem) {
  if (!problem.canReveal()) {
    lastOperation = "That problem is not about one element";
    return;
  }
  switch (problem.target) {
  case ProblemTarget::Action:
    m_domainGraph.setSelectedAction(problem.targetIndex);
    m_domainGraph.setSelectedPredicate(-1);
    break;
  case ProblemTarget::Fact:
    m_domainGraph.setSelectedPredicate(problem.targetIndex);
    m_domainGraph.setSelectedAction(-1);
    break;
  case ProblemTarget::Type:
  case ProblemTarget::Object:
  case ProblemTarget::None:
    break;
  }
  m_requestedTab = "Domain";
  lastOperation = "Opened " + problem.targetName;
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

// The four colours the rest of the tool already uses for made true, selected,
// needed, and made false, applied here to what each step of a run is doing.
static ImVec4 runStatusColour(RunNodeStatus status) {
  switch (status) {
  case RunNodeStatus::Finished:
    return ImVec4(0.32F, 0.84F, 0.60F, 1.0F);
  case RunNodeStatus::Happening:
    return ImVec4(0.0F, 0.85F, 1.0F, 1.0F);
  case RunNodeStatus::WentWrong:
    return ImVec4(0.95F, 0.51F, 0.42F, 1.0F);
  case RunNodeStatus::Waiting:
    break;
  }
  return ImVec4(0.55F, 0.70F, 0.80F, 1.0F);
}

static bool containsIgnoringCase(const std::string& text,
                                 const std::string& filter) {
  if (filter.empty()) {
    return true;
  }
  std::string lowerText = text;
  std::string lowerFilter = filter;
  std::transform(lowerText.begin(), lowerText.end(), lowerText.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::transform(lowerFilter.begin(), lowerFilter.end(), lowerFilter.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lowerText.find(lowerFilter) != std::string::npos;
}

static std::string happeningSentence(const RunState& state) {
  std::vector<std::string> happening;
  size_t finished = 0;
  for (const RunActionStep& step : state.actionSteps) {
    if (step.status == RunNodeStatus::Happening) {
      happening.push_back(step.signature);
    } else if (step.status == RunNodeStatus::Finished) {
      ++finished;
    }
  }
  if (!happening.empty()) {
    std::ostringstream sentence;
    sentence << "At tick " << state.tick << ", ";
    for (size_t i = 0; i < happening.size(); ++i) {
      if (i != 0) {
        sentence << (i + 1U == happening.size() ? " and " : ", ");
      }
      sentence << happening[i];
    }
    sentence << (happening.size() == 1U ? " is" : " are") << " happening.";
    return sentence.str();
  }
  if (state.tick == 0) {
    return "Nothing has started yet.";
  }
  return "At tick " + std::to_string(state.tick) + ", " +
         std::to_string(finished) +
         (finished == 1U ? " action has finished." : " actions have finished.");
}

static bool renderRunTimeline(unsigned current_tick,
                              const std::vector<RunFactChange>& fact_changes,
                              const RunState& state,
                              unsigned& selectedTick) {
  ImGui::TextDisabled("Timeline - click to inspect a tick");
  ImGui::SameLine();
  ImGui::Text("viewing tick %u", state.tick);

  const float rowHeight = 34.0F;
  const float labelWidth = 220.0F;
  const float canvasWidth =
      std::max(520.0F, ImGui::GetContentRegionAvail().x - 4.0F);
  const float canvasHeight =
      rowHeight * static_cast<float>(state.actionSteps.size() + 1U) + 8.0F;
  ImGui::BeginChild("##RunTimelineCanvas", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border,
                    ImGuiWindowFlags_HorizontalScrollbar);
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("##RunTimelineHitArea",
                         ImVec2(canvasWidth, canvasHeight));

  ImDrawList* draw = ImGui::GetWindowDrawList();
  const float axisStart = origin.x + labelWidth;
  const float axisEnd = origin.x + canvasWidth - 12.0F;
  const float axisWidth = std::max(1.0F, axisEnd - axisStart);
  const unsigned axisTicks = std::max(1U, current_tick);
  const auto tickX = [axisStart, axisWidth, axisTicks](unsigned tick) {
    return axisStart +
           axisWidth * static_cast<float>(std::min(tick, axisTicks)) /
               static_cast<float>(axisTicks);
  };

  const unsigned tickStride = std::max(1U, (axisTicks + 7U) / 8U);
  for (unsigned tick = 0; tick <= axisTicks; tick += tickStride) {
    const float x = tickX(tick);
    draw->AddLine(ImVec2(x, origin.y + rowHeight - 4.0F),
                  ImVec2(x, origin.y + canvasHeight),
                  IM_COL32(57, 79, 91, 180));
    draw->AddText(ImVec2(x + 3.0F, origin.y + 5.0F),
                  IM_COL32(147, 168, 179, 255),
                  std::to_string(tick).c_str());
  }
  if (axisTicks % tickStride != 0U) {
    draw->AddText(ImVec2(axisEnd - 18.0F, origin.y + 5.0F),
                  IM_COL32(147, 168, 179, 255),
                  std::to_string(axisTicks).c_str());
  }

  for (size_t i = 0; i < state.actionSteps.size(); ++i) {
    const RunActionStep& step = state.actionSteps[i];
    const float top = origin.y + rowHeight * static_cast<float>(i + 1U);
    const float middle = top + rowHeight * 0.5F;
    draw->AddText(ImVec2(origin.x + 8.0F, top + 8.0F),
                  IM_COL32(220, 230, 235, 255), step.signature.c_str());
    draw->AddLine(ImVec2(axisStart, middle), ImVec2(axisEnd, middle),
                  IM_COL32(67, 84, 94, 255), 2.0F);

    if (step.startTick != 0U) {
      const unsigned spanEnd =
          step.endTick == 0U ? current_tick : step.endTick;
      const float x1 = tickX(step.startTick);
      const float x2 = std::max(x1 + 4.0F, tickX(spanEnd));
      const ImVec4 colour = runStatusColour(step.status);
      draw->AddRectFilled(ImVec2(x1, top + 7.0F),
                          ImVec2(x2, top + rowHeight - 7.0F),
                          ImGui::ColorConvertFloat4ToU32(colour), 3.0F);
      const std::string span = step.endTick == 0U
                                   ? "starts " + std::to_string(step.startTick)
                                   : std::to_string(step.startTick) + "-" +
                                         std::to_string(step.endTick);
      draw->AddText(ImVec2(x1 + 4.0F, top + 9.0F),
                    IM_COL32(235, 245, 248, 255), span.c_str());
    }

    const std::string expectedSource = "PlannedAction:" + step.signature;
    for (const RunFactChange& change : fact_changes) {
      if (change.source != expectedSource) {
        continue;
      }
      const float x = tickX(change.tick);
      const ImU32 markerColour = change.value
                                     ? IM_COL32(255, 201, 77, 255)
                                     : IM_COL32(242, 130, 107, 255);
      draw->AddCircleFilled(ImVec2(x, middle), 4.5F, markerColour);
    }
  }

  const float playheadX = tickX(state.tick);
  draw->AddLine(ImVec2(playheadX, origin.y + rowHeight - 4.0F),
                ImVec2(playheadX, origin.y + canvasHeight),
                IM_COL32(0, 217, 255, 255), 2.0F);

  bool clicked = false;
  if (ImGui::IsItemHovered()) {
    const float mouseX = ImGui::GetIO().MousePos.x;
    if (mouseX >= axisStart && mouseX <= axisEnd) {
      const float fraction = (mouseX - axisStart) / axisWidth;
      const unsigned hoverTick = static_cast<unsigned>(
          std::lround(fraction * static_cast<float>(axisTicks)));
      ImGui::SetTooltip("View tick %u", hoverTick);
      if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        selectedTick = hoverTick;
        clicked = true;
      }
    }
  }
  ImGui::EndChild();
  return clicked;
}

static void renderFactList(const char* title,
                           const std::vector<std::string>& facts,
                           const char* prefix) {
  if (title[0] != '\0') {
    ImGui::TextDisabled("%s", title);
  }
  if (facts.empty()) {
    ImGui::TextDisabled("  none");
    return;
  }
  for (const std::string& fact : facts) {
    ImGui::BulletText("%s%s", prefix, fact.c_str());
  }
}

void AppShell::startRun() {
  if (m_selectedScenarioIdx < 0 ||
      m_selectedScenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    lastOperation = "No scenario selected";
    return;
  }

  const std::string scenarioName =
      m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)].name;
  m_simulation.setFaults(m_runFaults);
  if (m_simulation.start(m_model, scenarioName)) {
    // The tree on screen becomes the tree that is running, stand-in action
    // nodes and all, rather than a separately compiled copy of it.
    m_btGraph.setXml(m_simulation.compiledXml());
    m_runDisplayedReplans = 0;
    m_runViewingHistory = false;
    lastOperation = "Running: " + scenarioName;
  } else {
    m_btGraph.clearRunProgress();
    lastOperation = "Could not run: " + m_simulation.errorMessage();
  }
}

void AppShell::previewRunFaults() {
  if (m_selectedScenarioIdx < 0 ||
      m_selectedScenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    return;
  }
  m_simulation.setFaults(m_runFaults);
  const std::string& scenario_name =
      m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)].name;
  if (m_simulation.start(m_model, scenario_name)) {
    m_simulation.hold();
    m_btGraph.setXml(m_simulation.compiledXml());
    m_runDisplayedReplans = 0;
    m_runViewingHistory = false;
    lastOperation = "Loaded fault preview: " + scenario_name;
  } else {
    lastOperation = "Could not preview faults: " +
                    m_simulation.errorMessage();
  }
}

void AppShell::renderRunTab() {
  if (m_replay.loaded()) {
    renderReplayRunTab();
    return;
  }
  if (m_model.scenarios.empty()) {
    ImGui::TextDisabled(
        "A run needs a scenario: a situation to start from and a goal to reach. "
        "Add one on the Domain tab.");
    return;
  }

  if (m_selectedScenarioIdx < 0 ||
      m_selectedScenarioIdx >= static_cast<int>(m_model.scenarios.size())) {
    m_selectedScenarioIdx = 0;
  }

  // ---- The controls, in one row that never moves -------------------------
  const bool loaded = m_simulation.isLoaded();
  const bool finished = m_simulation.isFinished();

  // The control names are the reviewed ones. The pack proposed mission words
  // ("Hold", "One step", "Start again"); the review settled on the ordinary
  // machine words, because the people who use this are technical, just not
  // software engineers, and they say "pause" and "reset" in their own speech.
  if (ImGui::Button("Run")) {
    if (!loaded || finished) {
      startRun();
    } else {
      m_simulation.resume();
    }
    m_runViewingHistory = false;
  }
  ImGui::SameLine();
  if (ImGui::Button("Pause")) {
    m_simulation.hold();
  }
  ImGui::SameLine();
  if (ImGui::Button("Step")) {
    if (!loaded || finished) {
      startRun();
    }
    m_simulation.hold();
    m_simulation.stepOnce();
    m_runViewingHistory = false;
  }
  ImGui::SameLine();
  if (ImGui::Button("Stop")) {
    m_simulation.stop();
  }
  ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    if (loaded) {
      m_simulation.startAgain();
    } else {
      startRun();
    }
    m_runViewingHistory = false;
  }

  ImGui::SameLine();
  ImGui::SetNextItemWidth(120.0F);
  float ticksPerSecond = static_cast<float>(m_simulation.ticksPerSecond());
  if (ImGui::SliderFloat("speed", &ticksPerSecond, 0.5F, 20.0F,
                         "%.1f ticks / sec")) {
    m_simulation.setTicksPerSecond(static_cast<double>(ticksPerSecond));
  }

  ImGui::SameLine();
  ImGui::SetNextItemWidth(200.0F);
  const std::string& selectedScenario =
      m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)].name;
  if (ImGui::BeginCombo("scenario", selectedScenario.c_str())) {
    for (int i = 0; i < static_cast<int>(m_model.scenarios.size()); ++i) {
      const bool isSelected = i == m_selectedScenarioIdx;
      if (ImGui::Selectable(m_model.scenarios[static_cast<size_t>(i)].name.c_str(),
                            isSelected)) {
        m_selectedScenarioIdx = i;
        m_runFaultChoiceScenario.clear();
      }
    }
    ImGui::EndCombo();
  }

  if (m_runFaultChoiceScenario != selectedScenario) {
    m_runFaultChoiceScenario = selectedScenario;
    m_runFactChoices = groundedFactsForScenario(m_model, selectedScenario);
    m_runFaultFactIdx = 0;
    m_runFaultActionIdx = 0;
    m_runFaults = m_model.scenarios[static_cast<size_t>(m_selectedScenarioIdx)]
                      .expectation.runFault;
    std::snprintf(m_runFaultName, sizeof(m_runFaultName), "%s",
                  m_runFaults.name.c_str());
    m_simulation.setFaults(m_runFaults);
  }

  if (ImGui::CollapsingHeader("Make this run go wrong",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    m_runFaultPanelRendered = true;
    ImGui::SetNextItemWidth(180.0F);
    ImGui::InputTextWithHint("fault name", "for example sensor-lost",
                             m_runFaultName, sizeof(m_runFaultName));

    ImGui::TextDisabled("An action that will not work");
    if (!m_model.actions.empty()) {
      m_runFaultActionIdx = std::max(
          0, std::min(m_runFaultActionIdx,
                      static_cast<int>(m_model.actions.size()) - 1));
      ImGui::SetNextItemWidth(180.0F);
      if (ImGui::BeginCombo(
              "action##run-fault",
              m_model.actions[static_cast<size_t>(m_runFaultActionIdx)]
                  .name.c_str())) {
        for (int i = 0; i < static_cast<int>(m_model.actions.size()); ++i) {
          if (ImGui::Selectable(m_model.actions[static_cast<size_t>(i)].name.c_str(),
                                i == m_runFaultActionIdx)) {
            m_runFaultActionIdx = i;
          }
        }
        ImGui::EndCombo();
      }
      ImGui::SameLine();
      ImGui::SetNextItemWidth(90.0F);
      ImGui::InputInt("attempt##run-fault", &m_runFaultAttempt);
      m_runFaultAttempt = std::max(1, m_runFaultAttempt);
      ImGui::SameLine();
      if (ImGui::Button("Set action failure")) {
        m_runFaults.name = m_runFaultName;
        m_runFaults.actionFailures = {{
            m_model.actions[static_cast<size_t>(m_runFaultActionIdx)].name,
            static_cast<unsigned>(m_runFaultAttempt)}};
        previewRunFaults();
      }
      if (!m_runFaults.actionFailures.empty()) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear action failure")) {
          m_runFaults.actionFailures.clear();
          previewRunFaults();
        }
      }
    }

    ImGui::TextDisabled("Something outside the mission");
    if (!m_runFactChoices.empty()) {
      m_runFaultFactIdx = std::max(
          0, std::min(m_runFaultFactIdx,
                      static_cast<int>(m_runFactChoices.size()) - 1));
      ImGui::SetNextItemWidth(260.0F);
      if (ImGui::BeginCombo(
              "fact##run-fault",
              m_runFactChoices[static_cast<size_t>(m_runFaultFactIdx)].c_str())) {
        for (int i = 0; i < static_cast<int>(m_runFactChoices.size()); ++i) {
          if (ImGui::Selectable(m_runFactChoices[static_cast<size_t>(i)].c_str(),
                                i == m_runFaultFactIdx)) {
            m_runFaultFactIdx = i;
          }
        }
        ImGui::EndCombo();
      }
      ImGui::SameLine();
      ImGui::Checkbox("make true##run-fault", &m_runFaultFactValue);
      ImGui::SameLine();
      ImGui::SetNextItemWidth(80.0F);
      ImGui::InputInt("tick##run-fault", &m_runFaultTick);
      m_runFaultTick = std::max(1, m_runFaultTick);
      ImGui::SameLine();
      if (ImGui::Button("Set fact change")) {
        m_runFaults.name = m_runFaultName;
        m_runFaults.factChanges = {{
            parseGroundedFact(
                m_runFactChoices[static_cast<size_t>(m_runFaultFactIdx)]),
            m_runFaultFactValue,
            static_cast<unsigned>(m_runFaultTick)}};
        previewRunFaults();
      }
      if (!m_runFaults.factChanges.empty()) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear fact change")) {
          m_runFaults.factChanges.clear();
          previewRunFaults();
        }
      }
    }

    if (ImGui::SmallButton("Save this named fault with the scenario")) {
      m_runFaults.name = m_runFaultName;
      const int scenario_index = m_selectedScenarioIdx;
      const RunFaultSet faults = m_runFaults;
      m_commandStack.execute(m_model, "Save scenario run fault",
                             [scenario_index, faults](ProjectModel& model) {
        model.scenarios[static_cast<size_t>(scenario_index)]
            .expectation.runFault = faults;
      });
    }

    ImGui::Text("Repeatable: seed %u", m_model.simulationSeed);
    for (const FaultEffectExplanation& explanation :
         m_simulation.faultExplanations()) {
      ImGui::BulletText("%s", explanation.summary.c_str());
      if (!explanation.lostPrecondition.empty()) {
        ImGui::TextWrapped("  It can remove the precondition %s.",
                           explanation.lostPrecondition.c_str());
      }
      if (!explanation.respondingActions.empty()) {
        std::ostringstream actions;
        for (size_t i = 0; i < explanation.respondingActions.size(); ++i) {
          if (i != 0U) {
            actions << ", ";
          }
          actions << explanation.respondingActions[i];
        }
        ImGui::TextWrapped("  Domain actions that can respond: %s.",
                           actions.str().c_str());
      }
    }
  }

  // Rule 3 of the concept pack: a simulation is never evidence of field
  // behaviour, and the screen says so where it cannot be missed.
  ImGui::TextColored(ImVec4(0.88F, 0.69F, 0.32F, 1.0F),
                     "SIMULATED - every action is a stand-in, not the real one");

  renderRunComparison();

  if (!m_simulation.errorMessage().empty()) {
    ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F), "%s",
                       m_simulation.errorMessage().c_str());
  }

  if (m_simulation.replanCount() != m_runDisplayedReplans) {
    m_btGraph.setXml(m_simulation.compiledXml());
    m_runDisplayedReplans = m_simulation.replanCount();
  }

  if (!m_simulation.replans().empty()) {
    m_replanComparisonRendered = true;
    const ReplanEvent& replan = m_simulation.replans().back();
    ImGui::BeginChild("##ReplanComparison", ImVec2(0.0F, 190.0F),
                      ImGuiChildFlags_Border);
    ImGui::TextColored(ImVec4(0.95F, 0.51F, 0.42F, 1.0F),
                       "Replan %zu at tick %u", m_simulation.replanCount(),
                       replan.tick);
    ImGui::TextWrapped("%s", replan.reason.c_str());
    const float column_width = ImGui::GetContentRegionAvail().x * 0.5F - 4.0F;
    ImGui::BeginChild("##AbandonedPlan", ImVec2(column_width, 115.0F),
                      ImGuiChildFlags_Border);
    ImGui::TextDisabled("Plan that was abandoned");
    for (const std::string& action : replan.abandonedPlan) {
      ImGui::BulletText("%s", action.c_str());
    }
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("##ReplacementPlan", ImVec2(0.0F, 115.0F),
                      ImGuiChildFlags_Border);
    if (replan.replacementFound) {
      ImGui::TextDisabled("Plan that replaced it");
      for (const std::string& action : replan.replacementPlan) {
        ImGui::BulletText("%s", action.c_str());
      }
    } else {
      ImGui::TextDisabled("No replacement plan exists");
      if (replan.failureExplanation.rows.empty()) {
        ImGui::TextWrapped("The tool could not identify one blocking fact.");
      } else {
        for (const FailureExplanationRow& row :
             replan.failureExplanation.rows) {
          ImGui::BulletText("%s", row.text.c_str());
        }
      }
    }
    ImGui::EndChild();
    ImGui::EndChild();
  }

  RunState viewState;
  if (m_simulation.isLoaded()) {
    const unsigned requestedTick =
        m_runViewingHistory ? m_runViewTick : m_simulation.tick();
    viewState = m_simulation.stateAtTick(requestedTick);
    ImGui::Text("%s", m_simulation.summaryLine().c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("(%s)", runPhaseName(m_simulation.phase()));
    if (m_runViewingHistory) {
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(0.0F, 0.85F, 1.0F, 1.0F),
                         "viewing tick %u; live tick %u", viewState.tick,
                         m_simulation.tick());
    }
    // The tree is drawn from the run's own progress, so every node shows where
    // the run has got to rather than what the compiler emitted.
    m_btGraph.setRunProgress(viewState.actionSteps);
  } else {
    m_btGraph.clearRunProgress();
    ImGui::TextDisabled(
        "This is the tree the plan compiled. Press Run to watch it happen.");
  }

  if (m_btGraph.nodeCount() > 0U) {
    ImGui::SameLine();
    if (ImGui::SmallButton("Collapse all")) {
      m_btGraph.collapseAll();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Expand all")) {
      m_btGraph.expandAll();
    }
  }
  if (!m_btGraph.lastError().empty()) {
    ImGui::TextColored(ImVec4(1.0F, 0.35F, 0.35F, 1.0F), "Parse error: %s",
                       m_btGraph.lastError().c_str());
  }

  if (!m_simulation.isLoaded()) {
    ImGui::BeginChild("##RunTree", ImVec2(0.0F, 0.0F), ImGuiChildFlags_Border);
    m_btGraph.render();
    ImGui::EndChild();
    return;
  }

  // ---- The primary run view: timing and world facts ----------------------
  const float primaryHeight =
      std::max(230.0F, ImGui::GetContentRegionAvail().y * 0.53F);
  ImGui::BeginChild("##RunPrimary", ImVec2(0.0F, primaryHeight));
  ImGui::BeginChild("##RunTimelinePanel", ImVec2(-350.0F, 0.0F),
                    ImGuiChildFlags_Border);
  m_runTimelineRendered = true;
  unsigned selectedTick = viewState.tick;
  if (renderRunTimeline(m_simulation.tick(), m_simulation.factChanges(),
                        viewState, selectedTick)) {
    m_runViewTick = selectedTick;
    m_runViewingHistory = selectedTick != m_simulation.tick();
  }
  ImGui::EndChild();

  ImGui::SameLine();
  ImGui::BeginChild("##RunFactsPanel", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border);
  m_runFactsRendered = true;
  ImGui::Text("Facts at tick %u", viewState.tick);
  ImGui::SetNextItemWidth(-1.0F);
  ImGui::InputTextWithHint("##RunFactFilter", "Filter by object or fact name",
                           m_runFactFilter, sizeof(m_runFactFilter));
  ImGui::Separator();
  const std::string filter = m_runFactFilter;
  for (const RunFactState& fact : viewState.facts) {
    if (!containsIgnoringCase(fact.fact, filter)) {
      continue;
    }
    const bool justChanged =
        fact.changedDuringRun && fact.lastChangedTick == viewState.tick;
    if (justChanged) {
      const ImVec2 rowStart = ImGui::GetCursorScreenPos();
      ImGui::GetWindowDrawList()->AddRectFilled(
          rowStart,
          ImVec2(rowStart.x + ImGui::GetContentRegionAvail().x,
                 rowStart.y + ImGui::GetTextLineHeightWithSpacing()),
          IM_COL32(67, 74, 26, 220), 2.0F);
    }
    ImGui::TextColored(fact.value ? ImVec4(0.32F, 0.84F, 0.60F, 1.0F)
                                  : ImVec4(0.55F, 0.70F, 0.80F, 1.0F),
                       "%s", fact.value ? "true " : "false");
    ImGui::SameLine(55.0F);
    ImGui::TextUnformatted(fact.fact.c_str());
    ImGui::SameLine();
    if (fact.changedDuringRun) {
      ImGui::TextDisabled("tick %u%s", fact.lastChangedTick,
                          justChanged ? " - changed now" : "");
    } else {
      ImGui::TextDisabled("%s", fact.value ? "from the start" : "not yet");
    }
  }
  ImGui::EndChild();
  ImGui::EndChild();

  // ---- The tree answers where the run is; selection explains one node ----
  ImGui::TextColored(ImVec4(0.0F, 0.85F, 1.0F, 1.0F), "%s",
                     happeningSentence(viewState).c_str());
  ImGui::BeginChild("##RunTree", ImVec2(-350.0F, 0.0F), ImGuiChildFlags_Border);
  m_btGraph.render();
  ImGui::EndChild();

  ImGui::SameLine();
  ImGui::BeginChild("##RunState", ImVec2(0.0F, 0.0F), ImGuiChildFlags_Border);
  BtNodeDetail detail;
  if (m_btGraph.nodeDetail(m_btGraph.selectedNodeIndex(), detail)) {
    ImGui::TextDisabled("Selected tree node");
    ImGui::TextWrapped("%s", detail.label.c_str());
    if (detail.isAction) {
      ImGui::Text("Action: %s", detail.action.c_str());
      renderFactList("Before it can run - must be true",
                     detail.preconditions, "");
      renderFactList("Before it can run - must be observed",
                     detail.confirmedPreconditions, "");
      renderFactList("Before it can run - must be false",
                     detail.negativePreconditions, "");
      renderFactList("What it changes", detail.addEffects,
                     "makes true: ");
      renderFactList("", detail.deleteEffects, "makes false: ");
    } else {
      ImGui::TextDisabled(
          "This node organises other parts of the mission. Select an action "
          "node to see the facts it needs and changes.");
    }
  } else {
    ImGui::TextDisabled("Select a tree node to see what it needs and changes.");
  }

  ImGui::Separator();
  ImGui::TextDisabled("Goals at the viewed tick");
  for (const RunGoal& goal : m_simulation.goals()) {
    const auto found = std::find_if(
        viewState.facts.begin(), viewState.facts.end(),
        [&goal](const RunFactState& fact) { return fact.fact == goal.fact; });
    const bool met = found != viewState.facts.end() && found->value;
    ImGui::TextColored(met ? ImVec4(0.32F, 0.84F, 0.60F, 1.0F)
                           : ImVec4(0.55F, 0.70F, 0.80F, 1.0F),
                       "%s %s", met ? "met" : "not yet", goal.fact.c_str());
  }
  ImGui::EndChild();
}

void AppShell::renderRunComparison() {
  if (!m_comparisonFirst.loaded() || !m_comparisonSecond.loaded()) {
    return;
  }
  m_runComparisonRendered = true;
  ImGui::BeginChild("##RecordedRunComparison", ImVec2(0.0F, 260.0F),
                    ImGuiChildFlags_Border);
  ImGui::TextColored(ImVec4(0.88F, 0.69F, 0.32F, 1.0F), "Comparing two runs");
  ImGui::TextWrapped("%s", m_runComparison.summary.c_str());

  const float width = ImGui::GetContentRegionAvail().x * 0.5F - 4.0F;
  ImGui::BeginChild("##ComparisonFirst", ImVec2(width, 70.0F),
                    ImGuiChildFlags_Border);
  ImGui::TextDisabled("First run");
  ImGui::TextWrapped("%s · %s", m_comparisonFirst.manifest().scenario.c_str(),
                     m_comparisonFirst.simulated() ? "SIMULATED" : "REAL SYSTEM");
  ImGui::Text("%u ticks", m_comparisonFirst.tick());
  ImGui::EndChild();
  ImGui::SameLine();
  ImGui::BeginChild("##ComparisonSecond", ImVec2(0.0F, 70.0F),
                    ImGuiChildFlags_Border);
  ImGui::TextDisabled("Second run");
  ImGui::TextWrapped("%s · %s", m_comparisonSecond.manifest().scenario.c_str(),
                     m_comparisonSecond.simulated() ? "SIMULATED" : "REAL SYSTEM");
  ImGui::Text("%u ticks", m_comparisonSecond.tick());
  ImGui::EndChild();

  if (m_runComparison.treesDiffer) {
    ImGui::Text("First tree difference: tick %u",
                m_runComparison.firstDifferentTick);
  } else {
    ImGui::TextDisabled("The tree states do not differ.");
  }
  for (const std::string& action : m_runComparison.actionsOnlyInFirst) {
    ImGui::BulletText("Only the first run: %s", action.c_str());
  }
  for (const std::string& action : m_runComparison.actionsOnlyInSecond) {
    ImGui::BulletText("Only the second run: %s", action.c_str());
  }
  for (const std::string& fact : m_runComparison.endFactDifferences) {
    ImGui::BulletText("Different final fact: %s", fact.c_str());
  }
  if (ImGui::SmallButton("Close comparison")) {
    m_comparisonFirst = RecordedRun{};
    m_comparisonSecond = RecordedRun{};
  }
  ImGui::EndChild();
}

void AppShell::renderReplayRunTab() {
  m_replayRendered = true;
  if (ImGui::Button("Close recorded run")) {
    m_replay = RecordedRun{};
    m_btGraph.clearRunProgress();
    return;
  }
  ImGui::SameLine();
  ImGui::Text("Replay: %s", m_replay.manifest().scenario.empty()
                                  ? "recorded mission"
                                  : m_replay.manifest().scenario.c_str());

  if (m_replay.simulated()) {
    ImGui::TextColored(ImVec4(0.88F, 0.69F, 0.32F, 1.0F),
                       "SIMULATED - this is a replay of stand-in actions");
  } else {
    ImGui::TextColored(ImVec4(0.32F, 0.84F, 0.60F, 1.0F),
                       "REAL SYSTEM - this replay came from field execution");
  }
  ImGui::Text("%s", m_replay.summaryLine().c_str());
  renderRunComparison();

  const unsigned requested_tick =
      m_runViewingHistory ? m_runViewTick : m_replay.tick();
  const RunState state = m_replay.stateAtTick(requested_tick);
  m_btGraph.setRunProgress(state.actionSteps);

  const float primary_height =
      std::max(230.0F, ImGui::GetContentRegionAvail().y * 0.53F);
  ImGui::BeginChild("##ReplayPrimary", ImVec2(0.0F, primary_height));
  ImGui::BeginChild("##ReplayTimelinePanel", ImVec2(-350.0F, 0.0F),
                    ImGuiChildFlags_Border);
  m_runTimelineRendered = true;
  unsigned selected_tick = state.tick;
  if (renderRunTimeline(m_replay.tick(), m_replay.factChanges(), state,
                        selected_tick)) {
    m_runViewTick = selected_tick;
    m_runViewingHistory = selected_tick != m_replay.tick();
  }
  ImGui::EndChild();
  ImGui::SameLine();
  ImGui::BeginChild("##ReplayFactsPanel", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border);
  m_runFactsRendered = true;
  ImGui::Text("Facts at tick %u", state.tick);
  ImGui::SetNextItemWidth(-1.0F);
  ImGui::InputTextWithHint("##ReplayFactFilter", "Filter by object or fact name",
                           m_runFactFilter, sizeof(m_runFactFilter));
  for (const RunFactState& fact : state.facts) {
    if (!containsIgnoringCase(fact.fact, m_runFactFilter)) {
      continue;
    }
    ImGui::TextColored(fact.value ? ImVec4(0.32F, 0.84F, 0.60F, 1.0F)
                                  : ImVec4(0.55F, 0.70F, 0.80F, 1.0F),
                       "%s  %s", fact.value ? "true" : "false",
                       fact.fact.c_str());
  }
  ImGui::EndChild();
  ImGui::EndChild();

  ImGui::BeginChild("##ReplayTree", ImVec2(-350.0F, 0.0F),
                    ImGuiChildFlags_Border);
  m_btGraph.render();
  ImGui::EndChild();
  ImGui::SameLine();
  ImGui::BeginChild("##ReplayGoals", ImVec2(0.0F, 0.0F),
                    ImGuiChildFlags_Border);
  ImGui::TextDisabled("Goals at the viewed tick");
  for (const RunGoal& goal : m_replay.goalsAtTick(state.tick)) {
    ImGui::TextColored(goal.met ? ImVec4(0.32F, 0.84F, 0.60F, 1.0F)
                                : ImVec4(0.55F, 0.70F, 0.80F, 1.0F),
                       "%s %s", goal.met ? "met" : "not yet",
                       goal.fact.c_str());
  }
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
    // Wording avoids "confirmed predicate": the user is being asked about
    // evidence, not about the domain section this becomes.
    bool mustBeObserved = pred.confirmed;
    if (ImGui::Checkbox("Only counts once it has been observed##predconfirmed",
                        &mustBeObserved)) {
      m_commandStack.execute(m_model,
                             mustBeObserved
                                 ? "Require this fact to be observed"
                                 : "Accept this fact as expected",
                             [selPred, mustBeObserved](ProjectModel& model) {
        model.predicates[static_cast<size_t>(selPred)].confirmed = mustBeObserved;
      });
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "An action waiting on this fact will not start until something has\n"
          "reported it. Leave this off to let an action rely on the fact an\n"
          "earlier step was expected to bring about.");
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
      // The order these are written in is the order their values are given in,
      // so it is worth being able to change. The conditions and outcomes name
      // their parameters, so they follow the move untouched.
      ImGui::SameLine();
      const bool isFirst = parameter_index == 0;
      const bool isLast =
          parameter_index + 1 == static_cast<int>(action.params.size());
      if (isFirst) {
        ImGui::BeginDisabled();
      }
      if (ImGui::SmallButton("up") && !isFirst) {
        const int moved = parameter_index;
        m_commandStack.execute(m_model, "Move it earlier",
                               [selAction, moved](ProjectModel& model) {
          ModelEdits::moveActionParameter(model,
                                          static_cast<size_t>(selAction),
                                          static_cast<size_t>(moved), false);
        });
      }
      if (isFirst) {
        ImGui::EndDisabled();
      }
      ImGui::SameLine();
      if (isLast) {
        ImGui::BeginDisabled();
      }
      if (ImGui::SmallButton("down") && !isLast) {
        const int moved = parameter_index;
        m_commandStack.execute(m_model, "Move it later",
                               [selAction, moved](ProjectModel& model) {
          ModelEdits::moveActionParameter(model,
                                          static_cast<size_t>(selAction),
                                          static_cast<size_t>(moved), true);
        });
      }
      if (isLast) {
        ImGui::EndDisabled();
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
      m_commandStack.executeCoalescing(
          m_model, "Set the behaviour tree node",
          "action:" + std::to_string(selAction) + ":btnode",
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
      m_commandStack.executeCoalescing(
          m_model, "Set the subtree",
          "action:" + std::to_string(selAction) + ":subtree",
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

    // Settings a run uses in place of the real action. Everything here has a
    // working default, so a user who never opens it can still press Run.
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.31F, 0.66F, 0.78F, 1.0F), "In a simulated run");
    int ticks = action.simulation.ticks;
    ImGui::SetNextItemWidth(140.0F);
    if (ImGui::InputInt("how long it takes, in ticks##sim", &ticks)) {
      const int chosen = ticks < 1 ? 1 : ticks;
      m_commandStack.execute(m_model, "Set how long the action takes",
                             [selAction, chosen](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].simulation.ticks = chosen;
      });
    }
    bool succeeds = action.simulation.succeeds;
    if (ImGui::Checkbox("it works##sim", &succeeds)) {
      m_commandStack.execute(m_model,
                             succeeds ? "Let the action work"
                                      : "Make the action fail",
                             [selAction, succeeds](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].simulation.succeeds =
            succeeds;
      });
    }
    float failureChance =
        static_cast<float>(action.simulation.failureChance) * 100.0F;
    ImGui::SetNextItemWidth(200.0F);
    if (ImGui::SliderFloat("chance it goes wrong##sim", &failureChance, 0.0F,
                           100.0F, "%.0f%%")) {
      const double chance = static_cast<double>(failureChance) / 100.0;
      m_commandStack.execute(m_model, "Set the chance the action goes wrong",
                             [selAction, chance](ProjectModel& model) {
        model.actions[static_cast<size_t>(selAction)].simulation.failureChance =
            chance;
      });
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Runs draw against the project's seed, so the same seed gives the\n"
          "same run every time.");
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
  m_batchRunner.stop();
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
  m_lastBatchReport = ScenarioRunner::runAll(m_model);
}

void AppShell::selfTestStartBatch() {
  runAllScenarios();
}

void AppShell::selfTestStopBatch() {
  m_batchRunner.stop();
  m_lastBatchReport = m_batchRunner.report();
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

bool AppShell::selfTestClickFirstProblem() {
  const std::vector<ProblemEntry> problems =
      ProblemList::build(m_model, m_structuralReport, m_lastValidation);
  for (const ProblemEntry& problem : problems) {
    if (problem.canReveal()) {
      revealProblemTarget(problem);
      return true;
    }
  }
  return false;
}

bool AppShell::selfTestStartRun(const std::string& scenarioName) {
  const auto found = std::find_if(
      m_model.scenarios.begin(), m_model.scenarios.end(),
      [&scenarioName](const ScenarioDef& scenario) {
        return scenario.name == scenarioName;
      });
  if (found != m_model.scenarios.end()) {
    m_selectedScenarioIdx = static_cast<int>(
        std::distance(m_model.scenarios.begin(), found));
  }
  startRun();
  m_requestedTab = "Run";
  return m_simulation.isLoaded();
}

bool AppShell::selfTestStepRun() {
  m_simulation.hold();
  return m_simulation.stepOnce();
}

bool AppShell::selfTestRunToCompletion() {
  return m_simulation.runToCompletion();
}

void AppShell::selfTestReplayCurrentRun() {
  if (!m_simulation.isLoaded()) {
    return;
  }
  m_replay = RecordedRun::fromSimulation(m_model, m_simulation);
  m_btGraph.setXml(m_replay.compiledXml());
  m_runViewingHistory = false;
  m_requestedTab = "Run";
}

void AppShell::selfTestCompareCurrentRunWithItself() {
  if (!m_simulation.isLoaded()) {
    return;
  }
  m_comparisonFirst = RecordedRun::fromSimulation(m_model, m_simulation);
  m_comparisonSecond = RecordedRun::fromSimulation(m_model, m_simulation);
  m_runComparison = compareRuns(m_comparisonFirst, m_comparisonSecond);
  m_requestedTab = "Run";
}

void AppShell::selfTestSetDomainView(int view) {
  m_domainViewMode = std::max(0, std::min(view, 4));
  m_requestedTab = "Domain";
}

bool AppShell::selfTestLoadProject(const std::string& path) {
  if (!m_model.load(path)) {
    return false;
  }
  projectName = m_model.projectName;
  m_commandStack.clear();
  clearDerivedResults();
  resetScenarioEditorState();
  return true;
}

bool AppShell::selfTestFocusBusiestPredicate() {
  if (m_model.predicates.empty()) {
    return false;
  }
  const RelationIndex index(m_model);
  size_t best = 0;
  size_t best_links = 0;
  for (size_t i = 0; i < m_model.predicates.size(); ++i) {
    const PredicateRelations& relations = index.predicate(i);
    const size_t links = relations.requiredBy.size() +
                         relations.madeTrueBy.size() +
                         relations.madeFalseBy.size();
    if (links > best_links) {
      best_links = links;
      best = i;
    }
  }
  m_domainGraph.setSelectedPredicate(static_cast<int>(best));
  return best_links > 0;
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
  m_batchRunner.start(m_model);
  m_lastBatchReport = m_batchRunner.report();
  validationState = "Scenarios: 0/" +
                    std::to_string(m_batchRunner.totalCount()) +
                    " simulated";
  lastOperation = m_batchRunner.isRunning()
                      ? "Started scenario simulation"
                      : "There are no scenarios to simulate";
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
    // Authoring previews plans while bindings are still being authored: actions
    // without a BT binding compile to stub units (effect predicates only)
    // instead of failing the whole compile. Production stays fail-closed.
    compiler.setStubUnregisteredActions(true);
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
  const ImVec2 padding(8.0F, kStatusPillPaddingY);
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
  const float kStatusBarHeight = statusBarHeight();
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

  // A window's default padding is meant for a panel of content and is far too
  // deep for a single row of pills: it pushed them past the bottom of the strip
  // and the window, so their lower edges were cut off. The padding here matches
  // what statusBarHeight() allows for.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
                      ImVec2(8.0F, kStatusBarPaddingY));
  ImGui::Begin("##StatusBar", nullptr, flags);

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

  // A loaded run says so wherever the user is looking, and says that what they
  // are watching is a simulation. Neither can be dismissed.
  if (m_replay.loaded()) {
    StatusPill(m_replay.summaryLine().c_str(), cyan, cyan);
    if (m_replay.simulated()) {
      StatusPill("SIMULATED", warn, warn);
    } else {
      StatusPill("REAL SYSTEM REPLAY", ok, ok);
    }
  } else if (m_simulation.isLoaded()) {
    StatusPill(m_simulation.summaryLine().c_str(), cyan, cyan);
    StatusPill("SIMULATED", warn, warn);
  }

  // Last operation pill (dim — informational)
  StatusPill(lastOperation.c_str(), dim, dim);

  ImGui::End();
  ImGui::PopStyleVar();
}
