#include "assurance_report.h"

#include "contingency_analyser.h"
#include "relation_index.h"
#include "scenario_runner.h"
#include "structural_validator.h"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string outcomeWord(ScenarioOutcome outcome) {
  switch (outcome) {
  case ScenarioOutcome::Pass:
    return "as expected";
  case ScenarioOutcome::Fail:
    return "not as expected";
  case ScenarioOutcome::Error:
    break;
  }
  return "could not be run";
}

/// A row of a Markdown table, with the pipes and the padding.
void row(std::ostringstream& out,
         const std::vector<std::string>& cells) {
  out << '|';
  for (const std::string& cell : cells) {
    out << ' ' << cell << " |";
  }
  out << '\n';
}

void tableHeader(std::ostringstream& out,
                 const std::vector<std::string>& headings) {
  row(out, headings);
  out << '|';
  for (size_t i = 0; i < headings.size(); ++i) {
    out << "---|";
  }
  out << '\n';
}

/// What an action is bound to, in words rather than in field names.
std::string describeBinding(const ActionDef& action) {
  if (!action.btBinding.subtreeXml.empty()) {
    return std::string("a subtree of its own") +
           (action.btBinding.reactive ? ", re-checked while it runs" : "");
  }
  if (!action.btBinding.nodeType.empty()) {
    return action.btBinding.nodeType +
           (action.btBinding.reactive ? ", re-checked while it runs" : "");
  }
  return "";
}

}  // namespace

std::string AssuranceReport::generate(const ProjectModel& model,
                                      const std::string& generatedOn) {
  std::ostringstream out;

  out << "# Assurance evidence: " << model.projectName << "\n\n";
  out << "Generated from the mission model";
  if (!generatedOn.empty()) {
    out << " on " << generatedOn;
  }
  out << ". Everything below is read from the model itself, so this report and\n"
      << "the model cannot drift apart. It is evidence about a mission model,\n"
      << "not about a system in the field: the runs it reports are simulations,\n"
      << "in which every action is a stand-in.\n\n";
  out << "The framework this feeds is described in "
      << "`doc/plans/AME/autonomy_assurance_plan.md`.\n\n";

  // ---- What the model contains -----------------------------------------
  out << "## 1. What the model contains\n\n";
  tableHeader(out, {"", "How many"});
  row(out, {"Types", std::to_string(model.types.size())});
  row(out, {"Things", std::to_string(model.objects.size())});
  row(out, {"Facts", std::to_string(model.predicates.size())});
  row(out, {"Actions", std::to_string(model.actions.size())});
  row(out, {"Scenarios", std::to_string(model.scenarios.size())});
  out << '\n';

  const StructuralReport structural = StructuralValidator::check(model);
  if (structural.hasErrors()) {
    out << "**This model has " << structural.errorCount
        << " structural error(s).** Nothing below should be relied on until\n"
        << "they are fixed, because a model that does not hold together cannot\n"
        << "be planned for or run.\n\n";
  }

  // ---- Facts nothing produces -------------------------------------------
  out << "## 2. Facts that nothing in the mission brings about\n\n";
  const RelationIndex relations(model);
  const std::vector<size_t>& unproduced = relations.factsNoActionMakesTrue();
  if (unproduced.empty()) {
    out << "Every fact in this model is made true by some action.\n\n";
  } else {
    out << "These facts are never made true by any action. Each one is either\n"
        << "something the world decides, which the system must be told about,\n"
        << "or a gap in the model. **Which of the two is a judgement this\n"
        << "report cannot make**, and it is worth a reviewer's attention.\n\n";
    tableHeader(out, {"Fact", "Used as a condition by"});
    for (const size_t index : unproduced) {
      if (index >= model.predicates.size()) {
        continue;
      }
      const std::string& name = model.predicates[index].name;
      std::vector<std::string> users;
      for (const ActionDef& action : model.actions) {
        const bool uses = std::any_of(action.preconditions.begin(),
                                      action.preconditions.end(),
                                      [&name](const EffectRef& reference) {
                                        return reference.predicateName == name;
                                      });
        if (uses) {
          users.push_back(action.name);
        }
      }
      std::string joined;
      for (size_t i = 0; i < users.size(); ++i) {
        joined += (i == 0 ? "" : ", ") + users[i];
      }
      row(out, {name, joined.empty() ? "nothing" : joined});
    }
    out << '\n';
  }

  // ---- What can execute ---------------------------------------------------
  out << "## 3. What each action is bound to\n\n";
  out << "An action with nothing bound to it can be planned but not carried\n"
      << "out. In a simulated run it is stood in for; in the field there would\n"
      << "be nothing to call.\n\n";
  size_t unbound = 0;
  tableHeader(out, {"Action", "Bound to"});
  for (const ActionDef& action : model.actions) {
    const std::string binding = describeBinding(action);
    if (binding.empty()) {
      ++unbound;
    }
    row(out, {action.name, binding.empty() ? "**nothing**" : binding});
  }
  out << '\n';
  if (unbound > 0) {
    out << "**" << unbound << " of " << model.actions.size()
        << " actions have nothing bound to them.**\n\n";
  }

  // ---- Facts that must be observed ---------------------------------------
  std::vector<std::string> confirmed;
  for (const PredicateDef& predicate : model.predicates) {
    if (predicate.confirmed) {
      confirmed.push_back(predicate.name);
    }
  }
  out << "## 4. Facts an action may act on only once observed\n\n";
  if (confirmed.empty()) {
    out << "This model declares none. Every action may proceed on a fact an\n"
        << "earlier step was expected to bring about, without waiting for\n"
        << "anything to report it.\n\n";
  } else {
    out << "These facts must be reported by something before an action waiting\n"
        << "on them will start. A plan effect alone does not satisfy them.\n\n";
    for (const std::string& name : confirmed) {
      out << "- " << name << '\n';
    }
    out << '\n';
  }

  // ---- Scenarios ----------------------------------------------------------
  out << "## 5. Scenarios, and how they behaved\n\n";
  if (model.scenarios.empty()) {
    out << "**This model has no scenarios, so nothing has been run at all.**\n\n";
  } else {
    const ScenarioBatchReport batch = ScenarioRunner::runAll(model);
    out << batch.passCount << " of " << batch.results.size()
        << " scenarios behaved as the model says they should.\n\n";
    tableHeader(out, {"Scenario", "Outcome", "Goal", "Actions", "Replans",
                      "Why"});
    for (const ScenarioRunResult& result : batch.results) {
      row(out, {result.scenarioName, outcomeWord(result.outcome),
                result.goalReached ? "reached" : "not reached",
                std::to_string(result.runActionCount),
                std::to_string(result.replanCount),
                result.reason.empty() ? "" : result.reason});
    }
    out << "\nEvery run above is a simulation, repeated from seed "
        << model.simulationSeed << ".\n\n";
  }

  // ---- Contingencies ------------------------------------------------------
  out << "## 6. Contingencies, and what stayed reachable\n\n";
  std::vector<std::string> declaring;
  for (const ScenarioDef& scenario : model.scenarios) {
    if (!scenario.contingency.isEmpty()) {
      declaring.push_back(scenario.name);
    }
  }
  if (declaring.empty()) {
    out << "**No scenario declares what counts as a contingency or as a safe\n"
        << "state, so no contingency has been checked.** Until one does, this\n"
        << "model carries no evidence that a safe state stays reachable when\n"
        << "something goes wrong.\n\n";
  } else {
    for (const std::string& name : declaring) {
      const ContingencyReport report = ContingencyAnalyser::analyse(model, name);
      out << "### " << name << "\n\n";
      if (!report.ok) {
        out << "This could not be checked: " << report.error << "\n\n";
        continue;
      }
      out << report.coverageSentence() << "\n\n";
      out << "A safe state was reachable in " << report.feasibleCount
          << " of " << report.results.size()
          << " of them.\n\n";
      if (report.infeasibleCount > 0) {
        out << "**" << report.infeasibleCount
            << " leave no way back to a safe state.** Each is a combination of\n"
            << "circumstances this mission cannot recover from:\n\n";
        size_t shown = 0;
        for (const ContingencyContext& context : report.results) {
          if (context.planFound || shown >= 10) {
            continue;
          }
          std::string trueFacts;
          for (size_t i = 0; i < context.trueFluents.size(); ++i) {
            trueFacts += (i == 0 ? "" : ", ") + context.trueFluents[i];
          }
          out << "- " << (trueFacts.empty() ? "nothing true" : trueFacts) << '\n';
          ++shown;
        }
        if (report.infeasibleCount > shown) {
          out << "- and " << (report.infeasibleCount - shown) << " more\n";
        }
        out << '\n';
      }
    }
  }

  // ---- What this does not cover ------------------------------------------
  out << "## 7. What this report does not tell you\n\n";
  out << "- **Nothing here is evidence about the field.** Every run is a\n"
      << "  simulation in which each action is a stand-in that takes a set\n"
      << "  number of ticks and then reports the outcome it was told to. A run\n"
      << "  shows what the model says would happen, not what the vehicle would\n"
      << "  do.\n";
  out << "- **A scenario that behaved as expected is not a scenario that is\n"
      << "  right.** It means the model matched what somebody wrote down about\n"
      << "  it, and whether that was the right thing to write down is outside\n"
      << "  this report.\n";
  const size_t withoutScenarios =
      model.scenarios.empty() ? 1U : 0U;
  if (withoutScenarios == 0U && declaring.size() < model.scenarios.size()) {
    const size_t silent = model.scenarios.size() - declaring.size();
    out << "- **" << silent << " of " << model.scenarios.size()
        << (silent == 1U ? " scenarios declares" : " scenarios declare")
        << " no contingency**, so nothing has been checked\n"
        << "  about what happens to "
        << (silent == 1U ? "it" : "them") << " when something goes wrong.\n";
  }
  if (unbound > 0) {
    out << "- **" << unbound << (unbound == 1U ? " action has" : " actions have")
        << " nothing bound to them**, so no evidence here says\n"
        << "  they can be carried out at all.\n";
  }
  out << "- The planner, the world model and the compiled tree used here are\n"
      << "  the runtime's own, so what this report says about feasibility and\n"
      << "  reachability holds for the runtime too. What it cannot speak for is\n"
      << "  anything the model does not describe.\n";

  return out.str();
}
