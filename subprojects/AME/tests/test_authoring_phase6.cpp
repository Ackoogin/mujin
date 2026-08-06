#include <gtest/gtest.h>

#include "fact_action_matrix.h"
#include "failure_explainer.h"
#include "guided_editor_model.h"
#include "lifecycle_model.h"
#include "neighbourhood_model.h"
#include "pddl_importer.h"
#include "relation_index.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>

namespace {

std::string readFile(const std::string& path) {
  std::ifstream input(path);
  EXPECT_TRUE(input.is_open()) << path;
  return std::string((std::istreambuf_iterator<char>(input)),
                     std::istreambuf_iterator<char>());
}

ProjectModel loadDomain() {
  const std::string root = std::string(AME_DOMAINS_DIR) + "/mission_autonomy";
  const PddlImportResult imported =
      PddlImporter::importDomain(readFile(root + "/domain.pddl"));
  EXPECT_TRUE(imported.ok) << imported.error;
  return imported.model;
}

ProjectModel loadFailureScenario() {
  ProjectModel model = loadDomain();
  const std::string root = std::string(AME_DOMAINS_DIR) + "/mission_autonomy";
  const PddlImportResult imported = PddlImporter::importProblem(
      model, readFile(root + "/problem_comms_lost_realloc_needed.pddl"),
      "comms-lost-realloc-needed");
  EXPECT_TRUE(imported.ok) << imported.error;
  return imported.model;
}

} // namespace

TEST(FactActionMatrix, RepresentsMultipleMarksAndExportsEvidence) {
  const ProjectModel model = loadDomain();
  const RelationIndex index(model);
  const FactActionMatrix matrix(model, index);
  const int at = index.predicateIndex("at");
  const int move = index.actionIndex("move");
  ASSERT_GE(at, 0);
  ASSERT_GE(move, 0);

  const FactActionCell& cell =
      matrix.cell(static_cast<size_t>(at), static_cast<size_t>(move));
  EXPECT_TRUE(cell.requires);
  EXPECT_TRUE(cell.makesTrue);
  EXPECT_TRUE(cell.makesFalse);
  EXPECT_NE(matrix.toCsv(model).find("at,R + -"), std::string::npos);
  EXPECT_NE(matrix.toMarkdown(model).find("| comms-available |"),
            std::string::npos);
}

TEST(GuidedEditorModel, KeepsIllegalChoicesVisibleWithTypeReasons) {
  const ProjectModel model = loadDomain();
  const auto action = std::find_if(
      model.actions.begin(), model.actions.end(),
      [](const ActionDef& candidate) { return candidate.name == "withdraw-agent"; });
  ASSERT_NE(action, model.actions.end());

  const auto predicateChoices = guidedPredicateChoices(model, *action);
  ASSERT_EQ(predicateChoices.size(), model.predicates.size());
  const auto searched = std::find_if(
      predicateChoices.begin(), predicateChoices.end(),
      [](const GuidedEditorChoice& choice) { return choice.name == "searched"; });
  ASSERT_NE(searched, predicateChoices.end());
  EXPECT_FALSE(searched->legal);
  EXPECT_EQ(searched->reason, "applies to a sector");

  const auto available = std::find_if(
      predicateChoices.begin(), predicateChoices.end(),
      [](const GuidedEditorChoice& choice) {
        return choice.name == "agent-available";
      });
  ASSERT_NE(available, predicateChoices.end());
  EXPECT_TRUE(available->legal);
}

TEST(NeighbourhoodModel, TaskAssignedUsesThreeColumnsAndKeepsDuplicateRoles) {
  const ProjectModel model = loadDomain();
  const RelationIndex index(model);
  const int task_assigned = index.predicateIndex("task-assigned");
  ASSERT_GE(task_assigned, 0);
  const NeighbourhoodModel neighbourhood(
      model, index,
      {DomainElementKind::Predicate, static_cast<size_t>(task_assigned)});

  EXPECT_EQ(neighbourhood.nodes().size(), 7U);
  EXPECT_EQ(neighbourhood.edges().size(), 6U);
  size_t mark_task_failed_occurrences = 0;
  for (const auto& node : neighbourhood.nodes()) {
    if (node.element.kind == DomainElementKind::Action &&
        model.actions[node.element.index].name == "mark-task-failed") {
      ++mark_task_failed_occurrences;
    }
  }
  EXPECT_EQ(mark_task_failed_occurrences, 2U);
}

TEST(NeighbourhoodModel, CapsBusyFactsAndAddsMoreNode) {
  const ProjectModel model = loadDomain();
  const RelationIndex index(model);
  const int task_assigned = index.predicateIndex("task-assigned");
  const NeighbourhoodModel neighbourhood(
      model, index,
      {DomainElementKind::Predicate, static_cast<size_t>(task_assigned)},
      2, ShowEverything, 3);

  EXPECT_GT(neighbourhood.hiddenCount(), 0U);
  ASSERT_FALSE(neighbourhood.nodes().empty());
  EXPECT_EQ(neighbourhood.nodes().back().element.kind, DomainElementKind::More);
  EXPECT_EQ(neighbourhood.nodes().back().hiddenCount, neighbourhood.hiddenCount());
}

TEST(LifecycleModel, DerivesOnlyDeleteThenAddTransitionsWithinDeclaredGroup) {
  ProjectModel model = loadDomain();
  model.stateGroups = {
      {"availability", "agent", {"agent-available", "agent-unavailable"}},
      {"sensor state", "agent", {"sensor-operational", "sensor-degraded"}},
  };
  const RelationIndex index(model);
  const LifecycleModel lifecycles(model, index);

  ASSERT_EQ(lifecycles.diagrams().size(), 2U);
  ASSERT_EQ(lifecycles.diagrams()[0].transitions.size(), 1U);
  EXPECT_EQ(model.actions[lifecycles.diagrams()[0].transitions[0].actionIndex].name,
            "withdraw-agent");
  EXPECT_TRUE(lifecycles.diagrams()[1].transitions.empty());
}

TEST(FailureExplainer, ExplainsCommsLostReallocationWithReviewedConclusion) {
  const ProjectModel model = loadFailureScenario();
  const RelationIndex index(model);
  ASSERT_EQ(model.scenarios.size(), 1U);
  ASSERT_EQ(model.scenarios[0].goals.size(), 4U);
  const FailureExplanation explanation =
      FailureExplainer::explain(model, index, model.scenarios[0], 3);

  ASSERT_TRUE(explanation.available);
  std::string emitted_rows;
  for (const auto& row : explanation.rows) {
    emitted_rows += "\n" + row.text;
  }
  ASSERT_EQ(explanation.rows.size(), 5U) << emitted_rows;
  EXPECT_EQ(explanation.failedGoal.predicateName, "classified");
  EXPECT_EQ(explanation.blockingFact.predicateName, "comms-available");
  EXPECT_EQ(explanation.rows.back().text,
            "No action in this domain ever makes (comms-available) true, "
            "and the scenario does not start with it.");
  ASSERT_EQ(explanation.fixes.size(), 3U);
  EXPECT_EQ(explanation.fixes.back(), "Mark this scenario expected-to-fail");
}

TEST(FailureExplainer, ReportsDirectlyUnproducedGoal) {
  ProjectModel model = loadDomain();
  ScenarioDef scenario;
  scenario.goals.push_back({"comms-available", {}});
  const RelationIndex index(model);
  const FailureExplanation explanation =
      FailureExplainer::explain(model, index, scenario);

  ASSERT_EQ(explanation.rows.size(), 1U);
  EXPECT_EQ(explanation.blockingFact.predicateName, "comms-available");
}
