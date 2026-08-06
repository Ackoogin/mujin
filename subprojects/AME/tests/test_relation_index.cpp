#include <gtest/gtest.h>

#include "pddl_importer.h"
#include "relation_index.h"

#include <fstream>
#include <iterator>
#include <set>
#include <string>

namespace {

ProjectModel loadMissionAutonomyDomain() {
  const std::string path =
      std::string(AME_DOMAINS_DIR) + "/mission_autonomy/domain.pddl";
  std::ifstream input(path);
  EXPECT_TRUE(input.is_open()) << path;
  const std::string pddl((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  const PddlImportResult imported = PddlImporter::importDomain(pddl);
  EXPECT_TRUE(imported.ok) << imported.error;
  return imported.model;
}

std::set<std::string> actionNames(const ProjectModel& model,
                                  const std::vector<PredicateActionRelation>& relations) {
  std::set<std::string> names;
  for (const auto& relation : relations) {
    names.insert(model.actions[relation.actionIndex].name);
  }
  return names;
}

} // namespace

TEST(RelationIndex, MissionAutonomyCountsMatchReviewedConcept) {
  const ProjectModel model = loadMissionAutonomyDomain();
  const RelationIndex index(model);

  EXPECT_EQ(index.predicateCount(), 10U);
  EXPECT_EQ(index.actionCount(), 7U);
  EXPECT_EQ(index.linkCount(), 34U);
}

TEST(RelationIndex, TaskAssignedAppearsInEveryLegitimateList) {
  const ProjectModel model = loadMissionAutonomyDomain();
  const RelationIndex index(model);
  const int predicate_index = index.predicateIndex("task-assigned");
  ASSERT_GE(predicate_index, 0);
  const PredicateRelations& relations =
      index.predicate(static_cast<size_t>(predicate_index));

  EXPECT_EQ(actionNames(model, relations.requiredBy),
            (std::set<std::string>{"classify", "mark-task-failed", "search",
                                   "search-degraded"}));
  EXPECT_EQ(actionNames(model, relations.madeTrueBy),
            (std::set<std::string>{"reallocate-task"}));
  EXPECT_EQ(actionNames(model, relations.madeFalseBy),
            (std::set<std::string>{"mark-task-failed"}));
}

TEST(RelationIndex, FindsFourFactsNoActionMakesTrue) {
  const ProjectModel model = loadMissionAutonomyDomain();
  const RelationIndex index(model);
  std::set<std::string> names;
  for (const size_t predicate_index : index.factsNoActionMakesTrue()) {
    names.insert(model.predicates[predicate_index].name);
  }

  EXPECT_EQ(names, (std::set<std::string>{"agent-available", "comms-available",
                                         "sensor-degraded", "sensor-operational"}));
}

TEST(RelationIndex, DerivesActionEnablingRelationships) {
  const ProjectModel model = loadMissionAutonomyDomain();
  const RelationIndex index(model);
  const int reallocate_index = index.actionIndex("reallocate-task");
  const int search_index = index.actionIndex("search");
  ASSERT_GE(reallocate_index, 0);
  ASSERT_GE(search_index, 0);

  const auto& enabled = index.action(static_cast<size_t>(reallocate_index)).mayEnable;
  EXPECT_NE(std::find(enabled.begin(), enabled.end(), static_cast<size_t>(search_index)),
            enabled.end());
  EXPECT_FALSE(index.causalLinks().empty());
}

TEST(RelationIndex, RejectsCausalRelationshipWhenParameterTypesDoNotOverlap) {
  ProjectModel model;
  model.types = {{"agent", "object"}, {"sector", "object"}};
  model.predicates = {{"ready", {{"?x", "object"}}}};
  ActionDef source;
  source.name = "source";
  source.params = {{"?a", "agent"}};
  source.addEffects = {{"ready", {"?a"}}};
  ActionDef target;
  target.name = "target";
  target.params = {{"?s", "sector"}};
  target.preconditions = {{"ready", {"?s"}}};
  model.actions = {source, target};

  EXPECT_FALSE(relationTypeCompatible(model, 0, 0, 1, 0));
  EXPECT_TRUE(RelationIndex(model).causalLinks().empty());
}
