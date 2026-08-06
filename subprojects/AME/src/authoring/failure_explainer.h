#pragma once

#include "project_model.h"
#include "relation_index.h"

#include <string>
#include <vector>

enum class FailureExplanationKind {
  Producer,
  Requirement,
  RemovedAndRestored,
  Conclusion,
};

struct FailureExplanationRow {
  FailureExplanationKind kind = FailureExplanationKind::Requirement;
  std::string text;
  std::string actionName;
  FactRef fact;
};

struct FailureExplanation {
  bool available = false;
  FactRef failedGoal;
  FactRef blockingFact;
  std::vector<FailureExplanationRow> rows;
  std::vector<std::string> fixes;
};

/// \brief Explain an unreachable goal by walking backwards over the relation index.
class FailureExplainer {
public:
  static FailureExplanation explain(const ProjectModel& model,
                                    const RelationIndex& index,
                                    const ScenarioDef& scenario,
                                    size_t goalIndex = 0);

  static std::string formatFact(const FactRef& fact);
};
