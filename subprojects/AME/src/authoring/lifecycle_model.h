#pragma once

#include "project_model.h"
#include "relation_index.h"

#include <vector>

/// \brief One action-derived transition between two declared states.
struct LifecycleTransition {
  size_t actionIndex = 0;
  size_t fromPredicateIndex = 0;
  size_t toPredicateIndex = 0;
};

/// \brief The state and transition data for one user-declared grouping.
struct LifecycleDiagram {
  size_t groupIndex = 0;
  std::vector<size_t> predicateIndices;
  std::vector<LifecycleTransition> transitions;
};

/// \brief Build per-type lifecycle diagrams from user-declared state groupings.
class LifecycleModel {
public:
  LifecycleModel(const ProjectModel& model, const RelationIndex& index);
  const std::vector<LifecycleDiagram>& diagrams() const { return diagrams_; }

private:
  std::vector<LifecycleDiagram> diagrams_;
};
