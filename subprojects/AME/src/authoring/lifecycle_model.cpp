#include "lifecycle_model.h"

#include <algorithm>

LifecycleModel::LifecycleModel(const ProjectModel& model,
                               const RelationIndex& index) {
  for (size_t group_index = 0; group_index < model.stateGroups.size(); ++group_index) {
    const StateGroupDef& group = model.stateGroups[group_index];
    LifecycleDiagram diagram;
    diagram.groupIndex = group_index;
    for (const auto& predicate_name : group.predicateNames) {
      const int predicate_index = index.predicateIndex(predicate_name);
      if (predicate_index >= 0) {
        diagram.predicateIndices.push_back(static_cast<size_t>(predicate_index));
      }
    }

    for (size_t action_index = 0; action_index < model.actions.size(); ++action_index) {
      const ActionRelations& action = index.action(action_index);
      for (const auto& removed : action.makesFalse) {
        if (std::find(diagram.predicateIndices.begin(), diagram.predicateIndices.end(),
                      removed.predicateIndex) == diagram.predicateIndices.end()) {
          continue;
        }
        for (const auto& added : action.makesTrue) {
          if (std::find(diagram.predicateIndices.begin(), diagram.predicateIndices.end(),
                        added.predicateIndex) != diagram.predicateIndices.end()) {
            diagram.transitions.push_back(
                {action_index, removed.predicateIndex, added.predicateIndex});
          }
        }
      }
    }
    diagrams_.push_back(std::move(diagram));
  }
}
