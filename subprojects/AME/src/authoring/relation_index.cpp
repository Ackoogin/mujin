#include "relation_index.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace {

const PredicateRelations kEmptyPredicateRelations;
const ActionRelations kEmptyActionRelations;

std::vector<std::string> argumentTypes(const ActionDef& action,
                                       const std::string& argument) {
  const auto found = std::find_if(action.params.begin(), action.params.end(),
                                  [&argument](const Parameter& parameter) {
                                    return parameter.name == argument;
                                  });
  if (found == action.params.end()) {
    return {};
  }
  if (!found->eitherTypes.empty()) {
    return found->eitherTypes;
  }
  return {found->type};
}

bool isSubtype(const ProjectModel& model,
               const std::string& child,
               const std::string& parent) {
  if (child.empty() || parent.empty() || child == parent) {
    return true;
  }

  std::string current = child;
  std::unordered_set<std::string> visited;
  while (!current.empty() && visited.insert(current).second) {
    const auto found = std::find_if(model.types.begin(), model.types.end(),
                                    [&current](const TypeDef& type) {
                                      return type.name == current;
                                    });
    if (found == model.types.end()) {
      return false;
    }
    current = found->parent;
    if (current == parent) {
      return true;
    }
  }
  return false;
}

bool typesOverlap(const ProjectModel& model,
                  const std::string& first,
                  const std::string& second) {
  return first.empty() || second.empty() ||
         isSubtype(model, first, second) || isSubtype(model, second, first);
}

bool anyTypesOverlap(const ProjectModel& model,
                     const std::vector<std::string>& first,
                     const std::vector<std::string>& second) {
  if (first.empty() || second.empty()) {
    return true;
  }
  for (const std::string& first_type : first) {
    for (const std::string& second_type : second) {
      if (typesOverlap(model, first_type, second_type)) {
        return true;
      }
    }
  }
  return false;
}

void appendUnique(std::vector<size_t>& values, size_t value) {
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

} // namespace

bool relationTypeCompatible(const ProjectModel& model,
                            size_t fromAction,
                            size_t fromAddEffect,
                            size_t toAction,
                            size_t toPrecondition) {
  if (fromAction >= model.actions.size() || toAction >= model.actions.size() ||
      fromAction == toAction) {
    return false;
  }

  const ActionDef& source = model.actions[fromAction];
  const ActionDef& target = model.actions[toAction];
  const std::vector<EffectRef> target_conditions = actionConditionFacts(target);
  if (fromAddEffect >= source.addEffects.size() ||
      toPrecondition >= target_conditions.size()) {
    return false;
  }

  const EffectRef& effect = source.addEffects[fromAddEffect];
  const EffectRef& precondition = target_conditions[toPrecondition];
  if (precondition.negated) {
    return false;
  }
  if (effect.predicateName != precondition.predicateName ||
      effect.argNames.size() != precondition.argNames.size()) {
    return false;
  }

  for (size_t i = 0; i < effect.argNames.size(); ++i) {
    if (!anyTypesOverlap(model,
                         argumentTypes(source, effect.argNames[i]),
                         argumentTypes(target, precondition.argNames[i]))) {
      return false;
    }
  }
  return true;
}

RelationIndex::RelationIndex(const ProjectModel& model) {
  predicate_relations_.resize(model.predicates.size());
  action_relations_.resize(model.actions.size());
  predicate_names_.reserve(model.predicates.size());
  action_names_.reserve(model.actions.size());

  std::unordered_map<std::string, size_t> predicate_indices;
  for (size_t i = 0; i < model.predicates.size(); ++i) {
    predicate_names_.push_back(model.predicates[i].name);
    predicate_indices.emplace(model.predicates[i].name, i);
  }
  for (const auto& action : model.actions) {
    action_names_.push_back(action.name);
  }

  const auto addReferences = [&](size_t action_index,
                                 const std::vector<EffectRef>& references,
                                 PredicateRelationKind kind) {
    for (size_t reference_index = 0; reference_index < references.size(); ++reference_index) {
      const auto found = predicate_indices.find(references[reference_index].predicateName);
      if (found == predicate_indices.end()) {
        continue;
      }
      const size_t predicate_index = found->second;
      const PredicateRelationKind actual_kind =
          kind == PredicateRelationKind::Requires && references[reference_index].negated
              ? PredicateRelationKind::RequiresFalse
              : kind == PredicateRelationKind::Requires &&
                    references[reference_index].alternative
                  ? PredicateRelationKind::AcceptsAlternative : kind;
      PredicateActionRelation predicate_relation{action_index, reference_index,
                                                  actual_kind};
      ActionPredicateRelation action_relation{predicate_index, reference_index,
                                              actual_kind};
      if (actual_kind == PredicateRelationKind::Requires) {
        predicate_relations_[predicate_index].requiredBy.push_back(predicate_relation);
        action_relations_[action_index].requires.push_back(action_relation);
      } else if (actual_kind == PredicateRelationKind::RequiresFalse) {
        predicate_relations_[predicate_index].requiredFalseBy.push_back(predicate_relation);
        action_relations_[action_index].requiresFalse.push_back(action_relation);
      } else if (actual_kind == PredicateRelationKind::AcceptsAlternative) {
        predicate_relations_[predicate_index].acceptedAsAlternativeBy.push_back(
            predicate_relation);
        action_relations_[action_index].acceptsAlternatives.push_back(action_relation);
      } else if (actual_kind == PredicateRelationKind::MakesTrue) {
        predicate_relations_[predicate_index].madeTrueBy.push_back(predicate_relation);
        action_relations_[action_index].makesTrue.push_back(action_relation);
      } else {
        predicate_relations_[predicate_index].madeFalseBy.push_back(predicate_relation);
        action_relations_[action_index].makesFalse.push_back(action_relation);
      }
      ++link_count_;
    }
  };

  for (size_t action_index = 0; action_index < model.actions.size(); ++action_index) {
    const ActionDef& action = model.actions[action_index];
    const std::vector<EffectRef> conditions = actionConditionFacts(action);
    addReferences(action_index, conditions, PredicateRelationKind::Requires);
    addReferences(action_index, action.addEffects, PredicateRelationKind::MakesTrue);
    addReferences(action_index, action.delEffects, PredicateRelationKind::MakesFalse);
  }

  for (size_t predicate_index = 0;
       predicate_index < predicate_relations_.size();
       ++predicate_index) {
    if (predicate_relations_[predicate_index].madeTrueBy.empty()) {
      facts_no_action_makes_true_.push_back(predicate_index);
    }
  }

  for (size_t from_action = 0; from_action < model.actions.size(); ++from_action) {
    for (size_t add_index = 0;
         add_index < model.actions[from_action].addEffects.size();
         ++add_index) {
      const int predicate_index =
          this->predicateIndex(model.actions[from_action].addEffects[add_index].predicateName);
      if (predicate_index < 0) {
        continue;
      }
      for (size_t to_action = 0; to_action < model.actions.size(); ++to_action) {
        const std::vector<EffectRef> target_conditions =
            actionConditionFacts(model.actions[to_action]);
        for (size_t precondition_index = 0;
             precondition_index < target_conditions.size();
             ++precondition_index) {
          if (!relationTypeCompatible(model, from_action, add_index,
                                      to_action, precondition_index)) {
            continue;
          }
          causal_links_.push_back({from_action, add_index, to_action,
                                   precondition_index,
                                   static_cast<size_t>(predicate_index)});
          appendUnique(action_relations_[from_action].mayEnable, to_action);
          appendUnique(action_relations_[to_action].mayBeEnabledBy, from_action);
        }
      }
    }
  }
}

const PredicateRelations& RelationIndex::predicate(size_t predicateIndex) const {
  return predicateIndex < predicate_relations_.size()
             ? predicate_relations_[predicateIndex]
             : kEmptyPredicateRelations;
}

const ActionRelations& RelationIndex::action(size_t actionIndex) const {
  return actionIndex < action_relations_.size()
             ? action_relations_[actionIndex]
             : kEmptyActionRelations;
}

int RelationIndex::predicateIndex(const std::string& name) const {
  const auto found = std::find(predicate_names_.begin(), predicate_names_.end(), name);
  return found == predicate_names_.end()
             ? -1
             : static_cast<int>(std::distance(predicate_names_.begin(), found));
}

int RelationIndex::actionIndex(const std::string& name) const {
  const auto found = std::find(action_names_.begin(), action_names_.end(), name);
  return found == action_names_.end()
             ? -1
             : static_cast<int>(std::distance(action_names_.begin(), found));
}

const std::vector<size_t>& RelationIndex::factsNoActionMakesTrue() const {
  return facts_no_action_makes_true_;
}

const std::vector<DerivedCausalLink>& RelationIndex::causalLinks() const {
  return causal_links_;
}
