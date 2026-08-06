#pragma once

#include "project_model.h"

#include <cstddef>
#include <string>
#include <vector>

/// \brief The relationship represented by one predicate reference in an action.
enum class PredicateRelationKind {
  Requires,
  MakesTrue,
  MakesFalse,
};

/// \brief One action reference associated with a predicate.
struct PredicateActionRelation {
  size_t actionIndex = 0;
  size_t referenceIndex = 0;
  PredicateRelationKind kind = PredicateRelationKind::Requires;
};

/// \brief All action references associated with one predicate.
struct PredicateRelations {
  std::vector<PredicateActionRelation> requiredBy;
  std::vector<PredicateActionRelation> madeTrueBy;
  std::vector<PredicateActionRelation> madeFalseBy;
};

/// \brief One predicate reference associated with an action.
struct ActionPredicateRelation {
  size_t predicateIndex = 0;
  size_t referenceIndex = 0;
  PredicateRelationKind kind = PredicateRelationKind::Requires;
};

/// \brief A computed action-to-action relationship.
struct DerivedCausalLink {
  size_t fromAction = 0;
  size_t fromAddEffectIndex = 0;
  size_t toAction = 0;
  size_t toPreconditionIndex = 0;
  size_t predicateIndex = 0;
};

/// \brief All predicate and action relationships associated with one action.
struct ActionRelations {
  std::vector<ActionPredicateRelation> requires;
  std::vector<ActionPredicateRelation> makesTrue;
  std::vector<ActionPredicateRelation> makesFalse;
  std::vector<size_t> mayEnable;
  std::vector<size_t> mayBeEnabledBy;
};

/// \brief A cheap, read-only relationship index built from a ProjectModel.
class RelationIndex {
public:
  explicit RelationIndex(const ProjectModel& model);

  /// \brief Return relationships for a predicate, or an empty value for an invalid index.
  const PredicateRelations& predicate(size_t predicateIndex) const;

  /// \brief Return relationships for an action, or an empty value for an invalid index.
  const ActionRelations& action(size_t actionIndex) const;

  /// \brief Return the predicate index for a name, or -1 when it does not exist.
  int predicateIndex(const std::string& name) const;

  /// \brief Return the action index for a name, or -1 when it does not exist.
  int actionIndex(const std::string& name) const;

  /// \brief Return predicates which no action ever makes true.
  const std::vector<size_t>& factsNoActionMakesTrue() const;

  /// \brief Return every derived action-to-action enabling relationship.
  const std::vector<DerivedCausalLink>& causalLinks() const;

  size_t predicateCount() const { return predicate_relations_.size(); }
  size_t actionCount() const { return action_relations_.size(); }
  size_t linkCount() const { return link_count_; }

private:
  std::vector<PredicateRelations> predicate_relations_;
  std::vector<ActionRelations> action_relations_;
  std::vector<size_t> facts_no_action_makes_true_;
  std::vector<DerivedCausalLink> causal_links_;
  std::vector<std::string> predicate_names_;
  std::vector<std::string> action_names_;
  size_t link_count_ = 0;
};

/// \brief Return whether an add effect can satisfy a precondition after grounding.
bool relationTypeCompatible(const ProjectModel& model,
                            size_t fromAction,
                            size_t fromAddEffect,
                            size_t toAction,
                            size_t toPrecondition);
