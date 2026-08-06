#include "neighbourhood_model.h"

#include <algorithm>

namespace {

constexpr float kColumnX[] = {40.0F, 390.0F, 720.0F};
constexpr float kFirstY = 70.0F;
constexpr float kRowHeight = 72.0F;

size_t columnIndex(NeighbourColumn column) {
  return static_cast<size_t>(column);
}

} // namespace

NeighbourhoodModel::NeighbourhoodModel(const ProjectModel& model,
                                       const RelationIndex& index,
                                       DomainElementRef focus,
                                       int depth,
                                       uint32_t filter,
                                       size_t neighbourCap) {
  nodes_.push_back({0, focus, NeighbourColumn::InFocus,
                    kColumnX[columnIndex(NeighbourColumn::InFocus)], 170.0F,
                    "in focus", 0});

  struct PendingNode {
    DomainElementRef element;
    NeighbourColumn column;
    PredicateRelationKind kind;
    std::string reason;
  };
  std::vector<PendingNode> pending;

  const auto appendPredicateRelations = [&](size_t predicate_index) {
    const PredicateRelations& relations = index.predicate(predicate_index);
    if ((filter & ShowMakesTrue) != 0U) {
      for (const auto& relation : relations.madeTrueBy) {
        pending.push_back({{DomainElementKind::Action, relation.actionIndex},
                           NeighbourColumn::ChangesIt,
                           PredicateRelationKind::MakesTrue, "makes it true"});
      }
    }
    if ((filter & ShowMakesFalse) != 0U) {
      for (const auto& relation : relations.madeFalseBy) {
        pending.push_back({{DomainElementKind::Action, relation.actionIndex},
                           NeighbourColumn::ChangesIt,
                           PredicateRelationKind::MakesFalse, "makes it false"});
      }
    }
    if ((filter & ShowRequires) != 0U) {
      for (const auto& relation : relations.requiredBy) {
        pending.push_back({{DomainElementKind::Action, relation.actionIndex},
                           NeighbourColumn::NeedsIt,
                           PredicateRelationKind::Requires, "needs it"});
      }
    }
  };

  const auto appendActionRelations = [&](size_t action_index) {
    const ActionRelations& relations = index.action(action_index);
    if ((filter & ShowRequires) != 0U) {
      for (const auto& relation : relations.requires) {
        pending.push_back({{DomainElementKind::Predicate, relation.predicateIndex},
                           NeighbourColumn::ChangesIt,
                           PredicateRelationKind::Requires, "required before"});
      }
    }
    if ((filter & ShowMakesTrue) != 0U) {
      for (const auto& relation : relations.makesTrue) {
        pending.push_back({{DomainElementKind::Predicate, relation.predicateIndex},
                           NeighbourColumn::NeedsIt,
                           PredicateRelationKind::MakesTrue, "becomes true"});
      }
    }
    if ((filter & ShowMakesFalse) != 0U) {
      for (const auto& relation : relations.makesFalse) {
        pending.push_back({{DomainElementKind::Predicate, relation.predicateIndex},
                           NeighbourColumn::NeedsIt,
                           PredicateRelationKind::MakesFalse, "becomes false"});
      }
    }
  };

  if (focus.kind == DomainElementKind::Predicate && focus.index < model.predicates.size()) {
    appendPredicateRelations(focus.index);
  } else if (focus.kind == DomainElementKind::Action && focus.index < model.actions.size()) {
    appendActionRelations(focus.index);
  }

  if (depth >= 2) {
    const std::vector<PendingNode> first_step = pending;
    for (const PendingNode& neighbour : first_step) {
      if (neighbour.element.kind != DomainElementKind::Action) {
        continue;
      }
      const ActionRelations& relations = index.action(neighbour.element.index);
      for (const auto& enabled : relations.mayEnable) {
        pending.push_back({{DomainElementKind::Action, enabled},
                           NeighbourColumn::NeedsIt,
                           PredicateRelationKind::MakesTrue, "may enable"});
      }
      for (const auto& enabling : relations.mayBeEnabledBy) {
        pending.push_back({{DomainElementKind::Action, enabling},
                           NeighbourColumn::ChangesIt,
                           PredicateRelationKind::MakesTrue, "may be enabled by"});
      }
    }
  }

  hidden_count_ = pending.size() > neighbourCap ? pending.size() - neighbourCap : 0U;
  pending.resize(std::min(pending.size(), neighbourCap));
  size_t column_rows[] = {0U, 0U, 0U};
  int next_id = 1;
  for (const PendingNode& item : pending) {
    const size_t column = columnIndex(item.column);
    const float y = kFirstY + kRowHeight * static_cast<float>(column_rows[column]++);
    nodes_.push_back({next_id, item.element, item.column, kColumnX[column], y,
                      item.reason, 0});
    if (item.column == NeighbourColumn::ChangesIt) {
      edges_.push_back({next_id, 0, item.kind});
    } else {
      edges_.push_back({0, next_id, item.kind});
    }
    ++next_id;
  }

  if (hidden_count_ > 0U) {
    const size_t column = columnIndex(NeighbourColumn::NeedsIt);
    nodes_.push_back({next_id, {DomainElementKind::More, 0},
                      NeighbourColumn::NeedsIt, kColumnX[column],
                      kFirstY + kRowHeight * static_cast<float>(column_rows[column]),
                      "+ more", hidden_count_});
  }
}
