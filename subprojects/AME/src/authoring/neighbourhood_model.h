#pragma once

#include "project_model.h"
#include "relation_index.h"

#include <cstdint>
#include <string>
#include <vector>

enum class DomainElementKind { Predicate, Action, More };
enum class NeighbourColumn { ChangesIt, InFocus, NeedsIt };

struct DomainElementRef {
  DomainElementKind kind = DomainElementKind::Predicate;
  size_t index = 0;
};

struct NeighbourNode {
  int id = 0;
  DomainElementRef element;
  NeighbourColumn column = NeighbourColumn::InFocus;
  float x = 0.0F;
  float y = 0.0F;
  std::string reason;
  size_t hiddenCount = 0;
};

struct NeighbourEdge {
  int fromNode = 0;
  int toNode = 0;
  PredicateRelationKind kind = PredicateRelationKind::Requires;
};

enum NeighbourRelationshipFilter : uint32_t {
  ShowRequires = 1U,
  ShowMakesTrue = 2U,
  ShowMakesFalse = 4U,
  ShowEverything = ShowRequires | ShowMakesTrue | ShowMakesFalse,
};

/// \brief A computed, capped, three-column neighbourhood layout.
class NeighbourhoodModel {
public:
  NeighbourhoodModel(const ProjectModel& model,
                     const RelationIndex& index,
                     DomainElementRef focus,
                     int depth = 1,
                     uint32_t filter = ShowEverything,
                     size_t neighbourCap = 20);

  const std::vector<NeighbourNode>& nodes() const { return nodes_; }
  const std::vector<NeighbourEdge>& edges() const { return edges_; }
  size_t hiddenCount() const { return hidden_count_; }

private:
  std::vector<NeighbourNode> nodes_;
  std::vector<NeighbourEdge> edges_;
  size_t hidden_count_ = 0;
};
