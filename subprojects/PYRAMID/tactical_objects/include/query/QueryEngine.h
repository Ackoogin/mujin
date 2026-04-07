#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectStore.h>
#include <spatial/SpatialIndex.h>
#include <tl/optional.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tactical_objects {

struct QueryRequest {
  tl::optional<UUIDKey> by_uuid;
  tl::optional<std::string> by_source_system;
  tl::optional<std::string> by_source_entity_id;
  tl::optional<ObjectType> by_type;
  tl::optional<Affiliation> by_affiliation;
  tl::optional<BoundingBox> by_region;
  tl::optional<double> max_age_seconds;
  double current_time = 0.0;
};

struct QueryResultEntry {
  UUIDKey id;
  EntityRecord record;
};

struct QueryResponse {
  std::vector<QueryResultEntry> entries;
  size_t total = 0;
};

/// \brief Compound predicate queries over the object store and indexes.
class QueryEngine {
public:
  QueryEngine(std::shared_ptr<ObjectStore> store,
              std::shared_ptr<SpatialIndex> spatial);

  /// \brief Evaluate a compound query and return matching entities.
  QueryResponse query(const QueryRequest& req) const;

private:
  std::shared_ptr<ObjectStore> store_;
  std::shared_ptr<SpatialIndex> spatial_;

  bool matchesPredicates(const UUIDKey& id, const QueryRequest& req) const;
};

} // namespace tactical_objects
