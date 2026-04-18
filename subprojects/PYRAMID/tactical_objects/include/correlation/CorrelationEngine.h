#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectStore.h>
#include <spatial/SpatialIndex.h>
#include <milclass/MilClassEngine.h>

#include <memory>
#include <string>

namespace tactical_objects {

struct CorrelationConfig {
  double merge_threshold = 0.8;
  double create_threshold = 0.3;
  double gate_radius_rad = 0.008726646259971648;  // 0.5° in radians
  int split_incompatibility_count = 3;
};

enum class CorrelationOutcome {
  Created,
  Merged,
  Split
};

struct CorrelationResult {
  CorrelationOutcome outcome;
  UUIDKey object_id;
  UUIDKey track_id;
};

/// \brief Evidence-based entity creation via correlation, merge, split, and lineage.
class CorrelationEngine {
public:
  CorrelationEngine(std::shared_ptr<ObjectStore> store,
                    std::shared_ptr<SpatialIndex> spatial,
                    std::shared_ptr<MilClassEngine> milclass,
                    CorrelationConfig config = CorrelationConfig{});

  /// \brief Process a single observation through the correlation pipeline.
  CorrelationResult processObservation(const Observation& obs);

  /// \brief Read-only access to current config.
  const CorrelationConfig& config() const { return config_; }

private:
  std::shared_ptr<ObjectStore> store_;
  std::shared_ptr<SpatialIndex> spatial_;
  std::shared_ptr<MilClassEngine> milclass_;
  CorrelationConfig config_;

  struct IncompatibilityTracker {
    int count = 0;
  };
  std::unordered_map<UUIDKey, IncompatibilityTracker> incompatibility_;

  double scoreCandidate(const Observation& obs, const UUIDKey& candidate) const;
  UUIDKey createNewEntity(const Observation& obs);
  void mergeIntoEntity(const Observation& obs, const UUIDKey& entity_id);
  UUIDKey splitEntity(const Observation& obs, const UUIDKey& source_id);
};

} // namespace tactical_objects
