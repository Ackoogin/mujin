#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectComponents.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace tactical_objects {

struct HistoricalSnapshot {
  UUIDKey object_id;
  double timestamp = 0.0;
  uint64_t version = 0;
  Position position;
  Affiliation affiliation = Affiliation::Unknown;
  double confidence = 0.0;
  std::string operational_state;
};

/// \brief Retains timestamped snapshots for as-of and interval queries.
class TacticalHistory {
public:
  /// \brief Record a snapshot at the given time.
  void recordSnapshot(const HistoricalSnapshot& snapshot);

  /// \brief Retrieve the snapshot closest to (but not after) the given time.
  tl::optional<HistoricalSnapshot> queryAtTime(const UUIDKey& object_id,
                                               double timestamp) const;

  /// \brief Retrieve all snapshots within a time interval [start, end].
  std::vector<HistoricalSnapshot> queryInterval(const UUIDKey& object_id,
                                                double start, double end) const;

  /// \brief Total snapshot count across all objects.
  size_t totalSnapshots() const;

private:
  std::unordered_map<UUIDKey, std::vector<HistoricalSnapshot>> history_;
};

} // namespace tactical_objects
