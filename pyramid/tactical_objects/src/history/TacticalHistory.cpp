#include <history/TacticalHistory.h>

#include <algorithm>

namespace tactical_objects {

void TacticalHistory::recordSnapshot(const HistoricalSnapshot& snapshot) {
  auto& vec = history_[snapshot.object_id];
  vec.push_back(snapshot);
}

tl::optional<HistoricalSnapshot> TacticalHistory::queryAtTime(
    const UUIDKey& object_id, double timestamp) const {
  auto it = history_.find(object_id);
  if (it == history_.end() || it->second.empty()) return tl::nullopt;

  const HistoricalSnapshot* best = nullptr;
  for (auto& snap : it->second) {
    if (snap.timestamp <= timestamp) {
      if (!best || snap.timestamp > best->timestamp) {
        best = &snap;
      }
    }
  }

  if (best) return *best;
  return tl::nullopt;
}

std::vector<HistoricalSnapshot> TacticalHistory::queryInterval(
    const UUIDKey& object_id, double start, double end) const {
  std::vector<HistoricalSnapshot> result;
  auto it = history_.find(object_id);
  if (it == history_.end()) return result;

  for (auto& snap : it->second) {
    if (snap.timestamp >= start && snap.timestamp <= end) {
      result.push_back(snap);
    }
  }

  std::sort(result.begin(), result.end(),
            [](const HistoricalSnapshot& a, const HistoricalSnapshot& b) {
              return a.timestamp < b.timestamp;
            });
  return result;
}

size_t TacticalHistory::totalSnapshots() const {
  size_t total = 0;
  for (auto& pair : history_) {
    total += pair.second.size();
  }
  return total;
}

} // namespace tactical_objects
