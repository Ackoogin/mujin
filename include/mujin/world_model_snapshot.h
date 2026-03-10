#pragma once

#include "mujin/world_model.h"

#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace mujin {

// WorldModelSnapshot: immutable point-in-time copy of WorldModel state.
//
// Taken atomically with respect to perception writes (via SnapshotManager).
// BT ticks read from a stable snapshot; perception threads write to the
// live WorldModel and a new snapshot is swapped in between ticks.
struct WorldModelSnapshot {
    uint64_t version = 0;
    std::vector<uint64_t> state_bits;
    std::vector<std::string> fluent_names;
    std::unordered_map<std::string, unsigned> fluent_index;

    // Query a fact by string key.  Returns false for unknown keys.
    bool getFact(const std::string& key) const {
        auto it = fluent_index.find(key);
        if (it == fluent_index.end()) return false;
        return getFact(it->second);
    }

    // Query a fact by fluent index.
    bool getFact(unsigned id) const {
        unsigned word = id / 64;
        unsigned bit  = id % 64;
        if (word >= state_bits.size()) return false;
        return (state_bits[word] >> bit) & 1u;
    }

    unsigned numFluents() const {
        return static_cast<unsigned>(fluent_names.size());
    }
};

// SnapshotManager: orchestrates consistent snapshot hand-off between the
// perception (writer) thread and the BT (reader) thread.
//
// Usage pattern:
//
//   // Perception thread:
//   bridge.updateFact("(at uav1 sector_a)", true);   // buffered
//   bridge.flush();                                    // writes to WorldModel
//   manager.publish();                                 // snapshot for next tick
//
//   // BT tick thread:
//   auto snap = manager.current();  // grab latest consistent snapshot
//   bool v = snap->getFact("(at uav1 sector_a)");
//
// Note: SnapshotManager does NOT add locks around WorldModel::setFact().
// It relies on PerceptionBridge::flush() being called from a single writer
// thread, or an external mutex when multiple writers exist.
class SnapshotManager {
public:
    explicit SnapshotManager(const WorldModel& wm);

    // Build a new snapshot from the current WorldModel state and make it
    // the current one.  Safe to call from any thread.
    void publish();

    // Return the most-recently published snapshot.
    // The returned shared_ptr keeps the snapshot alive even if publish() is
    // called concurrently.
    std::shared_ptr<const WorldModelSnapshot> current() const;

private:
    const WorldModel& wm_;
    mutable std::shared_mutex mutex_;
    std::shared_ptr<const WorldModelSnapshot> current_;
};

} // namespace mujin
