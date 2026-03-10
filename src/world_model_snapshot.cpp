#include "mujin/world_model_snapshot.h"

namespace mujin {

SnapshotManager::SnapshotManager(const WorldModel& wm) : wm_(wm) {
    // Publish an initial snapshot so current() is never null.
    publish();
}

void SnapshotManager::publish() {
    auto snap = std::make_shared<WorldModelSnapshot>();
    snap->version     = wm_.version();
    snap->fluent_names = std::vector<std::string>();
    snap->fluent_index = std::unordered_map<std::string, unsigned>();

    // Copy fluent metadata and bit state from the live WorldModel.
    const unsigned n = wm_.numFluents();
    snap->fluent_names.reserve(n);
    snap->fluent_index.reserve(n);

    // Build state_bits big enough for n fluents.
    const unsigned words = (n == 0) ? 0 : (n - 1) / 64 + 1;
    snap->state_bits.resize(words, 0);

    for (unsigned i = 0; i < n; ++i) {
        const std::string& name = wm_.fluentName(i);
        snap->fluent_names.push_back(name);
        snap->fluent_index[name] = i;
        if (wm_.getFact(i)) {
            unsigned word = i / 64;
            unsigned bit  = i % 64;
            snap->state_bits[word] |= (uint64_t(1) << bit);
        }
    }

    std::unique_lock<std::shared_mutex> lk(mutex_);
    current_ = std::move(snap);
}

std::shared_ptr<const WorldModelSnapshot> SnapshotManager::current() const {
    std::shared_lock<std::shared_mutex> lk(mutex_);
    return current_;
}

} // namespace mujin
