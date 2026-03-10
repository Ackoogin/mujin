#pragma once

#include "mujin/world_model.h"

#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace mujin {

// PerceptionBridge: thread-safe adapter for injecting sensor observations
// into the WorldModel.
//
// External perception systems (computer vision, sensors, ROS2 topics) call
// updateFact() to push state changes tagged with source "perception".
// Updates are buffered and flushed atomically between BT ticks to prevent
// torn reads during tree execution.
//
// Usage:
//   PerceptionBridge bridge(world_model);
//   // From a sensor thread:
//   bridge.updateFact("(at uav1 sector_a)", true);
//   // From the BT tick thread (between ticks):
//   bridge.flush();
class PerceptionBridge {
public:
    // Callback fired after each fact is applied to the world model.
    // Parameters: (fact_name, value)
    using UpdateCallback = std::function<void(const std::string&, bool)>;

    explicit PerceptionBridge(WorldModel& wm);

    // Thread-safe: buffer a perception fact update.
    // source_tag is appended to "perception:" in the audit log, e.g. "perception:camera_front".
    void updateFact(const std::string& fact, bool value,
                    const std::string& source_tag = "");

    // Apply all buffered updates to the WorldModel.
    // Call this between BT ticks (on the BT tick thread) to maintain consistency.
    // Returns the number of facts applied.
    unsigned flush();

    // Register a callback invoked for each fact applied during flush().
    void setUpdateCallback(UpdateCallback cb);

    // Returns the number of pending (unflushed) updates.
    unsigned pendingCount() const;

private:
    struct PendingUpdate {
        std::string fact;
        bool value;
        std::string source;
    };

    WorldModel& wm_;
    std::vector<PendingUpdate> pending_;
    mutable std::mutex mutex_;
    UpdateCallback callback_;
};

} // namespace mujin
