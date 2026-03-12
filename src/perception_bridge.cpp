#include "ame/perception_bridge.h"

namespace ame {

PerceptionBridge::PerceptionBridge(WorldModel& wm) : wm_(wm) {}

void PerceptionBridge::updateFact(const std::string& fact, bool value,
                                   const std::string& source_tag) {
    std::string source = "perception";
    if (!source_tag.empty()) {
        source += ":" + source_tag;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    pending_.push_back({fact, value, std::move(source)});
}

unsigned PerceptionBridge::flush() {
    std::vector<PendingUpdate> batch;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        batch.swap(pending_);
    }

    for (auto& upd : batch) {
        wm_.setFact(upd.fact, upd.value, upd.source);
        if (callback_) {
            callback_(upd.fact, upd.value);
        }
    }

    return static_cast<unsigned>(batch.size());
}

void PerceptionBridge::setUpdateCallback(UpdateCallback cb) {
    callback_ = std::move(cb);
}

unsigned PerceptionBridge::pendingCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<unsigned>(pending_.size());
}

} // namespace ame
