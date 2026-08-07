#include "ame/bt_logger.h"
#include "ame/detail/escape.h"
#include "ame/world_model.h"

#include <chrono>
#include <sstream>

namespace ame {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const char* statusStr(BT::NodeStatus s) {
    switch (s) {
        case BT::NodeStatus::IDLE:    return "IDLE";
        case BT::NodeStatus::RUNNING: return "RUNNING";
        case BT::NodeStatus::SUCCESS: return "SUCCESS";
        case BT::NodeStatus::FAILURE: return "FAILURE";
        default:                      return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

AmeBTLogger::AmeBTLogger(const BT::Tree& tree,
                             const std::string& tree_id,
                             const WorldModel* wm)
    : BT::StatusChangeLogger(tree.rootNode())
    , tree_id_(tree_id)
    , wm_(wm)
{}

AmeBTLogger::~AmeBTLogger() {
    flush();
}

// ---------------------------------------------------------------------------
// Sink configuration
// ---------------------------------------------------------------------------

void AmeBTLogger::addFileSink(const std::string& path) {
    file_.open(path, std::ios::out | std::ios::trunc);
}

void AmeBTLogger::addCallbackSink(SinkCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void AmeBTLogger::setMaxRetainedEvents(std::size_t max_events) {
    max_retained_events_ = max_events;
    while (events_.size() > max_retained_events_) {
        events_.erase(events_.begin());
    }
}

// ---------------------------------------------------------------------------
// StatusChangeLogger interface
// ---------------------------------------------------------------------------

void AmeBTLogger::callback(BT::Duration timestamp,
                             const BT::TreeNode& node,
                             BT::NodeStatus prev_status,
                             BT::NodeStatus status) {
    // Compute wall-clock microseconds
    auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    uint64_t wm_ver = wm_ ? wm_->version() : 0;

    // Build JSON object (hand-rolled to avoid external JSON dependency)
    std::ostringstream os;
    os << "{"
       << "\"ts_us\":" << wall_us << ","
       << "\"node\":\"" << detail::jsonEscape(node.name()) << "\","
       << "\"type\":\"" << detail::jsonEscape(node.registrationName()) << "\","
       << "\"prev\":\"" << statusStr(prev_status) << "\","
       << "\"status\":\"" << statusStr(status) << "\","
       << "\"tree_id\":\"" << detail::jsonEscape(tree_id_) << "\","
       << "\"wm_version\":" << wm_ver
       << "}";

    std::string line = os.str();

    // Dispatch to sinks
    if (file_.is_open()) {
        file_ << line << '\n';
    }
    for (auto& cb : callbacks_) {
        cb(line);
    }

    // Store in memory
    events_.push_back(std::move(line));
    while (events_.size() > max_retained_events_) {
        events_.erase(events_.begin());
    }
}

void AmeBTLogger::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

} // namespace ame
