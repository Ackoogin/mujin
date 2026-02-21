#include "mujin/bt_logger.h"
#include "mujin/world_model.h"

#include <chrono>
#include <sstream>

namespace mujin {

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

/// Minimal JSON-string escaping (backslash + double-quote).
static std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 4);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;      break;
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MujinBTLogger::MujinBTLogger(const BT::Tree& tree,
                             const std::string& tree_id,
                             const WorldModel* wm)
    : BT::StatusChangeLogger(tree.rootNode())
    , tree_id_(tree_id)
    , wm_(wm)
{}

MujinBTLogger::~MujinBTLogger() {
    flush();
}

// ---------------------------------------------------------------------------
// Sink configuration
// ---------------------------------------------------------------------------

void MujinBTLogger::addFileSink(const std::string& path) {
    file_.open(path, std::ios::out | std::ios::trunc);
}

void MujinBTLogger::addCallbackSink(SinkCallback cb) {
    callbacks_.push_back(std::move(cb));
}

// ---------------------------------------------------------------------------
// StatusChangeLogger interface
// ---------------------------------------------------------------------------

void MujinBTLogger::callback(BT::Duration timestamp,
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
       << "\"node\":\"" << jsonEscape(node.name()) << "\","
       << "\"type\":\"" << jsonEscape(node.registrationName()) << "\","
       << "\"prev\":\"" << statusStr(prev_status) << "\","
       << "\"status\":\"" << statusStr(status) << "\","
       << "\"tree_id\":\"" << jsonEscape(tree_id_) << "\","
       << "\"wm_version\":" << wm_ver
       << "}";

    std::string line = os.str();

    // Store in memory
    events_.push_back(line);

    // Dispatch to sinks
    if (file_.is_open()) {
        file_ << line << '\n';
    }
    for (auto& cb : callbacks_) {
        cb(line);
    }
}

void MujinBTLogger::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

} // namespace mujin
