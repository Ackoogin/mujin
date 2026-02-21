#pragma once

#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace mujin {

class WorldModel;

/// Structured JSON event logger for BT.CPP node state transitions.
///
/// Emits one JSONL record per node status change:
/// {
///   "ts_us": <epoch microseconds>,
///   "node":  "<node name>",
///   "type":  "<registration name>",
///   "prev":  "<IDLE|RUNNING|SUCCESS|FAILURE>",
///   "status":"<IDLE|RUNNING|SUCCESS|FAILURE>",
///   "tree_id":"<tree identifier>",
///   "wm_version": <uint64>
/// }
///
/// Sinks are configurable: file (JSONL), or user-supplied callback.
class MujinBTLogger : public BT::StatusChangeLogger {
public:
    /// User-defined sink: receives the formatted JSON string for each event.
    using SinkCallback = std::function<void(const std::string& json_line)>;

    /// Construct the logger and attach to a tree.
    /// @param tree       The BT to observe.
    /// @param tree_id    Logical name for this tree (appears in every event).
    /// @param wm         Optional WorldModel pointer for version tracking.
    MujinBTLogger(const BT::Tree& tree,
                  const std::string& tree_id = "MissionPlan",
                  const WorldModel* wm = nullptr);

    ~MujinBTLogger() override;

    // -- Sink configuration --------------------------------------------------

    /// Open a JSONL file sink.  Each event is appended as one line.
    void addFileSink(const std::string& path);

    /// Register an arbitrary callback sink.
    void addCallbackSink(SinkCallback cb);

    // -- StatusChangeLogger interface ----------------------------------------

    void callback(BT::Duration timestamp,
                  const BT::TreeNode& node,
                  BT::NodeStatus prev_status,
                  BT::NodeStatus status) override;

    void flush() override;

    // -- Query ---------------------------------------------------------------

    /// All JSON lines emitted so far (useful for testing / in-memory query).
    const std::vector<std::string>& events() const { return events_; }

private:
    std::string tree_id_;
    const WorldModel* wm_ = nullptr;

    std::ofstream file_;
    std::vector<SinkCallback> callbacks_;
    std::vector<std::string> events_;
};

} // namespace mujin
