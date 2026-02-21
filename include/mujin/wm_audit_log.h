#pragma once

#include <cstdint>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace mujin {

/// Structured audit log for WorldModel fact changes.
///
/// Each entry records:
/// {
///   "wm_version": <uint64>,
///   "ts_us":      <epoch microseconds>,
///   "fact":       "<predicate key>",
///   "value":      <true|false>,
///   "source":     "<originator>"
/// }
///
/// Sources follow a convention:
///   - "SetWorldPredicate:<node_name>"  — BT effect node
///   - "perception"                     — external update
///   - "planner_init"                   — initial state sync
///   - ""                               — untagged (legacy callers)
class WmAuditLog {
public:
    struct Entry {
        uint64_t    wm_version;
        uint64_t    ts_us;
        std::string fact;
        bool        value;
        std::string source;
    };

    /// Construct without a file sink (in-memory only).
    WmAuditLog() = default;

    /// Construct with a JSONL file sink.
    explicit WmAuditLog(const std::string& filepath);

    ~WmAuditLog();

    /// Callback compatible with WorldModel::setAuditCallback().
    void onFactChange(uint64_t wm_version,
                      uint64_t ts_us,
                      const std::string& fact,
                      bool value,
                      const std::string& source);

    /// Flush the file sink.
    void flush();

    /// All entries recorded so far (useful for testing / queries).
    const std::vector<Entry>& entries() const { return entries_; }

    /// Number of entries.
    size_t size() const { return entries_.size(); }

private:
    std::ofstream file_;
    std::vector<Entry> entries_;
};

} // namespace mujin
