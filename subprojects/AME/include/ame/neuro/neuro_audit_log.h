#pragma once

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace ame::neuro {

// One record per Advisor::advise() call (Layer 6 observability).
struct NeuroAuditRecord {
    uint64_t    record_id = 0;
    uint64_t    ts_us = 0;                  // wall-clock at start of call (us)
    std::string integration_kind;           // e.g. "goal_interpreter"
    std::string backend_id;
    std::string model_id;
    std::string request_digest;
    std::string proposal_digest;
    std::string outcome;                    // "Accepted", "RejectedFellBack", etc.
    std::string verdict_reason;
    std::vector<std::string> evidence;
    double latency_ms = 0.0;
    unsigned retries = 0;
    bool affected_behaviour = false;        // true iff outcome == Accepted
    std::string raw_request;               // set if verbosity >= 1
    std::string raw_proposal;
};

// Append-only neuro audit trail.
// Mirrors PlanAuditLog: in-memory + optional JSONL sink (ame_neuro_events.jsonl).
class NeuroAuditLog {
public:
    NeuroAuditLog() = default;
    explicit NeuroAuditLog(const std::string& filepath);
    ~NeuroAuditLog();

    // Append a record; assigns record_id if zero. Thread-safe.
    uint64_t append(NeuroAuditRecord rec);

    void flush();
    uint64_t next_id();

    const std::vector<NeuroAuditRecord>& records() const { return records_; }
    size_t size() const { return records_.size(); }

private:
    std::ofstream file_;
    std::vector<NeuroAuditRecord> records_;
    uint64_t next_id_ = 1;
    mutable std::mutex mx_;
};

} // namespace ame::neuro
