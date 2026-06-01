#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame::neuro {

// A single indexed log record (one JSONL line from any of the four streams).
struct LogRecord {
    std::string stream;   // "bt", "wm", "plan", "neuro"
    uint64_t ts_us = 0;
    std::string entity;   // primary key (fluent name, episode_id, record_id, etc.)
    std::string raw;      // original JSON line
};

// TrainingSample: (state, goal, plan, cost, outcome) tuple from a plan episode.
struct TrainingSample {
    uint64_t episode_id = 0;
    std::vector<std::string> init_facts;
    std::vector<std::string> goal_fluents;
    std::vector<std::string> plan_actions;
    float cost = 0.0f;
    bool success = false;
};

// Shared retrieval substrate: load N JSONL streams, build time+entity indices,
// return bounded windows and training-data exports.
class AuditIndex {
public:
    // Load a JSONL file into the index. stream_name: "bt", "wm", "plan", or "neuro".
    void load(const std::string& filepath, const std::string& stream_name);

    // Load records from in-memory lines (for tests).
    void load_lines(const std::vector<std::string>& lines,
                    const std::string& stream_name);

    // Return all records in [t0_us, t1_us] inclusive.
    std::vector<LogRecord> window(uint64_t t0_us, uint64_t t1_us) const;

    // Return k records before and after the record whose entity matches key,
    // bounded by a maximum total byte size.
    std::vector<LogRecord> around(const std::string& entity_key,
                                   unsigned k,
                                   size_t max_bytes = 1024 * 64) const;

    // Extract training samples from plan audit records loaded via load()/load_lines().
    std::vector<TrainingSample> training_export() const;

    // Map a free-form reference back to record ids that mention it.
    std::vector<uint64_t> cite(const std::string& reference) const;

    size_t size() const { return records_.size(); }

private:
    std::vector<LogRecord> records_;
    // entity -> list of record indices
    std::unordered_map<std::string, std::vector<size_t>> entity_index_;

    void index_record(size_t idx);
    static LogRecord parse_line(const std::string& line, const std::string& stream);
};

} // namespace ame::neuro
