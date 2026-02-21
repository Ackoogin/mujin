#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace mujin {

/// Audit trail for planning episodes.
///
/// Each episode records a complete snapshot of why the planner produced a
/// given plan: initial state, goals, solver, timing, plan steps, and the
/// compiled BT XML.  Written as JSONL for post-hoc analysis.
class PlanAuditLog {
public:
    struct Episode {
        uint64_t    ts_us;                        // When planning started
        std::vector<std::string> init_facts;      // Fluent keys that were true
        std::vector<std::string> goal_fluents;    // Goal fluent keys
        std::string solver;                       // Solver name (e.g. "BRFS")
        double      solve_time_ms;                // Wall-clock solve duration
        bool        success;
        unsigned    expanded;                     // Nodes expanded
        unsigned    generated;                    // Nodes generated
        float       cost;                         // Plan cost
        std::vector<std::string> plan_actions;    // Ordered action signatures
        std::string bt_xml;                       // Compiled BT XML
    };

    /// Construct without a file sink (in-memory only).
    PlanAuditLog() = default;

    /// Construct with a JSONL file sink.
    explicit PlanAuditLog(const std::string& filepath);

    ~PlanAuditLog();

    /// Record a planning episode.
    void recordEpisode(const Episode& ep);

    /// Flush the file sink.
    void flush();

    /// All episodes recorded so far.
    const std::vector<Episode>& episodes() const { return episodes_; }

    /// Number of episodes.
    size_t size() const { return episodes_.size(); }

private:
    std::ofstream file_;
    std::vector<Episode> episodes_;
};

} // namespace mujin
