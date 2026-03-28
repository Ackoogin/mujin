#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace ame {

/// Audit trail for planning episodes.
///
/// Each episode records a complete snapshot of why the planner produced a
/// given plan: initial state, goals, solver, timing, plan steps, and the
/// compiled BT XML.  Written as JSONL for post-hoc analysis.
class PlanAuditLog {
public:
    struct Episode {
        uint64_t    episode_id = 0;               // Unique ID (auto-assigned by recordEpisode)
        uint64_t    parent_episode_id = 0;        // 0 = top-level; non-zero = sub-plan of parent
        std::string phase_name;                   // Human-readable phase label (hierarchical only)
        uint64_t    ts_us = 0;                    // When planning started
        std::vector<std::string> init_facts;      // Fluent keys that were true
        std::vector<std::string> goal_fluents;    // Goal fluent keys
        std::string solver;                       // Solver name (e.g. "BRFS")
        double      solve_time_ms = 0.0;          // Wall-clock solve duration
        bool        success = false;
        unsigned    expanded = 0;                 // Nodes expanded
        unsigned    generated = 0;                // Nodes generated
        float       cost = 0.0f;                  // Plan cost
        std::vector<std::string> plan_actions;    // Ordered action signatures
        std::string bt_xml;                       // Compiled BT XML
    };

    /// Construct without a file sink (in-memory only).
    PlanAuditLog() = default;

    /// Construct with a JSONL file sink.
    explicit PlanAuditLog(const std::string& filepath);

    ~PlanAuditLog();

    /// Record a planning episode.
    /// Assigns a unique episode_id before recording if ep.episode_id == 0.
    /// Returns the assigned episode_id.
    uint64_t recordEpisode(Episode ep);

    /// Flush the file sink.
    void flush();

    /// Allocate the next unique episode ID without recording an episode.
    uint64_t nextEpisodeId();

    /// All episodes recorded so far.
    const std::vector<Episode>& episodes() const { return episodes_; }

    /// Number of episodes.
    size_t size() const { return episodes_.size(); }

private:
    std::ofstream file_;
    std::vector<Episode> episodes_;
    uint64_t next_episode_id_ = 1;
};

} // namespace ame
