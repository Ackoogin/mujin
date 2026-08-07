#include "ame/plan_audit_log.h"
#include "ame/detail/escape.h"

#include <sstream>

namespace ame {

static std::string jsonStringArray(const std::vector<std::string>& arr) {
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < arr.size(); ++i) {
        if (i > 0) os << ",";
        os << "\"" << detail::jsonEscape(arr[i]) << "\"";
    }
    os << "]";
    return os.str();
}

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

PlanAuditLog::PlanAuditLog(const std::string& filepath) {
    file_.open(filepath, std::ios::out | std::ios::trunc);
}

PlanAuditLog::~PlanAuditLog() {
    flush();
}

// ---------------------------------------------------------------------------
// Core
// ---------------------------------------------------------------------------

uint64_t PlanAuditLog::nextEpisodeId() {
    return next_episode_id_++;
}

uint64_t PlanAuditLog::recordEpisode(Episode ep) {
    if (ep.episode_id == 0) {
        ep.episode_id = nextEpisodeId();
    }

    uint64_t id = ep.episode_id;

    if (file_.is_open()) {
        std::ostringstream os;
        os << "{"
           << "\"episode_id\":" << ep.episode_id << ","
           << "\"parent_episode_id\":" << ep.parent_episode_id << ",";
        if (!ep.phase_name.empty()) {
            os << "\"phase_name\":\"" << detail::jsonEscape(ep.phase_name) << "\",";
        }
        if (!ep.session_id.empty()) {
            os << "\"session_id\":\"" << detail::jsonEscape(ep.session_id) << "\",";
        }
        os << "\"ts_us\":" << ep.ts_us << ","
           << "\"solver\":\"" << detail::jsonEscape(ep.solver) << "\","
           << "\"solve_time_ms\":" << ep.solve_time_ms << ","
           << "\"success\":" << (ep.success ? "true" : "false") << ","
           << "\"expanded\":" << ep.expanded << ","
           << "\"generated\":" << ep.generated << ","
           << "\"cost\":" << ep.cost << ","
           << "\"init_facts\":" << jsonStringArray(ep.init_facts) << ","
           << "\"goal_fluents\":" << jsonStringArray(ep.goal_fluents) << ","
           << "\"plan_actions\":" << jsonStringArray(ep.plan_actions) << ","
           << "\"bt_xml\":\"" << detail::jsonEscape(ep.bt_xml) << "\","
           << "\"heuristic_source\":\"" << detail::jsonEscape(ep.heuristic_source) << "\","
           << "\"goal_source\":\"" << detail::jsonEscape(ep.goal_source) << "\","
           << "\"repair_source\":\"" << detail::jsonEscape(ep.repair_source) << "\"";
        if (!ep.neuro_record_ids.empty()) {
            os << ",\"neuro_record_ids\":[";
            for (size_t i = 0; i < ep.neuro_record_ids.size(); ++i) {
                if (i) os << ",";
                os << ep.neuro_record_ids[i];
            }
            os << "]";
        }
        os << "}";
        file_ << os.str() << '\n';
    }

    episodes_.push_back(std::move(ep));
    while (episodes_.size() > max_retained_episodes_) {
        episodes_.erase(episodes_.begin());
    }

    return id;
}

void PlanAuditLog::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

void PlanAuditLog::setMaxRetainedEpisodes(std::size_t max_episodes) {
    max_retained_episodes_ = max_episodes;
    while (episodes_.size() > max_retained_episodes_) {
        episodes_.erase(episodes_.begin());
    }
}

} // namespace ame
