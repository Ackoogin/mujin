#include "ame/plan_audit_log.h"

#include <sstream>

namespace ame {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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

static std::string jsonStringArray(const std::vector<std::string>& arr) {
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < arr.size(); ++i) {
        if (i > 0) os << ",";
        os << "\"" << jsonEscape(arr[i]) << "\"";
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
    episodes_.push_back(std::move(ep));
    const auto& stored = episodes_.back();

    if (file_.is_open()) {
        std::ostringstream os;
        os << "{"
           << "\"episode_id\":" << stored.episode_id << ","
           << "\"parent_episode_id\":" << stored.parent_episode_id << ",";
        if (!stored.phase_name.empty()) {
            os << "\"phase_name\":\"" << jsonEscape(stored.phase_name) << "\",";
        }
        os << "\"ts_us\":" << stored.ts_us << ","
           << "\"solver\":\"" << jsonEscape(stored.solver) << "\","
           << "\"solve_time_ms\":" << stored.solve_time_ms << ","
           << "\"success\":" << (stored.success ? "true" : "false") << ","
           << "\"expanded\":" << stored.expanded << ","
           << "\"generated\":" << stored.generated << ","
           << "\"cost\":" << stored.cost << ","
           << "\"init_facts\":" << jsonStringArray(stored.init_facts) << ","
           << "\"goal_fluents\":" << jsonStringArray(stored.goal_fluents) << ","
           << "\"plan_actions\":" << jsonStringArray(stored.plan_actions) << ","
           << "\"bt_xml\":\"" << jsonEscape(stored.bt_xml) << "\""
           << "}";
        file_ << os.str() << '\n';
    }

    return id;
}

void PlanAuditLog::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

} // namespace ame
