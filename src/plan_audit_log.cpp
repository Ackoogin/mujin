#include "mujin/plan_audit_log.h"

#include <sstream>

namespace mujin {

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

void PlanAuditLog::recordEpisode(const Episode& ep) {
    episodes_.push_back(ep);

    if (file_.is_open()) {
        std::ostringstream os;
        os << "{"
           << "\"ts_us\":" << ep.ts_us << ","
           << "\"solver\":\"" << jsonEscape(ep.solver) << "\","
           << "\"solve_time_ms\":" << ep.solve_time_ms << ","
           << "\"success\":" << (ep.success ? "true" : "false") << ","
           << "\"expanded\":" << ep.expanded << ","
           << "\"generated\":" << ep.generated << ","
           << "\"cost\":" << ep.cost << ","
           << "\"init_facts\":" << jsonStringArray(ep.init_facts) << ","
           << "\"goal_fluents\":" << jsonStringArray(ep.goal_fluents) << ","
           << "\"plan_actions\":" << jsonStringArray(ep.plan_actions) << ","
           << "\"bt_xml\":\"" << jsonEscape(ep.bt_xml) << "\""
           << "}";
        file_ << os.str() << '\n';
    }
}

void PlanAuditLog::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

} // namespace mujin
