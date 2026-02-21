#include "mujin/wm_audit_log.h"

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

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

WmAuditLog::WmAuditLog(const std::string& filepath) {
    file_.open(filepath, std::ios::out | std::ios::trunc);
}

WmAuditLog::~WmAuditLog() {
    flush();
}

// ---------------------------------------------------------------------------
// Core callback
// ---------------------------------------------------------------------------

void WmAuditLog::onFactChange(uint64_t wm_version,
                               uint64_t ts_us,
                               const std::string& fact,
                               bool value,
                               const std::string& source) {
    entries_.push_back({wm_version, ts_us, fact, value, source});

    if (file_.is_open()) {
        std::ostringstream os;
        os << "{"
           << "\"wm_version\":" << wm_version << ","
           << "\"ts_us\":" << ts_us << ","
           << "\"fact\":\"" << jsonEscape(fact) << "\","
           << "\"value\":" << (value ? "true" : "false") << ","
           << "\"source\":\"" << jsonEscape(source) << "\""
           << "}";
        file_ << os.str() << '\n';
    }
}

void WmAuditLog::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

} // namespace mujin
