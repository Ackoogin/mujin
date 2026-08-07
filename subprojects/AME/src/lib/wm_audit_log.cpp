#include "ame/wm_audit_log.h"
#include "ame/detail/escape.h"

#include <sstream>

namespace ame {

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
    Entry entry{wm_version, ts_us, fact, value, source};

    if (file_.is_open()) {
        std::ostringstream os;
        os << "{"
           << "\"wm_version\":" << wm_version << ","
           << "\"ts_us\":" << ts_us << ","
           << "\"fact\":\"" << detail::jsonEscape(fact) << "\","
           << "\"value\":" << (value ? "true" : "false") << ","
           << "\"source\":\"" << detail::jsonEscape(source) << "\""
           << "}";
        file_ << os.str() << '\n';
    }

    entries_.push_back(std::move(entry));
    while (entries_.size() > max_retained_entries_) {
        entries_.erase(entries_.begin());
    }
}

void WmAuditLog::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

void WmAuditLog::setMaxRetainedEntries(std::size_t max_entries) {
    max_retained_entries_ = max_entries;
    while (entries_.size() > max_retained_entries_) {
        entries_.erase(entries_.begin());
    }
}

} // namespace ame
