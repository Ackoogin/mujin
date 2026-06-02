#include "ame/neuro/neuro_audit_log.h"

#include <sstream>

namespace ame::neuro {

namespace {

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 4);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    // \uXXXX escape for remaining ASCII control bytes
                    static const char hex[] = "0123456789abcdef";
                    out += "\\u00";
                    out += hex[(static_cast<unsigned char>(c) >> 4) & 0xf];
                    out += hex[static_cast<unsigned char>(c) & 0xf];
                } else {
                    out += c;
                }
                break;
        }
    }
    return out;
}

std::string json_str_array(const std::vector<std::string>& arr) {
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < arr.size(); ++i) {
        if (i) os << ",";
        os << "\"" << json_escape(arr[i]) << "\"";
    }
    os << "]";
    return os.str();
}

std::string serialise(const NeuroAuditRecord& r) {
    std::ostringstream os;
    os << "{"
       << "\"record_id\":"     << r.record_id << ","
       << "\"ts_us\":"         << r.ts_us << ","
       << "\"integration_kind\":\"" << json_escape(r.integration_kind) << "\","
       << "\"backend_id\":\""  << json_escape(r.backend_id) << "\","
       << "\"model_id\":\""    << json_escape(r.model_id) << "\","
       << "\"request_digest\":\"" << json_escape(r.request_digest) << "\","
       << "\"proposal_digest\":\"" << json_escape(r.proposal_digest) << "\","
       << "\"outcome\":\""     << json_escape(r.outcome) << "\","
       << "\"verdict_reason\":\"" << json_escape(r.verdict_reason) << "\","
       << "\"evidence\":"      << json_str_array(r.evidence) << ","
       << "\"latency_ms\":"    << r.latency_ms << ","
       << "\"retries\":"       << r.retries << ","
       << "\"affected_behaviour\":" << (r.affected_behaviour ? "true" : "false");
    if (!r.raw_request.empty())
        os << ",\"raw_request\":\"" << json_escape(r.raw_request) << "\"";
    if (!r.raw_proposal.empty())
        os << ",\"raw_proposal\":\"" << json_escape(r.raw_proposal) << "\"";
    os << "}";
    return os.str();
}

} // namespace

NeuroAuditLog::NeuroAuditLog(const std::string& filepath) {
    file_.open(filepath, std::ios::out | std::ios::trunc);
}

NeuroAuditLog::~NeuroAuditLog() {
    flush();
}

uint64_t NeuroAuditLog::next_id() {
    std::lock_guard<std::mutex> lk(mx_);
    return next_id_++;
}

uint64_t NeuroAuditLog::append(NeuroAuditRecord rec) {
    std::lock_guard<std::mutex> lk(mx_);
    if (rec.record_id == 0) rec.record_id = next_id_++;
    uint64_t id = rec.record_id;
    records_.push_back(rec);
    if (file_.is_open()) {
        file_ << serialise(records_.back()) << '\n';
    }
    return id;
}

void NeuroAuditLog::flush() {
    std::lock_guard<std::mutex> lk(mx_);
    if (file_.is_open()) file_.flush();
}

} // namespace ame::neuro
