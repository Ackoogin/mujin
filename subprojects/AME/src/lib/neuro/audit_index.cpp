#include "ame/neuro/audit_index.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace ame::neuro {

namespace {

// Minimal JSON value extractor: returns the string value of the first
// occurrence of "key":value in a JSON line. Handles strings and numbers.
// Returns empty string if key not found.
std::string json_get(const std::string& line, const std::string& key) {
    std::string search = "\"" + key + "\":";
    auto pos = line.find(search);
    if (pos == std::string::npos) return {};
    pos += search.size();
    // Skip whitespace
    while (pos < line.size() && line[pos] == ' ') ++pos;
    if (pos >= line.size()) return {};
    if (line[pos] == '"') {
        // String value
        ++pos;
        std::string val;
        while (pos < line.size() && line[pos] != '"') {
            if (line[pos] == '\\' && pos + 1 < line.size()) {
                ++pos;
                switch (line[pos]) {
                    case '"':  val += '"';  break;
                    case '\\': val += '\\'; break;
                    case 'n':  val += '\n'; break;
                    case 'r':  val += '\r'; break;
                    case 't':  val += '\t'; break;
                    default:   val += line[pos]; break;
                }
            } else {
                val += line[pos];
            }
            ++pos;
        }
        return val;
    } else if (line[pos] == '[' || line[pos] == '{') {
        // Array or object value — read the full balanced bracket span.
        char open  = line[pos];
        char close = (open == '[') ? ']' : '}';
        int depth = 0;
        std::string val;
        bool in_str = false;
        while (pos < line.size()) {
            char c = line[pos];
            if (in_str) {
                val += c;
                if (c == '\\' && pos + 1 < line.size()) {
                    val += line[++pos]; // consume escaped char
                } else if (c == '"') {
                    in_str = false;
                }
            } else {
                if (c == '"')       { in_str = true; val += c; }
                else if (c == open) { ++depth;       val += c; }
                else if (c == close){ --depth;       val += c; if (depth == 0) { ++pos; break; } }
                else                {                val += c; }
            }
            ++pos;
        }
        return val;
    } else {
        // Numeric or bool value — read until , } \n
        std::string val;
        while (pos < line.size() && line[pos] != ',' && line[pos] != '}' && line[pos] != '\n')
            val += line[pos++];
        return val;
    }
}

// Parse the primary entity from a log line based on stream type.
std::string extract_entity(const std::string& line, const std::string& stream) {
    if (stream == "plan") return json_get(line, "episode_id");
    if (stream == "neuro") return json_get(line, "record_id");
    if (stream == "wm")   return json_get(line, "fact_name");
    if (stream == "bt")   return json_get(line, "node_name");
    return {};
}

uint64_t extract_ts(const std::string& line) {
    std::string val = json_get(line, "ts_us");
    if (val.empty()) return 0;
    try { return std::stoull(val); } catch (...) { return 0; }
}

} // namespace

LogRecord AuditIndex::parse_line(const std::string& line, const std::string& stream) {
    LogRecord r;
    r.stream = stream;
    r.raw = line;
    r.ts_us = extract_ts(line);
    r.entity = extract_entity(line, stream);
    return r;
}

void AuditIndex::index_record(size_t idx) {
    const auto& r = records_[idx];
    if (!r.entity.empty())
        entity_index_[r.entity].push_back(idx);
    // Also index by stream
    entity_index_[r.stream].push_back(idx);
}

void AuditIndex::load_lines(const std::vector<std::string>& lines,
                              const std::string& stream_name) {
    for (const auto& line : lines) {
        if (line.empty() || line[0] != '{') continue;
        size_t idx = records_.size();
        records_.push_back(parse_line(line, stream_name));
        index_record(idx);
    }
}

void AuditIndex::load(const std::string& filepath, const std::string& stream_name) {
    std::ifstream f(filepath);
    if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line[0] == '{')
            load_lines({line}, stream_name);
    }
}

std::vector<LogRecord> AuditIndex::window(uint64_t t0_us, uint64_t t1_us) const {
    std::vector<LogRecord> out;
    for (const auto& r : records_) {
        if (r.ts_us >= t0_us && r.ts_us <= t1_us)
            out.push_back(r);
    }
    return out;
}

std::vector<LogRecord> AuditIndex::around(const std::string& entity_key,
                                            unsigned k,
                                            size_t max_bytes) const {
    auto it = entity_index_.find(entity_key);
    if (it == entity_index_.end()) return {};

    // Find the first matching index
    if (it->second.empty()) return {};
    size_t centre = it->second.front();

    size_t lo = (centre >= k) ? (centre - k) : 0;
    size_t hi = std::min(records_.size() - 1, centre + k);

    std::vector<LogRecord> out;
    size_t total_bytes = 0;
    for (size_t i = lo; i <= hi; ++i) {
        total_bytes += records_[i].raw.size();
        if (total_bytes > max_bytes && !out.empty()) break;
        out.push_back(records_[i]);
    }
    return out;
}

std::vector<TrainingSample> AuditIndex::training_export() const {
    std::vector<TrainingSample> out;
    for (const auto& r : records_) {
        if (r.stream != "plan") continue;
        // Parse fields from the raw plan audit JSON line.
        std::string ep_id_str = json_get(r.raw, "episode_id");
        std::string success_str = json_get(r.raw, "success");
        std::string cost_str = json_get(r.raw, "cost");
        if (ep_id_str.empty()) continue;

        TrainingSample s;
        try { s.episode_id = std::stoull(ep_id_str); } catch (...) { continue; }
        s.success = (success_str == "true");
        try { s.cost = std::stof(cost_str); } catch (...) {}

        auto parse_json_str_array = [](const std::string& arr, std::vector<std::string>& out) {
            // Parses a JSON string array: ["a","b","c"] → {"a","b","c"}
            std::size_t pos = 0;
            while (pos < arr.size() && arr[pos] != '"') ++pos;
            while (pos < arr.size()) {
                if (arr[pos] != '"') { ++pos; continue; }
                ++pos; // skip opening quote
                std::string elem;
                while (pos < arr.size() && arr[pos] != '"') {
                    if (arr[pos] == '\\' && pos + 1 < arr.size()) {
                        ++pos; // skip backslash, take next char literally
                    }
                    elem += arr[pos++];
                }
                if (pos < arr.size()) ++pos; // skip closing quote
                out.push_back(std::move(elem));
                // advance past comma/whitespace to next element or closing bracket
                while (pos < arr.size() && arr[pos] != '"' && arr[pos] != ']') ++pos;
            }
        };

        std::string init = json_get(r.raw, "init_facts");
        if (!init.empty()) parse_json_str_array(init, s.init_facts);
        std::string goal = json_get(r.raw, "goal_fluents");
        if (!goal.empty()) parse_json_str_array(goal, s.goal_fluents);
        std::string plan = json_get(r.raw, "plan_actions");
        if (!plan.empty()) parse_json_str_array(plan, s.plan_actions);

        out.push_back(std::move(s));
    }
    return out;
}

std::vector<uint64_t> AuditIndex::cite(const std::string& reference) const {
    std::vector<uint64_t> ids;
    for (const auto& r : records_) {
        if (r.raw.find(reference) != std::string::npos) {
            // Try to extract record_id or episode_id
            std::string rid = json_get(r.raw, "record_id");
            if (rid.empty()) rid = json_get(r.raw, "episode_id");
            try {
                if (!rid.empty()) ids.push_back(std::stoull(rid));
            } catch (...) {}
        }
    }
    return ids;
}

} // namespace ame::neuro
