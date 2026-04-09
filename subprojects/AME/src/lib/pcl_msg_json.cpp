#include <ame/pcl_msg_json.h>

#include <sstream>
#include <cstdlib>

namespace ame {

// ---------------------------------------------------------------------------
// Internal JSON utilities
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
    std::string out = "[";
    for (size_t i = 0; i < arr.size(); ++i) {
        if (i > 0) out += ',';
        out += '"';
        out += jsonEscape(arr[i]);
        out += '"';
    }
    out += ']';
    return out;
}

// ---------------------------------------------------------------------------
// Minimal JSON parsing helpers
// These work on deterministic JSON produced by the pack functions above.
// ---------------------------------------------------------------------------

/// Find the raw value substring for a key in a flat JSON object.
/// Returns the raw token (unquoted for strings, literal for others).
static std::string jsonFindRaw(const std::string& json, const std::string& key) {
    // Search for "key": (with possible whitespace)
    auto pattern = "\"" + key + "\":";
    auto pos = json.find(pattern);
    if (pos == std::string::npos) {
        return {};
    }
    pos += pattern.size();
    // Skip whitespace
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) {
        ++pos;
    }
    if (pos >= json.size()) {
        return {};
    }
    // Determine end of value
    char first = json[pos];
    if (first == '"') {
        // String value: scan to closing quote with escape handling
        ++pos;
        std::string val;
        while (pos < json.size() && json[pos] != '"') {
            if (json[pos] == '\\' && pos + 1 < json.size()) {
                ++pos;
                switch (json[pos]) {
                    case '"':  val += '"';  break;
                    case '\\': val += '\\'; break;
                    case 'n':  val += '\n'; break;
                    case 'r':  val += '\r'; break;
                    case 't':  val += '\t'; break;
                    default:   val += json[pos]; break;
                }
            } else {
                val += json[pos];
            }
            ++pos;
        }
        return val;
    }
    // Non-string: read until comma, brace, bracket, or end
    size_t start = pos;
    while (pos < json.size() &&
           json[pos] != ',' && json[pos] != '}' &&
           json[pos] != ']' && json[pos] != ' ') {
        ++pos;
    }
    return json.substr(start, pos - start);
}

/// Parse a bool field.
static bool jsonGetBool(const std::string& json, const std::string& key,
                        bool default_val = false) {
    auto raw = jsonFindRaw(json, key);
    if (raw == "true")  return true;
    if (raw == "false") return false;
    return default_val;
}

/// Parse a uint64_t field.
static uint64_t jsonGetU64(const std::string& json, const std::string& key,
                           uint64_t default_val = 0) {
    auto raw = jsonFindRaw(json, key);
    if (raw.empty()) return default_val;
    return static_cast<uint64_t>(std::strtoull(raw.c_str(), nullptr, 10));
}

/// Parse a uint32_t field.
static uint32_t jsonGetU32(const std::string& json, const std::string& key,
                           uint32_t default_val = 0) {
    auto raw = jsonFindRaw(json, key);
    if (raw.empty()) return default_val;
    return static_cast<uint32_t>(std::strtoul(raw.c_str(), nullptr, 10));
}

/// Parse a double field.
static double jsonGetF64(const std::string& json, const std::string& key,
                         double default_val = 0.0) {
    auto raw = jsonFindRaw(json, key);
    if (raw.empty()) return default_val;
    return std::strtod(raw.c_str(), nullptr);
}

/// Parse a string field.
static std::string jsonGetStr(const std::string& json, const std::string& key,
                               const std::string& default_val = {}) {
    auto pattern = "\"" + key + "\":";
    auto pos = json.find(pattern);
    if (pos == std::string::npos) return default_val;
    pos += pattern.size();
    while (pos < json.size() && json[pos] == ' ') ++pos;
    if (pos >= json.size() || json[pos] != '"') return default_val;
    return jsonFindRaw(json, key);
}

/// Extract the raw JSON text for an array value: "key":[...].
/// Returns "[]" if not found.
static std::string jsonGetArrayRaw(const std::string& json, const std::string& key) {
    auto pattern = "\"" + key + "\":[";
    auto pos = json.find(pattern);
    if (pos == std::string::npos) return "[]";
    pos += pattern.size() - 1;  // points to '['
    size_t start = pos;
    int depth = 0;
    bool in_string = false;
    while (pos < json.size()) {
        char c = json[pos];
        if (in_string) {
            if (c == '\\') { ++pos; }
            else if (c == '"') { in_string = false; }
        } else {
            if (c == '"') { in_string = true; }
            else if (c == '[') { ++depth; }
            else if (c == ']') { --depth; if (depth == 0) { ++pos; break; } }
        }
        ++pos;
    }
    return json.substr(start, pos - start);
}

/// Parse a JSON string array ["a","b","c"] into a vector.
static std::vector<std::string> jsonParseStringArray(const std::string& arr) {
    std::vector<std::string> result;
    size_t pos = 0;
    while (pos < arr.size()) {
        pos = arr.find('"', pos);
        if (pos == std::string::npos) break;
        ++pos;
        std::string s;
        while (pos < arr.size() && arr[pos] != '"') {
            if (arr[pos] == '\\' && pos + 1 < arr.size()) {
                ++pos;
                switch (arr[pos]) {
                    case '"':  s += '"';  break;
                    case '\\': s += '\\'; break;
                    case 'n':  s += '\n'; break;
                    case 'r':  s += '\r'; break;
                    case 't':  s += '\t'; break;
                    default:   s += arr[pos]; break;
                }
            } else {
                s += arr[pos];
            }
            ++pos;
        }
        result.push_back(std::move(s));
        ++pos;
    }
    return result;
}

/// Split a JSON object array "[{...},{...}]" into individual object strings.
static std::vector<std::string> jsonParseObjectArray(const std::string& arr) {
    std::vector<std::string> result;
    size_t pos = 0;
    while (pos < arr.size()) {
        pos = arr.find('{', pos);
        if (pos == std::string::npos) break;
        size_t start = pos;
        int depth = 0;
        bool in_string = false;
        while (pos < arr.size()) {
            char c = arr[pos];
            if (in_string) {
                if (c == '\\') { ++pos; }
                else if (c == '"') { in_string = false; }
            } else {
                if (c == '"') { in_string = true; }
                else if (c == '{') { ++depth; }
                else if (c == '}') { --depth; if (depth == 0) { ++pos; break; } }
            }
            ++pos;
        }
        result.push_back(arr.substr(start, pos - start));
    }
    return result;
}

// ---------------------------------------------------------------------------
// WorldState
// ---------------------------------------------------------------------------

std::string ame_pack_world_state(const WorldStateSnapshot& snap) {
    std::ostringstream os;
    os << "{\"success\":" << (snap.success ? "true" : "false")
       << ",\"wm_version\":" << snap.wm_version
       << ",\"facts\":[";
    for (size_t i = 0; i < snap.facts.size(); ++i) {
        if (i > 0) os << ',';
        const auto& f = snap.facts[i];
        os << "{\"key\":\"" << jsonEscape(f.key) << "\""
           << ",\"value\":" << (f.value ? "true" : "false")
           << ",\"source\":\"" << jsonEscape(f.source) << "\""
           << ",\"wm_version\":" << f.wm_version << '}';
    }
    os << "],\"goal_fluents\":" << jsonStringArray(snap.goal_fluents) << '}';
    return os.str();
}

WorldStateSnapshot ame_unpack_world_state(const pcl_msg_t* msg) {
    WorldStateSnapshot snap;
    if (!msg || !msg->data || msg->size == 0) return snap;
    std::string json(static_cast<const char*>(msg->data), msg->size);

    snap.success    = jsonGetBool(json, "success", true);
    snap.wm_version = jsonGetU64(json, "wm_version");

    auto facts_raw  = jsonGetArrayRaw(json, "facts");
    for (const auto& obj : jsonParseObjectArray(facts_raw)) {
        WorldFactValue f;
        f.key        = jsonGetStr(obj, "key");
        f.value      = jsonGetBool(obj, "value");
        f.source     = jsonGetStr(obj, "source");
        f.wm_version = jsonGetU64(obj, "wm_version");
        snap.facts.push_back(std::move(f));
    }

    auto gf_raw = jsonGetArrayRaw(json, "goal_fluents");
    snap.goal_fluents = jsonParseStringArray(gf_raw);
    return snap;
}

// ---------------------------------------------------------------------------
// Detection
// ---------------------------------------------------------------------------

std::string ame_pack_detection(const Detection& det) {
    std::ostringstream os;
    os << "{\"confidence\":" << det.confidence
       << ",\"sensor_source\":\"" << jsonEscape(det.sensor_source) << "\""
       << ",\"entity_id\":\"" << jsonEscape(det.entity_id) << "\""
       << ",\"property_keys\":" << jsonStringArray(det.property_keys)
       << ",\"property_values\":" << jsonStringArray(det.property_values) << '}';
    return os.str();
}

Detection ame_unpack_detection(const pcl_msg_t* msg) {
    Detection det;
    if (!msg || !msg->data || msg->size == 0) return det;
    std::string json(static_cast<const char*>(msg->data), msg->size);

    det.confidence    = static_cast<float>(jsonGetF64(json, "confidence"));
    det.sensor_source = jsonGetStr(json, "sensor_source");
    det.entity_id     = jsonGetStr(json, "entity_id");

    auto keys_raw = jsonGetArrayRaw(json, "property_keys");
    det.property_keys = jsonParseStringArray(keys_raw);

    auto vals_raw = jsonGetArrayRaw(json, "property_values");
    det.property_values = jsonParseStringArray(vals_raw);
    return det;
}

// ---------------------------------------------------------------------------
// GetFact
// ---------------------------------------------------------------------------

std::string ame_pack_get_fact_request(const std::string& key) {
    return "{\"key\":\"" + jsonEscape(key) + "\"}";
}

std::string ame_unpack_get_fact_request_key(const pcl_msg_t* msg) {
    if (!msg || !msg->data || msg->size == 0) return {};
    std::string json(static_cast<const char*>(msg->data), msg->size);
    return jsonGetStr(json, "key");
}

std::string ame_pack_get_fact_response(const GetFactResult& result) {
    std::ostringstream os;
    os << "{\"found\":" << (result.found ? "true" : "false")
       << ",\"value\":" << (result.value ? "true" : "false")
       << ",\"wm_version\":" << result.wm_version << '}';
    return os.str();
}

GetFactResult ame_unpack_get_fact_response(const pcl_msg_t* msg) {
    GetFactResult r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    r.found      = jsonGetBool(json, "found");
    r.value      = jsonGetBool(json, "value");
    r.wm_version = jsonGetU64(json, "wm_version");
    return r;
}

// ---------------------------------------------------------------------------
// SetFact
// ---------------------------------------------------------------------------

std::string ame_pack_set_fact_request(const SetFactRequest& req) {
    std::ostringstream os;
    os << "{\"key\":\"" << jsonEscape(req.key) << "\""
       << ",\"value\":" << (req.value ? "true" : "false")
       << ",\"source\":\"" << jsonEscape(req.source) << "\"}";
    return os.str();
}

SetFactRequest ame_unpack_set_fact_request(const pcl_msg_t* msg) {
    SetFactRequest r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    r.key    = jsonGetStr(json, "key");
    r.value  = jsonGetBool(json, "value");
    r.source = jsonGetStr(json, "source");
    return r;
}

std::string ame_pack_set_fact_response(const SetFactResult& result) {
    std::ostringstream os;
    os << "{\"success\":" << (result.success ? "true" : "false")
       << ",\"wm_version\":" << result.wm_version << '}';
    return os.str();
}

SetFactResult ame_unpack_set_fact_response(const pcl_msg_t* msg) {
    SetFactResult r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    r.success    = jsonGetBool(json, "success");
    r.wm_version = jsonGetU64(json, "wm_version");
    return r;
}

// ---------------------------------------------------------------------------
// QueryState
// ---------------------------------------------------------------------------

std::string ame_pack_query_state_request(const std::vector<std::string>& keys) {
    return "{\"keys\":" + jsonStringArray(keys) + '}';
}

std::vector<std::string> ame_unpack_query_state_request_keys(const pcl_msg_t* msg) {
    if (!msg || !msg->data || msg->size == 0) return {};
    std::string json(static_cast<const char*>(msg->data), msg->size);
    auto arr_raw = jsonGetArrayRaw(json, "keys");
    return jsonParseStringArray(arr_raw);
}

std::string ame_pack_query_state_response(const WorldStateSnapshot& snap) {
    return ame_pack_world_state(snap);
}

WorldStateSnapshot ame_unpack_query_state_response(const pcl_msg_t* msg) {
    return ame_unpack_world_state(msg);
}

// ---------------------------------------------------------------------------
// LoadDomain
// ---------------------------------------------------------------------------

std::string ame_pack_load_domain_request(const LoadDomainRequest& req) {
    std::ostringstream os;
    os << "{\"domain_id\":\"" << jsonEscape(req.domain_id) << "\""
       << ",\"domain_pddl\":\"" << jsonEscape(req.domain_pddl) << "\""
       << ",\"problem_pddl\":\"" << jsonEscape(req.problem_pddl) << "\"}";
    return os.str();
}

LoadDomainRequest ame_unpack_load_domain_request(const pcl_msg_t* msg) {
    LoadDomainRequest r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    r.domain_id   = jsonGetStr(json, "domain_id");
    r.domain_pddl = jsonGetStr(json, "domain_pddl");
    r.problem_pddl = jsonGetStr(json, "problem_pddl");
    return r;
}

std::string ame_pack_load_domain_response(const LoadDomainResponse& resp) {
    std::ostringstream os;
    os << "{\"success\":" << (resp.success ? "true" : "false")
       << ",\"error_msg\":\"" << jsonEscape(resp.error_msg) << "\""
       << ",\"num_fluents\":" << resp.num_fluents
       << ",\"num_ground_actions\":" << resp.num_ground_actions << '}';
    return os.str();
}

LoadDomainResponse ame_unpack_load_domain_response(const pcl_msg_t* msg) {
    LoadDomainResponse r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    r.success          = jsonGetBool(json, "success");
    r.error_msg        = jsonGetStr(json, "error_msg");
    r.num_fluents      = jsonGetU32(json, "num_fluents");
    r.num_ground_actions = jsonGetU32(json, "num_ground_actions");
    return r;
}

// ---------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------

std::string ame_pack_plan_request(const PlanRequest& req) {
    return "{\"goal_fluents\":" + jsonStringArray(req.goal_fluents) + '}';
}

PlanRequest ame_unpack_plan_request(const pcl_msg_t* msg) {
    PlanRequest r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    auto arr_raw = jsonGetArrayRaw(json, "goal_fluents");
    r.goal_fluents = jsonParseStringArray(arr_raw);
    return r;
}

std::string ame_pack_plan_response(const PlanResponse& resp) {
    std::ostringstream os;
    os << "{\"success\":" << (resp.success ? "true" : "false")
       << ",\"solve_time_ms\":" << resp.solve_time_ms
       << ",\"plan_actions\":" << jsonStringArray(resp.plan_actions)
       << ",\"bt_xml\":\"" << jsonEscape(resp.bt_xml) << "\""
       << ",\"error_msg\":\"" << jsonEscape(resp.error_msg) << "\"}";
    return os.str();
}

PlanResponse ame_unpack_plan_response(const pcl_msg_t* msg) {
    PlanResponse r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    r.success      = jsonGetBool(json, "success");
    r.solve_time_ms = jsonGetF64(json, "solve_time_ms");
    r.bt_xml       = jsonGetStr(json, "bt_xml");
    r.error_msg    = jsonGetStr(json, "error_msg");
    auto arr_raw   = jsonGetArrayRaw(json, "plan_actions");
    r.plan_actions = jsonParseStringArray(arr_raw);
    return r;
}

// ---------------------------------------------------------------------------
// DispatchGoals
// ---------------------------------------------------------------------------

std::string ame_pack_dispatch_goals_request(const DispatchGoalsRequest& req) {
    std::ostringstream os;
    os << "{\"goal_fluents\":" << jsonStringArray(req.goal_fluents)
       << ",\"agent_ids\":" << jsonStringArray(req.agent_ids) << '}';
    return os.str();
}

DispatchGoalsRequest ame_unpack_dispatch_goals_request(const pcl_msg_t* msg) {
    DispatchGoalsRequest r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    auto gf_raw = jsonGetArrayRaw(json, "goal_fluents");
    r.goal_fluents = jsonParseStringArray(gf_raw);
    auto ai_raw = jsonGetArrayRaw(json, "agent_ids");
    r.agent_ids = jsonParseStringArray(ai_raw);
    return r;
}

std::string ame_pack_dispatch_goals_response(const DispatchGoalsResponse& resp) {
    std::ostringstream os;
    os << "{\"success\":" << (resp.success ? "true" : "false")
       << ",\"dispatched_agents\":" << jsonStringArray(resp.dispatched_agents)
       << ",\"error_msg\":\"" << jsonEscape(resp.error_msg) << "\"}";
    return os.str();
}

DispatchGoalsResponse ame_unpack_dispatch_goals_response(const pcl_msg_t* msg) {
    DispatchGoalsResponse r;
    if (!msg || !msg->data || msg->size == 0) return r;
    std::string json(static_cast<const char*>(msg->data), msg->size);
    r.success   = jsonGetBool(json, "success");
    r.error_msg = jsonGetStr(json, "error_msg");
    auto da_raw = jsonGetArrayRaw(json, "dispatched_agents");
    r.dispatched_agents = jsonParseStringArray(da_raw);
    return r;
}

}  // namespace ame
