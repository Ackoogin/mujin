#include "ame/neuro/neuro_config.h"

#include <sstream>

namespace ame::neuro {

const IntegrationConfig* NeuroConfig::find(const std::string& kind) const {
    for (const auto& ic : integrations) {
        if (ic.integration_kind == kind) return &ic;
    }
    return nullptr;
}

FallbackPolicy NeuroConfig::policy_for(const std::string& kind) const {
    if (!enabled) return FallbackPolicy::disabled();
    const IntegrationConfig* ic = find(kind);
    if (!ic) return FallbackPolicy::disabled();
    FallbackPolicy p = ic->policy;
    if (!enabled) p.enabled = false;
    return p;
}

// Minimal JSON parser: handles flat objects with string and bool fields.
// Sufficient for NeuroConfig; not a general-purpose parser.
NeuroConfig NeuroConfig::from_json(const std::string& json_text) {
    NeuroConfig cfg;

    // Extracts a value for a key that appears at top-level (depth == 1) only.
    // Prevents nested integration objects' "enabled" keys from poisoning the
    // global kill-switch field.
    // We check for the "key": pattern exactly when we encounter a '"' at depth 1
    // (before consuming the string), so the bracket depth is still accurate.
    auto get_top_val = [&](const std::string& key) -> std::string {
        std::string search = "\"" + key + "\":";
        int depth = 0;
        bool in_str = false;
        size_t i = 0;
        while (i < json_text.size()) {
            char c = json_text[i];
            if (in_str) {
                if (c == '\\') { i += 2; continue; } // skip escaped char
                if (c == '"') { in_str = false; }
                ++i; continue;
            }
            // At depth 1 and starting a quoted token: check for the target key
            // before entering string mode so we can test from the opening '"'.
            if (c == '"' && depth == 1 &&
                json_text.compare(i, search.size(), search) == 0) {
                size_t pos = i + search.size();
                while (pos < json_text.size() && json_text[pos] == ' ') ++pos;
                if (pos >= json_text.size()) return {};
                if (json_text[pos] == '"') {
                    ++pos;
                    std::string val;
                    while (pos < json_text.size() && json_text[pos] != '"') val += json_text[pos++];
                    return val;
                }
                std::string val;
                while (pos < json_text.size() && json_text[pos] != ',' &&
                       json_text[pos] != '}' && json_text[pos] != '\n')
                    val += json_text[pos++];
                return val;
            }
            if (c == '"')             { in_str = true;  ++i; continue; }
            if (c == '{' || c == '[') { ++depth;         ++i; continue; }
            if (c == '}' || c == ']') { --depth;         ++i; continue; }
            ++i;
        }
        return {};
    };

    std::string enabled_str = get_top_val("enabled");
    cfg.enabled = (enabled_str == "true");

    // Parse integrations array (simplified: one integration per object block).
    // Each integration block looks like: {"integration_kind":"x","enabled":true,...}
    size_t pos = 0;
    while ((pos = json_text.find("\"integration_kind\"", pos)) != std::string::npos) {
        // Find the containing object by looking backward for '{'
        size_t obj_start = json_text.rfind('{', pos);
        // Find the matching '}'
        size_t obj_end = json_text.find('}', pos);
        if (obj_start == std::string::npos || obj_end == std::string::npos) break;
        std::string block = json_text.substr(obj_start, obj_end - obj_start + 1);

        auto bget = [&](const std::string& k) -> std::string {
            std::string s = "\"" + k + "\":";
            auto bp = block.find(s);
            if (bp == std::string::npos) return {};
            bp += s.size();
            while (bp < block.size() && block[bp] == ' ') ++bp;
            if (bp >= block.size()) return {};
            if (block[bp] == '"') {
                ++bp; std::string v;
                while (bp < block.size() && block[bp] != '"') v += block[bp++];
                return v;
            }
            std::string v;
            while (bp < block.size() && block[bp] != ',' && block[bp] != '}') v += block[bp++];
            return v;
        };

        IntegrationConfig ic;
        ic.integration_kind = bget("integration_kind");
        if (ic.integration_kind.empty()) { pos = obj_end + 1; continue; }

        std::string ic_enabled = bget("enabled");
        ic.policy.enabled = (ic_enabled == "true") && cfg.enabled;

        std::string budget = bget("latency_budget_ms");
        if (!budget.empty()) try { ic.policy.latency_budget_ms = std::stod(budget); } catch (...) {}

        std::string retries = bget("max_retries");
        if (!retries.empty()) try { ic.policy.max_retries = (unsigned)std::stoul(retries); } catch (...) {}

        ic.policy.backend_id = bget("backend_id");
        ic.policy.model_id   = bget("model_id");
        ic.policy.authority_view = bget("authority_view");

        cfg.integrations.push_back(std::move(ic));
        pos = obj_end + 1;
    }

    return cfg;
}

} // namespace ame::neuro
