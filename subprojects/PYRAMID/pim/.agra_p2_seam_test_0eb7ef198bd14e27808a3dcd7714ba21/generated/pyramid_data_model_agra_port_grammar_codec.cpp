// Auto-generated data model JSON codec implementation
// Namespace: pyramid::domain_model::agra_port_grammar

#include "pyramid_data_model_agra_port_grammar_codec.hpp"

#include <nlohmann/json.hpp>


namespace pyramid::domain_model::agra_port_grammar {

std::string toJson(const Identifier& msg) {
    nlohmann::json obj;
    obj["id"] = msg.id;
    return obj.dump();
}

Identifier fromJson(const std::string& s, Identifier* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Identifier msg;
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    return msg;
}

std::string toJson(const Ack& msg) {
    nlohmann::json obj;
    obj["success"] = msg.success;
    return obj.dump();
}

Ack fromJson(const std::string& s, Ack* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Ack msg;
    if (j.contains("success")) msg.success = j["success"].get<bool>();
    return msg;
}

std::string toJson(const Query& msg) {
    nlohmann::json obj;
    return obj.dump();
}

Query fromJson(const std::string& s, Query* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Query msg;
    return msg;
}

} // namespace pyramid::domain_model::agra_port_grammar
