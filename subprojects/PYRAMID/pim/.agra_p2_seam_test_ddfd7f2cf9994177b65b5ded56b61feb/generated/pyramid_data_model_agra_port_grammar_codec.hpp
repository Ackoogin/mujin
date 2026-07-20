// Auto-generated data model JSON codec header
// Generated from: pyramid.data_model.agra.port_grammar.proto by generate_bindings.py (codec)
// Namespace: pyramid::domain_model::agra_port_grammar
#pragma once

#include "pyramid_data_model_agra_port_grammar_types.hpp"
#include <string>

namespace pyramid::domain_model::agra_port_grammar {

// JSON codec
std::string toJson(const Identifier& msg);
Identifier fromJson(const std::string& s, Identifier* /*tag*/ = nullptr);
std::string toJson(const Ack& msg);
Ack fromJson(const std::string& s, Ack* /*tag*/ = nullptr);
std::string toJson(const Query& msg);
Query fromJson(const std::string& s, Query* /*tag*/ = nullptr);

} // namespace pyramid::domain_model::agra_port_grammar
