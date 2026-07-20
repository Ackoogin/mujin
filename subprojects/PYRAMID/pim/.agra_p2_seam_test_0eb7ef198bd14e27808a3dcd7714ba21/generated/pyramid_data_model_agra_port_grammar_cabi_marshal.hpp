#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_agra_port_grammar_cabi.h"

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::agra_port_grammar::Identifier& in, pyramid_data_model_agra_port_grammar_Identifier_c* out);
void from_c(const pyramid_data_model_agra_port_grammar_Identifier_c* in, pyramid::domain_model::agra_port_grammar::Identifier& out);

void to_c(const pyramid::domain_model::agra_port_grammar::Ack& in, pyramid_data_model_agra_port_grammar_Ack_c* out);
void from_c(const pyramid_data_model_agra_port_grammar_Ack_c* in, pyramid::domain_model::agra_port_grammar::Ack& out);

void to_c(const pyramid::domain_model::agra_port_grammar::Query& in, pyramid_data_model_agra_port_grammar_Query_c* out);
void from_c(const pyramid_data_model_agra_port_grammar_Query_c* in, pyramid::domain_model::agra_port_grammar::Query& out);

} // namespace pyramid::cabi
