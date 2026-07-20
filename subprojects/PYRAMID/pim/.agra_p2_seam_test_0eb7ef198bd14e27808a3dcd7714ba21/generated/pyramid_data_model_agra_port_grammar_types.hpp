// Auto-generated types header
// Generated from: pyramid.data_model.agra.port_grammar.proto by generate_bindings.py (types)
// Namespace: pyramid::domain_model::agra_port_grammar
#pragma once

#include <cstdint>
#include <tl/optional.hpp>
#include <string>
#include <vector>

namespace pyramid::domain_model::agra_port_grammar {


/// \brief These port-grammar helpers are not part of the A-GRA XSD data model. They
/// use a distinct package because the binding generator emits one types header
/// per package. Reopening pyramid.data_model.agra here would replace the header
/// generated from the converted XSD tree instead of merging with it.
struct Identifier {
    std::string id = {};
};

struct Ack {
    bool success = false;
};
constexpr Ack kAckOk{ true  };
constexpr Ack kAckFail{ false };

struct Query {
};

} // namespace pyramid::domain_model::agra_port_grammar
