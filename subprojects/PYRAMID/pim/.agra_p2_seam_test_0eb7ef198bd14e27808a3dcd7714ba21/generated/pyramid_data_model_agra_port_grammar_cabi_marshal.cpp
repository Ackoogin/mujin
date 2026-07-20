// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_data_model_agra_port_grammar_cabi_marshal.hpp"
#include <pcl/pcl_alloc.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

void dup_str(pyramid_str_t& out, const std::string& in) {
  if (in.empty()) {
    out.ptr = nullptr;
    out.len = 0;
    return;
  }
  out.len = static_cast<uint32_t>(in.size());
  out.ptr = static_cast<const char*>(pcl_alloc(out.len));
  std::memcpy(const_cast<char*>(out.ptr), in.data(), out.len);
}

void free_str(pyramid_str_t& s) {
  if (s.ptr) {
    pcl_free(const_cast<char*>(s.ptr));
    s.ptr = nullptr;
    s.len = 0;
  }
}

} // namespace

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::agra_port_grammar::Identifier& in, pyramid_data_model_agra_port_grammar_Identifier_c* out) {
  std::memset(out, 0, sizeof(*out));
  dup_str(out->id, in.id);
}

void from_c(const pyramid_data_model_agra_port_grammar_Identifier_c* in, pyramid::domain_model::agra_port_grammar::Identifier& out) {
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
}

void _free_Identifier(pyramid_data_model_agra_port_grammar_Identifier_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
}

void to_c(const pyramid::domain_model::agra_port_grammar::Ack& in, pyramid_data_model_agra_port_grammar_Ack_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->success = in.success ? 1u : 0u;
}

void from_c(const pyramid_data_model_agra_port_grammar_Ack_c* in, pyramid::domain_model::agra_port_grammar::Ack& out) {
  out.success = in->success != 0;
}

void _free_Ack(pyramid_data_model_agra_port_grammar_Ack_c* value) {
  if (!value) {
    return;
  }
}

void to_c(const pyramid::domain_model::agra_port_grammar::Query& in, pyramid_data_model_agra_port_grammar_Query_c* out) {
  std::memset(out, 0, sizeof(*out));
}

void from_c(const pyramid_data_model_agra_port_grammar_Query_c* in, pyramid::domain_model::agra_port_grammar::Query& out) {
}

void _free_Query(pyramid_data_model_agra_port_grammar_Query_c* value) {
  if (!value) {
    return;
  }
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_data_model_agra_port_grammar_Identifier_c_free(pyramid_data_model_agra_port_grammar_Identifier_c* value) {
  pyramid::cabi::_free_Identifier(value);
}

void pyramid_data_model_agra_port_grammar_Ack_c_free(pyramid_data_model_agra_port_grammar_Ack_c* value) {
  pyramid::cabi::_free_Ack(value);
}

void pyramid_data_model_agra_port_grammar_Query_c_free(pyramid_data_model_agra_port_grammar_Query_c* value) {
  pyramid::cabi::_free_Query(value);
}

} // extern "C"
