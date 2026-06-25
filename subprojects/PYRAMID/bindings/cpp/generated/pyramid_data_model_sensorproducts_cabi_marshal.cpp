// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_data_model_sensorproducts_cabi_marshal.hpp"
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
  out.ptr = static_cast<const char*>(std::malloc(out.len));
  std::memcpy(const_cast<char*>(out.ptr), in.data(), out.len);
}

void free_str(pyramid_str_t& s) {
  if (s.ptr) {
    std::free(const_cast<char*>(s.ptr));
    s.ptr = nullptr;
    s.len = 0;
  }
}

} // namespace

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::RadarDisplayProductRequirement& in, pyramid_RadarDisplayProductRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
}

void from_c(const pyramid_RadarDisplayProductRequirement_c* in, pyramid::domain_model::RadarDisplayProductRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
}

void _free_RadarDisplayProductRequirement(pyramid_RadarDisplayProductRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
}

void to_c(const pyramid::domain_model::RadarProductRequirement& in, pyramid_RadarProductRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
}

void from_c(const pyramid_RadarProductRequirement_c* in, pyramid::domain_model::RadarProductRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
}

void _free_RadarProductRequirement(pyramid_RadarProductRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_RadarDisplayProductRequirement_c_free(pyramid_RadarDisplayProductRequirement_c* value) {
  pyramid::cabi::_free_RadarDisplayProductRequirement(value);
}

void pyramid_RadarProductRequirement_c_free(pyramid_RadarProductRequirement_c* value) {
  pyramid::cabi::_free_RadarProductRequirement(value);
}

} // extern "C"
