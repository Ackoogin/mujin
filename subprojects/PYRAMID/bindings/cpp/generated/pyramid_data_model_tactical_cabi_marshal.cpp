// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_data_model_tactical_cabi_marshal.hpp"
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

void to_c(const pyramid::domain_model::ObjectDetail& in, pyramid_ObjectDetail_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->entity_source, in.entity_source);
  {
    const auto count = in.source.size();
    if (count > 0) {
      auto* arr = static_cast<int32_t*>(std::malloc(count * sizeof(int32_t)));
      for (size_t i = 0; i < count; ++i) {
        arr[i] = static_cast<int32_t>(in.source[i]);
      }
      out->source.ptr = arr;
      out->source.len = static_cast<uint32_t>(count);
    }
  }
  to_c(in.position, &out->position);
  out->creation_time = in.creation_time;
  out->has_quality = in.quality.has_value() ? 1u : 0u;
  if (in.quality) {
    out->quality = *in.quality;
  }
  out->has_course = in.course.has_value() ? 1u : 0u;
  if (in.course) {
    out->course = *in.course;
  }
  out->has_speed = in.speed.has_value() ? 1u : 0u;
  if (in.speed) {
    out->speed = *in.speed;
  }
  out->has_length = in.length.has_value() ? 1u : 0u;
  if (in.length) {
    out->length = *in.length;
  }
  out->identity = static_cast<int32_t>(in.identity);
  out->dimension = static_cast<int32_t>(in.dimension);
}

void from_c(const pyramid_ObjectDetail_c* in, pyramid::domain_model::ObjectDetail& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->entity_source.ptr && in->entity_source.len > 0) {
    out.entity_source.assign(in->entity_source.ptr, in->entity_source.len);
  } else {
    out.entity_source.clear();
  }
  out.source.clear();
  if (in->source.ptr && in->source.len > 0) {
    const auto* arr = static_cast<const int32_t*>(in->source.ptr);
    out.source.reserve(in->source.len);
    for (uint32_t i = 0; i < in->source.len; ++i) {
      out.source.push_back(static_cast<pyramid::domain_model::ObjectSource>(arr[i]));
    }
  }
  from_c(&in->position, out.position);
  out.creation_time = in->creation_time;
  if (in->has_quality) {
    out.quality = in->quality;
  } else {
    out.quality = tl::nullopt;
  }
  if (in->has_course) {
    out.course = in->course;
  } else {
    out.course = tl::nullopt;
  }
  if (in->has_speed) {
    out.speed = in->speed;
  } else {
    out.speed = tl::nullopt;
  }
  if (in->has_length) {
    out.length = in->length;
  } else {
    out.length = tl::nullopt;
  }
  out.identity = static_cast<pyramid::domain_model::common::StandardIdentity>(in->identity);
  out.dimension = static_cast<pyramid::domain_model::common::BattleDimension>(in->dimension);
}

void _free_ObjectDetail(pyramid_ObjectDetail_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->entity_source);
  if (value->source.ptr) {
    std::free(const_cast<void*>(value->source.ptr));
    value->source.ptr = nullptr;
    value->source.len = 0;
  }
  pyramid_GeodeticPosition_c_free(&value->position);
}

void to_c(const pyramid::domain_model::ObjectEvidenceRequirement& in, pyramid_ObjectEvidenceRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  out->policy = static_cast<int32_t>(in.policy);
  {
    const auto count = in.dimension.size();
    if (count > 0) {
      auto* arr = static_cast<int32_t*>(std::malloc(count * sizeof(int32_t)));
      for (size_t i = 0; i < count; ++i) {
        arr[i] = static_cast<int32_t>(in.dimension[i]);
      }
      out->dimension.ptr = arr;
      out->dimension.len = static_cast<uint32_t>(count);
    }
  }
  out->has_poly_area = in.poly_area.has_value() ? 1u : 0u;
  if (in.poly_area) {
    to_c(*in.poly_area, &out->poly_area);
  }
  out->has_circle_area = in.circle_area.has_value() ? 1u : 0u;
  if (in.circle_area) {
    to_c(*in.circle_area, &out->circle_area);
  }
  out->has_point = in.point.has_value() ? 1u : 0u;
  if (in.point) {
    to_c(*in.point, &out->point);
  }
}

void from_c(const pyramid_ObjectEvidenceRequirement_c* in, pyramid::domain_model::ObjectEvidenceRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  out.policy = static_cast<pyramid::domain_model::common::DataPolicy>(in->policy);
  out.dimension.clear();
  if (in->dimension.ptr && in->dimension.len > 0) {
    const auto* arr = static_cast<const int32_t*>(in->dimension.ptr);
    out.dimension.reserve(in->dimension.len);
    for (uint32_t i = 0; i < in->dimension.len; ++i) {
      out.dimension.push_back(static_cast<pyramid::domain_model::common::BattleDimension>(arr[i]));
    }
  }
  if (in->has_poly_area) {
    out.poly_area.emplace();
    from_c(&in->poly_area, *out.poly_area);
  } else {
    out.poly_area = tl::nullopt;
  }
  if (in->has_circle_area) {
    out.circle_area.emplace();
    from_c(&in->circle_area, *out.circle_area);
  } else {
    out.circle_area = tl::nullopt;
  }
  if (in->has_point) {
    out.point.emplace();
    from_c(&in->point, *out.point);
  } else {
    out.point = tl::nullopt;
  }
}

void _free_ObjectEvidenceRequirement(pyramid_ObjectEvidenceRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
  if (value->dimension.ptr) {
    std::free(const_cast<void*>(value->dimension.ptr));
    value->dimension.ptr = nullptr;
    value->dimension.len = 0;
  }
  if (value->has_poly_area) {
    pyramid_PolyArea_c_free(&value->poly_area);
    value->has_poly_area = 0;
  }
  if (value->has_circle_area) {
    pyramid_CircleArea_c_free(&value->circle_area);
    value->has_circle_area = 0;
  }
  if (value->has_point) {
    pyramid_Point_c_free(&value->point);
    value->has_point = 0;
  }
}

void to_c(const pyramid::domain_model::ObjectInterestRequirement& in, pyramid_ObjectInterestRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  out->has_source = in.source.has_value() ? 1u : 0u;
  if (in.source) {
    out->source = static_cast<int32_t>(*in.source);
  }
  out->policy = static_cast<int32_t>(in.policy);
  {
    const auto count = in.dimension.size();
    if (count > 0) {
      auto* arr = static_cast<int32_t*>(std::malloc(count * sizeof(int32_t)));
      for (size_t i = 0; i < count; ++i) {
        arr[i] = static_cast<int32_t>(in.dimension[i]);
      }
      out->dimension.ptr = arr;
      out->dimension.len = static_cast<uint32_t>(count);
    }
  }
  out->has_poly_area = in.poly_area.has_value() ? 1u : 0u;
  if (in.poly_area) {
    to_c(*in.poly_area, &out->poly_area);
  }
  out->has_circle_area = in.circle_area.has_value() ? 1u : 0u;
  if (in.circle_area) {
    to_c(*in.circle_area, &out->circle_area);
  }
  out->has_point = in.point.has_value() ? 1u : 0u;
  if (in.point) {
    to_c(*in.point, &out->point);
  }
}

void from_c(const pyramid_ObjectInterestRequirement_c* in, pyramid::domain_model::ObjectInterestRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  if (in->has_source) {
    out.source = static_cast<pyramid::domain_model::ObjectSource>(in->source);
  } else {
    out.source = tl::nullopt;
  }
  out.policy = static_cast<pyramid::domain_model::common::DataPolicy>(in->policy);
  out.dimension.clear();
  if (in->dimension.ptr && in->dimension.len > 0) {
    const auto* arr = static_cast<const int32_t*>(in->dimension.ptr);
    out.dimension.reserve(in->dimension.len);
    for (uint32_t i = 0; i < in->dimension.len; ++i) {
      out.dimension.push_back(static_cast<pyramid::domain_model::common::BattleDimension>(arr[i]));
    }
  }
  if (in->has_poly_area) {
    out.poly_area.emplace();
    from_c(&in->poly_area, *out.poly_area);
  } else {
    out.poly_area = tl::nullopt;
  }
  if (in->has_circle_area) {
    out.circle_area.emplace();
    from_c(&in->circle_area, *out.circle_area);
  } else {
    out.circle_area = tl::nullopt;
  }
  if (in->has_point) {
    out.point.emplace();
    from_c(&in->point, *out.point);
  } else {
    out.point = tl::nullopt;
  }
}

void _free_ObjectInterestRequirement(pyramid_ObjectInterestRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
  if (value->dimension.ptr) {
    std::free(const_cast<void*>(value->dimension.ptr));
    value->dimension.ptr = nullptr;
    value->dimension.len = 0;
  }
  if (value->has_poly_area) {
    pyramid_PolyArea_c_free(&value->poly_area);
    value->has_poly_area = 0;
  }
  if (value->has_circle_area) {
    pyramid_CircleArea_c_free(&value->circle_area);
    value->has_circle_area = 0;
  }
  if (value->has_point) {
    pyramid_Point_c_free(&value->point);
    value->has_point = 0;
  }
}

void to_c(const pyramid::domain_model::ObjectMatch& in, pyramid_ObjectMatch_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  dup_str(out->matching_object_id, in.matching_object_id);
}

void from_c(const pyramid_ObjectMatch_c* in, pyramid::domain_model::ObjectMatch& out) {
  if (in->has_update_time) {
    out.update_time = in->update_time;
  } else {
    out.update_time = tl::nullopt;
  }
  if (in->id.ptr && in->id.len > 0) {
    out.id.assign(in->id.ptr, in->id.len);
  } else {
    out.id.clear();
  }
  if (in->source.ptr && in->source.len > 0) {
    out.source.assign(in->source.ptr, in->source.len);
  } else {
    out.source.clear();
  }
  if (in->matching_object_id.ptr && in->matching_object_id.len > 0) {
    out.matching_object_id.assign(in->matching_object_id.ptr, in->matching_object_id.len);
  } else {
    out.matching_object_id.clear();
  }
}

void _free_ObjectMatch(pyramid_ObjectMatch_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->matching_object_id);
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_ObjectDetail_c_free(pyramid_ObjectDetail_c* value) {
  pyramid::cabi::_free_ObjectDetail(value);
}

void pyramid_ObjectEvidenceRequirement_c_free(pyramid_ObjectEvidenceRequirement_c* value) {
  pyramid::cabi::_free_ObjectEvidenceRequirement(value);
}

void pyramid_ObjectInterestRequirement_c_free(pyramid_ObjectInterestRequirement_c* value) {
  pyramid::cabi::_free_ObjectInterestRequirement(value);
}

void pyramid_ObjectMatch_c_free(pyramid_ObjectMatch_c* value) {
  pyramid::cabi::_free_ObjectMatch(value);
}

} // extern "C"
