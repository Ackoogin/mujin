// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_data_model_common_cabi_marshal.hpp"
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

void to_c(const pyramid::domain_model::GeodeticPosition& in, pyramid_GeodeticPosition_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->latitude = in.latitude;
  out->longitude = in.longitude;
}

void from_c(const pyramid_GeodeticPosition_c* in, pyramid::domain_model::GeodeticPosition& out) {
  out.latitude = in->latitude;
  out.longitude = in->longitude;
}

void _free_GeodeticPosition(pyramid_GeodeticPosition_c* value) {
  if (!value) {
    return;
  }
}

void to_c(const pyramid::domain_model::PolyArea& in, pyramid_PolyArea_c* out) {
  std::memset(out, 0, sizeof(*out));
  {
    const auto count = in.points.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_GeodeticPosition_c*>(std::malloc(count * sizeof(pyramid_GeodeticPosition_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.points[i], &arr[i]);
      }
      out->points.ptr = arr;
      out->points.len = static_cast<uint32_t>(count);
    }
  }
}

void from_c(const pyramid_PolyArea_c* in, pyramid::domain_model::PolyArea& out) {
  out.points.clear();
  if (in->points.ptr && in->points.len > 0) {
    const auto* arr = static_cast<const pyramid_GeodeticPosition_c*>(in->points.ptr);
    out.points.reserve(in->points.len);
    for (uint32_t i = 0; i < in->points.len; ++i) {
      pyramid::domain_model::GeodeticPosition elem{};
      from_c(&arr[i], elem);
      out.points.push_back(std::move(elem));
    }
  }
}

void _free_PolyArea(pyramid_PolyArea_c* value) {
  if (!value) {
    return;
  }
  if (value->points.ptr) {
    auto* arr = static_cast<pyramid_GeodeticPosition_c*>(const_cast<void*>(value->points.ptr));
    for (uint32_t i = 0; i < value->points.len; ++i) {
      pyramid_GeodeticPosition_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->points.ptr));
    value->points.ptr = nullptr;
    value->points.len = 0;
  }
}

void to_c(const pyramid::domain_model::Achievement& in, pyramid_Achievement_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  out->status = static_cast<int32_t>(in.status);
  out->has_quality = in.quality.has_value() ? 1u : 0u;
  if (in.quality) {
    out->quality = *in.quality;
  }
  out->achieveability = static_cast<int32_t>(in.achieveability);
}

void from_c(const pyramid_Achievement_c* in, pyramid::domain_model::Achievement& out) {
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
  out.status = static_cast<pyramid::domain_model::Progress>(in->status);
  if (in->has_quality) {
    out.quality = in->quality;
  } else {
    out.quality = tl::nullopt;
  }
  out.achieveability = static_cast<pyramid::domain_model::Feasibility>(in->achieveability);
}

void _free_Achievement(pyramid_Achievement_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
}

void to_c(const pyramid::domain_model::Requirement& in, pyramid_Requirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  to_c(in.status, &out->status);
}

void from_c(const pyramid_Requirement_c* in, pyramid::domain_model::Requirement& out) {
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
  from_c(&in->status, out.status);
}

void _free_Requirement(pyramid_Requirement_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  pyramid_Achievement_c_free(&value->status);
}

void to_c(const pyramid::domain_model::Capability& in, pyramid_Capability_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
  out->availability = in.availability ? 1u : 0u;
  dup_str(out->name, in.name);
  {
    const auto count = in.contraint.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_Contraint_c*>(std::malloc(count * sizeof(pyramid_Contraint_c)));
      for (size_t i = 0; i < count; ++i) {
        to_c(in.contraint[i], &arr[i]);
      }
      out->contraint.ptr = arr;
      out->contraint.len = static_cast<uint32_t>(count);
    }
  }
}

void from_c(const pyramid_Capability_c* in, pyramid::domain_model::Capability& out) {
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
  out.availability = in->availability != 0;
  if (in->name.ptr && in->name.len > 0) {
    out.name.assign(in->name.ptr, in->name.len);
  } else {
    out.name.clear();
  }
  out.contraint.clear();
  if (in->contraint.ptr && in->contraint.len > 0) {
    const auto* arr = static_cast<const pyramid_Contraint_c*>(in->contraint.ptr);
    out.contraint.reserve(in->contraint.len);
    for (uint32_t i = 0; i < in->contraint.len; ++i) {
      pyramid::domain_model::Contraint elem{};
      from_c(&arr[i], elem);
      out.contraint.push_back(std::move(elem));
    }
  }
}

void _free_Capability(pyramid_Capability_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  free_str(value->name);
  if (value->contraint.ptr) {
    auto* arr = static_cast<pyramid_Contraint_c*>(const_cast<void*>(value->contraint.ptr));
    for (uint32_t i = 0; i < value->contraint.len; ++i) {
      pyramid_Contraint_c_free(&arr[i]);
    }
    std::free(const_cast<void*>(value->contraint.ptr));
    value->contraint.ptr = nullptr;
    value->contraint.len = 0;
  }
}

void to_c(const pyramid::domain_model::Entity& in, pyramid_Entity_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
}

void from_c(const pyramid_Entity_c* in, pyramid::domain_model::Entity& out) {
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
}

void _free_Entity(pyramid_Entity_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
}

void to_c(const pyramid::domain_model::CircleArea& in, pyramid_CircleArea_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.position, &out->position);
  out->radius = in.radius;
}

void from_c(const pyramid_CircleArea_c* in, pyramid::domain_model::CircleArea& out) {
  from_c(&in->position, out.position);
  out.radius = in->radius;
}

void _free_CircleArea(pyramid_CircleArea_c* value) {
  if (!value) {
    return;
  }
  pyramid_GeodeticPosition_c_free(&value->position);
}

void to_c(const pyramid::domain_model::Point& in, pyramid_Point_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.position, &out->position);
}

void from_c(const pyramid_Point_c* in, pyramid::domain_model::Point& out) {
  from_c(&in->position, out.position);
}

void _free_Point(pyramid_Point_c* value) {
  if (!value) {
    return;
  }
  pyramid_GeodeticPosition_c_free(&value->position);
}

void to_c(const pyramid::domain_model::Contraint& in, pyramid_Contraint_c* out) {
  std::memset(out, 0, sizeof(*out));
  dup_str(out->name, in.name);
  out->value = in.value;
}

void from_c(const pyramid_Contraint_c* in, pyramid::domain_model::Contraint& out) {
  if (in->name.ptr && in->name.len > 0) {
    out.name.assign(in->name.ptr, in->name.len);
  } else {
    out.name.clear();
  }
  out.value = in->value;
}

void _free_Contraint(pyramid_Contraint_c* value) {
  if (!value) {
    return;
  }
  free_str(value->name);
}

void to_c(const pyramid::domain_model::Ack& in, pyramid_Ack_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->success = in.success ? 1u : 0u;
}

void from_c(const pyramid_Ack_c* in, pyramid::domain_model::Ack& out) {
  out.success = in->success != 0;
}

void _free_Ack(pyramid_Ack_c* value) {
  if (!value) {
    return;
  }
}

void to_c(const pyramid::domain_model::Query& in, pyramid_Query_c* out) {
  std::memset(out, 0, sizeof(*out));
  {
    const auto count = in.id.size();
    if (count > 0) {
      auto* arr = static_cast<pyramid_str_t*>(std::malloc(count * sizeof(pyramid_str_t)));
      for (size_t i = 0; i < count; ++i) {
        dup_str(arr[i], in.id[i]);
      }
      out->id.ptr = arr;
      out->id.len = static_cast<uint32_t>(count);
    }
  }
  out->has_one_shot = in.one_shot.has_value() ? 1u : 0u;
  if (in.one_shot) {
    out->one_shot = (*in.one_shot) ? 1u : 0u;
  }
}

void from_c(const pyramid_Query_c* in, pyramid::domain_model::Query& out) {
  out.id.clear();
  if (in->id.ptr && in->id.len > 0) {
    const auto* arr = static_cast<const pyramid_str_t*>(in->id.ptr);
    out.id.reserve(in->id.len);
    for (uint32_t i = 0; i < in->id.len; ++i) {
      if (arr[i].ptr && arr[i].len > 0) {
        out.id.emplace_back(arr[i].ptr, arr[i].len);
      } else {
        out.id.emplace_back();
      }
    }
  }
  if (in->has_one_shot) {
    out.one_shot = in->one_shot != 0;
  } else {
    out.one_shot = tl::nullopt;
  }
}

void _free_Query(pyramid_Query_c* value) {
  if (!value) {
    return;
  }
  if (value->id.ptr) {
    auto* arr = static_cast<pyramid_str_t*>(const_cast<void*>(value->id.ptr));
    for (uint32_t i = 0; i < value->id.len; ++i) {
      free_str(arr[i]);
    }
    std::free(const_cast<void*>(value->id.ptr));
    value->id.ptr = nullptr;
    value->id.len = 0;
  }
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_GeodeticPosition_c_free(pyramid_GeodeticPosition_c* value) {
  pyramid::cabi::_free_GeodeticPosition(value);
}

void pyramid_PolyArea_c_free(pyramid_PolyArea_c* value) {
  pyramid::cabi::_free_PolyArea(value);
}

void pyramid_Achievement_c_free(pyramid_Achievement_c* value) {
  pyramid::cabi::_free_Achievement(value);
}

void pyramid_Requirement_c_free(pyramid_Requirement_c* value) {
  pyramid::cabi::_free_Requirement(value);
}

void pyramid_Capability_c_free(pyramid_Capability_c* value) {
  pyramid::cabi::_free_Capability(value);
}

void pyramid_Entity_c_free(pyramid_Entity_c* value) {
  pyramid::cabi::_free_Entity(value);
}

void pyramid_CircleArea_c_free(pyramid_CircleArea_c* value) {
  pyramid::cabi::_free_CircleArea(value);
}

void pyramid_Point_c_free(pyramid_Point_c* value) {
  pyramid::cabi::_free_Point(value);
}

void pyramid_Contraint_c_free(pyramid_Contraint_c* value) {
  pyramid::cabi::_free_Contraint(value);
}

void pyramid_Ack_c_free(pyramid_Ack_c* value) {
  pyramid::cabi::_free_Ack(value);
}

void pyramid_Query_c_free(pyramid_Query_c* value) {
  pyramid::cabi::_free_Query(value);
}

} // extern "C"
