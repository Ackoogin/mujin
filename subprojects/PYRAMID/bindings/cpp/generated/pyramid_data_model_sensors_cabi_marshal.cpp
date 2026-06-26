// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_data_model_sensors_cabi_marshal.hpp"
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

void to_c(const pyramid::domain_model::InterpretationRequirement& in, pyramid_InterpretationRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  out->policy = static_cast<int32_t>(in.policy);
  out->type = static_cast<int32_t>(in.type);
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

void from_c(const pyramid_InterpretationRequirement_c* in, pyramid::domain_model::InterpretationRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  out.policy = static_cast<pyramid::domain_model::InterpretationPolicy>(in->policy);
  out.type = static_cast<pyramid::domain_model::InterpretationType>(in->type);
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

void _free_InterpretationRequirement(pyramid_InterpretationRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
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

void to_c(const pyramid::domain_model::ManualTrackRequirement& in, pyramid_ManualTrackRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  to_c(in.position, &out->position);
}

void from_c(const pyramid_ManualTrackRequirement_c* in, pyramid::domain_model::ManualTrackRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  from_c(&in->position, out.position);
}

void _free_ManualTrackRequirement(pyramid_ManualTrackRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
  pyramid_GeodeticPosition_c_free(&value->position);
}

void to_c(const pyramid::domain_model::ATIRequirement& in, pyramid_ATIRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  to_c(in.auto_zone, &out->auto_zone);
}

void from_c(const pyramid_ATIRequirement_c* in, pyramid::domain_model::ATIRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  from_c(&in->auto_zone, out.auto_zone);
}

void _free_ATIRequirement(pyramid_ATIRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
  pyramid_PolyArea_c_free(&value->auto_zone);
}

void to_c(const pyramid::domain_model::TrackProvisionRequirement& in, pyramid_TrackProvisionRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
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

void from_c(const pyramid_TrackProvisionRequirement_c* in, pyramid::domain_model::TrackProvisionRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
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

void _free_TrackProvisionRequirement(pyramid_TrackProvisionRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
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

void to_c(const pyramid::domain_model::ObjectEvidenceProvisionRequirement& in, pyramid_ObjectEvidenceProvisionRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
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

void from_c(const pyramid_ObjectEvidenceProvisionRequirement_c* in, pyramid::domain_model::ObjectEvidenceProvisionRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
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

void _free_ObjectEvidenceProvisionRequirement(pyramid_ObjectEvidenceProvisionRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
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

void to_c(const pyramid::domain_model::ObjectAquisitionRequirement& in, pyramid_ObjectAquisitionRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
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

void from_c(const pyramid_ObjectAquisitionRequirement_c* in, pyramid::domain_model::ObjectAquisitionRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
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

void _free_ObjectAquisitionRequirement(pyramid_ObjectAquisitionRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
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

void to_c(const pyramid::domain_model::SensorObject& in, pyramid_SensorObject_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_update_time = in.update_time.has_value() ? 1u : 0u;
  if (in.update_time) {
    out->update_time = *in.update_time;
  }
  dup_str(out->id, in.id);
  dup_str(out->source, in.source);
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
}

void from_c(const pyramid_SensorObject_c* in, pyramid::domain_model::SensorObject& out) {
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
}

void _free_SensorObject(pyramid_SensorObject_c* value) {
  if (!value) {
    return;
  }
  free_str(value->id);
  free_str(value->source);
  pyramid_GeodeticPosition_c_free(&value->position);
}

void to_c(const pyramid::domain_model::RadarModeChangeRequirement& in, pyramid_RadarModeChangeRequirement_c* out) {
  std::memset(out, 0, sizeof(*out));
  to_c(in.base, &out->base);
  to_c(in.status, &out->status);
  out->mode = static_cast<int32_t>(in.mode);
}

void from_c(const pyramid_RadarModeChangeRequirement_c* in, pyramid::domain_model::RadarModeChangeRequirement& out) {
  from_c(&in->base, out.base);
  from_c(&in->status, out.status);
  out.mode = static_cast<pyramid::domain_model::radar::Radar_Operational_Mode>(in->mode);
}

void _free_RadarModeChangeRequirement(pyramid_RadarModeChangeRequirement_c* value) {
  if (!value) {
    return;
  }
  pyramid_Entity_c_free(&value->base);
  pyramid_Achievement_c_free(&value->status);
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_InterpretationRequirement_c_free(pyramid_InterpretationRequirement_c* value) {
  pyramid::cabi::_free_InterpretationRequirement(value);
}

void pyramid_ManualTrackRequirement_c_free(pyramid_ManualTrackRequirement_c* value) {
  pyramid::cabi::_free_ManualTrackRequirement(value);
}

void pyramid_ATIRequirement_c_free(pyramid_ATIRequirement_c* value) {
  pyramid::cabi::_free_ATIRequirement(value);
}

void pyramid_TrackProvisionRequirement_c_free(pyramid_TrackProvisionRequirement_c* value) {
  pyramid::cabi::_free_TrackProvisionRequirement(value);
}

void pyramid_ObjectEvidenceProvisionRequirement_c_free(pyramid_ObjectEvidenceProvisionRequirement_c* value) {
  pyramid::cabi::_free_ObjectEvidenceProvisionRequirement(value);
}

void pyramid_ObjectAquisitionRequirement_c_free(pyramid_ObjectAquisitionRequirement_c* value) {
  pyramid::cabi::_free_ObjectAquisitionRequirement(value);
}

void pyramid_SensorObject_c_free(pyramid_SensorObject_c* value) {
  pyramid::cabi::_free_SensorObject(value);
}

void pyramid_RadarModeChangeRequirement_c_free(pyramid_RadarModeChangeRequirement_c* value) {
  pyramid::cabi::_free_RadarModeChangeRequirement(value);
}

} // extern "C"
