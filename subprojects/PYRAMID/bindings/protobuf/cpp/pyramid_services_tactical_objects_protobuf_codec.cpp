#include "pyramid_services_tactical_objects_protobuf_codec.hpp"

#include "pyramid/data_model/base.pb.h"
#include "pyramid/data_model/common.pb.h"
#include "pyramid/data_model/tactical.pb.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::protobuf_codec {

namespace {

void append_varint32(uint32_t value, std::string& out) {
  while (value >= 0x80U) {
    out.push_back(static_cast<char>((value & 0x7FU) | 0x80U));
    value >>= 7U;
  }
  out.push_back(static_cast<char>(value));
}

bool read_varint32(const char*& cursor, const char* end, uint32_t& value) {
  value = 0U;
  uint32_t shift = 0U;
  while (cursor < end && shift <= 28U) {
    const auto byte = static_cast<uint8_t>(*cursor++);
    value |= static_cast<uint32_t>(byte & 0x7FU) << shift;
    if ((byte & 0x80U) == 0U) {
      return true;
    }
    shift += 7U;
  }
  return false;
}

template <typename MessageT>
std::string serialize_proto(const MessageT& message, const char* type_name) {
  std::string out;
  if (!message.SerializeToString(&out)) {
    throw std::runtime_error(std::string("protobuf serialization failed for ") + type_name);
  }
  return out;
}

template <typename MessageT>
MessageT parse_proto(const void* data, size_t size, const char* type_name) {
  MessageT message;
  if ((data == nullptr && size != 0) ||
      !message.ParseFromArray(data, static_cast<int>(size))) {
    throw std::runtime_error(std::string("protobuf decode failed for ") + type_name);
  }
  return message;
}

template <typename ValueT, typename ProtoT, typename ToProtoFn>
std::string serialize_proto_array(const std::vector<ValueT>& values,
                                  ToProtoFn to_proto_fn,
                                  const char* type_name) {
  std::string out;
  for (const auto& value : values) {
    const auto encoded = serialize_proto<ProtoT>(to_proto_fn(value), type_name);
    append_varint32(static_cast<uint32_t>(encoded.size()), out);
    out.append(encoded);
  }
  return out;
}

template <typename ValueT, typename ProtoT, typename FromProtoFn>
std::vector<ValueT> parse_proto_array(const void* data, size_t size,
                                      FromProtoFn from_proto_fn,
                                      const char* type_name) {
  const char* cursor = static_cast<const char*>(data);
  const char* end = cursor + size;
  std::vector<ValueT> out;
  while (cursor < end) {
    uint32_t frame_size = 0U;
    if (!read_varint32(cursor, end, frame_size) ||
        static_cast<size_t>(end - cursor) < frame_size) {
      throw std::runtime_error(
          std::string("invalid protobuf array framing for ") + type_name);
    }
    ProtoT item;
    if (!item.ParseFromArray(cursor, static_cast<int>(frame_size))) {
      throw std::runtime_error(std::string("protobuf decode failed for ") + type_name);
    }
    cursor += frame_size;
    out.push_back(from_proto_fn(item));
  }
  return out;
}

double timestamp_to_seconds(const ::pyramid::data_model::base::Timestamp& ts) {
  if (!ts.has_value()) {
    return 0.0;
  }
  return static_cast<double>(ts.value().seconds()) +
         static_cast<double>(ts.value().nanos()) / 1'000'000'000.0;
}

void set_timestamp(double seconds, ::pyramid::data_model::base::Timestamp* out) {
  if (out == nullptr) {
    return;
  }
  auto whole_seconds = static_cast<int64_t>(std::floor(seconds));
  auto nanos = static_cast<int32_t>(
      std::llround((seconds - static_cast<double>(whole_seconds)) * 1'000'000'000.0));
  if (nanos >= 1'000'000'000) {
    whole_seconds += 1;
    nanos -= 1'000'000'000;
  } else if (nanos < 0) {
    whole_seconds -= 1;
    nanos += 1'000'000'000;
  }
  auto* value = out->mutable_value();
  value->set_seconds(whole_seconds);
  value->set_nanos(nanos);
}

::pyramid::data_model::common::GeodeticPosition to_proto(
    const pyramid::domain_model::GeodeticPosition& msg) {
  ::pyramid::data_model::common::GeodeticPosition out;
  out.mutable_latitude()->set_radians(msg.latitude);
  out.mutable_longitude()->set_radians(msg.longitude);
  return out;
}

pyramid::domain_model::GeodeticPosition from_proto(
    const ::pyramid::data_model::common::GeodeticPosition& msg) {
  pyramid::domain_model::GeodeticPosition out;
  if (msg.has_latitude()) {
    out.latitude = msg.latitude().radians();
  }
  if (msg.has_longitude()) {
    out.longitude = msg.longitude().radians();
  }
  return out;
}

::pyramid::data_model::common::PolyArea to_proto(
    const pyramid::domain_model::PolyArea& msg) {
  ::pyramid::data_model::common::PolyArea out;
  for (const auto& point : msg.points) {
    *out.add_points() = to_proto(point);
  }
  return out;
}

pyramid::domain_model::PolyArea from_proto(
    const ::pyramid::data_model::common::PolyArea& msg) {
  pyramid::domain_model::PolyArea out;
  out.points.reserve(static_cast<size_t>(msg.points_size()));
  for (const auto& point : msg.points()) {
    out.points.push_back(from_proto(point));
  }
  return out;
}

::pyramid::data_model::common::Entity to_proto(
    const pyramid::domain_model::Entity& msg) {
  ::pyramid::data_model::common::Entity out;
  if (msg.update_time.has_value()) {
    set_timestamp(*msg.update_time, out.mutable_update_time());
  }
  if (!msg.id.empty()) {
    out.mutable_id()->set_value(msg.id);
  }
  if (!msg.source.empty()) {
    out.mutable_source()->set_value(msg.source);
  }
  return out;
}

pyramid::domain_model::Entity from_proto(
    const ::pyramid::data_model::common::Entity& msg) {
  pyramid::domain_model::Entity out;
  if (msg.has_update_time()) {
    out.update_time = timestamp_to_seconds(msg.update_time());
  }
  if (msg.has_id()) {
    out.id = msg.id().value();
  }
  if (msg.has_source()) {
    out.source = msg.source().value();
  }
  return out;
}

::pyramid::data_model::common::Achievement to_proto(
    const pyramid::domain_model::Achievement& msg) {
  ::pyramid::data_model::common::Achievement out;
  pyramid::domain_model::Entity base;
  base.update_time = msg.update_time;
  base.id = msg.id;
  base.source = msg.source;
  *out.mutable_base() = to_proto(base);
  out.set_status(static_cast<::pyramid::data_model::common::Progress>(
      static_cast<int>(msg.status)));
  if (msg.quality.has_value()) {
    out.mutable_quality()->set_value(*msg.quality);
  }
  out.set_achieveability(static_cast<::pyramid::data_model::common::Feasibility>(
      static_cast<int>(msg.achieveability)));
  return out;
}

pyramid::domain_model::Achievement from_proto(
    const ::pyramid::data_model::common::Achievement& msg) {
  pyramid::domain_model::Achievement out;
  if (msg.has_base()) {
    const auto base = from_proto(msg.base());
    out.update_time = base.update_time;
    out.id = base.id;
    out.source = base.source;
  }
  out.status = static_cast<pyramid::domain_model::Progress>(
      static_cast<int>(msg.status()));
  if (msg.has_quality()) {
    out.quality = msg.quality().value();
  }
  out.achieveability = static_cast<pyramid::domain_model::Feasibility>(
      static_cast<int>(msg.achieveability()));
  return out;
}

::pyramid::data_model::common::Requirement to_proto(
    const pyramid::domain_model::Requirement& msg) {
  ::pyramid::data_model::common::Requirement out;
  pyramid::domain_model::Entity entity;
  entity.update_time = msg.update_time;
  entity.id = msg.id;
  entity.source = msg.source;
  *out.mutable_base() = to_proto(entity);
  *out.mutable_status() = to_proto(msg.status);
  return out;
}

pyramid::domain_model::Requirement from_proto(
    const ::pyramid::data_model::common::Requirement& msg) {
  pyramid::domain_model::Requirement out;
  if (msg.has_base()) {
    const auto base = from_proto(msg.base());
    out.update_time = base.update_time;
    out.id = base.id;
    out.source = base.source;
  }
  if (msg.has_status()) {
    out.status = from_proto(msg.status());
  }
  return out;
}

::pyramid::data_model::common::Contraint to_proto(
    const pyramid::domain_model::Contraint& msg) {
  ::pyramid::data_model::common::Contraint out;
  out.set_name(msg.name);
  out.set_value(msg.value);
  return out;
}

pyramid::domain_model::Contraint from_proto(
    const ::pyramid::data_model::common::Contraint& msg) {
  pyramid::domain_model::Contraint out;
  out.name = msg.name();
  out.value = msg.value();
  return out;
}

::pyramid::data_model::common::Capability to_proto(
    const pyramid::domain_model::Capability& msg) {
  ::pyramid::data_model::common::Capability out;
  pyramid::domain_model::Entity entity;
  entity.update_time = msg.update_time;
  entity.id = msg.id;
  entity.source = msg.source;
  *out.mutable_base() = to_proto(entity);
  out.set_availability(msg.availability);
  out.set_name(msg.name);
  for (const auto& item : msg.contraint) {
    *out.add_contraint() = to_proto(item);
  }
  return out;
}

pyramid::domain_model::Capability from_proto(
    const ::pyramid::data_model::common::Capability& msg) {
  pyramid::domain_model::Capability out;
  if (msg.has_base()) {
    const auto base = from_proto(msg.base());
    out.update_time = base.update_time;
    out.id = base.id;
    out.source = base.source;
  }
  out.availability = msg.availability();
  out.name = msg.name();
  out.contraint.reserve(static_cast<size_t>(msg.contraint_size()));
  for (const auto& item : msg.contraint()) {
    out.contraint.push_back(from_proto(item));
  }
  return out;
}

::pyramid::data_model::common::CircleArea to_proto(
    const pyramid::domain_model::CircleArea& msg) {
  ::pyramid::data_model::common::CircleArea out;
  *out.mutable_position() = to_proto(msg.position);
  out.mutable_radius()->set_meters(msg.radius);
  return out;
}

pyramid::domain_model::CircleArea from_proto(
    const ::pyramid::data_model::common::CircleArea& msg) {
  pyramid::domain_model::CircleArea out;
  if (msg.has_position()) {
    out.position = from_proto(msg.position());
  }
  if (msg.has_radius()) {
    out.radius = msg.radius().meters();
  }
  return out;
}

::pyramid::data_model::common::Point to_proto(
    const pyramid::domain_model::Point& msg) {
  ::pyramid::data_model::common::Point out;
  *out.mutable_position() = to_proto(msg.position);
  return out;
}

pyramid::domain_model::Point from_proto(
    const ::pyramid::data_model::common::Point& msg) {
  pyramid::domain_model::Point out;
  if (msg.has_position()) {
    out.position = from_proto(msg.position());
  }
  return out;
}

::pyramid::data_model::common::Ack to_proto(
    const pyramid::domain_model::Ack& msg) {
  ::pyramid::data_model::common::Ack out;
  out.set_success(msg.success);
  return out;
}

pyramid::domain_model::Ack from_proto(
    const ::pyramid::data_model::common::Ack& msg) {
  pyramid::domain_model::Ack out;
  out.success = msg.success();
  return out;
}

::pyramid::data_model::common::Query to_proto(
    const pyramid::domain_model::Query& msg) {
  ::pyramid::data_model::common::Query out;
  for (const auto& id : msg.id) {
    out.add_id()->set_value(id);
  }
  if (msg.one_shot.has_value()) {
    out.set_one_shot(*msg.one_shot);
  }
  return out;
}

pyramid::domain_model::Query from_proto(
    const ::pyramid::data_model::common::Query& msg) {
  pyramid::domain_model::Query out;
  out.id.reserve(static_cast<size_t>(msg.id_size()));
  for (const auto& id : msg.id()) {
    out.id.push_back(id.value());
  }
  if (msg.has_one_shot()) {
    out.one_shot = msg.one_shot();
  }
  return out;
}

::pyramid::data_model::tactical::ObjectDetail to_proto(
    const pyramid::domain_model::ObjectDetail& msg) {
  ::pyramid::data_model::tactical::ObjectDetail out;
  auto* base = out.mutable_base();
  if (msg.update_time.has_value()) {
    set_timestamp(*msg.update_time, base->mutable_update_time());
  }
  if (!msg.id.empty()) {
    base->mutable_id()->set_value(msg.id);
  }
  if (!msg.entity_source.empty()) {
    base->mutable_source()->set_value(msg.entity_source);
  }
  for (const auto source : msg.source) {
    out.add_source(static_cast<::pyramid::data_model::tactical::ObjectSource>(
        static_cast<int>(source)));
  }
  *out.mutable_position() = to_proto(msg.position);
  set_timestamp(msg.creation_time, out.mutable_creation_time());
  if (msg.quality.has_value()) {
    out.mutable_quality()->set_value(*msg.quality);
  }
  if (msg.course.has_value()) {
    out.mutable_course()->set_radians(*msg.course);
  }
  if (msg.speed.has_value()) {
    out.mutable_speed()->set_meters_per_second(*msg.speed);
  }
  if (msg.length.has_value()) {
    out.mutable_length()->set_meters(*msg.length);
  }
  out.set_identity(static_cast<::pyramid::data_model::common::StandardIdentity>(
      static_cast<int>(msg.identity)));
  out.set_dimension(static_cast<::pyramid::data_model::common::BattleDimension>(
      static_cast<int>(msg.dimension)));
  return out;
}

pyramid::domain_model::ObjectDetail from_proto(
    const ::pyramid::data_model::tactical::ObjectDetail& msg) {
  pyramid::domain_model::ObjectDetail out;
  if (msg.has_base()) {
    const auto& base = msg.base();
    if (base.has_update_time()) {
      out.update_time = timestamp_to_seconds(base.update_time());
    }
    if (base.has_id()) {
      out.id = base.id().value();
    }
    if (base.has_source()) {
      out.entity_source = base.source().value();
    }
  }
  if (msg.has_position()) {
    out.position = from_proto(msg.position());
  }
  if (msg.has_creation_time()) {
    out.creation_time = timestamp_to_seconds(msg.creation_time());
  }
  if (msg.has_quality()) {
    out.quality = msg.quality().value();
  }
  if (msg.has_course()) {
    out.course = msg.course().radians();
  }
  if (msg.has_speed()) {
    out.speed = msg.speed().meters_per_second();
  }
  if (msg.has_length()) {
    out.length = msg.length().meters();
  }
  out.identity = static_cast<pyramid::domain_model::StandardIdentity>(
      static_cast<int>(msg.identity()));
  out.dimension = static_cast<pyramid::domain_model::BattleDimension>(
      static_cast<int>(msg.dimension()));
  out.source.reserve(static_cast<size_t>(msg.source_size()));
  for (const auto source : msg.source()) {
    out.source.push_back(static_cast<pyramid::domain_model::ObjectSource>(
        static_cast<int>(source)));
  }
  return out;
}

::pyramid::data_model::tactical::ObjectEvidenceRequirement to_proto(
    const pyramid::domain_model::ObjectEvidenceRequirement& msg) {
  ::pyramid::data_model::tactical::ObjectEvidenceRequirement out;
  pyramid::domain_model::Requirement base;
  base.update_time = msg.base.update_time;
  base.id = msg.base.id;
  base.source = msg.base.source;
  base.status = msg.status;
  *out.mutable_base() = to_proto(base);
  if (msg.poly_area.has_value()) {
    *out.mutable_poly_area() = to_proto(*msg.poly_area);
  } else if (msg.circle_area.has_value()) {
    *out.mutable_circle_area() = to_proto(*msg.circle_area);
  } else if (msg.point.has_value()) {
    *out.mutable_point() = to_proto(*msg.point);
  }
  out.set_policy(static_cast<::pyramid::data_model::common::DataPolicy>(
      static_cast<int>(msg.policy)));
  for (const auto dimension : msg.dimension) {
    out.add_dimension(static_cast<::pyramid::data_model::common::BattleDimension>(
        static_cast<int>(dimension)));
  }
  return out;
}

pyramid::domain_model::ObjectEvidenceRequirement from_proto(
    const ::pyramid::data_model::tactical::ObjectEvidenceRequirement& msg) {
  pyramid::domain_model::ObjectEvidenceRequirement out;
  if (msg.has_base()) {
    const auto base = from_proto(msg.base());
    out.base.update_time = base.update_time;
    out.base.id = base.id;
    out.base.source = base.source;
    out.status = base.status;
  }
  out.policy = static_cast<pyramid::domain_model::DataPolicy>(
      static_cast<int>(msg.policy()));
  out.dimension.reserve(static_cast<size_t>(msg.dimension_size()));
  for (const auto dimension : msg.dimension()) {
    out.dimension.push_back(
        static_cast<pyramid::domain_model::BattleDimension>(
            static_cast<int>(dimension)));
  }
  if (msg.has_poly_area()) {
    out.poly_area = from_proto(msg.poly_area());
  } else if (msg.has_circle_area()) {
    out.circle_area = from_proto(msg.circle_area());
  } else if (msg.has_point()) {
    out.point = from_proto(msg.point());
  }
  return out;
}

::pyramid::data_model::tactical::ObjectInterestRequirement to_proto(
    const pyramid::domain_model::ObjectInterestRequirement& msg) {
  ::pyramid::data_model::tactical::ObjectInterestRequirement out;
  pyramid::domain_model::Requirement base;
  base.update_time = msg.base.update_time;
  base.id = msg.base.id;
  base.source = msg.base.source;
  base.status = msg.status;
  *out.mutable_base() = to_proto(base);
  if (msg.source.has_value()) {
    out.set_source(static_cast<::pyramid::data_model::tactical::ObjectSource>(
        static_cast<int>(*msg.source)));
  }
  out.set_policy(static_cast<::pyramid::data_model::common::DataPolicy>(
      static_cast<int>(msg.policy)));
  if (msg.poly_area.has_value()) {
    *out.mutable_poly_area() = to_proto(*msg.poly_area);
  } else if (msg.circle_area.has_value()) {
    *out.mutable_circle_area() = to_proto(*msg.circle_area);
  } else if (msg.point.has_value()) {
    *out.mutable_point() = to_proto(*msg.point);
  }
  for (const auto dimension : msg.dimension) {
    out.add_dimension(static_cast<::pyramid::data_model::common::BattleDimension>(
        static_cast<int>(dimension)));
  }
  return out;
}

pyramid::domain_model::ObjectInterestRequirement from_proto(
    const ::pyramid::data_model::tactical::ObjectInterestRequirement& msg) {
  pyramid::domain_model::ObjectInterestRequirement out;
  if (msg.has_base()) {
    const auto base = from_proto(msg.base());
    out.base.update_time = base.update_time;
    out.base.id = base.id;
    out.base.source = base.source;
    out.status = base.status;
  }
  if (msg.has_source()) {
    out.source = static_cast<pyramid::domain_model::ObjectSource>(
        static_cast<int>(msg.source()));
  }
  out.policy = static_cast<pyramid::domain_model::DataPolicy>(
      static_cast<int>(msg.policy()));
  if (msg.has_poly_area()) {
    out.poly_area = from_proto(msg.poly_area());
  } else if (msg.has_circle_area()) {
    out.circle_area = from_proto(msg.circle_area());
  } else if (msg.has_point()) {
    out.point = from_proto(msg.point());
  }
  out.dimension.reserve(static_cast<size_t>(msg.dimension_size()));
  for (const auto dimension : msg.dimension()) {
    out.dimension.push_back(
        static_cast<pyramid::domain_model::BattleDimension>(
            static_cast<int>(dimension)));
  }
  return out;
}

::pyramid::data_model::tactical::ObjectMatch to_proto(
    const pyramid::domain_model::ObjectMatch& msg) {
  ::pyramid::data_model::tactical::ObjectMatch out;
  pyramid::domain_model::Entity entity;
  entity.update_time = msg.update_time;
  entity.id = msg.id;
  entity.source = msg.source;
  *out.mutable_base() = to_proto(entity);
  out.mutable_matching_object_id()->set_value(msg.matching_object_id);
  return out;
}

pyramid::domain_model::ObjectMatch from_proto(
    const ::pyramid::data_model::tactical::ObjectMatch& msg) {
  pyramid::domain_model::ObjectMatch out;
  if (msg.has_base()) {
    const auto base = from_proto(msg.base());
    out.update_time = base.update_time;
    out.id = base.id;
    out.source = base.source;
  }
  if (msg.has_matching_object_id()) {
    out.matching_object_id = msg.matching_object_id().value();
  }
  return out;
}

}  // namespace

std::string toBinary(const pyramid::domain_model::GeodeticPosition& msg) {
  return serialize_proto(to_proto(msg), "GeodeticPosition");
}

pyramid::domain_model::GeodeticPosition fromBinaryGeodeticPosition(const void* data,
                                                                 size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::GeodeticPosition>(
      data, size, "GeodeticPosition"));
}

std::string toBinary(const pyramid::domain_model::PolyArea& msg) {
  return serialize_proto(to_proto(msg), "PolyArea");
}

pyramid::domain_model::PolyArea fromBinaryPolyArea(const void* data, size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::PolyArea>(
      data, size, "PolyArea"));
}

std::string toBinary(const pyramid::domain_model::Achievement& msg) {
  return serialize_proto(to_proto(msg), "Achievement");
}

pyramid::domain_model::Achievement fromBinaryAchievement(const void* data,
                                                       size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::Achievement>(
      data, size, "Achievement"));
}

std::string toBinary(const pyramid::domain_model::Entity& msg) {
  return serialize_proto(to_proto(msg), "Entity");
}

pyramid::domain_model::Entity fromBinaryEntity(const void* data, size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::Entity>(
      data, size, "Entity"));
}

std::string toBinary(const pyramid::domain_model::CircleArea& msg) {
  return serialize_proto(to_proto(msg), "CircleArea");
}

pyramid::domain_model::CircleArea fromBinaryCircleArea(const void* data,
                                                     size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::CircleArea>(
      data, size, "CircleArea"));
}

std::string toBinary(const pyramid::domain_model::Point& msg) {
  return serialize_proto(to_proto(msg), "Point");
}

pyramid::domain_model::Point fromBinaryPoint(const void* data, size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::Point>(
      data, size, "Point"));
}

std::string toBinary(const pyramid::domain_model::Contraint& msg) {
  return serialize_proto(to_proto(msg), "Contraint");
}

pyramid::domain_model::Contraint fromBinaryContraint(const void* data,
                                                   size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::Contraint>(
      data, size, "Contraint"));
}

std::string toBinary(const pyramid::domain_model::Ack& msg) {
  return serialize_proto(to_proto(msg), "Ack");
}

pyramid::domain_model::Ack fromBinaryAck(const void* data, size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::Ack>(
      data, size, "Ack"));
}

std::string toBinary(const pyramid::domain_model::Query& msg) {
  return serialize_proto(to_proto(msg), "Query");
}

pyramid::domain_model::Query fromBinaryQuery(const void* data, size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::Query>(
      data, size, "Query"));
}

std::string toBinary(const pyramid::domain_model::ObjectDetail& msg) {
  return serialize_proto(to_proto(msg), "ObjectDetail");
}

pyramid::domain_model::ObjectDetail fromBinaryObjectDetail(const void* data,
                                                                 size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::tactical::ObjectDetail>(
      data, size, "ObjectDetail"));
}

std::string toBinary(const pyramid::domain_model::ObjectEvidenceRequirement& msg) {
  return serialize_proto(to_proto(msg), "ObjectEvidenceRequirement");
}

pyramid::domain_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(
    const void* data, size_t size) {
  return from_proto(
      parse_proto<::pyramid::data_model::tactical::ObjectEvidenceRequirement>(
          data, size, "ObjectEvidenceRequirement"));
}

std::string toBinary(const pyramid::domain_model::ObjectInterestRequirement& msg) {
  return serialize_proto(to_proto(msg), "ObjectInterestRequirement");
}

pyramid::domain_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(
    const void* data, size_t size) {
  return from_proto(
      parse_proto<::pyramid::data_model::tactical::ObjectInterestRequirement>(
          data, size, "ObjectInterestRequirement"));
}

std::string toBinary(const pyramid::domain_model::ObjectMatch& msg) {
  return serialize_proto(to_proto(msg), "ObjectMatch");
}

pyramid::domain_model::ObjectMatch fromBinaryObjectMatch(const void* data,
                                                       size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::tactical::ObjectMatch>(
      data, size, "ObjectMatch"));
}

std::string toBinary(const pyramid::domain_model::Capability& msg) {
  return serialize_proto(to_proto(msg), "Capability");
}

pyramid::domain_model::Capability fromBinaryCapability(const void* data,
                                                     size_t size) {
  return from_proto(parse_proto<::pyramid::data_model::common::Capability>(
      data, size, "Capability"));
}

std::string toBinary(const pyramid::domain_model::Identifier& msg) {
  ::pyramid::data_model::base::Identifier proto;
  proto.set_value(msg);
  return serialize_proto(proto, "Identifier");
}

pyramid::domain_model::Identifier fromBinaryIdentifier(const void* data,
                                                             size_t size) {
  return parse_proto<::pyramid::data_model::base::Identifier>(
      data, size, "Identifier").value();
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectDetail>& msg) {
  return serialize_proto_array<pyramid::domain_model::ObjectDetail,
                               ::pyramid::data_model::tactical::ObjectDetail>(
      msg,
      [](const pyramid::domain_model::ObjectDetail& value) {
        return to_proto(value);
      },
      "ObjectDetail");
}

std::vector<pyramid::domain_model::ObjectDetail> fromBinaryObjectDetailArray(
    const void* data, size_t size) {
  return parse_proto_array<pyramid::domain_model::ObjectDetail,
                           ::pyramid::data_model::tactical::ObjectDetail>(
      data, size,
      [](const ::pyramid::data_model::tactical::ObjectDetail& value) {
        return from_proto(value);
      },
      "ObjectDetail");
}

std::string toBinary(
    const std::vector<pyramid::domain_model::ObjectEvidenceRequirement>& msg) {
  return serialize_proto_array<pyramid::domain_model::ObjectEvidenceRequirement,
                               ::pyramid::data_model::tactical::ObjectEvidenceRequirement>(
      msg,
      [](const pyramid::domain_model::ObjectEvidenceRequirement& value) {
        return to_proto(value);
      },
      "ObjectEvidenceRequirement");
}

std::vector<pyramid::domain_model::ObjectEvidenceRequirement>
fromBinaryObjectEvidenceRequirementArray(const void* data, size_t size) {
  return parse_proto_array<pyramid::domain_model::ObjectEvidenceRequirement,
                           ::pyramid::data_model::tactical::ObjectEvidenceRequirement>(
      data, size,
      [](const ::pyramid::data_model::tactical::ObjectEvidenceRequirement& value) {
        return from_proto(value);
      },
      "ObjectEvidenceRequirement");
}

std::string toBinary(const std::vector<pyramid::domain_model::Capability>& msg) {
  return serialize_proto_array<pyramid::domain_model::Capability,
                               ::pyramid::data_model::common::Capability>(
      msg,
      [](const pyramid::domain_model::Capability& value) {
        return to_proto(value);
      },
      "Capability");
}

std::vector<pyramid::domain_model::Capability> fromBinaryCapabilityArray(
    const void* data, size_t size) {
  return parse_proto_array<pyramid::domain_model::Capability,
                           ::pyramid::data_model::common::Capability>(
      data, size,
      [](const ::pyramid::data_model::common::Capability& value) {
        return from_proto(value);
      },
      "Capability");
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectMatch>& msg) {
  return serialize_proto_array<pyramid::domain_model::ObjectMatch,
                               ::pyramid::data_model::tactical::ObjectMatch>(
      msg,
      [](const pyramid::domain_model::ObjectMatch& value) {
        return to_proto(value);
      },
      "ObjectMatch");
}

std::vector<pyramid::domain_model::ObjectMatch> fromBinaryObjectMatchArray(
    const void* data, size_t size) {
  return parse_proto_array<pyramid::domain_model::ObjectMatch,
                           ::pyramid::data_model::tactical::ObjectMatch>(
      data, size,
      [](const ::pyramid::data_model::tactical::ObjectMatch& value) {
        return from_proto(value);
      },
      "ObjectMatch");
}

std::string toBinary(
    const std::vector<pyramid::domain_model::ObjectInterestRequirement>& msg) {
  return serialize_proto_array<pyramid::domain_model::ObjectInterestRequirement,
                               ::pyramid::data_model::tactical::ObjectInterestRequirement>(
      msg,
      [](const pyramid::domain_model::ObjectInterestRequirement& value) {
        return to_proto(value);
      },
      "ObjectInterestRequirement");
}

std::vector<pyramid::domain_model::ObjectInterestRequirement>
fromBinaryObjectInterestRequirementArray(const void* data, size_t size) {
  return parse_proto_array<pyramid::domain_model::ObjectInterestRequirement,
                           ::pyramid::data_model::tactical::ObjectInterestRequirement>(
      data, size,
      [](const ::pyramid::data_model::tactical::ObjectInterestRequirement& value) {
        return from_proto(value);
      },
      "ObjectInterestRequirement");
}

}  // namespace pyramid::services::tactical_objects::protobuf_codec
