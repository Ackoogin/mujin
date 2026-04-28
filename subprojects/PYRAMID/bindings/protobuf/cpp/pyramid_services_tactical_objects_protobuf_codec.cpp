#include "pyramid_services_tactical_objects_protobuf_codec.hpp"

#include "pyramid_services_tactical_objects_protobuf_shim.h"

#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include "pyramid/data_model/base.pb.h"
#include "pyramid/data_model/common.pb.h"
#include "pyramid/data_model/tactical.pb.h"

#include <cstdlib>
#include <cstring>
#include <nlohmann/json.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::protobuf_codec {

namespace {

template <typename EncodeFn>
std::string encode_via_json(const std::string& json, EncodeFn encode_fn) {
  size_t size = 0;
  void* data = encode_fn(json.c_str(), &size);
  if (data == nullptr && size != 0) {
    throw std::runtime_error("protobuf encode failed");
  }
  std::string out;
  if (data != nullptr) {
    out.assign(static_cast<const char*>(data), size);
    std::free(data);
  }
  return out;
}

template <typename DecodeFn>
std::string decode_json(const void* data, size_t size, DecodeFn decode_fn) {
  char* json = decode_fn(data, size);
  if (json == nullptr) {
    throw std::runtime_error("protobuf decode failed");
  }
  std::string out(json);
  std::free(json);
  return out;
}

template <typename T, typename ToJsonFn, typename EncodeFn>
std::string encode_single(const T& msg, ToJsonFn to_json, EncodeFn encode_fn) {
  return encode_via_json(to_json(msg), encode_fn);
}

template <typename T, typename FromJsonFn, typename DecodeFn>
T decode_single(const void* data, size_t size, FromJsonFn from_json, DecodeFn decode_fn) {
  const std::string json = decode_json(data, size, decode_fn);
  return from_json(json, static_cast<T*>(nullptr));
}

template <typename T, typename ToJsonFn, typename EncodeFn>
std::string encode_array(const std::vector<T>& values, ToJsonFn to_json,
                         EncodeFn encode_fn) {
  nlohmann::json json = nlohmann::json::array();
  for (const auto& value : values) {
    json.push_back(nlohmann::json::parse(to_json(value)));
  }
  return encode_via_json(json.dump(), encode_fn);
}

template <typename T, typename FromJsonFn, typename DecodeFn>
std::vector<T> decode_array(const void* data, size_t size, FromJsonFn from_json,
                            DecodeFn decode_fn) {
  const std::string json = decode_json(data, size, decode_fn);
  const auto arr = nlohmann::json::parse(json);
  std::vector<T> out;
  if (arr.is_array()) {
    out.reserve(arr.size());
    for (const auto& item : arr) {
      out.push_back(from_json(item.dump(), static_cast<T*>(nullptr)));
    }
  }
  return out;
}

std::string encode_identifier_json(const pyramid::domain_model::Identifier& value) {
  return nlohmann::json(value).dump();
}

pyramid::domain_model::Identifier decode_identifier_json(const std::string& json) {
  auto parsed = nlohmann::json::parse(json);
  if (!parsed.is_string()) {
    return {};
  }
  return parsed.get<std::string>();
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

}  // namespace

std::string toBinary(const pyramid::domain_model::GeodeticPosition& msg) {
  return encode_single(msg, [](const pyramid::domain_model::GeodeticPosition& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_GeodeticPosition_to_protobuf_json);
}

pyramid::domain_model::GeodeticPosition fromBinaryGeodeticPosition(const void* data,
                                                                 size_t size) {
  return decode_single<pyramid::domain_model::GeodeticPosition>(
      data, size,
      [](const std::string& json, pyramid::domain_model::GeodeticPosition* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_GeodeticPosition_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::PolyArea& msg) {
  return encode_single(msg, [](const pyramid::domain_model::PolyArea& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_PolyArea_to_protobuf_json);
}

pyramid::domain_model::PolyArea fromBinaryPolyArea(const void* data, size_t size) {
  return decode_single<pyramid::domain_model::PolyArea>(
      data, size,
      [](const std::string& json, pyramid::domain_model::PolyArea* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_PolyArea_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::Achievement& msg) {
  return encode_single(msg, [](const pyramid::domain_model::Achievement& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Achievement_to_protobuf_json);
}

pyramid::domain_model::Achievement fromBinaryAchievement(const void* data,
                                                       size_t size) {
  return decode_single<pyramid::domain_model::Achievement>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Achievement* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Achievement_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::Entity& msg) {
  return encode_single(msg, [](const pyramid::domain_model::Entity& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Entity_to_protobuf_json);
}

pyramid::domain_model::Entity fromBinaryEntity(const void* data, size_t size) {
  return decode_single<pyramid::domain_model::Entity>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Entity* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Entity_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::CircleArea& msg) {
  return encode_single(msg, [](const pyramid::domain_model::CircleArea& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_CircleArea_to_protobuf_json);
}

pyramid::domain_model::CircleArea fromBinaryCircleArea(const void* data,
                                                     size_t size) {
  return decode_single<pyramid::domain_model::CircleArea>(
      data, size,
      [](const std::string& json, pyramid::domain_model::CircleArea* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_CircleArea_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::Point& msg) {
  return encode_single(msg, [](const pyramid::domain_model::Point& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Point_to_protobuf_json);
}

pyramid::domain_model::Point fromBinaryPoint(const void* data, size_t size) {
  return decode_single<pyramid::domain_model::Point>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Point* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Point_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::Contraint& msg) {
  return encode_single(msg, [](const pyramid::domain_model::Contraint& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Contraint_to_protobuf_json);
}

pyramid::domain_model::Contraint fromBinaryContraint(const void* data,
                                                   size_t size) {
  return decode_single<pyramid::domain_model::Contraint>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Contraint* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Contraint_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::Ack& msg) {
  return encode_single(msg, [](const pyramid::domain_model::Ack& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Ack_to_protobuf_json);
}

pyramid::domain_model::Ack fromBinaryAck(const void* data, size_t size) {
  return decode_single<pyramid::domain_model::Ack>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Ack* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Ack_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::Query& msg) {
  return encode_single(msg, [](const pyramid::domain_model::Query& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Query_to_protobuf_json);
}

pyramid::domain_model::Query fromBinaryQuery(const void* data, size_t size) {
  return decode_single<pyramid::domain_model::Query>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Query* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Query_from_protobuf_json);
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
  return encode_single(
      msg,
      [](const pyramid::domain_model::ObjectEvidenceRequirement& value) {
        return pyramid::domain_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_protobuf_json);
}

pyramid::domain_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(
    const void* data, size_t size) {
  return decode_single<pyramid::domain_model::ObjectEvidenceRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::domain_model::ObjectEvidenceRequirement* tag) {
        return pyramid::domain_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirement_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::ObjectInterestRequirement& msg) {
  return encode_single(
      msg,
      [](const pyramid::domain_model::ObjectInterestRequirement& value) {
        return pyramid::domain_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirement_to_protobuf_json);
}

pyramid::domain_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(
    const void* data, size_t size) {
  return decode_single<pyramid::domain_model::ObjectInterestRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::domain_model::ObjectInterestRequirement* tag) {
        return pyramid::domain_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirement_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::ObjectMatch& msg) {
  return encode_single(msg, [](const pyramid::domain_model::ObjectMatch& value) {
                         return pyramid::domain_model::tactical::toJson(value);
                       },
                       pyramid_services_tactical_objects_ObjectMatch_to_protobuf_json);
}

pyramid::domain_model::ObjectMatch fromBinaryObjectMatch(const void* data,
                                                       size_t size) {
  return decode_single<pyramid::domain_model::ObjectMatch>(
      data, size,
      [](const std::string& json, pyramid::domain_model::ObjectMatch* tag) {
        return pyramid::domain_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectMatch_from_protobuf_json);
}

std::string toBinary(const pyramid::domain_model::Capability& msg) {
  return encode_single(msg, [](const pyramid::domain_model::Capability& value) {
                         return pyramid::domain_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Capability_to_protobuf_json);
}

pyramid::domain_model::Capability fromBinaryCapability(const void* data,
                                                     size_t size) {
  return decode_single<pyramid::domain_model::Capability>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Capability* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Capability_from_protobuf_json);
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
  return encode_array(msg,
                      [](const pyramid::domain_model::ObjectDetail& value) {
                        return pyramid::domain_model::tactical::toJson(value);
                      },
                      pyramid_services_tactical_objects_ObjectDetailArray_to_protobuf_json);
}

std::vector<pyramid::domain_model::ObjectDetail> fromBinaryObjectDetailArray(
    const void* data, size_t size) {
  return decode_array<pyramid::domain_model::ObjectDetail>(
      data, size,
      [](const std::string& json, pyramid::domain_model::ObjectDetail* tag) {
        return pyramid::domain_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectDetailArray_from_protobuf_json);
}

std::string toBinary(
    const std::vector<pyramid::domain_model::ObjectEvidenceRequirement>& msg) {
  return encode_array(
      msg,
      [](const pyramid::domain_model::ObjectEvidenceRequirement& value) {
        return pyramid::domain_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_to_protobuf_json);
}

std::vector<pyramid::domain_model::ObjectEvidenceRequirement>
fromBinaryObjectEvidenceRequirementArray(const void* data, size_t size) {
  return decode_array<pyramid::domain_model::ObjectEvidenceRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::domain_model::ObjectEvidenceRequirement* tag) {
        return pyramid::domain_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_from_protobuf_json);
}

std::string toBinary(const std::vector<pyramid::domain_model::Capability>& msg) {
  return encode_array(msg,
                      [](const pyramid::domain_model::Capability& value) {
                        return pyramid::domain_model::common::toJson(value);
                      },
                      pyramid_services_tactical_objects_CapabilityArray_to_protobuf_json);
}

std::vector<pyramid::domain_model::Capability> fromBinaryCapabilityArray(
    const void* data, size_t size) {
  return decode_array<pyramid::domain_model::Capability>(
      data, size,
      [](const std::string& json, pyramid::domain_model::Capability* tag) {
        return pyramid::domain_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_CapabilityArray_from_protobuf_json);
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectMatch>& msg) {
  return encode_array(msg,
                      [](const pyramid::domain_model::ObjectMatch& value) {
                        return pyramid::domain_model::tactical::toJson(value);
                      },
                      pyramid_services_tactical_objects_ObjectMatchArray_to_protobuf_json);
}

std::vector<pyramid::domain_model::ObjectMatch> fromBinaryObjectMatchArray(
    const void* data, size_t size) {
  return decode_array<pyramid::domain_model::ObjectMatch>(
      data, size,
      [](const std::string& json, pyramid::domain_model::ObjectMatch* tag) {
        return pyramid::domain_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectMatchArray_from_protobuf_json);
}

std::string toBinary(
    const std::vector<pyramid::domain_model::ObjectInterestRequirement>& msg) {
  return encode_array(
      msg,
      [](const pyramid::domain_model::ObjectInterestRequirement& value) {
        return pyramid::domain_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirementArray_to_protobuf_json);
}

std::vector<pyramid::domain_model::ObjectInterestRequirement>
fromBinaryObjectInterestRequirementArray(const void* data, size_t size) {
  return decode_array<pyramid::domain_model::ObjectInterestRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::domain_model::ObjectInterestRequirement* tag) {
        return pyramid::domain_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirementArray_from_protobuf_json);
}

}  // namespace pyramid::services::tactical_objects::protobuf_codec
