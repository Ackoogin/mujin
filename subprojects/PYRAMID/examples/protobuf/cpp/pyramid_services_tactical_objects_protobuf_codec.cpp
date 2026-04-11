#include "pyramid_services_tactical_objects_protobuf_codec.hpp"

#include "pyramid_services_tactical_objects_protobuf_shim.h"

#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"

#include <cstdlib>
#include <cstring>
#include <nlohmann/json.hpp>
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

std::string encode_identifier_json(const pyramid::data_model::Identifier& value) {
  return nlohmann::json(value).dump();
}

pyramid::data_model::Identifier decode_identifier_json(const std::string& json) {
  auto parsed = nlohmann::json::parse(json);
  if (!parsed.is_string()) {
    return {};
  }
  return parsed.get<std::string>();
}

}  // namespace

std::string toBinary(const pyramid::data_model::GeodeticPosition& msg) {
  return encode_single(msg, [](const pyramid::data_model::GeodeticPosition& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_GeodeticPosition_to_protobuf_json);
}

pyramid::data_model::GeodeticPosition fromBinaryGeodeticPosition(const void* data,
                                                                 size_t size) {
  return decode_single<pyramid::data_model::GeodeticPosition>(
      data, size,
      [](const std::string& json, pyramid::data_model::GeodeticPosition* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_GeodeticPosition_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::PolyArea& msg) {
  return encode_single(msg, [](const pyramid::data_model::PolyArea& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_PolyArea_to_protobuf_json);
}

pyramid::data_model::PolyArea fromBinaryPolyArea(const void* data, size_t size) {
  return decode_single<pyramid::data_model::PolyArea>(
      data, size,
      [](const std::string& json, pyramid::data_model::PolyArea* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_PolyArea_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Achievement& msg) {
  return encode_single(msg, [](const pyramid::data_model::Achievement& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Achievement_to_protobuf_json);
}

pyramid::data_model::Achievement fromBinaryAchievement(const void* data,
                                                       size_t size) {
  return decode_single<pyramid::data_model::Achievement>(
      data, size,
      [](const std::string& json, pyramid::data_model::Achievement* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Achievement_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Entity& msg) {
  return encode_single(msg, [](const pyramid::data_model::Entity& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Entity_to_protobuf_json);
}

pyramid::data_model::Entity fromBinaryEntity(const void* data, size_t size) {
  return decode_single<pyramid::data_model::Entity>(
      data, size,
      [](const std::string& json, pyramid::data_model::Entity* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Entity_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::CircleArea& msg) {
  return encode_single(msg, [](const pyramid::data_model::CircleArea& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_CircleArea_to_protobuf_json);
}

pyramid::data_model::CircleArea fromBinaryCircleArea(const void* data,
                                                     size_t size) {
  return decode_single<pyramid::data_model::CircleArea>(
      data, size,
      [](const std::string& json, pyramid::data_model::CircleArea* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_CircleArea_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Point& msg) {
  return encode_single(msg, [](const pyramid::data_model::Point& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Point_to_protobuf_json);
}

pyramid::data_model::Point fromBinaryPoint(const void* data, size_t size) {
  return decode_single<pyramid::data_model::Point>(
      data, size,
      [](const std::string& json, pyramid::data_model::Point* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Point_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Contraint& msg) {
  return encode_single(msg, [](const pyramid::data_model::Contraint& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Contraint_to_protobuf_json);
}

pyramid::data_model::Contraint fromBinaryContraint(const void* data,
                                                   size_t size) {
  return decode_single<pyramid::data_model::Contraint>(
      data, size,
      [](const std::string& json, pyramid::data_model::Contraint* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Contraint_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Ack& msg) {
  return encode_single(msg, [](const pyramid::data_model::Ack& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Ack_to_protobuf_json);
}

pyramid::data_model::Ack fromBinaryAck(const void* data, size_t size) {
  return decode_single<pyramid::data_model::Ack>(
      data, size,
      [](const std::string& json, pyramid::data_model::Ack* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Ack_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Query& msg) {
  return encode_single(msg, [](const pyramid::data_model::Query& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Query_to_protobuf_json);
}

pyramid::data_model::Query fromBinaryQuery(const void* data, size_t size) {
  return decode_single<pyramid::data_model::Query>(
      data, size,
      [](const std::string& json, pyramid::data_model::Query* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Query_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::ObjectDetail& msg) {
  return encode_single(msg, [](const pyramid::data_model::ObjectDetail& value) {
                         return pyramid::data_model::tactical::toJson(value);
                       },
                       pyramid_services_tactical_objects_ObjectDetail_to_protobuf_json);
}

pyramid::data_model::ObjectDetail fromBinaryObjectDetail(const void* data,
                                                         size_t size) {
  return decode_single<pyramid::data_model::ObjectDetail>(
      data, size,
      [](const std::string& json, pyramid::data_model::ObjectDetail* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectDetail_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::ObjectEvidenceRequirement& msg) {
  return encode_single(
      msg,
      [](const pyramid::data_model::ObjectEvidenceRequirement& value) {
        return pyramid::data_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_protobuf_json);
}

pyramid::data_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(
    const void* data, size_t size) {
  return decode_single<pyramid::data_model::ObjectEvidenceRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::data_model::ObjectEvidenceRequirement* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirement_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::ObjectInterestRequirement& msg) {
  return encode_single(
      msg,
      [](const pyramid::data_model::ObjectInterestRequirement& value) {
        return pyramid::data_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirement_to_protobuf_json);
}

pyramid::data_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(
    const void* data, size_t size) {
  return decode_single<pyramid::data_model::ObjectInterestRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::data_model::ObjectInterestRequirement* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirement_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::ObjectMatch& msg) {
  return encode_single(msg, [](const pyramid::data_model::ObjectMatch& value) {
                         return pyramid::data_model::tactical::toJson(value);
                       },
                       pyramid_services_tactical_objects_ObjectMatch_to_protobuf_json);
}

pyramid::data_model::ObjectMatch fromBinaryObjectMatch(const void* data,
                                                       size_t size) {
  return decode_single<pyramid::data_model::ObjectMatch>(
      data, size,
      [](const std::string& json, pyramid::data_model::ObjectMatch* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectMatch_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Capability& msg) {
  return encode_single(msg, [](const pyramid::data_model::Capability& value) {
                         return pyramid::data_model::common::toJson(value);
                       },
                       pyramid_services_tactical_objects_Capability_to_protobuf_json);
}

pyramid::data_model::Capability fromBinaryCapability(const void* data,
                                                     size_t size) {
  return decode_single<pyramid::data_model::Capability>(
      data, size,
      [](const std::string& json, pyramid::data_model::Capability* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_Capability_from_protobuf_json);
}

std::string toBinary(const pyramid::data_model::Identifier& msg) {
  return encode_via_json(
      encode_identifier_json(msg),
      pyramid_services_tactical_objects_Identifier_to_protobuf_json);
}

pyramid::data_model::Identifier fromBinaryIdentifier(const void* data,
                                                     size_t size) {
  return decode_identifier_json(decode_json(
      data, size, pyramid_services_tactical_objects_Identifier_from_protobuf_json));
}

std::string toBinary(const std::vector<pyramid::data_model::ObjectDetail>& msg) {
  return encode_array(msg,
                      [](const pyramid::data_model::ObjectDetail& value) {
                        return pyramid::data_model::tactical::toJson(value);
                      },
                      pyramid_services_tactical_objects_ObjectDetailArray_to_protobuf_json);
}

std::vector<pyramid::data_model::ObjectDetail> fromBinaryObjectDetailArray(
    const void* data, size_t size) {
  return decode_array<pyramid::data_model::ObjectDetail>(
      data, size,
      [](const std::string& json, pyramid::data_model::ObjectDetail* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectDetailArray_from_protobuf_json);
}

std::string toBinary(
    const std::vector<pyramid::data_model::ObjectEvidenceRequirement>& msg) {
  return encode_array(
      msg,
      [](const pyramid::data_model::ObjectEvidenceRequirement& value) {
        return pyramid::data_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_to_protobuf_json);
}

std::vector<pyramid::data_model::ObjectEvidenceRequirement>
fromBinaryObjectEvidenceRequirementArray(const void* data, size_t size) {
  return decode_array<pyramid::data_model::ObjectEvidenceRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::data_model::ObjectEvidenceRequirement* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_from_protobuf_json);
}

std::string toBinary(const std::vector<pyramid::data_model::Capability>& msg) {
  return encode_array(msg,
                      [](const pyramid::data_model::Capability& value) {
                        return pyramid::data_model::common::toJson(value);
                      },
                      pyramid_services_tactical_objects_CapabilityArray_to_protobuf_json);
}

std::vector<pyramid::data_model::Capability> fromBinaryCapabilityArray(
    const void* data, size_t size) {
  return decode_array<pyramid::data_model::Capability>(
      data, size,
      [](const std::string& json, pyramid::data_model::Capability* tag) {
        return pyramid::data_model::common::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_CapabilityArray_from_protobuf_json);
}

std::string toBinary(const std::vector<pyramid::data_model::ObjectMatch>& msg) {
  return encode_array(msg,
                      [](const pyramid::data_model::ObjectMatch& value) {
                        return pyramid::data_model::tactical::toJson(value);
                      },
                      pyramid_services_tactical_objects_ObjectMatchArray_to_protobuf_json);
}

std::vector<pyramid::data_model::ObjectMatch> fromBinaryObjectMatchArray(
    const void* data, size_t size) {
  return decode_array<pyramid::data_model::ObjectMatch>(
      data, size,
      [](const std::string& json, pyramid::data_model::ObjectMatch* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectMatchArray_from_protobuf_json);
}

std::string toBinary(
    const std::vector<pyramid::data_model::ObjectInterestRequirement>& msg) {
  return encode_array(
      msg,
      [](const pyramid::data_model::ObjectInterestRequirement& value) {
        return pyramid::data_model::tactical::toJson(value);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirementArray_to_protobuf_json);
}

std::vector<pyramid::data_model::ObjectInterestRequirement>
fromBinaryObjectInterestRequirementArray(const void* data, size_t size) {
  return decode_array<pyramid::data_model::ObjectInterestRequirement>(
      data, size,
      [](const std::string& json,
         pyramid::data_model::ObjectInterestRequirement* tag) {
        return pyramid::data_model::tactical::fromJson(json, tag);
      },
      pyramid_services_tactical_objects_ObjectInterestRequirementArray_from_protobuf_json);
}

}  // namespace pyramid::services::tactical_objects::protobuf_codec
