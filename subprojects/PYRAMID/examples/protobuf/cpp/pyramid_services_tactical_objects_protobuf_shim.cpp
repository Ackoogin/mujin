#include "pyramid_services_tactical_objects_protobuf_shim.h"

#include "pyramid/data_model/base.pb.h"
#include "pyramid/data_model/common.pb.h"
#include "pyramid/data_model/tactical.pb.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace {

using json = nlohmann::json;

using BaseTimestamp = ::pyramid::data_model::base::Timestamp;
using BaseIdentifier = ::pyramid::data_model::base::Identifier;
using BaseAngle = ::pyramid::data_model::base::Angle;
using BaseLength = ::pyramid::data_model::base::Length;
using BaseSpeed = ::pyramid::data_model::base::Speed;
using BasePercentage = ::pyramid::data_model::base::Percentage;

using CommonAck = ::pyramid::data_model::common::Ack;
using CommonAchievement = ::pyramid::data_model::common::Achievement;
using CommonBattleDimension = ::pyramid::data_model::common::BattleDimension;
using CommonCapability = ::pyramid::data_model::common::Capability;
using CommonCircleArea = ::pyramid::data_model::common::CircleArea;
using CommonConstraint = ::pyramid::data_model::common::Contraint;
using CommonDataPolicy = ::pyramid::data_model::common::DataPolicy;
using CommonEntity = ::pyramid::data_model::common::Entity;
using CommonFeasibility = ::pyramid::data_model::common::Feasibility;
using CommonGeodeticPosition = ::pyramid::data_model::common::GeodeticPosition;
using CommonPoint = ::pyramid::data_model::common::Point;
using CommonPolyArea = ::pyramid::data_model::common::PolyArea;
using CommonProgress = ::pyramid::data_model::common::Progress;
using CommonQuery = ::pyramid::data_model::common::Query;
using CommonRequirement = ::pyramid::data_model::common::Requirement;
using CommonStandardIdentity = ::pyramid::data_model::common::StandardIdentity;

using TacticalObjectDetail = ::pyramid::data_model::tactical::ObjectDetail;
using TacticalObjectEvidenceRequirement =
    ::pyramid::data_model::tactical::ObjectEvidenceRequirement;
using TacticalObjectInterestRequirement =
    ::pyramid::data_model::tactical::ObjectInterestRequirement;
using TacticalObjectMatch = ::pyramid::data_model::tactical::ObjectMatch;
using TacticalObjectSource = ::pyramid::data_model::tactical::ObjectSource;

template <typename MessageT>
std::string serializeProto(const MessageT& message) {
  std::string out;
  if (!message.SerializeToString(&out)) {
    throw std::runtime_error("protobuf serialization failed");
  }
  return out;
}

template <typename MessageT>
MessageT parseProto(const void* data, size_t size, const char* type_name) {
  if (data == nullptr && size != 0) {
    throw std::runtime_error(std::string("missing protobuf payload for ") + type_name);
  }

  MessageT message;
  if (!message.ParseFromArray(data, static_cast<int>(size))) {
    throw std::runtime_error(std::string("invalid protobuf payload for ") + type_name);
  }
  return message;
}

void appendVarint32(uint32_t value, std::string& out) {
  while (value >= 0x80U) {
    out.push_back(static_cast<char>((value & 0x7FU) | 0x80U));
    value >>= 7U;
  }
  out.push_back(static_cast<char>(value));
}

bool readVarint32(const char*& cursor, const char* end, uint32_t& value) {
  value = 0;
  uint32_t shift = 0;
  while (cursor < end && shift <= 28U) {
    const uint8_t byte = static_cast<uint8_t>(*cursor++);
    value |= static_cast<uint32_t>(byte & 0x7FU) << shift;
    if ((byte & 0x80U) == 0U) {
      return true;
    }
    shift += 7U;
  }
  return false;
}

char* copyCString(const std::string& value) {
  char* out = static_cast<char*>(std::malloc(value.size() + 1));
  if (out == nullptr) {
    return nullptr;
  }
  std::memcpy(out, value.c_str(), value.size() + 1);
  return out;
}

void* copyBytes(const std::string& value, size_t* size_out) {
  if (size_out != nullptr) {
    *size_out = value.size();
  }
  if (value.empty()) {
    return nullptr;
  }
  void* out = std::malloc(value.size());
  if (out == nullptr) {
    return nullptr;
  }
  std::memcpy(out, value.data(), value.size());
  return out;
}

double timestampToSeconds(const BaseTimestamp& ts) {
  if (!ts.has_value()) {
    return 0.0;
  }
  const auto& value = ts.value();
  return static_cast<double>(value.seconds()) +
         static_cast<double>(value.nanos()) / 1'000'000'000.0;
}

void setTimestamp(double seconds, BaseTimestamp* out) {
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

void setIdentifier(const std::string& value, BaseIdentifier* out) {
  if (out != nullptr) {
    out->set_value(value);
  }
}

void setAngle(double value, BaseAngle* out) {
  if (out != nullptr) {
    out->set_radians(value);
  }
}

void setLength(double value, BaseLength* out) {
  if (out != nullptr) {
    out->set_meters(value);
  }
}

void setSpeed(double value, BaseSpeed* out) {
  if (out != nullptr) {
    out->set_meters_per_second(value);
  }
}

void setPercentage(double value, BasePercentage* out) {
  if (out != nullptr) {
    out->set_value(value);
  }
}

std::string identifierValue(const BaseIdentifier& value) {
  return value.value();
}

double angleValue(const BaseAngle& value) {
  return value.radians();
}

double lengthValue(const BaseLength& value) {
  return value.meters();
}

double speedValue(const BaseSpeed& value) {
  return value.meters_per_second();
}

double percentageValue(const BasePercentage& value) {
  return value.value();
}

CommonProgress parseProgress(const std::string& value) {
  if (value == "PROGRESS_NOT_STARTED") return CommonProgress::PROGRESS_NOT_STARTED;
  if (value == "PROGRESS_IN_PROGRESS") return CommonProgress::PROGRESS_IN_PROGRESS;
  if (value == "PROGRESS_COMPLETED") return CommonProgress::PROGRESS_COMPLETED;
  if (value == "PROGRESS_CANCELLED") return CommonProgress::PROGRESS_CANCELLED;
  if (value == "PROGRESS_FAILED") return CommonProgress::PROGRESS_FAILED;
  return CommonProgress::PROGRESS_UNSPECIFIED;
}

const char* progressString(CommonProgress value) {
  switch (value) {
    case CommonProgress::PROGRESS_NOT_STARTED: return "PROGRESS_NOT_STARTED";
    case CommonProgress::PROGRESS_IN_PROGRESS: return "PROGRESS_IN_PROGRESS";
    case CommonProgress::PROGRESS_COMPLETED: return "PROGRESS_COMPLETED";
    case CommonProgress::PROGRESS_CANCELLED: return "PROGRESS_CANCELLED";
    case CommonProgress::PROGRESS_FAILED: return "PROGRESS_FAILED";
    default: return "PROGRESS_UNSPECIFIED";
  }
}

CommonFeasibility parseFeasibility(const std::string& value) {
  if (value == "FEASIBILITY_FEASIBLE") return CommonFeasibility::FEASIBILITY_FEASIBLE;
  if (value == "FEASIBILITY_NOT_FEASIBLE") {
    return CommonFeasibility::FEASIBILITY_NOT_FEASIBLE;
  }
  if (value == "FEASIBILITY_PARTIALLY_FEASIBLE") {
    return CommonFeasibility::FEASIBILITY_PARTIALLY_FEASIBLE;
  }
  if (value == "FEASIBILITY_PENDING") return CommonFeasibility::FEASIBILITY_PENDING;
  return CommonFeasibility::FEASIBILITY_UNSPECIFIED;
}

const char* feasibilityString(CommonFeasibility value) {
  switch (value) {
    case CommonFeasibility::FEASIBILITY_FEASIBLE: return "FEASIBILITY_FEASIBLE";
    case CommonFeasibility::FEASIBILITY_NOT_FEASIBLE:
      return "FEASIBILITY_NOT_FEASIBLE";
    case CommonFeasibility::FEASIBILITY_PARTIALLY_FEASIBLE:
      return "FEASIBILITY_PARTIALLY_FEASIBLE";
    case CommonFeasibility::FEASIBILITY_PENDING: return "FEASIBILITY_PENDING";
    default: return "FEASIBILITY_UNSPECIFIED";
  }
}

CommonStandardIdentity parseStandardIdentity(const std::string& value) {
  if (value == "STANDARD_IDENTITY_UNKNOWN") return CommonStandardIdentity::STANDARD_IDENTITY_UNKNOWN;
  if (value == "STANDARD_IDENTITY_FRIENDLY") return CommonStandardIdentity::STANDARD_IDENTITY_FRIENDLY;
  if (value == "STANDARD_IDENTITY_HOSTILE") return CommonStandardIdentity::STANDARD_IDENTITY_HOSTILE;
  if (value == "STANDARD_IDENTITY_SUSPECT") return CommonStandardIdentity::STANDARD_IDENTITY_SUSPECT;
  if (value == "STANDARD_IDENTITY_NEUTRAL") return CommonStandardIdentity::STANDARD_IDENTITY_NEUTRAL;
  if (value == "STANDARD_IDENTITY_PENDING") return CommonStandardIdentity::STANDARD_IDENTITY_PENDING;
  if (value == "STANDARD_IDENTITY_JOKER") return CommonStandardIdentity::STANDARD_IDENTITY_JOKER;
  if (value == "STANDARD_IDENTITY_FAKER") return CommonStandardIdentity::STANDARD_IDENTITY_FAKER;
  if (value == "STANDARD_IDENTITY_ASSUMED_FRIENDLY") {
    return CommonStandardIdentity::STANDARD_IDENTITY_ASSUMED_FRIENDLY;
  }
  return CommonStandardIdentity::STANDARD_IDENTITY_UNSPECIFIED;
}

const char* standardIdentityString(CommonStandardIdentity value) {
  switch (value) {
    case CommonStandardIdentity::STANDARD_IDENTITY_UNKNOWN:
      return "STANDARD_IDENTITY_UNKNOWN";
    case CommonStandardIdentity::STANDARD_IDENTITY_FRIENDLY:
      return "STANDARD_IDENTITY_FRIENDLY";
    case CommonStandardIdentity::STANDARD_IDENTITY_HOSTILE:
      return "STANDARD_IDENTITY_HOSTILE";
    case CommonStandardIdentity::STANDARD_IDENTITY_SUSPECT:
      return "STANDARD_IDENTITY_SUSPECT";
    case CommonStandardIdentity::STANDARD_IDENTITY_NEUTRAL:
      return "STANDARD_IDENTITY_NEUTRAL";
    case CommonStandardIdentity::STANDARD_IDENTITY_PENDING:
      return "STANDARD_IDENTITY_PENDING";
    case CommonStandardIdentity::STANDARD_IDENTITY_JOKER:
      return "STANDARD_IDENTITY_JOKER";
    case CommonStandardIdentity::STANDARD_IDENTITY_FAKER:
      return "STANDARD_IDENTITY_FAKER";
    case CommonStandardIdentity::STANDARD_IDENTITY_ASSUMED_FRIENDLY:
      return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
    default: return "STANDARD_IDENTITY_UNSPECIFIED";
  }
}

CommonBattleDimension parseBattleDimension(const std::string& value) {
  if (value == "BATTLE_DIMENSION_GROUND") return CommonBattleDimension::BATTLE_DIMENSION_GROUND;
  if (value == "BATTLE_DIMENSION_SUBSURFACE") {
    return CommonBattleDimension::BATTLE_DIMENSION_SUBSURFACE;
  }
  if (value == "BATTLE_DIMENSION_SEA_SURFACE") {
    return CommonBattleDimension::BATTLE_DIMENSION_SEA_SURFACE;
  }
  if (value == "BATTLE_DIMENSION_AIR") return CommonBattleDimension::BATTLE_DIMENSION_AIR;
  if (value == "BATTLE_DIMENSION_UNKNOWN") return CommonBattleDimension::BATTLE_DIMENSION_UNKNOWN;
  return CommonBattleDimension::BATTLE_DIMENSION_UNSPECIFIED;
}

const char* battleDimensionString(CommonBattleDimension value) {
  switch (value) {
    case CommonBattleDimension::BATTLE_DIMENSION_GROUND:
      return "BATTLE_DIMENSION_GROUND";
    case CommonBattleDimension::BATTLE_DIMENSION_SUBSURFACE:
      return "BATTLE_DIMENSION_SUBSURFACE";
    case CommonBattleDimension::BATTLE_DIMENSION_SEA_SURFACE:
      return "BATTLE_DIMENSION_SEA_SURFACE";
    case CommonBattleDimension::BATTLE_DIMENSION_AIR:
      return "BATTLE_DIMENSION_AIR";
    case CommonBattleDimension::BATTLE_DIMENSION_UNKNOWN:
      return "BATTLE_DIMENSION_UNKNOWN";
    default: return "BATTLE_DIMENSION_UNSPECIFIED";
  }
}

CommonDataPolicy parseDataPolicy(const std::string& value) {
  if (value == "DATA_POLICY_QUERY") return CommonDataPolicy::DATA_POLICY_QUERY;
  if (value == "DATA_POLICY_OBTAIN") return CommonDataPolicy::DATA_POLICY_OBTAIN;
  return CommonDataPolicy::DATA_POLICY_UNSPECIFIED;
}

const char* dataPolicyString(CommonDataPolicy value) {
  switch (value) {
    case CommonDataPolicy::DATA_POLICY_QUERY: return "DATA_POLICY_QUERY";
    case CommonDataPolicy::DATA_POLICY_OBTAIN: return "DATA_POLICY_OBTAIN";
    default: return "DATA_POLICY_UNSPECIFIED";
  }
}

TacticalObjectSource parseObjectSource(const std::string& value) {
  if (value == "OBJECT_SOURCE_RADAR") return TacticalObjectSource::OBJECT_SOURCE_RADAR;
  if (value == "OBJECT_SOURCE_LOCAL") return TacticalObjectSource::OBJECT_SOURCE_LOCAL;
  return TacticalObjectSource::OBJECT_SOURCE_UNSPECIFIED;
}

const char* objectSourceString(TacticalObjectSource value) {
  switch (value) {
    case TacticalObjectSource::OBJECT_SOURCE_RADAR: return "OBJECT_SOURCE_RADAR";
    case TacticalObjectSource::OBJECT_SOURCE_LOCAL: return "OBJECT_SOURCE_LOCAL";
    default: return "OBJECT_SOURCE_UNSPECIFIED";
  }
}

CommonGeodeticPosition parseGeodeticPosition(const json& j) {
  CommonGeodeticPosition out;
  setAngle(j.value("latitude", 0.0), out.mutable_latitude());
  setAngle(j.value("longitude", 0.0), out.mutable_longitude());
  return out;
}

json toJson(const CommonGeodeticPosition& msg) {
  json out = json::object();
  if (msg.has_latitude()) out["latitude"] = angleValue(msg.latitude());
  if (msg.has_longitude()) out["longitude"] = angleValue(msg.longitude());
  return out;
}

CommonEntity parseEntity(const json& j) {
  CommonEntity out;
  if (j.contains("update_time")) {
    setTimestamp(j["update_time"].get<double>(), out.mutable_update_time());
  }
  if (j.contains("id")) {
    setIdentifier(j["id"].get<std::string>(), out.mutable_id());
  }
  if (j.contains("source")) {
    setIdentifier(j["source"].get<std::string>(), out.mutable_source());
  }
  return out;
}

json toJson(const CommonEntity& msg) {
  json out = json::object();
  if (msg.has_update_time()) out["update_time"] = timestampToSeconds(msg.update_time());
  if (msg.has_id()) out["id"] = identifierValue(msg.id());
  if (msg.has_source()) out["source"] = identifierValue(msg.source());
  return out;
}

CommonAchievement parseAchievement(const json& j) {
  CommonAchievement out;
  auto* base = out.mutable_base();
  if (j.contains("update_time")) {
    setTimestamp(j["update_time"].get<double>(), base->mutable_update_time());
  }
  if (j.contains("id")) {
    setIdentifier(j["id"].get<std::string>(), base->mutable_id());
  }
  if (j.contains("source")) {
    setIdentifier(j["source"].get<std::string>(), base->mutable_source());
  }
  out.set_status(parseProgress(j.value("status", "PROGRESS_UNSPECIFIED")));
  if (j.contains("quality")) {
    setPercentage(j["quality"].get<double>(), out.mutable_quality());
  }
  out.set_achieveability(
      parseFeasibility(j.value("achieveability", "FEASIBILITY_UNSPECIFIED")));
  return out;
}

json toJson(const CommonAchievement& msg) {
  json out = json::object();
  if (msg.has_base()) {
    const auto& base = msg.base();
    if (base.has_update_time()) out["update_time"] = timestampToSeconds(base.update_time());
    if (base.has_id()) out["id"] = identifierValue(base.id());
    if (base.has_source()) out["source"] = identifierValue(base.source());
  }
  out["status"] = progressString(msg.status());
  if (msg.has_quality()) out["quality"] = percentageValue(msg.quality());
  out["achieveability"] = feasibilityString(msg.achieveability());
  return out;
}

CommonConstraint parseConstraint(const json& j) {
  CommonConstraint out;
  out.set_name(j.value("name", ""));
  out.set_value(j.value("value", 0));
  return out;
}

json toJson(const CommonConstraint& msg) {
  return json{{"name", msg.name()}, {"value", msg.value()}};
}

CommonCapability parseCapability(const json& j) {
  CommonCapability out;
  auto* base = out.mutable_base();
  if (j.contains("update_time")) {
    setTimestamp(j["update_time"].get<double>(), base->mutable_update_time());
  }
  if (j.contains("id")) {
    setIdentifier(j["id"].get<std::string>(), base->mutable_id());
  }
  if (j.contains("source")) {
    setIdentifier(j["source"].get<std::string>(), base->mutable_source());
  }
  out.set_availability(j.value("availability", false));
  out.set_name(j.value("name", ""));
  if (j.contains("contraint") && j["contraint"].is_array()) {
    for (const auto& item : j["contraint"]) {
      *out.add_contraint() = parseConstraint(item);
    }
  }
  return out;
}

json toJson(const CommonCapability& msg) {
  json out = json::object();
  if (msg.has_base()) {
    const auto& base = msg.base();
    if (base.has_update_time()) out["update_time"] = timestampToSeconds(base.update_time());
    if (base.has_id()) out["id"] = identifierValue(base.id());
    if (base.has_source()) out["source"] = identifierValue(base.source());
  }
  out["availability"] = msg.availability();
  out["name"] = msg.name();
  json constraints = json::array();
  for (const auto& item : msg.contraint()) {
    constraints.push_back(toJson(item));
  }
  out["contraint"] = constraints;
  return out;
}

CommonPolyArea parsePolyArea(const json& j) {
  CommonPolyArea out;
  if (j.contains("points") && j["points"].is_array()) {
    for (const auto& item : j["points"]) {
      *out.add_points() = parseGeodeticPosition(item);
    }
  }
  return out;
}

json toJson(const CommonPolyArea& msg) {
  json out = json::object();
  json points = json::array();
  for (const auto& item : msg.points()) {
    points.push_back(toJson(item));
  }
  out["points"] = points;
  return out;
}

CommonCircleArea parseCircleArea(const json& j) {
  CommonCircleArea out;
  if (j.contains("position")) {
    *out.mutable_position() = parseGeodeticPosition(j["position"]);
  }
  setLength(j.value("radius", 0.0), out.mutable_radius());
  return out;
}

json toJson(const CommonCircleArea& msg) {
  json out = json::object();
  if (msg.has_position()) out["position"] = toJson(msg.position());
  if (msg.has_radius()) out["radius"] = lengthValue(msg.radius());
  return out;
}

CommonPoint parsePoint(const json& j) {
  CommonPoint out;
  if (j.contains("position")) {
    *out.mutable_position() = parseGeodeticPosition(j["position"]);
  }
  return out;
}

json toJson(const CommonPoint& msg) {
  json out = json::object();
  if (msg.has_position()) out["position"] = toJson(msg.position());
  return out;
}

CommonAck parseAck(const json& j) {
  CommonAck out;
  out.set_success(j.value("success", false));
  return out;
}

json toJson(const CommonAck& msg) {
  return json{{"success", msg.success()}};
}

CommonQuery parseQuery(const json& j) {
  CommonQuery out;
  if (j.contains("id") && j["id"].is_array()) {
    for (const auto& item : j["id"]) {
      setIdentifier(item.get<std::string>(), out.add_id());
    }
  }
  if (j.contains("one_shot")) {
    out.set_one_shot(j["one_shot"].get<bool>());
  }
  return out;
}

json toJson(const CommonQuery& msg) {
  json out = json::object();
  json ids = json::array();
  for (const auto& item : msg.id()) {
    ids.push_back(identifierValue(item));
  }
  out["id"] = ids;
  if (msg.has_one_shot()) out["one_shot"] = msg.one_shot();
  return out;
}

TacticalObjectDetail parseObjectDetail(const json& j) {
  TacticalObjectDetail out;
  auto* base = out.mutable_base();
  if (j.contains("update_time")) {
    setTimestamp(j["update_time"].get<double>(), base->mutable_update_time());
  }
  if (j.contains("id")) {
    setIdentifier(j["id"].get<std::string>(), base->mutable_id());
  }
  if (j.contains("entity_source")) {
    setIdentifier(j["entity_source"].get<std::string>(), base->mutable_source());
  }
  if (j.contains("source") && j["source"].is_array()) {
    for (const auto& item : j["source"]) {
      out.add_source(parseObjectSource(item.get<std::string>()));
    }
  }
  if (j.contains("position")) {
    *out.mutable_position() = parseGeodeticPosition(j["position"]);
  }
  if (j.contains("creation_time")) {
    setTimestamp(j["creation_time"].get<double>(), out.mutable_creation_time());
  }
  if (j.contains("quality")) {
    setPercentage(j["quality"].get<double>(), out.mutable_quality());
  }
  if (j.contains("course")) {
    setAngle(j["course"].get<double>(), out.mutable_course());
  }
  if (j.contains("speed")) {
    setSpeed(j["speed"].get<double>(), out.mutable_speed());
  }
  if (j.contains("length")) {
    setLength(j["length"].get<double>(), out.mutable_length());
  }
  out.set_identity(parseStandardIdentity(
      j.value("identity", "STANDARD_IDENTITY_UNSPECIFIED")));
  out.set_dimension(parseBattleDimension(
      j.value("dimension", "BATTLE_DIMENSION_UNSPECIFIED")));
  return out;
}

json toJson(const TacticalObjectDetail& msg) {
  json out = json::object();
  if (msg.has_base()) {
    const auto& base = msg.base();
    if (base.has_update_time()) out["update_time"] = timestampToSeconds(base.update_time());
    if (base.has_id()) out["id"] = identifierValue(base.id());
    if (base.has_source()) out["entity_source"] = identifierValue(base.source());
  }
  json sources = json::array();
  for (const auto& item : msg.source()) {
    sources.push_back(objectSourceString(static_cast<TacticalObjectSource>(item)));
  }
  out["source"] = sources;
  if (msg.has_position()) out["position"] = toJson(msg.position());
  if (msg.has_creation_time()) {
    out["creation_time"] = timestampToSeconds(msg.creation_time());
  }
  if (msg.has_quality()) out["quality"] = percentageValue(msg.quality());
  if (msg.has_course()) out["course"] = angleValue(msg.course());
  if (msg.has_speed()) out["speed"] = speedValue(msg.speed());
  if (msg.has_length()) out["length"] = lengthValue(msg.length());
  out["identity"] = standardIdentityString(msg.identity());
  out["dimension"] = battleDimensionString(msg.dimension());
  return out;
}

TacticalObjectEvidenceRequirement parseObjectEvidenceRequirement(const json& j) {
  TacticalObjectEvidenceRequirement out;
  auto* requirement = out.mutable_base();
  if (j.contains("base")) {
    *requirement->mutable_base() = parseEntity(j["base"]);
  }
  if (j.contains("status")) {
    *requirement->mutable_status() = parseAchievement(j["status"]);
  }
  out.set_policy(parseDataPolicy(
      j.value("policy", "DATA_POLICY_UNSPECIFIED")));
  if (j.contains("dimension") && j["dimension"].is_array()) {
    for (const auto& item : j["dimension"]) {
      out.add_dimension(parseBattleDimension(item.get<std::string>()));
    }
  }
  if (j.contains("poly_area")) {
    *out.mutable_poly_area() = parsePolyArea(j["poly_area"]);
  } else if (j.contains("circle_area")) {
    *out.mutable_circle_area() = parseCircleArea(j["circle_area"]);
  } else if (j.contains("point")) {
    *out.mutable_point() = parsePoint(j["point"]);
  }
  return out;
}

json toJson(const TacticalObjectEvidenceRequirement& msg) {
  json out = json::object();
  if (msg.has_base()) {
    if (msg.base().has_base()) out["base"] = toJson(msg.base().base());
    if (msg.base().has_status()) out["status"] = toJson(msg.base().status());
  }
  out["policy"] = dataPolicyString(msg.policy());
  json dimensions = json::array();
  for (const auto& item : msg.dimension()) {
    dimensions.push_back(
        battleDimensionString(static_cast<CommonBattleDimension>(item)));
  }
  out["dimension"] = dimensions;
  if (msg.has_poly_area()) {
    out["poly_area"] = toJson(msg.poly_area());
  } else if (msg.has_circle_area()) {
    out["circle_area"] = toJson(msg.circle_area());
  } else if (msg.has_point()) {
    out["point"] = toJson(msg.point());
  }
  return out;
}

TacticalObjectInterestRequirement parseObjectInterestRequirement(const json& j) {
  TacticalObjectInterestRequirement out;
  auto* requirement = out.mutable_base();
  if (j.contains("base")) {
    *requirement->mutable_base() = parseEntity(j["base"]);
  }
  if (j.contains("status")) {
    *requirement->mutable_status() = parseAchievement(j["status"]);
  }
  if (j.contains("source")) {
    out.set_source(parseObjectSource(j["source"].get<std::string>()));
  }
  out.set_policy(parseDataPolicy(
      j.value("policy", "DATA_POLICY_UNSPECIFIED")));
  if (j.contains("dimension") && j["dimension"].is_array()) {
    for (const auto& item : j["dimension"]) {
      out.add_dimension(parseBattleDimension(item.get<std::string>()));
    }
  }
  if (j.contains("poly_area")) {
    *out.mutable_poly_area() = parsePolyArea(j["poly_area"]);
  } else if (j.contains("circle_area")) {
    *out.mutable_circle_area() = parseCircleArea(j["circle_area"]);
  } else if (j.contains("point")) {
    *out.mutable_point() = parsePoint(j["point"]);
  }
  return out;
}

json toJson(const TacticalObjectInterestRequirement& msg) {
  json out = json::object();
  if (msg.has_base()) {
    if (msg.base().has_base()) out["base"] = toJson(msg.base().base());
    if (msg.base().has_status()) out["status"] = toJson(msg.base().status());
  }
  if (msg.has_source()) out["source"] = objectSourceString(msg.source());
  out["policy"] = dataPolicyString(msg.policy());
  json dimensions = json::array();
  for (const auto& item : msg.dimension()) {
    dimensions.push_back(
        battleDimensionString(static_cast<CommonBattleDimension>(item)));
  }
  out["dimension"] = dimensions;
  if (msg.has_poly_area()) {
    out["poly_area"] = toJson(msg.poly_area());
  } else if (msg.has_circle_area()) {
    out["circle_area"] = toJson(msg.circle_area());
  } else if (msg.has_point()) {
    out["point"] = toJson(msg.point());
  }
  return out;
}

TacticalObjectMatch parseObjectMatch(const json& j) {
  TacticalObjectMatch out;
  auto* base = out.mutable_base();
  if (j.contains("update_time")) {
    setTimestamp(j["update_time"].get<double>(), base->mutable_update_time());
  }
  if (j.contains("id")) {
    setIdentifier(j["id"].get<std::string>(), base->mutable_id());
  }
  if (j.contains("source")) {
    setIdentifier(j["source"].get<std::string>(), base->mutable_source());
  }
  if (j.contains("matching_object_id")) {
    setIdentifier(j["matching_object_id"].get<std::string>(),
                  out.mutable_matching_object_id());
  }
  return out;
}

json toJson(const TacticalObjectMatch& msg) {
  json out = json::object();
  if (msg.has_base()) {
    const auto& base = msg.base();
    if (base.has_update_time()) out["update_time"] = timestampToSeconds(base.update_time());
    if (base.has_id()) out["id"] = identifierValue(base.id());
    if (base.has_source()) out["source"] = identifierValue(base.source());
  }
  if (msg.has_matching_object_id()) {
    out["matching_object_id"] = identifierValue(msg.matching_object_id());
  }
  return out;
}

template <typename MessageT, typename ParseFn>
std::string encodeArrayJson(const char* json_text, ParseFn parse_fn) {
  const auto arr = json::parse(std::string(json_text ? json_text : "[]"));
  std::string out;
  if (!arr.is_array()) {
    throw std::runtime_error("protobuf array encode expects JSON array");
  }
  for (const auto& item : arr) {
    const auto encoded = serializeProto<MessageT>(parse_fn(item));
    appendVarint32(static_cast<uint32_t>(encoded.size()), out);
    out.append(encoded);
  }
  return out;
}

template <typename MessageT, typename ToJsonFn>
std::string decodeArrayJson(const void* data, size_t size, ToJsonFn to_json) {
  const char* cursor = static_cast<const char*>(data);
  const char* end = cursor + size;
  json arr = json::array();
  while (cursor < end) {
    uint32_t length = 0;
    if (!readVarint32(cursor, end, length) ||
        static_cast<size_t>(end - cursor) < length) {
      throw std::runtime_error("invalid protobuf array framing");
    }
    MessageT message;
    if (!message.ParseFromArray(cursor, static_cast<int>(length))) {
      throw std::runtime_error("invalid protobuf message in array");
    }
    cursor += length;
    arr.push_back(to_json(message));
  }
  return arr.dump();
}

template <typename MessageT, typename ParseFn>
void* encodeSingle(const char* json_text, size_t* size_out, ParseFn parse_fn) {
  const auto message = parse_fn(json::parse(std::string(json_text ? json_text : "{}")));
  return copyBytes(serializeProto(message), size_out);
}

template <typename MessageT, typename ToJsonFn>
char* decodeSingle(const void* data, size_t size, const char* type_name,
                   ToJsonFn to_json) {
  const auto message = parseProto<MessageT>(data, size, type_name);
  return copyCString(to_json(message).dump());
}

void* encodeIdentifier(const char* json_text, size_t* size_out) {
  BaseIdentifier message;
  auto parsed = json::parse(std::string(json_text ? json_text : "\"\""));
  if (parsed.is_string()) {
    message.set_value(parsed.get<std::string>());
  }
  return copyBytes(serializeProto(message), size_out);
}

char* decodeIdentifier(const void* data, size_t size) {
  const auto message = parseProto<BaseIdentifier>(data, size, "Identifier");
  return copyCString(json(message.value()).dump());
}

}  // namespace

extern "C" {

void* pyramid_services_tactical_objects_GeodeticPosition_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonGeodeticPosition>(json, size_out, parseGeodeticPosition);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_GeodeticPosition_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonGeodeticPosition>(
        data, size, "GeodeticPosition",
        [](const CommonGeodeticPosition& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_PolyArea_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonPolyArea>(json, size_out, parsePolyArea);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_PolyArea_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonPolyArea>(
        data, size, "PolyArea",
        [](const CommonPolyArea& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Achievement_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonAchievement>(json, size_out, parseAchievement);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Achievement_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonAchievement>(
        data, size, "Achievement",
        [](const CommonAchievement& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Entity_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonEntity>(json, size_out, parseEntity);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Entity_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonEntity>(
        data, size, "Entity",
        [](const CommonEntity& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_CircleArea_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonCircleArea>(json, size_out, parseCircleArea);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_CircleArea_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonCircleArea>(
        data, size, "CircleArea",
        [](const CommonCircleArea& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Point_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonPoint>(json, size_out, parsePoint);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Point_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonPoint>(
        data, size, "Point",
        [](const CommonPoint& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Contraint_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonConstraint>(json, size_out, parseConstraint);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Contraint_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonConstraint>(
        data, size, "Contraint",
        [](const CommonConstraint& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Ack_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonAck>(json, size_out, parseAck);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Ack_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonAck>(
        data, size, "Ack",
        [](const CommonAck& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Query_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonQuery>(json, size_out, parseQuery);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Query_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonQuery>(
        data, size, "Query",
        [](const CommonQuery& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectDetail_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<TacticalObjectDetail>(json, size_out, parseObjectDetail);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectDetail_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<TacticalObjectDetail>(
        data, size, "ObjectDetail",
        [](const TacticalObjectDetail& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<TacticalObjectEvidenceRequirement>(
        json, size_out, parseObjectEvidenceRequirement);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectEvidenceRequirement_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<TacticalObjectEvidenceRequirement>(
        data, size, "ObjectEvidenceRequirement",
        [](const TacticalObjectEvidenceRequirement& value) {
          return toJson(value);
        });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectInterestRequirement_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<TacticalObjectInterestRequirement>(
        json, size_out, parseObjectInterestRequirement);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectInterestRequirement_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<TacticalObjectInterestRequirement>(
        data, size, "ObjectInterestRequirement",
        [](const TacticalObjectInterestRequirement& value) {
          return toJson(value);
        });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectMatch_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<TacticalObjectMatch>(json, size_out, parseObjectMatch);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectMatch_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<TacticalObjectMatch>(
        data, size, "ObjectMatch",
        [](const TacticalObjectMatch& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Capability_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeSingle<CommonCapability>(json, size_out, parseCapability);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Capability_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeSingle<CommonCapability>(
        data, size, "Capability",
        [](const CommonCapability& value) { return toJson(value); });
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_Identifier_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return encodeIdentifier(json, size_out);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_Identifier_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return decodeIdentifier(data, size);
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectDetailArray_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return copyBytes(
        encodeArrayJson<TacticalObjectDetail>(json, parseObjectDetail), size_out);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectDetailArray_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return copyCString(decodeArrayJson<TacticalObjectDetail>(
        data, size, [](const TacticalObjectDetail& value) { return toJson(value); }));
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return copyBytes(
        encodeArrayJson<TacticalObjectEvidenceRequirement>(
            json, parseObjectEvidenceRequirement),
        size_out);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return copyCString(decodeArrayJson<TacticalObjectEvidenceRequirement>(
        data, size, [](const TacticalObjectEvidenceRequirement& value) {
          return toJson(value);
        }));
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_CapabilityArray_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return copyBytes(
        encodeArrayJson<CommonCapability>(json, parseCapability), size_out);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_CapabilityArray_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return copyCString(decodeArrayJson<CommonCapability>(
        data, size, [](const CommonCapability& value) { return toJson(value); }));
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectMatchArray_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return copyBytes(
        encodeArrayJson<TacticalObjectMatch>(json, parseObjectMatch), size_out);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectMatchArray_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return copyCString(decodeArrayJson<TacticalObjectMatch>(
        data, size, [](const TacticalObjectMatch& value) { return toJson(value); }));
  } catch (...) {
    return nullptr;
  }
}

void* pyramid_services_tactical_objects_ObjectInterestRequirementArray_to_protobuf_json(
    const char* json, size_t* size_out) {
  try {
    return copyBytes(
        encodeArrayJson<TacticalObjectInterestRequirement>(
            json, parseObjectInterestRequirement),
        size_out);
  } catch (...) {
    if (size_out != nullptr) *size_out = 0;
    return nullptr;
  }
}

char* pyramid_services_tactical_objects_ObjectInterestRequirementArray_from_protobuf_json(
    const void* data, size_t size) {
  try {
    return copyCString(decodeArrayJson<TacticalObjectInterestRequirement>(
        data, size, [](const TacticalObjectInterestRequirement& value) {
          return toJson(value);
        }));
  } catch (...) {
    return nullptr;
  }
}

}  // extern "C"
