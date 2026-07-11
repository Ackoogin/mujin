/// \file pyramid_oms_json_codec_uci.cpp
/// \brief OMS JSON codec for the initial UCI 2.5 starter-kit message subset.

#include <pcl/pcl_alloc.h>
#include <pcl/pcl_codec.h>

#include <pyramid/oms_json_types.h>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#if defined(_WIN32)
#  define PYRAMID_OMS_CODEC_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PYRAMID_OMS_CODEC_EXPORT __attribute__((visibility("default")))
#else
#  define PYRAMID_OMS_CODEC_EXPORT
#endif

namespace {

constexpr const char* kContentType = "application/oms-json";
constexpr const char* kSignalReport = "SignalReport";
constexpr const char* kPositionReport = "PositionReport";
constexpr const char* kActionCommand = "ActionCommand";
constexpr const char* kActionCommandStatus = "ActionCommandStatus";

bool is_hex(char value) {
  return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f') ||
         (value >= 'A' && value <= 'F');
}

bool is_uuid(const char* value) {
  if (!value || std::strlen(value) != 36u) return false;
  for (size_t i = 0; i < 36u; ++i) {
    if (i == 8u || i == 13u || i == 18u || i == 23u) {
      if (value[i] != '-') return false;
    } else if (!is_hex(value[i])) {
      return false;
    }
  }
  return true;
}

bool required_text(const char* value) {
  return value && value[0] != '\0';
}

bool valid_header(const pyramid_uci_oms_header_c& header) {
  if (!required_text(header.classification) ||
      !required_text(header.owner_producer) || !is_uuid(header.system_uuid) ||
      !required_text(header.timestamp) ||
      !required_text(header.schema_version) || !required_text(header.mode)) {
    return false;
  }
  if (header.mission_uuid[0] != '\0' && !is_uuid(header.mission_uuid)) {
    return false;
  }
  return header.service_uuid[0] == '\0' || is_uuid(header.service_uuid);
}

nlohmann::json encode_security(const pyramid_uci_oms_header_c& header) {
  return {{"Classification", header.classification},
          {"OwnerProducer",
           nlohmann::json::array(
               {{{"GovernmentIdentifier", header.owner_producer}}})}};
}

nlohmann::json encode_header(const pyramid_uci_oms_header_c& header) {
  nlohmann::json result = nlohmann::json::object();
  if (header.mission_uuid[0] != '\0') {
    result["MissionID"] = {{"UUID", header.mission_uuid}};
    if (header.mission_label[0] != '\0') {
      result["MissionID"]["DescriptiveLabel"] = header.mission_label;
    }
  }
  result["SystemID"] = {{"UUID", header.system_uuid}};
  if (header.service_uuid[0] != '\0') {
    result["ServiceID"] = {{"UUID", header.service_uuid}};
  }
  result["Timestamp"] = header.timestamp;
  result["SchemaVersion"] = header.schema_version;
  result["Mode"] = header.mode;
  return result;
}

nlohmann::json encode_number(double value) {
  if (std::isnan(value)) return "NaN";
  if (std::isinf(value)) return value > 0.0 ? "Infinity" : "-Infinity";
  return value;
}

double decode_number(const nlohmann::json& value) {
  if (value.is_number()) return value.get<double>();
  if (!value.is_string()) throw nlohmann::json::type_error::create(302, "", &value);
  const std::string token = value.get<std::string>();
  if (token == "NaN") return std::numeric_limits<double>::quiet_NaN();
  if (token == "Infinity") return std::numeric_limits<double>::infinity();
  if (token == "-Infinity") return -std::numeric_limits<double>::infinity();
  throw nlohmann::json::type_error::create(302, "invalid OMS JSON number", &value);
}

template <size_t Size>
void copy_text(const nlohmann::json& value, char (&out)[Size]) {
  const std::string text = value.get<std::string>();
  if (text.size() >= Size) {
    throw nlohmann::json::out_of_range::create(401, "string exceeds C ABI field", &value);
  }
  std::memcpy(out, text.c_str(), text.size() + 1u);
}

template <size_t Size>
void copy_optional_text(const nlohmann::json& object, const char* key,
                        char (&out)[Size]) {
  out[0] = '\0';
  auto it = object.find(key);
  if (it != object.end()) copy_text(*it, out);
}

void decode_header(const nlohmann::json& root,
                   pyramid_uci_oms_header_c* header) {
  std::memset(header, 0, sizeof(*header));
  const auto& security = root.at("SecurityInformation");
  copy_text(security.at("Classification"), header->classification);
  copy_text(security.at("OwnerProducer").at(0).at("GovernmentIdentifier"),
            header->owner_producer);

  const auto& message_header = root.at("MessageHeader");
  auto mission = message_header.find("MissionID");
  if (mission != message_header.end()) {
    copy_text(mission->at("UUID"), header->mission_uuid);
    copy_optional_text(*mission, "DescriptiveLabel", header->mission_label);
  }
  copy_text(message_header.at("SystemID").at("UUID"), header->system_uuid);
  auto service = message_header.find("ServiceID");
  if (service != message_header.end()) {
    copy_text(service->at("UUID"), header->service_uuid);
  }
  copy_text(message_header.at("Timestamp"), header->timestamp);
  copy_text(message_header.at("SchemaVersion"), header->schema_version);
  copy_text(message_header.at("Mode"), header->mode);

  if (!valid_header(*header)) {
    throw nlohmann::json::type_error::create(302, "invalid UCI message header", &root);
  }
}

nlohmann::json encode_signal_report(const pyramid_uci_signal_report_c& report) {
  if (!valid_header(report.header) || !is_uuid(report.signal_report_uuid) ||
      !is_uuid(report.signal_uuid) || !required_text(report.signal_state)) {
    throw nlohmann::json::type_error::create(302, "invalid SignalReport", nullptr);
  }

  nlohmann::json signal = {{"SignalID", {{"UUID", report.signal_uuid}}},
                           {"SignalState", report.signal_state}};
  if (report.has_frequency_average) {
    signal["Parametrics"]["Frequency"]["FrequencyAverage"] =
        encode_number(report.frequency_average_hz);
  }

  return {{kSignalReport,
           {{"SecurityInformation", encode_security(report.header)},
            {"MessageHeader", encode_header(report.header)},
            {"MessageData",
             {{"SignalReportID", {{"UUID", report.signal_report_uuid}}},
              {"Signal", nlohmann::json::array({std::move(signal)})}}}}}};
}

nlohmann::json encode_position_report(
    const pyramid_uci_position_report_c& report) {
  if (!valid_header(report.header) || !is_uuid(report.report_system_uuid) ||
      !required_text(report.source) ||
      !required_text(report.current_operating_domain) ||
      !required_text(report.position_timestamp)) {
    throw nlohmann::json::type_error::create(302, "invalid PositionReport", nullptr);
  }

  nlohmann::json position = {
      {"Latitude", encode_number(report.latitude_rad)},
      {"Longitude", encode_number(report.longitude_rad)},
      {"Altitude", encode_number(report.altitude_m)},
      {"Timestamp", report.position_timestamp}};
  if (report.has_altitude_reference) {
    if (!required_text(report.altitude_reference)) {
      throw nlohmann::json::type_error::create(
          302, "missing altitude reference", nullptr);
    }
    position["AltitudeReference"] = report.altitude_reference;
  }

  return {{kPositionReport,
           {{"SecurityInformation", encode_security(report.header)},
            {"MessageHeader", encode_header(report.header)},
            {"MessageData",
             {{"SystemID", {{"UUID", report.report_system_uuid}}},
              {"Source", report.source},
              {"CurrentOperatingDomain", report.current_operating_domain},
              {"InertialState", {{"Position", std::move(position)}}}}}}}};
}

void decode_signal_report(const nlohmann::json& document,
                          pyramid_uci_signal_report_c* report) {
  std::memset(report, 0, sizeof(*report));
  const auto& root = document.at(kSignalReport);
  decode_header(root, &report->header);
  const auto& data = root.at("MessageData");
  copy_text(data.at("SignalReportID").at("UUID"),
            report->signal_report_uuid);
  const auto& signal = data.at("Signal").at(0);
  copy_text(signal.at("SignalID").at("UUID"), report->signal_uuid);
  copy_text(signal.at("SignalState"), report->signal_state);
  if (!is_uuid(report->signal_report_uuid) || !is_uuid(report->signal_uuid)) {
    throw nlohmann::json::type_error::create(302, "invalid SignalReport UUID", &root);
  }

  auto parametrics = signal.find("Parametrics");
  if (parametrics != signal.end()) {
    auto frequency = parametrics->find("Frequency");
    if (frequency != parametrics->end()) {
      auto average = frequency->find("FrequencyAverage");
      if (average != frequency->end()) {
        report->has_frequency_average = true;
        report->frequency_average_hz = decode_number(*average);
      }
    }
  }
}

void decode_position_report(const nlohmann::json& document,
                            pyramid_uci_position_report_c* report) {
  std::memset(report, 0, sizeof(*report));
  const auto& root = document.at(kPositionReport);
  decode_header(root, &report->header);
  const auto& data = root.at("MessageData");
  copy_text(data.at("SystemID").at("UUID"), report->report_system_uuid);
  copy_text(data.at("Source"), report->source);
  copy_text(data.at("CurrentOperatingDomain"),
            report->current_operating_domain);
  const auto& position = data.at("InertialState").at("Position");
  report->latitude_rad = decode_number(position.at("Latitude"));
  report->longitude_rad = decode_number(position.at("Longitude"));
  report->altitude_m = decode_number(position.at("Altitude"));
  copy_text(position.at("Timestamp"), report->position_timestamp);
  auto altitude_reference = position.find("AltitudeReference");
  if (altitude_reference != position.end()) {
    report->has_altitude_reference = true;
    copy_text(*altitude_reference, report->altitude_reference);
  }
  if (!is_uuid(report->report_system_uuid)) {
    throw nlohmann::json::type_error::create(302, "invalid PositionReport UUID", &root);
  }
}

nlohmann::json encode_action_command(const pyramid_uci_action_command_c& cmd) {
  if (!valid_header(cmd.header) || !is_uuid(cmd.command_uuid) ||
      !required_text(cmd.command_state) || !is_uuid(cmd.capability_uuid) ||
      !is_uuid(cmd.action_uuid)) {
    throw nlohmann::json::type_error::create(302, "invalid ActionCommand", nullptr);
  }

  nlohmann::json capability = {
      {"CommandID", {{"UUID", cmd.command_uuid}}},
      {"CommandState", cmd.command_state},
      {"CapabilityID", {{"UUID", cmd.capability_uuid}}},
      {"Ranking", {{"Rank", {{"Priority", cmd.priority}}}}},
      {"ActionID", {{"UUID", cmd.action_uuid}}}};

  return {{kActionCommand,
           {{"SecurityInformation", encode_security(cmd.header)},
            {"MessageHeader", encode_header(cmd.header)},
            {"MessageData",
             {{"Command",
               nlohmann::json::array({{{"Capability", std::move(capability)}}})}}}}}};
}

void decode_action_command(const nlohmann::json& document,
                           pyramid_uci_action_command_c* cmd) {
  std::memset(cmd, 0, sizeof(*cmd));
  const auto& root = document.at(kActionCommand);
  decode_header(root, &cmd->header);
  const auto& capability =
      root.at("MessageData").at("Command").at(0).at("Capability");
  copy_text(capability.at("CommandID").at("UUID"), cmd->command_uuid);
  copy_text(capability.at("CommandState"), cmd->command_state);
  copy_text(capability.at("CapabilityID").at("UUID"), cmd->capability_uuid);
  cmd->priority = capability.at("Ranking").at("Rank").at("Priority").get<int32_t>();
  copy_text(capability.at("ActionID").at("UUID"), cmd->action_uuid);
  if (!is_uuid(cmd->command_uuid) || !is_uuid(cmd->capability_uuid) ||
      !is_uuid(cmd->action_uuid) || !required_text(cmd->command_state)) {
    throw nlohmann::json::type_error::create(302, "invalid ActionCommand", &root);
  }
}

nlohmann::json encode_action_command_status(
    const pyramid_uci_action_command_status_c& status) {
  if (!valid_header(status.header) || !is_uuid(status.command_uuid) ||
      !required_text(status.command_processing_state)) {
    throw nlohmann::json::type_error::create(302, "invalid ActionCommandStatus",
                                             nullptr);
  }

  return {{kActionCommandStatus,
           {{"SecurityInformation", encode_security(status.header)},
            {"MessageHeader", encode_header(status.header)},
            {"MessageData",
             {{"CommandID", {{"UUID", status.command_uuid}}},
              {"CommandProcessingState", status.command_processing_state}}}}}};
}

void decode_action_command_status(const nlohmann::json& document,
                                  pyramid_uci_action_command_status_c* status) {
  std::memset(status, 0, sizeof(*status));
  const auto& root = document.at(kActionCommandStatus);
  decode_header(root, &status->header);
  const auto& data = root.at("MessageData");
  copy_text(data.at("CommandID").at("UUID"), status->command_uuid);
  copy_text(data.at("CommandProcessingState"), status->command_processing_state);
  if (!is_uuid(status->command_uuid) ||
      !required_text(status->command_processing_state)) {
    throw nlohmann::json::type_error::create(302, "invalid ActionCommandStatus",
                                             &root);
  }
}

pcl_status_t assign_payload(const std::string& payload, pcl_msg_t* out_msg) {
  if (!out_msg || payload.size() > std::numeric_limits<uint32_t>::max()) {
    return PCL_ERR_INVALID;
  }
  void* copy = nullptr;
  if (!payload.empty()) {
    copy = pcl_alloc(payload.size());
    if (!copy) return PCL_ERR_NOMEM;
    std::memcpy(copy, payload.data(), payload.size());
  }
  out_msg->data = copy;
  out_msg->size = static_cast<uint32_t>(payload.size());
  out_msg->type_name = kContentType;
  return PCL_OK;
}

pcl_status_t plugin_encode(void*, const char* schema_id, const void* value,
                           pcl_msg_t* out_msg) {
  if (!schema_id || !value || !out_msg) return PCL_ERR_INVALID;
  try {
    if (std::strcmp(schema_id, kSignalReport) == 0) {
      return assign_payload(
          encode_signal_report(
              *static_cast<const pyramid_uci_signal_report_c*>(value))
              .dump(),
          out_msg);
    }
    if (std::strcmp(schema_id, kPositionReport) == 0) {
      return assign_payload(
          encode_position_report(
              *static_cast<const pyramid_uci_position_report_c*>(value))
              .dump(),
          out_msg);
    }
    if (std::strcmp(schema_id, kActionCommand) == 0) {
      return assign_payload(
          encode_action_command(
              *static_cast<const pyramid_uci_action_command_c*>(value))
              .dump(),
          out_msg);
    }
    if (std::strcmp(schema_id, kActionCommandStatus) == 0) {
      return assign_payload(
          encode_action_command_status(
              *static_cast<const pyramid_uci_action_command_status_c*>(value))
              .dump(),
          out_msg);
    }
  } catch (const nlohmann::json::exception&) {
    return PCL_ERR_INVALID;
  }
  return PCL_ERR_NOT_FOUND;
}

pcl_status_t plugin_decode(void*, const char* schema_id, const pcl_msg_t* msg,
                           void* out_value) {
  if (!schema_id || !msg || (!msg->data && msg->size != 0u) || !out_value) {
    return PCL_ERR_INVALID;
  }
  try {
    const std::string payload = msg->data
                                    ? std::string(
                                          static_cast<const char*>(msg->data),
                                          msg->size)
                                    : std::string();
    const nlohmann::json document = nlohmann::json::parse(payload);
    if (std::strcmp(schema_id, kSignalReport) == 0) {
      decode_signal_report(
          document, static_cast<pyramid_uci_signal_report_c*>(out_value));
      return PCL_OK;
    }
    if (std::strcmp(schema_id, kPositionReport) == 0) {
      decode_position_report(
          document, static_cast<pyramid_uci_position_report_c*>(out_value));
      return PCL_OK;
    }
    if (std::strcmp(schema_id, kActionCommand) == 0) {
      decode_action_command(
          document, static_cast<pyramid_uci_action_command_c*>(out_value));
      return PCL_OK;
    }
    if (std::strcmp(schema_id, kActionCommandStatus) == 0) {
      decode_action_command_status(
          document,
          static_cast<pyramid_uci_action_command_status_c*>(out_value));
      return PCL_OK;
    }
  } catch (const nlohmann::json::exception&) {
    return PCL_ERR_INVALID;
  }
  return PCL_ERR_NOT_FOUND;
}

void plugin_free_msg(void*, pcl_msg_t* msg) {
  if (!msg) return;
  pcl_free(const_cast<void*>(msg->data));
  msg->data = nullptr;
  msg->size = 0u;
  msg->type_name = nullptr;
}

pcl_codec_t codec = {PCL_CODEC_ABI_VERSION, kContentType, plugin_encode,
                     plugin_decode, plugin_free_msg, nullptr};

}  // namespace

extern "C" {

PYRAMID_OMS_CODEC_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(const char*) {
  return &codec;
}

}  // extern "C"
