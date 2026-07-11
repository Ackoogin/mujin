// Auto-generated UCI 2.5 OMS-JSON PCL codec plugin.
// Only the schema-shaped UCI seam contract is supported.

#include "pyramid_data_model_uci_types.hpp"
#include "pyramid_data_model_uci_cabi_marshal.hpp"
#include <pcl/pcl_alloc.h>
#include <pcl/pcl_codec.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace {
using json = nlohmann::json;
namespace uci = pyramid::domain_model::uci;
constexpr const char* kContentType = "application/oms-json";
bool uuid(const std::string& s) { if (s.size() != 36) return false; for (size_t i=0;i<s.size();++i) { if (i==8||i==13||i==18||i==23) { if(s[i]!='-') return false; } else if (!((s[i]>='0'&&s[i]<='9')||(s[i]>='a'&&s[i]<='f')||(s[i]>='A'&&s[i]<='F'))) return false; } return true; }
json number(double v) { if (std::isnan(v)) return "NaN"; if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity"; return v; }
double number_in(const json& v) { if (v.is_number()) return v.get<double>(); auto s=v.get<std::string>(); if(s=="NaN") return std::numeric_limits<double>::quiet_NaN(); if(s=="Infinity") return std::numeric_limits<double>::infinity(); if(s=="-Infinity") return -std::numeric_limits<double>::infinity(); throw json::type_error::create(302,"invalid OMS number",&v); }

json encode_Uuid(const uci::Uuid& m);
uci::Uuid decode_Uuid(const json& j);
json encode_Identifier(const uci::Identifier& m);
uci::Identifier decode_Identifier(const json& j);
json encode_GovernmentIdentifier(const uci::GovernmentIdentifier& m);
uci::GovernmentIdentifier decode_GovernmentIdentifier(const json& j);
json encode_SecurityInformation(const uci::SecurityInformation& m);
uci::SecurityInformation decode_SecurityInformation(const json& j);
json encode_MissionId(const uci::MissionId& m);
uci::MissionId decode_MissionId(const json& j);
json encode_SystemId(const uci::SystemId& m);
uci::SystemId decode_SystemId(const json& j);
json encode_ServiceId(const uci::ServiceId& m);
uci::ServiceId decode_ServiceId(const json& j);
json encode_CommandId(const uci::CommandId& m);
uci::CommandId decode_CommandId(const json& j);
json encode_CapabilityId(const uci::CapabilityId& m);
uci::CapabilityId decode_CapabilityId(const json& j);
json encode_ActionId(const uci::ActionId& m);
uci::ActionId decode_ActionId(const json& j);
json encode_MessageHeader(const uci::MessageHeader& m);
uci::MessageHeader decode_MessageHeader(const json& j);
json encode_Rank(const uci::Rank& m);
uci::Rank decode_Rank(const json& j);
json encode_Ranking(const uci::Ranking& m);
uci::Ranking decode_Ranking(const json& j);
json encode_Capability(const uci::Capability& m);
uci::Capability decode_Capability(const json& j);
json encode_Command(const uci::Command& m);
uci::Command decode_Command(const json& j);
json encode_ActionCommandMessageData(const uci::ActionCommandMessageData& m);
uci::ActionCommandMessageData decode_ActionCommandMessageData(const json& j);
json encode_ActionCommand(const uci::ActionCommand& m);
uci::ActionCommand decode_ActionCommand(const json& j);
json encode_ActionCommandStatusMessageData(const uci::ActionCommandStatusMessageData& m);
uci::ActionCommandStatusMessageData decode_ActionCommandStatusMessageData(const json& j);
json encode_ActionCommandStatus(const uci::ActionCommandStatus& m);
uci::ActionCommandStatus decode_ActionCommandStatus(const json& j);
json encode_Ack(const uci::Ack& m);
uci::Ack decode_Ack(const json& j);
json encode_Query(const uci::Query& m);
uci::Query decode_Query(const json& j);

json encode_Uuid(const uci::Uuid& m) { if (!uuid(m.uuid)) throw json::type_error::create(302,"invalid UUID",nullptr); return m.uuid; }

uci::Uuid decode_Uuid(const json& j) { uci::Uuid r{}; r.uuid=j.get<std::string>(); if (!uuid(r.uuid)) throw json::type_error::create(302,"invalid UUID",nullptr); return r; }

json encode_Identifier(const uci::Identifier& m) { json o=json::object();
  o["ID"] = m.id;
  return o; }

uci::Identifier decode_Identifier(const json& j) { uci::Identifier r{};
  r.id = j.at("ID").get<std::string>();
  return r; }

json encode_GovernmentIdentifier(const uci::GovernmentIdentifier& m) { json o=json::object();
  o["GovernmentIdentifier"] = m.government_identifier;
  return o; }

uci::GovernmentIdentifier decode_GovernmentIdentifier(const json& j) { uci::GovernmentIdentifier r{};
  r.government_identifier = j.at("GovernmentIdentifier").get<std::string>();
  return r; }

json encode_SecurityInformation(const uci::SecurityInformation& m) { json o=json::object();
  o["Classification"] = m.classification;
  o["OwnerProducer"] = ([&]{ json a=json::array(); for(const auto& x:m.owner_producer) a.push_back(encode_GovernmentIdentifier(x)); return a; })();
  return o; }

uci::SecurityInformation decode_SecurityInformation(const json& j) { uci::SecurityInformation r{};
  r.classification = j.at("Classification").get<std::string>();
  r.owner_producer = ([&]{ decltype(r.owner_producer) a; for(const auto& x:j.at("OwnerProducer")) a.push_back(decode_GovernmentIdentifier(x)); return a; })();
  return r; }

json encode_MissionId(const uci::MissionId& m) { json o=json::object();
  o["UUID"] = encode_Uuid(m.uuid);
  if (!m.descriptive_label.empty()) o["DescriptiveLabel"] = m.descriptive_label;
  return o; }

uci::MissionId decode_MissionId(const json& j) { uci::MissionId r{};
  r.uuid = decode_Uuid(j.at("UUID"));
  if (j.contains("DescriptiveLabel")) r.descriptive_label = j.at("DescriptiveLabel").get<std::string>();
  return r; }

json encode_SystemId(const uci::SystemId& m) { json o=json::object();
  o["UUID"] = encode_Uuid(m.uuid);
  return o; }

uci::SystemId decode_SystemId(const json& j) { uci::SystemId r{};
  r.uuid = decode_Uuid(j.at("UUID"));
  return r; }

json encode_ServiceId(const uci::ServiceId& m) { json o=json::object();
  o["UUID"] = encode_Uuid(m.uuid);
  return o; }

uci::ServiceId decode_ServiceId(const json& j) { uci::ServiceId r{};
  r.uuid = decode_Uuid(j.at("UUID"));
  return r; }

json encode_CommandId(const uci::CommandId& m) { json o=json::object();
  o["UUID"] = encode_Uuid(m.uuid);
  return o; }

uci::CommandId decode_CommandId(const json& j) { uci::CommandId r{};
  r.uuid = decode_Uuid(j.at("UUID"));
  return r; }

json encode_CapabilityId(const uci::CapabilityId& m) { json o=json::object();
  o["UUID"] = encode_Uuid(m.uuid);
  return o; }

uci::CapabilityId decode_CapabilityId(const json& j) { uci::CapabilityId r{};
  r.uuid = decode_Uuid(j.at("UUID"));
  return r; }

json encode_ActionId(const uci::ActionId& m) { json o=json::object();
  o["UUID"] = encode_Uuid(m.uuid);
  return o; }

uci::ActionId decode_ActionId(const json& j) { uci::ActionId r{};
  r.uuid = decode_Uuid(j.at("UUID"));
  return r; }

json encode_MessageHeader(const uci::MessageHeader& m) { json o=json::object();
  if (m.mission_id) o["MissionID"] = encode_MissionId(*m.mission_id);
  o["SystemID"] = encode_SystemId(m.system_id);
  if (m.service_id) o["ServiceID"] = encode_ServiceId(*m.service_id);
  o["Timestamp"] = m.timestamp;
  o["SchemaVersion"] = m.schema_version;
  o["Mode"] = m.mode;
  return o; }

uci::MessageHeader decode_MessageHeader(const json& j) { uci::MessageHeader r{};
  if (j.contains("MissionID")) r.mission_id = decode_MissionId(j.at("MissionID"));
  r.system_id = decode_SystemId(j.at("SystemID"));
  if (j.contains("ServiceID")) r.service_id = decode_ServiceId(j.at("ServiceID"));
  r.timestamp = j.at("Timestamp").get<std::string>();
  r.schema_version = j.at("SchemaVersion").get<std::string>();
  r.mode = j.at("Mode").get<std::string>();
  return r; }

json encode_Rank(const uci::Rank& m) { json o=json::object();
  o["Priority"] = m.priority;
  return o; }

uci::Rank decode_Rank(const json& j) { uci::Rank r{};
  r.priority = j.at("Priority").get<int32_t>();
  return r; }

json encode_Ranking(const uci::Ranking& m) { json o=json::object();
  o["Rank"] = encode_Rank(m.rank);
  return o; }

uci::Ranking decode_Ranking(const json& j) { uci::Ranking r{};
  r.rank = decode_Rank(j.at("Rank"));
  return r; }

json encode_Capability(const uci::Capability& m) { json o=json::object();
  o["CommandID"] = encode_CommandId(m.command_id);
  o["CommandState"] = m.command_state;
  o["CapabilityID"] = encode_CapabilityId(m.capability_id);
  o["Ranking"] = encode_Ranking(m.ranking);
  o["ActionID"] = encode_ActionId(m.action_id);
  return o; }

uci::Capability decode_Capability(const json& j) { uci::Capability r{};
  r.command_id = decode_CommandId(j.at("CommandID"));
  r.command_state = j.at("CommandState").get<std::string>();
  r.capability_id = decode_CapabilityId(j.at("CapabilityID"));
  r.ranking = decode_Ranking(j.at("Ranking"));
  r.action_id = decode_ActionId(j.at("ActionID"));
  return r; }

json encode_Command(const uci::Command& m) { json o=json::object();
  o["Capability"] = encode_Capability(m.capability);
  return o; }

uci::Command decode_Command(const json& j) { uci::Command r{};
  r.capability = decode_Capability(j.at("Capability"));
  return r; }

json encode_ActionCommandMessageData(const uci::ActionCommandMessageData& m) { json o=json::object();
  o["Command"] = ([&]{ json a=json::array(); for(const auto& x:m.command) a.push_back(encode_Command(x)); return a; })();
  return o; }

uci::ActionCommandMessageData decode_ActionCommandMessageData(const json& j) { uci::ActionCommandMessageData r{};
  r.command = ([&]{ decltype(r.command) a; for(const auto& x:j.at("Command")) a.push_back(decode_Command(x)); return a; })();
  return r; }

json encode_ActionCommand(const uci::ActionCommand& m) { json o=json::object();
  o["SecurityInformation"] = encode_SecurityInformation(m.security_information);
  o["MessageHeader"] = encode_MessageHeader(m.message_header);
  o["MessageData"] = encode_ActionCommandMessageData(m.message_data);
  return o; }

uci::ActionCommand decode_ActionCommand(const json& j) { uci::ActionCommand r{};
  r.security_information = decode_SecurityInformation(j.at("SecurityInformation"));
  r.message_header = decode_MessageHeader(j.at("MessageHeader"));
  r.message_data = decode_ActionCommandMessageData(j.at("MessageData"));
  return r; }

json encode_ActionCommandStatusMessageData(const uci::ActionCommandStatusMessageData& m) { json o=json::object();
  o["CommandID"] = encode_CommandId(m.command_id);
  o["CommandProcessingState"] = m.command_processing_state;
  return o; }

uci::ActionCommandStatusMessageData decode_ActionCommandStatusMessageData(const json& j) { uci::ActionCommandStatusMessageData r{};
  r.command_id = decode_CommandId(j.at("CommandID"));
  r.command_processing_state = j.at("CommandProcessingState").get<std::string>();
  return r; }

json encode_ActionCommandStatus(const uci::ActionCommandStatus& m) { json o=json::object();
  o["SecurityInformation"] = encode_SecurityInformation(m.security_information);
  o["MessageHeader"] = encode_MessageHeader(m.message_header);
  o["MessageData"] = encode_ActionCommandStatusMessageData(m.message_data);
  return o; }

uci::ActionCommandStatus decode_ActionCommandStatus(const json& j) { uci::ActionCommandStatus r{};
  r.security_information = decode_SecurityInformation(j.at("SecurityInformation"));
  r.message_header = decode_MessageHeader(j.at("MessageHeader"));
  r.message_data = decode_ActionCommandStatusMessageData(j.at("MessageData"));
  return r; }

json encode_Ack(const uci::Ack& m) { json o=json::object();
  o["Success"] = m.success;
  return o; }

uci::Ack decode_Ack(const json& j) { uci::Ack r{};
  r.success = j.at("Success").get<bool>();
  return r; }

json encode_Query(const uci::Query& m) { json o=json::object();
  return o; }

uci::Query decode_Query(const json& j) { uci::Query r{};
  return r; }

struct ActionCommandRequestWire { uint8_t has_action_command; pyramid_data_model_uci_ActionCommand_c action_command; uint8_t has_update; pyramid_data_model_uci_ActionCommandStatus_c update; };
struct ActionCommandRequirementWire { uint8_t has_action_command_status; pyramid_data_model_uci_ActionCommandStatus_c action_command_status; };

pcl_status_t assign(const std::string& s, pcl_msg_t* out) {
  if (!out || s.size()>std::numeric_limits<uint32_t>::max()) return PCL_ERR_INVALID;
  void* p=s.empty()?nullptr:pcl_alloc(s.size()); if (!s.empty() && !p) return PCL_ERR_NOMEM;
  if(p) std::memcpy(p,s.data(),s.size()); out->data=p; out->size=static_cast<uint32_t>(s.size()); out->type_name=kContentType; return PCL_OK; }
pcl_status_t encode(void*, const char* id, const void* value, pcl_msg_t* out) {
  if(!id||!value||!out) return PCL_ERR_INVALID; try {
    if(std::strcmp(id,"ActionCommand")==0) { uci::ActionCommand n; pyramid::cabi::from_c(static_cast<const pyramid_data_model_uci_ActionCommand_c*>(value),n); return assign(json({{"ActionCommand",encode_ActionCommand(n)}}).dump(),out); }
    if(std::strcmp(id,"ActionCommandStatus")==0) { uci::ActionCommandStatus n; pyramid::cabi::from_c(static_cast<const pyramid_data_model_uci_ActionCommandStatus_c*>(value),n); return assign(json({{"ActionCommandStatus",encode_ActionCommandStatus(n)}}).dump(),out); }
    if(std::strcmp(id,"ActionCommand_Service_Request")==0) { const auto& w=*static_cast<const ActionCommandRequestWire*>(value); if(w.has_action_command == w.has_update) return PCL_ERR_INVALID; if(w.has_action_command) { uci::ActionCommand n; pyramid::cabi::from_c(&w.action_command,n); return assign(json({{"ActionCommand",encode_ActionCommand(n)}}).dump(),out); } uci::ActionCommandStatus n; pyramid::cabi::from_c(&w.update,n); return assign(json({{"ActionCommandStatus",encode_ActionCommandStatus(n)}}).dump(),out); }
    if(std::strcmp(id,"ActionCommand_Service_Requirement")==0) { const auto& w=*static_cast<const ActionCommandRequirementWire*>(value); if(!w.has_action_command_status) return PCL_ERR_INVALID; uci::ActionCommandStatus n; pyramid::cabi::from_c(&w.action_command_status,n); return assign(json({{"ActionCommandStatus",encode_ActionCommandStatus(n)}}).dump(),out); }
  } catch (...) { return PCL_ERR_INVALID; } return PCL_ERR_NOT_FOUND; }
pcl_status_t decode(void*, const char* id, const pcl_msg_t* msg, void* value) {
  if(!id||!msg||(!msg->data&&msg->size)||!value) return PCL_ERR_INVALID; try { json d=json::parse(msg->data?std::string(static_cast<const char*>(msg->data),msg->size):std::string());
    if(std::strcmp(id,"ActionCommand")==0) { auto n=decode_ActionCommand(d.at("ActionCommand")); pyramid::cabi::to_c(n,static_cast<pyramid_data_model_uci_ActionCommand_c*>(value)); return PCL_OK; }
    if(std::strcmp(id,"ActionCommandStatus")==0) { auto n=decode_ActionCommandStatus(d.at("ActionCommandStatus")); pyramid::cabi::to_c(n,static_cast<pyramid_data_model_uci_ActionCommandStatus_c*>(value)); return PCL_OK; }
    if(std::strcmp(id,"ActionCommand_Service_Request")==0) { auto* w=static_cast<ActionCommandRequestWire*>(value); std::memset(w,0,sizeof(*w)); if(d.contains("ActionCommand")) { auto n=decode_ActionCommand(d.at("ActionCommand")); pyramid::cabi::to_c(n,&w->action_command); w->has_action_command=1; return PCL_OK; } if(d.contains("ActionCommandStatus")) { auto n=decode_ActionCommandStatus(d.at("ActionCommandStatus")); pyramid::cabi::to_c(n,&w->update); w->has_update=1; return PCL_OK; } return PCL_ERR_INVALID; }
    if(std::strcmp(id,"ActionCommand_Service_Requirement")==0) { auto* w=static_cast<ActionCommandRequirementWire*>(value); std::memset(w,0,sizeof(*w)); if(!d.contains("ActionCommandStatus")) return PCL_ERR_INVALID; auto n=decode_ActionCommandStatus(d.at("ActionCommandStatus")); pyramid::cabi::to_c(n,&w->action_command_status); w->has_action_command_status=1; return PCL_OK; }
  } catch (...) { return PCL_ERR_INVALID; } return PCL_ERR_NOT_FOUND; }
void free_msg(void*, pcl_msg_t* m) { if(!m)return; pcl_free(const_cast<void*>(m->data)); m->data=nullptr;m->size=0;m->type_name=nullptr; }
pcl_codec_t codec={PCL_CODEC_ABI_VERSION,kContentType,encode,decode,free_msg,nullptr};
} // namespace

extern "C" __attribute__((visibility("default"))) const pcl_codec_t* pcl_codec_plugin_entry(const char*) { return &codec; }
