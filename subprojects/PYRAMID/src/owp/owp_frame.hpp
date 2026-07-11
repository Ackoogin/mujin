#pragma once

#include <map>
#include <string>
#include <vector>

namespace pyramid::owp {

struct Init {
  std::vector<std::string> versions;
  std::string schema;
  std::string service_id;
  bool verbose = true;
};

struct Identifiers {
  std::string system;
  std::string service;
  std::string subsystem;
  std::map<std::string, std::string> capabilities;
  std::map<std::string, std::string> components;
};

struct Info {
  std::string version;
  std::string server_id;
  Identifiers uuids;
  std::string system_label;
  std::vector<std::string> connect_urls;
};

enum class ServerFrameType { Info, Message, Ok, Error };

struct ServerFrame {
  ServerFrameType type;
  Info info;
  std::string subscription_id;
  std::string body;
  std::string error_name;
  std::string error_details;
};

/// \brief Returns whether a protocol identifier matches the pinned grammar.
bool isIdentifier(const std::string& value);
std::string serializeInit(const Init& init);
std::string serializeSubscribe(const std::string& subscription_id,
                               const std::string& message_name,
                               const std::string& topic,
                               const std::string& group = "");
std::string serializeUnsubscribe(const std::string& subscription_id);
std::string serializePublish(const std::string& topic, const std::string& body);
ServerFrame parseServerFrame(const std::string& frame);

}  // namespace pyramid::owp
