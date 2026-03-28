#include "ame/bt_nodes/invoke_service.h"

#include <sstream>

namespace ame {

InvokeService::InvokeService(const std::string& name,
                             const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList InvokeService::providedPorts() {
  return {
      BT::InputPort<std::string>("service_name"),
      BT::InputPort<std::string>("operation"),
      BT::InputPort<unsigned>("timeout_ms", 5000u, "Timeout in ms (0 = no timeout)"),
      BT::InputPort<std::string>("request_json", "", "k=v;k=v encoded request fields"),
      BT::InputPort<std::string>("param_names", "", "Semicolon-separated PDDL param names"),
      BT::InputPort<std::string>("param_values", "", "Semicolon-separated PDDL param values"),
      BT::OutputPort<std::string>("response_json"),
  };
}

BT::NodeStatus InvokeService::onStart() {
  auto service_name = getInput<std::string>("service_name");
  auto operation = getInput<std::string>("operation");

  if (!service_name || !operation) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    service_ = config().blackboard->get<IPyramidService*>("pyramid_service");
  } catch (const std::exception&) {
    service_ = nullptr;
  }
  if (!service_) {
    return BT::NodeStatus::FAILURE;
  }

  // Build request from explicit k=v pairs
  std::string encoded;
  getInput("request_json", encoded);
  ServiceMessage request = parseRequest(encoded);

  // Merge PDDL param bindings (auto-mapping from ActionRegistry)
  std::string param_names, param_values;
  getInput("param_names", param_names);
  getInput("param_values", param_values);
  mergeParamBindings(param_names, param_values, request);

  // Configure timeout
  unsigned timeout_ms = 5000;
  getInput("timeout_ms", timeout_ms);
  if (timeout_ms > 0) {
    has_timeout_ = true;
    deadline_ = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(timeout_ms);
  } else {
    has_timeout_ = false;
  }

  // Initiate async call
  request_id_ = service_->callAsync(service_name.value(), operation.value(), request);

  // Check immediately in case the call completed synchronously
  return onRunning();
}

BT::NodeStatus InvokeService::onRunning() {
  if (!service_) {
    return BT::NodeStatus::FAILURE;
  }

  // Check timeout
  if (has_timeout_ && std::chrono::steady_clock::now() >= deadline_) {
    service_->cancelCall(request_id_);
    return BT::NodeStatus::FAILURE;
  }

  ServiceMessage response;
  AsyncCallStatus status = service_->pollResult(request_id_, response);

  switch (status) {
    case AsyncCallStatus::SUCCESS:
      setOutput("response_json", serializeResponse(response));
      return BT::NodeStatus::SUCCESS;
    case AsyncCallStatus::FAILURE:
      return BT::NodeStatus::FAILURE;
    case AsyncCallStatus::CANCELLED:
      return BT::NodeStatus::FAILURE;
    case AsyncCallStatus::PENDING:
      return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::FAILURE;
}

void InvokeService::onHalted() {
  if (service_ && request_id_ != 0) {
    service_->cancelCall(request_id_);
  }
  request_id_ = 0;
  service_ = nullptr;
}

ServiceMessage InvokeService::parseRequest(const std::string& encoded) {
  ServiceMessage msg;
  if (encoded.empty()) return msg;

  std::istringstream stream(encoded);
  std::string token;
  while (std::getline(stream, token, ';')) {
    auto eq = token.find('=');
    if (eq != std::string::npos) {
      msg.set(token.substr(0, eq), token.substr(eq + 1));
    }
  }
  return msg;
}

std::string InvokeService::serializeResponse(const ServiceMessage& msg) {
  std::string result;
  for (auto& [k, v] : msg.fields) {
    if (!result.empty()) result += ";";
    result += k + "=" + v;
  }
  return result;
}

void InvokeService::mergeParamBindings(const std::string& names,
                                       const std::string& values,
                                       ServiceMessage& msg) {
  if (names.empty() || values.empty()) return;

  std::vector<std::string> name_list, value_list;

  {
    std::istringstream stream(names);
    std::string token;
    while (std::getline(stream, token, ';')) {
      // Strip leading '?' from PDDL param names
      if (!token.empty() && token[0] == '?') {
        token = token.substr(1);
      }
      name_list.push_back(token);
    }
  }

  {
    std::istringstream stream(values);
    std::string token;
    while (std::getline(stream, token, ';')) {
      value_list.push_back(token);
    }
  }

  size_t count = std::min(name_list.size(), value_list.size());
  for (size_t i = 0; i < count; ++i) {
    if (!name_list[i].empty()) {
      msg.set(name_list[i], value_list[i]);
    }
  }
}

} // namespace ame
