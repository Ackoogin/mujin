#pragma once

#include "ame/autonomy_backend.h"

#include <behaviortree_cpp/tree_node.h>

#include <atomic>
#include <cctype>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>

namespace ame {
namespace nodes {

// ActionRegistry emits positional ports param0..param7 for native command nodes.
constexpr int kMaxActionParams = 8;

inline BT::PortsList actionParamPorts() {
  BT::PortsList ports;
  for (int i = 0; i < kMaxActionParams; ++i) {
    ports.insert(BT::InputPort<std::string>("param" + std::to_string(i), "",
                                            "Positional command parameter"));
  }
  return ports;
}

inline std::string nextCommandId(const std::string& verb) {
  static std::atomic<uint64_t> command_counter{0};
  return verb + "#" + std::to_string(command_counter.fetch_add(1));
}

inline bool actionParamIndex(const std::string& key, int& index) {
  constexpr const char* prefix = "param";
  constexpr size_t prefix_size = 5;
  if (key.compare(0, prefix_size, prefix) != 0 || key.size() == prefix_size) {
    return false;
  }

  int value = 0;
  for (size_t i = prefix_size; i < key.size(); ++i) {
    const auto c = static_cast<unsigned char>(key[i]);
    if (std::isdigit(c) == 0) {
      return false;
    }
    value = value * 10 + static_cast<int>(key[i] - '0');
    if (value >= kMaxActionParams) {
      index = value;
      return true;
    }
  }
  index = value;
  return true;
}

template <typename NodeT>
void rejectUnsupportedActionParams(const NodeT& node) {
  for (const auto& [key, value] : node.config().input_ports) {
    int index = 0;
    if (actionParamIndex(key, index) && index >= kMaxActionParams &&
        !value.empty()) {
      throw std::runtime_error(
          "Action command parameter '" + key +
          "' exceeds supported positional parameter limit param0..param7");
    }
  }
}

template <typename NodeT>
ActionCommand buildActionCommand(NodeT& node,
                                 const std::string& verb,
                                 const std::string& command_id) {
  rejectUnsupportedActionParams(node);

  ActionCommand command;
  command.action_name = verb;
  command.service_name = verb;
  command.operation = verb;
  command.command_id = command_id;

  std::ostringstream signature;
  signature << verb << "(";
  bool first = true;
  for (int i = 0; i < kMaxActionParams; ++i) {
    const std::string key = "param" + std::to_string(i);
    std::string value;
    if (!node.getInput(key, value) || value.empty()) {
      continue;
    }
    command.request_fields[key] = value;
    if (!first) {
      signature << ",";
    }
    signature << value;
    first = false;
  }
  signature << ")";
  command.signature = signature.str();
  return command;
}

}  // namespace nodes
}  // namespace ame
