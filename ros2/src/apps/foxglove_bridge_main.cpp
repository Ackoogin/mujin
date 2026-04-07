#include <ame/foxglove_bridge.h>
#include <ame_ros2/msg/world_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace {

std::string jsonEscape(const std::string& input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (char c : input) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

template <typename TimeMsgT>
uint64_t stampToMicroseconds(const TimeMsgT& stamp) {
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
  }
  return static_cast<uint64_t>(stamp.sec) * 1000000ULL + static_cast<uint64_t>(stamp.nanosec) / 1000ULL;
}

class FoxgloveBridgeNode : public rclcpp::Node {
public:
  FoxgloveBridgeNode()
  : rclcpp::Node("foxglove_bridge") {
    declare_parameter("port", 8765);
    declare_parameter("server_name", std::string("ame_ros2"));
    declare_parameter("bt_events_topic", std::string("/executor/bt_events"));
    declare_parameter("world_state_topic", std::string("/world_state"));

    int64_t port_value = get_parameter("port").as_int();
    if (port_value <= 0 || port_value > 65535) {
      RCLCPP_WARN(get_logger(), "Invalid port %lld, using 8765", static_cast<long long>(port_value));
      port_value = 8765;
    }

    const auto server_name = get_parameter("server_name").as_string();
    const auto bt_events_topic = get_parameter("bt_events_topic").as_string();
    const auto world_state_topic = get_parameter("world_state_topic").as_string();

    bridge_ = std::make_unique<ame::FoxgloveBridge>(
      ame::FoxgloveBridge::Options{static_cast<uint16_t>(port_value), server_name});
    bridge_->start();

    bt_sink_ = bridge_->btEventSink();
    wm_sink_ = bridge_->wmEventSink();

    sub_bt_events_ = create_subscription<std_msgs::msg::String>(
      bt_events_topic,
      rclcpp::QoS(50).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (bt_sink_) {
          bt_sink_(msg->data);
        }
      });

    sub_world_state_ = create_subscription<ame_ros2::msg::WorldState>(
      world_state_topic,
      rclcpp::QoS(10).reliable().transient_local(),
      [this](const ame_ros2::msg::WorldState::SharedPtr msg) {
        handleWorldState(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Foxglove bridge running on ws://localhost:%lld (bt_events=%s, world_state=%s)",
      static_cast<long long>(port_value),
      bt_events_topic.c_str(),
      world_state_topic.c_str());
  }

  ~FoxgloveBridgeNode() override {
    if (bridge_) {
      bridge_->stop();
    }
  }

private:
  void handleWorldState(const ame_ros2::msg::WorldState& msg) {
    std::unordered_map<std::string, ame_ros2::msg::WorldFact> current_true_facts;
    current_true_facts.reserve(msg.facts.size());
    for (const auto& fact : msg.facts) {
      current_true_facts.emplace(fact.key, fact);
    }

    const uint64_t ts_us = stampToMicroseconds(msg.header.stamp);

    if (!wm_initialized_) {
      for (const auto& [key, fact] : current_true_facts) {
        publishWmAudit(msg.wm_version, ts_us, key, true,
          fact.source.empty() ? std::string("world_state_init") : fact.source);
      }
      prev_true_facts_ = std::move(current_true_facts);
      wm_initialized_ = true;
      return;
    }

    for (const auto& [key, fact] : current_true_facts) {
      if (prev_true_facts_.find(key) == prev_true_facts_.end()) {
        publishWmAudit(msg.wm_version, ts_us, key, true,
          fact.source.empty() ? std::string("world_state_diff") : fact.source);
      }
    }

    for (const auto& [key, fact] : prev_true_facts_) {
      if (current_true_facts.find(key) == current_true_facts.end()) {
        publishWmAudit(msg.wm_version, ts_us, key, false,
          fact.source.empty() ? std::string("world_state_diff") : fact.source);
      }
    }

    prev_true_facts_ = std::move(current_true_facts);
  }

  void publishWmAudit(
    uint64_t wm_version,
    uint64_t ts_us,
    const std::string& fact,
    bool value,
    const std::string& source) {
    if (!wm_sink_) {
      return;
    }

    std::string json = std::string("{\"wm_version\":") + std::to_string(wm_version)
      + ",\"ts_us\":" + std::to_string(ts_us)
      + ",\"fact\":\"" + jsonEscape(fact) + "\""
      + ",\"value\":" + (value ? "true" : "false")
      + ",\"source\":\"" + jsonEscape(source) + "\"}";
    wm_sink_(json);
  }

  std::unique_ptr<ame::FoxgloveBridge> bridge_;
  ame::AmeBTLogger::SinkCallback bt_sink_;
  std::function<void(const std::string&)> wm_sink_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_bt_events_;
  rclcpp::Subscription<ame_ros2::msg::WorldState>::SharedPtr sub_world_state_;
  std::unordered_map<std::string, ame_ros2::msg::WorldFact> prev_true_facts_;
  bool wm_initialized_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FoxgloveBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
