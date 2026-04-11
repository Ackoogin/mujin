#pragma once

#include "pyramid_ros2_transport/msg/pcl_envelope.hpp"
#include "pyramid_ros2_transport/srv/pcl_open_stream.hpp"
#include "pyramid_ros2_transport/srv/pcl_service.hpp"

#include "pyramid_ros2_transport_support.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pyramid::transport::ros2 {

class RclcppRuntimeAdapter final : public Adapter {
 public:
  explicit RclcppRuntimeAdapter(const rclcpp::Node::SharedPtr& node);

  void subscribe(const TopicBinding& binding, TopicHandler handler) override;
  void advertise(const UnaryServiceBinding& binding,
                 UnaryHandler handler) override;
  void advertise(const StreamServiceBinding& binding,
                 StreamHandler handler) override;
  void publish(const TopicBinding& binding, const Envelope& envelope) override;

 private:
  using EnvelopeMsg = pyramid_ros2_transport::msg::PclEnvelope;
  using UnaryService = pyramid_ros2_transport::srv::PclService;
  using OpenStreamService = pyramid_ros2_transport::srv::PclOpenStream;

  static Envelope fromRosMessage(const EnvelopeMsg& msg);
  static EnvelopeMsg toRosMessage(const Envelope& envelope);
  static Envelope fromRosRequest(const UnaryService::Request& request);
  static void toRosResponse(const Envelope& envelope,
                            UnaryService::Response& response);
  static Envelope fromRosRequest(const OpenStreamService::Request& request);

  std::string ensureCorrelationId(const std::string& requested_id);
  bool isCancelled(const std::string& correlation_id) const;
  void clearCancelled(const std::string& correlation_id);

  rclcpp::QoS topicQos() const;
  rclcpp::QoS frameQos() const;

  rclcpp::Node::SharedPtr node_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string,
                     rclcpp::Publisher<EnvelopeMsg>::SharedPtr>
      envelope_publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::unordered_set<std::string> cancelled_streams_;
  uint64_t next_correlation_id_ = 1U;
};

}  // namespace pyramid::transport::ros2
