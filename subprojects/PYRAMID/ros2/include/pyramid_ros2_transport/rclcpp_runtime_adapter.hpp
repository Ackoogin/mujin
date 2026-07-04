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
  struct Options {
    bool use_envelope_wire = false;
  };

  explicit RclcppRuntimeAdapter(const rclcpp::Node::SharedPtr& node,
                                Options options = {});

  void subscribe(const TopicBinding& binding, TopicHandler handler) override;
  void advertise(const UnaryServiceBinding& binding,
                 UnaryHandler handler) override;
  void advertise(const StreamServiceBinding& binding,
                 StreamHandler handler) override;
  void publish(const TopicBinding& binding, const Envelope& envelope) override;

  // Consumed (client) side: call a remote ROS2 unary service and block for the
  // response (serviced by the transport's spin thread).
  Envelope invokeUnary(const UnaryServiceBinding& binding,
                       const Envelope& request) override;

  // Consumed streaming: open the remote server-streaming service and deliver each
  // published frame to on_frame until the terminal (end_of_stream) frame.
  void invokeStream(const StreamServiceBinding& binding, const Envelope& request,
                    const FrameSink& on_frame) override;

 private:
  using EnvelopeMsg = pyramid_ros2_transport::msg::PclEnvelope;
  using UnaryService = pyramid_ros2_transport::srv::PclService;
  using OpenStreamService = pyramid_ros2_transport::srv::PclOpenStream;

  static Envelope fromRosMessage(const EnvelopeMsg& msg);
  static EnvelopeMsg toRosMessage(const Envelope& envelope);
  static Envelope fromSerializedMessage(const rclcpp::SerializedMessage& msg);
  static rclcpp::SerializedMessage toSerializedMessage(const Envelope& envelope);
  static Envelope fromRosRequest(const UnaryService::Request& request);
  static void toRosResponse(const Envelope& envelope,
                            UnaryService::Response& response);
  static Envelope fromRosRequest(const OpenStreamService::Request& request);

  bool useTypedTopicWire(const TopicBinding& binding) const;
  bool useTypedTopicWire(const TopicBinding& binding,
                         const Envelope& envelope) const;

  std::string ensureCorrelationId(const std::string& requested_id);
  bool isCancelled(const std::string& correlation_id) const;
  void clearCancelled(const std::string& correlation_id);

  rclcpp::QoS topicQos(pcl_qos_t qos) const;
  rclcpp::QoS frameQos() const;

  rclcpp::Node::SharedPtr node_;
  Options options_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string,
                     rclcpp::Publisher<EnvelopeMsg>::SharedPtr>
      envelope_publishers_;
  std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr>
      typed_publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::unordered_map<std::string, rclcpp::Client<UnaryService>::SharedPtr>
      unary_clients_;
  std::unordered_map<std::string, rclcpp::Client<OpenStreamService>::SharedPtr>
      stream_clients_;
  std::unordered_set<std::string> cancelled_streams_;
  uint64_t next_correlation_id_ = 1U;
};

}  // namespace pyramid::transport::ros2
