#include "pyramid_ros2_transport/rclcpp_runtime_adapter.hpp"

#include <utility>

namespace pyramid::transport::ros2 {

RclcppRuntimeAdapter::RclcppRuntimeAdapter(const rclcpp::Node::SharedPtr& node)
    : node_(node) {}

Envelope RclcppRuntimeAdapter::fromRosMessage(const EnvelopeMsg& msg) {
  Envelope envelope;
  envelope.content_type = msg.content_type;
  envelope.correlation_id = msg.correlation_id;
  envelope.payload.assign(msg.payload.begin(), msg.payload.end());
  envelope.end_of_stream = msg.end_of_stream;
  envelope.status = static_cast<pcl_status_t>(msg.status);
  return envelope;
}

RclcppRuntimeAdapter::EnvelopeMsg RclcppRuntimeAdapter::toRosMessage(
    const Envelope& envelope) {
  EnvelopeMsg msg;
  msg.content_type = envelope.content_type;
  msg.correlation_id = envelope.correlation_id;
  msg.payload.assign(envelope.payload.begin(), envelope.payload.end());
  msg.end_of_stream = envelope.end_of_stream;
  msg.status = static_cast<int32_t>(envelope.status);
  return msg;
}

Envelope RclcppRuntimeAdapter::fromRosRequest(
    const UnaryService::Request& request) {
  Envelope envelope;
  envelope.content_type = request.content_type;
  envelope.correlation_id = request.correlation_id;
  envelope.payload.assign(request.payload.begin(), request.payload.end());
  return envelope;
}

void RclcppRuntimeAdapter::toRosResponse(const Envelope& envelope,
                                         UnaryService::Response& response) {
  response.content_type = envelope.content_type;
  response.correlation_id = envelope.correlation_id;
  response.payload.assign(envelope.payload.begin(), envelope.payload.end());
  response.status = static_cast<int32_t>(envelope.status);
}

Envelope RclcppRuntimeAdapter::fromRosRequest(
    const OpenStreamService::Request& request) {
  Envelope envelope;
  envelope.content_type = request.content_type;
  envelope.correlation_id = request.correlation_id;
  envelope.payload.assign(request.payload.begin(), request.payload.end());
  return envelope;
}

std::string RclcppRuntimeAdapter::ensureCorrelationId(
    const std::string& requested_id) {
  if (!requested_id.empty()) {
    return requested_id;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return "ros2-stream-" + std::to_string(next_correlation_id_++);
}

bool RclcppRuntimeAdapter::isCancelled(
    const std::string& correlation_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cancelled_streams_.find(correlation_id) != cancelled_streams_.end();
}

void RclcppRuntimeAdapter::clearCancelled(
    const std::string& correlation_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  cancelled_streams_.erase(correlation_id);
}

rclcpp::QoS RclcppRuntimeAdapter::topicQos() const {
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

rclcpp::QoS RclcppRuntimeAdapter::frameQos() const {
  return rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
}

void RclcppRuntimeAdapter::subscribe(const TopicBinding& binding,
                                     TopicHandler handler) {
  auto subscription = node_->create_subscription<EnvelopeMsg>(
      binding.ros2_topic, topicQos(),
      [handler = std::move(handler)](const EnvelopeMsg::SharedPtr msg) {
        handler(fromRosMessage(*msg));
      });
  subscriptions_.push_back(subscription);
}

void RclcppRuntimeAdapter::advertise(const UnaryServiceBinding& binding,
                                     UnaryHandler handler) {
  auto service = node_->create_service<UnaryService>(
      binding.ros2_service,
      [handler = std::move(handler)](
          const UnaryService::Request::SharedPtr request,
          UnaryService::Response::SharedPtr response) {
        auto envelope = handler(fromRosRequest(*request));
        toRosResponse(envelope, *response);
      });
  services_.push_back(service);
}

void RclcppRuntimeAdapter::advertise(const StreamServiceBinding& binding,
                                     StreamHandler handler) {
  auto frame_publisher = node_->create_publisher<EnvelopeMsg>(
      binding.ros2_frame_topic, frameQos());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    envelope_publishers_[binding.ros2_frame_topic] = frame_publisher;
  }

  auto cancel_subscription = node_->create_subscription<EnvelopeMsg>(
      binding.ros2_cancel_topic, topicQos(),
      [this](const EnvelopeMsg::SharedPtr msg) {
        if (msg->correlation_id.empty()) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_streams_.insert(msg->correlation_id);
      });
  subscriptions_.push_back(cancel_subscription);

  auto service = node_->create_service<OpenStreamService>(
      binding.ros2_open_service,
      [this, frame_publisher, handler = std::move(handler)](
          const OpenStreamService::Request::SharedPtr request,
          OpenStreamService::Response::SharedPtr response) {
        auto envelope = fromRosRequest(*request);
        const auto correlation_id = ensureCorrelationId(envelope.correlation_id);
        envelope.correlation_id = correlation_id;

        pcl_status_t final_status = PCL_OK;
        const auto emit = [this, frame_publisher, correlation_id, &final_status](
                              const Envelope& frame) {
          if (isCancelled(correlation_id)) {
            final_status = PCL_ERR_INVALID;
            return false;
          }
          Envelope published = frame;
          published.correlation_id = correlation_id;
          frame_publisher->publish(toRosMessage(published));
          final_status = published.status;
          return true;
        };

        handler(envelope, emit);
        response->accepted = true;
        response->status = static_cast<int32_t>(final_status);
        response->correlation_id = correlation_id;
        clearCancelled(correlation_id);
      });
  services_.push_back(service);
}

void RclcppRuntimeAdapter::publish(const TopicBinding& binding,
                                   const Envelope& envelope) {
  rclcpp::Publisher<EnvelopeMsg>::SharedPtr publisher;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = envelope_publishers_.find(binding.ros2_topic);
    if (it == envelope_publishers_.end()) {
      publisher =
          node_->create_publisher<EnvelopeMsg>(binding.ros2_topic, topicQos());
      envelope_publishers_.emplace(binding.ros2_topic, publisher);
    } else {
      publisher = it->second;
    }
  }
  publisher->publish(toRosMessage(envelope));
}

}  // namespace pyramid::transport::ros2
