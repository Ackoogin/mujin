#pragma once

#include <pcl/pcl_executor.h>
#include <pcl/pcl_types.h>

#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace pyramid::transport::ros2 {

inline constexpr const char* kTransportContentType = "application/ros2";
inline constexpr const char* kEnvelopeType = "pyramid_ros2/PclEnvelope";

struct Envelope {
  std::string content_type;
  std::string correlation_id;
  std::vector<unsigned char> payload;
  bool end_of_stream = false;
  pcl_status_t status = PCL_OK;
};

struct TopicBinding {
  std::string pcl_topic;
  std::string ros2_topic;
};

struct UnaryServiceBinding {
  std::string pcl_service;
  std::string ros2_service;
};

struct StreamServiceBinding {
  std::string pcl_service;
  std::string ros2_open_service;
  std::string ros2_frame_topic;
  std::string ros2_cancel_topic;
};

using TopicHandler = std::function<void(const Envelope&)>;
using UnaryHandler = std::function<Envelope(const Envelope&)>;
using StreamEmitter = std::function<bool(const Envelope&)>;
using StreamHandler = std::function<void(const Envelope&, const StreamEmitter&)>;

class Adapter {
 public:
  virtual ~Adapter() = default;

  virtual void subscribe(const TopicBinding& binding, TopicHandler handler) = 0;
  virtual void advertise(const UnaryServiceBinding& binding,
                         UnaryHandler handler) = 0;
  virtual void advertise(const StreamServiceBinding& binding,
                         StreamHandler handler) = 0;
  virtual void publish(const TopicBinding& binding, const Envelope& envelope) = 0;
};

TopicBinding makeTopicBinding(std::string_view pcl_topic);
UnaryServiceBinding makeUnaryServiceBinding(std::string_view pcl_service);
StreamServiceBinding makeStreamServiceBinding(std::string_view pcl_service);

Envelope envelopeFromMessage(const pcl_msg_t* msg);
pcl_status_t postIncomingEnvelope(pcl_executor_t* executor, const char* pcl_topic,
                                  const Envelope& envelope);
pcl_status_t publishOutboundEnvelope(Adapter& adapter, const char* pcl_topic,
                                     const pcl_msg_t* msg);
Envelope invokeUnaryOnExecutor(pcl_executor_t* executor, const char* pcl_service,
                               const Envelope& request);
void invokeStreamOnExecutor(pcl_executor_t* executor, const char* pcl_service,
                            const Envelope& request,
                            const StreamEmitter& emit_frame);

void bindTopicIngress(Adapter& adapter, pcl_executor_t* executor,
                      const char* pcl_topic);
void bindUnaryServiceIngress(Adapter& adapter, pcl_executor_t* executor,
                             const char* pcl_service);
void bindStreamServiceIngress(Adapter& adapter, pcl_executor_t* executor,
                              const char* pcl_service);

}  // namespace pyramid::transport::ros2
