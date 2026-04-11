#include "pyramid_ros2_transport_support.hpp"

#include <cctype>
#include <future>
#include <stdexcept>

namespace pyramid::transport::ros2 {

namespace {

std::string sanitizeName(std::string_view value) {
  std::string out;
  out.reserve(value.size() + 8U);
  for (char ch : value) {
    const auto uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) != 0) {
      out.push_back(static_cast<char>(std::tolower(uch)));
    } else if (ch == '.' || ch == '/') {
      if (out.empty() || out.back() != '/') {
        out.push_back('/');
      }
    } else if (ch == '_') {
      out.push_back('_');
    } else {
      out.push_back('_');
    }
  }
  while (!out.empty() && out.back() == '/') {
    out.pop_back();
  }
  return out;
}

bool readVarint32(const unsigned char*& cursor, const unsigned char* end,
                  uint32_t& value) {
  value = 0U;
  uint32_t shift = 0U;
  while (cursor < end && shift <= 28U) {
    const uint8_t byte = *cursor++;
    value |= static_cast<uint32_t>(byte & 0x7FU) << shift;
    if ((byte & 0x80U) == 0U) {
      return true;
    }
    shift += 7U;
  }
  return false;
}

struct UnaryState {
  std::promise<Envelope> promise;
};

void unaryResponseCallback(const pcl_msg_t* response, void* user_data) {
  auto* state = static_cast<UnaryState*>(user_data);
  if (!state) {
    return;
  }
  Envelope envelope;
  if (response && response->type_name) {
    envelope.content_type = response->type_name;
  }
  if (response && response->data && response->size > 0U) {
    const auto* begin =
        static_cast<const unsigned char*>(response->data);
    envelope.payload.assign(begin, begin + response->size);
  }
  state->promise.set_value(std::move(envelope));
}

}  // namespace

TopicBinding makeTopicBinding(std::string_view pcl_topic) {
  return {std::string(pcl_topic),
          std::string("/pyramid/topic/") + sanitizeName(pcl_topic)};
}

UnaryServiceBinding makeUnaryServiceBinding(std::string_view pcl_service) {
  return {std::string(pcl_service),
          std::string("/pyramid/service/") + sanitizeName(pcl_service)};
}

StreamServiceBinding makeStreamServiceBinding(std::string_view pcl_service) {
  const auto base = std::string("/pyramid/stream/") + sanitizeName(pcl_service);
  return {
      std::string(pcl_service),
      base + "/open",
      base + "/frames",
      base + "/cancel",
  };
}

Envelope envelopeFromMessage(const pcl_msg_t* msg) {
  Envelope envelope;
  if (!msg) {
    envelope.status = PCL_ERR_INVALID;
    return envelope;
  }
  if (msg->type_name) {
    envelope.content_type = msg->type_name;
  }
  if (msg->data && msg->size > 0U) {
    const auto* begin = static_cast<const unsigned char*>(msg->data);
    envelope.payload.assign(begin, begin + msg->size);
  }
  return envelope;
}

pcl_status_t postIncomingEnvelope(pcl_executor_t* executor, const char* pcl_topic,
                                  const Envelope& envelope) {
  if (!executor || !pcl_topic || !pcl_topic[0]) {
    return PCL_ERR_INVALID;
  }
  pcl_msg_t msg{};
  msg.data = envelope.payload.empty() ? nullptr : envelope.payload.data();
  msg.size = static_cast<uint32_t>(envelope.payload.size());
  msg.type_name =
      envelope.content_type.empty() ? nullptr : envelope.content_type.c_str();
  return pcl_executor_post_incoming(executor, pcl_topic, &msg);
}

pcl_status_t publishOutboundEnvelope(Adapter& adapter, const char* pcl_topic,
                                     const pcl_msg_t* msg) {
  if (!pcl_topic || !msg) {
    return PCL_ERR_INVALID;
  }
  adapter.publish(makeTopicBinding(pcl_topic), envelopeFromMessage(msg));
  return PCL_OK;
}

Envelope invokeUnaryOnExecutor(pcl_executor_t* executor, const char* pcl_service,
                               const Envelope& request) {
  Envelope error;
  if (!executor || !pcl_service || !pcl_service[0]) {
    error.status = PCL_ERR_INVALID;
    return error;
  }

  UnaryState state;
  pcl_msg_t request_msg{};
  request_msg.data = request.payload.empty() ? nullptr : request.payload.data();
  request_msg.size = static_cast<uint32_t>(request.payload.size());
  request_msg.type_name =
      request.content_type.empty() ? nullptr : request.content_type.c_str();

  const auto rc = pcl_executor_post_service_request(
      executor, pcl_service, &request_msg, unaryResponseCallback, &state);
  if (rc != PCL_OK) {
    error.status = rc;
    return error;
  }

  auto result = state.promise.get_future().get();
  result.correlation_id = request.correlation_id;
  return result;
}

void invokeStreamOnExecutor(pcl_executor_t* executor, const char* pcl_service,
                            const Envelope& request,
                            const StreamEmitter& emit_frame) {
  if (!emit_frame) {
    return;
  }

  auto response = invokeUnaryOnExecutor(executor, pcl_service, request);
  if (response.status != PCL_OK) {
    response.end_of_stream = true;
    emit_frame(response);
    return;
  }

  const auto* cursor = response.payload.data();
  const auto* end = cursor + response.payload.size();
  while (cursor < end) {
    uint32_t frame_size = 0U;
    if (!readVarint32(cursor, end, frame_size) ||
        static_cast<size_t>(end - cursor) < frame_size) {
      Envelope terminal;
      terminal.content_type = response.content_type;
      terminal.correlation_id = request.correlation_id;
      terminal.status = PCL_ERR_INVALID;
      terminal.end_of_stream = true;
      emit_frame(terminal);
      return;
    }

    Envelope frame;
    frame.content_type = response.content_type;
    frame.correlation_id = request.correlation_id;
    frame.payload.assign(cursor, cursor + frame_size);
    cursor += frame_size;

    if (!emit_frame(frame)) {
      Envelope terminal;
      terminal.content_type = response.content_type;
      terminal.correlation_id = request.correlation_id;
      terminal.status = PCL_ERR_INVALID;
      terminal.end_of_stream = true;
      emit_frame(terminal);
      return;
    }
  }

  Envelope terminal;
  terminal.content_type = response.content_type;
  terminal.correlation_id = request.correlation_id;
  terminal.end_of_stream = true;
  emit_frame(terminal);
}

void bindTopicIngress(Adapter& adapter, pcl_executor_t* executor,
                      const char* pcl_topic) {
  const auto binding = makeTopicBinding(pcl_topic);
  adapter.subscribe(binding, [executor, topic = std::string(pcl_topic)](
                                  const Envelope& envelope) {
    postIncomingEnvelope(executor, topic.c_str(), envelope);
  });
}

void bindUnaryServiceIngress(Adapter& adapter, pcl_executor_t* executor,
                             const char* pcl_service) {
  const auto binding = makeUnaryServiceBinding(pcl_service);
  adapter.advertise(binding,
                    [executor, service = std::string(pcl_service)](
                        const Envelope& request) {
                      return invokeUnaryOnExecutor(executor, service.c_str(),
                                                   request);
                    });
}

void bindStreamServiceIngress(Adapter& adapter, pcl_executor_t* executor,
                              const char* pcl_service) {
  const auto binding = makeStreamServiceBinding(pcl_service);
  adapter.advertise(binding,
                    [executor, service = std::string(pcl_service)](
                        const Envelope& request, const StreamEmitter& emit) {
                      invokeStreamOnExecutor(executor, service.c_str(), request,
                                             emit);
                    });
}

}  // namespace pyramid::transport::ros2
