#include <gtest/gtest.h>

#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace provided = pyramid::components::tactical_objects::services::provided;
namespace ros2_support = pyramid::transport::ros2;

namespace {

struct TopicState {
  std::atomic<bool> called{false};
  std::mutex mutex;
  std::thread::id callback_thread_id;
  std::string payload;
  std::string type_name;
};

struct UnaryServiceState {
  std::atomic<bool> called{false};
  std::mutex mutex;
  std::thread::id handler_thread_id;
  std::string request_payload;
  std::string request_type_name;
};

struct StreamServiceState {
  std::atomic<bool> called{false};
  std::mutex mutex;
  std::thread::id handler_thread_id;
  std::string request_payload;
  std::vector<unsigned char> framed_response;
};

class FakeAdapter final : public ros2_support::Adapter {
 public:
  void subscribe(const ros2_support::TopicBinding& binding,
                 ros2_support::TopicHandler handler) override {
    topic_handlers_[binding.ros2_topic] = std::move(handler);
  }

  void advertise(const ros2_support::UnaryServiceBinding& binding,
                 ros2_support::UnaryHandler handler) override {
    unary_handlers_[binding.ros2_service] = std::move(handler);
  }

  void advertise(const ros2_support::StreamServiceBinding& binding,
                 ros2_support::StreamHandler handler) override {
    stream_handlers_[binding.ros2_open_service] = std::move(handler);
  }

  void publish(const ros2_support::TopicBinding& binding,
               const ros2_support::Envelope& envelope) override {
    published_.push_back({binding.ros2_topic, envelope});
  }

  void deliverTopic(const std::string& ros2_topic,
                    const ros2_support::Envelope& envelope) {
    topic_handlers_.at(ros2_topic)(envelope);
  }

  ros2_support::Envelope callUnary(const std::string& ros2_service,
                                   const ros2_support::Envelope& request) {
    return unary_handlers_.at(ros2_service)(request);
  }

  std::vector<ros2_support::Envelope> callStream(
      const std::string& ros2_open_service,
      const ros2_support::Envelope& request) {
    std::vector<ros2_support::Envelope> responses;
    stream_handlers_.at(ros2_open_service)(
        request, [&responses](const ros2_support::Envelope& envelope) {
          responses.push_back(envelope);
          return true;
        });
    return responses;
  }

 private:
  std::unordered_map<std::string, ros2_support::TopicHandler> topic_handlers_;
  std::unordered_map<std::string, ros2_support::UnaryHandler> unary_handlers_;
  std::unordered_map<std::string, ros2_support::StreamHandler> stream_handlers_;
  std::vector<std::pair<std::string, ros2_support::Envelope>> published_;
};

void appendVarint32(std::vector<unsigned char>& out, uint32_t value) {
  while (value >= 0x80U) {
    out.push_back(static_cast<unsigned char>(value | 0x80U));
    value >>= 7U;
  }
  out.push_back(static_cast<unsigned char>(value));
}

bool waitUntil(const std::function<bool()>& predicate, int timeout_ms = 1000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

void topicSubscriber(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* state = static_cast<TopicState*>(user_data);
  std::lock_guard<std::mutex> lock(state->mutex);
  state->callback_thread_id = std::this_thread::get_id();
  state->payload.assign(static_cast<const char*>(msg->data), msg->size);
  state->type_name = msg->type_name ? msg->type_name : "";
  state->called.store(true);
}

pcl_status_t configureTopicContainer(pcl_container_t* container, void* user_data) {
  auto* state = static_cast<TopicState*>(user_data);
  auto* port = pcl_container_add_subscriber(
      container, provided::kTopicEntityMatches, "application/protobuf",
      topicSubscriber, state);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

pcl_status_t handleCreateRequirement(pcl_container_t*, const pcl_msg_t* request,
                                     pcl_msg_t* response, pcl_svc_context_t*,
                                     void* user_data) {
  auto* state = static_cast<UnaryServiceState*>(user_data);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->handler_thread_id = std::this_thread::get_id();
    state->request_payload.assign(static_cast<const char*>(request->data),
                                  request->size);
    state->request_type_name = request->type_name ? request->type_name : "";
  }
  state->called.store(true);

  static constexpr const char* kResponsePayload = "ros2-interest-42";
  response->data = const_cast<char*>(kResponsePayload);
  response->size = static_cast<uint32_t>(std::strlen(kResponsePayload));
  response->type_name = request->type_name;
  return PCL_OK;
}

pcl_status_t configureUnaryContainer(pcl_container_t* container, void* user_data) {
  auto* port = pcl_container_add_service(
      container, provided::kSvcCreateRequirement, "application/protobuf",
      handleCreateRequirement, user_data);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

pcl_status_t handleReadMatch(pcl_container_t*, const pcl_msg_t* request,
                             pcl_msg_t* response, pcl_svc_context_t*,
                             void* user_data) {
  auto* state = static_cast<StreamServiceState*>(user_data);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->handler_thread_id = std::this_thread::get_id();
    state->request_payload.assign(static_cast<const char*>(request->data),
                                  request->size);
  }
  state->called.store(true);

  std::vector<unsigned char> framed;
  const std::string first = "match-alpha";
  const std::string second = "match-bravo";
  appendVarint32(framed, static_cast<uint32_t>(first.size()));
  framed.insert(framed.end(), first.begin(), first.end());
  appendVarint32(framed, static_cast<uint32_t>(second.size()));
  framed.insert(framed.end(), second.begin(), second.end());

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->framed_response = std::move(framed);
    response->data = state->framed_response.data();
    response->size = static_cast<uint32_t>(state->framed_response.size());
  }
  response->type_name = request->type_name;
  return PCL_OK;
}

pcl_status_t configureStreamContainer(pcl_container_t* container, void* user_data) {
  auto* port = pcl_container_add_service(
      container, provided::kSvcReadMatch, "application/protobuf", handleReadMatch,
      user_data);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

}  // namespace

TEST(Ros2TransportSemantics, CanonicalNameMappingUsesRos2Channels) {
  const auto topic =
      ros2_support::makeTopicBinding(provided::kTopicEntityMatches);
  EXPECT_EQ(topic.ros2_topic, "/pyramid/topic/standard/entity_matches");

  const auto unary =
      ros2_support::makeUnaryServiceBinding(provided::kSvcCreateRequirement);
  EXPECT_EQ(unary.ros2_service,
            "/pyramid/service/object_of_interest/create_requirement");

  const auto stream =
      ros2_support::makeStreamServiceBinding(provided::kSvcReadMatch);
  EXPECT_EQ(stream.ros2_open_service,
            "/pyramid/stream/matching_objects/read_match/open");
  EXPECT_EQ(stream.ros2_frame_topic,
            "/pyramid/stream/matching_objects/read_match/frames");
  EXPECT_EQ(stream.ros2_cancel_topic,
            "/pyramid/stream/matching_objects/read_match/cancel");
}

TEST(Ros2TransportSemantics, TopicIngressRunsSubscriberOnExecutorThread) {
  TopicState state;
  FakeAdapter adapter;

  pcl_callbacks_t callbacks{};
  callbacks.on_configure = configureTopicContainer;

  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);
  pcl_container_t* container =
      pcl_container_create("ros2_topic_ingress", &callbacks, &state);
  ASSERT_NE(container, nullptr);
  ASSERT_EQ(pcl_container_configure(container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(executor, container), PCL_OK);

  std::atomic<bool> stop{false};
  std::thread::id executor_thread_id;
  std::thread executor_thread([&] {
    executor_thread_id = std::this_thread::get_id();
    while (!stop.load()) {
      pcl_executor_spin_once(executor, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  provided::bindRos2(adapter, executor);
  const auto binding =
      ros2_support::makeTopicBinding(provided::kTopicEntityMatches);

  std::thread ingress_thread([&] {
    ros2_support::Envelope envelope;
    envelope.content_type = "application/protobuf";
    envelope.payload.assign({'r', 'o', 's', '2', '-', 'e', 'v', 'd'});
    adapter.deliverTopic(binding.ros2_topic, envelope);
  });
  ingress_thread.join();

  ASSERT_TRUE(waitUntil([&] { return state.called.load(); }));
  EXPECT_NE(executor_thread_id, std::thread::id{});
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.callback_thread_id, executor_thread_id);
    EXPECT_EQ(state.payload, "ros2-evd");
    EXPECT_EQ(state.type_name, "application/protobuf");
  }

  stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
}

TEST(Ros2TransportSemantics, UnaryServiceBridgeRunsHandlerOnExecutorThread) {
  UnaryServiceState state;
  FakeAdapter adapter;

  pcl_callbacks_t callbacks{};
  callbacks.on_configure = configureUnaryContainer;

  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);
  pcl_container_t* container =
      pcl_container_create("ros2_unary_service", &callbacks, &state);
  ASSERT_NE(container, nullptr);
  ASSERT_EQ(pcl_container_configure(container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(executor, container), PCL_OK);

  std::atomic<bool> stop{false};
  std::thread::id executor_thread_id;
  std::thread executor_thread([&] {
    executor_thread_id = std::this_thread::get_id();
    while (!stop.load()) {
      pcl_executor_spin_once(executor, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  provided::bindRos2(adapter, executor);

  ros2_support::Envelope request;
  request.content_type = "application/protobuf";
  request.correlation_id = "corr-42";
  request.payload.assign({'p', 'r', 'o', 't', 'o', '-', 'r', 'e', 'q'});

  std::thread::id client_thread_id;
  ros2_support::Envelope response;
  std::thread client_thread([&] {
    client_thread_id = std::this_thread::get_id();
    const auto binding = ros2_support::makeUnaryServiceBinding(
        provided::kSvcCreateRequirement);
    response = adapter.callUnary(binding.ros2_service, request);
  });
  client_thread.join();

  ASSERT_TRUE(waitUntil([&] { return state.called.load(); }));
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.handler_thread_id, executor_thread_id);
    EXPECT_NE(state.handler_thread_id, client_thread_id);
    EXPECT_EQ(state.request_payload, "proto-req");
    EXPECT_EQ(state.request_type_name, "application/protobuf");
  }
  EXPECT_EQ(response.correlation_id, "corr-42");
  EXPECT_EQ(response.content_type, "application/protobuf");
  EXPECT_EQ(std::string(response.payload.begin(), response.payload.end()),
            "ros2-interest-42");

  stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
}

TEST(Ros2TransportSemantics, StreamServiceBridgePreservesFramesAndExecutorThread) {
  StreamServiceState state;
  FakeAdapter adapter;

  pcl_callbacks_t callbacks{};
  callbacks.on_configure = configureStreamContainer;

  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);
  pcl_container_t* container =
      pcl_container_create("ros2_stream_service", &callbacks, &state);
  ASSERT_NE(container, nullptr);
  ASSERT_EQ(pcl_container_configure(container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(executor, container), PCL_OK);

  std::atomic<bool> stop{false};
  std::thread::id executor_thread_id;
  std::thread executor_thread([&] {
    executor_thread_id = std::this_thread::get_id();
    while (!stop.load()) {
      pcl_executor_spin_once(executor, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  provided::bindRos2(adapter, executor);

  ros2_support::Envelope request;
  request.content_type = "application/protobuf";
  request.correlation_id = "stream-7";
  request.payload.assign({'f', 'i', 'n', 'd', '-', 'm', 'e'});

  std::thread::id client_thread_id;
  std::vector<ros2_support::Envelope> frames;
  std::thread client_thread([&] {
    client_thread_id = std::this_thread::get_id();
    const auto binding =
        ros2_support::makeStreamServiceBinding(provided::kSvcReadMatch);
    frames = adapter.callStream(binding.ros2_open_service, request);
  });
  client_thread.join();

  ASSERT_TRUE(waitUntil([&] { return state.called.load(); }));
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.handler_thread_id, executor_thread_id);
    EXPECT_NE(state.handler_thread_id, client_thread_id);
    EXPECT_EQ(state.request_payload, "find-me");
  }

  ASSERT_EQ(frames.size(), 3U);
  EXPECT_EQ(std::string(frames[0].payload.begin(), frames[0].payload.end()),
            "match-alpha");
  EXPECT_EQ(std::string(frames[1].payload.begin(), frames[1].payload.end()),
            "match-bravo");
  EXPECT_TRUE(frames[2].end_of_stream);
  EXPECT_EQ(frames[0].correlation_id, "stream-7");
  EXPECT_EQ(frames[1].correlation_id, "stream-7");
  EXPECT_EQ(frames[2].correlation_id, "stream-7");
  EXPECT_EQ(frames[0].content_type, "application/protobuf");

  stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
}
