#include <gtest/gtest.h>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "pyramid_components_tactical_objects_services_provided_ros2_transport.hpp"
#include "pyramid_ros2_transport/msg/pcl_envelope.hpp"
#include "pyramid_ros2_transport/rclcpp_runtime_adapter.hpp"
#include "pyramid_ros2_transport/srv/pcl_open_stream.hpp"
#include "pyramid_ros2_transport/srv/pcl_service.hpp"
#include "pyramid_msgs/msg/object_detail.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cstdio>
#include <future>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace provided_ros2 =
    pyramid::components::tactical_objects::services::provided::ros2_transport;
namespace ros2_support = pyramid::transport::ros2;
namespace ros2_msg = pyramid_ros2_transport::msg;
namespace ros2_srv = pyramid_ros2_transport::srv;
namespace typed_msg = pyramid_msgs::msg;

namespace {

constexpr const char* kTopicObjectEvidence = "standard.object_evidence";
constexpr const char* kSvcCreateRequirement =
    "object_of_interest.create_requirement";
constexpr const char* kSvcReadMatch = "matching_objects.read_match";

struct TopicState {
  std::atomic<bool> called{false};
  std::mutex mutex;
  std::thread::id thread_id;
  std::string payload;
  std::string type_name;
};

struct TopicPerfState {
  std::mutex mutex;
  std::condition_variable cv;
  bool complete = false;
  bool valid = false;
  std::string expected_id;
  std::chrono::steady_clock::time_point completed_at{};
};

struct UnaryState {
  std::atomic<bool> called{false};
  std::mutex mutex;
  std::thread::id thread_id;
  std::string request_payload;
};

struct StreamState {
  std::atomic<bool> called{false};
  std::mutex mutex;
  std::thread::id thread_id;
  std::string request_payload;
  std::vector<unsigned char> framed;
};

void appendVarint32(std::vector<unsigned char>& out, uint32_t value) {
  while (value >= 0x80U) {
    out.push_back(static_cast<unsigned char>(value | 0x80U));
    value >>= 7U;
  }
  out.push_back(static_cast<unsigned char>(value));
}

template <class MsgT>
std::vector<unsigned char> serializeTypedMessage(const MsgT& msg) {
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);
  const auto& rcl = serialized.get_rcl_serialized_message();
  return {rcl.buffer, rcl.buffer + rcl.buffer_length};
}

template <class MsgT>
MsgT deserializeTypedMessage(const std::vector<unsigned char>& payload) {
  rclcpp::SerializedMessage serialized(payload.size());
  auto& rcl = serialized.get_rcl_serialized_message();
  if (!payload.empty()) {
    std::memcpy(rcl.buffer, payload.data(), payload.size());
  }
  rcl.buffer_length = payload.size();
  rclcpp::Serialization<MsgT> serializer;
  MsgT msg;
  serializer.deserialize_message(&serialized, &msg);
  return msg;
}

bool waitUntil(const std::function<bool()>& predicate, int timeout_ms = 2000) {
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

void topicCallback(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* state = static_cast<TopicState*>(user_data);
  std::lock_guard<std::mutex> lock(state->mutex);
  state->thread_id = std::this_thread::get_id();
  state->payload.assign(static_cast<const char*>(msg->data), msg->size);
  state->type_name = msg->type_name ? msg->type_name : "";
  state->called.store(true);
}

void topicPerfCallback(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* state = static_cast<TopicPerfState*>(user_data);
  bool valid = false;
  if (msg != nullptr && msg->data != nullptr && msg->type_name != nullptr &&
      std::strcmp(msg->type_name, "application/ros2") == 0) {
    const auto payload = std::vector<unsigned char>(
        static_cast<const unsigned char*>(msg->data),
        static_cast<const unsigned char*>(msg->data) + msg->size);
    const auto decoded =
        deserializeTypedMessage<typed_msg::ObjectDetail>(payload);
    valid = decoded.id == state->expected_id;
  }
  const auto completed_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->valid = valid;
    state->completed_at = completed_at;
    state->complete = true;
  }
  state->cv.notify_all();
}

pcl_status_t onConfigureTopicPerf(pcl_container_t* container,
                                  void* user_data) {
  auto* port = pcl_container_add_subscriber(
      container, kTopicObjectEvidence, "application/ros2", topicPerfCallback,
      user_data);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

pcl_status_t onConfigureTopic(pcl_container_t* container, void* user_data) {
  auto* port = pcl_container_add_subscriber(
      container, kTopicObjectEvidence, "application/protobuf", topicCallback,
      user_data);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

pcl_status_t onUnaryService(pcl_container_t*, const pcl_msg_t* request,
                            pcl_msg_t* response, pcl_svc_context_t*, void* ud) {
  auto* state = static_cast<UnaryState*>(ud);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->thread_id = std::this_thread::get_id();
    state->request_payload.assign(static_cast<const char*>(request->data),
                                  request->size);
  }
  state->called.store(true);

  static constexpr const char* kResponse = "ros2-runtime-ok";
  response->data = const_cast<char*>(kResponse);
  response->size = static_cast<uint32_t>(std::strlen(kResponse));
  response->type_name = request->type_name;
  return PCL_OK;
}

pcl_status_t onConfigureUnary(pcl_container_t* container, void* user_data) {
  auto* port = pcl_container_add_service(container, kSvcCreateRequirement,
                                         "application/protobuf", onUnaryService,
                                         user_data);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

pcl_status_t onStreamService(pcl_container_t*, const pcl_msg_t* request,
                             pcl_msg_t* response, pcl_svc_context_t*, void* ud) {
  auto* state = static_cast<StreamState*>(ud);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->thread_id = std::this_thread::get_id();
    state->request_payload.assign(static_cast<const char*>(request->data),
                                  request->size);
    state->framed.clear();
    const std::string first = "alpha";
    const std::string second = "bravo";
    appendVarint32(state->framed, static_cast<uint32_t>(first.size()));
    state->framed.insert(state->framed.end(), first.begin(), first.end());
    appendVarint32(state->framed, static_cast<uint32_t>(second.size()));
    state->framed.insert(state->framed.end(), second.begin(), second.end());
    response->data = state->framed.data();
    response->size = static_cast<uint32_t>(state->framed.size());
  }
  state->called.store(true);
  response->type_name = request->type_name;
  return PCL_OK;
}

pcl_status_t onConfigureStream(pcl_container_t* container, void* user_data) {
  auto* port = pcl_container_add_service(container, kSvcReadMatch,
                                         "application/protobuf", onStreamService,
                                         user_data);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

class RuntimeAdapterFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    server_node_ = std::make_shared<rclcpp::Node>("pyramid_ros2_server");
    client_node_ = std::make_shared<rclcpp::Node>("pyramid_ros2_client");
    adapter_ =
        std::make_unique<ros2_support::RclcppRuntimeAdapter>(server_node_);

    ros_executor_ =
        std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    ros_executor_->add_node(server_node_);
    ros_executor_->add_node(client_node_);

    ros_thread_ = std::thread([this] { ros_executor_->spin(); });
  }

  void TearDown() override {
    if (ros_executor_) {
      ros_executor_->cancel();
    }
    if (ros_thread_.joinable()) {
      ros_thread_.join();
    }
    ros_executor_.reset();
    adapter_.reset();
    client_node_.reset();
    server_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void startPclExecutor(pcl_callbacks_t callbacks, void* user_data,
                        const char* name) {
    executor_ = pcl_executor_create();
    ASSERT_NE(executor_, nullptr);
    container_ = pcl_container_create(name, &callbacks, user_data);
    ASSERT_NE(container_, nullptr);
    ASSERT_EQ(pcl_container_configure(container_), PCL_OK);
    ASSERT_EQ(pcl_container_activate(container_), PCL_OK);
    ASSERT_EQ(pcl_executor_add(executor_, container_), PCL_OK);

    pcl_stop_.store(false);
    pcl_thread_ = std::thread([this] {
      pcl_thread_id_ = std::this_thread::get_id();
      while (!pcl_stop_.load()) {
        pcl_executor_spin_once(executor_, 0);
        std::this_thread::yield();
      }
    });
  }

  void stopPclExecutor() {
    pcl_stop_.store(true);
    if (pcl_thread_.joinable()) {
      pcl_thread_.join();
    }
    if (container_) {
      pcl_container_destroy(container_);
      container_ = nullptr;
    }
    if (executor_) {
      pcl_executor_destroy(executor_);
      executor_ = nullptr;
    }
    pcl_thread_id_ = {};
  }

  std::shared_ptr<rclcpp::Node> server_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::unique_ptr<ros2_support::RclcppRuntimeAdapter> adapter_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  std::thread ros_thread_;

  pcl_executor_t* executor_ = nullptr;
  pcl_container_t* container_ = nullptr;
  std::atomic<bool> pcl_stop_{false};
  std::thread pcl_thread_;
  std::thread::id pcl_thread_id_;
};

}  // namespace

struct TopicPerfStats {
  double average_us = 0.0;
  double minimum_us = 0.0;
  double p50_us = 0.0;
  double p99_us = 0.0;
};

template <typename Publish>
TopicPerfStats runTopicPerformance(TopicPerfState* state, Publish&& publish) {
  constexpr int kWarmups = 50;
  constexpr int kSamples = 500;
  std::vector<double> samples;
  samples.reserve(kSamples);
  for (int i = -kWarmups; i < kSamples; ++i) {
    typed_msg::ObjectDetail message;
    message.id = "fastdds-perf-" + std::to_string(i);
    message.entity_source = "perf-client";
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->complete = false;
      state->valid = false;
      state->expected_id = message.id;
    }
    const auto started_at = std::chrono::steady_clock::now();
    publish(message);
    std::chrono::steady_clock::time_point completed_at;
    bool valid = false;
    {
      std::unique_lock<std::mutex> lock(state->mutex);
      const bool complete = state->cv.wait_for(
          lock, std::chrono::seconds(2), [&] { return state->complete; });
      valid = complete && state->valid;
      completed_at = state->completed_at;
    }
    EXPECT_TRUE(valid) << "Fast DDS sample " << i << " was not delivered";
    if (i >= 0 && valid) {
      samples.push_back(std::chrono::duration<double, std::micro>(
          completed_at - started_at).count());
    }
  }
  EXPECT_EQ(samples.size(), 500u);
  std::sort(samples.begin(), samples.end());
  TopicPerfStats stats;
  if (samples.empty()) {
    return stats;
  }
  double sum = 0.0;
  for (double sample : samples) {
    sum += sample;
  }
  stats.average_us = sum / static_cast<double>(samples.size());
  stats.minimum_us = samples.front();
  stats.p50_us = samples[samples.size() / 2u];
  stats.p99_us = samples[static_cast<size_t>(samples.size() * 0.99)];
  return stats;
}

void printTopicPerformance(const char* label, const TopicPerfStats& stats) {
  std::printf("[PERF] %-36s avg=%7.1f us min=%7.1f us "
              "p50=%7.1f us p99=%7.1f us\n",
              label, stats.average_us, stats.minimum_us, stats.p50_us,
              stats.p99_us);
}

TEST_F(RuntimeAdapterFixture, FastDds_RawRclcpp_TypedPubsub) {
  const auto binding = ros2_support::makeTopicBinding(kTopicObjectEvidence);
  TopicPerfState state;
  auto subscription = server_node_->create_subscription<typed_msg::ObjectDetail>(
      binding.ros2_topic, 10,
      [&state](const typed_msg::ObjectDetail::SharedPtr message) {
        const auto completed_at = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(state.mutex);
          state.valid = message->id == state.expected_id;
          state.completed_at = completed_at;
          state.complete = true;
        }
        state.cv.notify_all();
      });
  auto publisher = client_node_->create_publisher<typed_msg::ObjectDetail>(
      binding.ros2_topic, 10);
  ASSERT_TRUE(waitUntil([&] { return publisher->get_subscription_count() > 0u; }));
  const auto stats = runTopicPerformance(
      &state, [&](const typed_msg::ObjectDetail& message) {
        publisher->publish(message);
      });
  printTopicPerformance("raw rclcpp / Fast DDS / typed CDR", stats);
  EXPECT_GT(stats.average_us, 0.0);
  (void)subscription;
}

TEST_F(RuntimeAdapterFixture, FastDds_PyramidPort_TypedPubsub) {
  TopicPerfState state;
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigureTopicPerf;
  startPclExecutor(callbacks, &state, "fastdds_pyramid_port_perf");
  ros2_support::bindTopicIngress(*adapter_, executor_, kTopicObjectEvidence);
  const auto binding = ros2_support::makeTopicBinding(kTopicObjectEvidence);
  auto publisher = client_node_->create_publisher<typed_msg::ObjectDetail>(
      binding.ros2_topic, 10);
  ASSERT_TRUE(waitUntil([&] { return publisher->get_subscription_count() > 0u; }));
  const auto stats = runTopicPerformance(
      &state, [&](const typed_msg::ObjectDetail& message) {
        publisher->publish(message);
      });
  printTopicPerformance("PYRAMID port / Fast DDS / typed CDR", stats);
  EXPECT_GT(stats.average_us, 0.0);
  stopPclExecutor();
}

TEST_F(RuntimeAdapterFixture, TopicIngressRunsSubscriberOnPclExecutorThread) {
  TopicState state;
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigureTopic;
  startPclExecutor(callbacks, &state, "ros2_topic_ingress_runtime");

  ros2_support::bindTopicIngress(*adapter_, executor_, kTopicObjectEvidence);
  const auto binding = ros2_support::makeTopicBinding(kTopicObjectEvidence);
  ASSERT_EQ(binding.ros2_message_type, "pyramid_msgs/msg/ObjectDetail");

  auto publisher =
      client_node_->create_publisher<typed_msg::ObjectDetail>(
          binding.ros2_topic, 10);
  typed_msg::ObjectDetail msg;
  msg.id = "typed-evidence-1";
  msg.entity_source = "sensor-a";
  publisher->publish(msg);

  ASSERT_TRUE(waitUntil([&] { return state.called.load(); }));
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.thread_id, pcl_thread_id_);
    const auto payload = std::vector<unsigned char>(
        state.payload.begin(), state.payload.end());
    const auto decoded = deserializeTypedMessage<typed_msg::ObjectDetail>(
        payload);
    EXPECT_EQ(decoded.id, "typed-evidence-1");
    EXPECT_EQ(decoded.entity_source, "sensor-a");
    EXPECT_EQ(state.type_name, "application/ros2");
  }

  stopPclExecutor();
}

TEST_F(RuntimeAdapterFixture, GeneratedUnaryBindingServesRos2ClientViaPclExecutor) {
  UnaryState state;
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigureUnary;
  startPclExecutor(callbacks, &state, "ros2_unary_runtime");

  provided_ros2::ServiceBinder binder(*adapter_, executor_);
  binder.bind();

  const auto binding = ros2_support::makeUnaryServiceBinding(
      kSvcCreateRequirement);
  auto client =
      client_node_->create_client<ros2_srv::PclService>(binding.ros2_service);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto request = std::make_shared<ros2_srv::PclService::Request>();
  request->content_type = "application/protobuf";
  request->correlation_id = "req-42";
  request->payload.assign({'r', 'e', 'q'});

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto response = future.get();

  ASSERT_TRUE(waitUntil([&] { return state.called.load(); }));
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.thread_id, pcl_thread_id_);
    EXPECT_EQ(state.request_payload, "req");
  }
  EXPECT_EQ(response->correlation_id, "req-42");
  EXPECT_EQ(response->content_type, "application/protobuf");
  EXPECT_EQ(std::string(response->payload.begin(), response->payload.end()),
            "ros2-runtime-ok");
  EXPECT_EQ(response->status, 0);

  stopPclExecutor();
}

TEST_F(RuntimeAdapterFixture, StreamBindingPublishesFramesForRos2Client) {
  StreamState state;
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigureStream;
  startPclExecutor(callbacks, &state, "ros2_stream_runtime");

  provided_ros2::ServiceBinder binder(*adapter_, executor_);
  binder.bind();

  const auto binding = ros2_support::makeStreamServiceBinding(kSvcReadMatch);
  auto frames = std::make_shared<std::vector<ros2_msg::PclEnvelope>>();
  auto subscription = client_node_->create_subscription<ros2_msg::PclEnvelope>(
      binding.ros2_frame_topic, 50,
      [frames](const ros2_msg::PclEnvelope::SharedPtr msg) {
        frames->push_back(*msg);
      });

  auto client = client_node_->create_client<ros2_srv::PclOpenStream>(
      binding.ros2_open_service);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto request = std::make_shared<ros2_srv::PclOpenStream::Request>();
  request->content_type = "application/protobuf";
  request->correlation_id = "stream-1";
  request->payload.assign({'f', 'i', 'n', 'd'});

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto response = future.get();
  ASSERT_TRUE(waitUntil([&] { return frames->size() >= 3U; }));

  ASSERT_TRUE(waitUntil([&] { return state.called.load(); }));
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.thread_id, pcl_thread_id_);
    EXPECT_EQ(state.request_payload, "find");
  }

  ASSERT_EQ(frames->size(), 3U);
  EXPECT_EQ(std::string(frames->at(0).payload.begin(), frames->at(0).payload.end()),
            "alpha");
  EXPECT_EQ(std::string(frames->at(1).payload.begin(), frames->at(1).payload.end()),
            "bravo");
  EXPECT_TRUE(frames->at(2).end_of_stream);
  EXPECT_EQ(frames->at(0).correlation_id, "stream-1");
  EXPECT_TRUE(response->accepted);
  EXPECT_EQ(response->correlation_id, "stream-1");

  (void)subscription;
  stopPclExecutor();
}

TEST_F(RuntimeAdapterFixture, PublishRoutesTypedMessageOntoRos2Topic) {
  const auto binding = ros2_support::makeTopicBinding(kTopicObjectEvidence);

  std::promise<typed_msg::ObjectDetail> received;
  auto future = received.get_future();
  auto subscription = client_node_->create_subscription<typed_msg::ObjectDetail>(
      binding.ros2_topic, 10,
      [&received](const typed_msg::ObjectDetail::SharedPtr msg) {
        received.set_value(*msg);
      });

  typed_msg::ObjectDetail typed;
  typed.id = "typed-out-7";
  typed.entity_source = "adapter";

  ros2_support::Envelope envelope;
  envelope.content_type = "application/ros2";
  envelope.correlation_id = "pub-7";
  envelope.payload = serializeTypedMessage(typed);
  adapter_->publish(binding, envelope);

  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto msg = future.get();
  EXPECT_EQ(msg.id, "typed-out-7");
  EXPECT_EQ(msg.entity_source, "adapter");

  (void)subscription;
}

// A4 interop proof: a plain rclcpp peer that speaks ONLY the generated
// pyramid_msgs typed IDL (no PYRAMID adapter/runtime API -- just rclcpp
// create_publisher/create_subscription over pyramid_msgs::msg::ObjectDetail)
// round-trips a message against the PYRAMID component over the standard ROS2
// typed wire, in both directions, correlated by id.
TEST_F(RuntimeAdapterFixture, PlainRclcppPeerRoundTripsGeneratedTypedIdl) {
  TopicState state;
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigureTopic;
  startPclExecutor(callbacks, &state, "ros2_interop_runtime");

  ros2_support::bindTopicIngress(*adapter_, executor_, kTopicObjectEvidence);
  const auto binding = ros2_support::makeTopicBinding(kTopicObjectEvidence);
  ASSERT_EQ(binding.ros2_message_type, "pyramid_msgs/msg/ObjectDetail");

  // The peer subscribes for the component's typed reply. It filters on the
  // reply marker so it ignores the echo of its own outbound publish on the
  // shared topic.
  std::promise<typed_msg::ObjectDetail> reply;
  auto reply_future = reply.get_future();
  std::atomic<bool> reply_seen{false};
  auto reply_sub = client_node_->create_subscription<typed_msg::ObjectDetail>(
      binding.ros2_topic, 10,
      [&](const typed_msg::ObjectDetail::SharedPtr msg) {
        if (msg->entity_source == "pyramid-component" &&
            !reply_seen.exchange(true)) {
          reply.set_value(*msg);
        }
      });

  // Leg A (external peer -> PYRAMID): a plain rclcpp publisher emits the
  // generated typed message; the adapter's ingress subscription deserializes
  // it and hands it to the PCL executor thread.
  auto peer_pub = client_node_->create_publisher<typed_msg::ObjectDetail>(
      binding.ros2_topic, 10);
  typed_msg::ObjectDetail outbound;
  outbound.id = "interop-roundtrip-1";
  outbound.entity_source = "external-peer";
  peer_pub->publish(outbound);

  ASSERT_TRUE(waitUntil([&] { return state.called.load(); }));
  typed_msg::ObjectDetail ingress_decoded;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    ingress_decoded = deserializeTypedMessage<typed_msg::ObjectDetail>(
        std::vector<unsigned char>(state.payload.begin(), state.payload.end()));
    EXPECT_EQ(state.type_name, "application/ros2");
  }
  EXPECT_EQ(ingress_decoded.id, "interop-roundtrip-1");
  EXPECT_EQ(ingress_decoded.entity_source, "external-peer");

  // Leg B (PYRAMID -> external peer): the component replies with a typed
  // message (correlated by id) onto the same wire; the plain peer receives it
  // decoded as the generated type -- proving the wire carries no
  // PYRAMID-specific framing.
  typed_msg::ObjectDetail reply_msg;
  reply_msg.id = ingress_decoded.id;
  reply_msg.entity_source = "pyramid-component";
  ros2_support::Envelope reply_env;
  reply_env.content_type = "application/ros2";
  reply_env.correlation_id = "interop-1";
  reply_env.payload = serializeTypedMessage(reply_msg);
  adapter_->publish(binding, reply_env);

  ASSERT_EQ(reply_future.wait_for(std::chrono::seconds(2)),
            std::future_status::ready);
  const auto received = reply_future.get();
  EXPECT_EQ(received.id, "interop-roundtrip-1");
  EXPECT_EQ(received.entity_source, "pyramid-component");

  (void)reply_sub;
  stopPclExecutor();
}

TEST_F(RuntimeAdapterFixture, EnvelopeFallbackPublishesPclEnvelopeOntoRos2Topic) {
  ros2_support::RclcppRuntimeAdapter::Options options;
  options.use_envelope_wire = true;
  ros2_support::RclcppRuntimeAdapter envelope_adapter(server_node_, options);
  const auto binding = ros2_support::makeTopicBinding(kTopicObjectEvidence);

  std::promise<ros2_msg::PclEnvelope> received;
  auto future = received.get_future();
  auto subscription = client_node_->create_subscription<ros2_msg::PclEnvelope>(
      binding.ros2_topic, 10,
      [&received](const ros2_msg::PclEnvelope::SharedPtr msg) {
        received.set_value(*msg);
      });

  ros2_support::Envelope envelope;
  envelope.content_type = "application/protobuf";
  envelope.correlation_id = "pub-7";
  envelope.payload.assign({'o', 'u', 't'});
  envelope_adapter.publish(binding, envelope);

  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto msg = future.get();
  EXPECT_EQ(msg.content_type, "application/protobuf");
  EXPECT_EQ(msg.correlation_id, "pub-7");
  EXPECT_EQ(std::string(msg.payload.begin(), msg.payload.end()), "out");

  (void)subscription;
}

// Consumed (client) side: invokeUnary calls a remote ROS2 service and returns
// its response -- the both-ways core for application/ros2 (§2.C.3 / §2.D.6).
TEST_F(RuntimeAdapterFixture, ConsumedInvokeUnaryCallsRemoteRos2Service) {
  const auto binding = ros2_support::makeUnaryServiceBinding("standard.echo");

  // Server: advertise an echo service on the server adapter/node.
  adapter_->advertise(binding, [](const ros2_support::Envelope& req) {
    ros2_support::Envelope resp;
    resp.content_type = req.content_type;
    resp.correlation_id = req.correlation_id;
    resp.payload = req.payload;
    resp.payload.push_back('!');
    resp.status = PCL_OK;
    return resp;
  });

  // Client: a second adapter on the client node invokes it. invokeUnary blocks
  // on this (test) thread while the fixture's ROS spin thread services both ends.
  ros2_support::RclcppRuntimeAdapter client_adapter(client_node_);
  ros2_support::Envelope request;
  request.content_type = "application/ros2";
  request.correlation_id = "consume-1";
  request.payload.assign({'h', 'i'});

  const auto response = client_adapter.invokeUnary(binding, request);

  EXPECT_EQ(response.status, PCL_OK);
  EXPECT_EQ(response.correlation_id, "consume-1");
  ASSERT_EQ(response.payload.size(), 3u);
  EXPECT_EQ(std::string(response.payload.begin(), response.payload.end()), "hi!");
}

// A missing server fails closed (no hang) rather than blocking forever.
TEST_F(RuntimeAdapterFixture, ConsumedInvokeUnaryFailsClosedWithoutServer) {
  ros2_support::RclcppRuntimeAdapter client_adapter(client_node_);
  const auto binding = ros2_support::makeUnaryServiceBinding("standard.absent");
  ros2_support::Envelope request;
  request.payload.assign({'x'});
  const auto response = client_adapter.invokeUnary(binding, request);
  EXPECT_NE(response.status, PCL_OK);
}

// Consumed streaming: invokeStream opens a remote ROS2 stream and delivers the
// server's frames, including the terminal end_of_stream frame, on the caller
// thread after the adapter buffers the ROS2 frame-topic messages.
TEST_F(RuntimeAdapterFixture, ConsumedInvokeStreamCallsRemoteRos2Service) {
  const auto binding = ros2_support::makeStreamServiceBinding("standard.stream");

  adapter_->advertise(
      binding,
      [](const ros2_support::Envelope& req,
         const ros2_support::StreamEmitter& emit) {
        ros2_support::Envelope first;
        first.content_type = req.content_type;
        first.correlation_id = req.correlation_id;
        first.payload.assign({'o', 'n', 'e'});
        ASSERT_TRUE(emit(first));

        ros2_support::Envelope second;
        second.content_type = req.content_type;
        second.correlation_id = req.correlation_id;
        second.payload.assign({'t', 'w', 'o'});
        ASSERT_TRUE(emit(second));

        ros2_support::Envelope terminal;
        terminal.content_type = req.content_type;
        terminal.correlation_id = req.correlation_id;
        terminal.end_of_stream = true;
        terminal.status = PCL_OK;
        ASSERT_TRUE(emit(terminal));
      });

  ros2_support::RclcppRuntimeAdapter client_adapter(client_node_);
  ros2_support::Envelope request;
  request.content_type = "application/ros2";
  request.correlation_id = "consume-stream-1";
  request.payload.assign({'g', 'o'});

  std::vector<ros2_support::Envelope> frames;
  client_adapter.invokeStream(binding, request,
                              [&frames](const ros2_support::Envelope& frame) {
                                frames.push_back(frame);
                                return !frame.end_of_stream;
                              });

  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[0].status, PCL_OK);
  EXPECT_EQ(frames[0].correlation_id, "consume-stream-1");
  EXPECT_EQ(std::string(frames[0].payload.begin(), frames[0].payload.end()),
            "one");
  EXPECT_EQ(std::string(frames[1].payload.begin(), frames[1].payload.end()),
            "two");
  EXPECT_TRUE(frames[2].end_of_stream);
  EXPECT_EQ(frames[2].status, PCL_OK);
}
