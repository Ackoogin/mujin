/// \file test_ros2_coupled_plugin_load.cpp
/// \brief Loader checks for the coupled ROS2 target plugin (application/ros2).
///
/// ROS2 is delivered as a coupled target: a single .so exposes both a transport
/// vtable and a codec vtable under application/ros2. These tests verify the
/// plugin loads through the generic PCL loader and presents both halves with the
/// expected ABI, and that the transport half stands up a real rclcpp-backed
/// adapter that a dlopen-only client can drive.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_capabilities.h>
#include <pcl/pcl_codec.h>
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
}

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

constexpr const char* kRos2ContentType = "application/ros2";

using DestroyFn = void (*)(const pcl_transport_t*);
using BindTopicFn = pcl_status_t (*)(const pcl_transport_t*, const char*);

}  // namespace

TEST(Ros2CoupledPluginLoad, ExposesFunctionalTransportVtable) {
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);

  // The transport entry needs the PCL executor it binds to. Mirror the
  // socket/shm transport plugins: pass the executor pointer as a JSON number.
  char config[256];
  std::snprintf(config, sizeof(config),
                "{\"node_name\":\"ros2_coupled_loader_test\",\"executor\":%llu}",
                static_cast<unsigned long long>(
                    reinterpret_cast<std::uintptr_t>(executor)));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(PYRAMID_ROS2_COUPLED_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_OK);
  ASSERT_NE(handle, nullptr);
  ASSERT_NE(transport, nullptr);
  EXPECT_NE(transport->publish, nullptr);
  EXPECT_NE(transport->subscribe, nullptr);

  // The plugin exposes server-side bind hooks plus a destroy lifecycle hook.
  // Loading the transport already stood up a real rclcpp node + spin thread
  // (the entry would have returned NULL otherwise); driving live ROS2 traffic
  // is covered by test_rclcpp_runtime_adapter, so here -- like the coupled gRPC
  // loader test -- we assert the loader/ABI contract: both bind and destroy
  // entry symbols resolve.
  auto bind_topic = reinterpret_cast<BindTopicFn>(
      pcl_plugin_symbol(handle, "pcl_ros2_transport_plugin_bind_topic"));
  auto advertise_unary = reinterpret_cast<BindTopicFn>(
      pcl_plugin_symbol(handle, "pcl_ros2_transport_plugin_advertise_unary"));
  auto advertise_stream = reinterpret_cast<BindTopicFn>(
      pcl_plugin_symbol(handle, "pcl_ros2_transport_plugin_advertise_stream"));
  auto destroy = reinterpret_cast<DestroyFn>(
      pcl_plugin_symbol(handle, "pcl_ros2_transport_plugin_destroy"));
  ASSERT_NE(bind_topic, nullptr);
  ASSERT_NE(advertise_unary, nullptr);
  ASSERT_NE(advertise_stream, nullptr);
  ASSERT_NE(destroy, nullptr);

  // Invalid arguments fail closed without touching the live ROS2 graph.
  EXPECT_EQ(bind_topic(nullptr, "standard.object_evidence"), PCL_ERR_INVALID);

  destroy(transport);
  pcl_plugin_unload(handle);
  pcl_executor_destroy(executor);
}

TEST(Ros2CoupledPluginLoad, DeclaresPubsubUnaryStreamCaps) {
  // Capabilities are declared via an exported symbol, so no rclcpp node needs
  // to stand up. The server-ingress ROS2 transport carries pub/sub (vtable) plus
  // unary + server-streaming service ingress (advertise_unary/advertise_stream,
  // which are not vtable slots, so derivation alone could not see them).
  pcl_plugin_handle_t* handle = pcl_plugin_open(PYRAMID_ROS2_COUPLED_PLUGIN_PATH);
  ASSERT_NE(handle, nullptr);

  auto caps_fn = reinterpret_cast<pcl_transport_plugin_caps_fn>(
      pcl_plugin_symbol(handle, PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL));
  ASSERT_NE(caps_fn, nullptr);

  EXPECT_EQ(caps_fn(nullptr),
            PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM);

  pcl_plugin_unload(handle);
}

TEST(Ros2CoupledPluginLoad, NullConfigFailsClosed) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(PYRAMID_ROS2_COUPLED_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(transport, nullptr);
}

TEST(Ros2CoupledPluginLoad, RegistersCodecUnderRos2ContentType) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* handle = nullptr;
  ASSERT_EQ(pcl_plugin_load_codec(PYRAMID_ROS2_COUPLED_PLUGIN_PATH, "{}",
                                  registry, &handle),
            PCL_OK);
  ASSERT_NE(handle, nullptr);

  const pcl_codec_t* codec = pcl_codec_registry_get(registry, kRos2ContentType);
  ASSERT_NE(codec, nullptr);
  EXPECT_EQ(codec->abi_version, PCL_CODEC_ABI_VERSION);
  ASSERT_NE(codec->content_type, nullptr);
  EXPECT_STREQ(codec->content_type, kRos2ContentType);
  ASSERT_NE(codec->encode, nullptr);
  ASSERT_NE(codec->decode, nullptr);
  ASSERT_NE(codec->free_msg, nullptr);

  // The application/ros2 codec is a passthrough envelope codec: the typed value
  // crossing the ABI is itself a pcl_msg_t of pre-encoded payload bytes.
  const char payload[] = "ros2-payload";
  pcl_msg_t value{};
  value.data = payload;
  value.size = static_cast<uint32_t>(std::strlen(payload));
  value.type_name = "application/protobuf";

  pcl_msg_t encoded{};
  ASSERT_EQ(codec->encode(codec->codec_ctx, "Envelope", &value, &encoded), PCL_OK);
  ASSERT_NE(encoded.data, nullptr);
  EXPECT_EQ(encoded.size, value.size);
  EXPECT_EQ(std::memcmp(encoded.data, payload, value.size), 0);

  pcl_msg_t decoded{};
  ASSERT_EQ(codec->decode(codec->codec_ctx, "Envelope", &encoded, &decoded), PCL_OK);
  ASSERT_NE(decoded.data, nullptr);
  EXPECT_EQ(decoded.size, value.size);
  EXPECT_EQ(std::memcmp(decoded.data, payload, value.size), 0);

  codec->free_msg(codec->codec_ctx, &decoded);
  codec->free_msg(codec->codec_ctx, &encoded);
  EXPECT_EQ(encoded.data, nullptr);
  EXPECT_EQ(encoded.size, 0u);

  pcl_plugin_unload(handle);
  pcl_codec_registry_destroy(registry);
}
