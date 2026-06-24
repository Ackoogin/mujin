#include <gtest/gtest.h>

#include <cstring>

extern "C" {
#include "pcl/pcl_codec_registry.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_plugin_loader.h"
#include "pcl/pcl_transport.h"
}

namespace {

constexpr const char* kStubMarker = "STUB_ENCODED";

using ResetFn = void (*)(void);
using CountFn = uint32_t (*)(void);
using LastTopicFn = const char* (*)(void);

}  // namespace

TEST(PclPluginLoader, LoadTransportPluginAndCallVtable) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;

  ASSERT_EQ(pcl_plugin_load_transport(CAPTURE_PLUGIN_PATH, "{}",
                                      &handle, &transport),
            PCL_OK);
  ASSERT_NE(handle, nullptr);
  ASSERT_NE(transport, nullptr);
  ASSERT_NE(transport->publish, nullptr);

  auto reset = reinterpret_cast<ResetFn>(
      pcl_plugin_symbol(handle, "pcl_capture_reset"));
  auto publish_count = reinterpret_cast<CountFn>(
      pcl_plugin_symbol(handle, "pcl_capture_publish_count"));
  auto last_topic = reinterpret_cast<LastTopicFn>(
      pcl_plugin_symbol(handle, "pcl_capture_last_topic"));

  ASSERT_NE(reset, nullptr);
  ASSERT_NE(publish_count, nullptr);
  ASSERT_NE(last_topic, nullptr);

  reset();
  pcl_msg_t msg = {};
  msg.data = "payload";
  msg.size = 7u;
  msg.type_name = "Test";

  EXPECT_EQ(transport->publish(transport->adapter_ctx, "my.topic", &msg),
            PCL_OK);
  EXPECT_EQ(publish_count(), 1u);
  EXPECT_STREQ(last_topic(), "my.topic");

  pcl_plugin_unload(handle);
}

TEST(PclPluginLoader, InstallLoadedTransportOnExecutor) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;

  ASSERT_EQ(pcl_plugin_load_transport(CAPTURE_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_OK);
  ASSERT_NE(handle, nullptr);
  ASSERT_NE(transport, nullptr);

  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);

  EXPECT_EQ(pcl_executor_set_transport(executor, transport), PCL_OK);
  const pcl_transport_t* installed = pcl_executor_get_transport(executor);
  ASSERT_NE(installed, nullptr);
  EXPECT_EQ(installed->publish, transport->publish);
  EXPECT_EQ(installed->serve, transport->serve);
  EXPECT_EQ(installed->adapter_ctx, transport->adapter_ctx);

  pcl_executor_destroy(executor);
  pcl_plugin_unload(handle);
}

TEST(PclPluginLoader, LoadCodecPluginRegistersByContentType) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* handle = nullptr;
  ASSERT_EQ(pcl_plugin_load_codec(STUB_CODEC_PLUGIN_PATH, registry, &handle),
            PCL_OK);
  ASSERT_NE(handle, nullptr);

  const pcl_codec_t* codec =
      pcl_codec_registry_get(registry, "application/stub");
  ASSERT_NE(codec, nullptr);
  ASSERT_NE(codec->encode, nullptr);
  ASSERT_NE(codec->decode, nullptr);
  ASSERT_NE(codec->free_msg, nullptr);

  pcl_msg_t msg = {};
  char decoded[32] = {};

  EXPECT_EQ(codec->encode(codec->codec_ctx, "StubSchema", nullptr, &msg),
            PCL_OK);
  ASSERT_NE(msg.data, nullptr);
  EXPECT_EQ(std::strcmp(static_cast<const char*>(msg.data), kStubMarker), 0);
  EXPECT_EQ(codec->decode(codec->codec_ctx, "StubSchema", &msg, decoded),
            PCL_OK);
  EXPECT_STREQ(decoded, kStubMarker);

  codec->free_msg(codec->codec_ctx, &msg);
  EXPECT_EQ(msg.data, nullptr);
  EXPECT_EQ(msg.size, 0u);

  pcl_plugin_unload(handle);
  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, MissingFileFailsClosed) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* handle = nullptr;
  EXPECT_EQ(pcl_plugin_load_codec("/no/such/plugin.so", registry, &handle),
            PCL_ERR_NOT_FOUND);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);

  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, BadAbiFailsClosed) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* handle = nullptr;
  EXPECT_EQ(pcl_plugin_load_codec(BADABI_CODEC_PLUGIN_PATH, registry, &handle),
            PCL_ERR_STATE);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(pcl_codec_registry_get(registry, "application/badabi"), nullptr);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);

  pcl_codec_registry_destroy(registry);
}
