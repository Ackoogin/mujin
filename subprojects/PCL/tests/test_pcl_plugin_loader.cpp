#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

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
  ASSERT_EQ(pcl_plugin_load_codec(STUB_CODEC_PLUGIN_PATH,
                                  "{\"flavor\":\"compact\"}", registry, &handle),
            PCL_OK);
  ASSERT_NE(handle, nullptr);

  // Codec config pass-through: the opaque config_json reached the plugin entry.
  auto last_config = reinterpret_cast<LastTopicFn>(
      pcl_plugin_symbol(handle, "pcl_stub_last_config"));
  ASSERT_NE(last_config, nullptr);
  EXPECT_STREQ(last_config(), "{\"flavor\":\"compact\"}");

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
  EXPECT_EQ(pcl_plugin_load_codec("/no/such/plugin.so", nullptr, registry,
                                  &handle),
            PCL_ERR_NOT_FOUND);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);

  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, LoadSharedMemoryTransportPluginViaConfig) {
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);

  // Build the opaque config_json the shm transport plugin understands: the bus
  // name, this participant id, and the executor pointer it binds to.
  char config[256];
  std::snprintf(config, sizeof(config),
                "{\"bus_name\":\"pcl_shm_plugin_test_bus\","
                "\"participant_id\":\"loader_test\",\"executor\":%llu}",
                static_cast<unsigned long long>(
                    reinterpret_cast<std::uintptr_t>(executor)));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(SHM_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_OK);
  ASSERT_NE(handle, nullptr);
  ASSERT_NE(transport, nullptr);
  EXPECT_NE(transport->publish, nullptr);

  // The shm plugin exposes the gateway + destroy hooks a service host needs.
  auto gateway = reinterpret_cast<void* (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(handle, "pcl_shm_transport_plugin_gateway"));
  auto destroy = reinterpret_cast<void (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(handle, "pcl_shm_transport_plugin_destroy"));
  ASSERT_NE(gateway, nullptr);
  ASSERT_NE(destroy, nullptr);
  EXPECT_NE(gateway(transport), nullptr);

  destroy(transport);
  pcl_plugin_unload(handle);
  pcl_executor_destroy(executor);
}

TEST(PclPluginLoader, SharedMemoryTransportPluginNullConfigFailsClosed) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(SHM_TRANSPORT_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(transport, nullptr);
}

TEST(PclPluginLoader, LoadUdpTransportPluginViaConfig) {
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);

  // Build the opaque config_json the udp transport plugin understands: an
  // ephemeral local port (0), the remote peer endpoint, an optional peer id,
  // and the executor pointer it binds to.
  char config[256];
  std::snprintf(config, sizeof(config),
                "{\"local_port\":0,\"remote_host\":\"127.0.0.1\","
                "\"remote_port\":18745,\"peer_id\":\"loader_test\","
                "\"executor\":%llu}",
                static_cast<unsigned long long>(
                    reinterpret_cast<std::uintptr_t>(executor)));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(UDP_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_OK);
  ASSERT_NE(handle, nullptr);
  ASSERT_NE(transport, nullptr);
  EXPECT_NE(transport->publish, nullptr);

  // UDP is pub/sub-only: no gateway hook, but a destroy hook tears it down.
  auto destroy = reinterpret_cast<void (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(handle, "pcl_udp_transport_plugin_destroy"));
  ASSERT_NE(destroy, nullptr);

  destroy(transport);
  pcl_plugin_unload(handle);
  pcl_executor_destroy(executor);
}

TEST(PclPluginLoader, UdpTransportPluginNullConfigFailsClosed) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(UDP_TRANSPORT_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(transport, nullptr);
}

TEST(PclPluginLoader, LoadCodecPluginsFromManifest) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  // A deployment manifest: comments and blank lines are ignored; a transport
  // plugin listed alongside the codec is skipped (not a codec), and the stub
  // codec is loaded.
  const std::string manifest_path =
      std::string(::testing::TempDir()) + "/pcl_codec_manifest.txt";
  {
    std::FILE* f = std::fopen(manifest_path.c_str(), "w");
    ASSERT_NE(f, nullptr);
    std::fprintf(f, "# codec manifest\n\n");
    std::fprintf(f, "  %s  \n", STUB_CODEC_PLUGIN_PATH);
    std::fprintf(f, "%s\n", SOCKET_TRANSPORT_PLUGIN_PATH);
    std::fclose(f);
  }

  EXPECT_EQ(pcl_codec_registry_load_plugins_from_manifest(registry,
                                                          manifest_path.c_str()),
            PCL_OK);
  EXPECT_NE(pcl_codec_registry_get(registry, "application/stub"), nullptr);

  std::remove(manifest_path.c_str());
  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, MissingManifestFailsClosed) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_manifest(
                registry, "/no/such/manifest.txt"),
            PCL_ERR_NOT_FOUND);
  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, BadAbiFailsClosed) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* handle = nullptr;
  EXPECT_EQ(pcl_plugin_load_codec(BADABI_CODEC_PLUGIN_PATH, nullptr, registry,
                                  &handle),
            PCL_ERR_STATE);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(pcl_codec_registry_get(registry, "application/badabi"), nullptr);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);

  pcl_codec_registry_destroy(registry);
}
