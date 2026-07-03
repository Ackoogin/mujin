#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

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

TEST(PclPluginLoader, UnloadTransportNullHandleFailsClosed) {
  EXPECT_EQ(pcl_plugin_unload_transport(nullptr, nullptr), PCL_ERR_INVALID);
}

TEST(PclPluginLoader, UnloadTransportDegradesToUnloadWithoutTeardownSymbol) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(CAPTURE_PLUGIN_PATH, "{}", &handle,
                                      &transport),
            PCL_OK);
  // The capture plugin is stateless (no PCL_TRANSPORT_PLUGIN_TEARDOWN_SYMBOL), so
  // the safe-unload path simply unloads the library.
  EXPECT_EQ(pcl_plugin_unload_transport(handle, transport), PCL_OK);
}

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

TEST(PclPluginLoader, CodecEntryReturningNullFailsClosed) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* handle = nullptr;
  EXPECT_EQ(pcl_plugin_load_codec(STUB_CODEC_PLUGIN_PATH,
                                  "{\"mode\":\"return_null\"}", registry,
                                  &handle),
            PCL_ERR_STATE);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);

  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, ReloadingSameCodecVtableFailsClosed) {
  // Loading the same codec plugin twice yields the identical static vtable
  // pointer; the registry rejects the re-registration with PCL_ERR_STATE and
  // the loader unloads the second handle.
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* first = nullptr;
  ASSERT_EQ(pcl_plugin_load_codec(STUB_CODEC_PLUGIN_PATH, nullptr, registry,
                                  &first),
            PCL_OK);

  pcl_plugin_handle_t* second = nullptr;
  EXPECT_EQ(pcl_plugin_load_codec(STUB_CODEC_PLUGIN_PATH, nullptr, registry,
                                  &second),
            PCL_ERR_STATE);
  EXPECT_EQ(second, nullptr);
  EXPECT_EQ(pcl_codec_registry_count(registry), 1u);

  pcl_plugin_unload(first);
  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, TransportPluginWithoutAbiSymbolFailsClosed) {
  // A codec plugin has no pcl_transport_abi_version symbol.
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(STUB_CODEC_PLUGIN_PATH, nullptr, &handle,
                                      &transport),
            PCL_ERR_NOT_FOUND);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(transport, nullptr);
}

TEST(PclPluginLoader, TransportPluginWithWrongAbiFailsClosed) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(WRONGABI_TRANSPORT_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(transport, nullptr);
}

TEST(PclPluginLoader, TransportPluginWithoutEntrySymbolFailsClosed) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(NOENTRY_TRANSPORT_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_ERR_NOT_FOUND);
  EXPECT_EQ(handle, nullptr);
  EXPECT_EQ(transport, nullptr);
}

TEST(PclPluginLoader, LoadCodecPluginsFromPathsSkipsBadEntries) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  EXPECT_EQ(pcl_codec_registry_load_plugins_from_paths(nullptr, nullptr, 0u),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_paths(registry, nullptr, 1u),
            PCL_ERR_INVALID);

  // NULL entries, empty strings, missing files, and non-codec plugins are all
  // skipped; the stub codec still loads and stays resident.
  const char* paths[] = {
      nullptr, "", "/no/such/plugin.so", SOCKET_TRANSPORT_PLUGIN_PATH,
      STUB_CODEC_PLUGIN_PATH};
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_paths(registry, paths, 5u),
            PCL_OK);
  EXPECT_EQ(pcl_codec_registry_count(registry), 1u);
  EXPECT_NE(pcl_codec_registry_get(registry, "application/stub"), nullptr);

  pcl_codec_registry_destroy(registry);
}

namespace {

void set_env_var(const char* name, const char* value) {
#ifdef _WIN32
  _putenv_s(name, value ? value : "");
#else
  if (value) {
    setenv(name, value, 1);
  } else {
    unsetenv(name);
  }
#endif
}

}  // namespace

TEST(PclPluginLoader, LoadCodecPluginsFromEnv) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  EXPECT_EQ(pcl_codec_registry_load_plugins_from_env(nullptr, "X"),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_env(registry, nullptr),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_env(registry, ""),
            PCL_ERR_INVALID);

  // Unset (or empty) environment variable: nothing to load, PCL_OK.
  set_env_var("PCL_TEST_CODEC_PLUGINS", nullptr);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_env(registry,
                                                     "PCL_TEST_CODEC_PLUGINS"),
            PCL_OK);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);

  // A path list mixing a missing plugin with the stub codec: the missing entry
  // is skipped, the stub codec registers.
#ifdef _WIN32
  const char sep = ';';
#else
  const char sep = ':';
#endif
  std::string value = std::string("/no/such/plugin.so") + sep +
                      STUB_CODEC_PLUGIN_PATH;
  set_env_var("PCL_TEST_CODEC_PLUGINS", value.c_str());
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_env(registry,
                                                     "PCL_TEST_CODEC_PLUGINS"),
            PCL_OK);
  EXPECT_EQ(pcl_codec_registry_count(registry), 1u);
  EXPECT_NE(pcl_codec_registry_get(registry, "application/stub"), nullptr);

  set_env_var("PCL_TEST_CODEC_PLUGINS", nullptr);
  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, ManifestNullArgsFailClosed) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_manifest(nullptr, "m.txt"),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_manifest(registry, nullptr),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_manifest(registry, ""),
            PCL_ERR_INVALID);
  pcl_codec_registry_destroy(registry);
}

TEST(PclPluginLoader, OpenSymbolUnloadRawLibrary) {
  EXPECT_EQ(pcl_plugin_open(nullptr), nullptr);
  EXPECT_EQ(pcl_plugin_open("/no/such/library.so"), nullptr);

  pcl_plugin_handle_t* handle = pcl_plugin_open(CAPTURE_PLUGIN_PATH);
  ASSERT_NE(handle, nullptr);
  EXPECT_NE(pcl_plugin_symbol(handle, "pcl_transport_plugin_entry"), nullptr);
  EXPECT_EQ(pcl_plugin_symbol(handle, "no_such_symbol"), nullptr);
  EXPECT_EQ(pcl_plugin_symbol(nullptr, "pcl_transport_plugin_entry"), nullptr);
  EXPECT_EQ(pcl_plugin_symbol(handle, nullptr), nullptr);
  pcl_plugin_unload(handle);
  pcl_plugin_unload(nullptr);  // NULL-safe.
}

TEST(PclPluginLoader, SocketPluginServerClientRoundTripViaEntry) {
  // Stand the socket transport up purely through the plugin ABI: a server
  // gateway on one executor, then a client on another executor connecting to
  // it, then teardown-then-unload for both via pcl_plugin_unload_transport.
  pcl_executor_t* server_exec = pcl_executor_create();
  pcl_executor_t* client_exec = pcl_executor_create();
  ASSERT_NE(server_exec, nullptr);
  ASSERT_NE(client_exec, nullptr);

  const unsigned short port = 18771;
  char server_config[256];
  std::snprintf(server_config, sizeof(server_config),
                "{\"role\":\"provided\",\"port\":%u,\"executor\":%llu}",
                (unsigned)port,
                static_cast<unsigned long long>(
                    reinterpret_cast<std::uintptr_t>(server_exec)));

  // The server-role entry blocks in accept() until a client connects, so
  // load it from a background thread and connect the client from here.
  pcl_plugin_handle_t* server_handle = nullptr;
  const pcl_transport_t* server_transport = nullptr;
  pcl_status_t server_rc = PCL_ERR_STATE;
  std::thread server_thread([&] {
    server_rc = pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH,
                                          server_config, &server_handle,
                                          &server_transport);
  });
  // Give the server a moment to reach accept().
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  char client_config[256];
  std::snprintf(client_config, sizeof(client_config),
                "{\"role\":\"consumed\",\"host\":\"127.0.0.1\",\"port\":%u,"
                "\"executor\":%llu}",
                (unsigned)port,
                static_cast<unsigned long long>(
                    reinterpret_cast<std::uintptr_t>(client_exec)));

  pcl_plugin_handle_t* client_handle = nullptr;
  const pcl_transport_t* client_transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH,
                                      client_config, &client_handle,
                                      &client_transport),
            PCL_OK);
  ASSERT_NE(client_transport, nullptr);

  server_thread.join();
  ASSERT_EQ(server_rc, PCL_OK);
  ASSERT_NE(server_transport, nullptr);

  // The socket plugin exposes the server gateway container.
  auto gateway = reinterpret_cast<pcl_container_t* (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(server_handle, "pcl_socket_transport_plugin_gateway"));
  ASSERT_NE(gateway, nullptr);
  EXPECT_NE(gateway(server_transport), nullptr);
  EXPECT_EQ(gateway(nullptr), nullptr);
  // Clients have no gateway.
  EXPECT_EQ(gateway(client_transport), nullptr);

  // Teardown-then-unload releases the live socket transports safely.
  EXPECT_EQ(pcl_plugin_unload_transport(client_handle, client_transport),
            PCL_OK);

  // A second client omitting "host" falls back to 127.0.0.1; tear it down
  // through the plugin's explicit destroy symbol instead of the teardown
  // symbol, then plain-unload the library.
  pcl_executor_t* second_exec = pcl_executor_create();
  ASSERT_NE(second_exec, nullptr);
  {
    // Re-arm the server for the second accept.
    pcl_plugin_handle_t* server2_handle = nullptr;
    const pcl_transport_t* server2_transport = nullptr;
    pcl_status_t server2_rc = PCL_ERR_STATE;
    char server2_config[256];
    std::snprintf(server2_config, sizeof(server2_config),
                  "{\"role\":\"server\",\"port\":%u,\"executor\":%llu}",
                  (unsigned)(port + 1),
                  static_cast<unsigned long long>(
                      reinterpret_cast<std::uintptr_t>(server_exec)));
    std::thread server2_thread([&] {
      server2_rc = pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH,
                                             server2_config, &server2_handle,
                                             &server2_transport);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    char hostless_config[256];
    std::snprintf(hostless_config, sizeof(hostless_config),
                  "{\"role\":\"client\",\"port\":%u,\"executor\":%llu}",
                  (unsigned)(port + 1),
                  static_cast<unsigned long long>(
                      reinterpret_cast<std::uintptr_t>(second_exec)));
    pcl_plugin_handle_t* hostless_handle = nullptr;
    const pcl_transport_t* hostless_transport = nullptr;
    ASSERT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH,
                                        hostless_config, &hostless_handle,
                                        &hostless_transport),
              PCL_OK);
    server2_thread.join();
    ASSERT_EQ(server2_rc, PCL_OK);

    auto plugin_destroy =
        reinterpret_cast<void (*)(const pcl_transport_t*)>(pcl_plugin_symbol(
            hostless_handle, "pcl_socket_transport_plugin_destroy"));
    ASSERT_NE(plugin_destroy, nullptr);
    plugin_destroy(nullptr);  // NULL-safe
    plugin_destroy(hostless_transport);
    pcl_plugin_unload(hostless_handle);
    EXPECT_EQ(pcl_plugin_unload_transport(server2_handle, server2_transport),
              PCL_OK);
  }
  pcl_executor_destroy(second_exec);

  EXPECT_EQ(pcl_plugin_unload_transport(server_handle, server_transport),
            PCL_OK);

  pcl_executor_destroy(client_exec);
  pcl_executor_destroy(server_exec);
}

TEST(PclPluginLoader, SocketPluginRejectsBadConfigs) {
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);
  const unsigned long long exec_val = static_cast<unsigned long long>(
      reinterpret_cast<std::uintptr_t>(executor));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  char config[256];

  // NULL config.
  EXPECT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH, nullptr,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Missing role.
  std::snprintf(config, sizeof(config), "{\"port\":18772,\"executor\":%llu}",
                exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Unknown role.
  std::snprintf(config, sizeof(config),
                "{\"role\":\"broker\",\"port\":18772,\"executor\":%llu}",
                exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Port out of range.
  std::snprintf(config, sizeof(config),
                "{\"role\":\"server\",\"port\":70000,\"executor\":%llu}",
                exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Missing port.
  std::snprintf(config, sizeof(config),
                "{\"role\":\"server\",\"executor\":%llu}", exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Missing executor.
  EXPECT_EQ(pcl_plugin_load_transport(SOCKET_TRANSPORT_PLUGIN_PATH,
                                      "{\"role\":\"server\",\"port\":18772}",
                                      &handle, &transport),
            PCL_ERR_STATE);

  pcl_executor_destroy(executor);
}

TEST(PclPluginLoader, LoadCodecPluginsFromPathsTwiceRetainsBothHandles) {
  // Two successive batch loads in one process: the first grows the resident
  // list, the second reuses its spare capacity.
  pcl_codec_registry_t* first = pcl_codec_registry_create();
  pcl_codec_registry_t* second = pcl_codec_registry_create();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  const char* paths[] = {STUB_CODEC_PLUGIN_PATH};
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_paths(first, paths, 1u),
            PCL_OK);
  EXPECT_EQ(pcl_codec_registry_load_plugins_from_paths(second, paths, 1u),
            PCL_OK);
  EXPECT_EQ(pcl_codec_registry_count(first), 1u);
  EXPECT_EQ(pcl_codec_registry_count(second), 1u);

  pcl_codec_registry_destroy(first);
  pcl_codec_registry_destroy(second);
}

TEST(PclPluginLoader, ShmPluginRejectsIncompleteConfigs) {
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);
  const unsigned long long exec_val = static_cast<unsigned long long>(
      reinterpret_cast<std::uintptr_t>(executor));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  char config[256];

  // Missing bus_name.
  std::snprintf(config, sizeof(config),
                "{\"participant_id\":\"x\",\"executor\":%llu}", exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(SHM_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Missing participant_id.
  std::snprintf(config, sizeof(config),
                "{\"bus_name\":\"shm_cfg_bus\",\"executor\":%llu}", exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(SHM_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Missing executor.
  EXPECT_EQ(pcl_plugin_load_transport(
                SHM_TRANSPORT_PLUGIN_PATH,
                "{\"bus_name\":\"shm_cfg_bus\",\"participant_id\":\"x\"}",
                &handle, &transport),
            PCL_ERR_STATE);

  pcl_executor_destroy(executor);
}

TEST(PclPluginLoader, ShmPluginTeardownThenUnload) {
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);

  char config[256];
  std::snprintf(config, sizeof(config),
                "{\"bus_name\":\"shm_teardown_bus\","
                "\"participant_id\":\"teardown\",\"executor\":%llu}",
                static_cast<unsigned long long>(
                    reinterpret_cast<std::uintptr_t>(executor)));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(SHM_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_OK);
  // Safe order: the plugin's teardown symbol runs before the dlclose.
  EXPECT_EQ(pcl_plugin_unload_transport(handle, transport), PCL_OK);
  pcl_executor_destroy(executor);
}

TEST(PclPluginLoader, UdpPluginRejectsIncompleteConfigs) {
  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);
  const unsigned long long exec_val = static_cast<unsigned long long>(
      reinterpret_cast<std::uintptr_t>(executor));

  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  char config[256];

  // Missing remote_host.
  std::snprintf(config, sizeof(config),
                "{\"remote_port\":18800,\"executor\":%llu}", exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(UDP_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Missing remote_port.
  std::snprintf(config, sizeof(config),
                "{\"remote_host\":\"127.0.0.1\",\"executor\":%llu}", exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(UDP_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // local_port out of range.
  std::snprintf(config, sizeof(config),
                "{\"remote_host\":\"127.0.0.1\",\"remote_port\":18800,"
                "\"local_port\":70000,\"executor\":%llu}",
                exec_val);
  EXPECT_EQ(pcl_plugin_load_transport(UDP_TRANSPORT_PLUGIN_PATH, config,
                                      &handle, &transport),
            PCL_ERR_STATE);

  // Missing executor.
  EXPECT_EQ(pcl_plugin_load_transport(
                UDP_TRANSPORT_PLUGIN_PATH,
                "{\"remote_host\":\"127.0.0.1\",\"remote_port\":18800}",
                &handle, &transport),
            PCL_ERR_STATE);

  pcl_executor_destroy(executor);
}
