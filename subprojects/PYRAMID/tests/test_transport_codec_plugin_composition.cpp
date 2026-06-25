/// \file test_transport_codec_plugin_composition.cpp
/// \brief Codec-plugin plus transport-plugin composition over generated facade.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
}

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <unistd.h>
#endif

#include "pyramid_codec_plugin_test_paths.hpp"
#include "pyramid_services_tactical_objects_consumed.hpp"

namespace cons = pyramid::components::tactical_objects::services::consumed;
namespace types = pyramid::domain_model;

namespace {

using DestroyTransportFn = void (*)(const pcl_transport_t*);

constexpr const char* kContentType = "application/flatbuffers";

types::ObjectDetail makeEvidence() {
  types::ObjectDetail ev;
  ev.id = "plugin-transport-object";
  ev.identity = types::StandardIdentity::Hostile;
  ev.dimension = types::BattleDimension::SeaSurface;
  ev.position.latitude = 0.8901;
  ev.position.longitude = 0.0012;
  ev.creation_time = 42.0;
  ev.quality = 0.95;
  return ev;
}

uint16_t testPort() {
#if defined(_WIN32)
  const auto pid = static_cast<uint32_t>(GetCurrentProcessId());
#else
  const auto pid = static_cast<uint32_t>(getpid());
#endif
  return static_cast<uint16_t>(23000u + (pid % 20000u));
}

std::string makeTransportConfig(const char* role,
                                uint16_t port,
                                pcl_executor_t* executor) {
  std::ostringstream out;
  out << "{\"role\":\"" << role
      << "\",\"host\":\"127.0.0.1\",\"port\":" << port
      << ",\"executor\":"
      << static_cast<unsigned long long>(
             reinterpret_cast<std::uintptr_t>(executor))
      << "}";
  return out.str();
}

void expectEvidenceEqual(const types::ObjectDetail& a,
                         const types::ObjectDetail& b) {
  EXPECT_EQ(a.id, b.id);
  EXPECT_EQ(a.identity, b.identity);
  EXPECT_EQ(a.dimension, b.dimension);
  EXPECT_DOUBLE_EQ(a.position.latitude, b.position.latitude);
  EXPECT_DOUBLE_EQ(a.position.longitude, b.position.longitude);
  EXPECT_DOUBLE_EQ(a.creation_time, b.creation_time);
  ASSERT_TRUE(a.quality.has_value());
  ASSERT_TRUE(b.quality.has_value());
  EXPECT_DOUBLE_EQ(a.quality.value(), b.quality.value());
}

struct ServerState {
  pcl_executor_t* executor = nullptr;
  std::atomic<bool> started{false};
  std::atomic<bool> connected{false};
  std::atomic<bool> done{false};
  std::atomic<bool> failed{false};
  std::atomic<int> received_count{0};
  std::mutex mutex;
  types::ObjectDetail received;
};

void objectEvidenceCb(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* state = static_cast<ServerState*>(user_data);
  types::ObjectDetail decoded;
  if (!cons::decodeObjectEvidence(msg, &decoded)) {
    state->failed.store(true);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->received = decoded;
  }
  state->received_count.fetch_add(1);
}

pcl_status_t configureServerSubscriber(pcl_container_t* container,
                                       void* user_data) {
  return cons::subscribeObjectEvidence(
             container, objectEvidenceCb, user_data, kContentType)
             ? PCL_OK
             : PCL_ERR_NOMEM;
}

struct ClientState {
  pcl_port_t* publisher = nullptr;
};

pcl_status_t configureClientPublisher(pcl_container_t* container,
                                      void* user_data) {
  auto* state = static_cast<ClientState*>(user_data);
  state->publisher = pcl_container_add_publisher(
      container, cons::kTopicObjectEvidence, kContentType);
  return state->publisher ? PCL_OK : PCL_ERR_NOMEM;
}

void serverThread(ServerState* state, uint16_t port) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  const std::string config =
      makeTransportConfig("server", port, state->executor);

  state->started.store(true);
  if (pcl_plugin_load_transport(kPclSocketTransportPlugin,
                                config.c_str(),
                                &handle,
                                &transport) != PCL_OK) {
    state->failed.store(true);
    return;
  }

  auto destroy_transport = reinterpret_cast<DestroyTransportFn>(
      pcl_plugin_symbol(handle, "pcl_socket_transport_plugin_destroy"));
  if (!transport || !destroy_transport ||
      pcl_executor_set_transport(state->executor, transport) != PCL_OK) {
    state->failed.store(true);
  } else {
    state->connected.store(true);
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(5);
  while (!state->done.load() &&
         std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(state->executor, 0);
    if (state->received_count.load() > 0) {
      state->done.store(true);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  destroy_transport(transport);
  pcl_plugin_unload(handle);
}

}  // namespace

TEST(TransportCodecPluginComposition,
     FlatBuffersCodecPluginOverSocketTransportPluginPubSub) {
  pcl_codec_registry_t* default_registry = pcl_codec_registry_default();
  ASSERT_NE(default_registry, nullptr);
  pcl_codec_registry_clear(default_registry);

  pcl_plugin_handle_t* codec_handle = nullptr;
  ASSERT_NE(kPyramidFlatbuffersCodecPluginTactical, nullptr);
  ASSERT_EQ(pcl_plugin_load_codec(kPyramidFlatbuffersCodecPluginTactical,
                                  default_registry,
                                  &codec_handle),
            PCL_OK);
  ASSERT_NE(codec_handle, nullptr);
  ASSERT_NE(pcl_codec_registry_get(default_registry, kContentType), nullptr);

  const uint16_t port = testPort();
  ServerState server_state;
  server_state.executor = pcl_executor_create();
  ASSERT_NE(server_state.executor, nullptr);

  pcl_callbacks_t server_callbacks = {};
  server_callbacks.on_configure = configureServerSubscriber;
  pcl_container_t* server_container = pcl_container_create(
      "transport_plugin_server", &server_callbacks, &server_state);
  ASSERT_NE(server_container, nullptr);
  ASSERT_EQ(pcl_container_configure(server_container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(server_container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server_state.executor, server_container), PCL_OK);

  std::thread server(serverThread, &server_state, port);
  while (!server_state.started.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  pcl_executor_t* client_executor = pcl_executor_create();
  ASSERT_NE(client_executor, nullptr);

  pcl_plugin_handle_t* client_transport_handle = nullptr;
  const pcl_transport_t* client_transport = nullptr;
  const std::string client_config =
      makeTransportConfig("client", port, client_executor);
  ASSERT_NE(kPclSocketTransportPlugin, nullptr);
  ASSERT_EQ(pcl_plugin_load_transport(kPclSocketTransportPlugin,
                                      client_config.c_str(),
                                      &client_transport_handle,
                                      &client_transport),
            PCL_OK);
  ASSERT_NE(client_transport_handle, nullptr);
  ASSERT_NE(client_transport, nullptr);
  auto destroy_client_transport = reinterpret_cast<DestroyTransportFn>(
      pcl_plugin_symbol(client_transport_handle,
                        "pcl_socket_transport_plugin_destroy"));
  ASSERT_NE(destroy_client_transport, nullptr);
  ASSERT_EQ(pcl_executor_set_transport(client_executor, client_transport),
            PCL_OK);

  const auto connected_deadline = std::chrono::steady_clock::now() +
                                  std::chrono::seconds(5);
  while (!server_state.connected.load() &&
         std::chrono::steady_clock::now() < connected_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(server_state.connected.load());
  ASSERT_FALSE(server_state.failed.load());

  ClientState client_state;
  pcl_callbacks_t client_callbacks = {};
  client_callbacks.on_configure = configureClientPublisher;
  pcl_container_t* client_container = pcl_container_create(
      "transport_plugin_client", &client_callbacks, &client_state);
  ASSERT_NE(client_container, nullptr);
  ASSERT_EQ(pcl_container_configure(client_container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(client_container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(client_executor, client_container), PCL_OK);
  ASSERT_NE(client_state.publisher, nullptr);

  const types::ObjectDetail expected = makeEvidence();
  ASSERT_EQ(cons::publishObjectEvidence(
                client_state.publisher, expected, kContentType),
            PCL_OK);

  const auto receive_deadline = std::chrono::steady_clock::now() +
                                std::chrono::seconds(5);
  while (server_state.received_count.load() == 0 &&
         std::chrono::steady_clock::now() < receive_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_GE(server_state.received_count.load(), 1);
  EXPECT_FALSE(server_state.failed.load());
  {
    std::lock_guard<std::mutex> lock(server_state.mutex);
    expectEvidenceEqual(server_state.received, expected);
  }

  server_state.done.store(true);
  destroy_client_transport(client_transport);
  pcl_plugin_unload(client_transport_handle);
  pcl_executor_destroy(client_executor);
  pcl_container_destroy(client_container);

  server.join();
  pcl_executor_destroy(server_state.executor);
  pcl_container_destroy(server_container);

  pcl_codec_registry_clear(default_registry);
  pcl_plugin_unload(codec_handle);
}
