/// \file lacal_seam_test.cpp
/// \brief Phase 5 positive: a correlated request/entity interaction
///        realized pub/sub on both legs over the LA-CAL transport, through
///        real Sleet, using the UCI ActionCommand / ActionCommandStatus pair.
///
/// The consumer (C2) publishes an ActionCommand on the request topic and
/// subscribes the entity topic; the provider (MA) subscribes the request
/// topic and, on receipt, publishes correlated ActionCommandStatus transitions
/// (RECEIVED then ACCEPTED) on the entity topic. Correlation is the UCI
/// CommandID.UUID. Both legs route over the LA-CAL (owp/asb) transport, so this
/// is the seam's "pubsub-works over the real broker" leg with a UCI vocabulary
/// Sleet's schema validates (the agra_example vocabulary would be rejected).

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

#include <pyramid/oms_json_types.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#ifndef PYRAMID_LACAL_PLUGIN_PATH
#error "PYRAMID_LACAL_PLUGIN_PATH must be defined by the build"
#endif
#ifndef PYRAMID_OMS_JSON_CODEC_PATH
#error "PYRAMID_OMS_JSON_CODEC_PATH must be defined by the build"
#endif

namespace {

using namespace std::chrono_literals;
constexpr const char* kRequestTopic = "mission.action-command";
constexpr const char* kEntityTopic = "mission.action-command-status";
constexpr const char* kCommandUuid = "7c9e6679-7425-40de-944b-e07fc1f90ae7";

template <size_t Size>
bool set_text(char (&field)[Size], const char* value) {
  const size_t length = std::strlen(value);
  if (length >= Size) return false;
  std::memcpy(field, value, length + 1u);
  return true;
}

void fill_header(pyramid_uci_oms_header_c* header) {
  set_text(header->classification, "U");
  set_text(header->owner_producer, "USA");
  set_text(header->system_uuid, "550e8400-e29b-41d4-a716-446655440000");
  set_text(header->service_uuid, "6eefc2b6-08d4-4c39-8267-b1f21745bc90");
  set_text(header->timestamp, "2026-07-11T12:00:00Z");
  set_text(header->schema_version, "002.5.0");
  set_text(header->mode, "LIVE");
}

std::filesystem::path write_manifest(const std::string& url,
                                     const char* service_id,
                                     const char* request_kind,
                                     const char* entity_kind) {
  const auto path = std::filesystem::temp_directory_path() /
                    (std::string("pyramid_lacal_seam_") + service_id + ".manifest");
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << "transport asb " << PYRAMID_LACAL_PLUGIN_PATH << " {\"url\":\"" << url
      << "\",\"service_id\":\"" << service_id
      << "\",\"schema\":\"002.5.0\",\"content_type\":\"application/oms-json\","
         "\"peer_id\":\"asb\",\"connect_timeout_ms\":3000}\n"
      << "route " << kRequestTopic << ' ' << request_kind << " asb best_effort\n"
      << "route " << kEntityTopic << ' ' << entity_kind
      << " asb best_effort\n";
  return path;
}

struct CodecOwner {
  pcl_codec_registry_t* registry = nullptr;
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = nullptr;

  bool load() {
    registry = pcl_codec_registry_create();
    if (!registry) return false;
    if (pcl_plugin_load_codec(PYRAMID_OMS_JSON_CODEC_PATH, nullptr, registry,
                              &handle) != PCL_OK) {
      return false;
    }
    codec = pcl_codec_registry_get(registry, "application/oms-json");
    return codec != nullptr;
  }

  ~CodecOwner() {
    if (handle) pcl_plugin_unload(handle);
    if (registry) pcl_codec_registry_destroy(registry);
  }
};

// -- Provider (MA): subscribe request, publish correlated status transitions --

struct ProviderState {
  const pcl_codec_t* codec = nullptr;
  pcl_port_t* status_port = nullptr;
  bool saw_request = false;
};

void publish_status(ProviderState* state, const char* command_uuid,
                    const char* processing_state) {
  pyramid_uci_action_command_status_c status;
  std::memset(&status, 0, sizeof(status));
  fill_header(&status.header);
  set_text(status.command_uuid, command_uuid);
  set_text(status.command_processing_state, processing_state);
  pcl_msg_t message{};
  if (state->codec->encode(state->codec->codec_ctx, "ActionCommandStatus",
                           &status, &message) == PCL_OK) {
    pcl_port_publish(state->status_port, &message);
    state->codec->free_msg(state->codec->codec_ctx, &message);
  }
}

void on_request(pcl_container_t*, const pcl_msg_t* message, void* user_data) {
  auto* state = static_cast<ProviderState*>(user_data);
  pyramid_uci_action_command_c command{};
  if (state->codec->decode(state->codec->codec_ctx, "ActionCommand", message,
                           &command) != PCL_OK) {
    return;
  }
  state->saw_request = true;
  // Emit correlated entity transitions keyed by the request's CommandID.
  publish_status(state, command.command_uuid, "RECEIVED");
  publish_status(state, command.command_uuid, "ACCEPTED");
}

int run_provider(const std::string& url, const std::filesystem::path& ready,
                 int timeout_seconds) {
  CodecOwner codecs;
  if (!codecs.load()) {
    std::cerr << "provider: failed to load OMS JSON codec\n";
    return 2;
  }
  pcl_executor_t* executor = pcl_executor_create();
  if (!executor) return 2;
  const auto manifest =
      write_manifest(url, "seam-ma", "subscriber", "publisher");
  pcl_transport_routing_t* routing = nullptr;
  char diag[512] = "";
  if (pcl_transport_routing_load(executor, manifest.string().c_str(), &routing,
                                 diag, sizeof(diag)) != PCL_OK) {
    std::cerr << "provider: routing load failed: " << diag << '\n';
    std::filesystem::remove(manifest);
    pcl_executor_destroy(executor);
    return 2;
  }

  ProviderState state{codecs.codec, nullptr, false};
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = [](pcl_container_t* container,
                              void* user_data) -> pcl_status_t {
    auto* provider = static_cast<ProviderState*>(user_data);
    provider->status_port = pcl_container_add_publisher(
        container, kEntityTopic, "ActionCommandStatus");
    if (!provider->status_port) return PCL_ERR_NOMEM;
    return pcl_container_add_subscriber(container, kRequestTopic,
                                        "ActionCommand", on_request, user_data)
               ? PCL_OK
               : PCL_ERR_NOMEM;
  };
  pcl_container_t* container =
      pcl_container_create("lacal_seam_ma", &callbacks, &state);
  int result = 2;
  if (container && pcl_container_configure(container) == PCL_OK &&
      pcl_container_activate(container) == PCL_OK &&
      pcl_executor_add(executor, container) == PCL_OK && state.status_port) {
    std::this_thread::sleep_for(200ms);
    std::ofstream(ready) << "ready\n";
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(executor, 10);
    }
    result = state.saw_request ? 0 : 2;
  }

  if (container) {
    pcl_executor_remove(executor, container);
    pcl_container_destroy(container);
  }
  pcl_transport_routing_destroy(routing);
  std::filesystem::remove(manifest);
  pcl_executor_destroy(executor);
  return result;
}

// -- Consumer (C2): publish request, collect correlated status transitions ----

struct ConsumerState {
  const pcl_codec_t* codec = nullptr;
  std::string expected_command_uuid;
  std::vector<std::string> states;
};

void on_status(pcl_container_t*, const pcl_msg_t* message, void* user_data) {
  auto* state = static_cast<ConsumerState*>(user_data);
  pyramid_uci_action_command_status_c status{};
  if (state->codec->decode(state->codec->codec_ctx, "ActionCommandStatus",
                           message, &status) != PCL_OK) {
    return;
  }
  if (state->expected_command_uuid == status.command_uuid) {
    state->states.emplace_back(status.command_processing_state);
  }
}

int run_consumer(const std::string& url, const std::filesystem::path& output,
                 int timeout_seconds) {
  CodecOwner codecs;
  if (!codecs.load()) {
    std::cerr << "consumer: failed to load OMS JSON codec\n";
    return 2;
  }
  pcl_executor_t* executor = pcl_executor_create();
  if (!executor) return 2;
  const auto manifest =
      write_manifest(url, "seam-c2", "publisher", "subscriber");
  pcl_transport_routing_t* routing = nullptr;
  char diag[512] = "";
  if (pcl_transport_routing_load(executor, manifest.string().c_str(), &routing,
                                 diag, sizeof(diag)) != PCL_OK) {
    std::cerr << "consumer: routing load failed: " << diag << '\n';
    std::filesystem::remove(manifest);
    pcl_executor_destroy(executor);
    return 2;
  }

  struct Ports {
    pcl_port_t* request = nullptr;
  } ports;
  ConsumerState state{codecs.codec, kCommandUuid, {}};

  struct Bundle {
    Ports* ports;
    ConsumerState* state;
  } bundle{&ports, &state};

  pcl_callbacks_t callbacks{};
  callbacks.on_configure = [](pcl_container_t* container,
                              void* user_data) -> pcl_status_t {
    auto* b = static_cast<Bundle*>(user_data);
    b->ports->request =
        pcl_container_add_publisher(container, kRequestTopic, "ActionCommand");
    if (!b->ports->request) return PCL_ERR_NOMEM;
    return pcl_container_add_subscriber(container, kEntityTopic,
                                        "ActionCommandStatus", on_status,
                                        b->state)
               ? PCL_OK
               : PCL_ERR_NOMEM;
  };
  pcl_container_t* container =
      pcl_container_create("lacal_seam_c2", &callbacks, &bundle);
  int result = 2;
  if (container && pcl_container_configure(container) == PCL_OK &&
      pcl_container_activate(container) == PCL_OK &&
      pcl_executor_add(executor, container) == PCL_OK && ports.request) {
    // Entity subscription is active before we publish the request, so the
    // provider's correlated status transitions cannot race an inactive sub.
    std::this_thread::sleep_for(300ms);

    pyramid_uci_action_command_c command;
    std::memset(&command, 0, sizeof(command));
    fill_header(&command.header);
    set_text(command.command_uuid, kCommandUuid);
    set_text(command.command_state, "NEW");
    set_text(command.capability_uuid, "6da3573c-2863-47a9-80cd-52b1ee2b4077");
    command.priority = 5;
    set_text(command.action_uuid, "eacfc454-2741-4d15-aeac-15c573d94090");

    pcl_msg_t message{};
    if (codecs.codec->encode(codecs.codec->codec_ctx, "ActionCommand", &command,
                             &message) == PCL_OK) {
      pcl_port_publish(ports.request, &message);
      codecs.codec->free_msg(codecs.codec->codec_ctx, &message);
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    while (state.states.size() < 2 &&
           std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(executor, 10);
    }
    const bool ok = state.states.size() == 2 && state.states[0] == "RECEIVED" &&
                    state.states[1] == "ACCEPTED";
    if (ok) {
      std::ofstream(output) << "seam-ok\n";
      result = 0;
    }
  }

  if (container) {
    pcl_executor_remove(executor, container);
    pcl_container_destroy(container);
  }
  pcl_transport_routing_destroy(routing);
  std::filesystem::remove(manifest);
  pcl_executor_destroy(executor);
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 5 && std::strcmp(argv[1], "provider") == 0) {
    return run_provider(argv[2], argv[3], std::stoi(argv[4]));
  }
  if (argc == 5 && std::strcmp(argv[1], "consumer") == 0) {
    return run_consumer(argv[2], argv[3], std::stoi(argv[4]));
  }
  std::cerr << "usage: lacal_seam_test provider <url> <ready> <timeout-seconds>\n"
               "       lacal_seam_test consumer <url> <output> <timeout-seconds>\n";
  return 2;
}
