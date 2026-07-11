/// \file lacal_e2e_test.cpp
/// \brief Cross-process PCL to Sleet to PCL LA-CAL proof.

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

#include <pyramid/oms_json_types.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#ifndef PYRAMID_LACAL_PLUGIN_PATH
#error "PYRAMID_LACAL_PLUGIN_PATH must be defined by the build"
#endif

#ifndef PYRAMID_OMS_JSON_CODEC_PATH
#error "PYRAMID_OMS_JSON_CODEC_PATH must be defined by the build"
#endif

namespace {

using namespace std::chrono_literals;
constexpr const char* kTopic = "mission.position-report";

template <size_t Size>
bool set_text(char (&field)[Size], const char* value) {
  const size_t length = std::strlen(value);
  if (length >= Size) return false;
  std::memcpy(field, value, length + 1u);
  return true;
}

std::filesystem::path write_manifest(const std::string& url,
                                     const char* service_id,
                                     const char* endpoint_kind) {
  const auto path = std::filesystem::temp_directory_path() /
                    (std::string("pyramid_lacal_") + service_id + ".manifest");
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << "transport asb " << PYRAMID_LACAL_PLUGIN_PATH << " {\"url\":\""
      << url << "\",\"service_id\":\"" << service_id
      << "\",\"schema\":\"002.5.0\","
         "\"content_type\":\"application/oms-json\","
         "\"peer_id\":\"asb\",\"connect_timeout_ms\":3000}\n"
      << "route " << kTopic << ' ' << endpoint_kind
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

bool fill_position(pyramid_uci_position_report_c* report) {
  std::memset(report, 0, sizeof(*report));
  return set_text(report->header.classification, "U") &&
         set_text(report->header.owner_producer, "USA") &&
         set_text(report->header.system_uuid,
                  "550e8400-e29b-41d4-a716-446655440000") &&
         set_text(report->header.service_uuid,
                  "6eefc2b6-08d4-4c39-8267-b1f21745bc90") &&
         set_text(report->header.timestamp, "2026-07-11T12:00:00Z") &&
         set_text(report->header.schema_version, "002.5.0") &&
         set_text(report->header.mode, "LIVE") &&
         set_text(report->report_system_uuid,
                  "550e8400-e29b-41d4-a716-446655440000") &&
         set_text(report->source, "ACTUAL") &&
         set_text(report->current_operating_domain, "AIR") &&
         set_text(report->position_timestamp, "2026-07-11T12:00:00Z") &&
         set_text(report->altitude_reference, "WGS_HAE") &&
         ((report->latitude_rad = 0.8989737191417272), true) &&
         ((report->longitude_rad = -0.002230530784048753), true) &&
         ((report->altitude_m = 1250.0), true) &&
         ((report->has_altitude_reference = true), true);
}

int run_publisher(const std::string& url) {
  CodecOwner codecs;
  if (!codecs.load()) {
    std::cerr << "publisher: failed to load OMS JSON codec\n";
    return 2;
  }

  pcl_executor_t* executor = pcl_executor_create();
  if (!executor) return 2;
  const auto manifest = write_manifest(url, "pyramid-publisher", "publisher");
  pcl_transport_routing_t* routing = nullptr;
  char diag[512] = "";
  if (pcl_transport_routing_load(executor, manifest.string().c_str(), &routing,
                                 diag, sizeof(diag)) != PCL_OK) {
    std::cerr << "publisher: routing load failed: " << diag << '\n';
    std::filesystem::remove(manifest);
    pcl_executor_destroy(executor);
    return 2;
  }

  struct PublisherState {
    pcl_port_t* port = nullptr;
  } state;
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = [](pcl_container_t* container,
                              void* user_data) -> pcl_status_t {
    auto* publisher = static_cast<PublisherState*>(user_data);
    publisher->port = pcl_container_add_publisher(
        container, kTopic, "PositionReport");
    return publisher->port ? PCL_OK : PCL_ERR_NOMEM;
  };
  pcl_container_t* container =
      pcl_container_create("lacal_e2e_publisher", &callbacks, &state);
  int result = 2;
  if (container && pcl_container_configure(container) == PCL_OK &&
      pcl_container_activate(container) == PCL_OK &&
      pcl_executor_add(executor, container) == PCL_OK && state.port) {
    pyramid_uci_position_report_c position;
    pcl_msg_t message{};
    if (fill_position(&position) &&
        codecs.codec->encode(codecs.codec->codec_ctx, "PositionReport",
                             &position, &message) == PCL_OK) {
      result = pcl_port_publish(state.port, &message) == PCL_OK ? 0 : 2;
      codecs.codec->free_msg(codecs.codec->codec_ctx, &message);
      if (result == 0) std::this_thread::sleep_for(500ms);
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

struct SubscriberState {
  const pcl_codec_t* codec = nullptr;
  std::filesystem::path output;
  bool received = false;
  bool valid = false;
};

void on_position(pcl_container_t*, const pcl_msg_t* message, void* user_data) {
  auto* state = static_cast<SubscriberState*>(user_data);
  pyramid_uci_position_report_c position{};
  state->received = true;
  state->valid = state->codec->decode(state->codec->codec_ctx, "PositionReport",
                                      message, &position) == PCL_OK &&
                 std::abs(position.latitude_rad - 0.8989737191417272) < 1e-9 &&
                 std::abs(position.longitude_rad + 0.002230530784048753) < 1e-9 &&
                 std::abs(position.altitude_m - 1250.0) < 1e-9 &&
                 std::strcmp(position.altitude_reference, "WGS_HAE") == 0;
  if (state->valid) {
    std::ofstream(state->output) << "position-ok\n";
  }
}

int run_subscriber(const std::string& url, const std::filesystem::path& ready,
                   const std::filesystem::path& output, int timeout_seconds) {
  CodecOwner codecs;
  if (!codecs.load()) {
    std::cerr << "subscriber: failed to load OMS JSON codec\n";
    return 2;
  }

  pcl_executor_t* executor = pcl_executor_create();
  if (!executor) return 2;
  const auto manifest =
      write_manifest(url, "pyramid-subscriber", "subscriber");
  pcl_transport_routing_t* routing = nullptr;
  char diag[512] = "";
  if (pcl_transport_routing_load(executor, manifest.string().c_str(), &routing,
                                 diag, sizeof(diag)) != PCL_OK) {
    std::cerr << "subscriber: routing load failed: " << diag << '\n';
    std::filesystem::remove(manifest);
    pcl_executor_destroy(executor);
    return 2;
  }

  SubscriberState state{codecs.codec, output, false, false};
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = [](pcl_container_t* container,
                              void* user_data) -> pcl_status_t {
    return pcl_container_add_subscriber(container, kTopic, "PositionReport",
                                        on_position, user_data)
               ? PCL_OK
               : PCL_ERR_NOMEM;
  };
  pcl_container_t* container =
      pcl_container_create("lacal_e2e_subscriber", &callbacks, &state);
  int result = 2;
  if (container && pcl_container_configure(container) == PCL_OK &&
      pcl_container_activate(container) == PCL_OK &&
      pcl_executor_add(executor, container) == PCL_OK) {
    std::this_thread::sleep_for(200ms);
    std::ofstream(ready) << "ready\n";
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    while (!state.received && std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(executor, 10);
    }
    result = state.valid ? 0 : 2;
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
  if (argc == 3 && std::strcmp(argv[1], "publisher") == 0) {
    return run_publisher(argv[2]);
  }
  if (argc == 6 && std::strcmp(argv[1], "subscriber") == 0) {
    return run_subscriber(argv[2], argv[3], argv[4], std::stoi(argv[5]));
  }
  std::cerr << "usage: lacal_e2e_test publisher <url>\n"
               "       lacal_e2e_test subscriber <url> <ready> <output> "
               "<timeout-seconds>\n";
  return 2;
}
