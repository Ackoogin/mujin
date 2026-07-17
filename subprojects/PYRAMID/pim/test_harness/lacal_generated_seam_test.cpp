/// \file lacal_generated_seam_test.cpp
/// \brief Cross-process LA-CAL witness for the generated UCI interaction facade.

#include "pyramid_components_uci_c2_station_services_consumed_types.hpp"
#include "pyramid_components_uci_mission_autonomy_services_provided_types.hpp"
#include "pyramid_services_uci_c2_station_consumed_components.hpp"
#include "pyramid_services_uci_mission_autonomy_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ma = pyramid::components::uci::mission_autonomy::services::provided;
namespace c2 = pyramid::components::uci::c2_station::services::consumed;
namespace uci = pyramid::domain_model::uci;
namespace uci_port_grammar = pyramid::domain_model::uci_port_grammar;

namespace {

using namespace std::chrono_literals;
constexpr const char* kCommandId = "7c9e6679-7425-40de-944b-e07fc1f90ae7";

void fillUciEnvelope(uci::SecurityInformationType& security,
                     uci::HeaderType& header) {
  security.classification = uci::ClassificationEnum::U;
  security.owner_producer.push_back(
      uci::OwnerProducerChoiceType{uci::OwnerProducerEnum::Usa, tl::nullopt});
  header.system_id.uuid = "550e8400-e29b-41d4-a716-446655440000";
  header.service_id.emplace();
  header.service_id->uuid = "6eefc2b6-08d4-4c39-8267-b1f21745bc90";
  header.timestamp = "2026-07-11T12:00:00Z";
  header.schema_version = "002.5.0";
  header.mode = uci::MessageModeEnum::Live;
}

bool writeText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
  return out.good();
}

bool fileExists(const std::filesystem::path& path) {
  std::ifstream in(path);
  return in.good();
}

std::filesystem::path writeManifest(const std::string& url, const char* service_id,
                                    const char* request_kind,
                                    const char* entity_kind,
                                    const std::filesystem::path& directory) {
  const auto path = directory / (std::string("lacal_generated_") + service_id + ".manifest");
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << "transport asb " << PYRAMID_LACAL_PLUGIN_PATH << " {\"url\":\"" << url
      << "\",\"service_id\":\"" << service_id
      << "\",\"schema\":\"002.5.0\",\"content_type\":\"application/oms-json\","
         "\"peer_id\":\"asb\",\"connect_timeout_ms\":3000,"
         "\"declare_reliability\":\"reliable\"}\n"
      << "route mission.action_command.request " << request_kind << " asb reliable\n"
      << "route mission.action_command.entity " << entity_kind
      << " asb reliable\n";
  return path;
}

struct CodecOwner {
  pcl_plugin_handle_t* handle = nullptr;

  bool load() {
    // The generated facade resolves its content codec from the default
    // registry (same wiring as agra_seam_interchange_test), so load there --
    // a private registry would leave the facade unable to find the codec.
    return pcl_plugin_load_codec(PYRAMID_OMS_JSON_CODEC_PATH, nullptr,
                                 pcl_codec_registry_default(), &handle) == PCL_OK &&
           pcl_codec_registry_get(pcl_codec_registry_default(),
                                  "application/oms-json") != nullptr;
  }

  ~CodecOwner() {
    if (handle) pcl_plugin_unload(handle);
    pcl_codec_registry_clear(pcl_codec_registry_default());
  }
};

class ProviderHandler final : public ma::ActioncommandRequestPortHandler {
 public:
  void bindWriter(ma::ActioncommandRequestPortProvider::TransitionWriter writer) {
    writer_.emplace(std::move(writer));
  }

  ma::Ack onCreate(const ma::ActionCommand_Service_Request& request) override {
    if (!request.action_command || !writer_) return uci_port_grammar::kAckFail;
    const auto& command = *request.action_command;
    if (command.message_data.command.empty()) return uci_port_grammar::kAckFail;
    if (!command.message_data.command.front().capability) return uci_port_grammar::kAckFail;
    const std::string& id = command.message_data.command.front().capability->base.command_id.uuid;
    send(id, "RECEIVED");
    send(id, "ACCEPTED");
    saw_create_ = true;
    return uci_port_grammar::kAckOk;
  }

  bool sawCreate() const { return saw_create_; }

 private:
  void send(const std::string& id, const char* state) {
    ma::ActionCommand_Service_Entity transition;
    uci::ActionCommandStatusMT status;
    fillUciEnvelope(status.security_information, status.message_header);
    status.message_data.base.command_id.uuid = id;
    status.message_data.base.command_processing_state =
        std::strcmp(state, "RECEIVED") == 0
            ? uci::CommandProcessingStateEnum::Received
            : uci::CommandProcessingStateEnum::Accepted;
    transition.action_command_status = std::move(status);
    (void)writer_->send(transition);
  }

  std::optional<ma::ActioncommandRequestPortProvider::TransitionWriter> writer_;
  bool saw_create_ = false;
};

class ProviderComponent final : public pcl::Component {
 public:
  ProviderComponent(pcl::Executor& executor, ProviderHandler& handler)
      : pcl::Component("lacal_generated_seam_ma"), provider_(*this, executor, handler,
                                                               "application/oms-json") {}
  ma::ActioncommandRequestPortProvider& provider() { return provider_; }

 protected:
  pcl_status_t on_configure() override { return provider_.bind(); }

 private:
  ma::ActioncommandRequestPortProvider provider_;
};

class ConsumerComponent final : public pcl::Component {
 public:
  explicit ConsumerComponent(pcl::Executor& executor)
      : pcl::Component("lacal_generated_seam_c2"), client_(*this, executor,
                                                             "application/oms-json") {}
  c2::ActioncommandRequestPortClient& client() { return client_; }

 protected:
  pcl_status_t on_configure() override { return client_.bind(); }

 private:
  c2::ActioncommandRequestPortClient client_;
};

int runProvider(const std::string& url, const std::filesystem::path& ready,
                const std::filesystem::path& directory, int timeout_seconds) {
  CodecOwner codecs;
  if (!codecs.load()) return 2;
  pcl::Executor executor;
  const auto manifest = writeManifest(url, "seam-ma", "subscriber", "publisher", directory);
  pcl_transport_routing_t* routing = nullptr;
  char diag[512] = "";
  if (pcl_transport_routing_load(executor.handle(), manifest.c_str(), &routing, diag,
                                 sizeof(diag)) != PCL_OK) {
    std::fprintf(stderr, "provider routing failed: %s\n", diag);
    return 2;
  }
  ProviderHandler handler;
  ProviderComponent component(executor, handler);
  handler.bindWriter(component.provider().transitionWriter());
  const bool configured = component.provider().configureInteractionBinding(
                              "{\"request_leg\":\"pubsub\",\"entity_leg\":\"pubsub\"}") == PCL_OK &&
                          component.configure() == PCL_OK &&
                          component.provider().consumedService().configurePubSubTransport(
                              "{\"transport\":\"remote\",\"peer\":\"asb\"}") == PCL_OK &&
                          component.activate() == PCL_OK &&
                          executor.add(component) == PCL_OK;
  if (!configured) return 2;
  writeText(ready, "ready\n");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds);
  while (!handler.sawCreate() && std::chrono::steady_clock::now() < deadline) {
    executor.spinOnce(10);
  }
  // OWP publication is asynchronous. Let both correlated transitions leave
  // the worker queue before component/plugin teardown closes the socket.
  if (handler.sawCreate()) std::this_thread::sleep_for(250ms);
  executor.remove(component);
  pcl_transport_routing_destroy(routing);
  std::filesystem::remove(manifest);
  return handler.sawCreate() ? 0 : 2;
}

int runConsumer(const std::string& url, const std::filesystem::path& output,
                const std::filesystem::path& directory, int timeout_seconds) {
  CodecOwner codecs;
  if (!codecs.load()) return 2;
  pcl::Executor executor;
  const auto manifest = writeManifest(url, "seam-c2", "publisher", "subscriber", directory);
  pcl_transport_routing_t* routing = nullptr;
  char diag[512] = "";
  if (pcl_transport_routing_load(executor.handle(), manifest.c_str(), &routing, diag,
                                 sizeof(diag)) != PCL_OK) return 2;
  ConsumerComponent component(executor);
  if (component.client().configureInteractionBinding(
          "{\"request_leg\":\"pubsub\",\"entity_leg\":\"pubsub\"}") != PCL_OK ||
      component.configure() != PCL_OK ||
      component.client().consumedService().configurePubSubTransport(
          "{\"transport\":\"remote\",\"peer\":\"asb\"}") != PCL_OK ||
      component.activate() != PCL_OK ||
      executor.add(component) != PCL_OK) return 2;

  std::vector<std::string> states;
  auto subscription = component.client().transitions(
      c2::Query{}, [&states](const c2::ActionCommand_Service_Entity& transition) {
        if (transition.action_command_status) {
          const auto& status = *transition.action_command_status;
          if (status.message_data.base.command_id.uuid == kCommandId) {
            states.push_back(status.message_data.base.command_processing_state ==
                                     uci::CommandProcessingStateEnum::Received
                                 ? "RECEIVED"
                                 : "ACCEPTED");
          }
        }
      });
  if (!subscription.valid()) return 2;
  std::this_thread::sleep_for(300ms);

  c2::ActionCommand_Service_Request request;
  uci::ActionCommandMT command;
  fillUciEnvelope(command.security_information, command.message_header);
  uci::ActionCommandType command_item;
  command_item.capability.emplace();
  command_item.capability->base.command_id.uuid = kCommandId;
  command_item.capability->base.command_state = uci::CommandStateEnum::New;
  command_item.capability->capability_id.uuid = "6da3573c-2863-47a9-80cd-52b1ee2b4077";
  command_item.capability->action_id.base.uuid = "eacfc454-2741-4d15-aeac-15c573d94090";
  command.message_data.command.push_back(std::move(command_item));
  request.action_command = std::move(command);
  const auto submit = component.client().submit(request).get();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds);
  while (states.size() < 2 && std::chrono::steady_clock::now() < deadline) executor.spinOnce(10);
  const bool ok = submit.accepted && states.size() == 2 && states[0] == "RECEIVED" &&
                  states[1] == "ACCEPTED";
  if (ok) writeText(output, "seam-ok\n");
  subscription.cancel();
  executor.remove(component);
  pcl_transport_routing_destroy(routing);
  std::filesystem::remove(manifest);
  return ok ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 6 && std::strcmp(argv[1], "provider") == 0) {
    return runProvider(argv[2], argv[3], argv[4], std::stoi(argv[5]));
  }
  if (argc == 6 && std::strcmp(argv[1], "consumer") == 0) {
    return runConsumer(argv[2], argv[3], argv[4], std::stoi(argv[5]));
  }
  std::fprintf(stderr, "usage: %s provider|consumer <url> <ready-or-output> <scratch> <timeout>\n", argv[0]);
  return 2;
}
