#include <StandardBridge.h>
#include <TacticalObjectsComponent.h>

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"

#include <nlohmann/json.hpp>
#include <pcl/component.hpp>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_socket.h>
#include <uuid/UUIDHelper.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

namespace {

using json = nlohmann::json;
namespace consumed = pyramid::services::tactical_objects::consumed;

std::atomic<bool> g_shutdown{false};

void signalHandler(int) {
  g_shutdown.store(true);
}

class LocalConsumedBridge : public pcl::Component {
public:
  LocalConsumedBridge(tactical_objects::TacticalObjectsRuntime& runtime,
                      pcl_executor_t* exec)
    : pcl::Component("tactical_objects_local_consumed_bridge"),
      runtime_(runtime),
      exec_(exec) {}

  void publishDemoEvidence() {
    if (!exec_) {
      return;
    }

    publish_buffer_ = "{\"identity\":\"STANDARD_IDENTITY_HOSTILE\","
                      "\"dimension\":\"BATTLE_DIMENSION_SEA_SURFACE\","
                      "\"latitude_rad\":0.8989737191417272,"
                      "\"longitude_rad\":-0.002230530784048753,"
                      "\"confidence\":0.95,"
                      "\"observed_at\":1.0}";

    pcl_msg_t msg{};
    msg.data = publish_buffer_.data();
    msg.size = static_cast<uint32_t>(publish_buffer_.size());
    msg.type_name = "application/json";
    pcl_executor_post_incoming(exec_, consumed::kTopicObjectEvidence, &msg);
  }

protected:
  pcl_status_t on_configure() override {
    addSubscriber(consumed::kTopicObjectEvidence, "application/json",
                  onStandardObjectEvidence, this);
    addSubscriber("evidence_requirements", "application/json",
                  onInternalEvidenceRequirements, this);
    evidence_requirements_port_ =
        addPublisher(provided_evidence_topic_, "application/json");
    return PCL_OK;
  }

private:
  static void onStandardObjectEvidence(pcl_container_t*, const pcl_msg_t* msg,
                                       void* user_data) {
    auto* self = static_cast<LocalConsumedBridge*>(user_data);
    if (!msg || !msg->data || msg->size == 0) {
      return;
    }

    std::string str(static_cast<const char*>(msg->data), msg->size);
    json j;
    try {
      j = json::parse(str);
    } catch (...) {
      return;
    }

    tactical_objects::Observation obs;
    obs.observation_id = pyramid::core::uuid::UUIDHelper::generateV4();
    obs.observed_at = j.value("observed_at", 0.0);
    obs.object_hint_type = tactical_objects::ObjectType::Platform;
    obs.affiliation_hint = tactical_objects::Affiliation::Unknown;
    obs.confidence = j.value("confidence", 0.0);

    obs.position.lat = radToDeg(j.value("latitude_rad", 0.0));
    obs.position.lon = radToDeg(j.value("longitude_rad", 0.0));
    obs.position.alt = 0.0;

    const std::string identity = j.value("identity", "");
    if (identity == "STANDARD_IDENTITY_HOSTILE") {
      obs.affiliation_hint = tactical_objects::Affiliation::Hostile;
    } else if (identity == "STANDARD_IDENTITY_FRIENDLY") {
      obs.affiliation_hint = tactical_objects::Affiliation::Friendly;
    } else if (identity == "STANDARD_IDENTITY_NEUTRAL") {
      obs.affiliation_hint = tactical_objects::Affiliation::Neutral;
    }

    const std::string dimension = j.value("dimension", "");
    if (dimension == "BATTLE_DIMENSION_SEA_SURFACE") {
      obs.source_sidc = "SHSP------*****";
    } else if (dimension == "BATTLE_DIMENSION_AIR") {
      obs.source_sidc = "SHAP------*****";
    } else if (dimension == "BATTLE_DIMENSION_SUBSURFACE") {
      obs.source_sidc = "SHUP------*****";
    } else if (dimension == "BATTLE_DIMENSION_GROUND") {
      obs.source_sidc = "SHGP------*****";
    }

    tactical_objects::ObservationBatch batch;
    batch.observations.push_back(obs);
    self->runtime_.processObservationBatch(batch);
  }

  static void onInternalEvidenceRequirements(pcl_container_t*, const pcl_msg_t* msg,
                                             void* user_data) {
    auto* self = static_cast<LocalConsumedBridge*>(user_data);
    if (!msg || !msg->data || msg->size == 0) {
      return;
    }

    std::string payload(static_cast<const char*>(msg->data), msg->size);

    if (self->evidence_requirements_port_) {
      pcl_msg_t pub{};
      pub.data = payload.data();
      pub.size = static_cast<uint32_t>(payload.size());
      pub.type_name = "application/json";
      pcl_port_publish(self->evidence_requirements_port_, &pub);
    }

    if (!self->exec_) {
      return;
    }

    pcl_msg_t req{};
    req.data = payload.data();
    req.size = static_cast<uint32_t>(payload.size());
    req.type_name = "application/json";

    char resp_buf[1024] = {};
    pcl_msg_t resp{};
    resp.data = resp_buf;
    resp.size = sizeof(resp_buf);
    resp.type_name = "application/json";

    const auto rc = pcl_executor_invoke_service(
        self->exec_, consumed::kSvcCreateRequirement, &req, &resp);
    if (rc != PCL_OK) {
      self->logWarn("Local consumed service %s unavailable",
                    consumed::kSvcCreateRequirement);
    }
  }

  static double radToDeg(double rad) {
    return rad * 57.29577951308232;
  }

  tactical_objects::TacticalObjectsRuntime& runtime_;
  pcl_executor_t* exec_ = nullptr;
  pcl_port_t* evidence_requirements_port_ = nullptr;
  std::string publish_buffer_;
  static constexpr const char* provided_evidence_topic_ = "standard.evidence_requirements";
};

class LocalEvidenceNodeStub : public pcl::Component {
public:
  LocalEvidenceNodeStub() : pcl::Component("tactical_objects_local_node_stub") {}

protected:
  pcl_status_t on_configure() override {
    addSubscriber("standard.evidence_requirements", "application/json",
                  onEvidenceRequirements, this);
    addService(consumed::kSvcCreateRequirement, "application/json",
               handleCreateRequirement, this);
    return PCL_OK;
  }

private:
  static void onEvidenceRequirements(pcl_container_t*, const pcl_msg_t* msg,
                                     void* user_data) {
    auto* self = static_cast<LocalEvidenceNodeStub*>(user_data);
    if (!msg || !msg->data || msg->size == 0) {
      return;
    }

    const std::string payload(static_cast<const char*>(msg->data), msg->size);
    self->logInfo("[LocalNodeStub] received standard.evidence_requirements: %s",
                  payload.c_str());
  }

  static pcl_status_t handleCreateRequirement(pcl_container_t*, const pcl_msg_t* request,
                                              pcl_msg_t* response, pcl_svc_context_t*,
                                              void* user_data) {
    auto* self = static_cast<LocalEvidenceNodeStub*>(user_data);
    if (!request || !request->data) {
      return PCL_ERR_INVALID;
    }

    const std::string payload(static_cast<const char*>(request->data), request->size);
    self->logInfo("[LocalNodeStub] service %s called with payload: %s",
                  consumed::kSvcCreateRequirement, payload.c_str());

    self->response_buffer_ = "\"stub-evidence-requirement-001\"";
    response->data = self->response_buffer_.data();
    response->size = static_cast<uint32_t>(self->response_buffer_.size());
    response->type_name = "application/json";
    return PCL_OK;
  }

  std::string response_buffer_;
};

} // namespace

int main(int argc, char* argv[]) {
  uint16_t port = 19123;
  std::string port_file;
  int timeout_secs = 0;
  bool emit_demo_evidence = false;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--port-file") == 0 && i + 1 < argc) {
      port_file = argv[++i];
    } else if (std::strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
      timeout_secs = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--demo-evidence") == 0) {
      emit_demo_evidence = true;
    }
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  pcl_executor_t* local_exec = pcl_executor_create();
  pcl_executor_t* remote_exec = pcl_executor_create();
  if (!local_exec || !remote_exec) {
    std::fprintf(stderr, "[tactical_objects_app] Failed to create executors\n");
    if (remote_exec) pcl_executor_destroy(remote_exec);
    if (local_exec) pcl_executor_destroy(local_exec);
    return 1;
  }

  tactical_objects::TacticalObjectsComponent tactical_objects_component;
  tactical_objects_component.configure();
  tactical_objects_component.activate();
  tactical_objects_component.setTickRateHz(100.0);
  pcl_executor_add(local_exec, tactical_objects_component.handle());

  LocalConsumedBridge local_consumed_bridge(
      tactical_objects_component.runtime(), local_exec);
  local_consumed_bridge.configure();
  local_consumed_bridge.activate();
  local_consumed_bridge.setTickRateHz(100.0);
  pcl_executor_add(local_exec, local_consumed_bridge.handle());

  LocalEvidenceNodeStub local_node_stub;
  local_node_stub.configure();
  local_node_stub.activate();
  local_node_stub.setTickRateHz(100.0);
  pcl_executor_add(local_exec, local_node_stub.handle());

  tactical_objects::StandardBridge standard_bridge(
      tactical_objects_component.runtime(), local_exec, false);
  standard_bridge.configure();
  standard_bridge.activate();
  standard_bridge.setTickRateHz(100.0);
  pcl_executor_add(remote_exec, standard_bridge.handle());

  if (!port_file.empty()) {
    std::ofstream stream(port_file);
    stream << port << '\n';
  }

  std::fprintf(stderr,
               "[tactical_objects_app] Waiting for remote PYRAMID client on port %u...\n",
               port);

  pcl_socket_transport_t* transport =
      pcl_socket_transport_create_server(port, remote_exec);
  if (!transport) {
    std::fprintf(stderr, "[tactical_objects_app] Failed to create socket server\n");
    pcl_executor_destroy(remote_exec);
    pcl_executor_destroy(local_exec);
    return 1;
  }

  pcl_executor_set_transport(remote_exec, pcl_socket_transport_get_transport(transport));

  pcl_container_t* gateway = pcl_socket_transport_gateway_container(transport);
  pcl_container_configure(gateway);
  pcl_container_activate(gateway);
  pcl_executor_add(remote_exec, gateway);

  std::fprintf(stderr,
               "[tactical_objects_app] Remote provided interface is live on port %u.\n",
               port);
  std::fprintf(stderr,
               "[tactical_objects_app] Local consumed interface is exposed on the local executor.\n");

  if (emit_demo_evidence) {
    local_consumed_bridge.publishDemoEvidence();
  }

  const auto start = std::chrono::steady_clock::now();
  auto last_demo_publish = start;
  while (!g_shutdown.load()) {
    pcl_executor_spin_once(local_exec, 0);
    pcl_executor_spin_once(remote_exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if (emit_demo_evidence) {
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(
              now - last_demo_publish).count() >= 1000) {
        local_consumed_bridge.publishDemoEvidence();
        last_demo_publish = now;
      }
    }

    if (timeout_secs > 0) {
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::steady_clock::now() - start)
              .count();
      if (elapsed >= timeout_secs) {
        break;
      }
    }
  }

  pcl_socket_transport_destroy(transport);
  pcl_executor_destroy(remote_exec);
  pcl_executor_destroy(local_exec);
  return 0;
}
