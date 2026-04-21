#include <StandardBridge.h>
#include <TacticalObjectsComponent.h>

#include "pyramid_services_tactical_objects_consumed.hpp"

#include <pcl/component.hpp>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_shared_memory.h>
#include <pcl/pcl_transport_socket.h>

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

namespace consumed = pyramid::services::tactical_objects::consumed;
constexpr const char* kJsonContentType = "application/json";
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

std::atomic<bool> g_shutdown{false};

void signalHandler(int) {
  g_shutdown.store(true);
}

class LocalEvidenceNodeStub : public pcl::Component {
public:
  explicit LocalEvidenceNodeStub(std::string content_type)
      : pcl::Component("tactical_objects_local_node_stub"),
        content_type_(std::move(content_type)) {}

protected:
  pcl_status_t on_configure() override {
    addSubscriber("standard.evidence_requirements", content_type_.c_str(),
                  onEvidenceRequirements, this);
    addService(consumed::kSvcCreateRequirement, content_type_.c_str(),
               handleCreateRequirement, this);
    return PCL_OK;
  }

private:
  class Handler : public consumed::ServiceHandler {
  public:
    pyramid::data_model::Identifier
    handleCreateRequirement(const pyramid::data_model::ObjectEvidenceRequirement&) override {
      return "stub-evidence-requirement-001";
    }
  };

  static void onEvidenceRequirements(pcl_container_t*, const pcl_msg_t* msg,
                                     void* user_data) {
    auto* self = static_cast<LocalEvidenceNodeStub*>(user_data);
    if (!msg || !msg->data || msg->size == 0) {
      return;
    }

    const std::string payload(static_cast<const char*>(msg->data), msg->size);
    self->logInfo("[LocalNodeStub] received standard.evidence_requirements (%s) size=%u",
                  msg->type_name ? msg->type_name : "<null>",
                  static_cast<unsigned>(payload.size()));
  }

  static pcl_status_t handleCreateRequirement(pcl_container_t*, const pcl_msg_t* request,
                                              pcl_msg_t* response, pcl_svc_context_t*,
                                              void* user_data) {
    auto* self = static_cast<LocalEvidenceNodeStub*>(user_data);
    if (!request) {
      return PCL_ERR_INVALID;
    }

    Handler handler;
    void* response_buf = nullptr;
    size_t response_size = 0;
    consumed::dispatch(handler, consumed::ServiceChannel::CreateRequirement,
                       request->data, request->size, self->content_type_.c_str(),
                       &response_buf, &response_size);

    self->response_buffer_.clear();
    if (response_buf && response_size > 0) {
      self->response_buffer_.assign(static_cast<const char*>(response_buf), response_size);
      std::free(response_buf);
    }

    response->data = self->response_buffer_.empty()
                         ? nullptr
                         : const_cast<char*>(self->response_buffer_.data());
    response->size = static_cast<uint32_t>(self->response_buffer_.size());
    response->type_name = self->content_type_.c_str();
    return PCL_OK;
  }

  std::string content_type_;
  std::string response_buffer_;
};

class DemoEvidencePublisher : public pcl::Component {
public:
  explicit DemoEvidencePublisher(std::string content_type)
      : pcl::Component("tactical_objects_demo_evidence_publisher"),
        content_type_(std::move(content_type)) {}

protected:
  pcl_status_t on_configure() override {
    publisher_ = addPublisher(consumed::kTopicObjectEvidence, content_type_.c_str());
    return publisher_ ? PCL_OK : PCL_ERR_CALLBACK;
  }

  pcl_status_t on_tick(double dt) override {
    elapsed_since_publish_ += dt;
    if (published_count_ > 0 && elapsed_since_publish_ < 1.0) {
      return PCL_OK;
    }

    pyramid::data_model::ObjectDetail evidence;
    evidence.id = "bridge-e2e-evidence";
    evidence.entity_source = "demo-radar";
    evidence.identity = pyramid::data_model::StandardIdentity::Hostile;
    evidence.dimension = pyramid::data_model::BattleDimension::SeaSurface;
    evidence.position.latitude = 51.0 * kDegToRad;
    evidence.position.longitude = 0.0;
    evidence.quality = 0.95;
    evidence.creation_time = 1.0;

    const pcl_status_t rc =
        consumed::publishObjectEvidence(publisher_, evidence, content_type_.c_str());
    if (rc == PCL_OK) {
      ++published_count_;
      elapsed_since_publish_ = 0.0;
      logInfo("[DemoEvidence] published standard.object_evidence id=%s source=%s",
              evidence.id.c_str(), evidence.entity_source.c_str());
    } else {
      logWarn("[DemoEvidence] failed to publish standard.object_evidence rc=%d",
              static_cast<int>(rc));
    }
    return PCL_OK;
  }

private:
  std::string content_type_;
  pcl_port_t* publisher_ = nullptr;
  double elapsed_since_publish_ = 1.0;
  int published_count_ = 0;
};

} // namespace

int main(int argc, char* argv[]) {
  uint16_t port = 19123;
  std::string port_file;
  std::string shmem_bus;
  int timeout_secs = 0;
  bool emit_demo_evidence = false;
  std::string frontend_content_type = kJsonContentType;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--port-file") == 0 && i + 1 < argc) {
      port_file = argv[++i];
    } else if (std::strcmp(argv[i], "--shmem-bus") == 0 && i + 1 < argc) {
      shmem_bus = argv[++i];
    } else if (std::strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
      timeout_secs = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--demo-evidence") == 0) {
      emit_demo_evidence = true;
    } else if ((std::strcmp(argv[i], "--content-type") == 0 ||
                std::strcmp(argv[i], "--frontend-content-type") == 0) &&
               i + 1 < argc) {
      frontend_content_type = argv[++i];
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

  LocalEvidenceNodeStub local_node_stub(frontend_content_type);
  local_node_stub.configure();
  local_node_stub.activate();
  local_node_stub.setTickRateHz(100.0);
  pcl_executor_add(local_exec, local_node_stub.handle());

  tactical_objects::StandardBridge standard_bridge(
      tactical_objects_component.runtime(), local_exec, frontend_content_type, true);
  standard_bridge.configure();
  standard_bridge.activate();
  standard_bridge.setTickRateHz(100.0);
  pcl_executor_add(remote_exec, standard_bridge.handle());

  DemoEvidencePublisher demo_evidence_publisher(frontend_content_type);
  if (emit_demo_evidence) {
    demo_evidence_publisher.configure();
    demo_evidence_publisher.activate();
    demo_evidence_publisher.setTickRateHz(10.0);
    pcl_executor_add(remote_exec, demo_evidence_publisher.handle());
  }

  pcl_socket_transport_t* socket_transport = nullptr;
  pcl_shared_memory_transport_t* shmem_transport = nullptr;
  pcl_container_t* gateway = nullptr;

  if (!shmem_bus.empty()) {
    std::fprintf(stderr,
                 "[tactical_objects_app] Joining shared-memory bus '%s'...\n",
                 shmem_bus.c_str());
    shmem_transport = pcl_shared_memory_transport_create(
        shmem_bus.c_str(), "tactical_objects_app", remote_exec);
    if (!shmem_transport) {
      std::fprintf(stderr,
                   "[tactical_objects_app] Failed to join shared-memory bus '%s'\n",
                   shmem_bus.c_str());
      pcl_executor_destroy(remote_exec);
      pcl_executor_destroy(local_exec);
      return 1;
    }
    pcl_executor_set_transport(
        remote_exec, pcl_shared_memory_transport_get_transport(shmem_transport));
    gateway = pcl_shared_memory_transport_gateway_container(shmem_transport);
  } else {
    if (!port_file.empty()) {
      std::ofstream stream(port_file);
      stream << port << '\n';
    }

    std::fprintf(stderr,
                 "[tactical_objects_app] Waiting for remote PYRAMID client on port %u...\n",
                 port);
    socket_transport = pcl_socket_transport_create_server(port, remote_exec);
    if (!socket_transport) {
      std::fprintf(stderr, "[tactical_objects_app] Failed to create socket server\n");
      pcl_executor_destroy(remote_exec);
      pcl_executor_destroy(local_exec);
      return 1;
    }
    pcl_executor_set_transport(
        remote_exec, pcl_socket_transport_get_transport(socket_transport));
    gateway = pcl_socket_transport_gateway_container(socket_transport);
  }

  pcl_container_configure(gateway);
  pcl_container_activate(gateway);
  pcl_executor_add(remote_exec, gateway);

  if (!port_file.empty() && !shmem_bus.empty()) {
    std::ofstream stream(port_file);
    stream << shmem_bus << '\n';
  }

  if (!shmem_bus.empty()) {
    std::fprintf(stderr,
                 "[tactical_objects_app] Remote provided interface is live on"
                 " shared-memory bus '%s' (%s).\n",
                 shmem_bus.c_str(), frontend_content_type.c_str());
  } else {
    std::fprintf(stderr,
                 "[tactical_objects_app] Remote provided interface is live on port %u (%s).\n",
                 port, frontend_content_type.c_str());
  }
  std::fprintf(stderr,
               "[tactical_objects_app] Local consumed interface is exposed on the local executor.\n");

  const auto start = std::chrono::steady_clock::now();
  while (!g_shutdown.load()) {
    pcl_executor_spin_once(local_exec, 0);
    pcl_executor_spin_once(remote_exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

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

  if (socket_transport) {
    pcl_socket_transport_destroy(socket_transport);
  }
  if (shmem_transport) {
    pcl_shared_memory_transport_destroy(shmem_transport);
  }
  pcl_executor_destroy(remote_exec);
  pcl_executor_destroy(local_exec);
  return 0;
}
