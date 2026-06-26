#include <StandardBridge.h>
#include <TacticalObjectsComponent.h>

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "tactical_objects_codec_plugin_paths.hpp"

#include <pcl/component.hpp>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
}

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace consumed = pyramid::components::tactical_objects::services::consumed;
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
    addService(consumed::kSvcObjectSolutionEvidenceCreateRequirement, content_type_.c_str(),
               handleCreateRequirement, this);
    return PCL_OK;
  }

private:
  class Handler : public consumed::ServiceHandler {
  public:
    pyramid::domain_model::Identifier
    handleObjectSolutionEvidenceCreateRequirement(
        const pyramid::domain_model::ObjectEvidenceRequirement&) override {
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
    consumed::dispatch(
        handler, consumed::ServiceChannel::ObjectSolutionEvidenceCreateRequirement,
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

    pyramid::domain_model::ObjectDetail evidence;
    evidence.id = "bridge-e2e-evidence";
    evidence.entity_source = "demo-radar";
    evidence.identity = pyramid::domain_model::StandardIdentity::Hostile;
    evidence.dimension = pyramid::domain_model::BattleDimension::SeaSurface;
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
  std::vector<std::string> codec_plugin_paths;
  std::string codec_config;
  std::string codec_manifest;
  std::string socket_transport_plugin =
      kTacticalObjectsSocketTransportPlugin ? kTacticalObjectsSocketTransportPlugin
                                            : "";
  std::string shm_transport_plugin =
      kTacticalObjectsShmTransportPlugin ? kTacticalObjectsShmTransportPlugin : "";

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
    } else if (std::strcmp(argv[i], "--codec-plugin") == 0 && i + 1 < argc) {
      codec_plugin_paths.push_back(argv[++i]);
    } else if (std::strcmp(argv[i], "--codec-config") == 0 && i + 1 < argc) {
      codec_config = argv[++i];
    } else if (std::strcmp(argv[i], "--transport-plugin") == 0 && i + 1 < argc) {
      socket_transport_plugin = argv[++i];
    } else if (std::strcmp(argv[i], "--shm-transport-plugin") == 0 &&
               i + 1 < argc) {
      shm_transport_plugin = argv[++i];
    } else if (std::strcmp(argv[i], "--codec-manifest") == 0 && i + 1 < argc) {
      codec_manifest = argv[++i];
    }
  }
  const char* codec_config_json =
      codec_config.empty() ? nullptr : codec_config.c_str();
  if (codec_manifest.empty()) {
    if (const char* env = std::getenv("PCL_CODEC_MANIFEST")) {
      codec_manifest = env;
    }
  }

  if (codec_plugin_paths.empty()) {
    // Deployment default: load codecs from a manifest when supplied, else from
    // the build-tree default plugin set.
    if (!codec_manifest.empty()) {
      if (pcl_codec_registry_load_plugins_from_manifest(
              pcl_codec_registry_default(), codec_manifest.c_str()) != PCL_OK) {
        std::fprintf(stderr, "failed to load codec manifest: %s\n",
                     codec_manifest.c_str());
        return 2;
      }
    } else {
      const auto rc = pcl_codec_registry_load_plugins_from_paths(
          pcl_codec_registry_default(),
          kTacticalObjectsDefaultCodecPlugins.data(),
          kTacticalObjectsDefaultCodecPlugins.size());
      if (rc != PCL_OK) {
        std::fprintf(stderr, "failed to load default codec plugins\n");
        return 2;
      }
    }
  }

  std::vector<pcl_plugin_handle_t*> codec_plugin_handles;
  for (const auto& path : codec_plugin_paths) {
    pcl_plugin_handle_t* handle = nullptr;
    if (pcl_plugin_load_codec(path.c_str(), codec_config_json,
                              pcl_codec_registry_default(),
                              &handle) != PCL_OK) {
      std::fprintf(stderr, "failed to load codec plugin: %s\n", path.c_str());
      return 2;
    }
    codec_plugin_handles.push_back(handle);
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

  // Transport is loaded entirely from a runtime .so: the app links no transport
  // library. Socket and shared-memory both arrive as plugins, configured via the
  // same opaque config_json the codec plugins receive (with the executor pointer
  // threaded through). The plugin also supplies the gateway + destroy hooks.
  using GatewayFn = pcl_container_t* (*)(const pcl_transport_t*);
  using TransportDestroyFn = void (*)(const pcl_transport_t*);

  pcl_plugin_handle_t* transport_handle = nullptr;
  const pcl_transport_t* transport_vtable = nullptr;
  const char* transport_destroy_sym = nullptr;
  pcl_container_t* gateway = nullptr;

  std::string transport_plugin;
  std::string transport_config;
  const char* gateway_sym = nullptr;

  if (!shmem_bus.empty()) {
    std::fprintf(stderr,
                 "[tactical_objects_app] Joining shared-memory bus '%s'...\n",
                 shmem_bus.c_str());
    transport_plugin = shm_transport_plugin;
    std::ostringstream cfg;
    cfg << "{\"bus_name\":\"" << shmem_bus
        << "\",\"participant_id\":\"tactical_objects_app\",\"executor\":"
        << static_cast<unsigned long long>(
               reinterpret_cast<std::uintptr_t>(remote_exec))
        << "}";
    transport_config = cfg.str();
    gateway_sym = "pcl_shm_transport_plugin_gateway";
    transport_destroy_sym = "pcl_shm_transport_plugin_destroy";
  } else {
    if (!port_file.empty()) {
      std::ofstream stream(port_file);
      stream << port << '\n';
    }
    std::fprintf(stderr,
                 "[tactical_objects_app] Waiting for remote PYRAMID client on port %u...\n",
                 port);
    transport_plugin = socket_transport_plugin;
    std::ostringstream cfg;
    cfg << "{\"role\":\"server\",\"port\":" << port << ",\"executor\":"
        << static_cast<unsigned long long>(
               reinterpret_cast<std::uintptr_t>(remote_exec))
        << "}";
    transport_config = cfg.str();
    gateway_sym = "pcl_socket_transport_plugin_gateway";
    transport_destroy_sym = "pcl_socket_transport_plugin_destroy";
  }

  if (transport_plugin.empty()) {
    std::fprintf(stderr,
                 "[tactical_objects_app] no transport plugin: pass "
                 "--transport-plugin / --shm-transport-plugin\n");
    pcl_executor_destroy(remote_exec);
    pcl_executor_destroy(local_exec);
    return 2;
  }

  if (pcl_plugin_load_transport(transport_plugin.c_str(),
                                transport_config.c_str(), &transport_handle,
                                &transport_vtable) != PCL_OK ||
      !transport_vtable) {
    std::fprintf(stderr,
                 "[tactical_objects_app] Failed to load transport plugin %s\n",
                 transport_plugin.c_str());
    pcl_executor_destroy(remote_exec);
    pcl_executor_destroy(local_exec);
    return 1;
  }
  pcl_executor_set_transport(remote_exec, transport_vtable);

  auto gateway_fn = reinterpret_cast<GatewayFn>(
      pcl_plugin_symbol(transport_handle, gateway_sym));
  gateway = gateway_fn ? gateway_fn(transport_vtable) : nullptr;
  if (!gateway) {
    std::fprintf(stderr,
                 "[tactical_objects_app] transport plugin exposed no gateway\n");
    pcl_executor_destroy(remote_exec);
    pcl_executor_destroy(local_exec);
    return 1;
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

  pcl_executor_destroy(remote_exec);
  pcl_executor_destroy(local_exec);
  if (transport_handle) {
    auto destroy_fn = reinterpret_cast<TransportDestroyFn>(
        pcl_plugin_symbol(transport_handle, transport_destroy_sym));
    if (destroy_fn && transport_vtable) {
      destroy_fn(transport_vtable);
    }
    pcl_plugin_unload(transport_handle);
  }
  for (auto* handle : codec_plugin_handles) {
    pcl_plugin_unload(handle);
  }
  return 0;
}
