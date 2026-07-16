#pragma once

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>

#include <chrono>
#include <initializer_list>
#include <string>
#include <vector>

namespace agra_p3_example {

/// \brief One generated wire endpoint used by a logical port realization.
struct DeploymentEndpoint {
  /// \brief Generated wire endpoint name.
  std::string name;
  /// \brief Direction and interaction primitive required by PCL.
  pcl_endpoint_kind_t kind = PCL_ENDPOINT_CONSUMED;
};

/// \brief Generated endpoint alternatives associated with a configuration key.
struct DeploymentPort {
  /// \brief Copy a generated facade descriptor under a local config key.
  template <typename Descriptor>
  DeploymentPort(const char* port_name, const Descriptor& descriptor)
      : name(port_name ? port_name : "") {
    for (const auto& endpoint : descriptor.rpc_endpoints) {
      rpc_endpoints.push_back(
          {endpoint.endpoint_name, endpoint.endpoint_kind});
    }
    for (const auto& endpoint : descriptor.pubsub_endpoints) {
      pubsub_endpoints.push_back(
          {endpoint.endpoint_name, endpoint.endpoint_kind});
    }
  }

  /// \brief Local key used by the `.ports` deployment file.
  std::string name;
  /// \brief Endpoints selected when the port mode is `rpc`.
  std::vector<DeploymentEndpoint> rpc_endpoints;
  /// \brief Endpoints selected when the port mode is `pubsub`.
  std::vector<DeploymentEndpoint> pubsub_endpoints;
};

/// \brief Owns the PCL executor and deployment-selected transport routing.
class ProcessRuntime {
 public:
  /// \brief Load codecs and the per-port deployment configuration.
  ProcessRuntime(int argc, char** argv,
                 std::initializer_list<const char*> codec_plugin_paths,
                 std::initializer_list<DeploymentPort> deployment_ports);
  ~ProcessRuntime();

  ProcessRuntime(const ProcessRuntime&) = delete;
  ProcessRuntime& operator=(const ProcessRuntime&) = delete;

  /// \brief Return the executor used to compose the process component.
  pcl::Executor& executor() { return executor_; }

  /// \brief Run one component until the configured example duration expires.
  int run(pcl::Component& component);

 private:
  void loadCodecs(std::initializer_list<const char*> codec_plugin_paths);
  void loadPortConfiguration(
      const std::string& config_path,
      std::initializer_list<DeploymentPort> deployment_ports);
  void activateGateways(const std::vector<std::string>& peer_ids);

  pcl::Executor executor_;
  std::vector<pcl_plugin_handle_t*> codec_plugins_;
  pcl_transport_routing_t* routing_ = nullptr;
  std::vector<pcl_container_t*> gateways_;
  std::chrono::seconds duration_{15};
};

}  // namespace agra_p3_example
