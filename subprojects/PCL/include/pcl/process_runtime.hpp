/// \file process_runtime.hpp
/// \brief C++ process runtime over the stable PCL C API.
#ifndef PCL_PROCESS_RUNTIME_HPP
#define PCL_PROCESS_RUNTIME_HPP

#include "component.hpp"
#include "executor.hpp"
#include "pcl_process_runtime.h"

#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

namespace pcl {

/// \brief One generated wire endpoint used by a logical port realization.
struct DeploymentEndpoint {
  std::string name;
  pcl_endpoint_kind_t kind = PCL_ENDPOINT_CONSUMED;
};

/// \brief Generated endpoint alternatives associated with a `.ports` key.
struct DeploymentPort {
  template <typename Descriptor>
  DeploymentPort(const char* port_name, const Descriptor& descriptor)
      : name(port_name ? port_name : "") {
    for (const auto& endpoint : descriptor.rpc_endpoints) {
      rpc_endpoints.push_back(
          {endpoint.endpoint_name ? endpoint.endpoint_name : "",
           endpoint.endpoint_kind});
    }
    for (const auto& endpoint : descriptor.pubsub_endpoints) {
      pubsub_endpoints.push_back(
          {endpoint.endpoint_name ? endpoint.endpoint_name : "",
           endpoint.endpoint_kind});
    }
  }

  std::string name;
  std::vector<DeploymentEndpoint> rpc_endpoints;
  std::vector<DeploymentEndpoint> pubsub_endpoints;
};

/// \brief Owns process-level PCL lifecycle and deployment routing.
class ProcessRuntime {
public:
  ProcessRuntime(
      int argc,
      char** argv,
      std::initializer_list<const char*> codec_plugin_paths,
      std::initializer_list<DeploymentPort> deployment_ports)
      : ProcessRuntime(
            argc, argv, codec_plugin_paths,
            std::vector<DeploymentPort>(deployment_ports)) {}

  ProcessRuntime(
      int argc,
      char** argv,
      std::initializer_list<const char*> codec_plugin_paths,
      const std::vector<DeploymentPort>& deployment_ports)
      : runtime_(create(parseDuration(argc, argv))),
        executor_(pcl_process_runtime_executor(runtime_), false) {
    try {
      if (argc < 2 || !argv[1] || argv[1][0] == '\0') {
        throw std::invalid_argument(
            "usage: <process> <port-config> [--duration-seconds=N]");
      }
      for (const char* path : codec_plugin_paths) {
        require(pcl_process_runtime_load_codec(runtime_, path));
      }
      loadPorts(argv[1], deployment_ports);
    } catch (...) {
      pcl_process_runtime_destroy(runtime_);
      runtime_ = nullptr;
      throw;
    }
  }

  ~ProcessRuntime() {
    pcl_process_runtime_destroy(runtime_);
  }

  ProcessRuntime(const ProcessRuntime&) = delete;
  ProcessRuntime& operator=(const ProcessRuntime&) = delete;

  pcl::Executor& executor() { return executor_; }

  /// \brief Content type selected by the ports file's `codec` line.
  ///
  /// Defaults to "application/json" when the loaded ports file names no
  /// codec. Pass this to the component so its generated ports encode with
  /// the codec the deployment loaded.
  std::string contentType() const {
    const char* content_type = pcl_process_runtime_content_type(runtime_);
    return content_type ? content_type : "application/json";
  }

  /// \brief Content type selected for one port by a `port_codec` line.
  ///
  /// Falls back to contentType() (and then "application/json") when the port
  /// has no override. Pass this per port so each of a component's ports can
  /// encode with its own codec.
  std::string contentTypeFor(const std::string& port_name) const {
    const char* content_type =
        pcl_process_runtime_port_content_type(runtime_, port_name.c_str());
    return content_type ? content_type : "application/json";
  }

  int run(pcl::Component& component) {
    require(pcl_process_runtime_run(runtime_, component.handle()));
    return 0;
  }

  void requestShutdown() {
    pcl_process_runtime_request_shutdown(runtime_);
  }

private:
  static uint32_t parseDuration(int argc, char** argv) {
    constexpr const char* prefix = "--duration-seconds=";
    constexpr std::size_t prefix_length = 19u;
    uint32_t duration = 0u;
    for (int index = 2; index < argc; ++index) {
      const std::string argument(argv[index] ? argv[index] : "");
      if (argument.rfind(prefix, 0) != 0) {
        throw std::invalid_argument("unknown argument: " + argument);
      }
      const std::string value = argument.substr(prefix_length);
      std::size_t used = 0u;
      const unsigned long parsed = std::stoul(value, &used);
      if (used != value.size() || parsed > UINT32_MAX) {
        throw std::invalid_argument("invalid duration: " + value);
      }
      duration = static_cast<uint32_t>(parsed);
    }
    return duration;
  }

  static pcl_process_runtime_t* create(uint32_t duration) {
    pcl_process_runtime_t* runtime = nullptr;
    const pcl_status_t status =
        pcl_process_runtime_create(duration, &runtime);
    if (status != PCL_OK) {
      throw std::runtime_error(
          "create process runtime failed with PCL status " +
          std::to_string(static_cast<int>(status)));
    }
    return runtime;
  }

  void require(pcl_status_t status) const {
    if (status == PCL_OK) return;
    throw std::runtime_error(
        std::string(pcl_process_runtime_error(runtime_)) +
        " (PCL status " +
        std::to_string(static_cast<int>(status)) + ")");
  }

  void loadPorts(
      const char* config_path,
      const std::vector<DeploymentPort>& deployment_ports) {
    std::vector<std::vector<pcl_process_endpoint_descriptor_t>> rpc;
    std::vector<std::vector<pcl_process_endpoint_descriptor_t>> pubsub;
    std::vector<pcl_process_port_descriptor_t> ports;
    rpc.reserve(deployment_ports.size());
    pubsub.reserve(deployment_ports.size());
    ports.reserve(deployment_ports.size());
    for (const auto& port : deployment_ports) {
      rpc.emplace_back();
      pubsub.emplace_back();
      rpc.back().reserve(port.rpc_endpoints.size());
      pubsub.back().reserve(port.pubsub_endpoints.size());
      for (const auto& endpoint : port.rpc_endpoints) {
        rpc.back().push_back({endpoint.name.c_str(), endpoint.kind});
      }
      for (const auto& endpoint : port.pubsub_endpoints) {
        pubsub.back().push_back({endpoint.name.c_str(), endpoint.kind});
      }
      ports.push_back({
          port.name.c_str(),
          rpc.back().data(),
          rpc.back().size(),
          pubsub.back().data(),
          pubsub.back().size(),
      });
    }
    require(pcl_process_runtime_load_ports_file(
        runtime_, config_path, ports.data(), ports.size()));
  }

  pcl_process_runtime_t* runtime_ = nullptr;
  pcl::Executor executor_;
};

}  // namespace pcl

#endif
