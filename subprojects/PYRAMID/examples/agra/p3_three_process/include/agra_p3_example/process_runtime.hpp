#pragma once

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>

#include <chrono>
#include <string>
#include <vector>

namespace agra_p3_example {

/// \brief Owns the PCL executor and deployment-selected transport routing.
class ProcessRuntime {
 public:
  /// \brief Load the codec and routing manifest selected on the command line.
  ProcessRuntime(int argc, char** argv, const char* codec_plugin_path);
  ~ProcessRuntime();

  ProcessRuntime(const ProcessRuntime&) = delete;
  ProcessRuntime& operator=(const ProcessRuntime&) = delete;

  /// \brief Return the executor used to compose the process component.
  pcl::Executor& executor() { return executor_; }

  /// \brief Run one component until the configured example duration expires.
  int run(pcl::Component& component);

 private:
  void loadCodec(const char* codec_plugin_path);
  void loadRouting(const std::string& manifest_path);
  void activateGateways(const std::vector<std::string>& peer_ids);
  static std::vector<std::string> readTransportPeerIds(
      const std::string& manifest_path);

  pcl::Executor executor_;
  pcl_plugin_handle_t* codec_plugin_ = nullptr;
  pcl_transport_routing_t* routing_ = nullptr;
  std::vector<pcl_container_t*> gateways_;
  std::chrono::seconds duration_{15};
};

}  // namespace agra_p3_example
