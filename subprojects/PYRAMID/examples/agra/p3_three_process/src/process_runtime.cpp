#include "agra_p3_example/process_runtime.hpp"

#include <pcl/pcl_container.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace agra_p3_example {

namespace {

std::chrono::seconds parseDuration(int argc, char** argv) {
  constexpr const char* kPrefix = "--duration-seconds=";
  for (int index = 2; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument.rfind(kPrefix, 0) == 0) {
      const auto prefix_length = std::char_traits<char>::length(kPrefix);
      const int seconds = std::stoi(argument.substr(prefix_length));
      if (seconds <= 0) {
        throw std::invalid_argument("duration must be greater than zero");
      }
      return std::chrono::seconds(seconds);
    }
    throw std::invalid_argument("unknown argument: " + argument);
  }
  return std::chrono::seconds(15);
}

void requireOk(pcl_status_t status, const std::string& operation) {
  if (status != PCL_OK) {
    throw std::runtime_error(operation + " failed with PCL status " +
                             std::to_string(static_cast<int>(status)));
  }
}

}  // namespace

ProcessRuntime::ProcessRuntime(int argc, char** argv,
                               const char* codec_plugin_path) {
  if (argc < 2) {
    throw std::invalid_argument(
        "usage: <process> <routing-manifest> [--duration-seconds=N]");
  }
  duration_ = parseDuration(argc, argv);
  loadCodec(codec_plugin_path);
  loadRouting(argv[1]);
}

ProcessRuntime::~ProcessRuntime() {
  for (auto* gateway : gateways_) {
    executor_.remove(gateway);
    pcl_container_deactivate(gateway);
    pcl_container_cleanup(gateway);
  }
  gateways_.clear();
  pcl_transport_routing_destroy(routing_);

  // The process-wide codec registry borrows the plugin vtable. There is no
  // unregister operation, so the plugin remains resident until process exit.
  codec_plugin_ = nullptr;
}

int ProcessRuntime::run(pcl::Component& component) {
  requireOk(component.configure(),
            std::string("configure component '") + component.name() + "'");
  requireOk(component.activate(),
            std::string("activate component '") + component.name() + "'");
  requireOk(executor_.add(component),
            std::string("add component '") + component.name() + "'");

  std::cout << "[runtime] " << component.name() << " running for "
            << duration_.count() << " seconds" << std::endl;
  const auto deadline = std::chrono::steady_clock::now() + duration_;
  while (std::chrono::steady_clock::now() < deadline) {
    requireOk(executor_.spinOnce(20), "executor spin");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  requireOk(executor_.remove(component),
            std::string("remove component '") + component.name() + "'");
  requireOk(component.deactivate(),
            std::string("deactivate component '") + component.name() + "'");
  requireOk(component.cleanup(),
            std::string("clean up component '") + component.name() + "'");
  return 0;
}

void ProcessRuntime::loadCodec(const char* codec_plugin_path) {
  if (!codec_plugin_path || codec_plugin_path[0] == '\0') {
    throw std::invalid_argument("the P3 C2 codec plugin path is empty");
  }
  requireOk(pcl_plugin_load_codec(codec_plugin_path, nullptr,
                                  pcl_codec_registry_default(),
                                  &codec_plugin_),
            std::string("load codec plugin '") + codec_plugin_path + "'");
}

void ProcessRuntime::loadRouting(const std::string& manifest_path) {
  const auto peer_ids = readTransportPeerIds(manifest_path);
  char diagnostic[1024] = {};
  const pcl_status_t status = pcl_transport_routing_load(
      executor_.handle(), manifest_path.c_str(), &routing_, diagnostic,
      sizeof(diagnostic));
  if (status != PCL_OK) {
    throw std::runtime_error("load routing manifest '" + manifest_path +
                             "' failed: " + diagnostic);
  }
  activateGateways(peer_ids);
}

void ProcessRuntime::activateGateways(
    const std::vector<std::string>& peer_ids) {
  for (const auto& peer_id : peer_ids) {
    pcl_container_t* gateway = nullptr;
    requireOk(pcl_transport_routing_get_gateway(routing_, peer_id.c_str(),
                                                &gateway),
              "get gateway for peer '" + peer_id + "'");
    if (!gateway) {
      continue;
    }
    requireOk(pcl_container_configure(gateway),
              "configure gateway for peer '" + peer_id + "'");
    requireOk(pcl_container_activate(gateway),
              "activate gateway for peer '" + peer_id + "'");
    requireOk(executor_.add(gateway),
              "add gateway for peer '" + peer_id + "'");
    gateways_.push_back(gateway);
  }
}

std::vector<std::string> ProcessRuntime::readTransportPeerIds(
    const std::string& manifest_path) {
  std::ifstream manifest(manifest_path);
  if (!manifest) {
    throw std::runtime_error("cannot open routing manifest '" + manifest_path +
                             "'");
  }

  std::vector<std::string> peer_ids;
  std::string line;
  while (std::getline(manifest, line)) {
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::istringstream fields(line.substr(first));
    std::string directive;
    std::string peer_id;
    fields >> directive >> peer_id;
    if (directive == "transport" && !peer_id.empty() &&
        std::find(peer_ids.begin(), peer_ids.end(), peer_id) == peer_ids.end()) {
      peer_ids.push_back(peer_id);
    }
  }
  return peer_ids;
}

}  // namespace agra_p3_example
