#include "agra_p3_example/process_runtime.hpp"

#include <pcl/pcl_container.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

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
                               std::initializer_list<const char*>
                                   codec_plugin_paths,
                               std::initializer_list<DeploymentPort>
                                   deployment_ports) {
  if (argc < 2) {
    throw std::invalid_argument(
        "usage: <process> <port-config> [--duration-seconds=N]");
  }
  duration_ = parseDuration(argc, argv);
  loadCodecs(codec_plugin_paths);
  loadPortConfiguration(argv[1], deployment_ports);
}

ProcessRuntime::~ProcessRuntime() {
  for (auto* gateway : gateways_) {
    executor_.remove(gateway);
    pcl_container_deactivate(gateway);
    pcl_container_cleanup(gateway);
  }
  gateways_.clear();
  pcl_transport_routing_destroy(routing_);

  // The process-wide codec registry borrows each plugin vtable. There is no
  // unregister operation, so the plugins remain resident until process exit.
  codec_plugins_.clear();
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

void ProcessRuntime::loadCodecs(
    std::initializer_list<const char*> codec_plugin_paths) {
  for (const char* codec_plugin_path : codec_plugin_paths) {
    if (!codec_plugin_path || codec_plugin_path[0] == '\0') {
      throw std::invalid_argument("a P3 codec plugin path is empty");
    }
    pcl_plugin_handle_t* codec_plugin = nullptr;
    requireOk(pcl_plugin_load_codec(codec_plugin_path, nullptr,
                                    pcl_codec_registry_default(),
                                    &codec_plugin),
              std::string("load codec plugin '") + codec_plugin_path + "'");
    codec_plugins_.push_back(codec_plugin);
  }
}

void ProcessRuntime::loadPortConfiguration(
    const std::string& config_path,
    std::initializer_list<DeploymentPort> deployment_ports) {
  struct PortConfig {
    std::string mode;
    std::string peer;
    std::string plugin;
    std::string plugin_config;
  };

  const std::map<pcl_endpoint_kind_t, std::string> kind_names = {
      {PCL_ENDPOINT_PUBLISHER, "publisher"},
      {PCL_ENDPOINT_SUBSCRIBER, "subscriber"},
      {PCL_ENDPOINT_PROVIDED, "provided"},
      {PCL_ENDPOINT_CONSUMED, "consumed"},
      {PCL_ENDPOINT_STREAM_PROVIDED, "stream_provided"},
      {PCL_ENDPOINT_STREAM_CONSUMED, "stream_consumed"},
  };
  std::map<std::string, DeploymentPort> definitions;
  for (const auto& port : deployment_ports) {
    if (port.name.empty() || !definitions.emplace(port.name, port).second) {
      throw std::invalid_argument("duplicate or empty deployment port name");
    }
  }

  std::ifstream config(config_path);
  if (!config) {
    throw std::runtime_error("cannot open port config '" + config_path + "'");
  }
  std::map<std::string, PortConfig> configured_ports;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(config, line)) {
    ++line_number;
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::istringstream fields(line.substr(first));
    std::string directive;
    std::string name;
    PortConfig port_config;
    fields >> directive >> name >> port_config.mode >> port_config.peer >>
        port_config.plugin;
    std::getline(fields, port_config.plugin_config);
    const auto config_first =
        port_config.plugin_config.find_first_not_of(" \t\r");
    if (config_first != std::string::npos) {
      port_config.plugin_config.erase(0, config_first);
    }
    if (directive != "port" || name.empty() || port_config.peer.empty() ||
        port_config.plugin.empty() || port_config.plugin_config.empty() ||
        (port_config.mode != "rpc" && port_config.mode != "pubsub")) {
      throw std::runtime_error("invalid port config line " +
                               std::to_string(line_number));
    }
    if (definitions.find(name) == definitions.end()) {
      throw std::runtime_error("unknown port '" + name + "' on line " +
                               std::to_string(line_number));
    }
    if (!configured_ports.emplace(name, std::move(port_config)).second) {
      throw std::runtime_error("duplicate port '" + name + "'");
    }
  }
  if (configured_ports.size() != definitions.size()) {
    throw std::runtime_error("port config must contain each component port");
  }

  std::map<std::string, std::pair<std::string, std::string>> transports;
  for (const auto& entry : configured_ports) {
    const auto& value = entry.second;
    const auto transport = std::make_pair(value.plugin, value.plugin_config);
    const auto existing = transports.find(value.peer);
    if (existing != transports.end() && existing->second != transport) {
      throw std::runtime_error("peer '" + value.peer +
                               "' has conflicting plugin configurations");
    }
    transports[value.peer] = transport;
  }

  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto compiled_path =
      std::filesystem::temp_directory_path() /
      (std::filesystem::path(config_path).filename().string() + "." +
       std::to_string(stamp) + ".routes");
  {
    std::ofstream manifest(compiled_path);
    if (!manifest) {
      throw std::runtime_error("cannot create compiled routing manifest");
    }
    manifest << "# Generated from per-port configuration.\n";
    for (const auto& entry : transports) {
      manifest << "transport " << entry.first << " " << entry.second.first
               << " " << entry.second.second << "\n";
    }
    for (const auto& entry : configured_ports) {
      const auto& name = entry.first;
      const auto& value = entry.second;
      const auto& definition = definitions.at(name);
      manifest << "exclusive " << name << " ";
      for (std::size_t index = 0; index < definition.rpc_endpoints.size();
           ++index) {
        if (index > 0) manifest << ",";
        manifest << definition.rpc_endpoints[index].name;
      }
      manifest << " ";
      for (std::size_t index = 0; index < definition.pubsub_endpoints.size();
           ++index) {
        if (index > 0) manifest << ",";
        manifest << definition.pubsub_endpoints[index].name;
      }
      manifest << "\n";

      const auto& selected = value.mode == "rpc"
                                 ? definition.rpc_endpoints
                                 : definition.pubsub_endpoints;
      for (const auto& endpoint : selected) {
        const auto kind = kind_names.find(endpoint.kind);
        if (kind == kind_names.end()) {
          throw std::runtime_error("unsupported endpoint kind for port '" +
                                   name + "'");
        }
        manifest << "route " << endpoint.name << " " << kind->second << " "
                 << value.peer << " reliable\n";
      }
    }
  }

  char diagnostic[1024] = {};
  const pcl_status_t status = pcl_transport_routing_load(
      executor_.handle(), compiled_path.string().c_str(), &routing_, diagnostic,
      sizeof(diagnostic));
  std::error_code remove_error;
  std::filesystem::remove(compiled_path, remove_error);
  if (status != PCL_OK) {
    throw std::runtime_error("load port config '" + config_path +
                             "' failed: " + diagnostic);
  }
  std::vector<std::string> peer_ids;
  for (const auto& entry : transports) {
    peer_ids.push_back(entry.first);
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

}  // namespace agra_p3_example
