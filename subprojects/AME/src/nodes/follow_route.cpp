#include "ame/bt_nodes/follow_route.h"

#include <sstream>

namespace ame {

FollowRoute::FollowRoute(const std::string& name,
                         const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList FollowRoute::providedPorts() {
  return {
      BT::InputPort<std::string>("agent"),
      BT::InputPort<std::string>("route", "", "Serialised waypoints"),
      BT::OutputPort<std::string>("progress"),
  };
}

BT::NodeStatus FollowRoute::onStart() {
  auto agent = getInput<std::string>("agent");
  if (!agent) {
    return BT::NodeStatus::FAILURE;
  }

  // Parse route data. The route input may be either:
  //   (a) Raw waypoints: pipe-separated "lat,lon,alt" triples
  //   (b) InvokeService response: semicolon-separated k=v pairs where the
  //       "waypoints" key holds pipe-separated triples
  waypoints_.clear();
  current_wp_ = 0;

  std::string route_str;
  getInput("route", route_str);

  // If the string contains '=' it is a k=v encoded service response —
  // extract the "waypoints" field.
  if (!route_str.empty() && route_str.find('=') != std::string::npos) {
    std::istringstream kv_stream(route_str);
    std::string kv_token;
    while (std::getline(kv_stream, kv_token, ';')) {
      auto eq = kv_token.find('=');
      if (eq != std::string::npos && kv_token.substr(0, eq) == "waypoints") {
        route_str = kv_token.substr(eq + 1);
        break;
      }
    }
    // If "waypoints" key not found, treat as empty
    if (route_str.find('=') != std::string::npos) {
      route_str.clear();
    }
  }

  if (!route_str.empty()) {
    std::istringstream stream(route_str);
    std::string token;
    while (std::getline(stream, token, '|')) {
      if (token.empty()) continue;
      Waypoint wp;
      std::istringstream wp_stream(token);
      std::string component;
      if (std::getline(wp_stream, component, ',')) wp.lat = std::stod(component);
      if (std::getline(wp_stream, component, ',')) wp.lon = std::stod(component);
      if (std::getline(wp_stream, component, ',')) wp.alt = std::stod(component);
      waypoints_.push_back(wp);
    }
  }

  // If no waypoints (or empty route), succeed immediately (stub/test mode)
  if (waypoints_.empty()) {
    setOutput("progress", "0/0");
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("progress", "0/" + std::to_string(waypoints_.size()));
  return onRunning();
}

BT::NodeStatus FollowRoute::onRunning() {
  if (current_wp_ >= waypoints_.size()) {
    return BT::NodeStatus::SUCCESS;
  }

  // In a real implementation this would poll the agent's position and advance.
  // For simulation/test, we advance through all waypoints immediately.
  current_wp_ = waypoints_.size();

  setOutput("progress",
            std::to_string(current_wp_) + "/" +
                std::to_string(waypoints_.size()));
  return BT::NodeStatus::SUCCESS;
}

void FollowRoute::onHalted() {
  waypoints_.clear();
  current_wp_ = 0;
}

} // namespace ame
