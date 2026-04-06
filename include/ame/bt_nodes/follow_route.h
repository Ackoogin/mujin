#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>
#include <vector>

namespace ame {

/// \brief BT node that executes a waypoint sequence received from a path
/// planner service. In production this would interface with a flight controller;
/// in test/simulation it completes immediately on the first tick.
///
/// Ports:
///   agent    (input)  — agent identifier (e.g. "uav1")
///   route    (input)  — serialised waypoint data (semicolon-separated
///                        "lat,lon,alt" triples from InvokeService response)
///   progress (output) — "current/total" progress string (e.g. "3/12")
class FollowRoute : public BT::StatefulActionNode {
public:
  FollowRoute(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct Waypoint {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
  };

  std::vector<Waypoint> waypoints_;
  size_t current_wp_ = 0;
};

} // namespace ame
