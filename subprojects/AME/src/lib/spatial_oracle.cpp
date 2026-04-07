#include "ame/spatial_oracle.h"

#include <algorithm>
#include <set>

namespace ame {

void StubSpatialOracle::updateReachability(WorldModel& wm) {
  // Stub: mark all locations reachable from all robots.
  // Iterate grounded fluents looking for "(reachable <robot> <location>)" names
  // and set them all to true with CONFIRMED authority.
  unsigned n = wm.numFluents();
  for (unsigned i = 0; i < n; ++i) {
    const auto& name = wm.fluentName(i);
    if (name.substr(0, 10) == "(reachable") {
      wm.setFact(i, true, "stub_spatial_oracle", FactAuthority::CONFIRMED);
    }
  }
}

void StubSpatialOracle::updateNearest(WorldModel& wm) {
  // Stub: for each robot, find the first alphabetically-sorted location that
  // is reachable and not yet visited (i.e. not (at robot loc) true), then
  // set (nearest robot loc) to true.

  // First, clear all existing nearest facts
  unsigned n = wm.numFluents();
  for (unsigned i = 0; i < n; ++i) {
    const auto& name = wm.fluentName(i);
    if (name.substr(0, 8) == "(nearest") {
      wm.setFact(i, false, "stub_spatial_oracle", FactAuthority::CONFIRMED);
    }
  }

  // Collect robot names and their current locations from (at robot loc) facts
  struct RobotLoc {
    std::string robot;
    std::string location;
  };
  std::set<std::string> robots;
  std::set<std::string> robot_locations; // "robot@location" pairs where at=true

  for (unsigned i = 0; i < n; ++i) {
    const auto& fname = wm.fluentName(i);
    if (fname.substr(0, 3) == "(at" && wm.getFact(i)) {
      // Parse "(at robot location)"
      auto first_space = fname.find(' ', 1);
      auto second_space = fname.find(' ', first_space + 1);
      if (first_space != std::string::npos && second_space != std::string::npos) {
        std::string robot = fname.substr(first_space + 1,
                                         second_space - first_space - 1);
        std::string loc = fname.substr(second_space + 1,
                                       fname.size() - second_space - 2);
        robots.insert(robot);
        robot_locations.insert(robot + "@" + loc);
      }
    }
  }

  // For each robot, find reachable locations sorted alphabetically, pick first
  // that the robot is not currently at
  for (const auto& robot : robots) {
    std::vector<std::string> reachable_locs;

    for (unsigned i = 0; i < n; ++i) {
      const auto& fname = wm.fluentName(i);
      std::string prefix = "(reachable " + robot + " ";
      if (fname.substr(0, prefix.size()) == prefix && wm.getFact(i)) {
        std::string loc = fname.substr(prefix.size(),
                                       fname.size() - prefix.size() - 1);
        // Exclude current location
        if (robot_locations.find(robot + "@" + loc) == robot_locations.end()) {
          reachable_locs.push_back(loc);
        }
      }
    }

    std::sort(reachable_locs.begin(), reachable_locs.end());

    if (!reachable_locs.empty()) {
      std::string fact = "(nearest " + robot + " " + reachable_locs[0] + ")";
      try {
        unsigned idx = wm.fluentIndex(fact);
        wm.setFact(idx, true, "stub_spatial_oracle", FactAuthority::CONFIRMED);
      } catch (...) {
        // Fluent not grounded — skip
      }
    }
  }
}

} // namespace ame
