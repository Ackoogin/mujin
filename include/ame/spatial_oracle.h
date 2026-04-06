#pragma once

#include "ame/world_model.h"

#include <string>
#include <vector>

namespace ame {

/// \brief Abstract interface for a spatial oracle that translates geometric
/// environment state into WorldModel predicates before each planning episode.
///
/// The oracle examines the current map/environment and populates reachability
/// and proximity facts with FactAuthority::CONFIRMED. The planner then uses
/// these facts as preconditions to produce spatially-informed abstract plans.
class ISpatialOracle {
public:
  virtual ~ISpatialOracle() = default;

  /// \brief Compute reachability from the current state. Sets (reachable ...)
  /// facts in wm. Called before each planning episode.
  virtual void updateReachability(WorldModel& wm) = 0;

  /// \brief Compute nearest unvisited target for each agent. Sets (nearest ...)
  /// facts in wm. Called before each planning episode.
  virtual void updateNearest(WorldModel& wm) = 0;
};

/// \brief Stub oracle for testing: all locations reachable from all robots,
/// nearest is the first alphabetically among unvisited locations.
class StubSpatialOracle : public ISpatialOracle {
public:
  void updateReachability(WorldModel& wm) override;
  void updateNearest(WorldModel& wm) override;
};

} // namespace ame
