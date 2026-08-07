#pragma once

#include "ame/fact_authority.h"

#include <string>

namespace ame {

class WorldModel;

/// \brief Access to the world facts used while a behavior tree is running.
///
/// Planned action nodes obtain this interface from the blackboard entry
/// "world_state". This keeps them independent of whether the world model is in
/// the same process or reached through a transport such as ROS2 services.
class IWorldStateAccess {
public:
  virtual ~IWorldStateAccess() = default;

  /// \brief Return the current value of a grounded fact.
  virtual bool getFact(const std::string& key) = 0;

  /// \brief Record a believed value for a grounded fact.
  /// \return True when the value was recorded.
  virtual bool setFact(const std::string& key,
                       bool value,
                       const std::string& source) = 0;

  /// \brief Return the authority recorded for a grounded fact.
  ///
  /// Only needed by an action that requires confirmed state before it will
  /// run. The default answer is BELIEVED, so an implementation that cannot
  /// report authority refuses such an action rather than letting it proceed on
  /// an unchecked fact.
  virtual FactAuthority factAuthority(const std::string& key);
};

/// \brief In-process world-state access backed by a WorldModel.
class LocalWorldStateAccess : public IWorldStateAccess {
public:
  explicit LocalWorldStateAccess(WorldModel* world_model);

  bool getFact(const std::string& key) override;
  bool setFact(const std::string& key,
               bool value,
               const std::string& source) override;
  FactAuthority factAuthority(const std::string& key) override;

private:
  WorldModel* world_model_ = nullptr;
};

}  // namespace ame
