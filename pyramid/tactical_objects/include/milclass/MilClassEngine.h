#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectStore.h>
#include <tl/optional.hpp>

#include <memory>
#include <string>

namespace tactical_objects {

/// \brief Stores, normalizes, and derives symbol keys from MIL-STD-2525B fields.
class MilClassEngine {
public:
  /// \brief Construct with a shared ObjectStore.
  explicit MilClassEngine(std::shared_ptr<ObjectStore> store);

  /// \brief Store a full classification profile for an entity.
  void setProfile(const UUIDKey& id, const MilClassProfile& profile);

  /// \brief Retrieve the classification profile. Empty if absent.
  tl::optional<MilClassProfile> getProfile(const UUIDKey& id) const;

  /// \brief Derive a deterministic symbol key from stored fields.
  std::string deriveSymbolKey(const UUIDKey& id) const;

private:
  std::shared_ptr<ObjectStore> store_;
};

} // namespace tactical_objects
