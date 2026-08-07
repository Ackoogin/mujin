#include "ame/world_state_access.h"

#include "ame/world_model.h"

namespace ame {

FactAuthority IWorldStateAccess::factAuthority(const std::string&) {
  return FactAuthority::BELIEVED;
}

LocalWorldStateAccess::LocalWorldStateAccess(WorldModel* world_model)
    : world_model_(world_model) {}

bool LocalWorldStateAccess::getFact(const std::string& key) {
  return world_model_ != nullptr && world_model_->getFact(key);
}

bool LocalWorldStateAccess::setFact(const std::string& key,
                                    bool value,
                                    const std::string& source) {
  if (world_model_ == nullptr) {
    return false;
  }
  world_model_->setFact(key, value, source, FactAuthority::BELIEVED);
  return true;
}

FactAuthority LocalWorldStateAccess::factAuthority(const std::string& key) {
  if (world_model_ == nullptr) {
    return FactAuthority::BELIEVED;
  }
  try {
    return world_model_->getFactMetadata(key).authority;
  } catch (const std::exception&) {
    // Unknown fluent. Answer BELIEVED so an action that demands confirmed
    // state refuses to run rather than proceeding on a fact we cannot judge.
    return FactAuthority::BELIEVED;
  }
}

}  // namespace ame
