#include "ame/world_state_access.h"

#include "ame/world_model.h"

namespace ame {

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

}  // namespace ame
