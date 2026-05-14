#include "tobj_service_binding_handler.hpp"

#include <string>

namespace tobj_example {

model::Identifier DemoObjectInterestHandler::handleObjectOfInterestCreateRequirement(
    const model::ObjectInterestRequirement& request) {
  // Store the typed requirement exactly as the generated binding decoded it.
  model::ObjectInterestRequirement stored = request;
  if (stored.base.id.empty()) {
    stored.base.id = "interest-" + std::to_string(next_id_++);
  }

  requirements_[stored.base.id] = stored;
  ++create_count_;
  return stored.base.id;
}

std::vector<model::ObjectInterestRequirement>
DemoObjectInterestHandler::handleObjectOfInterestReadRequirement(
    const model::Query& request) {
  ++read_count_;

  // Empty Query.id means "return all" for this tiny example store.
  std::vector<model::ObjectInterestRequirement> result;
  if (request.id.empty()) {
    for (const auto& entry : requirements_) {
      result.push_back(entry.second);
    }
    return result;
  }

  for (const auto& id : request.id) {
    const auto found = requirements_.find(id);
    if (found != requirements_.end()) {
      result.push_back(found->second);
    }
  }
  return result;
}

model::Ack DemoObjectInterestHandler::handleObjectOfInterestDeleteRequirement(
    const model::Identifier& request) {
  ++delete_count_;
  return model::Ack{requirements_.erase(request) == 1u};
}

}  // namespace tobj_example
