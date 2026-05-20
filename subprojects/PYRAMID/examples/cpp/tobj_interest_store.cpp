#include "tobj_interest_store.hpp"

#include <string>
#include <utility>

namespace tobj_example {

model::Identifier InterestStore::onObjectOfInterestCreateRequirement(
    const model::ObjectInterestRequirement& request) {
  auto stored = request;
  if (stored.base.id.empty()) {
    stored.base.id = "interest-" + std::to_string(next_id_++);
  }
  store_[stored.base.id] = stored;
  ++creates_;
  return stored.base.id;
}

void InterestStore::onObjectOfInterestReadRequirement(
    const model::Query& query,
    svc::StreamWriter<model::ObjectInterestRequirement> writer) {
  ++reads_;
  // Emit matching entries as they are visited. A real implementation could
  // capture the writer and emit frames lazily across multiple ticks; here we
  // pump synchronously to keep the showcase compact.
  if (query.id.empty()) {
    for (const auto& [_, v] : store_) {
      if (writer.cancelled()) break;  // client gave up; stop emitting
      writer.send(v);
    }
  } else {
    for (const auto& id : query.id) {
      if (writer.cancelled()) break;
      if (auto it = store_.find(id); it != store_.end()) {
        writer.send(it->second);
      }
    }
  }
  writer.end();
}

model::Ack InterestStore::onObjectOfInterestDeleteRequirement(
    const model::Identifier& id) {
  ++deletes_;
  return model::Ack{store_.erase(id) == 1u};
}

}  // namespace tobj_example
