#include "tobj_service_binding_handler.hpp"

#include <string>
#include <utility>

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

pcl_status_t DemoObjectInterestHandler::streamObjectOfInterestReadRequirement(
    const model::Query& request,
    pcl_stream_context_t* stream_context,
    const char* content_type) {
  ++stream_count_;
  // Send one frame per matching requirement. pcl_stream_send does not free the
  // context so it is safe to call within the stream handler. pcl_stream_end
  // DOES free the context, so it must be deferred until after the handler
  // returns to avoid a use-after-free in the PCL executor.
  const auto results = handleObjectOfInterestReadRequirement(request);
  for (const auto& req : results) {
    svc::sendObjectOfInterestReadRequirementStreamFrame(
        stream_context, req, content_type);
  }
  pending_stream_ctx_ = stream_context;
  pending_stream_content_type_ = content_type ? content_type : svc::kJsonContentType;
  return PCL_STREAMING;
}

model::Ack DemoObjectInterestHandler::handleObjectOfInterestDeleteRequirement(
    const model::Identifier& request) {
  ++delete_count_;
  return model::Ack{requirements_.erase(request) == 1u};
}

void DemoObjectInterestHandler::flushPendingStreamEnd() {
  if (!pending_stream_ctx_) {
    return;
  }
  pcl_stream_context_t* ctx = std::exchange(pending_stream_ctx_, nullptr);
  pending_stream_content_type_.clear();
  pcl_stream_end(ctx);
}

}  // namespace tobj_example
