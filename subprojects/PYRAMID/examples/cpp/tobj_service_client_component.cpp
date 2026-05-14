#include "tobj_service_client_component.hpp"

#include <chrono>
#include <thread>
#include <utility>

namespace tobj_example {

namespace {
constexpr auto kTimeout = std::chrono::seconds(3);
}

ObjectInterestClientComponent::ObjectInterestClientComponent(
    std::string content_type)
    : pcl::Component("object_interest_client"),
      content_type_(std::move(content_type)) {}

pcl_status_t ObjectInterestClientComponent::on_configure() {
  return svc::supportsContentType(content_type_.c_str()) ? PCL_OK
                                                         : PCL_ERR_INVALID;
}

bool ObjectInterestClientComponent::runCreateReadDelete(
    pcl_executor_t* executor) {
  // Build a typed requirement. No JSON, FlatBuffers, or PCL buffer code is
  // needed here; the generated invoke helper owns serialization.
  model::ObjectInterestRequirement request;
  request.base.id = "interest-demo-001";
  request.base.source = "example-client";
  request.policy = model::DataPolicy::Obtain;
  request.dimension.push_back(model::BattleDimension::Air);
  model::Point point;
  point.position.latitude = 51.477811;
  point.position.longitude = -0.001475;
  request.point = point;

  model::Identifier created_id;
  if (!invokeCreate(executor, request, &created_id) ||
      created_id != request.base.id) {
    return false;
  }

  // Read back only the requirement we created. This proves request/response
  // typing works in both directions over the transport.
  std::vector<model::ObjectInterestRequirement> read_back;
  model::Query query;
  query.id.push_back(created_id);
  if (!invokeRead(executor, query, &read_back) ||
      read_back.size() != 1u ||
      read_back.front().base.id != created_id ||
      read_back.front().policy != model::DataPolicy::Obtain) {
    return false;
  }

  // Delete the requirement to prove the full CRUD-style service path.
  model::Ack deleted;
  return invokeDelete(executor, created_id, &deleted) && deleted.success;
}

bool ObjectInterestClientComponent::waitForResponse(pcl_executor_t* executor,
                                                    ResponseState& state) {
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (!state.done.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(executor, 10);
    std::this_thread::yield();
  }
  return state.done.load(std::memory_order_acquire) && state.decoded;
}

bool ObjectInterestClientComponent::invokeCreate(
    pcl_executor_t* executor,
    const model::ObjectInterestRequirement& request,
    model::Identifier* out) {
  ResponseState state;
  const pcl_status_t rc = svc::invokeObjectOfInterestCreateRequirement(
      executor,
      request,
      [](const pcl_msg_t* msg, void* user_data) {
        auto* response = static_cast<ResponseState*>(user_data);
        response->decoded =
            svc::decodeObjectOfInterestCreateRequirementResponse(
                msg,
                &response->id);
        response->done.store(true, std::memory_order_release);
      },
      &state,
      nullptr,
      content_type_.c_str());

  if (rc != PCL_OK || !waitForResponse(executor, state)) {
    return false;
  }
  *out = state.id;
  return true;
}

bool ObjectInterestClientComponent::invokeRead(
    pcl_executor_t* executor,
    const model::Query& request,
    std::vector<model::ObjectInterestRequirement>* out) {
  ResponseState state;
  const pcl_status_t rc = svc::invokeObjectOfInterestReadRequirement(
      executor,
      request,
      [](const pcl_msg_t* msg, void* user_data) {
        auto* response = static_cast<ResponseState*>(user_data);
        response->decoded =
            svc::decodeObjectOfInterestReadRequirementResponse(
                msg,
                &response->requirements);
        response->done.store(true, std::memory_order_release);
      },
      &state,
      nullptr,
      content_type_.c_str());

  if (rc != PCL_OK || !waitForResponse(executor, state)) {
    return false;
  }
  *out = state.requirements;
  return true;
}

bool ObjectInterestClientComponent::invokeDelete(
    pcl_executor_t* executor,
    const model::Identifier& request,
    model::Ack* out) {
  ResponseState state;
  const pcl_status_t rc = svc::invokeObjectOfInterestDeleteRequirement(
      executor,
      request,
      [](const pcl_msg_t* msg, void* user_data) {
        auto* response = static_cast<ResponseState*>(user_data);
        response->decoded =
            svc::decodeObjectOfInterestDeleteRequirementResponse(
                msg,
                &response->ack);
        response->done.store(true, std::memory_order_release);
      },
      &state,
      nullptr,
      content_type_.c_str());

  if (rc != PCL_OK || !waitForResponse(executor, state)) {
    return false;
  }
  *out = state.ack;
  return true;
}

}  // namespace tobj_example
