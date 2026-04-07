// Auto-generated service binding implementation
// Generated from: services by cpp_service_generator
// Namespace: pyramid::services::tactical_objects::provided

#include "pyramid_services_tactical_objects_provided.hpp"

#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::provided {

// Bring data model codec functions into scope
using pyramid::data_model::common::toJson;
using pyramid::data_model::common::fromJson;
using pyramid::data_model::tactical::toJson;
using pyramid::data_model::tactical::fromJson;

// ---------------------------------------------------------------------------
// PCL message utility
// ---------------------------------------------------------------------------

std::string msgToString(const void* data, unsigned size) {
    return std::string(static_cast<const char*>(data), size);
}

// ---------------------------------------------------------------------------
// ServiceHandler — default stub implementations
// ---------------------------------------------------------------------------

std::vector<ObjectMatch>
ServiceHandler::handleReadMatch(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateRequirement(const ObjectInterestRequirement& /*request*/) {
    return {};
}

std::vector<ObjectInterestRequirement>
ServiceHandler::handleReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateRequirement(const ObjectInterestRequirement& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
}

std::vector<ObjectDetail>
ServiceHandler::handleReadDetail(const Query& /*request*/) {
    return {};
}

// ---------------------------------------------------------------------------
// Internal PCL helpers
// ---------------------------------------------------------------------------

namespace {

pcl_status_t invoke_async(pcl_executor_t* executor,
                           const char*             service_name,
                           const std::string&      payload,
                           pcl_resp_cb_fn_t        callback,
                           void*                   user_data)
{
    pcl_msg_t msg{};
    msg.data      = payload.data();
    msg.size      = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/json";
    return pcl_executor_invoke_async(
        executor, service_name, &msg, callback, user_data);
}

} // namespace

// ---------------------------------------------------------------------------
// PCL subscribe wrappers
// ---------------------------------------------------------------------------

void subscribeEntityMatches(pcl_container_t*   container,
                            pcl_sub_callback_t  callback,
                            void*              user_data,
                            const char*        content_type)
{
    pcl_container_add_subscriber(container,
                                 kTopicEntityMatches,
                                 content_type,
                                 callback,
                                 user_data);
}

void subscribeEvidenceRequirements(pcl_container_t*   container,
                                   pcl_sub_callback_t  callback,
                                   void*              user_data,
                                   const char*        content_type)
{
    pcl_container_add_subscriber(container,
                                 kTopicEvidenceRequirements,
                                 content_type,
                                 callback,
                                 user_data);
}

// ---------------------------------------------------------------------------
// Typed invoke wrappers — serialise and dispatch via executor transport
// ---------------------------------------------------------------------------

pcl_status_t invokeReadMatch(pcl_executor_t* executor,
                             const Query&                 request,
                             pcl_resp_cb_fn_t        callback,
                             void*                   user_data)
{
    std::string payload = toJson(request);
    return invoke_async(executor, kSvcReadMatch, payload, callback, user_data);
}

pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data)
{
    std::string payload = toJson(request);
    return invoke_async(executor, kSvcCreateRequirement, payload, callback, user_data);
}

pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data)
{
    std::string payload = toJson(request);
    return invoke_async(executor, kSvcReadRequirement, payload, callback, user_data);
}

pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data)
{
    std::string payload = toJson(request);
    return invoke_async(executor, kSvcUpdateRequirement, payload, callback, user_data);
}

pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data)
{
    return invoke_async(executor, kSvcDeleteRequirement, request, callback, user_data);
}

pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data)
{
    std::string payload = toJson(request);
    return invoke_async(executor, kSvcReadDetail, payload, callback, user_data);
}

// ---------------------------------------------------------------------------
// Dispatch — deserialise request, call handler, serialise response
// ---------------------------------------------------------------------------

void dispatch(ServiceHandler& handler,
              ServiceChannel  channel,
              const void*     request_buf,
              size_t          request_size,
              void**          response_buf,
              size_t*         response_size)
{
    std::string req_str(static_cast<const char*>(request_buf), request_size);
    std::string rsp_str;

    switch (channel) {
    case ServiceChannel::ReadMatch: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadMatch(req);
        rsp_str = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_str += ",";
            rsp_str += toJson(rsp[i]);
        }
        rsp_str += "]";
        break;
    }
    case ServiceChannel::CreateRequirement: {
        auto req = fromJson(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
        auto rsp = handler.handleCreateRequirement(req);
        rsp_str = rsp;
        break;
    }
    case ServiceChannel::ReadRequirement: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadRequirement(req);
        rsp_str = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_str += ",";
            rsp_str += toJson(rsp[i]);
        }
        rsp_str += "]";
        break;
    }
    case ServiceChannel::UpdateRequirement: {
        auto req = fromJson(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
        auto rsp = handler.handleUpdateRequirement(req);
        rsp_str = toJson(rsp);
        break;
    }
    case ServiceChannel::DeleteRequirement: {
        auto& req = req_str;
        auto rsp = handler.handleDeleteRequirement(req);
        rsp_str = toJson(rsp);
        break;
    }
    case ServiceChannel::ReadDetail: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDetail(req);
        rsp_str = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_str += ",";
            rsp_str += toJson(rsp[i]);
        }
        rsp_str += "]";
        break;
    }
    }

    if (!rsp_str.empty()) {
        *response_size = rsp_str.size();
        *response_buf = std::malloc(rsp_str.size());
        std::memcpy(*response_buf, rsp_str.data(), rsp_str.size());
    } else {
        *response_buf = nullptr;
        *response_size = 0;
    }
}

} // namespace pyramid::services::tactical_objects::provided
