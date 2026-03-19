// Auto-generated service binding implementation
// Generated from: services by cpp_service_generator
// Namespace: pyramid::services::tactical_objects::provided

#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>

#include <nlohmann/json.hpp>

#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::provided {

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
    return kAckOk;
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return kAckOk;
}

std::vector<ObjectDetail>
ServiceHandler::handleReadDetail(const Query& /*request*/) {
    return {};
}

// ---------------------------------------------------------------------------
// JSON builder: buildStandardRequirementJson
// ---------------------------------------------------------------------------

std::string buildStandardRequirementJson(
    std::string_view policy,
    std::string_view identity,
    std::string_view dimension,
    double min_lat_rad,
    double max_lat_rad,
    double min_lon_rad,
    double max_lon_rad)
{
    nlohmann::json obj;
    obj["policy"]      = std::string(policy);
    obj["identity"]    = std::string(identity);
    if (!dimension.empty()) {
        obj["dimension"] = std::string(dimension);
    }
    obj["min_lat_rad"] = min_lat_rad;
    obj["max_lat_rad"] = max_lat_rad;
    obj["min_lon_rad"] = min_lon_rad;
    obj["max_lon_rad"] = max_lon_rad;
    return obj.dump();
}

// ---------------------------------------------------------------------------
// JSON builder: buildStandardEvidenceJson
// ---------------------------------------------------------------------------

std::string buildStandardEvidenceJson(
    std::string_view identity,
    std::string_view dimension,
    double lat_rad,
    double lon_rad,
    double confidence,
    double observed_at)
{
    nlohmann::json obj;
    obj["identity"]      = std::string(identity);
    obj["dimension"]     = std::string(dimension);
    obj["latitude_rad"]  = lat_rad;
    obj["longitude_rad"] = lon_rad;
    obj["confidence"]    = confidence;
    obj["observed_at"]   = observed_at;
    return obj.dump();
}

// ---------------------------------------------------------------------------
// PCL binding helpers
// ---------------------------------------------------------------------------

namespace {

/// \brief Generic invoke helper — builds a pcl_msg_t and dispatches via
///        pcl_socket_transport_invoke_remote_async.
pcl_status_t invoke_async(pcl_socket_transport_t* transport,
                           const char*             service_name,
                           const std::string&      request,
                           pcl_resp_cb_fn_t        callback,
                           void*                   user_data)
{
    pcl_msg_t msg{};
    msg.data      = request.data();
    msg.size      = static_cast<uint32_t>(request.size());
    msg.type_name = "application/json";
    return pcl_socket_transport_invoke_remote_async(
        transport, service_name, &msg, callback, user_data);
}

} // namespace

// ---------------------------------------------------------------------------
// PCL subscribe wrappers
// ---------------------------------------------------------------------------

void subscribeEntityMatches(pcl_container_t*   container,
                            pcl_sub_callback_t  callback,
                            void*              user_data)
{
    pcl_container_add_subscriber(container,
                                 kTopicEntityMatches,
                                 "application/json",
                                 callback,
                                 user_data);
}

void subscribeEvidenceRequirements(pcl_container_t*   container,
                                   pcl_sub_callback_t  callback,
                                   void*              user_data)
{
    pcl_container_add_subscriber(container,
                                 kTopicEvidenceRequirements,
                                 "application/json",
                                 callback,
                                 user_data);
}

// ---------------------------------------------------------------------------
// PCL invoke wrappers
// ---------------------------------------------------------------------------

pcl_status_t invokeReadMatch(pcl_socket_transport_t* transport,
                             const std::string&      request,
                             pcl_resp_cb_fn_t        callback,
                             void*                   user_data)
{
    return invoke_async(transport, kSvcReadMatch, request, callback, user_data);
}

pcl_status_t invokeCreateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data)
{
    return invoke_async(transport, kSvcCreateRequirement, request,
                        callback, user_data);
}

pcl_status_t invokeReadRequirement(pcl_socket_transport_t* transport,
                                   const std::string&      request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data)
{
    return invoke_async(transport, kSvcReadRequirement, request,
                        callback, user_data);
}

pcl_status_t invokeUpdateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data)
{
    return invoke_async(transport, kSvcUpdateRequirement, request,
                        callback, user_data);
}

pcl_status_t invokeDeleteRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data)
{
    return invoke_async(transport, kSvcDeleteRequirement, request,
                        callback, user_data);
}

pcl_status_t invokeReadDetail(pcl_socket_transport_t* transport,
                              const std::string&      request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data)
{
    return invoke_async(transport, kSvcReadDetail, request, callback, user_data);
}

// ---------------------------------------------------------------------------
// Dispatch — routes raw buffer to the appropriate handler stub
// ---------------------------------------------------------------------------

void dispatch(ServiceChannel channel,
              const void*    /*request_buf*/,
              size_t         /*request_size*/,
              void**         response_buf,
              size_t*        response_size)
{
    *response_buf  = nullptr;
    *response_size = 0;

    switch (channel) {
        case ServiceChannel::ReadMatch:
            // TODO: deserialise request, call handler, serialise response
            break;
        case ServiceChannel::CreateRequirement:
            // TODO: deserialise request, call handler, serialise response
            break;
        case ServiceChannel::ReadRequirement:
            // TODO: deserialise request, call handler, serialise response
            break;
        case ServiceChannel::UpdateRequirement:
            // TODO: deserialise request, call handler, serialise response
            break;
        case ServiceChannel::DeleteRequirement:
            // TODO: deserialise request, call handler, serialise response
            break;
        case ServiceChannel::ReadDetail:
            // TODO: deserialise request, call handler, serialise response
            break;
    }
}

} // namespace pyramid::services::tactical_objects::provided
