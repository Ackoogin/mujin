// Auto-generated service binding implementation
// Generated from: provided.proto by cpp_service_generator
// Namespace: pyramid::services::tactical_objects::provided

#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>

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
// PCL binding helpers
// ---------------------------------------------------------------------------

namespace {

/// \brief Generic invoke helper — builds a pcl_msg_t and dispatches via
///        pcl_socket_transport_invoke_remote_async.
pcl_status_t invoke_async(pcl_socket_transport_t* transport,
                           const char*             service_name,
                           const std::string&      request,
                           pcl_resp_cb_fn_t        callback,
                           void*                   user_data,
                           const char*             content_type)
{
    pcl_msg_t msg{};
    msg.data      = request.data();
    msg.size      = static_cast<uint32_t>(request.size());
    msg.type_name = content_type;
    return pcl_socket_transport_invoke_remote_async(
        transport, service_name, &msg, callback, user_data);
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
// PCL invoke wrappers
// ---------------------------------------------------------------------------

pcl_status_t invokeReadMatch(pcl_socket_transport_t* transport,
                             const std::string&      request,
                             pcl_resp_cb_fn_t        callback,
                             void*                   user_data,
                             const char*             content_type)
{
    return invoke_async(transport, kSvcReadMatch, request, callback, user_data, content_type);
}

pcl_status_t invokeCreateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const char*             content_type)
{
    return invoke_async(transport, kSvcCreateRequirement, request, callback, user_data, content_type);
}

pcl_status_t invokeReadRequirement(pcl_socket_transport_t* transport,
                                   const std::string&      request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data,
                                   const char*             content_type)
{
    return invoke_async(transport, kSvcReadRequirement, request, callback, user_data, content_type);
}

pcl_status_t invokeUpdateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const char*             content_type)
{
    return invoke_async(transport, kSvcUpdateRequirement, request, callback, user_data, content_type);
}

pcl_status_t invokeDeleteRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const char*             content_type)
{
    return invoke_async(transport, kSvcDeleteRequirement, request, callback, user_data, content_type);
}

pcl_status_t invokeReadDetail(pcl_socket_transport_t* transport,
                              const std::string&      request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data,
                              const char*             content_type)
{
    return invoke_async(transport, kSvcReadDetail, request, callback, user_data, content_type);
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
