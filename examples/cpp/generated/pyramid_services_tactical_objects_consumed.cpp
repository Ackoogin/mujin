// Auto-generated service binding implementation
// Generated from: consumed.proto by cpp_service_generator
// Namespace: pyramid::services::tactical_objects::consumed

#include "pyramid_services_tactical_objects_consumed.hpp"

#include <pcl/pcl_container.h>

#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::consumed {

// ---------------------------------------------------------------------------
// PCL message utility
// ---------------------------------------------------------------------------

std::string msgToString(const void* data, unsigned size) {
    return std::string(static_cast<const char*>(data), size);
}

// ---------------------------------------------------------------------------
// ServiceHandler — default stub implementations
// ---------------------------------------------------------------------------

std::vector<ObjectDetail>
ServiceHandler::handleReadDetail(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateRequirement(const ObjectEvidenceRequirement& /*request*/) {
    return {};
}

std::vector<ObjectEvidenceRequirement>
ServiceHandler::handleReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateRequirement(const ObjectEvidenceRequirement& /*request*/) {
    return kAckOk;
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return kAckOk;
}

std::vector<Identifier>
ServiceHandler::handleReadCapability(const Query& /*request*/) {
    return {};
}

// ---------------------------------------------------------------------------
// PCL subscribe wrapper
// ---------------------------------------------------------------------------

void subscribeObjectEvidence(pcl_container_t*   container,
                             pcl_sub_callback_t  callback,
                             void*              user_data)
{
    pcl_container_add_subscriber(container,
                                 kTopicObjectEvidence,
                                 "application/json",
                                 callback,
                                 user_data);
}

// ---------------------------------------------------------------------------
// PCL publish wrapper
// ---------------------------------------------------------------------------

pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const std::string& payload)
{
    pcl_msg_t msg{};
    msg.data      = payload.data();
    msg.size      = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/json";
    return pcl_port_publish(publisher, &msg);
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
        case ServiceChannel::ReadDetail:
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
        case ServiceChannel::ReadCapability:
            // TODO: deserialise request, call handler, serialise response
            break;
    }
}

} // namespace pyramid::services::tactical_objects::consumed
