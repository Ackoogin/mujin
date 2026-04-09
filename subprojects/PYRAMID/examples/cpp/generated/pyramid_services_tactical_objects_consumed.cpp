// Auto-generated service binding implementation
// Generated from: consumed.proto by generate_bindings.py
// Namespace: pyramid::services::tactical_objects::consumed

#include "pyramid_services_tactical_objects_consumed.hpp"

#include "pyramid_services_tactical_objects_json_codec.hpp"
#if __has_include("flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp")
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#define PYRAMID_HAVE_SERVICE_FLATBUFFERS 1
#else
#define PYRAMID_HAVE_SERVICE_FLATBUFFERS 0
#endif
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::consumed {

// Bring data model codec functions into scope
using pyramid::data_model::common::toJson;
using pyramid::data_model::common::fromJson;
using pyramid::data_model::tactical::toJson;
using pyramid::data_model::tactical::fromJson;
namespace wire_types = pyramid::services::tactical_objects::wire_types;
namespace json_codec = pyramid::services::tactical_objects::json_codec;
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
#endif

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
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
}

std::vector<Identifier>
ServiceHandler::handleReadCapability(const Query& /*request*/) {
    return {};
}

// ---------------------------------------------------------------------------
// Internal PCL helpers
// ---------------------------------------------------------------------------

namespace {

constexpr const char* kJsonContentType = "application/json";
constexpr const char* kFlatBuffersContentType = "application/flatbuffers";

bool is_json_content_type(const char* content_type)
{
    return !content_type || std::strcmp(content_type, kJsonContentType) == 0;
}

bool is_flatbuffers_content_type(const char* content_type)
{
    return content_type && std::strcmp(content_type, kFlatBuffersContentType) == 0;
}

std::string json_request_body(const void* data, size_t size)
{
    if (!data && size != 0) return {};
    return std::string(static_cast<const char*>(data), size);
}

ObjectEvidenceRequirement to_domain_evidence_requirement(
    const wire_types::EvidenceRequirement& wire_req)
{
    ObjectEvidenceRequirement result{};
    result.base.id = wire_req.id;
    result.policy = wire_req.policy;
    if (wire_req.dimension != pyramid::data_model::BattleDimension::Unspecified)
        result.dimension.push_back(wire_req.dimension);
    if (wire_req.min_lat_rad != 0.0 || wire_req.max_lat_rad != 0.0
        || wire_req.min_lon_rad != 0.0 || wire_req.max_lon_rad != 0.0) {
        pyramid::data_model::common::PolyArea area{};
        area.points.push_back({wire_req.min_lat_rad, wire_req.min_lon_rad});
        area.points.push_back({wire_req.min_lat_rad, wire_req.max_lon_rad});
        area.points.push_back({wire_req.max_lat_rad, wire_req.max_lon_rad});
        area.points.push_back({wire_req.max_lat_rad, wire_req.min_lon_rad});
        result.poly_area = area;
    }
    return result;
}

wire_types::CreateRequirementResponse to_wire_create_requirement_response(
    const Identifier& interest_id)
{
    wire_types::CreateRequirementResponse result{};
    result.interest_id = interest_id;
    return result;
}

} // namespace

// ---------------------------------------------------------------------------
// PCL subscribe wrapper
// ---------------------------------------------------------------------------

pcl_port_t* subscribeObjectEvidence(pcl_container_t*   container,
                                    pcl_sub_callback_t  callback,
                                    void*              user_data,
                                    const char*        content_type)
{
    return pcl_container_add_subscriber(container,
                                 kTopicObjectEvidence,
                                 content_type,
                                 callback,
                                 user_data);
}

// ---------------------------------------------------------------------------
// PCL publish wrapper
// ---------------------------------------------------------------------------

pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const wire_types::ObjectEvidence& payload,
                                   const char*        content_type)
{
    std::string wire_payload = json_codec::toJson(payload);
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        wire_payload = flatbuffers_codec::wrapPayload(wire_payload);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    }
    return publishObjectEvidence(publisher, wire_payload, content_type);
}

pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const std::string& payload,
                                   const char*        content_type)
{
    pcl_msg_t msg{};
    msg.data      = payload.data();
    msg.size      = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_port_publish(publisher, &msg);
}

// ---------------------------------------------------------------------------
// Dispatch — deserialise request, call handler, serialise response
// ---------------------------------------------------------------------------

void dispatch(ServiceHandler& handler,
              ServiceChannel  channel,
              const void*     request_buf,
              size_t          request_size,
              const char*     content_type,
              void**          response_buf,
              size_t*         response_size)
{
    std::string req_str;
    std::string rsp_payload;

    if (is_json_content_type(content_type)) {
        req_str = json_request_body(request_buf, request_size);
        if (request_size != 0 && req_str.empty()) {
            *response_buf = nullptr;
            *response_size = 0;
            return;
        }
    } else if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        try {
            req_str = flatbuffers_codec::unwrapPayload(request_buf, request_size);
        } catch (...) {
            *response_buf = nullptr;
            *response_size = 0;
            return;
        }
#else
        *response_buf = nullptr;
        *response_size = 0;
        return;
#endif
    } else {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    switch (channel) {
    case ServiceChannel::ReadDetail: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDetail(req);
        rsp_payload = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_payload += ",";
            rsp_payload += toJson(rsp[i]);
        }
        rsp_payload += "]";
        break;
    }
    case ServiceChannel::CreateRequirement: {
        auto wire_req = json_codec::evidenceRequirementFromJson(req_str);
        auto req = to_domain_evidence_requirement(wire_req);
        auto rsp = handler.handleCreateRequirement(req);
        {
            auto wire_rsp = to_wire_create_requirement_response(rsp);
            rsp_payload = json_codec::toJson(wire_rsp);
        }
        break;
    }
    case ServiceChannel::ReadRequirement: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadRequirement(req);
        rsp_payload = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_payload += ",";
            rsp_payload += toJson(rsp[i]);
        }
        rsp_payload += "]";
        break;
    }
    case ServiceChannel::UpdateRequirement: {
        auto wire_req = json_codec::evidenceRequirementFromJson(req_str);
        auto req = to_domain_evidence_requirement(wire_req);
        auto rsp = handler.handleUpdateRequirement(req);
        rsp_payload = toJson(rsp);
        break;
    }
    case ServiceChannel::DeleteRequirement: {
        auto& req = req_str;
        auto rsp = handler.handleDeleteRequirement(req);
        rsp_payload = toJson(rsp);
        break;
    }
    case ServiceChannel::ReadCapability: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadCapability(req);
        rsp_payload = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_payload += ",";
            rsp_payload += "\"" + rsp[i] + "\""; 
        }
        rsp_payload += "]";
        break;
    }
    }

    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        rsp_payload = flatbuffers_codec::wrapPayload(rsp_payload);
#else
        *response_buf = nullptr;
        *response_size = 0;
        return;
#endif
    }

    if (!rsp_payload.empty()) {
        *response_size = rsp_payload.size();
        *response_buf = std::malloc(rsp_payload.size());
        std::memcpy(*response_buf, rsp_payload.data(), rsp_payload.size());
    } else {
        *response_buf = nullptr;
        *response_size = 0;
    }
}

} // namespace pyramid::services::tactical_objects::consumed
