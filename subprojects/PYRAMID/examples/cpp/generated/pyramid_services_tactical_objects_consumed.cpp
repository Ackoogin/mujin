// Auto-generated service binding implementation
// Generated from: consumed.proto by generate_bindings.py
// Namespace: pyramid::services::tactical_objects::consumed

#include "pyramid_services_tactical_objects_consumed.hpp"

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
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::consumed {

// Bring data model codec functions into scope
using pyramid::data_model::common::toJson;
using pyramid::data_model::common::fromJson;
using pyramid::data_model::tactical::toJson;
using pyramid::data_model::tactical::fromJson;
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

std::vector<Capability>
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

std::string encode_identifier_payload(const Identifier& value)
{
    return nlohmann::json(value).dump();
}

Identifier decode_identifier_payload(const std::string& payload)
{
    if (payload.empty()) {
        return {};
    }
    try {
        auto j = nlohmann::json::parse(payload);
        if (j.is_string()) {
            return j.get<std::string>();
        }
        if (j.is_object() && j.contains("uuid") && j["uuid"].is_string()) {
            return j["uuid"].get<std::string>();
        }
    } catch (...) {
    }
    return payload;
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
                                   const ObjectDetail& payload,
                                   const char*        content_type)
{
    std::string wire_payload;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        wire_payload = flatbuffers_codec::toBinary(payload);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        wire_payload = toJson(payload);
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

    bool rsp_is_binary = false;

    if (is_json_content_type(content_type)) {
        req_str = json_request_body(request_buf, request_size);
        if (request_size != 0 && req_str.empty()) {
            *response_buf = nullptr;
            *response_size = 0;
            return;
        }
    } else if (is_flatbuffers_content_type(content_type)) {
#if !PYRAMID_HAVE_SERVICE_FLATBUFFERS
        *response_buf = nullptr;
        *response_size = 0;
        return;
#endif
    } else {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    try {
    switch (channel) {
    case ServiceChannel::ReadDetail: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryQuery(request_buf, request_size)
            : fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDetail(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        }
        break;
    }
    case ServiceChannel::CreateRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryObjectEvidenceRequirement(request_buf, request_size)
            : fromJson(req_str, static_cast<ObjectEvidenceRequirement*>(nullptr));
        auto rsp = handler.handleCreateRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = encode_identifier_payload(rsp);
        }
        break;
    }
    case ServiceChannel::ReadRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryQuery(request_buf, request_size)
            : fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        }
        break;
    }
    case ServiceChannel::UpdateRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryObjectEvidenceRequirement(request_buf, request_size)
            : fromJson(req_str, static_cast<ObjectEvidenceRequirement*>(nullptr));
        auto rsp = handler.handleUpdateRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::DeleteRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size)
            : decode_identifier_payload(req_str);
        auto rsp = handler.handleDeleteRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::ReadCapability: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryQuery(request_buf, request_size)
            : fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadCapability(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        }
        break;
    }
    }
    } catch (...) {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        (void) rsp_is_binary;
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
