// Auto-generated service binding header
// Generated from: services by cpp_service_generator
// Namespace: pyramid::services::tactical_objects::consumed
//
// Architecture: component logic > service binding (this) > PCL
//
// This header provides:
//   1. Wire-name constants and topic constants
//   2. EntityActions handler base class (ServiceHandler — override Handle*)
//   3. PCL binding functions (subscribe*, publish*)
//   4. msgToString utility for PCL message payloads
#pragma once

#include "pyramid_data_model_types.hpp"

#include "pyramid_data_model_base_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_socket.h>
#include <pcl/pcl_types.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::consumed {

// ---------------------------------------------------------------------------
// Service wire-name constants (generated from proto)
// ---------------------------------------------------------------------------

constexpr const char* kSvcReadDetail         = "object_evidence.read_detail";
constexpr const char* kSvcCreateRequirement  = "object_solution_evidence.create_requirement";
constexpr const char* kSvcReadRequirement    = "object_solution_evidence.read_requirement";
constexpr const char* kSvcUpdateRequirement  = "object_solution_evidence.update_requirement";
constexpr const char* kSvcDeleteRequirement  = "object_solution_evidence.delete_requirement";
constexpr const char* kSvcReadCapability     = "object_source_capability.read_capability";

// ---------------------------------------------------------------------------
// Standard topic name constants
// ---------------------------------------------------------------------------

constexpr const char* kTopicObjectEvidence = "standard.object_evidence";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    ReadDetail,
    CreateRequirement,
    ReadRequirement,
    UpdateRequirement,
    DeleteRequirement,
    ReadCapability,
};

// ---------------------------------------------------------------------------
// PCL message utility
// ---------------------------------------------------------------------------

/// \brief Convert a raw PCL message buffer to a std::string.
std::string msgToString(const void* data, unsigned size);

// ---------------------------------------------------------------------------
// EntityActions handler base class
//
// Subclass and override the handle* methods to implement business logic.
// Default implementations return empty / null values (stub behaviour).
// ---------------------------------------------------------------------------

using pyramid::data_model::Ack;
using pyramid::data_model::Capability;
using pyramid::data_model::Identifier;
using pyramid::data_model::ObjectDetail;
using pyramid::data_model::ObjectEvidenceRequirement;
using pyramid::data_model::Query;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Object_Evidence_Service
    virtual std::vector<ObjectDetail>
    handleReadDetail(const Query& request);

    // Object_Solution_Evidence_Service
    virtual Identifier
    handleCreateRequirement(const ObjectEvidenceRequirement& request);

    virtual std::vector<ObjectEvidenceRequirement>
    handleReadRequirement(const Query& request);

    virtual Ack
    handleUpdateRequirement(const ObjectEvidenceRequirement& request);

    virtual Ack
    handleDeleteRequirement(const Identifier& request);

    // Object_Source_Capability_Service
    virtual std::vector<Identifier>
    handleReadCapability(const Query& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish wrappers
// ---------------------------------------------------------------------------

/// \brief Subscribe to object-evidence publications on kTopicObjectEvidence.
void subscribeObjectEvidence(pcl_container_t*  container,
                             pcl_sub_callback_t callback,
                             void*             user_data = nullptr,
                             const char*       content_type = "application/json");

/// \brief Publish an object-evidence payload on kTopicObjectEvidence.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicObjectEvidence, obtained during on_configure.
pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const std::string& payload,
                                   const char*        content_type = "application/json");

// ---------------------------------------------------------------------------
// Transport dispatch point
//
// Routes a raw request buffer to the appropriate handler.
// Response buffer is heap-allocated; caller is responsible for freeing it.
// ---------------------------------------------------------------------------

void dispatch(ServiceChannel channel,
              const void*    request_buf,
              size_t         request_size,
              void**         response_buf,
              size_t*        response_size);

// ---------------------------------------------------------------------------
// Typed service interface (codec-injected)
//
// Template parameter Codec must provide:
//   static constexpr const char* content_type();
//   static std::string serialize(const T& msg);
//   static T deserialize(const std::string& data, T* tag);
//   static std::string serializeArray(const std::vector<T>& msgs);
// ---------------------------------------------------------------------------

/// Default JSON codec adapter — wraps the generated data
/// model toJson/fromJson functions into the Codec concept.
struct DataModelJsonCodec {
    static constexpr const char* content_type() { return "application/json"; }

    template<typename T>
    static std::string serialize(const T& msg) {
        using pyramid::data_model::common::toJson;
        using pyramid::data_model::tactical::toJson;
        return toJson(msg);
    }

    static std::string serialize(const std::string& msg) { return msg; }

    template<typename T>
    static T deserialize(const std::string& data, T* tag) {
        using pyramid::data_model::common::fromJson;
        using pyramid::data_model::tactical::fromJson;
        return fromJson(data, tag);
    }

    static std::string deserialize(const std::string& data, std::string*) { return data; }

    template<typename T>
    static std::string serializeArray(const std::vector<T>& msgs) {
        std::string result = "[";
        for (size_t i = 0; i < msgs.size(); ++i) {
            if (i > 0) result += ",";
            result += serialize(msgs[i]);
        }
        result += "]";
        return result;
    }
};

/// \brief Typed publish for object_evidence.
template<typename Codec, typename Msg>
pcl_status_t publishObjectEvidence(pcl_port_t* publisher,
                                   const Msg& msg)
{
    std::string payload = Codec::serialize(msg);
    return publishObjectEvidence(publisher, payload, Codec::content_type());
}

// ---------------------------------------------------------------------------
// Typed dispatch — deserialises with Codec, calls handler, serialises response.
//
// Response buffer is heap-allocated via std::malloc; caller frees with std::free.
// ---------------------------------------------------------------------------

template<typename Codec>
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
    case ServiceChannel::ReadDetail: {
        auto req = Codec::deserialize(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDetail(req);
        rsp_str = Codec::serializeArray(rsp);
        break;
    }
    case ServiceChannel::CreateRequirement: {
        auto req = Codec::deserialize(req_str, static_cast<ObjectEvidenceRequirement*>(nullptr));
        auto rsp = handler.handleCreateRequirement(req);
        rsp_str = Codec::serialize(rsp);
        break;
    }
    case ServiceChannel::ReadRequirement: {
        auto req = Codec::deserialize(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadRequirement(req);
        rsp_str = Codec::serializeArray(rsp);
        break;
    }
    case ServiceChannel::UpdateRequirement: {
        auto req = Codec::deserialize(req_str, static_cast<ObjectEvidenceRequirement*>(nullptr));
        auto rsp = handler.handleUpdateRequirement(req);
        rsp_str = Codec::serialize(rsp);
        break;
    }
    case ServiceChannel::DeleteRequirement: {
        auto req = Codec::deserialize(req_str, static_cast<Identifier*>(nullptr));
        auto rsp = handler.handleDeleteRequirement(req);
        rsp_str = Codec::serialize(rsp);
        break;
    }
    case ServiceChannel::ReadCapability: {
        auto req = Codec::deserialize(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadCapability(req);
        rsp_str = Codec::serializeArray(rsp);
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

} // namespace pyramid::services::tactical_objects::consumed
