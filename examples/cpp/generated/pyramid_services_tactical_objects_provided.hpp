// Auto-generated service binding header
// Generated from: services by cpp_service_generator
// Namespace: pyramid::services::tactical_objects::provided
//
// Architecture: component logic > service binding (this) > PCL
//
// This header provides:
//   1. Wire-name constants and topic constants
//   2. EntityActions handler base class (ServiceHandler — override Handle*)
//   3. PCL binding functions (subscribe*, invoke*)
//   4. msgToString utility for PCL message payloads
#pragma once

#include "pyramid_data_model_types.hpp"

#include "pyramid_data_model_base_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>
#include <pcl/pcl_types.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace pyramid::services::tactical_objects::provided {

// ---------------------------------------------------------------------------
// Service wire-name constants (generated from proto)
// ---------------------------------------------------------------------------

constexpr const char* kSvcReadMatch          = "matching_objects.read_match";
constexpr const char* kSvcCreateRequirement  = "object_of_interest.create_requirement";
constexpr const char* kSvcReadRequirement    = "object_of_interest.read_requirement";
constexpr const char* kSvcUpdateRequirement  = "object_of_interest.update_requirement";
constexpr const char* kSvcDeleteRequirement  = "object_of_interest.delete_requirement";
constexpr const char* kSvcReadDetail         = "specific_object_detail.read_detail";

// ---------------------------------------------------------------------------
// Standard topic name constants
// ---------------------------------------------------------------------------

constexpr const char* kTopicEntityMatches         = "standard.entity_matches";
constexpr const char* kTopicEvidenceRequirements  = "standard.evidence_requirements";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    ReadMatch,
    CreateRequirement,
    ReadRequirement,
    UpdateRequirement,
    DeleteRequirement,
    ReadDetail,
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
using pyramid::data_model::Identifier;
using pyramid::data_model::ObjectDetail;
using pyramid::data_model::ObjectInterestRequirement;
using pyramid::data_model::ObjectMatch;
using pyramid::data_model::Query;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Matching_Objects_Service
    virtual std::vector<ObjectMatch>
    handleReadMatch(const Query& request);

    // Object_Of_Interest_Service
    virtual Identifier
    handleCreateRequirement(const ObjectInterestRequirement& request);

    virtual std::vector<ObjectInterestRequirement>
    handleReadRequirement(const Query& request);

    virtual Ack
    handleUpdateRequirement(const ObjectInterestRequirement& request);

    virtual Ack
    handleDeleteRequirement(const Identifier& request);

    // Specific_Object_Detail_Service
    virtual std::vector<ObjectDetail>
    handleReadDetail(const Query& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Invoke wrappers
// ---------------------------------------------------------------------------

/// \brief Subscribe to entity-match publications on kTopicEntityMatches.
void subscribeEntityMatches(pcl_container_t*  container,
                            pcl_sub_callback_t callback,
                            void*             user_data = nullptr,
                            const char*       content_type = "application/json");

/// \brief Subscribe to evidence-requirement publications on
///        kTopicEvidenceRequirements.
void subscribeEvidenceRequirements(pcl_container_t*  container,
                                   pcl_sub_callback_t callback,
                                   void*             user_data = nullptr,
                                   const char*       content_type = "application/json");

/// \brief Asynchronously invoke matching_objects.read_match.
pcl_status_t invokeReadMatch(pcl_socket_transport_t* transport,
                             const std::string&      request,
                             pcl_resp_cb_fn_t        callback,
                             void*                   user_data = nullptr,
                             const char*             content_type = "application/json");

/// \brief Asynchronously invoke object_of_interest.create_requirement.
pcl_status_t invokeCreateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const char*             content_type = "application/json");

/// \brief Asynchronously invoke object_of_interest.read_requirement.
pcl_status_t invokeReadRequirement(pcl_socket_transport_t* transport,
                                   const std::string&      request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data = nullptr,
                                   const char*             content_type = "application/json");

/// \brief Asynchronously invoke object_of_interest.update_requirement.
pcl_status_t invokeUpdateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const char*             content_type = "application/json");

/// \brief Asynchronously invoke object_of_interest.delete_requirement.
pcl_status_t invokeDeleteRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const char*             content_type = "application/json");

/// \brief Asynchronously invoke specific_object_detail.read_detail.
pcl_status_t invokeReadDetail(pcl_socket_transport_t* transport,
                              const std::string&      request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr,
                              const char*             content_type = "application/json");

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

/// \brief Typed invoke for matching_objects.read_match.
template<typename Codec>
pcl_status_t invokeReadMatch(pcl_socket_transport_t* transport,
                             const Query& request,
                             pcl_resp_cb_fn_t callback,
                             void* user_data = nullptr)
{
    std::string payload = Codec::serialize(request);
    return invokeReadMatch(transport, payload, callback, user_data, Codec::content_type());
}

/// \brief Typed invoke for object_of_interest.create_requirement.
template<typename Codec>
pcl_status_t invokeCreateRequirement(pcl_socket_transport_t* transport,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t callback,
                                     void* user_data = nullptr)
{
    std::string payload = Codec::serialize(request);
    return invokeCreateRequirement(transport, payload, callback, user_data, Codec::content_type());
}

/// \brief Typed invoke for object_of_interest.read_requirement.
template<typename Codec>
pcl_status_t invokeReadRequirement(pcl_socket_transport_t* transport,
                                   const Query& request,
                                   pcl_resp_cb_fn_t callback,
                                   void* user_data = nullptr)
{
    std::string payload = Codec::serialize(request);
    return invokeReadRequirement(transport, payload, callback, user_data, Codec::content_type());
}

/// \brief Typed invoke for object_of_interest.update_requirement.
template<typename Codec>
pcl_status_t invokeUpdateRequirement(pcl_socket_transport_t* transport,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t callback,
                                     void* user_data = nullptr)
{
    std::string payload = Codec::serialize(request);
    return invokeUpdateRequirement(transport, payload, callback, user_data, Codec::content_type());
}

/// \brief Typed invoke for object_of_interest.delete_requirement.
template<typename Codec>
pcl_status_t invokeDeleteRequirement(pcl_socket_transport_t* transport,
                                     const Identifier& request,
                                     pcl_resp_cb_fn_t callback,
                                     void* user_data = nullptr)
{
    std::string payload = Codec::serialize(request);
    return invokeDeleteRequirement(transport, payload, callback, user_data, Codec::content_type());
}

/// \brief Typed invoke for specific_object_detail.read_detail.
template<typename Codec>
pcl_status_t invokeReadDetail(pcl_socket_transport_t* transport,
                              const Query& request,
                              pcl_resp_cb_fn_t callback,
                              void* user_data = nullptr)
{
    std::string payload = Codec::serialize(request);
    return invokeReadDetail(transport, payload, callback, user_data, Codec::content_type());
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
    case ServiceChannel::ReadMatch: {
        auto req = Codec::deserialize(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadMatch(req);
        rsp_str = Codec::serializeArray(rsp);
        break;
    }
    case ServiceChannel::CreateRequirement: {
        auto req = Codec::deserialize(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
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
        auto req = Codec::deserialize(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
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
    case ServiceChannel::ReadDetail: {
        auto req = Codec::deserialize(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDetail(req);
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

} // namespace pyramid::services::tactical_objects::provided
