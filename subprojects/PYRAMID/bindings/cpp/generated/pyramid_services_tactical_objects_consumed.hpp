// Auto-generated service binding header
// Generated from: pyramid.components.tactical_objects.services.provided.consumed.proto by generate_bindings.py
// Namespace: pyramid::components::tactical_objects::services::consumed
//
// Architecture: component logic > service binding (this) > PCL
//
// This header provides:
//   1. Wire-name constants and topic constants
//   2. EntityActions handler base class (ServiceHandler — override Handle*)
//   3. PCL binding functions (subscribe*, publish*, invoke*)
//   4. Content-type support metadata and msgToString utility
#pragma once

#include "pyramid_data_model_types.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_types.h>

#include <string>
#include <vector>

namespace pyramid::components::tactical_objects::services::consumed {

// ---------------------------------------------------------------------------
// Content-type constants and support metadata
// ---------------------------------------------------------------------------

constexpr const char* kJsonContentType = "application/json";
constexpr const char* kFlatBuffersContentType = "application/flatbuffers";

bool supportsContentType(const char* content_type);
std::vector<const char*> supportedContentTypes();

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

using pyramid::domain_model::Ack;
using pyramid::domain_model::Capability;
using pyramid::domain_model::Identifier;
using pyramid::domain_model::ObjectDetail;
using pyramid::domain_model::ObjectEvidenceRequirement;
using pyramid::domain_model::Query;

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
    virtual std::vector<Capability>
    handleReadCapability(const Query& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish / Invoke (typed)
// ---------------------------------------------------------------------------

/// \brief Subscribe to object-evidence publications on kTopicObjectEvidence.
pcl_port_t* subscribeObjectEvidence(pcl_container_t*  container,
                                    pcl_sub_callback_t callback,
                                    void*             user_data = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Publish a typed message on kTopicObjectEvidence.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicObjectEvidence, obtained during on_configure.
pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const ObjectDetail& payload,
                                   const char*        content_type = "application/json");

pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const std::string& payload,
                                   const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicObjectEvidence.
bool encodeObjectEvidence(const ObjectDetail& payload,
                          const char*        content_type,
                          std::string*       out);

/// \brief Decode a PCL message from kTopicObjectEvidence.
bool decodeObjectEvidence(const pcl_msg_t* msg,
                          ObjectDetail* out);

/// \brief Decode a response from object_evidence.read_detail.
bool decodeReadDetailResponse(const pcl_msg_t* msg,
                              std::vector<ObjectDetail>* out);

/// \brief Invoke object_evidence.read_detail (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr,
                              const pcl_endpoint_route_t* route = nullptr,
                              const char*       content_type = "application/json");

/// \brief Invoke object_evidence.read_detail and ignore the async response.
pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              const char*       content_type = "application/json",
                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_solution_evidence.create_requirement.
bool decodeCreateRequirementResponse(const pcl_msg_t* msg,
                                     Identifier* out);

/// \brief Invoke object_solution_evidence.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectEvidenceRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke object_solution_evidence.create_requirement and ignore the async response.
pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectEvidenceRequirement& request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_solution_evidence.read_requirement.
bool decodeReadRequirementResponse(const pcl_msg_t* msg,
                                   std::vector<ObjectEvidenceRequirement>* out);

/// \brief Invoke object_solution_evidence.read_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data = nullptr,
                                   const pcl_endpoint_route_t* route = nullptr,
                                   const char*       content_type = "application/json");

/// \brief Invoke object_solution_evidence.read_requirement and ignore the async response.
pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   const char*       content_type = "application/json",
                                   const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_solution_evidence.update_requirement.
bool decodeUpdateRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out);

/// \brief Invoke object_solution_evidence.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectEvidenceRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke object_solution_evidence.update_requirement and ignore the async response.
pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectEvidenceRequirement& request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_solution_evidence.delete_requirement.
bool decodeDeleteRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out);

/// \brief Invoke object_solution_evidence.delete_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke object_solution_evidence.delete_requirement and ignore the async response.
pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_source_capability.read_capability.
bool decodeReadCapabilityResponse(const pcl_msg_t* msg,
                                  std::vector<Capability>* out);

/// \brief Invoke object_source_capability.read_capability (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadCapability(pcl_executor_t* executor,
                                  const Query&                 request,
                                  pcl_resp_cb_fn_t        callback,
                                  void*                   user_data = nullptr,
                                  const pcl_endpoint_route_t* route = nullptr,
                                  const char*       content_type = "application/json");

/// \brief Invoke object_source_capability.read_capability and ignore the async response.
pcl_status_t invokeReadCapability(pcl_executor_t* executor,
                                  const Query&                 request,
                                  const char*       content_type = "application/json",
                                  const pcl_endpoint_route_t* route = nullptr);

// ---------------------------------------------------------------------------
// Dispatch -- deserialises request, calls handler, serialises response.
//
// Response buffer is heap-allocated via std::malloc; caller frees with std::free.
// ---------------------------------------------------------------------------

void dispatch(ServiceHandler& handler,
              ServiceChannel  channel,
              const void*     request_buf,
              size_t          request_size,
              const char*     content_type,
              void**          response_buf,
              size_t*         response_size);

inline void dispatch(ServiceHandler& handler,
                     ServiceChannel  channel,
                     const void*     request_buf,
                     size_t          request_size,
                     void**          response_buf,
                     size_t*         response_size)
{
    dispatch(handler, channel, request_buf, request_size, "application/json", response_buf, response_size);
}

} // namespace pyramid::components::tactical_objects::services::consumed

// Legacy service namespace alias for existing callers.
namespace pyramid::services::tactical_objects {
namespace consumed = ::pyramid::components::tactical_objects::services::consumed;
} // namespace pyramid::services::tactical_objects
