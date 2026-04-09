// Auto-generated service binding header
// Generated from: provided.proto by generate_bindings.py
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

#include "pyramid_services_tactical_objects_wire_types.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_types.h>

#include <string>
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
namespace wire_types = pyramid::services::tactical_objects::wire_types;

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
// PCL binding functions — Subscribe / Invoke (typed)
// ---------------------------------------------------------------------------

/// \brief Subscribe to entity-match publications on kTopicEntityMatches.
pcl_port_t* subscribeEntityMatches(pcl_container_t*  container,
                                   pcl_sub_callback_t callback,
                                   void*             user_data = nullptr,
                                   const char*       content_type = "application/json");

/// \brief Subscribe to evidence-requirement publications on
///        kTopicEvidenceRequirements.
pcl_port_t* subscribeEvidenceRequirements(pcl_container_t*  container,
                                          pcl_sub_callback_t callback,
                                          void*             user_data = nullptr,
                                          const char*       content_type = "application/json");

/// \brief Invoke matching_objects.read_match (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadMatch(pcl_executor_t* executor,
                             const Query&                 request,
                             pcl_resp_cb_fn_t        callback,
                             void*                   user_data = nullptr,
                             const pcl_endpoint_route_t* route = nullptr,
                             const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const wire_types::CreateRequirementRequest& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.read_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data = nullptr,
                                   const pcl_endpoint_route_t* route = nullptr,
                                   const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const wire_types::CreateRequirementRequest& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.delete_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke specific_object_detail.read_detail (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr,
                              const pcl_endpoint_route_t* route = nullptr,
                              const char*       content_type = "application/json");

// ---------------------------------------------------------------------------
// Dispatch — deserialises request, calls handler, serialises response.
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

} // namespace pyramid::services::tactical_objects::provided
