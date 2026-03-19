// Auto-generated service binding header
// Generated from: services by cpp_service_generator
// Namespace: pyramid::services::tactical_objects::provided
//
// Architecture: component logic > service binding (this) > PCL
//
// This header provides:
//   1. Wire-name constants and topic constants
//   2. EntityActions handler base class (ServiceHandler — override Handle*)
//   3. JSON builder functions (nlohmann::json / string)
//   4. PCL binding functions (subscribe*, invoke*)
//   5. msgToString utility for PCL message payloads
#pragma once

#include "pyramid_services_tactical_objects_types.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>
#include <pcl/pcl_types.h>

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

using pyramid::services::tactical_objects::Ack;
using pyramid::services::tactical_objects::Identifier;
using pyramid::services::tactical_objects::ObjectDetail;
using pyramid::services::tactical_objects::ObjectInterestRequirement;
using pyramid::services::tactical_objects::ObjectMatch;
using pyramid::services::tactical_objects::Query;

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
// JSON builder functions
// ---------------------------------------------------------------------------

/// \brief Build a JSON object for an interest/evidence requirement.
///
/// Produces: {"policy":..., "identity":..., "dimension":...,
///            "min_lat_rad":..., "max_lat_rad":...,
///            "min_lon_rad":..., "max_lon_rad":...}
std::string buildStandardRequirementJson(
    std::string_view policy,
    std::string_view identity,
    std::string_view dimension  = "",
    double min_lat_rad          = 0.0,
    double max_lat_rad          = 0.0,
    double min_lon_rad          = 0.0,
    double max_lon_rad          = 0.0);

/// \brief Build a JSON object for an observation evidence report.
///
/// Produces: {"identity":..., "dimension":...,
///            "latitude_rad":..., "longitude_rad":...,
///            "confidence":..., "observed_at":...}
std::string buildStandardEvidenceJson(
    std::string_view identity,
    std::string_view dimension,
    double lat_rad,
    double lon_rad,
    double confidence,
    double observed_at = 0.5);

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Invoke wrappers
// ---------------------------------------------------------------------------

/// \brief Subscribe to entity-match publications on kTopicEntityMatches.
void subscribeEntityMatches(pcl_container_t*  container,
                            pcl_sub_callback_t callback,
                            void*             user_data = nullptr);

/// \brief Subscribe to evidence-requirement publications on
///        kTopicEvidenceRequirements.
void subscribeEvidenceRequirements(pcl_container_t*  container,
                                   pcl_sub_callback_t callback,
                                   void*             user_data = nullptr);

/// \brief Asynchronously invoke matching_objects.read_match on a remote peer.
pcl_status_t invokeReadMatch(pcl_socket_transport_t* transport,
                             const std::string&      request,
                             pcl_resp_cb_fn_t        callback,
                             void*                   user_data = nullptr);

/// \brief Asynchronously invoke object_of_interest.create_requirement.
pcl_status_t invokeCreateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr);

/// \brief Asynchronously invoke object_of_interest.read_requirement.
pcl_status_t invokeReadRequirement(pcl_socket_transport_t* transport,
                                   const std::string&      request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data = nullptr);

/// \brief Asynchronously invoke object_of_interest.update_requirement.
pcl_status_t invokeUpdateRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr);

/// \brief Asynchronously invoke object_of_interest.delete_requirement.
pcl_status_t invokeDeleteRequirement(pcl_socket_transport_t* transport,
                                     const std::string&      request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr);

/// \brief Asynchronously invoke specific_object_detail.read_detail.
pcl_status_t invokeReadDetail(pcl_socket_transport_t* transport,
                              const std::string&      request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr);

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

} // namespace pyramid::services::tactical_objects::provided
