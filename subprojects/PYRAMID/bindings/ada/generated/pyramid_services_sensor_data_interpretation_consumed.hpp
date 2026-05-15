// Auto-generated service binding header
// Generated from: pyramid.components.sensor_data_interpretation.services.consumed.proto by generate_bindings.py
// Namespace: pyramid::components::sensor_data_interpretation::services::consumed
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

namespace pyramid::components::sensor_data_interpretation::services::consumed {

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

constexpr const char* kSvcDataProvisionDependencyCreateRequirement   = "data_provision_dependency.create_requirement";
constexpr const char* kSvcDataProvisionDependencyReadRequirement     = "data_provision_dependency.read_requirement";
constexpr const char* kSvcDataProvisionDependencyUpdateRequirement   = "data_provision_dependency.update_requirement";
constexpr const char* kSvcDataProvisionDependencyDeleteRequirement   = "data_provision_dependency.delete_requirement";
constexpr const char* kSvcDataProcessingDependencyCreateRequirement  = "data_processing_dependency.create_requirement";
constexpr const char* kSvcDataProcessingDependencyReadRequirement    = "data_processing_dependency.read_requirement";
constexpr const char* kSvcDataProcessingDependencyUpdateRequirement  = "data_processing_dependency.update_requirement";
constexpr const char* kSvcDataProcessingDependencyDeleteRequirement  = "data_processing_dependency.delete_requirement";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    DataProvisionDependencyCreateRequirement,
    DataProvisionDependencyReadRequirement,
    DataProvisionDependencyUpdateRequirement,
    DataProvisionDependencyDeleteRequirement,
    DataProcessingDependencyCreateRequirement,
    DataProcessingDependencyReadRequirement,
    DataProcessingDependencyUpdateRequirement,
    DataProcessingDependencyDeleteRequirement,
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
using pyramid::domain_model::Identifier;
using pyramid::domain_model::ObjectAquisitionRequirement;
using pyramid::domain_model::ObjectEvidenceProvisionRequirement;
using pyramid::domain_model::Query;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Data_Provision_Dependency_Service
    virtual Identifier
    handleDataProvisionDependencyCreateRequirement(const ObjectEvidenceProvisionRequirement& request);

    virtual std::vector<ObjectEvidenceProvisionRequirement>
    handleDataProvisionDependencyReadRequirement(const Query& request);

    virtual Ack
    handleDataProvisionDependencyUpdateRequirement(const ObjectEvidenceProvisionRequirement& request);

    virtual Ack
    handleDataProvisionDependencyDeleteRequirement(const Identifier& request);

    // Data_Processing_Dependency_Service
    virtual Identifier
    handleDataProcessingDependencyCreateRequirement(const ObjectAquisitionRequirement& request);

    virtual std::vector<ObjectAquisitionRequirement>
    handleDataProcessingDependencyReadRequirement(const Query& request);

    virtual Ack
    handleDataProcessingDependencyUpdateRequirement(const ObjectAquisitionRequirement& request);

    virtual Ack
    handleDataProcessingDependencyDeleteRequirement(const Identifier& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish / Invoke (typed)
// ---------------------------------------------------------------------------

/// \brief Decode a response from data_provision_dependency.create_requirement.
bool decodeDataProvisionDependencyCreateRequirementResponse(const pcl_msg_t* msg,
                                                            Identifier* out);

/// \brief Invoke data_provision_dependency.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProvisionDependencyCreateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
                                                            pcl_resp_cb_fn_t        callback,
                                                            void*                   user_data = nullptr,
                                                            const pcl_endpoint_route_t* route = nullptr,
                                                            const char*       content_type = "application/json");

/// \brief Invoke data_provision_dependency.create_requirement and ignore the async response.
pcl_status_t invokeDataProvisionDependencyCreateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
                                                            const char*       content_type = "application/json",
                                                            const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from data_provision_dependency.read_requirement.
bool decodeDataProvisionDependencyReadRequirementResponse(const pcl_msg_t* msg,
                                                          std::vector<ObjectEvidenceProvisionRequirement>* out);

/// \brief Invoke data_provision_dependency.read_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProvisionDependencyReadRequirement(pcl_executor_t* executor,
                                                          const Query&                 request,
                                                          pcl_resp_cb_fn_t        callback,
                                                          void*                   user_data = nullptr,
                                                          const pcl_endpoint_route_t* route = nullptr,
                                                          const char*       content_type = "application/json");

/// \brief Invoke data_provision_dependency.read_requirement and ignore the async response.
pcl_status_t invokeDataProvisionDependencyReadRequirement(pcl_executor_t* executor,
                                                          const Query&                 request,
                                                          const char*       content_type = "application/json",
                                                          const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from data_provision_dependency.update_requirement.
bool decodeDataProvisionDependencyUpdateRequirementResponse(const pcl_msg_t* msg,
                                                            Ack* out);

/// \brief Invoke data_provision_dependency.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProvisionDependencyUpdateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
                                                            pcl_resp_cb_fn_t        callback,
                                                            void*                   user_data = nullptr,
                                                            const pcl_endpoint_route_t* route = nullptr,
                                                            const char*       content_type = "application/json");

/// \brief Invoke data_provision_dependency.update_requirement and ignore the async response.
pcl_status_t invokeDataProvisionDependencyUpdateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
                                                            const char*       content_type = "application/json",
                                                            const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from data_provision_dependency.delete_requirement.
bool decodeDataProvisionDependencyDeleteRequirementResponse(const pcl_msg_t* msg,
                                                            Ack* out);

/// \brief Invoke data_provision_dependency.delete_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProvisionDependencyDeleteRequirement(pcl_executor_t* executor,
                                                            const Identifier&            request,
                                                            pcl_resp_cb_fn_t        callback,
                                                            void*                   user_data = nullptr,
                                                            const pcl_endpoint_route_t* route = nullptr,
                                                            const char*       content_type = "application/json");

/// \brief Invoke data_provision_dependency.delete_requirement and ignore the async response.
pcl_status_t invokeDataProvisionDependencyDeleteRequirement(pcl_executor_t* executor,
                                                            const Identifier&            request,
                                                            const char*       content_type = "application/json",
                                                            const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from data_processing_dependency.create_requirement.
bool decodeDataProcessingDependencyCreateRequirementResponse(const pcl_msg_t* msg,
                                                             Identifier* out);

/// \brief Invoke data_processing_dependency.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProcessingDependencyCreateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
                                                             pcl_resp_cb_fn_t        callback,
                                                             void*                   user_data = nullptr,
                                                             const pcl_endpoint_route_t* route = nullptr,
                                                             const char*       content_type = "application/json");

/// \brief Invoke data_processing_dependency.create_requirement and ignore the async response.
pcl_status_t invokeDataProcessingDependencyCreateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
                                                             const char*       content_type = "application/json",
                                                             const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from data_processing_dependency.read_requirement.
bool decodeDataProcessingDependencyReadRequirementResponse(const pcl_msg_t* msg,
                                                           std::vector<ObjectAquisitionRequirement>* out);

/// \brief Invoke data_processing_dependency.read_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProcessingDependencyReadRequirement(pcl_executor_t* executor,
                                                           const Query&                 request,
                                                           pcl_resp_cb_fn_t        callback,
                                                           void*                   user_data = nullptr,
                                                           const pcl_endpoint_route_t* route = nullptr,
                                                           const char*       content_type = "application/json");

/// \brief Invoke data_processing_dependency.read_requirement and ignore the async response.
pcl_status_t invokeDataProcessingDependencyReadRequirement(pcl_executor_t* executor,
                                                           const Query&                 request,
                                                           const char*       content_type = "application/json",
                                                           const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from data_processing_dependency.update_requirement.
bool decodeDataProcessingDependencyUpdateRequirementResponse(const pcl_msg_t* msg,
                                                             Ack* out);

/// \brief Invoke data_processing_dependency.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProcessingDependencyUpdateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
                                                             pcl_resp_cb_fn_t        callback,
                                                             void*                   user_data = nullptr,
                                                             const pcl_endpoint_route_t* route = nullptr,
                                                             const char*       content_type = "application/json");

/// \brief Invoke data_processing_dependency.update_requirement and ignore the async response.
pcl_status_t invokeDataProcessingDependencyUpdateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
                                                             const char*       content_type = "application/json",
                                                             const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from data_processing_dependency.delete_requirement.
bool decodeDataProcessingDependencyDeleteRequirementResponse(const pcl_msg_t* msg,
                                                             Ack* out);

/// \brief Invoke data_processing_dependency.delete_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDataProcessingDependencyDeleteRequirement(pcl_executor_t* executor,
                                                             const Identifier&            request,
                                                             pcl_resp_cb_fn_t        callback,
                                                             void*                   user_data = nullptr,
                                                             const pcl_endpoint_route_t* route = nullptr,
                                                             const char*       content_type = "application/json");

/// \brief Invoke data_processing_dependency.delete_requirement and ignore the async response.
pcl_status_t invokeDataProcessingDependencyDeleteRequirement(pcl_executor_t* executor,
                                                             const Identifier&            request,
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

} // namespace pyramid::components::sensor_data_interpretation::services::consumed

// Legacy service namespace alias for existing callers.
namespace pyramid::services::sensor_data_interpretation {
namespace consumed = ::pyramid::components::sensor_data_interpretation::services::consumed;
} // namespace pyramid::services::sensor_data_interpretation
