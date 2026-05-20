// Auto-generated service binding header
// Generated from: pyramid.components.sensor_data_interpretation.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::sensor_data_interpretation::services::provided
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

namespace pyramid::components::sensor_data_interpretation::services::provided {

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

constexpr const char* kSvcInterpretationRequirementReadCapability     = "interpretation_requirement.read_capability";
constexpr const char* kSvcInterpretationRequirementCreateRequirement  = "interpretation_requirement.create_requirement";
constexpr const char* kSvcInterpretationRequirementReadRequirement    = "interpretation_requirement.read_requirement";
constexpr const char* kSvcInterpretationRequirementUpdateRequirement  = "interpretation_requirement.update_requirement";
constexpr const char* kSvcInterpretationRequirementDeleteRequirement  = "interpretation_requirement.delete_requirement";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    InterpretationRequirementReadCapability,
    InterpretationRequirementCreateRequirement,
    InterpretationRequirementReadRequirement,
    InterpretationRequirementUpdateRequirement,
    InterpretationRequirementDeleteRequirement,
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
using pyramid::domain_model::InterpretationRequirement;
using pyramid::domain_model::Query;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Interpretation_Requirement_Service
    virtual std::vector<Capability>
    handleInterpretationRequirementReadCapability(const Query& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamInterpretationRequirementReadCapability(const Query& request,
                                                  pcl_stream_context_t* stream_context,
                                                  const char* content_type);

    virtual Identifier
    handleInterpretationRequirementCreateRequirement(const InterpretationRequirement& request);

    virtual std::vector<InterpretationRequirement>
    handleInterpretationRequirementReadRequirement(const Query& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamInterpretationRequirementReadRequirement(const Query& request,
                                                   pcl_stream_context_t* stream_context,
                                                   const char* content_type);

    virtual Ack
    handleInterpretationRequirementUpdateRequirement(const InterpretationRequirement& request);

    virtual Ack
    handleInterpretationRequirementDeleteRequirement(const Identifier& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish / Invoke (typed)
// ---------------------------------------------------------------------------

/// \brief Decode a response from interpretation_requirement.read_capability.
bool decodeInterpretationRequirementReadCapabilityResponse(const pcl_msg_t* msg,
                                                           std::vector<Capability>* out);

/// \brief Encode one stream frame for interpretation_requirement.read_capability.
bool encodeInterpretationRequirementReadCapabilityStreamFrame(const Capability& payload,
                                                              const char*        content_type,
                                                              std::string*       out);

/// \brief Decode one stream frame from interpretation_requirement.read_capability.
bool decodeInterpretationRequirementReadCapabilityStreamFrame(const pcl_msg_t* msg,
                                                              Capability* out);

/// \brief Send one typed stream frame for interpretation_requirement.read_capability.
pcl_status_t sendInterpretationRequirementReadCapabilityStreamFrame(pcl_stream_context_t* stream_context,
                                                                    const Capability& payload,
                                                                    const char*        content_type = "application/json");

/// \brief Invoke interpretation_requirement.read_capability (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeInterpretationRequirementReadCapability(pcl_executor_t* executor,
                                                           const Query&                 request,
                                                           pcl_resp_cb_fn_t        callback,
                                                           void*                   user_data = nullptr,
                                                           const pcl_endpoint_route_t* route = nullptr,
                                                           const char*       content_type = "application/json");

/// \brief Invoke interpretation_requirement.read_capability and ignore the async response.
pcl_status_t invokeInterpretationRequirementReadCapability(pcl_executor_t* executor,
                                                           const Query&                 request,
                                                           const char*       content_type = "application/json",
                                                           const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke interpretation_requirement.read_capability as an asynchronous stream.
pcl_status_t invokeInterpretationRequirementReadCapabilityStream(pcl_executor_t* executor,
                                                                 const Query&                 request,
                                                                 pcl_stream_msg_fn_t   callback,
                                                                 void*                   user_data = nullptr,
                                                                 pcl_stream_context_t** out_context = nullptr,
                                                                 const pcl_endpoint_route_t* route = nullptr,
                                                                 const char*       content_type = "application/json");

/// \brief Decode a response from interpretation_requirement.create_requirement.
bool decodeInterpretationRequirementCreateRequirementResponse(const pcl_msg_t* msg,
                                                              Identifier* out);

/// \brief Invoke interpretation_requirement.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeInterpretationRequirementCreateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
                                                              pcl_resp_cb_fn_t        callback,
                                                              void*                   user_data = nullptr,
                                                              const pcl_endpoint_route_t* route = nullptr,
                                                              const char*       content_type = "application/json");

/// \brief Invoke interpretation_requirement.create_requirement and ignore the async response.
pcl_status_t invokeInterpretationRequirementCreateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
                                                              const char*       content_type = "application/json",
                                                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from interpretation_requirement.read_requirement.
bool decodeInterpretationRequirementReadRequirementResponse(const pcl_msg_t* msg,
                                                            std::vector<InterpretationRequirement>* out);

/// \brief Encode one stream frame for interpretation_requirement.read_requirement.
bool encodeInterpretationRequirementReadRequirementStreamFrame(const InterpretationRequirement& payload,
                                                               const char*        content_type,
                                                               std::string*       out);

/// \brief Decode one stream frame from interpretation_requirement.read_requirement.
bool decodeInterpretationRequirementReadRequirementStreamFrame(const pcl_msg_t* msg,
                                                               InterpretationRequirement* out);

/// \brief Send one typed stream frame for interpretation_requirement.read_requirement.
pcl_status_t sendInterpretationRequirementReadRequirementStreamFrame(pcl_stream_context_t* stream_context,
                                                                     const InterpretationRequirement& payload,
                                                                     const char*        content_type = "application/json");

/// \brief Invoke interpretation_requirement.read_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeInterpretationRequirementReadRequirement(pcl_executor_t* executor,
                                                            const Query&                 request,
                                                            pcl_resp_cb_fn_t        callback,
                                                            void*                   user_data = nullptr,
                                                            const pcl_endpoint_route_t* route = nullptr,
                                                            const char*       content_type = "application/json");

/// \brief Invoke interpretation_requirement.read_requirement and ignore the async response.
pcl_status_t invokeInterpretationRequirementReadRequirement(pcl_executor_t* executor,
                                                            const Query&                 request,
                                                            const char*       content_type = "application/json",
                                                            const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke interpretation_requirement.read_requirement as an asynchronous stream.
pcl_status_t invokeInterpretationRequirementReadRequirementStream(pcl_executor_t* executor,
                                                                  const Query&                 request,
                                                                  pcl_stream_msg_fn_t   callback,
                                                                  void*                   user_data = nullptr,
                                                                  pcl_stream_context_t** out_context = nullptr,
                                                                  const pcl_endpoint_route_t* route = nullptr,
                                                                  const char*       content_type = "application/json");

/// \brief Decode a response from interpretation_requirement.update_requirement.
bool decodeInterpretationRequirementUpdateRequirementResponse(const pcl_msg_t* msg,
                                                              Ack* out);

/// \brief Invoke interpretation_requirement.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeInterpretationRequirementUpdateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
                                                              pcl_resp_cb_fn_t        callback,
                                                              void*                   user_data = nullptr,
                                                              const pcl_endpoint_route_t* route = nullptr,
                                                              const char*       content_type = "application/json");

/// \brief Invoke interpretation_requirement.update_requirement and ignore the async response.
pcl_status_t invokeInterpretationRequirementUpdateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
                                                              const char*       content_type = "application/json",
                                                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from interpretation_requirement.delete_requirement.
bool decodeInterpretationRequirementDeleteRequirementResponse(const pcl_msg_t* msg,
                                                              Ack* out);

/// \brief Invoke interpretation_requirement.delete_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeInterpretationRequirementDeleteRequirement(pcl_executor_t* executor,
                                                              const Identifier&            request,
                                                              pcl_resp_cb_fn_t        callback,
                                                              void*                   user_data = nullptr,
                                                              const pcl_endpoint_route_t* route = nullptr,
                                                              const char*       content_type = "application/json");

/// \brief Invoke interpretation_requirement.delete_requirement and ignore the async response.
pcl_status_t invokeInterpretationRequirementDeleteRequirement(pcl_executor_t* executor,
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

/// \brief Dispatch a server-streaming service request.
pcl_status_t dispatchStream(ServiceHandler& handler,
                            ServiceChannel  channel,
                            const void*     request_buf,
                            size_t          request_size,
                            const char*     content_type,
                            pcl_stream_context_t* stream_context);

inline pcl_status_t dispatchStream(ServiceHandler& handler,
                                   ServiceChannel  channel,
                                   const void*     request_buf,
                                   size_t          request_size,
                                   pcl_stream_context_t* stream_context)
{
    return dispatchStream(handler, channel, request_buf, request_size, "application/json", stream_context);
}

} // namespace pyramid::components::sensor_data_interpretation::services::provided

// Legacy service namespace alias for existing callers.
namespace pyramid::services::sensor_data_interpretation {
namespace provided = ::pyramid::components::sensor_data_interpretation::services::provided;
} // namespace pyramid::services::sensor_data_interpretation
