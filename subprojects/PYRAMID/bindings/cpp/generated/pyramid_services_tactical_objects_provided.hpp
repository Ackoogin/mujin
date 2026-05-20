// Auto-generated service binding header
// Generated from: pyramid.components.tactical_objects.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::tactical_objects::services::provided
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

namespace pyramid::components::tactical_objects::services::provided {

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

constexpr const char* kSvcMatchingObjectsReadMatch           = "matching_objects.read_match";
constexpr const char* kSvcObjectOfInterestCreateRequirement  = "object_of_interest.create_requirement";
constexpr const char* kSvcObjectOfInterestReadRequirement    = "object_of_interest.read_requirement";
constexpr const char* kSvcObjectOfInterestUpdateRequirement  = "object_of_interest.update_requirement";
constexpr const char* kSvcObjectOfInterestDeleteRequirement  = "object_of_interest.delete_requirement";
constexpr const char* kSvcSpecificObjectDetailReadDetail     = "specific_object_detail.read_detail";

// ---------------------------------------------------------------------------
// Standard topic name constants
// ---------------------------------------------------------------------------

constexpr const char* kTopicEntityMatches         = "standard.entity_matches";
constexpr const char* kTopicEvidenceRequirements  = "standard.evidence_requirements";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    MatchingObjectsReadMatch,
    ObjectOfInterestCreateRequirement,
    ObjectOfInterestReadRequirement,
    ObjectOfInterestUpdateRequirement,
    ObjectOfInterestDeleteRequirement,
    SpecificObjectDetailReadDetail,
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
using pyramid::domain_model::ObjectDetail;
using pyramid::domain_model::ObjectEvidenceRequirement;
using pyramid::domain_model::ObjectInterestRequirement;
using pyramid::domain_model::ObjectMatch;
using pyramid::domain_model::Query;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Matching_Objects_Service
    virtual std::vector<ObjectMatch>
    handleMatchingObjectsReadMatch(const Query& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMatchingObjectsReadMatch(const Query& request,
                                   pcl_stream_context_t* stream_context,
                                   const char* content_type);

    // Object_Of_Interest_Service
    virtual Identifier
    handleObjectOfInterestCreateRequirement(const ObjectInterestRequirement& request);

    virtual std::vector<ObjectInterestRequirement>
    handleObjectOfInterestReadRequirement(const Query& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamObjectOfInterestReadRequirement(const Query& request,
                                          pcl_stream_context_t* stream_context,
                                          const char* content_type);

    virtual Ack
    handleObjectOfInterestUpdateRequirement(const ObjectInterestRequirement& request);

    virtual Ack
    handleObjectOfInterestDeleteRequirement(const Identifier& request);

    // Specific_Object_Detail_Service
    virtual std::vector<ObjectDetail>
    handleSpecificObjectDetailReadDetail(const Query& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamSpecificObjectDetailReadDetail(const Query& request,
                                         pcl_stream_context_t* stream_context,
                                         const char* content_type);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish / Invoke (typed)
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

/// \brief Publish a typed message on kTopicEntityMatches.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicEntityMatches, obtained during on_configure.
pcl_status_t publishEntityMatches(pcl_port_t*        publisher,
                                  const std::vector<ObjectMatch>& payload,
                                  const char*        content_type = "application/json");

pcl_status_t publishEntityMatches(pcl_port_t*        publisher,
                                  const std::string& payload,
                                  const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicEntityMatches.
bool encodeEntityMatches(const std::vector<ObjectMatch>& payload,
                         const char*        content_type,
                         std::string*       out);

/// \brief Decode a PCL message from kTopicEntityMatches.
bool decodeEntityMatches(const pcl_msg_t* msg,
                         std::vector<ObjectMatch>* out);

/// \brief Publish a typed message on kTopicEvidenceRequirements.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicEvidenceRequirements, obtained during on_configure.
pcl_status_t publishEvidenceRequirements(pcl_port_t*        publisher,
                                         const ObjectEvidenceRequirement& payload,
                                         const char*        content_type = "application/json");

pcl_status_t publishEvidenceRequirements(pcl_port_t*        publisher,
                                         const std::string& payload,
                                         const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicEvidenceRequirements.
bool encodeEvidenceRequirements(const ObjectEvidenceRequirement& payload,
                                const char*        content_type,
                                std::string*       out);

/// \brief Decode a PCL message from kTopicEvidenceRequirements.
bool decodeEvidenceRequirements(const pcl_msg_t* msg,
                                ObjectEvidenceRequirement* out);

/// \brief Decode a response from matching_objects.read_match.
bool decodeMatchingObjectsReadMatchResponse(const pcl_msg_t* msg,
                                            std::vector<ObjectMatch>* out);

/// \brief Encode one stream frame for matching_objects.read_match.
bool encodeMatchingObjectsReadMatchStreamFrame(const ObjectMatch& payload,
                                               const char*        content_type,
                                               std::string*       out);

/// \brief Decode one stream frame from matching_objects.read_match.
bool decodeMatchingObjectsReadMatchStreamFrame(const pcl_msg_t* msg,
                                               ObjectMatch* out);

/// \brief Send one typed stream frame for matching_objects.read_match.
pcl_status_t sendMatchingObjectsReadMatchStreamFrame(pcl_stream_context_t* stream_context,
                                                     const ObjectMatch& payload,
                                                     const char*        content_type = "application/json");

/// \brief Invoke matching_objects.read_match (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMatchingObjectsReadMatch(pcl_executor_t* executor,
                                            const Query&                 request,
                                            pcl_resp_cb_fn_t        callback,
                                            void*                   user_data = nullptr,
                                            const pcl_endpoint_route_t* route = nullptr,
                                            const char*       content_type = "application/json");

/// \brief Invoke matching_objects.read_match and ignore the async response.
pcl_status_t invokeMatchingObjectsReadMatch(pcl_executor_t* executor,
                                            const Query&                 request,
                                            const char*       content_type = "application/json",
                                            const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke matching_objects.read_match as an asynchronous stream.
pcl_status_t invokeMatchingObjectsReadMatchStream(pcl_executor_t* executor,
                                                  const Query&                 request,
                                                  pcl_stream_msg_fn_t   callback,
                                                  void*                   user_data = nullptr,
                                                  pcl_stream_context_t** out_context = nullptr,
                                                  const pcl_endpoint_route_t* route = nullptr,
                                                  const char*       content_type = "application/json");

/// \brief Decode a response from object_of_interest.create_requirement.
bool decodeObjectOfInterestCreateRequirementResponse(const pcl_msg_t* msg,
                                                     Identifier* out);

/// \brief Invoke object_of_interest.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeObjectOfInterestCreateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data = nullptr,
                                                     const pcl_endpoint_route_t* route = nullptr,
                                                     const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.create_requirement and ignore the async response.
pcl_status_t invokeObjectOfInterestCreateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     const char*       content_type = "application/json",
                                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_of_interest.read_requirement.
bool decodeObjectOfInterestReadRequirementResponse(const pcl_msg_t* msg,
                                                   std::vector<ObjectInterestRequirement>* out);

/// \brief Encode one stream frame for object_of_interest.read_requirement.
bool encodeObjectOfInterestReadRequirementStreamFrame(const ObjectInterestRequirement& payload,
                                                      const char*        content_type,
                                                      std::string*       out);

/// \brief Decode one stream frame from object_of_interest.read_requirement.
bool decodeObjectOfInterestReadRequirementStreamFrame(const pcl_msg_t* msg,
                                                      ObjectInterestRequirement* out);

/// \brief Send one typed stream frame for object_of_interest.read_requirement.
pcl_status_t sendObjectOfInterestReadRequirementStreamFrame(pcl_stream_context_t* stream_context,
                                                            const ObjectInterestRequirement& payload,
                                                            const char*        content_type = "application/json");

/// \brief Invoke object_of_interest.read_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeObjectOfInterestReadRequirement(pcl_executor_t* executor,
                                                   const Query&                 request,
                                                   pcl_resp_cb_fn_t        callback,
                                                   void*                   user_data = nullptr,
                                                   const pcl_endpoint_route_t* route = nullptr,
                                                   const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.read_requirement and ignore the async response.
pcl_status_t invokeObjectOfInterestReadRequirement(pcl_executor_t* executor,
                                                   const Query&                 request,
                                                   const char*       content_type = "application/json",
                                                   const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke object_of_interest.read_requirement as an asynchronous stream.
pcl_status_t invokeObjectOfInterestReadRequirementStream(pcl_executor_t* executor,
                                                         const Query&                 request,
                                                         pcl_stream_msg_fn_t   callback,
                                                         void*                   user_data = nullptr,
                                                         pcl_stream_context_t** out_context = nullptr,
                                                         const pcl_endpoint_route_t* route = nullptr,
                                                         const char*       content_type = "application/json");

/// \brief Decode a response from object_of_interest.update_requirement.
bool decodeObjectOfInterestUpdateRequirementResponse(const pcl_msg_t* msg,
                                                     Ack* out);

/// \brief Invoke object_of_interest.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeObjectOfInterestUpdateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data = nullptr,
                                                     const pcl_endpoint_route_t* route = nullptr,
                                                     const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.update_requirement and ignore the async response.
pcl_status_t invokeObjectOfInterestUpdateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     const char*       content_type = "application/json",
                                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_of_interest.delete_requirement.
bool decodeObjectOfInterestDeleteRequirementResponse(const pcl_msg_t* msg,
                                                     Ack* out);

/// \brief Invoke object_of_interest.delete_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeObjectOfInterestDeleteRequirement(pcl_executor_t* executor,
                                                     const Identifier&            request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data = nullptr,
                                                     const pcl_endpoint_route_t* route = nullptr,
                                                     const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.delete_requirement and ignore the async response.
pcl_status_t invokeObjectOfInterestDeleteRequirement(pcl_executor_t* executor,
                                                     const Identifier&            request,
                                                     const char*       content_type = "application/json",
                                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from specific_object_detail.read_detail.
bool decodeSpecificObjectDetailReadDetailResponse(const pcl_msg_t* msg,
                                                  std::vector<ObjectDetail>* out);

/// \brief Encode one stream frame for specific_object_detail.read_detail.
bool encodeSpecificObjectDetailReadDetailStreamFrame(const ObjectDetail& payload,
                                                     const char*        content_type,
                                                     std::string*       out);

/// \brief Decode one stream frame from specific_object_detail.read_detail.
bool decodeSpecificObjectDetailReadDetailStreamFrame(const pcl_msg_t* msg,
                                                     ObjectDetail* out);

/// \brief Send one typed stream frame for specific_object_detail.read_detail.
pcl_status_t sendSpecificObjectDetailReadDetailStreamFrame(pcl_stream_context_t* stream_context,
                                                           const ObjectDetail& payload,
                                                           const char*        content_type = "application/json");

/// \brief Invoke specific_object_detail.read_detail (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeSpecificObjectDetailReadDetail(pcl_executor_t* executor,
                                                  const Query&                 request,
                                                  pcl_resp_cb_fn_t        callback,
                                                  void*                   user_data = nullptr,
                                                  const pcl_endpoint_route_t* route = nullptr,
                                                  const char*       content_type = "application/json");

/// \brief Invoke specific_object_detail.read_detail and ignore the async response.
pcl_status_t invokeSpecificObjectDetailReadDetail(pcl_executor_t* executor,
                                                  const Query&                 request,
                                                  const char*       content_type = "application/json",
                                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke specific_object_detail.read_detail as an asynchronous stream.
pcl_status_t invokeSpecificObjectDetailReadDetailStream(pcl_executor_t* executor,
                                                        const Query&                 request,
                                                        pcl_stream_msg_fn_t   callback,
                                                        void*                   user_data = nullptr,
                                                        pcl_stream_context_t** out_context = nullptr,
                                                        const pcl_endpoint_route_t* route = nullptr,
                                                        const char*       content_type = "application/json");

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

} // namespace pyramid::components::tactical_objects::services::provided

// Legacy service namespace alias for existing callers.
namespace pyramid::services::tactical_objects {
namespace provided = ::pyramid::components::tactical_objects::services::provided;
} // namespace pyramid::services::tactical_objects
