// Auto-generated service binding header
// Generated from: provided.proto by generate_bindings.py
// Namespace: pyramid::services::tactical_objects::provided
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

#include "ros2/cpp/pyramid_ros2_transport_support.hpp"

#include <memory>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::provided {

// ---------------------------------------------------------------------------
// Content-type constants and support metadata
// ---------------------------------------------------------------------------

constexpr const char* kJsonContentType = "application/json";
constexpr const char* kFlatBuffersContentType = "application/flatbuffers";
constexpr const char* kProtobufContentType = "application/protobuf";

bool supportsContentType(const char* content_type);
std::vector<const char*> supportedContentTypes();

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
using pyramid::data_model::ObjectEvidenceRequirement;
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
bool decodeReadMatchResponse(const pcl_msg_t* msg,
                             std::vector<ObjectMatch>* out);

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

/// \brief Invoke matching_objects.read_match and ignore the async response.
pcl_status_t invokeReadMatch(pcl_executor_t* executor,
                             const Query&                 request,
                             const char*       content_type = "application/json",
                             const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_of_interest.create_requirement.
bool decodeCreateRequirementResponse(const pcl_msg_t* msg,
                                     Identifier* out);

/// \brief Invoke object_of_interest.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.create_requirement and ignore the async response.
pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_of_interest.read_requirement.
bool decodeReadRequirementResponse(const pcl_msg_t* msg,
                                   std::vector<ObjectInterestRequirement>* out);

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

/// \brief Invoke object_of_interest.read_requirement and ignore the async response.
pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   const char*       content_type = "application/json",
                                   const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_of_interest.update_requirement.
bool decodeUpdateRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out);

/// \brief Invoke object_of_interest.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke object_of_interest.update_requirement and ignore the async response.
pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from object_of_interest.delete_requirement.
bool decodeDeleteRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out);

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

/// \brief Invoke object_of_interest.delete_requirement and ignore the async response.
pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from specific_object_detail.read_detail.
bool decodeReadDetailResponse(const pcl_msg_t* msg,
                              std::vector<ObjectDetail>* out);

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

/// \brief Invoke specific_object_detail.read_detail and ignore the async response.
pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              const char*       content_type = "application/json",
                              const pcl_endpoint_route_t* route = nullptr);

// ---------------------------------------------------------------------------
// gRPC binding startup hook
// ---------------------------------------------------------------------------

class GrpcServer {
public:
    GrpcServer();
    GrpcServer(GrpcServer&&) noexcept;
    GrpcServer& operator=(GrpcServer&&) noexcept;
    GrpcServer(const GrpcServer&) = delete;
    GrpcServer& operator=(const GrpcServer&) = delete;
    ~GrpcServer();

    bool started() const;
    explicit operator bool() const { return started(); }
    void wait();
    void shutdown();

private:
    struct Impl;
    explicit GrpcServer(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
    friend GrpcServer buildGrpcServer(const std::string& listen_address,
                                      pcl_executor_t* executor);
};

/// \brief Start generated gRPC ingress endpoints on a PCL executor.
GrpcServer buildGrpcServer(const std::string& listen_address,
                           pcl_executor_t* executor);

// ---------------------------------------------------------------------------
// ROS2 binding startup hook
// ---------------------------------------------------------------------------

/// \brief Bind generated ROS2 ingress endpoints to the executor.
inline void bindRos2(pyramid::transport::ros2::Adapter& adapter,
                     pcl_executor_t* executor)
{
    pyramid::transport::ros2::bindTopicIngress(adapter, executor, kTopicEntityMatches);
    pyramid::transport::ros2::bindTopicIngress(adapter, executor, kTopicEvidenceRequirements);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadMatch);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcCreateRequirement);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcUpdateRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcDeleteRequirement);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadDetail);
}

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
