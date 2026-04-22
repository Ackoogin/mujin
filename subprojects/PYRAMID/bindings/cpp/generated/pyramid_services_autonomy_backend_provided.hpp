// Auto-generated service binding header
// Generated from: provided.proto by generate_bindings.py
// Namespace: pyramid::services::autonomy_backend::provided
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

namespace pyramid::services::autonomy_backend::provided {

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

constexpr const char* kSvcReadCapabilities   = "capabilities.read_capabilities";
constexpr const char* kSvcCreateRequirement  = "planning_execution.create_requirement";
constexpr const char* kSvcReadRequirement    = "planning_execution.read_requirement";
constexpr const char* kSvcUpdateRequirement  = "planning_execution.update_requirement";
constexpr const char* kSvcDeleteRequirement  = "planning_execution.delete_requirement";
constexpr const char* kSvcCreateState        = "state.create_state";
constexpr const char* kSvcUpdateState        = "state.update_state";
constexpr const char* kSvcDeleteState        = "state.delete_state";
constexpr const char* kSvcReadPlan           = "plan.read_plan";
constexpr const char* kSvcReadRun            = "execution_run.read_run";
constexpr const char* kSvcReadPlacement      = "requirement_placement.read_placement";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    ReadCapabilities,
    CreateRequirement,
    ReadRequirement,
    UpdateRequirement,
    DeleteRequirement,
    CreateState,
    UpdateState,
    DeleteState,
    ReadPlan,
    ReadRun,
    ReadPlacement,
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
using pyramid::data_model::Capabilities;
using pyramid::data_model::ExecutionRun;
using pyramid::data_model::Identifier;
using pyramid::data_model::Plan;
using pyramid::data_model::PlanningExecutionRequirement;
using pyramid::data_model::Query;
using pyramid::data_model::RequirementPlacement;
using pyramid::data_model::StateUpdate;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Capabilities_Service
    virtual std::vector<Capabilities>
    handleReadCapabilities(const Query& request);

    // Planning_Execution_Service
    virtual Identifier
    handleCreateRequirement(const PlanningExecutionRequirement& request);

    virtual std::vector<PlanningExecutionRequirement>
    handleReadRequirement(const Query& request);

    virtual Ack
    handleUpdateRequirement(const PlanningExecutionRequirement& request);

    virtual Ack
    handleDeleteRequirement(const Identifier& request);

    // State_Service
    virtual Identifier
    handleCreateState(const StateUpdate& request);

    virtual Ack
    handleUpdateState(const StateUpdate& request);

    virtual Ack
    handleDeleteState(const Identifier& request);

    // Plan_Service
    virtual std::vector<Plan>
    handleReadPlan(const Query& request);

    // Execution_Run_Service
    virtual std::vector<ExecutionRun>
    handleReadRun(const Query& request);

    // Requirement_Placement_Service
    virtual std::vector<RequirementPlacement>
    handleReadPlacement(const Query& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish / Invoke (typed)
// ---------------------------------------------------------------------------

/// \brief Decode a response from capabilities.read_capabilities.
bool decodeReadCapabilitiesResponse(const pcl_msg_t* msg,
                                    std::vector<Capabilities>* out);

/// \brief Invoke capabilities.read_capabilities (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadCapabilities(pcl_executor_t* executor,
                                    const Query&                 request,
                                    pcl_resp_cb_fn_t        callback,
                                    void*                   user_data = nullptr,
                                    const pcl_endpoint_route_t* route = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Invoke capabilities.read_capabilities and ignore the async response.
pcl_status_t invokeReadCapabilities(pcl_executor_t* executor,
                                    const Query&                 request,
                                    const char*       content_type = "application/json",
                                    const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_execution.create_requirement.
bool decodeCreateRequirementResponse(const pcl_msg_t* msg,
                                     Identifier* out);

/// \brief Invoke planning_execution.create_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const PlanningExecutionRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke planning_execution.create_requirement and ignore the async response.
pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const PlanningExecutionRequirement& request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_execution.read_requirement.
bool decodeReadRequirementResponse(const pcl_msg_t* msg,
                                   std::vector<PlanningExecutionRequirement>* out);

/// \brief Invoke planning_execution.read_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data = nullptr,
                                   const pcl_endpoint_route_t* route = nullptr,
                                   const char*       content_type = "application/json");

/// \brief Invoke planning_execution.read_requirement and ignore the async response.
pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   const char*       content_type = "application/json",
                                   const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_execution.update_requirement.
bool decodeUpdateRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out);

/// \brief Invoke planning_execution.update_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const PlanningExecutionRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke planning_execution.update_requirement and ignore the async response.
pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const PlanningExecutionRequirement& request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_execution.delete_requirement.
bool decodeDeleteRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out);

/// \brief Invoke planning_execution.delete_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke planning_execution.delete_requirement and ignore the async response.
pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from state.create_state.
bool decodeCreateStateResponse(const pcl_msg_t* msg,
                               Identifier* out);

/// \brief Invoke state.create_state (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
                               pcl_resp_cb_fn_t        callback,
                               void*                   user_data = nullptr,
                               const pcl_endpoint_route_t* route = nullptr,
                               const char*       content_type = "application/json");

/// \brief Invoke state.create_state and ignore the async response.
pcl_status_t invokeCreateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
                               const char*       content_type = "application/json",
                               const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from state.update_state.
bool decodeUpdateStateResponse(const pcl_msg_t* msg,
                               Ack* out);

/// \brief Invoke state.update_state (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
                               pcl_resp_cb_fn_t        callback,
                               void*                   user_data = nullptr,
                               const pcl_endpoint_route_t* route = nullptr,
                               const char*       content_type = "application/json");

/// \brief Invoke state.update_state and ignore the async response.
pcl_status_t invokeUpdateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
                               const char*       content_type = "application/json",
                               const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from state.delete_state.
bool decodeDeleteStateResponse(const pcl_msg_t* msg,
                               Ack* out);

/// \brief Invoke state.delete_state (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteState(pcl_executor_t* executor,
                               const Identifier&            request,
                               pcl_resp_cb_fn_t        callback,
                               void*                   user_data = nullptr,
                               const pcl_endpoint_route_t* route = nullptr,
                               const char*       content_type = "application/json");

/// \brief Invoke state.delete_state and ignore the async response.
pcl_status_t invokeDeleteState(pcl_executor_t* executor,
                               const Identifier&            request,
                               const char*       content_type = "application/json",
                               const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from plan.read_plan.
bool decodeReadPlanResponse(const pcl_msg_t* msg,
                            std::vector<Plan>* out);

/// \brief Invoke plan.read_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadPlan(pcl_executor_t* executor,
                            const Query&                 request,
                            pcl_resp_cb_fn_t        callback,
                            void*                   user_data = nullptr,
                            const pcl_endpoint_route_t* route = nullptr,
                            const char*       content_type = "application/json");

/// \brief Invoke plan.read_plan and ignore the async response.
pcl_status_t invokeReadPlan(pcl_executor_t* executor,
                            const Query&                 request,
                            const char*       content_type = "application/json",
                            const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_run.read_run.
bool decodeReadRunResponse(const pcl_msg_t* msg,
                           std::vector<ExecutionRun>* out);

/// \brief Invoke execution_run.read_run (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadRun(pcl_executor_t* executor,
                           const Query&                 request,
                           pcl_resp_cb_fn_t        callback,
                           void*                   user_data = nullptr,
                           const pcl_endpoint_route_t* route = nullptr,
                           const char*       content_type = "application/json");

/// \brief Invoke execution_run.read_run and ignore the async response.
pcl_status_t invokeReadRun(pcl_executor_t* executor,
                           const Query&                 request,
                           const char*       content_type = "application/json",
                           const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from requirement_placement.read_placement.
bool decodeReadPlacementResponse(const pcl_msg_t* msg,
                                 std::vector<RequirementPlacement>* out);

/// \brief Invoke requirement_placement.read_placement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadPlacement(pcl_executor_t* executor,
                                 const Query&                 request,
                                 pcl_resp_cb_fn_t        callback,
                                 void*                   user_data = nullptr,
                                 const pcl_endpoint_route_t* route = nullptr,
                                 const char*       content_type = "application/json");

/// \brief Invoke requirement_placement.read_placement and ignore the async response.
pcl_status_t invokeReadPlacement(pcl_executor_t* executor,
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
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadCapabilities);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcCreateRequirement);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcUpdateRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcDeleteRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcCreateState);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcUpdateState);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcDeleteState);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadPlan);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadRun);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadPlacement);
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

} // namespace pyramid::services::autonomy_backend::provided
