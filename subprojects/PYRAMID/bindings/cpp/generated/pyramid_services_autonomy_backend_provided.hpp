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

constexpr const char* kSvcReadCapabilities            = "capabilities.read_capabilities";
constexpr const char* kSvcCreatePlanningRequirement   = "planning_requirement.create_planning_requirement";
constexpr const char* kSvcReadPlanningRequirement     = "planning_requirement.read_planning_requirement";
constexpr const char* kSvcUpdatePlanningRequirement   = "planning_requirement.update_planning_requirement";
constexpr const char* kSvcDeletePlanningRequirement   = "planning_requirement.delete_planning_requirement";
constexpr const char* kSvcCreateExecutionRequirement  = "execution_requirement.create_execution_requirement";
constexpr const char* kSvcReadExecutionRequirement    = "execution_requirement.read_execution_requirement";
constexpr const char* kSvcUpdateExecutionRequirement  = "execution_requirement.update_execution_requirement";
constexpr const char* kSvcDeleteExecutionRequirement  = "execution_requirement.delete_execution_requirement";
constexpr const char* kSvcCreateState                 = "state.create_state";
constexpr const char* kSvcUpdateState                 = "state.update_state";
constexpr const char* kSvcDeleteState                 = "state.delete_state";
constexpr const char* kSvcCreatePlan                  = "plan.create_plan";
constexpr const char* kSvcReadPlan                    = "plan.read_plan";
constexpr const char* kSvcUpdatePlan                  = "plan.update_plan";
constexpr const char* kSvcDeletePlan                  = "plan.delete_plan";
constexpr const char* kSvcReadRun                     = "execution_run.read_run";
constexpr const char* kSvcReadPlacement               = "requirement_placement.read_placement";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    ReadCapabilities,
    CreatePlanningRequirement,
    ReadPlanningRequirement,
    UpdatePlanningRequirement,
    DeletePlanningRequirement,
    CreateExecutionRequirement,
    ReadExecutionRequirement,
    UpdateExecutionRequirement,
    DeleteExecutionRequirement,
    CreateState,
    UpdateState,
    DeleteState,
    CreatePlan,
    ReadPlan,
    UpdatePlan,
    DeletePlan,
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
using pyramid::data_model::ExecutionRequirement;
using pyramid::data_model::ExecutionRun;
using pyramid::data_model::Identifier;
using pyramid::data_model::Plan;
using pyramid::data_model::PlanningRequirement;
using pyramid::data_model::Query;
using pyramid::data_model::RequirementPlacement;
using pyramid::data_model::StateUpdate;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Capabilities_Service
    virtual std::vector<Capabilities>
    handleReadCapabilities(const Query& request);

    // Planning_Requirement_Service
    virtual Identifier
    handleCreatePlanningRequirement(const PlanningRequirement& request);

    virtual std::vector<PlanningRequirement>
    handleReadPlanningRequirement(const Query& request);

    virtual Ack
    handleUpdatePlanningRequirement(const PlanningRequirement& request);

    virtual Ack
    handleDeletePlanningRequirement(const Identifier& request);

    // Execution_Requirement_Service
    virtual Identifier
    handleCreateExecutionRequirement(const ExecutionRequirement& request);

    virtual std::vector<ExecutionRequirement>
    handleReadExecutionRequirement(const Query& request);

    virtual Ack
    handleUpdateExecutionRequirement(const ExecutionRequirement& request);

    virtual Ack
    handleDeleteExecutionRequirement(const Identifier& request);

    // State_Service
    virtual Identifier
    handleCreateState(const StateUpdate& request);

    virtual Ack
    handleUpdateState(const StateUpdate& request);

    virtual Ack
    handleDeleteState(const Identifier& request);

    // Plan_Service
    virtual Identifier
    handleCreatePlan(const Plan& request);

    virtual std::vector<Plan>
    handleReadPlan(const Query& request);

    virtual Ack
    handleUpdatePlan(const Plan& request);

    virtual Ack
    handleDeletePlan(const Identifier& request);

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

/// \brief Decode a response from planning_requirement.create_planning_requirement.
bool decodeCreatePlanningRequirementResponse(const pcl_msg_t* msg,
                                             Identifier* out);

/// \brief Invoke planning_requirement.create_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreatePlanningRequirement(pcl_executor_t* executor,
                                             const PlanningRequirement&   request,
                                             pcl_resp_cb_fn_t        callback,
                                             void*                   user_data = nullptr,
                                             const pcl_endpoint_route_t* route = nullptr,
                                             const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.create_planning_requirement and ignore the async response.
pcl_status_t invokeCreatePlanningRequirement(pcl_executor_t* executor,
                                             const PlanningRequirement&   request,
                                             const char*       content_type = "application/json",
                                             const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_requirement.read_planning_requirement.
bool decodeReadPlanningRequirementResponse(const pcl_msg_t* msg,
                                           std::vector<PlanningRequirement>* out);

/// \brief Invoke planning_requirement.read_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadPlanningRequirement(pcl_executor_t* executor,
                                           const Query&                 request,
                                           pcl_resp_cb_fn_t        callback,
                                           void*                   user_data = nullptr,
                                           const pcl_endpoint_route_t* route = nullptr,
                                           const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.read_planning_requirement and ignore the async response.
pcl_status_t invokeReadPlanningRequirement(pcl_executor_t* executor,
                                           const Query&                 request,
                                           const char*       content_type = "application/json",
                                           const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_requirement.update_planning_requirement.
bool decodeUpdatePlanningRequirementResponse(const pcl_msg_t* msg,
                                             Ack* out);

/// \brief Invoke planning_requirement.update_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdatePlanningRequirement(pcl_executor_t* executor,
                                             const PlanningRequirement&   request,
                                             pcl_resp_cb_fn_t        callback,
                                             void*                   user_data = nullptr,
                                             const pcl_endpoint_route_t* route = nullptr,
                                             const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.update_planning_requirement and ignore the async response.
pcl_status_t invokeUpdatePlanningRequirement(pcl_executor_t* executor,
                                             const PlanningRequirement&   request,
                                             const char*       content_type = "application/json",
                                             const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_requirement.delete_planning_requirement.
bool decodeDeletePlanningRequirementResponse(const pcl_msg_t* msg,
                                             Ack* out);

/// \brief Invoke planning_requirement.delete_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeletePlanningRequirement(pcl_executor_t* executor,
                                             const Identifier&            request,
                                             pcl_resp_cb_fn_t        callback,
                                             void*                   user_data = nullptr,
                                             const pcl_endpoint_route_t* route = nullptr,
                                             const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.delete_planning_requirement and ignore the async response.
pcl_status_t invokeDeletePlanningRequirement(pcl_executor_t* executor,
                                             const Identifier&            request,
                                             const char*       content_type = "application/json",
                                             const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.create_execution_requirement.
bool decodeCreateExecutionRequirementResponse(const pcl_msg_t* msg,
                                              Identifier* out);

/// \brief Invoke execution_requirement.create_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateExecutionRequirement(pcl_executor_t* executor,
                                              const ExecutionRequirement&  request,
                                              pcl_resp_cb_fn_t        callback,
                                              void*                   user_data = nullptr,
                                              const pcl_endpoint_route_t* route = nullptr,
                                              const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.create_execution_requirement and ignore the async response.
pcl_status_t invokeCreateExecutionRequirement(pcl_executor_t* executor,
                                              const ExecutionRequirement&  request,
                                              const char*       content_type = "application/json",
                                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.read_execution_requirement.
bool decodeReadExecutionRequirementResponse(const pcl_msg_t* msg,
                                            std::vector<ExecutionRequirement>* out);

/// \brief Invoke execution_requirement.read_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadExecutionRequirement(pcl_executor_t* executor,
                                            const Query&                 request,
                                            pcl_resp_cb_fn_t        callback,
                                            void*                   user_data = nullptr,
                                            const pcl_endpoint_route_t* route = nullptr,
                                            const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.read_execution_requirement and ignore the async response.
pcl_status_t invokeReadExecutionRequirement(pcl_executor_t* executor,
                                            const Query&                 request,
                                            const char*       content_type = "application/json",
                                            const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.update_execution_requirement.
bool decodeUpdateExecutionRequirementResponse(const pcl_msg_t* msg,
                                              Ack* out);

/// \brief Invoke execution_requirement.update_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateExecutionRequirement(pcl_executor_t* executor,
                                              const ExecutionRequirement&  request,
                                              pcl_resp_cb_fn_t        callback,
                                              void*                   user_data = nullptr,
                                              const pcl_endpoint_route_t* route = nullptr,
                                              const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.update_execution_requirement and ignore the async response.
pcl_status_t invokeUpdateExecutionRequirement(pcl_executor_t* executor,
                                              const ExecutionRequirement&  request,
                                              const char*       content_type = "application/json",
                                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.delete_execution_requirement.
bool decodeDeleteExecutionRequirementResponse(const pcl_msg_t* msg,
                                              Ack* out);

/// \brief Invoke execution_requirement.delete_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteExecutionRequirement(pcl_executor_t* executor,
                                              const Identifier&            request,
                                              pcl_resp_cb_fn_t        callback,
                                              void*                   user_data = nullptr,
                                              const pcl_endpoint_route_t* route = nullptr,
                                              const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.delete_execution_requirement and ignore the async response.
pcl_status_t invokeDeleteExecutionRequirement(pcl_executor_t* executor,
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

/// \brief Decode a response from plan.create_plan.
bool decodeCreatePlanResponse(const pcl_msg_t* msg,
                              Identifier* out);

/// \brief Invoke plan.create_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreatePlan(pcl_executor_t* executor,
                              const Plan&                  request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr,
                              const pcl_endpoint_route_t* route = nullptr,
                              const char*       content_type = "application/json");

/// \brief Invoke plan.create_plan and ignore the async response.
pcl_status_t invokeCreatePlan(pcl_executor_t* executor,
                              const Plan&                  request,
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

/// \brief Decode a response from plan.update_plan.
bool decodeUpdatePlanResponse(const pcl_msg_t* msg,
                              Ack* out);

/// \brief Invoke plan.update_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdatePlan(pcl_executor_t* executor,
                              const Plan&                  request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr,
                              const pcl_endpoint_route_t* route = nullptr,
                              const char*       content_type = "application/json");

/// \brief Invoke plan.update_plan and ignore the async response.
pcl_status_t invokeUpdatePlan(pcl_executor_t* executor,
                              const Plan&                  request,
                              const char*       content_type = "application/json",
                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from plan.delete_plan.
bool decodeDeletePlanResponse(const pcl_msg_t* msg,
                              Ack* out);

/// \brief Invoke plan.delete_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeletePlan(pcl_executor_t* executor,
                              const Identifier&            request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr,
                              const pcl_endpoint_route_t* route = nullptr,
                              const char*       content_type = "application/json");

/// \brief Invoke plan.delete_plan and ignore the async response.
pcl_status_t invokeDeletePlan(pcl_executor_t* executor,
                              const Identifier&            request,
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
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcCreatePlanningRequirement);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadPlanningRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcUpdatePlanningRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcDeletePlanningRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcCreateExecutionRequirement);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadExecutionRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcUpdateExecutionRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcDeleteExecutionRequirement);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcCreateState);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcUpdateState);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcDeleteState);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcCreatePlan);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadPlan);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcUpdatePlan);
    pyramid::transport::ros2::bindUnaryServiceIngress(adapter, executor, kSvcDeletePlan);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadRun);
    pyramid::transport::ros2::bindStreamServiceIngress(adapter, executor, kSvcReadPlacement);
}

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

} // namespace pyramid::services::autonomy_backend::provided
