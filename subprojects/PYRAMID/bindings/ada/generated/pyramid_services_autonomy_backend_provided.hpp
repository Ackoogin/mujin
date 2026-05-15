// Auto-generated service binding header
// Generated from: pyramid.components.autonomy_backend.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::autonomy_backend::services::provided
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

namespace pyramid::components::autonomy_backend::services::provided {

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

constexpr const char* kSvcCapabilitiesReadCapabilities                    = "capabilities.read_capabilities";
constexpr const char* kSvcPlanningRequirementCreatePlanningRequirement    = "planning_requirement.create_planning_requirement";
constexpr const char* kSvcPlanningRequirementReadPlanningRequirement      = "planning_requirement.read_planning_requirement";
constexpr const char* kSvcPlanningRequirementUpdatePlanningRequirement    = "planning_requirement.update_planning_requirement";
constexpr const char* kSvcPlanningRequirementDeletePlanningRequirement    = "planning_requirement.delete_planning_requirement";
constexpr const char* kSvcExecutionRequirementCreateExecutionRequirement  = "execution_requirement.create_execution_requirement";
constexpr const char* kSvcExecutionRequirementReadExecutionRequirement    = "execution_requirement.read_execution_requirement";
constexpr const char* kSvcExecutionRequirementUpdateExecutionRequirement  = "execution_requirement.update_execution_requirement";
constexpr const char* kSvcExecutionRequirementDeleteExecutionRequirement  = "execution_requirement.delete_execution_requirement";
constexpr const char* kSvcStateCreateState                                = "state.create_state";
constexpr const char* kSvcStateUpdateState                                = "state.update_state";
constexpr const char* kSvcStateDeleteState                                = "state.delete_state";
constexpr const char* kSvcPlanCreatePlan                                  = "plan.create_plan";
constexpr const char* kSvcPlanReadPlan                                    = "plan.read_plan";
constexpr const char* kSvcPlanUpdatePlan                                  = "plan.update_plan";
constexpr const char* kSvcPlanDeletePlan                                  = "plan.delete_plan";
constexpr const char* kSvcExecutionRunReadRun                             = "execution_run.read_run";
constexpr const char* kSvcRequirementPlacementReadPlacement               = "requirement_placement.read_placement";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    CapabilitiesReadCapabilities,
    PlanningRequirementCreatePlanningRequirement,
    PlanningRequirementReadPlanningRequirement,
    PlanningRequirementUpdatePlanningRequirement,
    PlanningRequirementDeletePlanningRequirement,
    ExecutionRequirementCreateExecutionRequirement,
    ExecutionRequirementReadExecutionRequirement,
    ExecutionRequirementUpdateExecutionRequirement,
    ExecutionRequirementDeleteExecutionRequirement,
    StateCreateState,
    StateUpdateState,
    StateDeleteState,
    PlanCreatePlan,
    PlanReadPlan,
    PlanUpdatePlan,
    PlanDeletePlan,
    ExecutionRunReadRun,
    RequirementPlacementReadPlacement,
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
using pyramid::domain_model::Capabilities;
using pyramid::domain_model::ExecutionRequirement;
using pyramid::domain_model::ExecutionRun;
using pyramid::domain_model::Identifier;
using pyramid::domain_model::Plan;
using pyramid::domain_model::PlanningRequirement;
using pyramid::domain_model::Query;
using pyramid::domain_model::RequirementPlacement;
using pyramid::domain_model::StateUpdate;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Capabilities_Service
    virtual std::vector<Capabilities>
    handleCapabilitiesReadCapabilities(const Query& request);

    // Planning_Requirement_Service
    virtual Identifier
    handlePlanningRequirementCreatePlanningRequirement(const PlanningRequirement& request);

    virtual std::vector<PlanningRequirement>
    handlePlanningRequirementReadPlanningRequirement(const Query& request);

    virtual Ack
    handlePlanningRequirementUpdatePlanningRequirement(const PlanningRequirement& request);

    virtual Ack
    handlePlanningRequirementDeletePlanningRequirement(const Identifier& request);

    // Execution_Requirement_Service
    virtual Identifier
    handleExecutionRequirementCreateExecutionRequirement(const ExecutionRequirement& request);

    virtual std::vector<ExecutionRequirement>
    handleExecutionRequirementReadExecutionRequirement(const Query& request);

    virtual Ack
    handleExecutionRequirementUpdateExecutionRequirement(const ExecutionRequirement& request);

    virtual Ack
    handleExecutionRequirementDeleteExecutionRequirement(const Identifier& request);

    // State_Service
    virtual Identifier
    handleStateCreateState(const StateUpdate& request);

    virtual Ack
    handleStateUpdateState(const StateUpdate& request);

    virtual Ack
    handleStateDeleteState(const Identifier& request);

    // Plan_Service
    virtual Identifier
    handlePlanCreatePlan(const Plan& request);

    virtual std::vector<Plan>
    handlePlanReadPlan(const Query& request);

    virtual Ack
    handlePlanUpdatePlan(const Plan& request);

    virtual Ack
    handlePlanDeletePlan(const Identifier& request);

    // Execution_Run_Service
    virtual std::vector<ExecutionRun>
    handleExecutionRunReadRun(const Query& request);

    // Requirement_Placement_Service
    virtual std::vector<RequirementPlacement>
    handleRequirementPlacementReadPlacement(const Query& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish / Invoke (typed)
// ---------------------------------------------------------------------------

/// \brief Decode a response from capabilities.read_capabilities.
bool decodeCapabilitiesReadCapabilitiesResponse(const pcl_msg_t* msg,
                                                std::vector<Capabilities>* out);

/// \brief Invoke capabilities.read_capabilities (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCapabilitiesReadCapabilities(pcl_executor_t* executor,
                                                const Query&                 request,
                                                pcl_resp_cb_fn_t        callback,
                                                void*                   user_data = nullptr,
                                                const pcl_endpoint_route_t* route = nullptr,
                                                const char*       content_type = "application/json");

/// \brief Invoke capabilities.read_capabilities and ignore the async response.
pcl_status_t invokeCapabilitiesReadCapabilities(pcl_executor_t* executor,
                                                const Query&                 request,
                                                const char*       content_type = "application/json",
                                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_requirement.create_planning_requirement.
bool decodePlanningRequirementCreatePlanningRequirementResponse(const pcl_msg_t* msg,
                                                                Identifier* out);

/// \brief Invoke planning_requirement.create_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanningRequirementCreatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
                                                                pcl_resp_cb_fn_t        callback,
                                                                void*                   user_data = nullptr,
                                                                const pcl_endpoint_route_t* route = nullptr,
                                                                const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.create_planning_requirement and ignore the async response.
pcl_status_t invokePlanningRequirementCreatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
                                                                const char*       content_type = "application/json",
                                                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_requirement.read_planning_requirement.
bool decodePlanningRequirementReadPlanningRequirementResponse(const pcl_msg_t* msg,
                                                              std::vector<PlanningRequirement>* out);

/// \brief Invoke planning_requirement.read_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanningRequirementReadPlanningRequirement(pcl_executor_t* executor,
                                                              const Query&                 request,
                                                              pcl_resp_cb_fn_t        callback,
                                                              void*                   user_data = nullptr,
                                                              const pcl_endpoint_route_t* route = nullptr,
                                                              const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.read_planning_requirement and ignore the async response.
pcl_status_t invokePlanningRequirementReadPlanningRequirement(pcl_executor_t* executor,
                                                              const Query&                 request,
                                                              const char*       content_type = "application/json",
                                                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_requirement.update_planning_requirement.
bool decodePlanningRequirementUpdatePlanningRequirementResponse(const pcl_msg_t* msg,
                                                                Ack* out);

/// \brief Invoke planning_requirement.update_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanningRequirementUpdatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
                                                                pcl_resp_cb_fn_t        callback,
                                                                void*                   user_data = nullptr,
                                                                const pcl_endpoint_route_t* route = nullptr,
                                                                const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.update_planning_requirement and ignore the async response.
pcl_status_t invokePlanningRequirementUpdatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
                                                                const char*       content_type = "application/json",
                                                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from planning_requirement.delete_planning_requirement.
bool decodePlanningRequirementDeletePlanningRequirementResponse(const pcl_msg_t* msg,
                                                                Ack* out);

/// \brief Invoke planning_requirement.delete_planning_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanningRequirementDeletePlanningRequirement(pcl_executor_t* executor,
                                                                const Identifier&            request,
                                                                pcl_resp_cb_fn_t        callback,
                                                                void*                   user_data = nullptr,
                                                                const pcl_endpoint_route_t* route = nullptr,
                                                                const char*       content_type = "application/json");

/// \brief Invoke planning_requirement.delete_planning_requirement and ignore the async response.
pcl_status_t invokePlanningRequirementDeletePlanningRequirement(pcl_executor_t* executor,
                                                                const Identifier&            request,
                                                                const char*       content_type = "application/json",
                                                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.create_execution_requirement.
bool decodeExecutionRequirementCreateExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                  Identifier* out);

/// \brief Invoke execution_requirement.create_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeExecutionRequirementCreateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
                                                                  pcl_resp_cb_fn_t        callback,
                                                                  void*                   user_data = nullptr,
                                                                  const pcl_endpoint_route_t* route = nullptr,
                                                                  const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.create_execution_requirement and ignore the async response.
pcl_status_t invokeExecutionRequirementCreateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
                                                                  const char*       content_type = "application/json",
                                                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.read_execution_requirement.
bool decodeExecutionRequirementReadExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                std::vector<ExecutionRequirement>* out);

/// \brief Invoke execution_requirement.read_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeExecutionRequirementReadExecutionRequirement(pcl_executor_t* executor,
                                                                const Query&                 request,
                                                                pcl_resp_cb_fn_t        callback,
                                                                void*                   user_data = nullptr,
                                                                const pcl_endpoint_route_t* route = nullptr,
                                                                const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.read_execution_requirement and ignore the async response.
pcl_status_t invokeExecutionRequirementReadExecutionRequirement(pcl_executor_t* executor,
                                                                const Query&                 request,
                                                                const char*       content_type = "application/json",
                                                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.update_execution_requirement.
bool decodeExecutionRequirementUpdateExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                  Ack* out);

/// \brief Invoke execution_requirement.update_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeExecutionRequirementUpdateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
                                                                  pcl_resp_cb_fn_t        callback,
                                                                  void*                   user_data = nullptr,
                                                                  const pcl_endpoint_route_t* route = nullptr,
                                                                  const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.update_execution_requirement and ignore the async response.
pcl_status_t invokeExecutionRequirementUpdateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
                                                                  const char*       content_type = "application/json",
                                                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_requirement.delete_execution_requirement.
bool decodeExecutionRequirementDeleteExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                  Ack* out);

/// \brief Invoke execution_requirement.delete_execution_requirement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeExecutionRequirementDeleteExecutionRequirement(pcl_executor_t* executor,
                                                                  const Identifier&            request,
                                                                  pcl_resp_cb_fn_t        callback,
                                                                  void*                   user_data = nullptr,
                                                                  const pcl_endpoint_route_t* route = nullptr,
                                                                  const char*       content_type = "application/json");

/// \brief Invoke execution_requirement.delete_execution_requirement and ignore the async response.
pcl_status_t invokeExecutionRequirementDeleteExecutionRequirement(pcl_executor_t* executor,
                                                                  const Identifier&            request,
                                                                  const char*       content_type = "application/json",
                                                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from state.create_state.
bool decodeStateCreateStateResponse(const pcl_msg_t* msg,
                                    Identifier* out);

/// \brief Invoke state.create_state (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeStateCreateState(pcl_executor_t* executor,
                                    const StateUpdate&           request,
                                    pcl_resp_cb_fn_t        callback,
                                    void*                   user_data = nullptr,
                                    const pcl_endpoint_route_t* route = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Invoke state.create_state and ignore the async response.
pcl_status_t invokeStateCreateState(pcl_executor_t* executor,
                                    const StateUpdate&           request,
                                    const char*       content_type = "application/json",
                                    const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from state.update_state.
bool decodeStateUpdateStateResponse(const pcl_msg_t* msg,
                                    Ack* out);

/// \brief Invoke state.update_state (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeStateUpdateState(pcl_executor_t* executor,
                                    const StateUpdate&           request,
                                    pcl_resp_cb_fn_t        callback,
                                    void*                   user_data = nullptr,
                                    const pcl_endpoint_route_t* route = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Invoke state.update_state and ignore the async response.
pcl_status_t invokeStateUpdateState(pcl_executor_t* executor,
                                    const StateUpdate&           request,
                                    const char*       content_type = "application/json",
                                    const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from state.delete_state.
bool decodeStateDeleteStateResponse(const pcl_msg_t* msg,
                                    Ack* out);

/// \brief Invoke state.delete_state (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeStateDeleteState(pcl_executor_t* executor,
                                    const Identifier&            request,
                                    pcl_resp_cb_fn_t        callback,
                                    void*                   user_data = nullptr,
                                    const pcl_endpoint_route_t* route = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Invoke state.delete_state and ignore the async response.
pcl_status_t invokeStateDeleteState(pcl_executor_t* executor,
                                    const Identifier&            request,
                                    const char*       content_type = "application/json",
                                    const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from plan.create_plan.
bool decodePlanCreatePlanResponse(const pcl_msg_t* msg,
                                  Identifier* out);

/// \brief Invoke plan.create_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanCreatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
                                  pcl_resp_cb_fn_t        callback,
                                  void*                   user_data = nullptr,
                                  const pcl_endpoint_route_t* route = nullptr,
                                  const char*       content_type = "application/json");

/// \brief Invoke plan.create_plan and ignore the async response.
pcl_status_t invokePlanCreatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
                                  const char*       content_type = "application/json",
                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from plan.read_plan.
bool decodePlanReadPlanResponse(const pcl_msg_t* msg,
                                std::vector<Plan>* out);

/// \brief Invoke plan.read_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanReadPlan(pcl_executor_t* executor,
                                const Query&                 request,
                                pcl_resp_cb_fn_t        callback,
                                void*                   user_data = nullptr,
                                const pcl_endpoint_route_t* route = nullptr,
                                const char*       content_type = "application/json");

/// \brief Invoke plan.read_plan and ignore the async response.
pcl_status_t invokePlanReadPlan(pcl_executor_t* executor,
                                const Query&                 request,
                                const char*       content_type = "application/json",
                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from plan.update_plan.
bool decodePlanUpdatePlanResponse(const pcl_msg_t* msg,
                                  Ack* out);

/// \brief Invoke plan.update_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanUpdatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
                                  pcl_resp_cb_fn_t        callback,
                                  void*                   user_data = nullptr,
                                  const pcl_endpoint_route_t* route = nullptr,
                                  const char*       content_type = "application/json");

/// \brief Invoke plan.update_plan and ignore the async response.
pcl_status_t invokePlanUpdatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
                                  const char*       content_type = "application/json",
                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from plan.delete_plan.
bool decodePlanDeletePlanResponse(const pcl_msg_t* msg,
                                  Ack* out);

/// \brief Invoke plan.delete_plan (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokePlanDeletePlan(pcl_executor_t* executor,
                                  const Identifier&            request,
                                  pcl_resp_cb_fn_t        callback,
                                  void*                   user_data = nullptr,
                                  const pcl_endpoint_route_t* route = nullptr,
                                  const char*       content_type = "application/json");

/// \brief Invoke plan.delete_plan and ignore the async response.
pcl_status_t invokePlanDeletePlan(pcl_executor_t* executor,
                                  const Identifier&            request,
                                  const char*       content_type = "application/json",
                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from execution_run.read_run.
bool decodeExecutionRunReadRunResponse(const pcl_msg_t* msg,
                                       std::vector<ExecutionRun>* out);

/// \brief Invoke execution_run.read_run (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeExecutionRunReadRun(pcl_executor_t* executor,
                                       const Query&                 request,
                                       pcl_resp_cb_fn_t        callback,
                                       void*                   user_data = nullptr,
                                       const pcl_endpoint_route_t* route = nullptr,
                                       const char*       content_type = "application/json");

/// \brief Invoke execution_run.read_run and ignore the async response.
pcl_status_t invokeExecutionRunReadRun(pcl_executor_t* executor,
                                       const Query&                 request,
                                       const char*       content_type = "application/json",
                                       const pcl_endpoint_route_t* route = nullptr);

/// \brief Decode a response from requirement_placement.read_placement.
bool decodeRequirementPlacementReadPlacementResponse(const pcl_msg_t* msg,
                                                     std::vector<RequirementPlacement>* out);

/// \brief Invoke requirement_placement.read_placement (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeRequirementPlacementReadPlacement(pcl_executor_t* executor,
                                                     const Query&                 request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data = nullptr,
                                                     const pcl_endpoint_route_t* route = nullptr,
                                                     const char*       content_type = "application/json");

/// \brief Invoke requirement_placement.read_placement and ignore the async response.
pcl_status_t invokeRequirementPlacementReadPlacement(pcl_executor_t* executor,
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

} // namespace pyramid::components::autonomy_backend::services::provided

// Legacy service namespace alias for existing callers.
namespace pyramid::services::autonomy_backend {
namespace provided = ::pyramid::components::autonomy_backend::services::provided;
} // namespace pyramid::services::autonomy_backend
