// Auto-generated service binding header
// Generated from: provided.proto by generate_bindings.py
// Namespace: pyramid::services::autonomy_backend::provided
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

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_types.h>

#include <string>
#include <vector>

namespace pyramid::services::autonomy_backend::provided {

// ---------------------------------------------------------------------------
// Service wire-name constants (generated from proto)
// ---------------------------------------------------------------------------

constexpr const char* kSvcReadCapabilities      = "capabilities.read_capabilities";
constexpr const char* kSvcCreateSession         = "session.create_session";
constexpr const char* kSvcReadSession           = "session.read_session";
constexpr const char* kSvcUpdateSession         = "session.update_session";
constexpr const char* kSvcDeleteSession         = "session.delete_session";
constexpr const char* kSvcCreateState           = "state.create_state";
constexpr const char* kSvcUpdateState           = "state.update_state";
constexpr const char* kSvcDeleteState           = "state.delete_state";
constexpr const char* kSvcCreateIntent          = "intent.create_intent";
constexpr const char* kSvcUpdateIntent          = "intent.update_intent";
constexpr const char* kSvcDeleteIntent          = "intent.delete_intent";
constexpr const char* kSvcReadCommand           = "command.read_command";
constexpr const char* kSvcReadGoalDispatch      = "goal_dispatch.read_goal_dispatch";
constexpr const char* kSvcReadDecisionRecord    = "decision_record.read_decision_record";
constexpr const char* kSvcCreateCommandResult   = "command_result.create_command_result";
constexpr const char* kSvcUpdateCommandResult   = "command_result.update_command_result";
constexpr const char* kSvcDeleteCommandResult   = "command_result.delete_command_result";
constexpr const char* kSvcCreateDispatchResult  = "dispatch_result.create_dispatch_result";
constexpr const char* kSvcUpdateDispatchResult  = "dispatch_result.update_dispatch_result";
constexpr const char* kSvcDeleteDispatchResult  = "dispatch_result.delete_dispatch_result";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    ReadCapabilities,
    CreateSession,
    ReadSession,
    UpdateSession,
    DeleteSession,
    CreateState,
    UpdateState,
    DeleteState,
    CreateIntent,
    UpdateIntent,
    DeleteIntent,
    ReadCommand,
    ReadGoalDispatch,
    ReadDecisionRecord,
    CreateCommandResult,
    UpdateCommandResult,
    DeleteCommandResult,
    CreateDispatchResult,
    UpdateDispatchResult,
    DeleteDispatchResult,
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
using pyramid::data_model::Command;
using pyramid::data_model::CommandResult;
using pyramid::data_model::DecisionRecord;
using pyramid::data_model::DispatchResult;
using pyramid::data_model::GoalDispatch;
using pyramid::data_model::Identifier;
using pyramid::data_model::MissionIntent;
using pyramid::data_model::Query;
using pyramid::data_model::Session;
using pyramid::data_model::SessionSnapshot;
using pyramid::data_model::SessionStepRequest;
using pyramid::data_model::SessionStopRequest;
using pyramid::data_model::StateUpdate;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Capabilities_Service
    virtual std::vector<Capabilities>
    handleReadCapabilities(const Query& request);

    // Session_Service
    virtual Identifier
    handleCreateSession(const Session& request);

    virtual std::vector<SessionSnapshot>
    handleReadSession(const Query& request);

    virtual Ack
    handleUpdateSession(const SessionStepRequest& request);

    virtual Ack
    handleDeleteSession(const SessionStopRequest& request);

    // State_Service
    virtual Identifier
    handleCreateState(const StateUpdate& request);

    virtual Ack
    handleUpdateState(const StateUpdate& request);

    virtual Ack
    handleDeleteState(const Identifier& request);

    // Intent_Service
    virtual Identifier
    handleCreateIntent(const MissionIntent& request);

    virtual Ack
    handleUpdateIntent(const MissionIntent& request);

    virtual Ack
    handleDeleteIntent(const Identifier& request);

    // Command_Service
    virtual std::vector<Command>
    handleReadCommand(const Query& request);

    // GoalDispatch_Service
    virtual std::vector<GoalDispatch>
    handleReadGoalDispatch(const Query& request);

    // DecisionRecord_Service
    virtual std::vector<DecisionRecord>
    handleReadDecisionRecord(const Query& request);

    // CommandResult_Service
    virtual Identifier
    handleCreateCommandResult(const CommandResult& request);

    virtual Ack
    handleUpdateCommandResult(const CommandResult& request);

    virtual Ack
    handleDeleteCommandResult(const Identifier& request);

    // DispatchResult_Service
    virtual Identifier
    handleCreateDispatchResult(const DispatchResult& request);

    virtual Ack
    handleUpdateDispatchResult(const DispatchResult& request);

    virtual Ack
    handleDeleteDispatchResult(const Identifier& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Invoke (typed)
// ---------------------------------------------------------------------------

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

/// \brief Invoke session.create_session (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateSession(pcl_executor_t* executor,
                                 const Session&               request,
                                 pcl_resp_cb_fn_t        callback,
                                 void*                   user_data = nullptr,
                                 const pcl_endpoint_route_t* route = nullptr,
                                 const char*       content_type = "application/json");

/// \brief Invoke session.read_session (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadSession(pcl_executor_t* executor,
                               const Query&                 request,
                               pcl_resp_cb_fn_t        callback,
                               void*                   user_data = nullptr,
                               const pcl_endpoint_route_t* route = nullptr,
                               const char*       content_type = "application/json");

/// \brief Invoke session.update_session (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateSession(pcl_executor_t* executor,
                                 const SessionStepRequest&    request,
                                 pcl_resp_cb_fn_t        callback,
                                 void*                   user_data = nullptr,
                                 const pcl_endpoint_route_t* route = nullptr,
                                 const char*       content_type = "application/json");

/// \brief Invoke session.delete_session (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteSession(pcl_executor_t* executor,
                                 const SessionStopRequest&    request,
                                 pcl_resp_cb_fn_t        callback,
                                 void*                   user_data = nullptr,
                                 const pcl_endpoint_route_t* route = nullptr,
                                 const char*       content_type = "application/json");

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

/// \brief Invoke intent.create_intent (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateIntent(pcl_executor_t* executor,
                                const MissionIntent&         request,
                                pcl_resp_cb_fn_t        callback,
                                void*                   user_data = nullptr,
                                const pcl_endpoint_route_t* route = nullptr,
                                const char*       content_type = "application/json");

/// \brief Invoke intent.update_intent (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateIntent(pcl_executor_t* executor,
                                const MissionIntent&         request,
                                pcl_resp_cb_fn_t        callback,
                                void*                   user_data = nullptr,
                                const pcl_endpoint_route_t* route = nullptr,
                                const char*       content_type = "application/json");

/// \brief Invoke intent.delete_intent (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteIntent(pcl_executor_t* executor,
                                const Identifier&            request,
                                pcl_resp_cb_fn_t        callback,
                                void*                   user_data = nullptr,
                                const pcl_endpoint_route_t* route = nullptr,
                                const char*       content_type = "application/json");

/// \brief Invoke command.read_command (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadCommand(pcl_executor_t* executor,
                               const Query&                 request,
                               pcl_resp_cb_fn_t        callback,
                               void*                   user_data = nullptr,
                               const pcl_endpoint_route_t* route = nullptr,
                               const char*       content_type = "application/json");

/// \brief Invoke goal_dispatch.read_goal_dispatch (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadGoalDispatch(pcl_executor_t* executor,
                                    const Query&                 request,
                                    pcl_resp_cb_fn_t        callback,
                                    void*                   user_data = nullptr,
                                    const pcl_endpoint_route_t* route = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Invoke decision_record.read_decision_record (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeReadDecisionRecord(pcl_executor_t* executor,
                                      const Query&                 request,
                                      pcl_resp_cb_fn_t        callback,
                                      void*                   user_data = nullptr,
                                      const pcl_endpoint_route_t* route = nullptr,
                                      const char*       content_type = "application/json");

/// \brief Invoke command_result.create_command_result (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateCommandResult(pcl_executor_t* executor,
                                       const CommandResult&         request,
                                       pcl_resp_cb_fn_t        callback,
                                       void*                   user_data = nullptr,
                                       const pcl_endpoint_route_t* route = nullptr,
                                       const char*       content_type = "application/json");

/// \brief Invoke command_result.update_command_result (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateCommandResult(pcl_executor_t* executor,
                                       const CommandResult&         request,
                                       pcl_resp_cb_fn_t        callback,
                                       void*                   user_data = nullptr,
                                       const pcl_endpoint_route_t* route = nullptr,
                                       const char*       content_type = "application/json");

/// \brief Invoke command_result.delete_command_result (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteCommandResult(pcl_executor_t* executor,
                                       const Identifier&            request,
                                       pcl_resp_cb_fn_t        callback,
                                       void*                   user_data = nullptr,
                                       const pcl_endpoint_route_t* route = nullptr,
                                       const char*       content_type = "application/json");

/// \brief Invoke dispatch_result.create_dispatch_result (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeCreateDispatchResult(pcl_executor_t* executor,
                                        const DispatchResult&        request,
                                        pcl_resp_cb_fn_t        callback,
                                        void*                   user_data = nullptr,
                                        const pcl_endpoint_route_t* route = nullptr,
                                        const char*       content_type = "application/json");

/// \brief Invoke dispatch_result.update_dispatch_result (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeUpdateDispatchResult(pcl_executor_t* executor,
                                        const DispatchResult&        request,
                                        pcl_resp_cb_fn_t        callback,
                                        void*                   user_data = nullptr,
                                        const pcl_endpoint_route_t* route = nullptr,
                                        const char*       content_type = "application/json");

/// \brief Invoke dispatch_result.delete_dispatch_result (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeDeleteDispatchResult(pcl_executor_t* executor,
                                        const Identifier&            request,
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

} // namespace pyramid::services::autonomy_backend::provided
