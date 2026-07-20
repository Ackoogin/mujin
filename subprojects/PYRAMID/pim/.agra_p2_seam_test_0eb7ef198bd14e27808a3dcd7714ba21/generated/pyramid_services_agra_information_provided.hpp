// Auto-generated service binding header
// Generated from: pyramid.components.agra.information.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::agra::information::services::provided
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
#include "pyramid_components_agra_information_services_provided_types.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_types.h>

#include <string>
#include <vector>

namespace pyramid::components::agra::information::services::provided {

// ---------------------------------------------------------------------------
// Content-type constants and support metadata
// ---------------------------------------------------------------------------

constexpr const char* kJsonContentType = "application/json";

bool supportsContentType(const char* content_type);
std::vector<const char*> supportedContentTypes();

// ---------------------------------------------------------------------------
// Service wire-name constants (generated from proto)
// ---------------------------------------------------------------------------

constexpr const char* kSvcMaActionRead                       = "ma_action.read";
constexpr const char* kSvcMaMissionplanRead                  = "ma_mission_plan.read";
constexpr const char* kSvcMaPlanningfunctionRead             = "ma_planning_function.read";
constexpr const char* kSvcMaResponseRead                     = "ma_response.read";
constexpr const char* kSvcMaTaskRead                         = "ma_task.read";
constexpr const char* kSvcMissioncontingencyalertRead        = "mission_contingency_alert.read";
constexpr const char* kSvcMaActionstatusRead                 = "ma_action_status.read";
constexpr const char* kSvcMaMissionplanactivationstatusRead  = "ma_mission_plan_activation_status.read";
constexpr const char* kSvcMaMissionplanexecutionstatusRead   = "ma_mission_plan_execution_status.read";
constexpr const char* kSvcMaPlanningfunctionstatusRead       = "ma_planning_function_status.read";
constexpr const char* kSvcMaTaskstatusRead                   = "ma_task_status.read";
constexpr const char* kSvcMaApprovalpolicyRead               = "ma_approval_policy.read";

// ---------------------------------------------------------------------------
// Standard topic name constants
// ---------------------------------------------------------------------------

constexpr const char* kTopicMaAction                       = "MA_Action";
constexpr const char* kTopicMaMissionplan                  = "MA_MissionPlan";
constexpr const char* kTopicMaPlanningfunction             = "MA_PlanningFunction";
constexpr const char* kTopicMaResponse                     = "MA_Response";
constexpr const char* kTopicMaTask                         = "MA_Task";
constexpr const char* kTopicMissioncontingencyalert        = "MissionContingencyAlert";
constexpr const char* kTopicMaActionstatus                 = "MA_ActionStatus";
constexpr const char* kTopicMaMissionplanactivationstatus  = "MA_MissionPlanActivationStatus";
constexpr const char* kTopicMaMissionplanexecutionstatus   = "MA_MissionPlanExecutionStatus";
constexpr const char* kTopicMaPlanningfunctionstatus       = "MA_PlanningFunctionStatus";
constexpr const char* kTopicMaTaskstatus                   = "MA_TaskStatus";
constexpr const char* kTopicMaApprovalpolicy               = "MA_ApprovalPolicy";

inline constexpr pcl_qos_t kTopicMaActionQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaMissionplanQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaPlanningfunctionQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaResponseQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaTaskQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMissioncontingencyalertQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaActionstatusQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaMissionplanactivationstatusQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaMissionplanexecutionstatusQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaPlanningfunctionstatusQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaTaskstatusQos = {PCL_QOS_RELIABILITY_RELIABLE};
inline constexpr pcl_qos_t kTopicMaApprovalpolicyQos = {PCL_QOS_RELIABILITY_RELIABLE};

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    MaActionRead,
    MaMissionplanRead,
    MaPlanningfunctionRead,
    MaResponseRead,
    MaTaskRead,
    MissioncontingencyalertRead,
    MaActionstatusRead,
    MaMissionplanactivationstatusRead,
    MaMissionplanexecutionstatusRead,
    MaPlanningfunctionstatusRead,
    MaTaskstatusRead,
    MaApprovalpolicyRead,
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


class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // MA_Action_Service
    virtual std::vector<MA_Action_Service_Information>
    handleMaActionRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaActionRead(const Empty& request,
                       pcl_stream_context_t* stream_context,
                       const char* content_type);

    // MA_MissionPlan_Service
    virtual std::vector<MA_MissionPlan_Service_Information>
    handleMaMissionplanRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaMissionplanRead(const Empty& request,
                            pcl_stream_context_t* stream_context,
                            const char* content_type);

    // MA_PlanningFunction_Service
    virtual std::vector<MA_PlanningFunction_Service_Information>
    handleMaPlanningfunctionRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaPlanningfunctionRead(const Empty& request,
                                 pcl_stream_context_t* stream_context,
                                 const char* content_type);

    // MA_Response_Service
    virtual std::vector<MA_Response_Service_Information>
    handleMaResponseRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaResponseRead(const Empty& request,
                         pcl_stream_context_t* stream_context,
                         const char* content_type);

    // MA_Task_Service
    virtual std::vector<MA_Task_Service_Information>
    handleMaTaskRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaTaskRead(const Empty& request,
                     pcl_stream_context_t* stream_context,
                     const char* content_type);

    // MissionContingencyAlert_Service
    virtual std::vector<MissionContingencyAlert_Service_Information>
    handleMissioncontingencyalertRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMissioncontingencyalertRead(const Empty& request,
                                      pcl_stream_context_t* stream_context,
                                      const char* content_type);

    // MA_ActionStatus_Service
    virtual std::vector<MA_ActionStatus_Service_Information>
    handleMaActionstatusRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaActionstatusRead(const Empty& request,
                             pcl_stream_context_t* stream_context,
                             const char* content_type);

    // MA_MissionPlanActivationStatus_Service
    virtual std::vector<MA_MissionPlanActivationStatus_Service_Information>
    handleMaMissionplanactivationstatusRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaMissionplanactivationstatusRead(const Empty& request,
                                            pcl_stream_context_t* stream_context,
                                            const char* content_type);

    // MA_MissionPlanExecutionStatus_Service
    virtual std::vector<MA_MissionPlanExecutionStatus_Service_Information>
    handleMaMissionplanexecutionstatusRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaMissionplanexecutionstatusRead(const Empty& request,
                                           pcl_stream_context_t* stream_context,
                                           const char* content_type);

    // MA_PlanningFunctionStatus_Service
    virtual std::vector<MA_PlanningFunctionStatus_Service_Information>
    handleMaPlanningfunctionstatusRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaPlanningfunctionstatusRead(const Empty& request,
                                       pcl_stream_context_t* stream_context,
                                       const char* content_type);

    // MA_TaskStatus_Service
    virtual std::vector<MA_TaskStatus_Service_Information>
    handleMaTaskstatusRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaTaskstatusRead(const Empty& request,
                           pcl_stream_context_t* stream_context,
                           const char* content_type);

    // MA_ApprovalPolicy_Service
    virtual std::vector<MA_ApprovalPolicy_Service_Information>
    handleMaApprovalpolicyRead(const Empty& request);

    /// \brief Begin an asynchronous stream for this RPC.
    ///
    /// Override this for true server streaming. Store stream_context,
    /// return PCL_STREAMING, then emit frames with send*StreamFrame().
    virtual pcl_status_t
    streamMaApprovalpolicyRead(const Empty& request,
                               pcl_stream_context_t* stream_context,
                               const char* content_type);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish / Invoke (typed)
// ---------------------------------------------------------------------------

/// \brief Subscribe to MA-Action publications on kTopicMaAction.
pcl_port_t* subscribeMaAction(pcl_container_t*  container,
                              pcl_sub_callback_t callback,
                              void*             user_data = nullptr,
                              const char*       content_type = "application/json");

/// \brief Subscribe to MA-MissionPlan publications on kTopicMaMissionplan.
pcl_port_t* subscribeMaMissionplan(pcl_container_t*  container,
                                   pcl_sub_callback_t callback,
                                   void*             user_data = nullptr,
                                   const char*       content_type = "application/json");

/// \brief Subscribe to MA-PlanningFunction publications on
///        kTopicMaPlanningfunction.
pcl_port_t* subscribeMaPlanningfunction(pcl_container_t*  container,
                                        pcl_sub_callback_t callback,
                                        void*             user_data = nullptr,
                                        const char*       content_type = "application/json");

/// \brief Subscribe to MA-Response publications on kTopicMaResponse.
pcl_port_t* subscribeMaResponse(pcl_container_t*  container,
                                pcl_sub_callback_t callback,
                                void*             user_data = nullptr,
                                const char*       content_type = "application/json");

/// \brief Subscribe to MA-Task publications on kTopicMaTask.
pcl_port_t* subscribeMaTask(pcl_container_t*  container,
                            pcl_sub_callback_t callback,
                            void*             user_data = nullptr,
                            const char*       content_type = "application/json");

/// \brief Subscribe to MissionContingencyAlert publications on
///        kTopicMissioncontingencyalert.
pcl_port_t* subscribeMissioncontingencyalert(pcl_container_t*  container,
                                             pcl_sub_callback_t callback,
                                             void*             user_data = nullptr,
                                             const char*       content_type = "application/json");

/// \brief Subscribe to MA-ActionStatu publications on kTopicMaActionstatus.
pcl_port_t* subscribeMaActionstatus(pcl_container_t*  container,
                                    pcl_sub_callback_t callback,
                                    void*             user_data = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Subscribe to MA-MissionPlanActivationStatu publications on
///        kTopicMaMissionplanactivationstatus.
pcl_port_t* subscribeMaMissionplanactivationstatus(pcl_container_t*  container,
                                                   pcl_sub_callback_t callback,
                                                   void*             user_data = nullptr,
                                                   const char*       content_type = "application/json");

/// \brief Subscribe to MA-MissionPlanExecutionStatu publications on
///        kTopicMaMissionplanexecutionstatus.
pcl_port_t* subscribeMaMissionplanexecutionstatus(pcl_container_t*  container,
                                                  pcl_sub_callback_t callback,
                                                  void*             user_data = nullptr,
                                                  const char*       content_type = "application/json");

/// \brief Subscribe to MA-PlanningFunctionStatu publications on
///        kTopicMaPlanningfunctionstatus.
pcl_port_t* subscribeMaPlanningfunctionstatus(pcl_container_t*  container,
                                              pcl_sub_callback_t callback,
                                              void*             user_data = nullptr,
                                              const char*       content_type = "application/json");

/// \brief Subscribe to MA-TaskStatu publications on kTopicMaTaskstatus.
pcl_port_t* subscribeMaTaskstatus(pcl_container_t*  container,
                                  pcl_sub_callback_t callback,
                                  void*             user_data = nullptr,
                                  const char*       content_type = "application/json");

/// \brief Subscribe to MA-ApprovalPolicy publications on
///        kTopicMaApprovalpolicy.
pcl_port_t* subscribeMaApprovalpolicy(pcl_container_t*  container,
                                      pcl_sub_callback_t callback,
                                      void*             user_data = nullptr,
                                      const char*       content_type = "application/json");

/// \brief Publish a typed message on kTopicMaAction.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaAction, obtained during on_configure.
pcl_status_t publishMaAction(pcl_port_t*        publisher,
                             const MA_Action_Service_Information& payload,
                             const char*        content_type = "application/json");

pcl_status_t publishMaAction(pcl_port_t*        publisher,
                             const std::string& payload,
                             const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaAction.
bool encodeMaAction(const MA_Action_Service_Information& payload,
                    const char*        content_type,
                    std::string*       out);

/// \brief Decode a PCL message from kTopicMaAction.
bool decodeMaAction(const pcl_msg_t* msg,
                    MA_Action_Service_Information* out);

/// \brief Publish a typed message on kTopicMaMissionplan.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaMissionplan, obtained during on_configure.
pcl_status_t publishMaMissionplan(pcl_port_t*        publisher,
                                  const MA_MissionPlan_Service_Information& payload,
                                  const char*        content_type = "application/json");

pcl_status_t publishMaMissionplan(pcl_port_t*        publisher,
                                  const std::string& payload,
                                  const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaMissionplan.
bool encodeMaMissionplan(const MA_MissionPlan_Service_Information& payload,
                         const char*        content_type,
                         std::string*       out);

/// \brief Decode a PCL message from kTopicMaMissionplan.
bool decodeMaMissionplan(const pcl_msg_t* msg,
                         MA_MissionPlan_Service_Information* out);

/// \brief Publish a typed message on kTopicMaPlanningfunction.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaPlanningfunction, obtained during on_configure.
pcl_status_t publishMaPlanningfunction(pcl_port_t*        publisher,
                                       const MA_PlanningFunction_Service_Information& payload,
                                       const char*        content_type = "application/json");

pcl_status_t publishMaPlanningfunction(pcl_port_t*        publisher,
                                       const std::string& payload,
                                       const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaPlanningfunction.
bool encodeMaPlanningfunction(const MA_PlanningFunction_Service_Information& payload,
                              const char*        content_type,
                              std::string*       out);

/// \brief Decode a PCL message from kTopicMaPlanningfunction.
bool decodeMaPlanningfunction(const pcl_msg_t* msg,
                              MA_PlanningFunction_Service_Information* out);

/// \brief Publish a typed message on kTopicMaResponse.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaResponse, obtained during on_configure.
pcl_status_t publishMaResponse(pcl_port_t*        publisher,
                               const MA_Response_Service_Information& payload,
                               const char*        content_type = "application/json");

pcl_status_t publishMaResponse(pcl_port_t*        publisher,
                               const std::string& payload,
                               const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaResponse.
bool encodeMaResponse(const MA_Response_Service_Information& payload,
                      const char*        content_type,
                      std::string*       out);

/// \brief Decode a PCL message from kTopicMaResponse.
bool decodeMaResponse(const pcl_msg_t* msg,
                      MA_Response_Service_Information* out);

/// \brief Publish a typed message on kTopicMaTask.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaTask, obtained during on_configure.
pcl_status_t publishMaTask(pcl_port_t*        publisher,
                           const MA_Task_Service_Information& payload,
                           const char*        content_type = "application/json");

pcl_status_t publishMaTask(pcl_port_t*        publisher,
                           const std::string& payload,
                           const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaTask.
bool encodeMaTask(const MA_Task_Service_Information& payload,
                  const char*        content_type,
                  std::string*       out);

/// \brief Decode a PCL message from kTopicMaTask.
bool decodeMaTask(const pcl_msg_t* msg,
                  MA_Task_Service_Information* out);

/// \brief Publish a typed message on kTopicMissioncontingencyalert.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMissioncontingencyalert, obtained during on_configure.
pcl_status_t publishMissioncontingencyalert(pcl_port_t*        publisher,
                                            const MissionContingencyAlert_Service_Information& payload,
                                            const char*        content_type = "application/json");

pcl_status_t publishMissioncontingencyalert(pcl_port_t*        publisher,
                                            const std::string& payload,
                                            const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMissioncontingencyalert.
bool encodeMissioncontingencyalert(const MissionContingencyAlert_Service_Information& payload,
                                   const char*        content_type,
                                   std::string*       out);

/// \brief Decode a PCL message from kTopicMissioncontingencyalert.
bool decodeMissioncontingencyalert(const pcl_msg_t* msg,
                                   MissionContingencyAlert_Service_Information* out);

/// \brief Publish a typed message on kTopicMaActionstatus.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaActionstatus, obtained during on_configure.
pcl_status_t publishMaActionstatus(pcl_port_t*        publisher,
                                   const MA_ActionStatus_Service_Information& payload,
                                   const char*        content_type = "application/json");

pcl_status_t publishMaActionstatus(pcl_port_t*        publisher,
                                   const std::string& payload,
                                   const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaActionstatus.
bool encodeMaActionstatus(const MA_ActionStatus_Service_Information& payload,
                          const char*        content_type,
                          std::string*       out);

/// \brief Decode a PCL message from kTopicMaActionstatus.
bool decodeMaActionstatus(const pcl_msg_t* msg,
                          MA_ActionStatus_Service_Information* out);

/// \brief Publish a typed message on kTopicMaMissionplanactivationstatus.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaMissionplanactivationstatus, obtained during on_configure.
pcl_status_t publishMaMissionplanactivationstatus(pcl_port_t*        publisher,
                                                  const MA_MissionPlanActivationStatus_Service_Information& payload,
                                                  const char*        content_type = "application/json");

pcl_status_t publishMaMissionplanactivationstatus(pcl_port_t*        publisher,
                                                  const std::string& payload,
                                                  const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaMissionplanactivationstatus.
bool encodeMaMissionplanactivationstatus(const MA_MissionPlanActivationStatus_Service_Information& payload,
                                         const char*        content_type,
                                         std::string*       out);

/// \brief Decode a PCL message from kTopicMaMissionplanactivationstatus.
bool decodeMaMissionplanactivationstatus(const pcl_msg_t* msg,
                                         MA_MissionPlanActivationStatus_Service_Information* out);

/// \brief Publish a typed message on kTopicMaMissionplanexecutionstatus.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaMissionplanexecutionstatus, obtained during on_configure.
pcl_status_t publishMaMissionplanexecutionstatus(pcl_port_t*        publisher,
                                                 const MA_MissionPlanExecutionStatus_Service_Information& payload,
                                                 const char*        content_type = "application/json");

pcl_status_t publishMaMissionplanexecutionstatus(pcl_port_t*        publisher,
                                                 const std::string& payload,
                                                 const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaMissionplanexecutionstatus.
bool encodeMaMissionplanexecutionstatus(const MA_MissionPlanExecutionStatus_Service_Information& payload,
                                        const char*        content_type,
                                        std::string*       out);

/// \brief Decode a PCL message from kTopicMaMissionplanexecutionstatus.
bool decodeMaMissionplanexecutionstatus(const pcl_msg_t* msg,
                                        MA_MissionPlanExecutionStatus_Service_Information* out);

/// \brief Publish a typed message on kTopicMaPlanningfunctionstatus.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaPlanningfunctionstatus, obtained during on_configure.
pcl_status_t publishMaPlanningfunctionstatus(pcl_port_t*        publisher,
                                             const MA_PlanningFunctionStatus_Service_Information& payload,
                                             const char*        content_type = "application/json");

pcl_status_t publishMaPlanningfunctionstatus(pcl_port_t*        publisher,
                                             const std::string& payload,
                                             const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaPlanningfunctionstatus.
bool encodeMaPlanningfunctionstatus(const MA_PlanningFunctionStatus_Service_Information& payload,
                                    const char*        content_type,
                                    std::string*       out);

/// \brief Decode a PCL message from kTopicMaPlanningfunctionstatus.
bool decodeMaPlanningfunctionstatus(const pcl_msg_t* msg,
                                    MA_PlanningFunctionStatus_Service_Information* out);

/// \brief Publish a typed message on kTopicMaTaskstatus.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaTaskstatus, obtained during on_configure.
pcl_status_t publishMaTaskstatus(pcl_port_t*        publisher,
                                 const MA_TaskStatus_Service_Information& payload,
                                 const char*        content_type = "application/json");

pcl_status_t publishMaTaskstatus(pcl_port_t*        publisher,
                                 const std::string& payload,
                                 const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaTaskstatus.
bool encodeMaTaskstatus(const MA_TaskStatus_Service_Information& payload,
                        const char*        content_type,
                        std::string*       out);

/// \brief Decode a PCL message from kTopicMaTaskstatus.
bool decodeMaTaskstatus(const pcl_msg_t* msg,
                        MA_TaskStatus_Service_Information* out);

/// \brief Publish a typed message on kTopicMaApprovalpolicy.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicMaApprovalpolicy, obtained during on_configure.
pcl_status_t publishMaApprovalpolicy(pcl_port_t*        publisher,
                                     const MA_ApprovalPolicy_Service_Information& payload,
                                     const char*        content_type = "application/json");

pcl_status_t publishMaApprovalpolicy(pcl_port_t*        publisher,
                                     const std::string& payload,
                                     const char*        content_type = "application/json");

/// \brief Encode a typed message for kTopicMaApprovalpolicy.
bool encodeMaApprovalpolicy(const MA_ApprovalPolicy_Service_Information& payload,
                            const char*        content_type,
                            std::string*       out);

/// \brief Decode a PCL message from kTopicMaApprovalpolicy.
bool decodeMaApprovalpolicy(const pcl_msg_t* msg,
                            MA_ApprovalPolicy_Service_Information* out);

/// \brief Decode a response from ma_action.read.
bool decodeMaActionReadResponse(const pcl_msg_t* msg,
                                std::vector<MA_Action_Service_Information>* out);

/// \brief Encode one stream frame for ma_action.read.
bool encodeMaActionReadStreamFrame(const MA_Action_Service_Information& payload,
                                   const char*        content_type,
                                   std::string*       out);

/// \brief Decode one stream frame from ma_action.read.
bool decodeMaActionReadStreamFrame(const pcl_msg_t* msg,
                                   MA_Action_Service_Information* out);

/// \brief Send one typed stream frame for ma_action.read.
pcl_status_t sendMaActionReadStreamFrame(pcl_stream_context_t* stream_context,
                                         const MA_Action_Service_Information& payload,
                                         const char*        content_type = "application/json");

/// \brief Invoke ma_action.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaActionRead(pcl_executor_t* executor,
                                const Empty&                 request,
                                pcl_resp_cb_fn_t        callback,
                                void*                   user_data = nullptr,
                                const pcl_endpoint_route_t* route = nullptr,
                                const char*       content_type = "application/json");

/// \brief Invoke ma_action.read and ignore the async response.
pcl_status_t invokeMaActionRead(pcl_executor_t* executor,
                                const Empty&                 request,
                                const char*       content_type = "application/json",
                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_action.read as an asynchronous stream.
pcl_status_t invokeMaActionReadStream(pcl_executor_t* executor,
                                      const Empty&                 request,
                                      pcl_stream_msg_fn_t   callback,
                                      void*                   user_data = nullptr,
                                      pcl_stream_context_t** out_context = nullptr,
                                      const pcl_endpoint_route_t* route = nullptr,
                                      const char*       content_type = "application/json");

/// \brief Decode a response from ma_mission_plan.read.
bool decodeMaMissionplanReadResponse(const pcl_msg_t* msg,
                                     std::vector<MA_MissionPlan_Service_Information>* out);

/// \brief Encode one stream frame for ma_mission_plan.read.
bool encodeMaMissionplanReadStreamFrame(const MA_MissionPlan_Service_Information& payload,
                                        const char*        content_type,
                                        std::string*       out);

/// \brief Decode one stream frame from ma_mission_plan.read.
bool decodeMaMissionplanReadStreamFrame(const pcl_msg_t* msg,
                                        MA_MissionPlan_Service_Information* out);

/// \brief Send one typed stream frame for ma_mission_plan.read.
pcl_status_t sendMaMissionplanReadStreamFrame(pcl_stream_context_t* stream_context,
                                              const MA_MissionPlan_Service_Information& payload,
                                              const char*        content_type = "application/json");

/// \brief Invoke ma_mission_plan.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaMissionplanRead(pcl_executor_t* executor,
                                     const Empty&                 request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data = nullptr,
                                     const pcl_endpoint_route_t* route = nullptr,
                                     const char*       content_type = "application/json");

/// \brief Invoke ma_mission_plan.read and ignore the async response.
pcl_status_t invokeMaMissionplanRead(pcl_executor_t* executor,
                                     const Empty&                 request,
                                     const char*       content_type = "application/json",
                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_mission_plan.read as an asynchronous stream.
pcl_status_t invokeMaMissionplanReadStream(pcl_executor_t* executor,
                                           const Empty&                 request,
                                           pcl_stream_msg_fn_t   callback,
                                           void*                   user_data = nullptr,
                                           pcl_stream_context_t** out_context = nullptr,
                                           const pcl_endpoint_route_t* route = nullptr,
                                           const char*       content_type = "application/json");

/// \brief Decode a response from ma_planning_function.read.
bool decodeMaPlanningfunctionReadResponse(const pcl_msg_t* msg,
                                          std::vector<MA_PlanningFunction_Service_Information>* out);

/// \brief Encode one stream frame for ma_planning_function.read.
bool encodeMaPlanningfunctionReadStreamFrame(const MA_PlanningFunction_Service_Information& payload,
                                             const char*        content_type,
                                             std::string*       out);

/// \brief Decode one stream frame from ma_planning_function.read.
bool decodeMaPlanningfunctionReadStreamFrame(const pcl_msg_t* msg,
                                             MA_PlanningFunction_Service_Information* out);

/// \brief Send one typed stream frame for ma_planning_function.read.
pcl_status_t sendMaPlanningfunctionReadStreamFrame(pcl_stream_context_t* stream_context,
                                                   const MA_PlanningFunction_Service_Information& payload,
                                                   const char*        content_type = "application/json");

/// \brief Invoke ma_planning_function.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaPlanningfunctionRead(pcl_executor_t* executor,
                                          const Empty&                 request,
                                          pcl_resp_cb_fn_t        callback,
                                          void*                   user_data = nullptr,
                                          const pcl_endpoint_route_t* route = nullptr,
                                          const char*       content_type = "application/json");

/// \brief Invoke ma_planning_function.read and ignore the async response.
pcl_status_t invokeMaPlanningfunctionRead(pcl_executor_t* executor,
                                          const Empty&                 request,
                                          const char*       content_type = "application/json",
                                          const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_planning_function.read as an asynchronous stream.
pcl_status_t invokeMaPlanningfunctionReadStream(pcl_executor_t* executor,
                                                const Empty&                 request,
                                                pcl_stream_msg_fn_t   callback,
                                                void*                   user_data = nullptr,
                                                pcl_stream_context_t** out_context = nullptr,
                                                const pcl_endpoint_route_t* route = nullptr,
                                                const char*       content_type = "application/json");

/// \brief Decode a response from ma_response.read.
bool decodeMaResponseReadResponse(const pcl_msg_t* msg,
                                  std::vector<MA_Response_Service_Information>* out);

/// \brief Encode one stream frame for ma_response.read.
bool encodeMaResponseReadStreamFrame(const MA_Response_Service_Information& payload,
                                     const char*        content_type,
                                     std::string*       out);

/// \brief Decode one stream frame from ma_response.read.
bool decodeMaResponseReadStreamFrame(const pcl_msg_t* msg,
                                     MA_Response_Service_Information* out);

/// \brief Send one typed stream frame for ma_response.read.
pcl_status_t sendMaResponseReadStreamFrame(pcl_stream_context_t* stream_context,
                                           const MA_Response_Service_Information& payload,
                                           const char*        content_type = "application/json");

/// \brief Invoke ma_response.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaResponseRead(pcl_executor_t* executor,
                                  const Empty&                 request,
                                  pcl_resp_cb_fn_t        callback,
                                  void*                   user_data = nullptr,
                                  const pcl_endpoint_route_t* route = nullptr,
                                  const char*       content_type = "application/json");

/// \brief Invoke ma_response.read and ignore the async response.
pcl_status_t invokeMaResponseRead(pcl_executor_t* executor,
                                  const Empty&                 request,
                                  const char*       content_type = "application/json",
                                  const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_response.read as an asynchronous stream.
pcl_status_t invokeMaResponseReadStream(pcl_executor_t* executor,
                                        const Empty&                 request,
                                        pcl_stream_msg_fn_t   callback,
                                        void*                   user_data = nullptr,
                                        pcl_stream_context_t** out_context = nullptr,
                                        const pcl_endpoint_route_t* route = nullptr,
                                        const char*       content_type = "application/json");

/// \brief Decode a response from ma_task.read.
bool decodeMaTaskReadResponse(const pcl_msg_t* msg,
                              std::vector<MA_Task_Service_Information>* out);

/// \brief Encode one stream frame for ma_task.read.
bool encodeMaTaskReadStreamFrame(const MA_Task_Service_Information& payload,
                                 const char*        content_type,
                                 std::string*       out);

/// \brief Decode one stream frame from ma_task.read.
bool decodeMaTaskReadStreamFrame(const pcl_msg_t* msg,
                                 MA_Task_Service_Information* out);

/// \brief Send one typed stream frame for ma_task.read.
pcl_status_t sendMaTaskReadStreamFrame(pcl_stream_context_t* stream_context,
                                       const MA_Task_Service_Information& payload,
                                       const char*        content_type = "application/json");

/// \brief Invoke ma_task.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaTaskRead(pcl_executor_t* executor,
                              const Empty&                 request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data = nullptr,
                              const pcl_endpoint_route_t* route = nullptr,
                              const char*       content_type = "application/json");

/// \brief Invoke ma_task.read and ignore the async response.
pcl_status_t invokeMaTaskRead(pcl_executor_t* executor,
                              const Empty&                 request,
                              const char*       content_type = "application/json",
                              const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_task.read as an asynchronous stream.
pcl_status_t invokeMaTaskReadStream(pcl_executor_t* executor,
                                    const Empty&                 request,
                                    pcl_stream_msg_fn_t   callback,
                                    void*                   user_data = nullptr,
                                    pcl_stream_context_t** out_context = nullptr,
                                    const pcl_endpoint_route_t* route = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Decode a response from mission_contingency_alert.read.
bool decodeMissioncontingencyalertReadResponse(const pcl_msg_t* msg,
                                               std::vector<MissionContingencyAlert_Service_Information>* out);

/// \brief Encode one stream frame for mission_contingency_alert.read.
bool encodeMissioncontingencyalertReadStreamFrame(const MissionContingencyAlert_Service_Information& payload,
                                                  const char*        content_type,
                                                  std::string*       out);

/// \brief Decode one stream frame from mission_contingency_alert.read.
bool decodeMissioncontingencyalertReadStreamFrame(const pcl_msg_t* msg,
                                                  MissionContingencyAlert_Service_Information* out);

/// \brief Send one typed stream frame for mission_contingency_alert.read.
pcl_status_t sendMissioncontingencyalertReadStreamFrame(pcl_stream_context_t* stream_context,
                                                        const MissionContingencyAlert_Service_Information& payload,
                                                        const char*        content_type = "application/json");

/// \brief Invoke mission_contingency_alert.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMissioncontingencyalertRead(pcl_executor_t* executor,
                                               const Empty&                 request,
                                               pcl_resp_cb_fn_t        callback,
                                               void*                   user_data = nullptr,
                                               const pcl_endpoint_route_t* route = nullptr,
                                               const char*       content_type = "application/json");

/// \brief Invoke mission_contingency_alert.read and ignore the async response.
pcl_status_t invokeMissioncontingencyalertRead(pcl_executor_t* executor,
                                               const Empty&                 request,
                                               const char*       content_type = "application/json",
                                               const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke mission_contingency_alert.read as an asynchronous stream.
pcl_status_t invokeMissioncontingencyalertReadStream(pcl_executor_t* executor,
                                                     const Empty&                 request,
                                                     pcl_stream_msg_fn_t   callback,
                                                     void*                   user_data = nullptr,
                                                     pcl_stream_context_t** out_context = nullptr,
                                                     const pcl_endpoint_route_t* route = nullptr,
                                                     const char*       content_type = "application/json");

/// \brief Decode a response from ma_action_status.read.
bool decodeMaActionstatusReadResponse(const pcl_msg_t* msg,
                                      std::vector<MA_ActionStatus_Service_Information>* out);

/// \brief Encode one stream frame for ma_action_status.read.
bool encodeMaActionstatusReadStreamFrame(const MA_ActionStatus_Service_Information& payload,
                                         const char*        content_type,
                                         std::string*       out);

/// \brief Decode one stream frame from ma_action_status.read.
bool decodeMaActionstatusReadStreamFrame(const pcl_msg_t* msg,
                                         MA_ActionStatus_Service_Information* out);

/// \brief Send one typed stream frame for ma_action_status.read.
pcl_status_t sendMaActionstatusReadStreamFrame(pcl_stream_context_t* stream_context,
                                               const MA_ActionStatus_Service_Information& payload,
                                               const char*        content_type = "application/json");

/// \brief Invoke ma_action_status.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaActionstatusRead(pcl_executor_t* executor,
                                      const Empty&                 request,
                                      pcl_resp_cb_fn_t        callback,
                                      void*                   user_data = nullptr,
                                      const pcl_endpoint_route_t* route = nullptr,
                                      const char*       content_type = "application/json");

/// \brief Invoke ma_action_status.read and ignore the async response.
pcl_status_t invokeMaActionstatusRead(pcl_executor_t* executor,
                                      const Empty&                 request,
                                      const char*       content_type = "application/json",
                                      const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_action_status.read as an asynchronous stream.
pcl_status_t invokeMaActionstatusReadStream(pcl_executor_t* executor,
                                            const Empty&                 request,
                                            pcl_stream_msg_fn_t   callback,
                                            void*                   user_data = nullptr,
                                            pcl_stream_context_t** out_context = nullptr,
                                            const pcl_endpoint_route_t* route = nullptr,
                                            const char*       content_type = "application/json");

/// \brief Decode a response from ma_mission_plan_activation_status.read.
bool decodeMaMissionplanactivationstatusReadResponse(const pcl_msg_t* msg,
                                                     std::vector<MA_MissionPlanActivationStatus_Service_Information>* out);

/// \brief Encode one stream frame for ma_mission_plan_activation_status.read.
bool encodeMaMissionplanactivationstatusReadStreamFrame(const MA_MissionPlanActivationStatus_Service_Information& payload,
                                                        const char*        content_type,
                                                        std::string*       out);

/// \brief Decode one stream frame from ma_mission_plan_activation_status.read.
bool decodeMaMissionplanactivationstatusReadStreamFrame(const pcl_msg_t* msg,
                                                        MA_MissionPlanActivationStatus_Service_Information* out);

/// \brief Send one typed stream frame for ma_mission_plan_activation_status.read.
pcl_status_t sendMaMissionplanactivationstatusReadStreamFrame(pcl_stream_context_t* stream_context,
                                                              const MA_MissionPlanActivationStatus_Service_Information& payload,
                                                              const char*        content_type = "application/json");

/// \brief Invoke ma_mission_plan_activation_status.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaMissionplanactivationstatusRead(pcl_executor_t* executor,
                                                     const Empty&                 request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data = nullptr,
                                                     const pcl_endpoint_route_t* route = nullptr,
                                                     const char*       content_type = "application/json");

/// \brief Invoke ma_mission_plan_activation_status.read and ignore the async response.
pcl_status_t invokeMaMissionplanactivationstatusRead(pcl_executor_t* executor,
                                                     const Empty&                 request,
                                                     const char*       content_type = "application/json",
                                                     const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_mission_plan_activation_status.read as an asynchronous stream.
pcl_status_t invokeMaMissionplanactivationstatusReadStream(pcl_executor_t* executor,
                                                           const Empty&                 request,
                                                           pcl_stream_msg_fn_t   callback,
                                                           void*                   user_data = nullptr,
                                                           pcl_stream_context_t** out_context = nullptr,
                                                           const pcl_endpoint_route_t* route = nullptr,
                                                           const char*       content_type = "application/json");

/// \brief Decode a response from ma_mission_plan_execution_status.read.
bool decodeMaMissionplanexecutionstatusReadResponse(const pcl_msg_t* msg,
                                                    std::vector<MA_MissionPlanExecutionStatus_Service_Information>* out);

/// \brief Encode one stream frame for ma_mission_plan_execution_status.read.
bool encodeMaMissionplanexecutionstatusReadStreamFrame(const MA_MissionPlanExecutionStatus_Service_Information& payload,
                                                       const char*        content_type,
                                                       std::string*       out);

/// \brief Decode one stream frame from ma_mission_plan_execution_status.read.
bool decodeMaMissionplanexecutionstatusReadStreamFrame(const pcl_msg_t* msg,
                                                       MA_MissionPlanExecutionStatus_Service_Information* out);

/// \brief Send one typed stream frame for ma_mission_plan_execution_status.read.
pcl_status_t sendMaMissionplanexecutionstatusReadStreamFrame(pcl_stream_context_t* stream_context,
                                                             const MA_MissionPlanExecutionStatus_Service_Information& payload,
                                                             const char*        content_type = "application/json");

/// \brief Invoke ma_mission_plan_execution_status.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaMissionplanexecutionstatusRead(pcl_executor_t* executor,
                                                    const Empty&                 request,
                                                    pcl_resp_cb_fn_t        callback,
                                                    void*                   user_data = nullptr,
                                                    const pcl_endpoint_route_t* route = nullptr,
                                                    const char*       content_type = "application/json");

/// \brief Invoke ma_mission_plan_execution_status.read and ignore the async response.
pcl_status_t invokeMaMissionplanexecutionstatusRead(pcl_executor_t* executor,
                                                    const Empty&                 request,
                                                    const char*       content_type = "application/json",
                                                    const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_mission_plan_execution_status.read as an asynchronous stream.
pcl_status_t invokeMaMissionplanexecutionstatusReadStream(pcl_executor_t* executor,
                                                          const Empty&                 request,
                                                          pcl_stream_msg_fn_t   callback,
                                                          void*                   user_data = nullptr,
                                                          pcl_stream_context_t** out_context = nullptr,
                                                          const pcl_endpoint_route_t* route = nullptr,
                                                          const char*       content_type = "application/json");

/// \brief Decode a response from ma_planning_function_status.read.
bool decodeMaPlanningfunctionstatusReadResponse(const pcl_msg_t* msg,
                                                std::vector<MA_PlanningFunctionStatus_Service_Information>* out);

/// \brief Encode one stream frame for ma_planning_function_status.read.
bool encodeMaPlanningfunctionstatusReadStreamFrame(const MA_PlanningFunctionStatus_Service_Information& payload,
                                                   const char*        content_type,
                                                   std::string*       out);

/// \brief Decode one stream frame from ma_planning_function_status.read.
bool decodeMaPlanningfunctionstatusReadStreamFrame(const pcl_msg_t* msg,
                                                   MA_PlanningFunctionStatus_Service_Information* out);

/// \brief Send one typed stream frame for ma_planning_function_status.read.
pcl_status_t sendMaPlanningfunctionstatusReadStreamFrame(pcl_stream_context_t* stream_context,
                                                         const MA_PlanningFunctionStatus_Service_Information& payload,
                                                         const char*        content_type = "application/json");

/// \brief Invoke ma_planning_function_status.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaPlanningfunctionstatusRead(pcl_executor_t* executor,
                                                const Empty&                 request,
                                                pcl_resp_cb_fn_t        callback,
                                                void*                   user_data = nullptr,
                                                const pcl_endpoint_route_t* route = nullptr,
                                                const char*       content_type = "application/json");

/// \brief Invoke ma_planning_function_status.read and ignore the async response.
pcl_status_t invokeMaPlanningfunctionstatusRead(pcl_executor_t* executor,
                                                const Empty&                 request,
                                                const char*       content_type = "application/json",
                                                const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_planning_function_status.read as an asynchronous stream.
pcl_status_t invokeMaPlanningfunctionstatusReadStream(pcl_executor_t* executor,
                                                      const Empty&                 request,
                                                      pcl_stream_msg_fn_t   callback,
                                                      void*                   user_data = nullptr,
                                                      pcl_stream_context_t** out_context = nullptr,
                                                      const pcl_endpoint_route_t* route = nullptr,
                                                      const char*       content_type = "application/json");

/// \brief Decode a response from ma_task_status.read.
bool decodeMaTaskstatusReadResponse(const pcl_msg_t* msg,
                                    std::vector<MA_TaskStatus_Service_Information>* out);

/// \brief Encode one stream frame for ma_task_status.read.
bool encodeMaTaskstatusReadStreamFrame(const MA_TaskStatus_Service_Information& payload,
                                       const char*        content_type,
                                       std::string*       out);

/// \brief Decode one stream frame from ma_task_status.read.
bool decodeMaTaskstatusReadStreamFrame(const pcl_msg_t* msg,
                                       MA_TaskStatus_Service_Information* out);

/// \brief Send one typed stream frame for ma_task_status.read.
pcl_status_t sendMaTaskstatusReadStreamFrame(pcl_stream_context_t* stream_context,
                                             const MA_TaskStatus_Service_Information& payload,
                                             const char*        content_type = "application/json");

/// \brief Invoke ma_task_status.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaTaskstatusRead(pcl_executor_t* executor,
                                    const Empty&                 request,
                                    pcl_resp_cb_fn_t        callback,
                                    void*                   user_data = nullptr,
                                    const pcl_endpoint_route_t* route = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Invoke ma_task_status.read and ignore the async response.
pcl_status_t invokeMaTaskstatusRead(pcl_executor_t* executor,
                                    const Empty&                 request,
                                    const char*       content_type = "application/json",
                                    const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_task_status.read as an asynchronous stream.
pcl_status_t invokeMaTaskstatusReadStream(pcl_executor_t* executor,
                                          const Empty&                 request,
                                          pcl_stream_msg_fn_t   callback,
                                          void*                   user_data = nullptr,
                                          pcl_stream_context_t** out_context = nullptr,
                                          const pcl_endpoint_route_t* route = nullptr,
                                          const char*       content_type = "application/json");

/// \brief Decode a response from ma_approval_policy.read.
bool decodeMaApprovalpolicyReadResponse(const pcl_msg_t* msg,
                                        std::vector<MA_ApprovalPolicy_Service_Information>* out);

/// \brief Encode one stream frame for ma_approval_policy.read.
bool encodeMaApprovalpolicyReadStreamFrame(const MA_ApprovalPolicy_Service_Information& payload,
                                           const char*        content_type,
                                           std::string*       out);

/// \brief Decode one stream frame from ma_approval_policy.read.
bool decodeMaApprovalpolicyReadStreamFrame(const pcl_msg_t* msg,
                                           MA_ApprovalPolicy_Service_Information* out);

/// \brief Send one typed stream frame for ma_approval_policy.read.
pcl_status_t sendMaApprovalpolicyReadStreamFrame(pcl_stream_context_t* stream_context,
                                                 const MA_ApprovalPolicy_Service_Information& payload,
                                                 const char*        content_type = "application/json");

/// \brief Invoke ma_approval_policy.read (typed, serialisation handled internally).
///
/// Uses the configured endpoint route, or the legacy
/// executor transport fallback when no route is supplied.
pcl_status_t invokeMaApprovalpolicyRead(pcl_executor_t* executor,
                                        const Empty&                 request,
                                        pcl_resp_cb_fn_t        callback,
                                        void*                   user_data = nullptr,
                                        const pcl_endpoint_route_t* route = nullptr,
                                        const char*       content_type = "application/json");

/// \brief Invoke ma_approval_policy.read and ignore the async response.
pcl_status_t invokeMaApprovalpolicyRead(pcl_executor_t* executor,
                                        const Empty&                 request,
                                        const char*       content_type = "application/json",
                                        const pcl_endpoint_route_t* route = nullptr);

/// \brief Invoke ma_approval_policy.read as an asynchronous stream.
pcl_status_t invokeMaApprovalpolicyReadStream(pcl_executor_t* executor,
                                              const Empty&                 request,
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

} // namespace pyramid::components::agra::information::services::provided

// Legacy service namespace alias for existing callers.
namespace pyramid::services::agra::information {
namespace provided = ::pyramid::components::agra::information::services::provided;
} // namespace pyramid::services::agra::information
