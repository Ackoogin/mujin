#pragma once

#include "ame/pyramid_service.h"
#include "ame/bt_nodes/planned_action_node.h"

#include <chrono>
#include <cstdint>
#include <string>

namespace ame {

/// \brief BT node that maps a PDDL action to an async PYRAMID SDK service call.
///
/// Uses the planned-action lifecycle for non-blocking execution:
/// onActionStart() initiates the call, onActionRunning() polls for completion,
/// and onActionHalted() cancels the pending request.
///
/// Blackboard key: "pyramid_service" (IPyramidService*)
///
/// Ports:
///   service_name  (input)  -- PYRAMID service endpoint name
///   operation     (input)  -- Operation / method within the service
///   timeout_ms    (input)  -- Timeout in milliseconds (0 = no timeout, default 5000)
///   request_json  (input)  -- Optional request fields (k=v pairs separated by ';',
///                            e.g. "target=sector_a;priority=1")
///   param_names   (input)  -- Optional semicolon-separated PDDL param names for
///                            auto-mapping (e.g. "?robot;?from;?to")
///   param_values  (input)  -- Optional semicolon-separated PDDL param values for
///                            auto-mapping (e.g. "uav1;base;sector_a")
///   response_json (output) -- Serialised response fields in k=v format
///
/// Lifecycle:
///   onActionStart()   -- initiate async service call via IPyramidService::callAsync()
///   onActionRunning() -- poll result while the request is pending
///   onActionHalted()  -- cancel the pending async call
class InvokeService : public PlannedActionNode {
public:
  InvokeService(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

private:
  /// \brief Parse "k1=v1;k2=v2" into a ServiceMessage.
  static ServiceMessage parseRequest(const std::string& encoded);

  /// \brief Serialise a ServiceMessage to "k1=v1;k2=v2".
  static std::string serializeResponse(const ServiceMessage& msg);

  /// \brief Merge PDDL param bindings into the request message.
  static void mergeParamBindings(const std::string& names,
                                 const std::string& values,
                                 ServiceMessage& msg);

  uint64_t request_id_ = 0;
  std::chrono::steady_clock::time_point deadline_;
  bool has_timeout_ = false;
  IPyramidService* service_ = nullptr;
};

} // namespace ame
