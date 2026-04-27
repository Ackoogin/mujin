#pragma once

#include "ame/pyramid_service.h"

#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <cstdint>
#include <string>

namespace ame {

/// \brief BT node that maps a PDDL action to an async PYRAMID SDK service call.
///
/// Inherits from BT::StatefulActionNode to support non-blocking async
/// execution: onStart() initiates the call, onRunning() polls for completion,
/// and onHalted() cancels the pending request.
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
///   onStart()   -- initiate async service call via IPyramidService::callAsync()
///   onRunning() -- poll result; return RUNNING while pending, SUCCESS/FAILURE on completion
///   onHalted()  -- cancel the pending async call
class InvokeService : public BT::StatefulActionNode {
public:
  InvokeService(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

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
