#pragma once

#include "ame/pyramid_service.h"

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace ame {

// InvokeService: BT node that maps a PDDL action to a PYRAMID SDK service call.
//
// Blackboard key: "pyramid_service" (IPyramidService*)
//
// Ports:
//   service_name  (input)  — PYRAMID service endpoint name
//   operation     (input)  — Operation / method within the service
//   request_json  (input)  — Optional JSON-encoded request fields (k=v pairs
//                            separated by ';', e.g. "target=sector_a;priority=1")
//   response_json (output) — Serialised response fields in same k=v format
//
// Returns SUCCESS when IPyramidService::call() returns true, FAILURE otherwise.
class InvokeService : public BT::SyncActionNode {
public:
    InvokeService(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    // Parse "k1=v1;k2=v2" into a ServiceMessage
    static ServiceMessage parseRequest(const std::string& encoded);

    // Serialise a ServiceMessage to "k1=v1;k2=v2"
    static std::string serializeResponse(const ServiceMessage& msg);
};

} // namespace ame
