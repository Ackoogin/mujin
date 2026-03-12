#include "ame/bt_nodes/invoke_service.h"

#include <sstream>

namespace ame {

InvokeService::InvokeService(const std::string& name,
                             const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList InvokeService::providedPorts() {
    return {
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<std::string>("operation"),
        BT::InputPort<std::string>("request_json", "", "k=v;k=v encoded request fields"),
        BT::OutputPort<std::string>("response_json"),
    };
}

BT::NodeStatus InvokeService::tick() {
    auto service_name = getInput<std::string>("service_name");
    auto operation    = getInput<std::string>("operation");

    if (!service_name || !operation) {
        return BT::NodeStatus::FAILURE;
    }

    std::string encoded;
    getInput("request_json", encoded);
    ServiceMessage request = parseRequest(encoded);

    auto* svc = config().blackboard->get<IPyramidService*>("pyramid_service");
    if (!svc) {
        return BT::NodeStatus::FAILURE;
    }

    ServiceMessage response;
    bool ok = svc->call(service_name.value(), operation.value(), request, response);

    if (ok) {
        setOutput("response_json", serializeResponse(response));
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

ServiceMessage InvokeService::parseRequest(const std::string& encoded) {
    ServiceMessage msg;
    if (encoded.empty()) return msg;

    std::istringstream stream(encoded);
    std::string token;
    while (std::getline(stream, token, ';')) {
        auto eq = token.find('=');
        if (eq != std::string::npos) {
            msg.set(token.substr(0, eq), token.substr(eq + 1));
        }
    }
    return msg;
}

std::string InvokeService::serializeResponse(const ServiceMessage& msg) {
    std::string result;
    for (auto& [k, v] : msg.fields) {
        if (!result.empty()) result += ";";
        result += k + "=" + v;
    }
    return result;
}

} // namespace ame
