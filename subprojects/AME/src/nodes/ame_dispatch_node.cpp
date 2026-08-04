#include "ame/bt_nodes/ame_dispatch_node.h"

#include "action_command_builder.h"
#include "ame/autonomy_backend.h"
#include "ame/execution_sink.h"

namespace ame {

AmeDispatchNode::AmeDispatchNode(const std::string& name,
                                 const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList AmeDispatchNode::providedPorts() {
  return nodes::actionParamPorts();
}

BT::NodeStatus AmeDispatchNode::onStart() {
  try {
    sink_ = config().blackboard->get<IExecutionSink*>("action_sink");
  } catch (const std::exception&) {
    sink_ = nullptr;
  }
  if (!sink_) {
    return BT::NodeStatus::FAILURE;
  }

  const std::string verb = registrationName();

  command_id_ = nodes::nextCommandId(verb);
  auto command = nodes::buildActionCommand(*this, verb, command_id_);

  ExecutionSubmission submission = sink_->submit(command);
  if (!submission.accepted) {
    return BT::NodeStatus::FAILURE;
  }

  // The command may complete synchronously (non-movement verbs); check now.
  return onRunning();
}

BT::NodeStatus AmeDispatchNode::onRunning() {
  if (!sink_) {
    return BT::NodeStatus::FAILURE;
  }
  if (sink_->isPending(command_id_)) {
    return BT::NodeStatus::RUNNING;
  }

  auto result = sink_->resultFor(command_id_);
  if (!result.has_value()) {
    // No terminal result recorded but no longer pending: lost result.
    return BT::NodeStatus::FAILURE;
  }
  switch (result->status) {
    case CommandStatus::SUCCEEDED:
      return BT::NodeStatus::SUCCESS;
    case CommandStatus::PENDING:
    case CommandStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

void AmeDispatchNode::onHalted() {
  if (sink_ && !command_id_.empty()) {
    sink_->cancel(command_id_);
  }
  command_id_.clear();
  sink_ = nullptr;
}

}  // namespace ame
