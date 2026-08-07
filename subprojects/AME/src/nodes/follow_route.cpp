#include "ame/bt_nodes/follow_route.h"

#include "ame/execution_sink.h"

#include <array>
#include <atomic>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace ame {

namespace {

constexpr const char* kFollowRouteVerb = "follow-route";

std::atomic<uint64_t> g_command_counter{0};

std::optional<double> parseDouble(const std::string& component) {
  try {
    size_t parsed = 0;
    const double value = std::stod(component, &parsed);
    if (parsed != component.size()) {
      return std::nullopt;
    }
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<std::array<double, 3>> parseWaypoint(const std::string& token) {
  std::istringstream wp_stream(token);
  std::string component;
  std::array<double, 3> values{};
  for (size_t i = 0; i < values.size(); ++i) {
    if (!std::getline(wp_stream, component, ',')) {
      return std::nullopt;
    }
    auto parsed = parseDouble(component);
    if (!parsed.has_value()) {
      return std::nullopt;
    }
    values[i] = *parsed;
  }
  if (std::getline(wp_stream, component, ',')) {
    return std::nullopt;
  }
  return values;
}

std::string extractWaypoints(const std::string& route_input) {
  if (route_input.empty() ||
      route_input.find('=') == std::string::npos) {
    return route_input;
  }

  std::istringstream kv_stream(route_input);
  std::string kv_token;
  while (std::getline(kv_stream, kv_token, ';')) {
    auto eq = kv_token.find('=');
    if (eq != std::string::npos && kv_token.substr(0, eq) == "waypoints") {
      return kv_token.substr(eq + 1);
    }
  }
  return "";
}

}  // namespace

FollowRoute::FollowRoute(const std::string& name,
                         const BT::NodeConfiguration& config)
    : PlannedActionNode(name, config) {}

BT::PortsList FollowRoute::providedPorts() {
  return withBasePorts({
      BT::InputPort<std::string>("agent"),
      BT::InputPort<std::string>("route", "", "Serialised waypoints"),
      BT::OutputPort<std::string>("progress"),
  });
}

BT::NodeStatus FollowRoute::onActionStart() {
  sink_ = nullptr;
  command_id_.clear();
  waypoints_.clear();
  current_wp_ = 0;

  try {
    sink_ = config().blackboard->get<IExecutionSink*>("action_sink");
  } catch (const std::exception&) {
    sink_ = nullptr;
  }
  if (!sink_) {
    return BT::NodeStatus::FAILURE;
  }


  auto agent = getInput<std::string>("agent");
  if (!agent || agent->empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // Parse route data. The route input may be either:
  //   (a) Raw waypoints: pipe-separated "lat,lon,alt" triples
  //   (b) InvokeService response: semicolon-separated k=v pairs where the
  //       "waypoints" key holds pipe-separated triples
  std::string route_str;
  getInput("route", route_str);

  route_str = extractWaypoints(route_str);

  if (!route_str.empty()) {
    std::istringstream stream(route_str);
    std::string token;
    while (std::getline(stream, token, '|')) {
      if (token.empty()) continue;
      auto wp = parseWaypoint(token);
      if (!wp.has_value()) {
        return BT::NodeStatus::FAILURE;
      }
      waypoints_.push_back(Waypoint{(*wp)[0], (*wp)[1], (*wp)[2]});
    }
  }

  if (waypoints_.empty()) {
    setOutput("progress", "0/0");
    return BT::NodeStatus::FAILURE;
  }

  ActionCommand command;
  command.action_name = kFollowRouteVerb;
  command.service_name = kFollowRouteVerb;
  command.operation = kFollowRouteVerb;
  command.request_fields["agent"] = *agent;
  command.request_fields["route"] = route_str;
  command.request_fields["waypoints"] = route_str;
  command.command_id =
      std::string(kFollowRouteVerb) + "#" +
      std::to_string(g_command_counter.fetch_add(1));

  std::ostringstream signature;
  signature << kFollowRouteVerb << "(" << *agent << "," << route_str << ")";
  command.signature = signature.str();

  const ExecutionSubmission submission = sink_->submit(command);
  if (!submission.accepted) {
    return BT::NodeStatus::FAILURE;
  }
  command_id_ = command.command_id;

  setOutput("progress", "0/" + std::to_string(waypoints_.size()));
  return onActionRunning();
}

BT::NodeStatus FollowRoute::onActionRunning() {
  if (!sink_ || command_id_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  if (sink_->isPending(command_id_)) {
    setOutput("progress",
              std::to_string(current_wp_) + "/" +
                  std::to_string(waypoints_.size()));
    return BT::NodeStatus::RUNNING;
  }

  auto result = sink_->resultFor(command_id_);
  if (!result.has_value()) {
    return BT::NodeStatus::FAILURE;
  }

  switch (result->status) {
    case CommandStatus::SUCCEEDED:
      current_wp_ = waypoints_.size();
      setOutput("progress",
                std::to_string(current_wp_) + "/" +
                    std::to_string(waypoints_.size()));
      return BT::NodeStatus::SUCCESS;
    case CommandStatus::PENDING:
    case CommandStatus::RUNNING:
      setOutput("progress",
                std::to_string(current_wp_) + "/" +
                    std::to_string(waypoints_.size()));
      return BT::NodeStatus::RUNNING;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

void FollowRoute::onActionHalted() {
  if (sink_ && !command_id_.empty()) {
    sink_->cancel(command_id_);
  }
  command_id_.clear();
  sink_ = nullptr;
  waypoints_.clear();
  current_wp_ = 0;
}

} // namespace ame
