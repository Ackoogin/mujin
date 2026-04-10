#include <ame_ros2/world_model_node.hpp>

#include <ame/world_model_component.h>

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>

namespace ame_ros2 {

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("world_model_node", options)
  , component_(std::make_unique<ame::WorldModelComponent>())
{}

WorldModelNode::~WorldModelNode() = default;

ame::WorldModel& WorldModelNode::worldModel()             { return component_->worldModel(); }
const ame::WorldModel& WorldModelNode::worldModel() const { return component_->worldModel(); }
ame::WorldModelComponent& WorldModelNode::component()     { return *component_; }

WorldModelNode::CallbackReturn
WorldModelNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("domain.pddl_file",               std::string(""));
  declare_parameter("domain.problem_file",             std::string(""));
  declare_parameter("audit_log.enabled",               true);
  declare_parameter("audit_log.path",                  std::string("wm_audit.jsonl"));
  declare_parameter("publish_rate_hz",                 10.0);
  declare_parameter("perception.enabled",              true);
  declare_parameter("perception.confidence_threshold", 0.5);

  component_->setParam("domain.pddl_file",
                      get_parameter("domain.pddl_file").as_string().c_str());
  component_->setParam("domain.problem_file",
                      get_parameter("domain.problem_file").as_string().c_str());
  component_->setParam("audit_log.enabled",
                      get_parameter("audit_log.enabled").as_bool());
  component_->setParam("audit_log.path",
                      get_parameter("audit_log.path").as_string().c_str());
  component_->setParam("publish_rate_hz",
                      get_parameter("publish_rate_hz").as_double());
  component_->setParam("perception.enabled",
                      get_parameter("perception.enabled").as_bool());
  component_->setParam("perception.confidence_threshold",
                      get_parameter("perception.confidence_threshold").as_double());

  if (component_->configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure WorldModelComponent");
    return CallbackReturn::FAILURE;
  }

  // -- ROS2 service servers (bridge for devenv / external ROS2 callers) -------
  svc_get_fact_ = create_service<ame_ros2::srv::GetFact>(
      "~/get_fact",
      [this](const ame_ros2::srv::GetFact::Request::SharedPtr req,
             ame_ros2::srv::GetFact::Response::SharedPtr resp) {
        RCLCPP_DEBUG(get_logger(), "get_fact: key='%s'", req->key.c_str());
        auto r = component_->getFact(req->key);
        resp->found      = r.found;
        resp->value      = r.value;
        resp->wm_version = r.wm_version;
        RCLCPP_DEBUG(get_logger(), "get_fact: found=%d value=%d v%lu",
                     resp->found, resp->value, resp->wm_version);
      });

  svc_set_fact_ = create_service<ame_ros2::srv::SetFact>(
      "~/set_fact",
      [this](const ame_ros2::srv::SetFact::Request::SharedPtr req,
             ame_ros2::srv::SetFact::Response::SharedPtr resp) {
        RCLCPP_INFO(get_logger(), "set_fact: key='%s' value=%d source='%s'",
                    req->key.c_str(), req->value, req->source.c_str());
        auto r = component_->setFact(req->key, req->value, req->source);
        resp->success    = r.success;
        resp->wm_version = r.wm_version;
        RCLCPP_INFO(get_logger(), "set_fact: success=%d v%lu",
                    resp->success, resp->wm_version);
      });

  svc_query_state_ = create_service<ame_ros2::srv::QueryState>(
      "~/query_state",
      [this](const ame_ros2::srv::QueryState::Request::SharedPtr req,
             ame_ros2::srv::QueryState::Response::SharedPtr resp) {
        RCLCPP_INFO(get_logger(), "query_state: %zu key filter(s)", req->keys.size());
        auto snap = component_->queryState(
            std::vector<std::string>(req->keys.begin(), req->keys.end()));
        resp->success    = snap.success;
        resp->wm_version = snap.wm_version;
        resp->goal_fluents.assign(snap.goal_fluents.begin(), snap.goal_fluents.end());
        for (const auto& f : snap.facts) {
          ame_ros2::msg::WorldFact wf;
          wf.key        = f.key;
          wf.value      = f.value;
          wf.source     = f.source;
          wf.wm_version = f.wm_version;
          resp->facts.push_back(wf);
        }
        RCLCPP_INFO(get_logger(), "query_state: v%lu, %zu facts, %zu goals",
                    resp->wm_version, resp->facts.size(), resp->goal_fluents.size());
      });

  svc_load_domain_ = create_service<ame_ros2::srv::LoadDomain>(
      "~/load_domain",
      [this](const ame_ros2::srv::LoadDomain::Request::SharedPtr req,
             ame_ros2::srv::LoadDomain::Response::SharedPtr resp) {
        RCLCPP_INFO(get_logger(), "load_domain: %zu domain bytes, %zu problem bytes",
                    req->domain_pddl.size(), req->problem_pddl.size());
        auto r = component_->loadDomainFromStrings(req->domain_pddl, req->problem_pddl);
        resp->success            = r.success;
        resp->error_msg          = r.error_msg;
        resp->num_fluents        = r.num_fluents;
        resp->num_ground_actions = r.num_ground_actions;
        if (r.success) {
          RCLCPP_INFO(get_logger(), "load_domain: OK — %u fluents, %u ground actions",
                      r.num_fluents, r.num_ground_actions);
        } else {
          RCLCPP_ERROR(get_logger(), "load_domain: FAILED — %s", r.error_msg.c_str());
        }
      });

  // -- ROS2 world_state publisher -------------------------------------------
  pub_world_state_ = create_publisher<ame_ros2::msg::WorldState>("world_state", 10);
  component_->setWmPublishCallback(
      [this](const ame::WorldStateSnapshot& snap) {
        ame_ros2::msg::WorldState msg;
        msg.header.stamp = now();
        msg.wm_version   = snap.wm_version;
        msg.goal_fluents.assign(snap.goal_fluents.begin(), snap.goal_fluents.end());
        for (const auto& f : snap.facts) {
          ame_ros2::msg::WorldFact wf;
          wf.key        = f.key;
          wf.value      = f.value;
          wf.source     = f.source;
          wf.wm_version = f.wm_version;
          msg.facts.push_back(wf);
        }
        if (pub_world_state_) {
          pub_world_state_->publish(msg);
        }
      });

  RCLCPP_INFO(get_logger(),
              "WorldModelNode configured: %u fluents, %u ground actions",
              component_->worldModel().numFluents(),
              component_->worldModel().numGroundActions());
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_activate(const rclcpp_lifecycle::State&) {
  if (pub_world_state_) {
    pub_world_state_->on_activate();
  }
  if (component_->activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate WorldModelComponent");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "WorldModelNode activated");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_deactivate(const rclcpp_lifecycle::State&) {
  if (pub_world_state_) {
    pub_world_state_->on_deactivate();
  }
  component_->deactivate();
  RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State&) {
  component_->setWmPublishCallback({});
  svc_get_fact_.reset();
  svc_set_fact_.reset();
  svc_query_state_.reset();
  svc_load_domain_.reset();
  pub_world_state_.reset();
  component_->cleanup();
  RCLCPP_INFO(get_logger(), "WorldModelNode cleaned up");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_->shutdown();
  return CallbackReturn::SUCCESS;
}

}  // namespace ame_ros2
