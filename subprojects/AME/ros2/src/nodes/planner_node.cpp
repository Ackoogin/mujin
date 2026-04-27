#include <ame_ros2/planner_node.hpp>

#include <ame/planner_component.h>

#include <thread>

namespace ame_ros2 {

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("planner_node", options)
  , component_(std::make_unique<ame::PlannerComponent>())
{}

PlannerNode::~PlannerNode() = default;

void PlannerNode::setInProcessWorldModel(ame::WorldModel* wm) {
  inprocess_wm_ = wm;
  component_->setInProcessWorldModel(wm);
}

ame::ActionRegistry& PlannerNode::actionRegistry()   { return component_->actionRegistry(); }
ame::Planner&        PlannerNode::planner()           { return component_->planner(); }
ame::PlanCompiler&   PlannerNode::compiler()          { return component_->compiler(); }
ame::PlanAuditLog*   PlannerNode::planAuditLog()      { return component_->planAuditLog(); }
ame::PlannerComponent& PlannerNode::plannerComponent() { return *component_; }

PlannerNode::CallbackReturn
PlannerNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("domain.pddl_file",    std::string(""));
  declare_parameter("domain.problem_file", std::string(""));
  declare_parameter("plan_audit.enabled",  true);
  declare_parameter("plan_audit.path",     std::string("plan_audit.jsonl"));
  declare_parameter("compiler.parallel",   false);

  component_->setParam("domain.pddl_file",
                      get_parameter("domain.pddl_file").as_string().c_str());
  component_->setParam("domain.problem_file",
                      get_parameter("domain.problem_file").as_string().c_str());
  component_->setParam("plan_audit.enabled",
                      get_parameter("plan_audit.enabled").as_bool());
  component_->setParam("plan_audit.path",
                      get_parameter("plan_audit.path").as_string().c_str());
  component_->setParam("compiler.parallel",
                      get_parameter("compiler.parallel").as_bool());

  component_->setInProcessWorldModel(inprocess_wm_);

  if (component_->configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure PlannerComponent");
    return CallbackReturn::FAILURE;
  }

  // -- ROS2 plan action server ----------------------------------------------
  using PlanAction = ame_ros2::action::Plan;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanAction>;

  plan_action_srv_ = rclcpp_action::create_server<PlanAction>(
      this,
      "~/plan",
      [](const rclcpp_action::GoalUUID&,
         std::shared_ptr<const PlanAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> goal_handle) {
        // Run planning in a detached thread so we don't block the ROS2 executor.
        std::thread([this, goal_handle]() {
          auto goal   = goal_handle->get_goal();
          RCLCPP_INFO(get_logger(), "plan: received goal with %zu fluents",
                      goal->goal_fluents.size());
          auto result = std::make_shared<PlanAction::Result>();
          try {
            ame::PlannerExecutionResult r;
            {
              // Hold the WM mutex (if provided) to protect the WM copy taken
              // inside snapshotWorldModel() against concurrent on_tick writes.
              std::unique_lock<std::mutex> wm_lock;
              if (wm_mutex_) {
                wm_lock = std::unique_lock<std::mutex>(*wm_mutex_);
              }
              r = component_->solveGoal(
                  std::vector<std::string>(goal->goal_fluents.begin(),
                                           goal->goal_fluents.end()));
            }  // wm_lock released here

            result->success       = r.success;
            result->plan_actions.assign(r.plan_actions.begin(), r.plan_actions.end());
            result->action_indices.assign(r.action_indices.begin(), r.action_indices.end());
            result->bt_xml        = r.bt_xml;
            result->solve_time_ms = r.solve_time_ms;
            result->expanded      = r.expanded;
            result->generated     = r.generated;
            result->error_msg     = r.error_msg;
            if (r.success) {
              RCLCPP_INFO(get_logger(), "plan: found %zu steps in %.1f ms",
                          r.plan_actions.size(), r.solve_time_ms);
            } else {
              RCLCPP_WARN(get_logger(), "plan: FAILED -- %s", r.error_msg.c_str());
            }
          } catch (const std::exception& e) {
            result->success   = false;
            result->error_msg = e.what();
            RCLCPP_ERROR(get_logger(), "plan: exception -- %s", e.what());
          }
          if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
          } else {
            goal_handle->succeed(result);
          }
        }).detach();
      });

  // -- ROS2 load_domain service (planner side) ------------------------------
  svc_load_domain_ = create_service<ame_ros2::srv::LoadDomain>(
      "~/load_domain",
      [this](const ame_ros2::srv::LoadDomain::Request::SharedPtr req,
             ame_ros2::srv::LoadDomain::Response::SharedPtr resp) {
        RCLCPP_INFO(get_logger(), "load_domain: domain_id='%s'", req->domain_id.c_str());
        auto r = component_->loadDomainFromStrings(
            req->domain_id, req->domain_pddl, req->problem_pddl);
        resp->success            = r.success;
        resp->error_msg          = r.error_msg;
        resp->num_fluents        = r.num_fluents;
        resp->num_ground_actions = r.num_ground_actions;
        if (r.success) {
          RCLCPP_INFO(get_logger(), "load_domain: OK -- %u fluents, %u ground actions",
                      r.num_fluents, r.num_ground_actions);
        } else {
          RCLCPP_ERROR(get_logger(), "load_domain: FAILED -- %s", r.error_msg.c_str());
        }
      });

  RCLCPP_INFO(get_logger(), "PlannerNode configured");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_activate(const rclcpp_lifecycle::State&) {
  if (component_->activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate PlannerComponent");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "PlannerNode activated");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_deactivate(const rclcpp_lifecycle::State&) {
  component_->deactivate();
  RCLCPP_INFO(get_logger(), "PlannerNode deactivated");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_cleanup(const rclcpp_lifecycle::State&) {
  plan_action_srv_.reset();
  svc_load_domain_.reset();
  component_->cleanup();
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_->shutdown();
  return CallbackReturn::SUCCESS;
}

}  // namespace ame_ros2
