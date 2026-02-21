#include "mujin_ros2/world_model_node.hpp"
#include "mujin/pddl_parser.h"

#include <rclcpp/qos.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <stdexcept>

namespace mujin_ros2 {

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("world_model_node", options)
{}

WorldModelNode::CallbackReturn
WorldModelNode::on_configure(const rclcpp_lifecycle::State&) {
    declare_parameter("domain.pddl_file", "");
    declare_parameter("domain.problem_file", "");
    declare_parameter("audit_log.enabled", true);
    declare_parameter("audit_log.path", std::string("wm_audit.jsonl"));
    declare_parameter("publish_rate_hz", 10.0);

    try {
        loadDomainFromParams();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load domain: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    // Set up audit log
    if (get_parameter("audit_log.enabled").as_bool()) {
        wm_audit_ = mujin::WmAuditLog(get_parameter("audit_log.path").as_string());
    }

    // Wire audit callback
    wm_.setAuditCallback([this](uint64_t ver, uint64_t ts_us,
                                const std::string& fact, bool val,
                                const std::string& src) {
        wm_audit_.onFactChange(ver, ts_us, fact, val, src);
        state_dirty_.store(true);
    });

    // Services are created in on_configure so they are available immediately
    // (other nodes may start querying before this node is activated)
    using std::placeholders::_1;
    using std::placeholders::_2;

    srv_get_fact_ = create_service<mujin_ros2::srv::GetFact>(
        "~/get_fact",
        [this](auto req, auto res) { handleGetFact(req, res); });

    srv_set_fact_ = create_service<mujin_ros2::srv::SetFact>(
        "~/set_fact",
        [this](auto req, auto res) { handleSetFact(req, res); });

    srv_query_state_ = create_service<mujin_ros2::srv::QueryState>(
        "~/query_state",
        [this](auto req, auto res) { handleQueryState(req, res); });

    RCLCPP_INFO(get_logger(), "WorldModelNode configured: %u fluents, %u ground actions",
                wm_.numFluents(), wm_.numGroundActions());
    return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_activate(const rclcpp_lifecycle::State&) {
    // Reliable + transient_local: late subscribers get the current state on connection
    auto qos = rclcpp::QoS(10).reliable().transient_local();
    pub_world_state_ = create_publisher<mujin_ros2::msg::WorldState>("/world_state", qos);

    double rate_hz = get_parameter("publish_rate_hz").as_double();
    if (rate_hz <= 0.0) rate_hz = 10.0;
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));

    publish_timer_ = create_wall_timer(period, [this]() {
        if (state_dirty_.exchange(false)) {
            publishWorldState();
        }
    });

    // Publish initial state immediately
    publishWorldState();

    RCLCPP_INFO(get_logger(), "WorldModelNode activated (publish %.1f Hz)", rate_hz);
    return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_deactivate(const rclcpp_lifecycle::State&) {
    publish_timer_.reset();
    pub_world_state_.reset();
    RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");
    return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State&) {
    srv_get_fact_.reset();
    srv_set_fact_.reset();
    srv_query_state_.reset();
    wm_audit_.flush();
    RCLCPP_INFO(get_logger(), "WorldModelNode cleaned up");
    return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_shutdown(const rclcpp_lifecycle::State&) {
    wm_audit_.flush();
    return CallbackReturn::SUCCESS;
}

void WorldModelNode::loadDomainFromParams() {
    const auto domain_file  = get_parameter("domain.pddl_file").as_string();
    const auto problem_file = get_parameter("domain.problem_file").as_string();

    if (!domain_file.empty() && !problem_file.empty()) {
        RCLCPP_INFO(get_logger(), "Parsing PDDL: %s + %s",
                    domain_file.c_str(), problem_file.c_str());
        mujin::PddlParser::parse(domain_file, problem_file, wm_);
    } else {
        RCLCPP_WARN(get_logger(),
            "No PDDL files specified (domain.pddl_file / domain.problem_file). "
            "WorldModel is empty — populate programmatically via set_fact service.");
    }
}

void WorldModelNode::handleGetFact(
    std::shared_ptr<mujin_ros2::srv::GetFact::Request>  req,
    std::shared_ptr<mujin_ros2::srv::GetFact::Response> res)
{
    try {
        res->value      = wm_.getFact(req->key);
        res->found      = true;
        res->wm_version = wm_.version();
    } catch (...) {
        res->found      = false;
        res->value      = false;
        res->wm_version = wm_.version();
    }
}

void WorldModelNode::handleSetFact(
    std::shared_ptr<mujin_ros2::srv::SetFact::Request>  req,
    std::shared_ptr<mujin_ros2::srv::SetFact::Response> res)
{
    try {
        wm_.setFact(req->key, req->value, req->source);
        res->success    = true;
        res->wm_version = wm_.version();
    } catch (...) {
        res->success    = false;
        res->wm_version = wm_.version();
    }
}

void WorldModelNode::handleQueryState(
    std::shared_ptr<mujin_ros2::srv::QueryState::Request>  req,
    std::shared_ptr<mujin_ros2::srv::QueryState::Response> res)
{
    res->wm_version = wm_.version();
    res->success    = true;

    if (req->keys.empty()) {
        // Return all true fluents
        for (unsigned i = 0; i < wm_.numFluents(); ++i) {
            if (wm_.getFact(i)) {
                mujin_ros2::msg::WorldFact f;
                f.key        = wm_.fluentName(i);
                f.value      = true;
                f.source     = "";
                f.wm_version = wm_.version();
                res->facts.push_back(f);
            }
        }
    } else {
        for (const auto& key : req->keys) {
            mujin_ros2::msg::WorldFact f;
            f.key        = key;
            f.wm_version = wm_.version();
            try {
                f.value = wm_.getFact(key);
            } catch (...) {
                f.value = false;
            }
            f.source = "";
            res->facts.push_back(f);
        }
    }
}

void WorldModelNode::publishWorldState() {
    if (!pub_world_state_ || !pub_world_state_->is_activated()) return;

    mujin_ros2::msg::WorldState msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "";
    msg.wm_version      = wm_.version();

    for (unsigned i = 0; i < wm_.numFluents(); ++i) {
        if (wm_.getFact(i)) {
            mujin_ros2::msg::WorldFact f;
            f.key        = wm_.fluentName(i);
            f.value      = true;
            f.source     = "";
            f.wm_version = wm_.version();
            msg.facts.push_back(f);
        }
    }

    for (auto gid : wm_.goalFluentIds()) {
        msg.goal_fluents.push_back(wm_.fluentName(gid));
    }

    pub_world_state_->publish(msg);
}

} // namespace mujin_ros2
