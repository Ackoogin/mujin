/**
 * @file ame_py.cpp
 * @brief pybind11 Python bindings for ame_core library.
 *
 * Exposes WorldModel, Planner, PlanCompiler, and PDDL parsing to Python,
 * enabling the devenv to run without ROS2.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <ame/world_model.h>
#include <ame/planner.h>
#include <ame/plan_compiler.h>
#include <ame/action_registry.h>
#include <ame/pddl_parser.h>
#include <ame/goal_allocator.h>
#include <ame/executor_component.h>
#include <ame/autonomy_backend.h>
#include <ame/current_ame_backend_adapter.h>

namespace py = pybind11;

PYBIND11_MODULE(_ame_py, m) {
    m.doc() = "AME Core Python bindings - direct access to planning components";

    // -------------------------------------------------------------------------
    // FactAuthority enum
    // -------------------------------------------------------------------------
    py::enum_<ame::FactAuthority>(m, "FactAuthority")
        .value("BELIEVED", ame::FactAuthority::BELIEVED)
        .value("CONFIRMED", ame::FactAuthority::CONFIRMED)
        .export_values();

    py::enum_<ame::FactAuthorityLevel>(m, "FactAuthorityLevel")
        .value("BELIEVED", ame::FactAuthorityLevel::BELIEVED)
        .value("CONFIRMED", ame::FactAuthorityLevel::CONFIRMED)
        .export_values();

    py::enum_<ame::AutonomyBackendState>(m, "AutonomyBackendState")
        .value("IDLE", ame::AutonomyBackendState::IDLE)
        .value("READY", ame::AutonomyBackendState::READY)
        .value("WAITING_FOR_RESULTS", ame::AutonomyBackendState::WAITING_FOR_RESULTS)
        .value("COMPLETE", ame::AutonomyBackendState::COMPLETE)
        .value("FAILED", ame::AutonomyBackendState::FAILED)
        .value("STOPPED", ame::AutonomyBackendState::STOPPED)
        .export_values();

    py::enum_<ame::CommandStatus>(m, "CommandStatus")
        .value("PENDING", ame::CommandStatus::PENDING)
        .value("RUNNING", ame::CommandStatus::RUNNING)
        .value("SUCCEEDED", ame::CommandStatus::SUCCEEDED)
        .value("FAILED_TRANSIENT", ame::CommandStatus::FAILED_TRANSIENT)
        .value("FAILED_PERMANENT", ame::CommandStatus::FAILED_PERMANENT)
        .value("CANCELLED", ame::CommandStatus::CANCELLED)
        .export_values();

    py::enum_<ame::StopMode>(m, "StopMode")
        .value("DRAIN", ame::StopMode::DRAIN)
        .value("IMMEDIATE", ame::StopMode::IMMEDIATE)
        .export_values();

    py::class_<ame::FactUpdate>(m, "FactUpdate")
        .def(py::init<>())
        .def_readwrite("key", &ame::FactUpdate::key)
        .def_readwrite("value", &ame::FactUpdate::value)
        .def_readwrite("source", &ame::FactUpdate::source)
        .def_readwrite("authority", &ame::FactUpdate::authority);

    py::class_<ame::StateUpdate>(m, "StateUpdate")
        .def(py::init<>())
        .def_readwrite("fact_updates", &ame::StateUpdate::fact_updates);

    py::class_<ame::MissionIntent>(m, "MissionIntent")
        .def(py::init<>())
        .def_readwrite("goal_fluents", &ame::MissionIntent::goal_fluents);

    py::class_<ame::AgentState>(m, "AgentState")
        .def(py::init<>())
        .def_readwrite("agent_id", &ame::AgentState::agent_id)
        .def_readwrite("agent_type", &ame::AgentState::agent_type)
        .def_readwrite("available", &ame::AgentState::available);

    py::class_<ame::PolicyEnvelope>(m, "PolicyEnvelope")
        .def(py::init<>())
        .def_readwrite("max_replans", &ame::PolicyEnvelope::max_replans)
        .def_readwrite("enable_goal_dispatch", &ame::PolicyEnvelope::enable_goal_dispatch);

    py::class_<ame::SessionRequest>(m, "SessionRequest")
        .def(py::init<>())
        .def_readwrite("session_id", &ame::SessionRequest::session_id)
        .def_readwrite("intent", &ame::SessionRequest::intent)
        .def_readwrite("policy", &ame::SessionRequest::policy)
        .def_readwrite("available_agents", &ame::SessionRequest::available_agents);

    py::class_<ame::AutonomyBackendCapabilities>(m, "AutonomyBackendCapabilities")
        .def(py::init<>())
        .def_readwrite("backend_id", &ame::AutonomyBackendCapabilities::backend_id)
        .def_readwrite("supports_batch_planning", &ame::AutonomyBackendCapabilities::supports_batch_planning)
        .def_readwrite("supports_external_command_dispatch", &ame::AutonomyBackendCapabilities::supports_external_command_dispatch)
        .def_readwrite("supports_replanning", &ame::AutonomyBackendCapabilities::supports_replanning);

    py::class_<ame::ActionCommand>(m, "ActionCommand")
        .def(py::init<>())
        .def_readwrite("command_id", &ame::ActionCommand::command_id)
        .def_readwrite("action_name", &ame::ActionCommand::action_name)
        .def_readwrite("signature", &ame::ActionCommand::signature)
        .def_readwrite("service_name", &ame::ActionCommand::service_name)
        .def_readwrite("operation", &ame::ActionCommand::operation)
        .def_readwrite("request_fields", &ame::ActionCommand::request_fields);

    py::class_<ame::GoalDispatch>(m, "GoalDispatch")
        .def(py::init<>())
        .def_readwrite("dispatch_id", &ame::GoalDispatch::dispatch_id)
        .def_readwrite("agent_id", &ame::GoalDispatch::agent_id)
        .def_readwrite("goals", &ame::GoalDispatch::goals);

    py::class_<ame::DecisionRecord>(m, "DecisionRecord")
        .def(py::init<>())
        .def_readwrite("session_id", &ame::DecisionRecord::session_id)
        .def_readwrite("backend_id", &ame::DecisionRecord::backend_id)
        .def_readwrite("world_version", &ame::DecisionRecord::world_version)
        .def_readwrite("replan_count", &ame::DecisionRecord::replan_count)
        .def_readwrite("plan_success", &ame::DecisionRecord::plan_success)
        .def_readwrite("solve_time_ms", &ame::DecisionRecord::solve_time_ms)
        .def_readwrite("planned_action_signatures", &ame::DecisionRecord::planned_action_signatures)
        .def_readwrite("compiled_bt_xml", &ame::DecisionRecord::compiled_bt_xml);

    py::class_<ame::CommandResult>(m, "CommandResult")
        .def(py::init<>())
        .def_readwrite("command_id", &ame::CommandResult::command_id)
        .def_readwrite("status", &ame::CommandResult::status)
        .def_readwrite("observed_updates", &ame::CommandResult::observed_updates)
        .def_readwrite("source", &ame::CommandResult::source);

    py::class_<ame::DispatchResult>(m, "DispatchResult")
        .def(py::init<>())
        .def_readwrite("dispatch_id", &ame::DispatchResult::dispatch_id)
        .def_readwrite("status", &ame::DispatchResult::status)
        .def_readwrite("observed_updates", &ame::DispatchResult::observed_updates)
        .def_readwrite("source", &ame::DispatchResult::source);

    py::class_<ame::AutonomyBackendSnapshot>(m, "AutonomyBackendSnapshot")
        .def(py::init<>())
        .def_readwrite("session_id", &ame::AutonomyBackendSnapshot::session_id)
        .def_readwrite("state", &ame::AutonomyBackendSnapshot::state)
        .def_readwrite("world_version", &ame::AutonomyBackendSnapshot::world_version)
        .def_readwrite("replan_count", &ame::AutonomyBackendSnapshot::replan_count)
        .def_readwrite("agent_states", &ame::AutonomyBackendSnapshot::agent_states)
        .def_readwrite("outstanding_commands", &ame::AutonomyBackendSnapshot::outstanding_commands)
        .def_readwrite("outstanding_goal_dispatches", &ame::AutonomyBackendSnapshot::outstanding_goal_dispatches)
        .def_readwrite("decision_history", &ame::AutonomyBackendSnapshot::decision_history);

    // -------------------------------------------------------------------------
    // AgentInfo struct
    // -------------------------------------------------------------------------
    py::class_<ame::AgentInfo>(m, "AgentInfo")
        .def(py::init<>())
        .def_readwrite("id", &ame::AgentInfo::id)
        .def_readwrite("type", &ame::AgentInfo::type)
        .def_readwrite("available", &ame::AgentInfo::available);

    py::class_<ame::FactMetadata>(m, "FactMetadata")
        .def(py::init<>())
        .def_readwrite("authority", &ame::FactMetadata::authority)
        .def_readwrite("timestamp_us", &ame::FactMetadata::timestamp_us)
        .def_readwrite("source", &ame::FactMetadata::source);

    // -------------------------------------------------------------------------
    // WorldModel
    // -------------------------------------------------------------------------
    py::class_<ame::WorldModel>(m, "WorldModel")
        .def(py::init<>())
        .def("type_system", static_cast<ame::TypeSystem&(ame::WorldModel::*)()>(&ame::WorldModel::typeSystem),
            py::return_value_policy::reference_internal,
            "Access the type system")
        // Fact management - use lambda to avoid overload issues
        .def("set_fact", [](ame::WorldModel& wm, const std::string& key, bool value) {
            wm.setFact(key, value);
        }, py::arg("key"), py::arg("value"),
            "Set a fact by string key")
        .def("set_fact_with_metadata", [](ame::WorldModel& wm,
                                          const std::string& key,
                                          bool value,
                                          const std::string& source,
                                          ame::FactAuthority authority) {
            wm.setFact(key, value, source, authority);
        }, py::arg("key"), py::arg("value"), py::arg("source"), py::arg("authority"),
            "Set a fact by string key with source and authority")
        .def("get_fact", [](const ame::WorldModel& wm, const std::string& key) {
            return wm.getFact(key);
        }, py::arg("key"),
            "Get a fact value by string key")
        .def("get_fact_metadata", [](const ame::WorldModel& wm, const std::string& key) {
            return wm.getFactMetadata(key);
        }, py::arg("key"),
            "Get fact metadata by string key")
        .def("version", &ame::WorldModel::version,
            "Get the current world model version")
        .def("num_fluents", &ame::WorldModel::numFluents,
            "Get the number of fluents")
        .def("fluent_name", &ame::WorldModel::fluentName,
            py::arg("id"),
            "Get fluent name by ID")
        // Goals
        .def("set_goal", &ame::WorldModel::setGoal,
            py::arg("goal_fluents"),
            "Set goal fluents by string keys")
        .def("goal_fluent_ids", &ame::WorldModel::goalFluentIds,
            "Get goal fluent IDs")
        // Types and predicates
        .def("register_predicate", &ame::WorldModel::registerPredicate,
            py::arg("name"), py::arg("param_types"),
            "Register a predicate with parameter types")
        .def("add_object", &ame::WorldModel::addObject,
            py::arg("name"), py::arg("type"),
            "Add an object with a type")
        // Actions
        .def("register_action", &ame::WorldModel::registerAction,
            py::arg("name"), py::arg("params"), py::arg("param_types"),
            py::arg("preconditions"), py::arg("add_effects"), py::arg("del_effects"),
            "Register an action schema")
        .def("num_ground_actions", &ame::WorldModel::numGroundActions,
            "Get number of grounded actions")
        // Agent management
        .def("register_agent", &ame::WorldModel::registerAgent,
            py::arg("id"), py::arg("type"),
            "Register an agent")
        .def("get_agent", static_cast<ame::AgentInfo*(ame::WorldModel::*)(const std::string&)>(&ame::WorldModel::getAgent),
            py::arg("id"),
            py::return_value_policy::reference_internal,
            "Get mutable agent info by id, or None if missing")
        .def("agent_ids", &ame::WorldModel::agentIds,
            "Get all agent IDs")
        .def("available_agent_ids", &ame::WorldModel::availableAgentIds,
            "Get available agent IDs")
        .def("num_agents", &ame::WorldModel::numAgents,
            "Get number of registered agents")
        // Audit callback: fires on every fact state change
        .def("set_audit_callback", [](ame::WorldModel& wm, py::function callback) {
            wm.setAuditCallback([callback](uint64_t version, uint64_t ts_us,
                                           const std::string& fact, bool value,
                                           const std::string& source) {
                py::gil_scoped_acquire gil;
                try {
                    callback(version, ts_us, fact, value, source);
                } catch (...) {}
            });
        }, py::arg("callback"),
            "Set callback fired on every WM fact change: (version, ts_us, fact, value, source)")
        // Snapshot for iteration
        .def("all_true_facts", [](const ame::WorldModel& wm) {
            std::vector<std::string> facts;
            for (unsigned i = 0; i < wm.numFluents(); ++i) {
                if (wm.getFact(i)) {
                    facts.push_back(wm.fluentName(i));
                }
            }
            return facts;
        }, "Get all true facts as strings");

    // -------------------------------------------------------------------------
    // TypeSystem
    // -------------------------------------------------------------------------
    py::class_<ame::TypeSystem>(m, "TypeSystem")
        .def("add_type", &ame::TypeSystem::addType,
            py::arg("name"), py::arg("parent") = "",
            "Add a type with optional parent");

    // -------------------------------------------------------------------------
    // PlanStep
    // -------------------------------------------------------------------------
    py::class_<ame::PlanStep>(m, "PlanStep")
        .def(py::init<>())
        .def_readwrite("action_index", &ame::PlanStep::action_index);

    // -------------------------------------------------------------------------
    // PlanResult
    // -------------------------------------------------------------------------
    py::class_<ame::PlanResult>(m, "PlanResult")
        .def(py::init<>())
        .def_readonly("success", &ame::PlanResult::success)
        .def_readonly("steps", &ame::PlanResult::steps)
        .def_readonly("solve_time_ms", &ame::PlanResult::solve_time_ms)
        .def_readonly("expanded", &ame::PlanResult::expanded)
        .def_readonly("generated", &ame::PlanResult::generated)
        .def_readonly("cost", &ame::PlanResult::cost);

    // -------------------------------------------------------------------------
    // Planner
    // -------------------------------------------------------------------------
    py::class_<ame::Planner>(m, "Planner")
        .def(py::init<>())
        .def("solve", &ame::Planner::solve,
            py::arg("wm"),
            "Solve the planning problem defined in WorldModel");

    // -------------------------------------------------------------------------
    // ActionRegistry
    // -------------------------------------------------------------------------
    py::class_<ame::ActionRegistry>(m, "ActionRegistry")
        .def(py::init<>())
        .def("register_action", &ame::ActionRegistry::registerAction,
            py::arg("pddl_name"), py::arg("bt_node_type"), py::arg("reactive") = false)
        .def("register_action_subtree", &ame::ActionRegistry::registerActionSubTree,
            py::arg("pddl_name"), py::arg("subtree_xml_template"), py::arg("reactive") = false)
        .def("has_action", &ame::ActionRegistry::hasAction, py::arg("pddl_name"));

    // -------------------------------------------------------------------------
    // PlanCompiler
    // -------------------------------------------------------------------------
    py::class_<ame::PlanCompiler>(m, "PlanCompiler")
        .def(py::init<>())
        .def("compile", [](const ame::PlanCompiler& compiler,
                           const std::vector<ame::PlanStep>& plan,
                           const ame::WorldModel& wm,
                           const ame::ActionRegistry& registry) {
            return compiler.compile(plan, wm, registry);
        }, py::arg("plan"), py::arg("wm"), py::arg("registry"),
            "Compile a plan to BehaviorTree XML")
        .def("compile_with_agent", [](const ame::PlanCompiler& compiler,
                                       const std::vector<ame::PlanStep>& plan,
                                       const ame::WorldModel& wm,
                                       const ame::ActionRegistry& registry,
                                       const std::string& agent_id) {
            return compiler.compile(plan, wm, registry, agent_id);
        }, py::arg("plan"), py::arg("wm"), py::arg("registry"), py::arg("agent_id"),
            "Compile a plan to BehaviorTree XML with agent context");

    py::class_<ame::CurrentAmeBackendAdapter>(m, "CurrentAmeBackendAdapter")
        .def(py::init<ame::WorldModel&, const ame::ActionRegistry&, const ame::Planner&, const ame::PlanCompiler&>(),
            py::arg("world_model"),
            py::arg("action_registry"),
            py::arg("planner"),
            py::arg("plan_compiler"))
        .def("describe_capabilities", &ame::CurrentAmeBackendAdapter::describeCapabilities)
        .def("start", &ame::CurrentAmeBackendAdapter::start, py::arg("request"))
        .def("push_state", &ame::CurrentAmeBackendAdapter::pushState, py::arg("update"))
        .def("push_intent", &ame::CurrentAmeBackendAdapter::pushIntent, py::arg("intent"))
        .def("step", &ame::CurrentAmeBackendAdapter::step)
        .def("pull_commands", &ame::CurrentAmeBackendAdapter::pullCommands)
        .def("pull_goal_dispatches", &ame::CurrentAmeBackendAdapter::pullGoalDispatches)
        .def("pull_decision_records", &ame::CurrentAmeBackendAdapter::pullDecisionRecords)
        .def("push_command_result", &ame::CurrentAmeBackendAdapter::pushCommandResult, py::arg("result"))
        .def("push_dispatch_result", &ame::CurrentAmeBackendAdapter::pushDispatchResult, py::arg("result"))
        .def("request_stop", &ame::CurrentAmeBackendAdapter::requestStop, py::arg("mode"))
        .def("read_snapshot", &ame::CurrentAmeBackendAdapter::readSnapshot);

    // -------------------------------------------------------------------------
    // GoalAllocator
    // -------------------------------------------------------------------------
    py::class_<ame::AgentGoalAssignment>(m, "AgentGoalAssignment")
        .def(py::init<>())
        .def_readwrite("agent_id", &ame::AgentGoalAssignment::agent_id)
        .def_readwrite("goals", &ame::AgentGoalAssignment::goals);

    py::class_<ame::GoalAllocator>(m, "GoalAllocator")
        .def(py::init<>())
        .def("allocate", static_cast<std::vector<ame::AgentGoalAssignment> (ame::GoalAllocator::*)(
                const std::vector<std::string>&,
                const ame::WorldModel&) const>(&ame::GoalAllocator::allocate),
            py::arg("goals"), py::arg("wm"),
            "Allocate goals to available agents from a WorldModel");

    // -------------------------------------------------------------------------
    // PDDL Parser - use PddlParser::parse
    // -------------------------------------------------------------------------
    m.def("parse_pddl", [](ame::WorldModel& wm,
                           const std::string& domain_path,
                           const std::string& problem_path) {
        ame::PddlParser::parse(domain_path, problem_path, wm);
    }, py::arg("wm"), py::arg("domain_path"), py::arg("problem_path"),
        "Parse PDDL domain and problem files into WorldModel");

    // -------------------------------------------------------------------------
    // Utility: get action signature from plan step
    // -------------------------------------------------------------------------
    m.def("get_action_signature", [](const ame::WorldModel& wm, unsigned action_index) {
        if (action_index < wm.numGroundActions()) {
            return wm.groundActions()[action_index].signature;
        }
        return std::string{};
    }, py::arg("wm"), py::arg("action_index"),
    "Get action signature string from action index");

    // -------------------------------------------------------------------------
    // BT NodeStatus enum
    // -------------------------------------------------------------------------
    py::enum_<BT::NodeStatus>(m, "NodeStatus")
        .value("IDLE", BT::NodeStatus::IDLE)
        .value("RUNNING", BT::NodeStatus::RUNNING)
        .value("SUCCESS", BT::NodeStatus::SUCCESS)
        .value("FAILURE", BT::NodeStatus::FAILURE)
        .value("SKIPPED", BT::NodeStatus::SKIPPED)
        .export_values();

    // -------------------------------------------------------------------------
    // ExecutorComponent - BT execution
    // -------------------------------------------------------------------------
    py::class_<ame::ExecutorComponent>(m, "ExecutorComponent")
        .def(py::init<>())
        .def("set_inprocess_world_model", &ame::ExecutorComponent::setInProcessWorldModel,
            py::arg("wm"),
            "Inject world model for in-process execution")
        .def("set_event_sink", [](ame::ExecutorComponent& exec, py::function callback) {
            exec.setEventSink([callback](const std::string& event) {
                py::gil_scoped_acquire gil;
                try {
                    callback(event);
                } catch (...) {}
            });
        }, py::arg("callback"),
            "Set callback for BT events")
        .def("configure", [](ame::ExecutorComponent& exec) {
            return exec.configure() == PCL_OK;
        }, "Configure the executor")
        .def("activate", [](ame::ExecutorComponent& exec) {
            return exec.activate() == PCL_OK;
        }, "Activate the executor")
        .def("deactivate", [](ame::ExecutorComponent& exec) {
            return exec.deactivate() == PCL_OK;
        }, "Deactivate the executor")
        .def("load_and_execute", &ame::ExecutorComponent::loadAndExecute,
            py::arg("bt_xml"),
            "Load BT XML and start execution")
        .def("tick_once", &ame::ExecutorComponent::tickOnce,
            "Tick the BT once")
        .def("halt_execution", &ame::ExecutorComponent::haltExecution,
            "Halt execution explicitly")
        .def("is_executing", &ame::ExecutorComponent::isExecuting,
            "Check if BT is currently executing")
        .def("last_status", &ame::ExecutorComponent::lastStatus,
            "Get last BT status");
}
