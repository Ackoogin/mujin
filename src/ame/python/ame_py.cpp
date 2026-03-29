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

    // -------------------------------------------------------------------------
    // AgentInfo struct
    // -------------------------------------------------------------------------
    py::class_<ame::AgentInfo>(m, "AgentInfo")
        .def(py::init<>())
        .def_readwrite("id", &ame::AgentInfo::id)
        .def_readwrite("type", &ame::AgentInfo::type)
        .def_readwrite("available", &ame::AgentInfo::available);

    // -------------------------------------------------------------------------
    // WorldModel
    // -------------------------------------------------------------------------
    py::class_<ame::WorldModel>(m, "WorldModel")
        .def(py::init<>())
        // Fact management - use lambda to avoid overload issues
        .def("set_fact", [](ame::WorldModel& wm, const std::string& key, bool value) {
            wm.setFact(key, value);
        }, py::arg("key"), py::arg("value"),
            "Set a fact by string key")
        .def("get_fact", [](const ame::WorldModel& wm, const std::string& key) {
            return wm.getFact(key);
        }, py::arg("key"),
            "Get a fact value by string key")
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
        .def("agent_ids", &ame::WorldModel::agentIds,
            "Get all agent IDs")
        .def("available_agent_ids", &ame::WorldModel::availableAgentIds,
            "Get available agent IDs")
        .def("num_agents", &ame::WorldModel::numAgents,
            "Get number of registered agents")
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
            py::arg("name"), py::arg("parent") = "object",
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
        .def(py::init<>());

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

    // -------------------------------------------------------------------------
    // GoalAllocator
    // -------------------------------------------------------------------------
    py::class_<ame::AgentGoalAssignment>(m, "AgentGoalAssignment")
        .def(py::init<>())
        .def_readwrite("agent_id", &ame::AgentGoalAssignment::agent_id)
        .def_readwrite("goals", &ame::AgentGoalAssignment::goals);

    py::class_<ame::GoalAllocator>(m, "GoalAllocator")
        .def(py::init<>())
        .def("allocate", &ame::GoalAllocator::allocate,
            py::arg("goals"), py::arg("wm"),
            "Allocate goals to available agents");

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
        .def("is_executing", &ame::ExecutorComponent::isExecuting,
            "Check if BT is currently executing")
        .def("last_status", &ame::ExecutorComponent::lastStatus,
            "Get last BT status");
}
