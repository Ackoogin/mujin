import unittest

from subprojects.AME.autonomy_backend import (
    AmeAutonomyBackend,
    AgentState,
    AutonomyBackendState,
    CommandResult,
    CommandStatus,
    DispatchResult,
    FactAuthorityLevel,
    FactUpdate,
    GoalDispatch,
    MissionIntent,
    PolicyEnvelope,
    SessionRequest,
    StateUpdate,
)
from _ame_py import (
    ActionRegistry,
    FactAuthority,
    WorldModel,
)


def build_domain() -> WorldModel:
    wm = WorldModel()
    ts = wm.type_system()
    ts.add_type("object")
    ts.add_type("location", "object")
    ts.add_type("sector", "location")
    ts.add_type("robot", "object")

    wm.add_object("uav1", "robot")
    wm.add_object("base", "location")
    wm.add_object("sector_a", "sector")

    wm.register_predicate("at", ["robot", "location"])
    wm.register_predicate("searched", ["sector"])

    wm.register_action(
        "move",
        ["?r", "?from", "?to"],
        ["robot", "location", "location"],
        ["(at ?r ?from)"],
        ["(at ?r ?to)"],
        ["(at ?r ?from)"],
    )
    wm.register_action(
        "search",
        ["?r", "?s"],
        ["robot", "sector"],
        ["(at ?r ?s)"],
        ["(searched ?s)"],
        [],
    )
    return wm


def build_registry() -> ActionRegistry:
    registry = ActionRegistry()
    registry.register_action_subtree(
        "move",
        "<InvokeService service_name=\"mobility\" operation=\"move\" "
        "param_names=\"?robot;?from;?to\" "
        "param_values=\"{param0};{param1};{param2}\" timeout_ms=\"0\"/>",
    )
    registry.register_action_subtree(
        "search",
        "<InvokeService service_name=\"imaging\" operation=\"search\" "
        "param_names=\"?robot;?sector\" "
        "param_values=\"{param0};{param1}\" timeout_ms=\"0\"/>",
    )
    return registry


class TestAutonomyBackendPython(unittest.TestCase):
    def make_backend(self):
        wm = build_domain()
        wm.set_fact("(at uav1 base)", True)
        registry = build_registry()
        backend = AmeAutonomyBackend(wm, registry)

        request = SessionRequest(
            session_id="py-session",
            intent=MissionIntent(["(searched sector_a)"]),
            policy=PolicyEnvelope(max_replans=4),
        )
        backend.start(request)
        return wm, backend

    def test_python_wrapper_matches_cpp_flow(self):
        wm, backend = self.make_backend()

        backend.step()

        records = backend.pull_decision_records()
        self.assertEqual(len(records), 1)
        self.assertTrue(records[0].plan_success)
        self.assertIn("move(uav1,base,sector_a)", records[0].planned_action_signatures)

        commands = backend.pull_commands()
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0].service_name, "mobility")
        self.assertEqual(commands[0].operation, "move")
        self.assertEqual(backend.read_snapshot().state, AutonomyBackendState.WAITING_FOR_RESULTS)

        move_result = CommandResult(
            command_id=commands[0].command_id,
            status=CommandStatus.SUCCEEDED,
            source="dispatcher:move",
        )
        backend.push_command_result(move_result)

        backend.step()
        commands = backend.pull_commands()
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0].service_name, "imaging")
        self.assertEqual(commands[0].operation, "search")
        self.assertTrue(wm.get_fact("(at uav1 sector_a)"))
        self.assertFalse(wm.get_fact("(at uav1 base)"))
        self.assertEqual(wm.get_fact_metadata("(at uav1 sector_a)").authority, FactAuthority.BELIEVED)

        search_result = CommandResult(
            command_id=commands[0].command_id,
            status=CommandStatus.SUCCEEDED,
            source="dispatcher:search",
        )
        backend.push_command_result(search_result)

        backend.step()
        self.assertTrue(wm.get_fact("(searched sector_a)"))
        self.assertEqual(backend.read_snapshot().state, AutonomyBackendState.COMPLETE)

    def test_confirmed_updates_override_predicted_effects(self):
        wm, backend = self.make_backend()

        backend.step()
        commands = backend.pull_commands()

        move_result = CommandResult(
            command_id=commands[0].command_id,
            status=CommandStatus.SUCCEEDED,
            observed_updates=[
                FactUpdate(
                    key="(at uav1 base)",
                    value=False,
                    source="perception:gps",
                    authority=FactAuthorityLevel.CONFIRMED,
                ),
                FactUpdate(
                    key="(at uav1 sector_a)",
                    value=True,
                    source="perception:gps",
                    authority=FactAuthorityLevel.CONFIRMED,
                ),
            ],
        )

        backend.push_command_result(move_result)
        metadata = wm.get_fact_metadata("(at uav1 sector_a)")
        self.assertEqual(metadata.authority, FactAuthority.CONFIRMED)
        self.assertEqual(metadata.source, "perception:gps")

    def test_goal_dispatch_surface(self):
        wm = build_domain()
        wm.set_fact("(at uav1 base)", True)
        registry = build_registry()
        backend = AmeAutonomyBackend(wm, registry)

        backend.start(
            SessionRequest(
                session_id="dispatch-session",
                intent=MissionIntent(["(searched sector_a)"]),
                policy=PolicyEnvelope(max_replans=3, enable_goal_dispatch=True),
                available_agents=[
                    AgentState("uav1", "uav", True),
                    AgentState("uav2", "uav", True),
                ],
            )
        )

        backend.step()
        dispatches = backend.pull_goal_dispatches()
        self.assertEqual(len(dispatches), 1)
        self.assertIsInstance(dispatches[0], GoalDispatch)
        self.assertEqual(dispatches[0].agent_id, "uav1")
        self.assertEqual(dispatches[0].goals, ["(searched sector_a)"])
        snapshot = backend.read_snapshot()
        self.assertEqual(len(snapshot.agent_states), 2)
        self.assertFalse(snapshot.agent_states[0].available)
        self.assertTrue(snapshot.agent_states[1].available)
        self.assertEqual(len(snapshot.outstanding_goal_dispatches), 1)
        self.assertEqual(snapshot.state, AutonomyBackendState.WAITING_FOR_RESULTS)

        backend.push_dispatch_result(
            DispatchResult(
                dispatch_id=dispatches[0].dispatch_id,
                status=CommandStatus.SUCCEEDED,
            )
        )
        done = backend.read_snapshot()
        self.assertTrue(done.agent_states[0].available)
        self.assertTrue(done.agent_states[1].available)
        self.assertEqual(len(done.outstanding_goal_dispatches), 0)

    def test_failed_goal_dispatch_restores_agent_availability(self):
        wm = build_domain()
        registry = build_registry()
        backend = AmeAutonomyBackend(wm, registry)

        backend.start(
            SessionRequest(
                session_id="dispatch-fail",
                intent=MissionIntent(["(searched sector_a)"]),
                policy=PolicyEnvelope(max_replans=3, enable_goal_dispatch=True),
                available_agents=[AgentState("uav1", "uav", True)],
            )
        )

        backend.step()
        dispatches = backend.pull_goal_dispatches()
        self.assertEqual(len(dispatches), 1)
        self.assertFalse(backend.read_snapshot().agent_states[0].available)

        backend.push_dispatch_result(
            DispatchResult(
                dispatch_id=dispatches[0].dispatch_id,
                status=CommandStatus.FAILED_TRANSIENT,
            )
        )

        snapshot = backend.read_snapshot()
        self.assertTrue(snapshot.agent_states[0].available)
        self.assertEqual(snapshot.state, AutonomyBackendState.READY)


if __name__ == "__main__":
    unittest.main()
