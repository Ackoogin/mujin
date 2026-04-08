import unittest

from subprojects.AME.autonomy_backend import AmeAutonomyBackend
from _ame_py import (
    ActionRegistry,
    AutonomyBackendState,
    CommandResult,
    CommandStatus,
    FactAuthority,
    FactAuthorityLevel,
    FactUpdate,
    MissionIntent,
    PolicyEnvelope,
    SessionRequest,
    StateUpdate,
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

        request = SessionRequest()
        request.session_id = "py-session"
        request.intent = MissionIntent()
        request.intent.goal_fluents = ["(searched sector_a)"]
        request.policy = PolicyEnvelope()
        request.policy.max_replans = 4
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

        move_result = CommandResult()
        move_result.command_id = commands[0].command_id
        move_result.status = CommandStatus.SUCCEEDED
        move_result.source = "dispatcher:move"
        backend.push_command_result(move_result)

        backend.step()
        commands = backend.pull_commands()
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0].service_name, "imaging")
        self.assertEqual(commands[0].operation, "search")
        self.assertTrue(wm.get_fact("(at uav1 sector_a)"))
        self.assertFalse(wm.get_fact("(at uav1 base)"))
        self.assertEqual(wm.get_fact_metadata("(at uav1 sector_a)").authority, FactAuthority.BELIEVED)

        search_result = CommandResult()
        search_result.command_id = commands[0].command_id
        search_result.status = CommandStatus.SUCCEEDED
        search_result.source = "dispatcher:search"
        backend.push_command_result(search_result)

        backend.step()
        self.assertTrue(wm.get_fact("(searched sector_a)"))
        self.assertEqual(backend.read_snapshot().state, AutonomyBackendState.COMPLETE)

    def test_confirmed_updates_override_predicted_effects(self):
        wm, backend = self.make_backend()

        backend.step()
        commands = backend.pull_commands()

        move_result = CommandResult()
        move_result.command_id = commands[0].command_id
        move_result.status = CommandStatus.SUCCEEDED

        update = FactUpdate()
        update.key = "(at uav1 sector_a)"
        update.value = True
        update.source = "perception:gps"
        update.authority = FactAuthorityLevel.CONFIRMED

        clear = FactUpdate()
        clear.key = "(at uav1 base)"
        clear.value = False
        clear.source = "perception:gps"
        clear.authority = FactAuthorityLevel.CONFIRMED

        observed = StateUpdate()
        observed.fact_updates = [clear, update]
        move_result.observed_updates = observed.fact_updates

        backend.push_command_result(move_result)
        metadata = wm.get_fact_metadata("(at uav1 sector_a)")
        self.assertEqual(metadata.authority, FactAuthority.CONFIRMED)
        self.assertEqual(metadata.source, "perception:gps")


if __name__ == "__main__":
    unittest.main()
