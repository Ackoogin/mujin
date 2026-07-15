"""Tests for the Windows-safe A-GRA P2 interaction overlay."""

import json
import shutil
import subprocess
import sys
import unittest
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from binding_contract import build_contract
from proto_parser import is_binding_proto, parse_proto_tree


PIM_DIR = Path(__file__).resolve().parent
SEAM = PIM_DIR / "agra_p2_seam"
GENERATED = PIM_DIR / "uci_generated" / "agra_5_0a"
OPTIONS = PIM_DIR / "uci_seam_example" / "pyramid" / "options" / "pyramid.options.proto"

DATA_MODEL_REL = Path("pyramid/data_model/pyramid.data_model.agra.proto")
OPTIONS_REL = Path("pyramid/options/pyramid.options.proto")

COMMAND_TOPICS = {
    "MA_MissionPlanCommand_Service": (
        "MA_MissionPlanCommand", "MA_MissionPlanCommandStatus"),
    "MA_MissionPlanActivationCommand_Service": (
        "MA_MissionPlanActivationCommand",
        "MA_MissionPlanActivationCommandStatus",
    ),
    "MA_PlanningFunctionSettingsCommand_Service": (
        "MA_PlanningFunctionSettingsCommand",
        "MA_PlanningFunctionSettingsCommandStatus",
    ),
    "MA_ApprovalRequest_Service": (
        "MA_ApprovalRequest", "MA_ApprovalRequestStatus"),
}

INFORMATION_TOPICS = {
    "MA_Action",
    "MA_MissionPlan",
    "MA_PlanningFunction",
    "MA_Response",
    "MA_Task",
    "MissionContingencyAlert",
    "MA_ActionStatus",
    "MA_MissionPlanActivationStatus",
    "MA_MissionPlanExecutionStatus",
    "MA_PlanningFunctionStatus",
    "MA_TaskStatus",
    "MA_ApprovalPolicy",
}

IDENTITY = {
    "drop": "agra_5_0a",
    "owp_init_schema": "005.0a.ASK",
    "schema_version": "005.0a.ASK-20260423-f1380e7",
}


class AgraP2SeamInputTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.files = [
            pf for pf in parse_proto_tree(SEAM) if is_binding_proto(pf)
        ]
        cls.services = {
            service.name: service
            for pf in cls.files
            for service in pf.services
        }
        cls.messages = {
            message.name: message
            for pf in cls.files
            for message in pf.messages
        }

    def _field_type(self, message_name, field_name):
        fields = {
            field.name: field.type
            for field in self.messages[message_name].all_fields()
        }
        return fields[field_name]

    def test_copied_generated_inputs_are_byte_identical(self):
        self.assertEqual(
            (SEAM / DATA_MODEL_REL).read_bytes(),
            (GENERATED / DATA_MODEL_REL).read_bytes(),
            "the overlay data model drifted from the checked-in P2 tree",
        )
        self.assertEqual(
            (SEAM / "wire_names.json").read_bytes(),
            (GENERATED / "wire_names.json").read_bytes(),
            "the overlay sidecar drifted; wire-name fallback can fail silently",
        )
        self.assertEqual(
            (SEAM / OPTIONS_REL).read_bytes(),
            OPTIONS.read_bytes(),
            "the overlay options proto drifted from the shared contract",
        )

    def test_complete_service_set_and_counts(self):
        self.assertEqual(len(self.services), 16)
        self.assertEqual(
            set(self.services),
            set(COMMAND_TOPICS)
            | {f"{topic}_Service" for topic in INFORMATION_TOPICS},
        )
        self.assertEqual(sum(len(pf.messages) for pf in self.files), 1192)
        self.assertEqual(sum(len(pf.enums) for pf in self.files), 297)

    def test_command_topics_and_qos(self):
        for service_name, (command_topic, status_topic) in COMMAND_TOPICS.items():
            service = self.services[service_name]
            self.assertEqual(service.port_kind, "request")
            self.assertEqual([rpc.name for rpc in service.rpcs],
                             ["Create", "Read", "Update", "Cancel"])
            for rpc in service.rpcs:
                expected_topic = status_topic if rpc.name == "Read" else command_topic
                expected_pattern = "PUBLISH" if rpc.name == "Read" else "SUBSCRIBE"
                self.assertEqual(rpc.topic, expected_topic)
                self.assertEqual(rpc.pattern, expected_pattern)
                self.assertEqual(
                    rpc.qos,
                    {
                        "reliability": "RELIABLE",
                        "durability": "VOLATILE",
                        "depth": 10,
                    },
                )

    def test_information_topics_and_qos(self):
        for topic in INFORMATION_TOPICS:
            service = self.services[f"{topic}_Service"]
            self.assertEqual(service.port_kind, "information")
            self.assertEqual(len(service.rpcs), 1)
            rpc = service.rpcs[0]
            self.assertEqual(rpc.name, "Read")
            self.assertEqual(rpc.topic, topic)
            self.assertEqual(rpc.pattern, "PUBLISH")
            self.assertEqual(
                rpc.qos,
                {
                    "reliability": "RELIABLE",
                    "durability": "VOLATILE",
                    "depth": 10,
                },
            )

    def test_exact_topics_still_produce_interaction_manifest_model(self):
        contract = build_contract(self.files, layout="pyramid")
        self.assertEqual(len(contract.interactions), 16)
        self.assertEqual(
            sorted(interaction.port_kind
                   for interaction in contract.interactions.values()).count("request"),
            4,
        )
        self.assertEqual(
            sorted(interaction.port_kind
                   for interaction in contract.interactions.values()).count("information"),
            12,
        )

    def test_command_status_pairs_share_the_documented_correlation_types(self):
        direct_pairs = (
            (
                "MA_MissionPlanCommandMDT",
                "MA_MissionPlanCommandStatusMDT",
                "MissionPlanCommandID_Type",
            ),
            (
                "MA_MissionPlanActivationCommandMDT",
                "MA_MissionPlanActivationCommandStatusMDT",
                "MissionPlanActivationCommandID_Type",
            ),
        )
        for command_mdt, status_mdt, identifier_type in direct_pairs:
            self.assertEqual(
                self._field_type(command_mdt, "command_id"), identifier_type)
            self.assertEqual(
                self._field_type(status_mdt, "command_id"), identifier_type)
            self.assertEqual(
                self._field_type(identifier_type, "base"), "ID_Type")

        self.assertEqual(
            self._field_type("MA_PlanningFunctionSettingsCommandMDT", "base"),
            "CommandBaseType",
        )
        self.assertEqual(
            self._field_type(
                "MA_PlanningFunctionSettingsCommandStatusMDT", "base"),
            "CommandStatusBaseType",
        )
        for base_type in ("CommandBaseType", "CommandStatusBaseType"):
            self.assertEqual(
                self._field_type(base_type, "command_id"), "CommandID_Type")

        self.assertEqual(
            self._field_type("MA_ApprovalRequestMDT", "base"),
            "RequestBaseType",
        )
        self.assertEqual(
            self._field_type("MA_ApprovalRequestStatusMDT", "base"),
            "RequestStatusBaseType",
        )
        for base_type in ("RequestBaseType", "RequestStatusBaseType"):
            self.assertEqual(
                self._field_type(base_type, "request_id"), "RequestID_Type")

        for identifier_type in ("CommandID_Type", "RequestID_Type"):
            self.assertEqual(
                self._field_type(identifier_type, "base"), "ID_Type")
        self.assertEqual(self._field_type("ID_Type", "uuid"), "bytes")


class AgraP2SeamGenerationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Python's TemporaryDirectory uses restrictive permissions that can
        # prevent a child process from traversing it in a native Windows
        # sandbox. A normal workspace directory keeps the smoke portable.
        cls.scratch = PIM_DIR / f".agra_p2_seam_test_{uuid.uuid4().hex}"
        cls.scratch.mkdir()
        cls.out = cls.scratch / "generated"
        cls.result = subprocess.run(
            [
                sys.executable,
                str(PIM_DIR / "generate_bindings.py"),
                str(SEAM),
                str(cls.out),
                "--languages",
                "cpp",
                "--backends",
                "oms_json",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        cls.manifest = json.loads(
            (cls.out / "binding_manifest.json").read_text(encoding="utf-8"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.scratch)

    def test_cpp_and_oms_json_generation_smoke(self):
        self.assertIn(
            "Found 4 proto files: 1192 messages, 297 enums, 16 services",
            self.result.stdout,
        )
        self.assertTrue(
            (self.out / "pyramid_data_model_agra_types.hpp").is_file())
        self.assertTrue(
            (
                self.out
                / "oms_json"
                / "cpp"
                / "pyramid_data_model_agra_oms_json_codec_plugin.cpp"
            ).is_file()
        )
        self.assertTrue(
            (self.out / "pyramid_services_agra_information_provided.cpp").is_file())
        self.assertTrue(
            (
                self.out
                / "pyramid_services_agra_mission_autonomy_provided.cpp"
            ).is_file()
        )

    def test_manifest_identity(self):
        self.assertEqual(self.manifest["metadata"], IDENTITY)

    def test_manifest_contains_all_topics_and_interactions(self):
        self.assertEqual(len(self.manifest["topics"]), 20)
        self.assertEqual(
            {topic["wire_name"] for topic in self.manifest["topics"]},
            INFORMATION_TOPICS
            | {value for pair in COMMAND_TOPICS.values() for value in pair},
        )
        self.assertEqual(len(self.manifest["interactions"]), 16)


if __name__ == "__main__":
    unittest.main()
