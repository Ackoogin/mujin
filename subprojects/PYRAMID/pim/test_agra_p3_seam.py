"""Tests for the generated A-GRA P3 (Core MMS) interaction seam.

The P3 seam differs from the hand-authored P1/P2 seams in one important
way: it is entirely produced by ``gen_interaction_seam.py`` from checked-in
inputs (the profile manifest, the Table 3-1 interface/direction data, and
the xsd2proto output tree). The strongest guard is therefore regeneration:
running the generator must reproduce the checked-in seam byte for byte.
The structural tests then pin the port-grammar properties a regeneration
bug could silently change while staying self-consistent.
"""

import json
import os
import shutil
import subprocess
import sys
import unittest
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import is_binding_proto, parse_proto_tree


PIM_DIR = Path(__file__).resolve().parent
SEAM = PIM_DIR / "agra_p3_seam"
GENERATED = PIM_DIR / "uci_generated_p3" / "agra_5_0a"
MANIFEST = PIM_DIR / "uci_profiles" / "p3_agra_core_mms.json"
INTERFACES = PIM_DIR / "uci_profiles" / "p3_agra_core_mms_interfaces.json"

DATA_MODEL_REL = Path(
    "pyramid/data_model/pyramid.data_model.agra.proto")
P2_SEAM = PIM_DIR / "agra_p2_seam"
P2_DATA_MODEL_REL = Path("pyramid/data_model/pyramid.data_model.agra.proto")

QOS = {"reliability": "RELIABLE", "durability": "VOLATILE", "depth": 10}

IDENTITY = {
    "drop": "agra_5_0a",
    "owp_init_schema": "005.0a.ASK",
    "schema_version": "005.0a.ASK-20260423-f1380e7",
}


def _tree_bytes(root: Path) -> dict:
    return {
        p.relative_to(root).as_posix(): p.read_bytes()
        for p in sorted(root.rglob("*")) if p.is_file()
    }


class AgraP3SeamRegenerationTest(unittest.TestCase):
    def test_generator_reproduces_the_checked_in_seam_byte_for_byte(self):
        scratch = PIM_DIR / f".agra_p3_seam_test_{uuid.uuid4().hex}"
        try:
            subprocess.run(
                [sys.executable, str(PIM_DIR / "gen_interaction_seam.py"),
                 "--out", str(scratch)],
                check=True, capture_output=True, text=True)
            checked_in = _tree_bytes(SEAM)
            regenerated = _tree_bytes(scratch)
            self.assertEqual(sorted(checked_in), sorted(regenerated))
            for rel in checked_in:
                self.assertEqual(
                    checked_in[rel], regenerated[rel],
                    f"{rel} drifted from what gen_interaction_seam.py "
                    "produces; regenerate the seam instead of hand-editing")
        finally:
            shutil.rmtree(scratch, ignore_errors=True)


class AgraP3SeamInputTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.table = json.loads(INTERFACES.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.wire_roots = json.loads(
            (SEAM / "wire_names.json").read_text(encoding="utf-8"))["roots"]
        cls.files = {
            pf.path.name: pf
            for pf in parse_proto_tree(SEAM) if is_binding_proto(pf)
        }

    def _component(self, iface: str, role: str):
        return self.files[
            f"pyramid.components.agra.{iface}.services.{role}.proto"]

    def test_copied_generated_inputs_are_byte_identical(self):
        self.assertEqual(
            (SEAM / DATA_MODEL_REL).read_bytes(),
            (GENERATED / DATA_MODEL_REL).read_bytes(),
            "the seam data model drifted from the checked-in P3 tree")
        self.assertEqual(
            (SEAM / "wire_names.json").read_bytes(),
            (GENERATED / "wire_names.json").read_bytes(),
            "the seam sidecar drifted; wire-name lookup can fail silently")

    def test_interface_table_union_matches_the_manifest_roots(self):
        union = set()
        for elements in self.table["interfaces"].values():
            union.update(elements)
        self.assertEqual(union, set(self.manifest["roots"]))

    def test_binding_metadata_carries_the_drop_identity(self):
        metadata = json.loads(
            (SEAM / "binding_metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata, IDENTITY)

    def test_p2_client_surface_is_retained_byte_for_byte(self):
        """P3 is an extension build: every P2 import remains available."""
        self.assertEqual(
            (SEAM / "pyramid/data_model/pyramid.data_model.agra.port_grammar.proto"
             ).read_bytes(),
            (P2_SEAM / "pyramid/data_model/pyramid.data_model.agra.port_grammar.proto"
             ).read_bytes())
        p2_components = P2_SEAM / "pyramid/components"
        for source in p2_components.glob("*.proto"):
            with self.subTest(source=source.name):
                self.assertEqual(
                    (SEAM / "pyramid/components" / source.name).read_bytes(),
                    source.read_bytes())

    def test_p3_data_model_contains_every_p2_message_and_enum(self):
        p2_file = next(
            pf for pf in parse_proto_tree(P2_SEAM)
            if pf.path.name == P2_DATA_MODEL_REL.name)
        p3_file = next(
            pf for pf in parse_proto_tree(SEAM)
            if pf.path.name == DATA_MODEL_REL.name)
        self.assertEqual(p3_file.package, p2_file.package)
        p3_messages = {message.name: message for message in p3_file.messages}
        p3_enums = {enum.name: enum for enum in p3_file.enums}
        for message in p2_file.messages:
            with self.subTest(message=message.name):
                self.assertEqual(p3_messages.get(message.name), message)
        for enum in p2_file.enums:
            with self.subTest(enum=enum.name):
                self.assertEqual(p3_enums.get(enum.name), enum)

    def test_p3_preserves_p2_wire_names(self):
        p2_wire = json.loads(
            (P2_SEAM / "wire_names.json").read_text(encoding="utf-8"))
        p3_wire = json.loads(
            (SEAM / "wire_names.json").read_text(encoding="utf-8"))
        self.assertEqual(p3_wire["package"], p2_wire["package"])
        for section in ("roots", "messages", "enums"):
            for name, mapping in p2_wire[section].items():
                with self.subTest(section=section, name=name):
                    self.assertEqual(p3_wire[section].get(name), mapping)

    def test_all_eight_components_exist_with_pinned_service_counts(self):
        counts = {
            (iface, role): len(self._component(iface, role).services)
            for iface in ("c2", "ms", "p2p", "vi")
            for role in ("provided", "consumed")
        }
        # Regression pin from the 2026-07-16 Table 3-1 parse. A change here
        # means either the table data or the derivation rules changed; both
        # must be deliberate.
        self.assertEqual(counts, {
            ("c2", "provided"): 179, ("c2", "consumed"): 49,
            ("ms", "provided"): 10, ("ms", "consumed"): 85,
            ("p2p", "provided"): 177, ("p2p", "consumed"): 177,
            ("vi", "provided"): 7, ("vi", "consumed"): 38,
        })
        self.assertEqual(sum(counts.values()), 722)

    def test_every_rpc_uses_the_approved_qos_and_a_real_element_topic(self):
        for pf in self.files.values():
            for service in pf.services:
                for rpc in service.rpcs:
                    self.assertEqual(rpc.qos, QOS,
                                     f"{service.name}.{rpc.name}")
                    self.assertIn(rpc.topic, self.wire_roots,
                                  f"{service.name}.{rpc.name} topic "
                                  f"{rpc.topic!r} is not an element name")

    def test_provided_command_pair_polarity(self):
        # MA_ActionCommand is C2 direction 'out': the C2 station commands,
        # the MA system executes -- so the provided side subscribes to the
        # command and publishes the status.
        service = {
            s.name: s for s in self._component("c2", "provided").services
        }["MA_ActionCommand_Service"]
        self.assertEqual([r.name for r in service.rpcs],
                         ["Create", "Read", "Update", "Cancel"])
        for rpc in service.rpcs:
            if rpc.name == "Read":
                self.assertEqual((rpc.pattern, rpc.topic),
                                 ("PUBLISH", "MA_ActionCommandStatus"))
            else:
                self.assertEqual((rpc.pattern, rpc.topic),
                                 ("SUBSCRIBE", "MA_ActionCommand"))

    def test_consumed_command_pair_polarity(self):
        # MA_FlightCommand is VI direction 'in': the MA system commands the
        # vehicle -- so the consumed side publishes the command and
        # subscribes to the status.
        service = {
            s.name: s for s in self._component("vi", "consumed").services
        }["MA_FlightCommand_Service"]
        for rpc in service.rpcs:
            if rpc.name == "Read":
                self.assertEqual((rpc.pattern, rpc.topic),
                                 ("SUBSCRIBE", "MA_FlightCommandStatus"))
            else:
                self.assertEqual((rpc.pattern, rpc.topic),
                                 ("PUBLISH", "MA_FlightCommand"))

    def test_inout_information_appears_on_both_sides(self):
        # Entity is C2 'inout': published by the provided side and
        # subscribed by the consumed side, one single-rpc service each.
        for role, pattern in (("provided", "PUBLISH"),
                              ("consumed", "SUBSCRIBE")):
            service = {
                s.name: s for s in self._component("c2", role).services
            }["Entity_Service"]
            self.assertEqual(len(service.rpcs), 1)
            rpc = service.rpcs[0]
            self.assertEqual((rpc.name, rpc.pattern, rpc.topic),
                             ("Read", pattern, "Entity"))

    def test_paired_statuses_do_not_also_appear_as_information(self):
        provided = {s.name for s in self._component("c2", "provided").services}
        self.assertIn("MA_ActionCommand_Service", provided)
        self.assertNotIn("MA_ActionCommandStatus_Service", provided,
                         "a paired status must ride its command service, "
                         "not get its own information service")


@unittest.skipUnless(
    os.environ.get("AGRA_P3_BINDINGS_SMOKE") == "1",
    "set AGRA_P3_BINDINGS_SMOKE=1 to run the full binding-generation smoke "
    "(over a minute: 2,856 messages, 722 services)")
class AgraP3SeamGenerationSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scratch = PIM_DIR / f".agra_p3_seam_smoke_{uuid.uuid4().hex}"
        cls.scratch.mkdir()
        cls.out = cls.scratch / "generated"
        cls.result = subprocess.run(
            [sys.executable, str(PIM_DIR / "generate_bindings.py"),
             str(SEAM), str(cls.out),
             "--languages", "cpp", "--backends", "json,oms_json"],
            check=True, capture_output=True, text=True)
        cls.manifest = json.loads(
            (cls.out / "binding_manifest.json").read_text(encoding="utf-8"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.scratch)

    def test_generation_smoke_and_identity(self):
        self.assertIn(
            "Found 12 proto files: 3204 messages, 501 enums, 738 services",
            self.result.stdout)
        self.assertEqual(self.manifest["metadata"], IDENTITY)
        self.assertTrue(
            (self.out / "oms_json" / "cpp" /
             "pyramid_data_model_agra_oms_json_codec_plugin.cpp"
             ).is_file())


if __name__ == "__main__":
    unittest.main()
