"""Tests for pim/xsd2proto.py (uci_mms_conversion_plan.md Phase 1).

Fixture-driven: the mapping-rule table, closure selection, determinism,
wire-name sidecar, and proto_parser compatibility are all pinned against
pim/test_xsd2proto_fixtures/uci_fixture.xsd, which needs no real schema
drop.  The final class exercises the real UCI 2.5 P1 conversion and its
reconciliation against the hand-authored uci_seam_example contract, and
SKIPs with a printed reason when no pinned XSD is available (the
Sleet/GNAT-absence pattern) -- run pim/schemas/fetch_schemas.py first in an
environment that can reach a source.
"""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import xsd2proto
from proto_parser import parse_proto_tree

PIM_DIR = Path(__file__).resolve().parent
FIXTURE_XSD = PIM_DIR / "test_xsd2proto_fixtures" / "uci_fixture.xsd"

FIXTURE_PROFILE = {
    "profile": "p_fixture",
    "drop": "test_fixture",
    "schema_version": "FIXTURE-1",
    "proto_package": "pyramid.data_model.uci_fixture",
    "roots": ["TestCommand", "TestCommandStatus"],
}


def convert_fixture(profile=None, lax=False):
    index = xsd2proto.SchemaIndex([FIXTURE_XSD], lax=lax)
    conv = xsd2proto.Converter(index, profile or FIXTURE_PROFILE)
    conv.convert()
    return conv


class NameConversionTest(unittest.TestCase):
    def test_snake_case_handles_acronym_runs(self):
        cases = {
            "SystemID": "system_id",
            "UUID": "uuid",
            "MessageData": "message_data",
            "OwnerProducer": "owner_producer",
            "LOS_Reference": "los_reference",
            "MA_Action": "ma_action",
            "ByWaypoint": "by_waypoint",
            "CommandProcessingState": "command_processing_state",
        }
        for xsd_name, expected in cases.items():
            self.assertEqual(xsd2proto.snake_case(xsd_name), expected, xsd_name)


class MappingRuleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.conv = convert_fixture()
        cls.proto = xsd2proto.emit_proto(cls.conv)
        cls.wire = json.loads(xsd2proto.emit_wire_names(cls.conv))

    def field(self, message, name):
        for fld in self.conv.messages[message].fields:
            if fld.proto_name == name:
                return fld
        self.fail(f"{message}.{name} not generated")

    def test_roots_resolve_to_mt_types(self):
        self.assertEqual(self.conv.roots,
                         {"TestCommand": "TestCommandMT",
                          "TestCommandStatus": "TestCommandStatusMT"})

    def test_extension_becomes_base_composition_field_one(self):
        base = self.field("TestCommandMT", "base")
        self.assertEqual(base.proto_type, "MessageType")
        self.assertEqual(base.number, 1)
        data = self.field("TestCommandMT", "message_data")
        self.assertEqual(data.proto_type, "TestCommandMDT")
        self.assertEqual(data.number, 2)

    def test_min_occurs_zero_maps_to_optional(self):
        self.assertEqual(self.field("HeaderType", "mission_id").label,
                         "optional")
        self.assertEqual(self.field("HeaderType", "system_id").label, "")
        self.assertEqual(self.field("TestCommandMDT", "priority").label,
                         "optional")

    def test_max_occurs_unbounded_maps_to_repeated(self):
        self.assertEqual(
            self.field("SecurityInformationType", "owner_producer").label,
            "repeated")
        self.assertEqual(self.field("TestCommandMDT", "target").label,
                         "repeated")

    def test_element_ref_keeps_referencing_particle_occurrences(self):
        # <xs:element ref="uci:Remark" minOccurs="0" maxOccurs="unbounded"/>:
        # the occurrence constraints live on the referencing particle, not
        # on the global declaration (Codex review of PR #120).
        remark = self.field("TestCommandMDT", "remark")
        self.assertEqual(remark.label, "repeated")
        self.assertEqual(remark.proto_type, "string")
        self.assertEqual(remark.wire_name, "Remark")

    def test_optional_sequence_group_propagates_to_children(self):
        # <xs:sequence minOccurs="0"> children default to minOccurs="1" but
        # are optional on the wire because the group may be absent
        # (Codex review of PR #120).
        self.assertEqual(self.field("TestCommandMDT", "grouped_label").label,
                         "optional")
        self.assertEqual(self.field("TestCommandMDT", "grouped_count").label,
                         "optional")

    def test_inline_choice_becomes_oneof_with_shared_numbering(self):
        by_label = self.field("TestCommandMDT", "by_label")
        by_number = self.field("TestCommandMDT", "by_number")
        self.assertEqual(by_label.oneof, "choice")
        self.assertEqual(by_number.oneof, "choice")
        # command_id=1, then choice members continue the message numbering.
        self.assertEqual(self.field("TestCommandMDT", "command_id").number, 1)
        self.assertEqual(by_label.number, 2)
        self.assertEqual(by_number.number, 3)

    def test_repeated_member_inside_choice_gets_list_wrapper(self):
        by_waypoint = self.field("TestCommandMDT", "by_waypoint")
        self.assertEqual(by_waypoint.oneof, "choice")
        self.assertEqual(by_waypoint.proto_type, "TestCommandMDT_ByWaypoint_List")
        wrapper = self.conv.messages["TestCommandMDT_ByWaypoint_List"]
        self.assertTrue(wrapper.synthesized)
        self.assertEqual(wrapper.fields[0].label, "repeated")
        self.assertEqual(wrapper.fields[0].proto_type, "string")

    def test_repeated_choice_gets_wrapper_message(self):
        choice_field = self.field("TargetType", "choice")
        self.assertEqual(choice_field.label, "repeated")
        self.assertEqual(choice_field.proto_type, "TargetType_Choice")
        wrapper = self.conv.messages["TargetType_Choice"]
        self.assertTrue(wrapper.synthesized)
        self.assertEqual({f.oneof for f in wrapper.fields}, {"choice"})

    def test_enum_gets_prefixed_values_and_unspecified_sentinel(self):
        self.assertIn("ModeEnum", self.conv.enums)
        self.assertIn("  MODE_ENUM_UNSPECIFIED = 0;", self.proto)
        self.assertIn("  MODE_ENUM_REAL = 1;", self.proto)
        self.assertIn("  PROCESSING_STATE_ENUM_REJECTED = 3;", self.proto)

    def test_simple_type_alias_collapses_to_scalar_with_facet_comment(self):
        uuid_field = self.field("SystemID_Type", "uuid")
        self.assertEqual(uuid_field.proto_type, "string")
        self.assertTrue(any("UUIDType" in c and "pattern" in c
                            for c in uuid_field.comment), uuid_field.comment)

    def test_datetime_maps_to_string_with_deviation_comment(self):
        ts = self.field("HeaderType", "timestamp")
        self.assertEqual(ts.proto_type, "string")
        self.assertTrue(any("dateTime" in c for c in ts.comment))

    def test_version_attribute_becomes_comment_not_field(self):
        names = [f.proto_name for f in self.conv.messages["MessageType"].fields]
        self.assertNotIn("version", names)
        self.assertTrue(any("version attribute" in c
                            for c in self.conv.messages["MessageType"].comment))

    def test_documentation_is_carried_as_comments(self):
        self.assertIn("// Common envelope carried by every message.",
                      self.proto)

    def test_closure_excludes_unprofiled_roots(self):
        self.assertNotIn("TestReportMT", self.conv.messages)
        self.assertNotIn("OrphanType", self.conv.messages)

    def test_wire_names_sidecar_carries_exact_element_names(self):
        fields = self.wire["messages"]["TestCommandMDT"]["fields"]
        self.assertEqual(fields["command_id"]["element"], "CommandID")
        self.assertEqual(fields["by_label"]["element"], "ByLabel")

    def test_required_repeated_rides_the_sidecar(self):
        # OwnerProducer: maxOccurs unbounded with minOccurs defaulting to 1
        # -- an empty list is schema-invalid, which proto3 'repeated' cannot
        # express, so the sidecar carries it (Codex review of PR #120).
        sec = self.wire["messages"]["SecurityInformationType"]["fields"]
        self.assertTrue(sec["owner_producer"].get("required"))
        # Target and the referenced Remark are minOccurs="0": no flag.
        mdt = self.wire["messages"]["TestCommandMDT"]["fields"]
        self.assertNotIn("required", mdt["target"])
        self.assertNotIn("required", mdt["remark"])
        header = self.wire["messages"]["HeaderType"]["fields"]
        self.assertEqual(header["mission_id"]["element"], "MissionID")
        uuid = self.wire["messages"]["SystemID_Type"]["fields"]["uuid"]
        self.assertEqual(uuid["element"], "UUID")
        self.assertEqual(self.wire["enums"]["ModeEnum"]["MODE_ENUM_REAL"],
                         "REAL")
        self.assertEqual(self.wire["roots"]["TestCommand"], "TestCommandMT")

    def test_closure_report_counts_and_fan_in(self):
        report = json.loads(xsd2proto.emit_closure_report(self.conv))
        self.assertEqual(report["counts"]["roots"], 2)
        self.assertEqual(report["counts"]["messages"],
                         len(self.conv.messages))
        # SystemID_Type is the fixture's hub type (CommandID twice, SystemID,
        # AreaID): fan-in must reflect that so pruning has data to work with.
        self.assertGreaterEqual(report["fan_in"]["SystemID_Type"], 4)
        self.assertEqual(report["skipped"], [])


class DeterminismAndCheckModeTest(unittest.TestCase):
    def test_regeneration_is_byte_identical(self):
        first = xsd2proto.outputs(convert_fixture())
        second = xsd2proto.outputs(convert_fixture())
        self.assertEqual(first, second)

    def test_check_mode_passes_on_fresh_output_and_fails_on_drift(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile_path = Path(tmp) / "p_fixture.json"
            profile_path.write_text(json.dumps(FIXTURE_PROFILE),
                                    encoding="utf-8")
            base_cmd = [sys.executable, str(PIM_DIR / "xsd2proto.py"),
                        str(profile_path), "--xsd", str(FIXTURE_XSD),
                        "--out", tmp]
            gen = subprocess.run(base_cmd, capture_output=True, text=True)
            self.assertEqual(gen.returncode, 0, gen.stderr)
            check = subprocess.run(base_cmd + ["--check"],
                                   capture_output=True, text=True)
            self.assertEqual(check.returncode, 0, check.stderr)
            proto = next((Path(tmp) / "test_fixture").rglob("*.proto"))
            proto.write_text(proto.read_text(encoding="utf-8") + "// drift\n",
                             encoding="utf-8")
            drifted = subprocess.run(base_cmd + ["--check"],
                                     capture_output=True, text=True)
            self.assertEqual(drifted.returncode, 1)
            self.assertIn("drift", drifted.stderr)


class GeneratedTreeParsesTest(unittest.TestCase):
    def test_output_parses_with_the_repo_proto_parser(self):
        conv = convert_fixture()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for rel, content in xsd2proto.outputs(conv).items():
                if not rel.endswith(".proto"):
                    continue
                path = root / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            files = parse_proto_tree(root)
        self.assertEqual(len(files), 1)
        pf = files[0]
        self.assertEqual(pf.package, "pyramid.data_model.uci_fixture")
        messages = {m.name: m for m in pf.messages}
        self.assertIn("TestCommandMT", messages)
        mdt = messages["TestCommandMDT"]
        self.assertEqual([oo.name for oo in mdt.oneofs], ["choice"])
        self.assertEqual({f.name for f in mdt.oneofs[0].fields},
                         {"by_label", "by_number", "by_waypoint"})
        enums = {e.name for e in pf.enums}
        self.assertEqual(enums, {"ModeEnum", "ProcessingStateEnum"})

    def test_strict_mode_rejects_unsupported_constructs(self):
        bad = FIXTURE_XSD.read_text(encoding="utf-8").replace(
            '<xs:element name="Ignored" type="xs:string"/>',
            '<xs:any processContents="lax"/>')
        with tempfile.TemporaryDirectory() as tmp:
            bad_path = Path(tmp) / "bad.xsd"
            bad_path.write_text(bad, encoding="utf-8")
            index = xsd2proto.SchemaIndex([bad_path], lax=False)
            conv = xsd2proto.Converter(index, {
                **FIXTURE_PROFILE, "roots": ["TestReport"]})
            with self.assertRaises(xsd2proto.ConversionError):
                conv.convert()
            # Same schema under --lax: skipped and recorded, not fatal.
            index = xsd2proto.SchemaIndex([bad_path], lax=True)
            conv = xsd2proto.Converter(index, {
                **FIXTURE_PROFILE, "roots": ["TestReport"]})
            conv.convert()
            report = json.loads(xsd2proto.emit_closure_report(conv))
            self.assertTrue(any("xs:any" in s for s in report["skipped"]))

    def test_missing_root_is_a_loud_error(self):
        index = xsd2proto.SchemaIndex([FIXTURE_XSD], lax=False)
        conv = xsd2proto.Converter(index, {
            **FIXTURE_PROFILE, "roots": ["NoSuchMessage"]})
        with self.assertRaises(xsd2proto.ConversionError) as ctx:
            conv.convert()
        self.assertIn("NoSuchMessage", str(ctx.exception))


class RealAgraP2ConversionTest(unittest.TestCase):
    """SKIP-gated: needs the pinned A-GRA 5.0a XSDs (fetch_schemas.py).

    Pins the result first demonstrated 2026-07-11: the full P2
    planning-core conversion runs strict-clean (no --lax) over the real
    8.6 MB drop, and its output parses with the repo proto parser."""

    @classmethod
    def setUpClass(cls):
        dl = PIM_DIR / "schemas" / "dl" / "agra_5_0a"
        cls.xsds = [dl / "A-GRA_MessageDefinitions_v5_0_a.xsd",
                    dl / "A-GRA_SecurityMarkings_v5_0_a.xsd"]
        if not all(p.is_file() for p in cls.xsds):
            raise unittest.SkipTest(
                "A-GRA 5.0a XSDs absent -- run pim/schemas/fetch_schemas.py "
                "agra_5_0a")
        cls.profile = json.loads(
            (PIM_DIR / "uci_profiles" / "p2_agra_planning_core.json")
            .read_text(encoding="utf-8"))

    def test_p2_converts_strict_and_parses(self):
        index = xsd2proto.SchemaIndex(self.xsds, lax=False)
        conv = xsd2proto.Converter(index, self.profile)
        conv.convert()  # strict: any unsupported construct raises
        self.assertEqual(len(conv.roots), len(self.profile["roots"]))
        report = json.loads(xsd2proto.emit_closure_report(conv))
        self.assertEqual(report["skipped"], [])
        # Closure scale recorded 2026-07-11: 1163 messages / 297 enums.
        # A pinned-drop re-run must reproduce it exactly; a changed count
        # on the same sha256-pinned bytes means converter behaviour drift.
        self.assertEqual(report["counts"]["messages"], 1163)
        self.assertEqual(report["counts"]["enums"], 297)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for rel, content in xsd2proto.outputs(conv).items():
                if not rel.endswith(".proto"):
                    continue
                path = root / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            files = parse_proto_tree(root)
        pf = files[0]
        self.assertEqual(pf.package, "pyramid.data_model.agra")
        self.assertEqual(len(pf.messages), 1163)
        self.assertEqual(len(pf.enums), 297)


def _find_real_uci_xsds():
    """Locate the pinned UCI 2.5 drop (both files), or None (callers SKIP).

    MessageDefinitions xs:includes SecurityMarkings, so the drop is only
    usable as a pair."""
    import os

    names = ("UCI_MessageDefinitions_v2_5_0.xsd",
             "UCI_SecurityMarkings_v2_5_0.xsd")
    dl = PIM_DIR / "schemas" / "dl" / "uci_2_5_0"
    if all((dl / n).is_file() for n in names):
        return [dl / n for n in names]
    env = os.environ.get("UCI_XSD_PATH")
    if env:
        root = Path(env)
        if root.is_file():
            root = root.parent
        if root.is_dir():
            hits = [sorted(root.rglob(n)) for n in names]
            if all(hits):
                return [h[0] for h in hits]
    return None


class RealUciP1ConversionTest(unittest.TestCase):
    """SKIP-gated: needs the pinned UCI 2.5 drop (fetch_schemas.py)."""

    @classmethod
    def setUpClass(cls):
        cls.xsds = _find_real_uci_xsds()
        if cls.xsds is None:
            raise unittest.SkipTest(
                "UCI 2.5 drop absent -- run pim/schemas/fetch_schemas.py "
                "uci_2_5_0 or set UCI_XSD_PATH")
        profile = json.loads(
            (PIM_DIR / "uci_profiles" / "p1_kitty_hawk.json").read_text(
                encoding="utf-8"))
        index = xsd2proto.SchemaIndex(cls.xsds, lax=False)
        cls.conv = xsd2proto.Converter(index, profile)
        cls.conv.convert()

    def test_all_p1_roots_resolve(self):
        self.assertEqual(
            sorted(self.conv.roots),
            ["ActionCommand", "ActionCommandStatus",
             "ObservationMeasurementReport", "PositionReport",
             "ServiceStatus", "SignalReport"])

    def test_p1_closure_scale_is_pinned(self):
        # Recorded 2026-07-11 on the sha256-pinned drop: a changed count on
        # the same bytes means converter behaviour drift.
        report = json.loads(xsd2proto.emit_closure_report(self.conv))
        self.assertEqual(report["skipped"], [])
        self.assertEqual(report["counts"]["messages"], 515)
        self.assertEqual(report["counts"]["enums"], 188)

    def test_checked_in_p1_tree_is_current(self):
        """The plan-D2 byte-stability guard: the committed
        pim/uci_generated/uci_2_5_0/ tree must equal a fresh conversion."""
        out_dir = PIM_DIR / "uci_generated" / "uci_2_5_0"
        for rel, content in xsd2proto.outputs(self.conv).items():
            path = out_dir / rel
            self.assertTrue(path.is_file(), f"missing checked-in {rel}")
            self.assertEqual(path.read_text(encoding="utf-8"), content,
                             f"{rel} drifted -- rerun xsd2proto.py "
                             "uci_profiles/p1_kitty_hawk.json and commit")

    def test_output_parses_with_proto_parser(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for rel, content in xsd2proto.outputs(self.conv).items():
                if not rel.endswith(".proto"):
                    continue
                path = root / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            files = parse_proto_tree(root)
        self.assertEqual(files[0].package, "pyramid.data_model.uci")

    def test_reconciliation_action_command_shape_vs_hand_contract(self):
        """Plan D6: the converted ActionCommand closure must carry the same
        wire elements the hand-authored uci_seam_example contract carries.
        Structural (wire-name-level) reconciliation, not byte equality --
        naming conventions differ by design and divergences are expected to
        be *documented*, which this test forces by failing on any element
        the hand contract encodes that the conversion lost."""
        wire = json.loads(xsd2proto.emit_wire_names(self.conv))
        root_msg = wire["roots"]["ActionCommand"]
        elements = {f["element"]
                    for f in wire["messages"][root_msg]["fields"].values()}
        # The hand contract's ActionCommand carries SecurityInformation,
        # MessageHeader, MessageData (pyramid.data_model.uci.proto); the
        # envelope may sit on a base type rather than the root message.
        flat = set(elements)
        for fld in self.conv.messages[root_msg].fields:
            if fld.proto_name == "base":
                flat |= {f["element"] for f in
                         wire["messages"][fld.proto_type]["fields"].values()}
        for required in ("SecurityInformation", "MessageHeader",
                         "MessageData"):
            self.assertIn(required, flat)


if __name__ == "__main__":
    unittest.main()
