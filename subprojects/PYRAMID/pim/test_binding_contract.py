"""Tests for binding_contract.py's D2 per-command projectability classification
(doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md, Phase 0).

Three cases, matching Phase 0's accept criteria:
  1. Fully-projectable: pim/agra_example/ post-Phase-0.1 fix (Create/Update/
     Cancel all classify projectable).
  2. The pre-existing uniform pim/test/ shape: Update non-projectable
     (`Update`'s request type is the `_Requirement` wrapper everywhere in
     the tree, never a `_Request` variant), Create/Cancel projectable.
  3. A synthetic ambiguous-variant negative: a wrapper whose oneof carries
     two variants of the same type, so a command matching that type cannot
     be resolved to a single variant and must classify not-projectable.
"""
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import parse_proto, parse_proto_tree, ProtoTypeIndex
from binding_contract import command_projectability_for_service, build_contract

REPO_ROOT = Path(__file__).resolve().parents[3]
AGRA_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "pim" / "agra_example"
PIM_TEST_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "pim" / "test"

PIM_TEST_SPR_PROTO = (
    PIM_TEST_ROOT
    / "pyramid"
    / "components"
    / "pyramid.components.pim_osprey.sensor_products.services.provided.proto"
)

# A synthetic Request-shape port whose wrapper has two oneof variants of the
# same type (Identifier), constructed to trigger D2's ambiguous-match case.
# Grammar-conforming enough for classify_port_service to accept it as
# port_kind "request" (Ack responses, Query/Identifier-typed Read/Cancel
# requests, Update's request type == Read's response type) without needing
# the pyramid.options plugin or any imports.
CONFLICTING_VARIANT_PROTO = """
syntax = "proto3";
package fixture.conflict;

message Ack {}
message Query {}
message Identifier {}
message Widget {}

message Widget_Service_Request {
  oneof payload {
    Widget widget = 1;
    Identifier cancel = 2;
    Identifier cancel_dup = 3;
  }
}

message Widget_Service_Requirement {
  oneof payload {
    Widget widget_status = 1;
  }
}

service Widget_Service {
  rpc Create(Widget_Service_Request) returns (Ack);
  rpc Read(Query) returns (stream Widget_Service_Requirement);
  rpc Update(Widget_Service_Requirement) returns (Ack);
  rpc Cancel(Identifier) returns (Ack);
}
"""


def _projectability_by_rpc(index, pf, service):
    return {
        p.rpc_name: p
        for p in command_projectability_for_service(index, pf, service)
    }


class AgraExampleProjectabilityTest(unittest.TestCase):
    """Case 1: fully-projectable, post-Phase-0.1 fix."""

    def _assert_fully_projectable(self, pf, service):
        index = ProtoTypeIndex([pf])
        by_rpc = _projectability_by_rpc(index, pf, service)

        self.assertEqual(set(by_rpc), {"Create", "Update", "Cancel"})
        self.assertTrue(all(p.projectable for p in by_rpc.values()), by_rpc)

        self.assertEqual(by_rpc["Create"].reason, "is_wrapper")
        self.assertEqual(by_rpc["Update"].reason, "matches_variant")
        self.assertEqual(by_rpc["Update"].variant_field, "update")
        self.assertEqual(by_rpc["Cancel"].reason, "matches_variant")
        self.assertEqual(by_rpc["Cancel"].variant_field, "cancel")

    def test_provided_side_fully_projectable(self):
        files = parse_proto_tree(AGRA_ROOT)
        pf = next(
            pf for pf in files
            if pf.package == "pyramid.components.agra.mission_autonomy.services.provided"
        )
        service = next(s for s in pf.services if s.name == "MAAction_Service")
        self._assert_fully_projectable(pf, service)

    def test_consumed_side_fully_projectable(self):
        files = parse_proto_tree(AGRA_ROOT)
        pf = next(
            pf for pf in files
            if pf.package == "pyramid.components.agra.c2_station.services.consumed"
        )
        service = next(s for s in pf.services if s.name == "MAAction_Service")
        self._assert_fully_projectable(pf, service)

    def test_build_contract_records_projectability(self):
        # build_contract() wires command_projectability_for_service into the
        # neutral BindingContract additively, keyed like service_topics.
        files = parse_proto_tree(AGRA_ROOT)
        contract = build_contract(files, "pyramid")
        key = "pyramid.components.agra.mission_autonomy.services.provided.MAAction_Service"
        self.assertIn(key, contract.command_projectability)
        self.assertTrue(
            all(p.projectable for p in contract.command_projectability[key])
        )


class PimTestUniformProjectabilityTest(unittest.TestCase):
    """Case 2: the pre-existing uniform pim/test/ shape (D2's grammar-wide
    finding) -- Update non-projectable, Create/Cancel projectable, on a real
    pim/test/ contract (SPRRequirement_Service)."""

    def test_update_non_projectable_create_cancel_projectable(self):
        pf = parse_proto(PIM_TEST_SPR_PROTO)
        service = next(s for s in pf.services if s.name == "SPRRequirement_Service")
        self.assertEqual(service.port_kind, "request")

        index = ProtoTypeIndex([pf])
        by_rpc = _projectability_by_rpc(index, pf, service)

        self.assertTrue(by_rpc["Create"].projectable)
        self.assertEqual(by_rpc["Create"].reason, "is_wrapper")

        self.assertFalse(by_rpc["Update"].projectable)
        self.assertEqual(by_rpc["Update"].reason, "no_match")

        self.assertTrue(by_rpc["Cancel"].projectable)
        self.assertEqual(by_rpc["Cancel"].reason, "matches_variant")
        self.assertEqual(by_rpc["Cancel"].variant_field, "cancel")

    def test_uniform_across_whole_pim_test_tree(self):
        # D2's grammar-wide fact, checked directly rather than taken on
        # faith: every Request-shape service in pim/test/ has Update
        # non-projectable and Create/Cancel projectable.
        files = parse_proto_tree(PIM_TEST_ROOT)
        index = ProtoTypeIndex(files)
        request_services = [
            (pf, svc)
            for pf in files
            for svc in pf.services
            if svc.port_kind == "request"
        ]
        self.assertGreater(len(request_services), 0)

        for pf, svc in request_services:
            by_rpc = _projectability_by_rpc(index, pf, svc)
            self.assertTrue(
                by_rpc["Create"].projectable,
                f"{pf.package}.{svc.name}.Create expected projectable",
            )
            self.assertFalse(
                by_rpc["Update"].projectable,
                f"{pf.package}.{svc.name}.Update expected non-projectable",
            )
            self.assertTrue(
                by_rpc["Cancel"].projectable,
                f"{pf.package}.{svc.name}.Cancel expected projectable",
            )


class ConflictingVariantProjectabilityTest(unittest.TestCase):
    """Case 3: synthetic ambiguous-variant negative -- a wrapper with two
    oneof variants of the same type must not silently pick one; the command
    whose request type collides classifies not-projectable."""

    def test_ambiguous_variant_is_not_projectable(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "conflict.proto"
            path.write_text(CONFLICTING_VARIANT_PROTO, encoding="utf-8")
            pf = parse_proto(path)

        service = next(s for s in pf.services if s.name == "Widget_Service")
        # Grammar-conforming enough for the Layer-1 classifier fallback to
        # recognise it as a Request-shape port with no options present.
        self.assertEqual(service.port_kind, "request")

        index = ProtoTypeIndex([pf])
        by_rpc = _projectability_by_rpc(index, pf, service)

        self.assertTrue(by_rpc["Create"].projectable)
        self.assertEqual(by_rpc["Create"].reason, "is_wrapper")

        # Cancel's request type (Identifier) matches *two* oneof variants
        # (cancel, cancel_dup) -- ambiguous, not projectable.
        self.assertFalse(by_rpc["Cancel"].projectable)
        self.assertEqual(by_rpc["Cancel"].reason, "ambiguous_variant")
        self.assertEqual(by_rpc["Cancel"].variant_field, "")

        # Update's request type (Widget_Service_Requirement) matches no
        # variant at all -- ordinary no_match, same as the uniform pim/test/
        # shape.
        self.assertFalse(by_rpc["Update"].projectable)
        self.assertEqual(by_rpc["Update"].reason, "no_match")

    def test_no_wrapper_message_is_not_projectable(self):
        # A Request-shape service whose wrapper message doesn't exist under
        # the naming convention at all (D2's "wrapper doesn't exist" case).
        # classify_port_service only inspects rpc signatures, not whether a
        # <ServiceName>_Request message happens to exist, so this parses and
        # classifies as port_kind "request" with no wrapper resolvable.
        source = CONFLICTING_VARIANT_PROTO.replace(
            "message Widget_Service_Request {",
            "message Renamed_Wrapper_Does_Not_Match {",
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "no_wrapper.proto"
            path.write_text(source, encoding="utf-8")
            pf = parse_proto(path)

        service = next(s for s in pf.services if s.name == "Widget_Service")
        self.assertEqual(service.port_kind, "request")

        index = ProtoTypeIndex([pf])
        by_rpc = _projectability_by_rpc(index, pf, service)

        self.assertTrue(
            all(not p.projectable for p in by_rpc.values()), by_rpc
        )
        self.assertTrue(all(p.reason == "no_wrapper" for p in by_rpc.values()))


if __name__ == "__main__":
    unittest.main()
