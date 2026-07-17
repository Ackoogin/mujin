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
from binding_contract import (
    command_projectability_for_service,
    build_contract,
    interaction_for_service,
)

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


class InteractionModelTest(unittest.TestCase):
    """Tests for Phase 1's `Interaction`/`InteractionLeg`/`InteractionEndpoint`
    model (doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md, D5/§2.3).

    An `Interaction` groups the rpc (side A) and topic (side B) realizations
    of each leg of a Request or Information port -- purely a view over
    `topics_for_proto_service` + `command_projectability_for_service`, keyed
    the same way as `service_topics`/`command_projectability`.
    """

    def test_agra_request_service_yields_two_legs_with_expected_sides(self):
        files = parse_proto_tree(AGRA_ROOT)
        pf = next(
            pf for pf in files
            if pf.package == "pyramid.components.agra.mission_autonomy.services.provided"
        )
        service = next(s for s in pf.services if s.name == "MAAction_Service")
        index = ProtoTypeIndex(files)

        interaction = interaction_for_service(index, pf, service)
        self.assertIsNotNone(interaction)
        self.assertEqual(interaction.port_kind, "request")
        self.assertEqual(
            interaction.service_key,
            "pyramid.components.agra.mission_autonomy.services.provided.MAAction_Service",
        )

        legs_by_name = {leg.name: leg for leg in interaction.legs}
        self.assertEqual(set(legs_by_name), {"request", "requirement"})

        request_leg = legs_by_name["request"]
        self.assertEqual(
            request_leg.group_name,
            f"{interaction.service_key}.request_leg",
        )
        # Side A: the three command endpoints, all fully projectable
        # post-Phase-0 (the A-GRA example gained an `update` variant).
        side_a_names = {ep.endpoint_name for ep in request_leg.side_a}
        self.assertEqual(
            side_a_names, {"ma_action.create", "ma_action.update", "ma_action.cancel"}
        )
        for ep in request_leg.side_a:
            self.assertEqual(ep.kind, "provided")
            self.assertTrue(ep.projectable, (ep.endpoint_name, ep.reason))
        # Side B: the single `.request` topic endpoint.
        self.assertEqual(len(request_leg.side_b), 1)
        self.assertEqual(request_leg.side_b[0].endpoint_name, "agra.ma_action.request")
        self.assertEqual(request_leg.side_b[0].kind, "subscriber")

        requirement_leg = legs_by_name["requirement"]
        self.assertEqual(
            requirement_leg.group_name,
            f"{interaction.service_key}.requirement_leg",
        )
        self.assertEqual(len(requirement_leg.side_a), 1)
        self.assertEqual(requirement_leg.side_a[0].endpoint_name, "ma_action.read")
        self.assertEqual(requirement_leg.side_a[0].rpc_name, "Read")
        self.assertEqual(len(requirement_leg.side_b), 1)
        self.assertEqual(
            requirement_leg.side_b[0].endpoint_name, "agra.ma_action.requirement"
        )
        self.assertEqual(requirement_leg.side_b[0].kind, "publisher")

    def test_agra_information_service_yields_single_leg(self):
        files = parse_proto_tree(AGRA_ROOT)
        pf = next(
            pf for pf in files
            if pf.package == "pyramid.components.agra.mission_autonomy.services.provided"
        )
        service = next(s for s in pf.services if s.name == "MAActionPlan_Service")
        index = ProtoTypeIndex(files)

        interaction = interaction_for_service(index, pf, service)
        self.assertIsNotNone(interaction)
        self.assertEqual(interaction.port_kind, "information")
        self.assertEqual(len(interaction.legs), 1)

        leg = interaction.legs[0]
        self.assertEqual(leg.name, "information")
        self.assertEqual(leg.group_name, f"{interaction.service_key}.information_leg")
        self.assertEqual(len(leg.side_a), 1)
        self.assertEqual(leg.side_a[0].rpc_name, "Read")
        self.assertEqual(len(leg.side_b), 1)
        self.assertEqual(leg.side_b[0].endpoint_name, "agra.ma_action_plan.information")

    def test_build_contract_records_interactions(self):
        files = parse_proto_tree(AGRA_ROOT)
        contract = build_contract(files, "pyramid")
        key = "pyramid.components.agra.mission_autonomy.services.provided.MAAction_Service"
        self.assertIn(key, contract.interactions)
        self.assertEqual(contract.interactions[key].port_kind, "request")

    def test_pim_test_non_projectable_update_still_in_side_a(self):
        # D2/Phase 1: projectability gates whether the pub/sub side is safe
        # to use for a specific command -- it does not remove the command
        # from the rpc side's membership. SPRRequirement_Service's Update is
        # uniformly non-projectable across pim/test/ (see
        # PimTestUniformProjectabilityTest) and must still appear in the
        # request leg's side A, flagged non-projectable.
        pf = parse_proto(PIM_TEST_SPR_PROTO)
        service = next(s for s in pf.services if s.name == "SPRRequirement_Service")
        index = ProtoTypeIndex([pf])

        interaction = interaction_for_service(index, pf, service)
        self.assertIsNotNone(interaction)
        request_leg = next(leg for leg in interaction.legs if leg.name == "request")

        side_a_by_rpc = {ep.rpc_name: ep for ep in request_leg.side_a}
        self.assertEqual(set(side_a_by_rpc), {"Create", "Update", "Cancel"})

        update_ep = side_a_by_rpc["Update"]
        self.assertEqual(update_ep.endpoint_name, "spr_requirement.update")
        self.assertFalse(update_ep.projectable)
        self.assertEqual(update_ep.reason, "no_match")

        # Create/Cancel remain projectable in the same side A.
        self.assertTrue(side_a_by_rpc["Create"].projectable)
        self.assertTrue(side_a_by_rpc["Cancel"].projectable)

    def test_non_request_non_information_service_yields_no_interaction(self):
        # A service with no recognised port_kind (free-form / grammar-
        # nonconforming) is out of scope for this plan (D7) and must yield
        # no Interaction at all, not a partially-populated one.
        source = CONFLICTING_VARIANT_PROTO.replace(
            "service Widget_Service {",
            "service Widget_Service {\n  // no Create/Read/Update/Cancel shape below is altered;"
            " port_kind classification is unaffected by this comment.\n",
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "conflict.proto"
            path.write_text(source, encoding="utf-8")
            pf = parse_proto(path)
        service = next(s for s in pf.services if s.name == "Widget_Service")
        index = ProtoTypeIndex([pf])
        # Sanity: this fixture is still a request-shape port (unchanged from
        # ConflictingVariantProjectabilityTest) -- confirms an Interaction
        # *is* built for a normal request-shape service before testing the
        # negative case below via an explicit port_kind override.
        self.assertEqual(service.port_kind, "request")
        self.assertIsNotNone(interaction_for_service(index, pf, service))

        service.port_kind = "neutral"
        self.assertIsNone(interaction_for_service(index, pf, service))


class ComponentGroupTest(unittest.TestCase):
    """Tests for the component-spanning skeleton generation contract."""

    @classmethod
    def setUpClass(cls):
        cls.files = parse_proto_tree(PIM_TEST_ROOT / "pyramid")
        cls.contract = build_contract(cls.files, "pyramid")

    def test_component_count_and_keys_are_pinned(self):
        self.assertEqual(len(self.contract.components), 8)
        self.assertEqual(
            [group.key for group in self.contract.components],
            [
                "pim_osprey.sensor_data_interpretation",
                "pim_osprey.sensor_products",
                "pim_osprey.sensors",
                "pim_osprey.tactical_objects",
                "pim_seaspray.hmi_dialogue",
                "pim_seaspray.information_presentation",
                "pim_seaspray.sensor_products",
                "pim_seaspray.sensors",
            ],
        )

    def test_osprey_sensors_port_mapping(self):
        group = next(
            item
            for item in self.contract.components
            if item.key == "pim_osprey.sensors"
        )
        self.assertEqual(
            group.provided_package,
            "pyramid.components.pim_osprey.sensors.services.provided",
        )
        self.assertEqual(
            group.consumed_package,
            "pyramid.components.pim_osprey.sensors.services.consumed",
        )
        self.assertEqual(
            [
                (
                    port.port_key,
                    port.role,
                    port.port_kind,
                    port.facade,
                    port.has_interaction,
                )
                for port in group.ports
            ],
            [
                (
                    "authorisation_dependency_request",
                    "provided",
                    "request",
                    "AuthorisationDependencyRequestPortProvider",
                    True,
                ),
                (
                    "capability_evidence_information",
                    "consumed",
                    "information",
                    "CapabilityEvidenceInformationPortSink",
                    True,
                ),
                (
                    "capability_information",
                    "provided",
                    "information",
                    "CapabilityInformationPortSource",
                    True,
                ),
                (
                    "sen_requirement_request",
                    "provided",
                    "request",
                    "SenrequirementRequestPortProvider",
                    True,
                ),
            ],
        )

    def test_component_groups_are_deterministic(self):
        reversed_contract = build_contract(list(reversed(self.files)), "pyramid")
        self.assertEqual(reversed_contract.components, self.contract.components)

    def test_generic_layout_has_no_component_groups(self):
        contract = build_contract(self.files, "generic")
        self.assertEqual(contract.components, ())


if __name__ == "__main__":
    unittest.main()
