"""Tests for pim/agra_example/, the hand-authored A-GRA-vocabulary example
contract (doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md, Phase B).

Three things are pinned here:
  1. Both services (MAAction_Service, MAActionPlan_Service) classify
     correctly via the authoritative Layer-2 options path.
  2. The same services classify identically with the options stripped,
     exercising the Layer-1 classifier fallback on the same port grammar
     (4-rpc Request shape / 1-rpc Information shape).
  3. The vendored pyramid.options/base/common protos stay byte-identical to
     their pim/test/ originals, so the copies cannot drift silently (plan
     §3.1's checksum guard).
"""
import hashlib
import re
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import parse_proto_tree

REPO_ROOT = Path(__file__).resolve().parents[3]
AGRA_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "pim" / "agra_example"
MBSE_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "pim" / "test"

VENDORED = {
    "pyramid/options/pyramid.options.proto",
    "pyramid/data_model/pyramid.data_model.base.proto",
    "pyramid/data_model/pyramid.data_model.common.proto",
}

OPTION_BLOCK_RE = re.compile(
    r"\s*option \(pyramid\.options\.pyramid_op\) = \{.*?\};", re.DOTALL
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class VendoredProtoChecksumTest(unittest.TestCase):
    def test_vendored_copies_are_byte_identical_to_pim_test_originals(self):
        for rel in sorted(VENDORED):
            vendored = AGRA_ROOT / rel
            original = MBSE_ROOT / rel
            self.assertTrue(vendored.is_file(), f"missing vendored copy: {rel}")
            self.assertTrue(original.is_file(), f"missing pim/test original: {rel}")
            self.assertEqual(
                _sha256(vendored),
                _sha256(original),
                f"{rel} has drifted from its pim/test/ original -- re-vendor, "
                "never fork (plan §3.1 / risks)",
            )


class AgraExampleClassificationTest(unittest.TestCase):
    def test_both_services_classify_with_options_present(self):
        files = parse_proto_tree(AGRA_ROOT)
        services = {svc.name: svc for pf in files for svc in pf.services}

        self.assertIn("MAAction_Service", services)
        self.assertIn("MAActionPlan_Service", services)
        self.assertEqual(services["MAAction_Service"].port_kind, "request")
        self.assertEqual(services["MAActionPlan_Service"].port_kind, "information")

        # The BEST_EFFORT stamp on the information topic must survive into
        # ProtoRpc.qos (plan §3.3's load-bearing choice).
        info_rpc = services["MAActionPlan_Service"].rpcs[0]
        self.assertEqual(info_rpc.topic, "agra.ma_action_plan.information")
        self.assertEqual(info_rpc.qos["reliability"], "BEST_EFFORT")

        request_rpc = services["MAAction_Service"].rpcs[0]
        self.assertEqual(request_rpc.topic, "agra.ma_action.request")
        self.assertEqual(request_rpc.qos["reliability"], "RELIABLE")

    def test_both_services_classify_with_options_stripped(self):
        # Classifier fallback: strip every (pyramid.options.pyramid_op)
        # option block from a copy of the provided-side proto and confirm
        # classify_port_service still recognises the same two port shapes
        # from RPC signatures alone.
        provided = (
            AGRA_ROOT
            / "pyramid"
            / "components"
            / "pyramid.components.agra.mission_autonomy.services.provided.proto"
        )
        stripped_text = OPTION_BLOCK_RE.sub("", provided.read_text(encoding="utf-8"))
        self.assertNotIn("pyramid_op", stripped_text)

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            # classify_port_service only needs the rpc signatures; give the
            # stripped file its own throwaway package so it doesn't collide
            # with the real tree, and vendor just enough imports to parse.
            (root / "pyramid" / "options").mkdir(parents=True)
            (root / "pyramid" / "data_model").mkdir(parents=True)
            (root / "pyramid" / "components").mkdir(parents=True)
            for rel in VENDORED:
                dst = root / rel
                dst.write_bytes((AGRA_ROOT / rel).read_bytes())
            (root / "pyramid" / "data_model" / "pyramid.data_model.agra.proto").write_bytes(
                (AGRA_ROOT / "pyramid" / "data_model" / "pyramid.data_model.agra.proto").read_bytes()
            )
            (
                root
                / "pyramid"
                / "components"
                / "pyramid.components.agra.mission_autonomy.services.provided.proto"
            ).write_text(stripped_text, encoding="utf-8")

            files = parse_proto_tree(root)

        services = {svc.name: svc for pf in files for svc in pf.services}
        self.assertEqual(services["MAAction_Service"].port_kind, "request")
        self.assertEqual(services["MAActionPlan_Service"].port_kind, "information")
        # Fallback means no Layer-2 metadata is present at all.
        self.assertTrue(
            all(
                not (rpc.pattern or rpc.topic or rpc.qos)
                for svc in services.values()
                for rpc in svc.rpcs
            )
        )


if __name__ == "__main__":
    unittest.main()
