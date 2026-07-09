"""Regression test for the PCL.078 / REQ_PCL_470-472 follow-up (PR review on
doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md Phase 5): a
server-streaming rpc's *consumed*-side (client) route must be installed
under the distinct `PCL_ENDPOINT_STREAM_CONSUMED` kind, not the unary
`PCL_ENDPOINT_CONSUMED` every other rpc uses -- `pcl_executor_invoke_stream()`
only ever consults a route table entry keyed on the former.

Generates the A-GRA example bindings (its consumed-side MAAction_Service has
one server-streaming rpc, Read, alongside three unary ones) and inspects
`ConsumedService::routeAllRemote()`/`routeAllLocal()` directly, since this is
about what the generator emits, not runtime behaviour (covered separately,
end-to-end, by pim/test_harness/agra_seam_interchange_test.cpp and by
PCL's own test_pcl_transport_routing.cpp unit tests).
"""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PIM_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "pim"
AGRA_ROOT = PIM_ROOT / "agra_example"


class StreamConsumedRoutingTest(unittest.TestCase):
    def test_consumed_service_routes_streaming_rpc_under_stream_consumed_kind(self):
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            result = subprocess.run(
                [sys.executable, str(PIM_ROOT / "generate_bindings.py"),
                 str(AGRA_ROOT), str(out_dir),
                 "--languages", "cpp", "--backends", "json"],
                capture_output=True, text=True, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)

            components = out_dir / "pyramid_services_agra_c2_station_consumed_components.hpp"
            self.assertTrue(components.is_file(), f"not generated: {components}")
            text = components.read_text(encoding="utf-8")

            # Read (server-streaming) must route under the stream kind.
            read_block_start = text.index("Route every consumed endpoint to a named peer")
            # The three routeAllRemote()/routeAllLocal() bodies sit together;
            # scan the whole remainder for the per-rpc route calls rather than
            # trying to carve out one exact method body.
            body = text[read_block_start:]
            self.assertIn(
                "kSvcMaactionRead, peer_id, PCL_ENDPOINT_STREAM_CONSUMED", body,
                "Read's remote route call must pass PCL_ENDPOINT_STREAM_CONSUMED",
            )
            self.assertIn(
                "kSvcMaactionRead, PCL_ENDPOINT_STREAM_CONSUMED", body,
                "Read's no-arg remote/local route calls must pass "
                "PCL_ENDPOINT_STREAM_CONSUMED",
            )

            # Create/Update/Cancel (unary) must keep routing under the
            # ordinary consumed kind, not regress to the stream kind too.
            for svc_const in ("kSvcMaactionCreate", "kSvcMaactionUpdate",
                              "kSvcMaactionCancel"):
                self.assertIn(
                    f"{svc_const}, peer_id, PCL_ENDPOINT_CONSUMED)", body,
                    f"{svc_const}'s remote route call must stay PCL_ENDPOINT_CONSUMED",
                )


if __name__ == "__main__":
    unittest.main()
