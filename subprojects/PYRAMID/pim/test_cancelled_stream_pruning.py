"""Regression test for a PR review finding on interaction_facade_gen.py: a
client-cancelled RPC stream stays StreamWriter::live() forever --
pcl_stream_cancel() only sets the underlying pcl_stream_context_t's
`cancelled` flag, it does not clear ctx_ -- so both generated open-stream
fan-out loops (RequestPortProvider::fanOutRpc() and
InformationPortSource::publish()) must prune a cancelled entry (and end()
it) before calling send() on it, not just check live().

RequestPortProvider::fanOutRpc()'s fix is covered end-to-end by
PclGeneratedInteractionFacadeProvider.FanOutRpcDropsCancelledStreamBeforeSending
(test_pcl_generated_interaction_facade.cpp), which uses a Query-typed
request with a registered codec. InformationPortSource::publish()'s
identical fix can't get the same runtime coverage in this repo: every
Data-1 Read rpc takes google.protobuf.Empty, and no generated JSON codec
plugin registers a codec for Empty (a gap that predates this PR and is out
of scope for it), so a real subscribe() call fails closed before ever
reaching the fan-out loop under test. This generates the A-GRA example
bindings and inspects the emitted publish() body directly instead.
"""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PIM_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "pim"
AGRA_ROOT = PIM_ROOT / "agra_example"


class CancelledStreamPruningTest(unittest.TestCase):
    def test_information_port_source_publish_prunes_cancelled_stream(self):
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            result = subprocess.run(
                [sys.executable, str(PIM_ROOT / "generate_bindings.py"),
                 str(AGRA_ROOT), str(out_dir),
                 "--languages", "cpp", "--backends", "json"],
                capture_output=True, text=True, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)

            components = out_dir / "pyramid_services_agra_mission_autonomy_provided_components.hpp"
            self.assertTrue(components.is_file(), f"not generated: {components}")
            text = components.read_text(encoding="utf-8")

            publish_start = text.index("pcl_status_t publish(const MAActionPlan_Service_Information& msg)")
            # The information port's RPC fan-out loop is self-contained
            # inside publish(); the next method definition ends the slice.
            publish_end = text.index("private:", publish_start)
            body = text[publish_start:publish_end]

            self.assertIn(
                "if (!it->live()) { it = open_streams_.erase(it); continue; }",
                body,
                "publish() must still prune a dropped (non-live) stream",
            )
            self.assertIn(
                "if (it->cancelled()) {",
                body,
                "publish() must separately prune a cancelled-but-live stream "
                "-- pcl_stream_cancel() does not clear live(), so relying on "
                "live() alone leaves a cancelled stream in open_streams_ "
                "forever, and every future publish() reports "
                "PCL_ERR_CANCELLED for it",
            )
            cancelled_idx = body.index("if (it->cancelled()) {")
            cancelled_block = body[cancelled_idx:body.index("}", body.index("}", cancelled_idx) + 1)]
            self.assertIn("it->end();", cancelled_block)
            self.assertIn("erase(it)", cancelled_block)

            # The cancelled check must run before send(), not after -- a
            # cancelled StreamWriter::send() call is exactly what returns
            # PCL_ERR_CANCELLED and pollutes last_rc.
            self.assertLess(
                body.index("if (it->cancelled())"), body.index("last_rc = it->send(msg)"),
                "the cancelled() check must be pruned before send() is called",
            )


if __name__ == "__main__":
    unittest.main()
