"""Tests for the generalized OMS-JSON codec generator (UCI MMS plan Phase 2).

The regression bar (plan D4(b)): the emitter's output for the hand-authored
``pim/uci_seam_example`` contract is frozen -- byte-identical to the golden
fixture captured from the pre-generalization emitter, whose wire behaviour
was verified against real Sleet (byte-equivalent to the hand-written codec,
schema-accepted).  Everything new -- sidecar wire names, enums, oneofs,
nested-base flattening, omit-empty arrays -- rides the sidecar path, which
the seam tree never takes.

The compile-and-run smoke (encode -> decode -> re-encode over the generated
P1 codec) was demonstrated 2026-07-11 with g++ 13 and is repeatable via
OMS_JSON_COMPILE_SMOKE=1 in an environment with a C++17 toolchain and a
nlohmann/json single header (see OmsJsonCompileSmokeTest).
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import parse_proto_tree, ProtoTypeIndex
from cpp.oms_json_codec_gen import CppOmsJsonCodecGenerator, OmsJsonShapeError
from ada.oms_json_codec_gen import AdaOmsJsonCodecGenerator
import xsd2proto

PIM_DIR = Path(__file__).resolve().parent
SEAM_TREE = PIM_DIR / "uci_seam_example"
P1_TREE = PIM_DIR / "uci_generated" / "uci_2_5_0"
GOLDEN = PIM_DIR / "test_oms_json_gen_fixtures" / "seam_codec_plugin.golden.cpp"


def generate_cpp(tree: Path, out: Path):
    files = parse_proto_tree(tree)
    return CppOmsJsonCodecGenerator(ProtoTypeIndex(files)).generate(out)


class SeamRegressionTest(unittest.TestCase):
    def test_seam_output_is_byte_identical_to_golden(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = generate_cpp(SEAM_TREE, Path(tmp))
            self.assertEqual(len(paths), 1)
            self.assertEqual(paths[0].name,
                             "pyramid_data_model_uci_oms_json_codec_plugin.cpp")
            self.assertEqual(
                paths[0].read_text(encoding="utf-8"),
                GOLDEN.read_text(encoding="utf-8"),
                "seam emitter output drifted from the Sleet-verified golden "
                "-- the heuristic path is frozen (plan D4(b)); new behaviour "
                "belongs on the sidecar path")

    def test_ada_seam_emitter_still_emits_the_seam_contract(self):
        files = parse_proto_tree(SEAM_TREE)
        gen = AdaOmsJsonCodecGenerator(ProtoTypeIndex(files))
        with tempfile.TemporaryDirectory() as tmp:
            paths = gen.generate(Path(tmp))
        self.assertEqual(sorted(p.name for p in paths),
                         ["pyramid-data_model-uci-oms_json_codec.adb",
                          "pyramid-data_model-uci-oms_json_codec.ads"])


class P1SidecarGenerationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.paths = generate_cpp(P1_TREE, Path(cls.tmp.name))
        cls.text = cls.paths[0].read_text(encoding="utf-8")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_one_plugin_for_the_p1_package(self):
        self.assertEqual([p.name for p in self.paths],
                         ["pyramid_data_model_uci_oms_json_codec_plugin.cpp"])

    def test_roots_get_dual_id_dispatch(self):
        # Element name (transport/OWP identity) and proto message name
        # (facade schema identity) both resolve to the same handler.
        self.assertIn('std::strcmp(id,"ActionCommandStatus")==0 || '
                      'std::strcmp(id,"ActionCommandStatusMT")==0', self.text)
        self.assertIn('json({{"PositionReport",encode_PositionReportMT(n)}})',
                      self.text)

    def test_sidecar_wire_names_beat_the_heuristic(self):
        # Names the snake->Pascal heuristic provably mangles (191 such
        # fields measured in P1) must appear verbatim from wire_names.json.
        for wire_name in ('"CUI_Basic"', '"SAR_Identifier"',
                          '"FGI_SourceOpen"', '"NonIC_Markings"'):
            self.assertIn(wire_name, self.text, wire_name)

    def test_enums_use_xsd_literals_and_reject_unspecified(self):
        self.assertIn("wire_ClassificationEnum", self.text)
        self.assertIn('case uci::ClassificationEnum::Ts: return "TS";',
                      self.text)
        self.assertNotIn('return "UNSPECIFIED"', self.text)
        self.assertIn("enum value not on the UCI wire", self.text)

    def test_nested_base_is_flattened_on_the_wire(self):
        # ActionCommandStatusMDT retains a CommandStatusBaseType base member
        # (2-deep extension chain); its fields merge into the parent object.
        self.assertIn("o.update(encode_CommandStatusBaseType(m.base));",
                      self.text)
        self.assertIn("r.base = decode_CommandStatusBaseType(j);", self.text)

    def test_optional_repeated_arrays_are_omitted_when_empty(self):
        self.assertIn('if (!m.activity.empty()) o["Activity"]', self.text)

    def test_no_wrapper_structs_for_a_serviceless_tree(self):
        self.assertNotIn("_Service_", self.text)
        self.assertNotIn("Wire {", self.text)


class RepeatedChoiceCarrierTest(unittest.TestCase):
    def test_repeated_choice_carrier_fails_loud(self):
        """xsd2proto -> codec-gen end to end: the fixture schema's TargetType
        carries a repeated xs:choice, which the emitter does not yet put on
        the wire -- generation must fail with a clear shape error, never
        emit silently wrong JSON."""
        fixture_profile = {
            "profile": "p_fixture", "drop": "test_fixture",
            "schema_version": "FIXTURE-1",
            "proto_package": "pyramid.data_model.uci_fixture",
            "roots": ["TestCommand", "TestCommandStatus"],
        }
        index = xsd2proto.SchemaIndex(
            [PIM_DIR / "test_xsd2proto_fixtures" / "uci_fixture.xsd"],
            lax=False)
        conv = xsd2proto.Converter(index, fixture_profile)
        conv.convert()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for rel, content in xsd2proto.outputs(conv).items():
                path = root / ("test_fixture/" + rel if rel.endswith(".json")
                               else "test_fixture/" + rel)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            with self.assertRaises(OmsJsonShapeError) as ctx:
                generate_cpp(root / "test_fixture", Path(tmp) / "out")
        self.assertIn("TargetType", str(ctx.exception))


class AdaGuardTest(unittest.TestCase):
    def test_ada_emitter_skips_non_seam_uci_packages(self):
        # The Ada template hard-references the seam contract's shapes; fed
        # the xsd2proto-generated P1 tree it must skip, not emit
        # non-compiling Ada.
        files = parse_proto_tree(P1_TREE)
        gen = AdaOmsJsonCodecGenerator(ProtoTypeIndex(files))
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(gen.generate(Path(tmp)), [])


def _find_nlohmann():
    env = os.environ.get("NLOHMANN_JSON_INCLUDE")
    if env and (Path(env) / "nlohmann" / "json.hpp").is_file():
        return Path(env)
    return None


class OmsJsonCompileSmokeTest(unittest.TestCase):
    """Opt-in (slow: ~1 min of template instantiation): compile the
    generated P1 plugin and run an encode->decode->re-encode round trip.
    Demonstrated green 2026-07-11 (g++ 13, nlohmann 3.11.3):
    dual-id dispatch, sidecar names, enum literals, nested-base flattening,
    oneof choice, omit-empty arrays, byte-stable round trip."""

    @classmethod
    def setUpClass(cls):
        if os.environ.get("OMS_JSON_COMPILE_SMOKE") != "1":
            raise unittest.SkipTest("set OMS_JSON_COMPILE_SMOKE=1 to run")
        if shutil.which("g++") is None:
            raise unittest.SkipTest("g++ not available")
        cls.nlohmann = _find_nlohmann()
        if cls.nlohmann is None:
            raise unittest.SkipTest(
                "set NLOHMANN_JSON_INCLUDE to a dir containing "
                "nlohmann/json.hpp")

    def test_p1_codec_compiles_and_round_trips(self):
        repo = PIM_DIR.parents[2]
        driver = PIM_DIR / "test_oms_json_gen_fixtures" / "p1_smoke_driver.cpp"
        with tempfile.TemporaryDirectory() as tmp:
            gen = Path(tmp) / "gen"
            subprocess.run(
                [sys.executable, str(PIM_DIR / "generate_bindings.py"),
                 str(P1_TREE), str(gen), "--languages", "cpp",
                 "--backends", "oms_json"],
                check=True, capture_output=True)
            exe = Path(tmp) / "driver"
            compile_cmd = [
                "g++", "-std=c++17", "-o", str(exe), str(driver),
                str(gen / "oms_json" / "cpp" /
                    "pyramid_data_model_uci_oms_json_codec_plugin.cpp"),
                str(gen / "pyramid_data_model_uci_cabi_marshal.cpp"),
                f"-I{gen}", f"-I{self.nlohmann}",
                f"-I{repo / 'subprojects' / 'PCL' / 'include'}",
                f"-I{repo / 'subprojects' / 'PYRAMID' / 'core' / 'external'}",
            ]
            build = subprocess.run(compile_cmd, capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr[-4000:])
            run = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn("RT_OK", run.stdout)
            wire = json.loads(run.stdout.splitlines()[0])
            root = wire["ActionCommandStatus"]
            self.assertEqual(
                root["MessageData"]["CommandProcessingState"], "RECEIVED")
            self.assertEqual(
                root["SecurityInformation"]["OwnerProducer"][0]
                    ["GovernmentIdentifier"], "USA")
            self.assertNotIn("Activity", root["MessageData"])  # omit-empty


if __name__ == "__main__":
    unittest.main()
