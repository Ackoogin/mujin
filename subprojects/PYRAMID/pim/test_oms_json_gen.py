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

import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import parse_proto_tree, ProtoTypeIndex
from cpp.naming import _DEFAULT_NAMING_POLICY
from cpp.oms_json_codec_gen import (
    CppOmsJsonCodecGenerator,
    OmsJsonShapeError,
    _PackageEmitter,
)
from cpp.types_gen import find_scalar_wrappers
from ada.oms_json_codec_gen import AdaOmsJsonCodecGenerator
import xsd2proto

PIM_DIR = Path(__file__).resolve().parent
SEAM_TREE = PIM_DIR / "uci_seam_example"
P1_TREE = PIM_DIR / "uci_generated" / "uci_2_5_0"
P2_TREE = PIM_DIR / "uci_generated" / "agra_5_0a"
# The A-GRA P2 contract as packaged: the generated data model above plus the
# interaction overlay and the drop identity in binding_metadata.json.
P2_CONTRACT = PIM_DIR / "agra_p2_seam"
REPEATED_CHOICE_TREE = (
    PIM_DIR / "test_oms_json_gen_fixtures" / "repeated_choice_member"
)
GOLDEN = PIM_DIR / "test_oms_json_gen_fixtures" / "seam_codec_plugin.golden.cpp"


def _p2_closure_counts():
    report = json.loads((P2_TREE / "closure_report.json").read_text(
        encoding="utf-8"))
    return report["counts"]


def generate_cpp(tree: Path, out: Path):
    files = parse_proto_tree(tree)
    return CppOmsJsonCodecGenerator(ProtoTypeIndex(files)).generate(out)


def _shape_error_category(message: str) -> str:
    """Stable inventory categories for the P2-wide shape probe."""
    if "repeated xs:choice carriers" in message:
        return "repeated choice carrier"
    if "scalar-wrapper alias" in message:
        return "scalar-wrapper alias"
    if "extension base" in message and "oneof" in message:
        return "extension base carrying oneof"
    if "unresolved type" in message:
        return "unresolved type"
    return "other"


def _probe_cpp_shapes(files):
    """Attempt both per-message emission paths and inventory every gap.

    This deliberately includes synthesized messages.  Although normal full
    generation handles their list payload inline and skips standalone codec
    functions, probing all parsed messages catches sidecar or wrapper drift.
    """
    index = ProtoTypeIndex(files)
    aliases = find_scalar_wrappers(index)
    inventory = {}
    attempts = 0
    messages = 0
    for pf in files:
        if not pf.messages or pf.services:
            continue
        emitter = _PackageEmitter(index, pf, _DEFAULT_NAMING_POLICY, aliases)
        for msg in pf.messages:
            messages += 1
            for operation in (emitter._encode, emitter._decode):
                attempts += 1
                try:
                    operation(io.StringIO(), msg)
                except OmsJsonShapeError as exc:
                    category = _shape_error_category(str(exc))
                    entry = inventory.setdefault(category, {
                        "messages": set(), "details": set(),
                    })
                    entry["messages"].add(msg.name)
                    entry["details"].add(str(exc))
    return messages, attempts, inventory


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

    def test_uci_uuids_keep_the_hyphenated_rfc_4122_form(self):
        # UCI 2.5 types UUID as xs:string carrying the RFC-4122 pattern, so
        # it converts to `string` and must use the 36-character validator.
        # The hexBinary validator must not even be emitted for this tree.
        self.assertIn("bool uuid(const std::string& s) { if (s.size() != 36)",
                      self.text)
        self.assertNotIn("uuid_hex", self.text)

    def test_nested_base_is_flattened_on_the_wire(self):
        # ActionCommandStatusMDT retains a CommandStatusBaseType base member
        # (2-deep extension chain); its fields merge into the parent object.
        self.assertIn("o.update(encode_CommandStatusBaseType(m.base));",
                      self.text)
        self.assertIn("r.base = decode_CommandStatusBaseType(j);", self.text)

    def test_optional_repeated_arrays_are_omitted_when_empty(self):
        self.assertIn('if (!m.activity.empty()) o["Activity"]', self.text)

    def test_required_repeated_arrays_are_enforced(self):
        # ActionCommandMDT.Command is minOccurs>=1: encoding an empty list
        # or decoding a document without it must fail, not silently pass
        # (Codex review of PR #120).
        self.assertIn(
            'if (m.command.empty()) throw json::type_error::create(302,'
            '"required repeated element Command is empty"', self.text)
        # Decode stays strict: no contains-guard on the required element.
        self.assertNotIn('if (j.contains("Command")) r.command', self.text)

    def test_multiple_active_choice_arms_are_rejected(self):
        # types_gen's independent optionals cannot enforce xs:choice's
        # exactly-one; the codec must (Codex review of PR #120).
        self.assertIn('multiple active choice arms in', self.text)
        self.assertIn('(m.government_identifier ? 1 : 0) + '
                      '(m.nato_special_word ? 1 : 0)', self.text)

    def test_no_wrapper_structs_for_a_serviceless_tree(self):
        self.assertNotIn("_Service_", self.text)
        self.assertNotIn("Wire {", self.text)


class RepeatedChoiceMemberFixtureTest(unittest.TestCase):
    """Pinned A-GRA repeated-choice-member wire shape for both emitters.

    la-cal-harness, driven by A-GRA_MessageDefinitions_v5_0_a.xsd, emits
    PolygonPointChoiceType as ``{"Point2D": [...]}`` and
    LinePointChoiceType as ``{"Point": [...]}``.  The fixture is the small
    equivalent: the selected arm's element name owns the JSON array directly;
    the synthesized ``*_List`` message never adds an object level.
    """

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        files = parse_proto_tree(REPEATED_CHOICE_TREE)
        index = ProtoTypeIndex(files)
        cpp_paths = CppOmsJsonCodecGenerator(index).generate(
            Path(cls.tmp.name) / "cpp")
        ada_paths = AdaOmsJsonCodecGenerator(index).generate(
            Path(cls.tmp.name) / "ada")
        cls.cpp = cpp_paths[0].read_text(encoding="utf-8")
        cls.ada = next(p for p in ada_paths if p.suffix == ".adb") \
            .read_text(encoding="utf-8")
        cls.wire = json.loads((REPEATED_CHOICE_TREE /
                               "expected_wire_shape.json").read_text(
                                   encoding="utf-8"))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_fixture_pins_member_key_to_array_without_wrapper_object(self):
        choice = self.wire["ChoiceFixture"]["PointChoice"]
        self.assertEqual(list(choice), ["Point"])
        self.assertIsInstance(choice["Point"], list)
        self.assertEqual(choice["Point"], [{"X": 1.0}, {"X": 2.0}])

    def test_cpp_emits_and_decodes_the_member_array_inline(self):
        self.assertIn(
            'for(const auto& x:(*m.point).items) '
            'a.push_back(encode_PointType(x))', self.cpp)
        self.assertIn(
            'if (j.contains("Point")) { '
            'uci::PointChoiceType_Point_List w{}; for (const auto& x : '
            'j.at("Point")) w.items.push_back(decode_PointType(x)); '
            'r.point = w; }', self.cpp)
        self.assertNotIn("encode_PointChoiceType_Point_List", self.cpp)

    def test_ada_emits_the_member_array_inline(self):
        self.assertIn(
            "if M.Has_Point and then M.Point.Items /= null and then "
            "M.Point.Items'Length > 0 then", self.ada)
        self.assertIn(
            "Append (A, Encode_Point_Type (M.Point.Items (I)));", self.ada)
        self.assertIn('Add (B, "Point", To_String (V));', self.ada)
        self.assertNotIn("Encode_Point_Choice_Type_Point_List", self.ada)

    def test_both_emitters_reject_multiple_active_choice_arms(self):
        self.assertIn(
            "(m.point ? 1 : 0) + (m.relative ? 1 : 0)", self.cpp)
        self.assertIn(
            "Boolean'Pos (M.Has_Point) + Boolean'Pos (M.Has_Relative)",
            self.ada)


class AgraUuidFormTest(unittest.TestCase):
    """The A-GRA UUID lexical form, pinned on the cheap generator path.

    Deliberately not behind the P2 closure-probe gate below: a UUID
    validator that does not match the drop rejects every schema-valid
    instance of it, which is exactly the failure this guards, and the
    generation it needs costs a fraction of a second."""

    @classmethod
    def setUpClass(cls):
        if not P2_TREE.is_dir():
            raise unittest.SkipTest("P2 generated tree is absent")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.text = generate_cpp(P2_TREE, Path(cls.tmp.name))[0].read_text(
            encoding="utf-8")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_agra_uuids_use_the_hexbinary_form(self):
        # A-GRA 5.0a types UUID as xs:hexBinary length 16, which converts to
        # `bytes`.  Validating those against the UCI hyphenated form rejected
        # every schema-valid A-GRA instance until the validator followed the
        # converted type.
        self.assertIn(
            "bool uuid_hex(const std::string& s) { if (s.size() != 32)",
            self.text)
        self.assertIn("if(!uuid_hex(s))", self.text)

    def test_the_hyphenated_uci_validator_is_absent_from_the_agra_codec(self):
        # No A-GRA UUID field converts to `string`, so the 36-character
        # validator must not be emitted at all for this tree.
        self.assertNotIn("if (s.size() != 36)", self.text)
        self.assertNotIn("if(!uuid(s))", self.text)


@unittest.skipUnless(
    os.environ.get("OMS_JSON_P2_GENERATION") == "1",
    "set OMS_JSON_P2_GENERATION=1 to run the full P2 closure probe",
)
class P2SidecarGenerationTest(unittest.TestCase):
    """Opt-in full A-GRA P2 shape probe and generation closure gate."""

    @classmethod
    def setUpClass(cls):
        if not P2_TREE.is_dir():
            raise unittest.SkipTest("P2 generated tree is absent")
        cls.files = parse_proto_tree(P2_TREE)

    def test_all_closure_messages_have_zero_cpp_shape_errors(self):
        messages, attempts, inventory = _probe_cpp_shapes(self.files)
        expected_messages = _p2_closure_counts()["messages"]
        printable = {
            category: {
                "messages": sorted(entry["messages"]),
                "details": sorted(entry["details"]),
            }
            for category, entry in inventory.items()
        }
        self.assertEqual(messages, expected_messages)
        self.assertEqual(attempts, expected_messages * 2)
        self.assertEqual(inventory, {}, json.dumps(printable, indent=2))

    def test_full_p2_cpp_and_ada_generation_completes(self):
        index = ProtoTypeIndex(self.files)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cpp = CppOmsJsonCodecGenerator(index).generate(root / "cpp")
            ada = AdaOmsJsonCodecGenerator(index).generate(root / "ada")
        self.assertEqual(
            [p.name for p in cpp],
            ["pyramid_data_model_agra_oms_json_codec_plugin.cpp"])
        self.assertEqual(
            sorted(p.name for p in ada),
            ["pyramid-data_model-agra-oms_json_codec.adb",
             "pyramid-data_model-agra-oms_json_codec.ads"])


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


class UnspecifiedXsdLiteralFixtureTest(unittest.TestCase):
    """A real XSD UNSPECIFIED literal remains distinct from presence state."""

    @classmethod
    def setUpClass(cls):
        profile = {
            "profile": "p_fixture", "drop": "test_fixture",
            "schema_version": "FIXTURE-1",
            "proto_package": "pyramid.data_model.uci_fixture",
            "roots": ["TestCommandStatus"],
        }
        schema = PIM_DIR / "test_xsd2proto_fixtures" / "uci_fixture.xsd"
        conv = xsd2proto.Converter(
            xsd2proto.SchemaIndex([schema], lax=False), profile)
        conv.convert()
        cls.tmp = tempfile.TemporaryDirectory()
        tree = Path(cls.tmp.name) / "tree"
        for rel, content in xsd2proto.outputs(conv).items():
            path = tree / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
        files = parse_proto_tree(tree)
        index = ProtoTypeIndex(files)
        cpp_paths = CppOmsJsonCodecGenerator(index).generate(
            Path(cls.tmp.name) / "cpp")
        ada_paths = AdaOmsJsonCodecGenerator(index).generate(
            Path(cls.tmp.name) / "ada")
        cls.cpp = cpp_paths[0].read_text(encoding="utf-8")
        cls.ada = next(path for path in ada_paths if path.suffix == ".adb") \
            .read_text(encoding="utf-8")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_cpp_encodes_and_decodes_the_real_xsd_literal(self):
        self.assertIn(
            'case uci::ModeEnum::UnspecifiedXsdLiteral: '
            'return "UNSPECIFIED";', self.cpp)
        self.assertIn(
            'if (s=="UNSPECIFIED") return '
            'uci::ModeEnum::UnspecifiedXsdLiteral;', self.cpp)

    def test_cpp_rejects_the_zero_sentinel(self):
        self.assertNotIn(
            'case uci::ModeEnum::Unspecified: return', self.cpp)
        self.assertIn("enum value not on the UCI wire", self.cpp)

    def test_ada_encodes_literal_and_rejects_zero_sentinel(self):
        self.assertIn(
            'when Enum_UnspecifiedXsdLiteral => return """UNSPECIFIED""";',
            self.ada)
        self.assertIn('when Enum_Unspecified =>', self.ada)
        self.assertIn('"ModeEnum value not on the UCI wire"', self.ada)


class AdaGeneralizedEmitterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        files = parse_proto_tree(P1_TREE)
        cls.paths = AdaOmsJsonCodecGenerator(
            ProtoTypeIndex(files)).generate(Path(cls.tmp.name))
        cls.body = next(p for p in cls.paths if p.suffix == ".adb") \
            .read_text(encoding="utf-8")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_sidecar_package_gets_the_generalized_encoder(self):
        self.assertEqual(
            sorted(p.name for p in self.paths),
            ["pyramid-data_model-uci-oms_json_codec.adb",
             "pyramid-data_model-uci-oms_json_codec.ads"])

    def test_roots_and_wire_rules_present(self):
        # One To_Oms_Json per profile root, keys from the sidecar, enums as
        # XSD literals with Unspecified raising, retained-base flattening
        # re-rooted through the Base component.
        for root in ("Action_Command_Status_Mt", "Position_Report_Mt",
                     "Signal_Report_Mt"):
            self.assertIn(f"function To_Oms_Json (Msg : "
                          f"Pyramid.Data_Model.Uci.Types.{root})", self.body)
        self.assertIn('"CUI_Basic"', self.body)
        self.assertIn('when Enum_Ts => return """TS""";', self.body)
        self.assertIn("value not on the UCI wire", self.body)
        self.assertIn("M.Base.Command_Id", self.body)

    def test_optional_presence_uses_the_documented_envelope(self):
        # Optional enum: zero-sentinel; optional message: Is_Default probe;
        # optional string: emptiness -- the Ada layer's presence envelope.
        self.assertIn("/= Enum_Unspecified then", self.body)
        self.assertIn("if not Is_Default_", self.body)
        self.assertIn("if Length (M.", self.body)

    def test_required_repeated_and_choice_arms_are_enforced(self):
        # Mirrors of the C++ emitter's validity rules (Codex review of
        # PR #120): empty required lists and multiply-populated choice
        # arms raise before producing schema-invalid wire.
        self.assertIn('"required repeated element Command is empty"',
                      self.body)
        self.assertIn('"multiple active choice arms in', self.body)
        self.assertIn("Boolean'Pos (M.Has_Government_Identifier) + "
                      "Boolean'Pos (M.Has_Nato_Special_Word)", self.body)


def _find_nlohmann():
    env = os.environ.get("NLOHMANN_JSON_INCLUDE")
    if env and (Path(env) / "nlohmann" / "json.hpp").is_file():
        return Path(env)
    bundled = PIM_DIR.parent / "core" / "external"
    if (bundled / "nlohmann" / "json.hpp").is_file():
        return bundled
    return None


_DRIVER_DIR: dict = {}


def _require_toolchain():
    if os.environ.get("OMS_JSON_COMPILE_SMOKE") != "1":
        raise unittest.SkipTest("set OMS_JSON_COMPILE_SMOKE=1 to run")
    if shutil.which("g++") is None:
        raise unittest.SkipTest("g++ not available")
    nlohmann = _find_nlohmann()
    if nlohmann is None:
        raise unittest.SkipTest(
            "set NLOHMANN_JSON_INCLUDE to a dir containing nlohmann/json.hpp")
    return nlohmann


def _build_driver() -> Path:
    """Generate P1 bindings and compile the smoke driver, once per test run
    (~1 min of template instantiation for the 515-message tree)."""
    if "exe" in _DRIVER_DIR:
        return _DRIVER_DIR["exe"]
    nlohmann = _require_toolchain()
    import atexit
    tmp = Path(tempfile.mkdtemp(prefix="oms_json_smoke_"))
    atexit.register(shutil.rmtree, tmp, ignore_errors=True)
    repo = PIM_DIR.parents[2]
    driver = PIM_DIR / "test_oms_json_gen_fixtures" / "p1_smoke_driver.cpp"
    gen = tmp / "gen"
    subprocess.run(
        [sys.executable, str(PIM_DIR / "generate_bindings.py"),
         str(P1_TREE), str(gen), "--languages", "cpp",
         "--backends", "oms_json"],
        check=True, capture_output=True)
    exe = tmp / "driver"
    compile_cmd = [
        "g++", "-std=c++17", "-o", str(exe), str(driver),
        str(gen / "oms_json" / "cpp" /
            "pyramid_data_model_uci_oms_json_codec_plugin.cpp"),
        str(gen / "pyramid_data_model_uci_cabi_marshal.cpp"),
        f"-I{gen}", f"-I{nlohmann}",
        f"-I{repo / 'subprojects' / 'PCL' / 'include'}",
        f"-I{repo / 'subprojects' / 'PYRAMID' / 'core' / 'external'}",
    ]
    build = subprocess.run(compile_cmd, capture_output=True, text=True)
    if build.returncode != 0:
        raise AssertionError("driver build failed:\n" + build.stderr[-4000:])
    _DRIVER_DIR["exe"] = exe
    return exe


class OmsJsonCompileSmokeTest(unittest.TestCase):
    """Opt-in (slow: ~1 min of template instantiation): compile the
    generated P1 plugin and run an encode->decode->re-encode round trip.
    Demonstrated green 2026-07-11 (g++ 13, nlohmann 3.11.3):
    dual-id dispatch, sidecar names, enum literals, nested-base flattening,
    oneof choice, omit-empty arrays, byte-stable round trip."""

    @classmethod
    def setUpClass(cls):
        cls.exe = _build_driver()

    def test_p1_codec_compiles_and_round_trips(self):
        run = subprocess.run([str(self.exe)], capture_output=True, text=True)
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


class P2OmsJsonCompileSmokeTest(unittest.TestCase):
    """Opt-in object compile of the generated full-closure A-GRA codec TU."""

    @classmethod
    def setUpClass(cls):
        cls.nlohmann = _require_toolchain()
        if not P2_TREE.is_dir():
            raise unittest.SkipTest("P2 generated tree is absent")

    def test_p2_cpp_codec_object_compiles(self):
        repo = PIM_DIR.parents[2]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            gen = root / "gen"
            subprocess.run(
                [sys.executable, str(PIM_DIR / "generate_bindings.py"),
                 str(P2_TREE), str(gen), "--languages", "cpp",
                 "--backends", "oms_json"],
                check=True, capture_output=True)
            source = (gen / "oms_json" / "cpp" /
                      "pyramid_data_model_agra_oms_json_codec_plugin.cpp")
            output = root / "p2_oms_json_codec.o"
            command = [
                "g++", "-std=c++17", "-O1", "-c", str(source),
                "-o", str(output),
                f"-I{gen}", f"-I{self.nlohmann}",
                f"-I{repo / 'subprojects' / 'PCL' / 'include'}",
                f"-I{repo / 'subprojects' / 'PYRAMID' / 'core' / 'external'}",
            ]
            if os.name == "nt":
                # The full closure instantiates more COFF sections than the
                # default MinGW object format permits in one translation unit.
                command.append("-Wa,-mbig-obj")
            started = time.perf_counter()
            build = subprocess.run(command, capture_output=True, text=True)
            elapsed = time.perf_counter() - started
            self.assertEqual(
                build.returncode, 0,
                f"P2 object compile failed after {elapsed:.1f}s:\n"
                + build.stderr[-8000:])
            self.assertTrue(output.is_file())
            print(f"P2 OMS-JSON codec object compile: {elapsed:.1f}s")


def _strip_type_annotations(node):
    """Remove the la-cal-harness generator's non-UCI ``$type`` annotations
    (the Phase-4 interop convention: stripped before comparison)."""
    if isinstance(node, dict):
        return {k: _strip_type_annotations(v) for k, v in node.items()
                if k != "$type"}
    if isinstance(node, list):
        return [_strip_type_annotations(v) for v in node]
    return node


def _harness_src():
    """The la-cal-harness checkout, or None.

    LACAL_HARNESS_SRC wins.  Otherwise try the known clone directory names
    under external/ams-gra/: the repository is cloned under its short name
    here, but its full upstream name is also accepted."""
    env = os.environ.get("LACAL_HARNESS_SRC")
    if env:
        return Path(env) if (Path(env) / "la_cal_harness").is_dir() else None
    external = PIM_DIR.parents[2] / "external" / "ams-gra"
    for name in ("la-cal-harness",
                 "ams-gra-hello-world-sk-test-la-cal-harness"):
        candidate = external / name / "src"
        if (candidate / "la_cal_harness").is_dir():
            return candidate
    return None


def _load_harness_generator(xsd: Path):
    """The harness's XSD-derived OMS JSON generator, or SKIP with a reason."""
    src = _harness_src()
    if src is None:
        raise unittest.SkipTest(
            "la-cal-harness not found under external/ams-gra -- clone it or "
            "set LACAL_HARNESS_SRC")
    if not xsd.is_file():
        raise unittest.SkipTest(
            f"{xsd.name} absent -- run pim/schemas/fetch_schemas.py")
    sys.path.insert(0, str(src))
    try:
        from la_cal_harness.oms_json.generator import OMSJsonGenerator
    except ImportError as exc:
        raise unittest.SkipTest(f"harness deps missing: {exc}")
    return OMSJsonGenerator(str(xsd))


_HYPHENATED_UUID = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$")


def _normalize_agra_uuids(node, seen=None):
    """Rewrite the harness's hyphenated UUIDs into the A-GRA wire form.

    Known harness gap (recorded in the WS-G/G1 checkpoint log): the
    la-cal-harness generator emits RFC-4122 hyphenated UUID text whatever
    the schema says, but A-GRA 5.0a types UUID as
    ``UniversallyUniqueIdentifierType`` = ``xs:hexBinary`` length 16, whose
    lexical form is 32 hex digits with no hyphens.  Left alone, every
    harness instance would carry a schema-invalid UUID and the fidelity
    claim below would be vacuous.

    Patching this here rather than in the harness keeps the harness an
    independent, unmodified implementation of the same XSD, which is what
    makes the round-trip comparison evidence rather than a self-test.
    ``pim/test_agra_p2_seam.py`` and this module's
    ``AgraUuidNormalizationTest`` pin both halves: the raw value is invalid
    against the schema type and the rewritten one is valid."""
    if isinstance(node, dict):
        out = {}
        for key, value in node.items():
            if (key == "UUID" and isinstance(value, str)
                    and _HYPHENATED_UUID.match(value)):
                if seen is not None:
                    seen.append(value)
                out[key] = value.replace("-", "").upper()
            else:
                out[key] = _normalize_agra_uuids(value, seen)
        return out
    if isinstance(node, list):
        return [_normalize_agra_uuids(value, seen) for value in node]
    return node


class HarnessRoundTripTest(unittest.TestCase):
    """Plan D4(a): schema-derived instances through the generated codec.

    For every P1 root, the independently-authored la-cal-harness
    XSD-derived generator produces a minimal schema-valid instance; the
    generated codec must decode and re-encode it to a semantically
    identical document.  Since the input is derived from the pinned UCI
    2.5 XSD by a foreign implementation, equality here is simultaneously a
    decode-fidelity, encode-fidelity, and schema-shape check.

    Opt-in alongside the compile smoke (same toolchain gates), plus the
    harness checkout (LACAL_HARNESS_SRC or the external/ default) with
    lxml + xmlschema importable.  Demonstrated green 2026-07-11 for all
    six P1 roots."""

    @classmethod
    def setUpClass(cls):
        cls.exe = _build_driver()
        cls.generator = _load_harness_generator(_UCI_XSD)

    def _roundtrip(self, root: str):
        doc = _strip_type_annotations(self.generator.generate_minimal(root))
        run = subprocess.run(
            [str(self.exe), "--roundtrip", root],
            input=json.dumps(doc), capture_output=True, text=True)
        self.assertEqual(run.returncode, 0,
                         f"{root}: {run.stderr}\ninput: {json.dumps(doc)[:2000]}")
        out = json.loads(run.stdout)
        self.assertEqual(out, doc, root)

    def test_action_command(self):
        self._roundtrip("ActionCommand")

    def test_action_command_status(self):
        self._roundtrip("ActionCommandStatus")

    def test_observation_measurement_report(self):
        self._roundtrip("ObservationMeasurementReport")

    def test_position_report(self):
        self._roundtrip("PositionReport")

    def test_service_status(self):
        self._roundtrip("ServiceStatus")

    def test_signal_report(self):
        self._roundtrip("SignalReport")


_P2_ROOTS = json.loads(
    (PIM_DIR / "uci_profiles" / "p2_agra_planning_core.json").read_text(
        encoding="utf-8"))["roots"]

_AGRA_XSD = (PIM_DIR / "schemas" / "dl" / "agra_5_0a" /
             "A-GRA_MessageDefinitions_v5_0_a.xsd")

_UCI_XSD = (PIM_DIR / "schemas" / "dl" / "uci_2_5_0" /
            "UCI_MessageDefinitions_v2_5_0.xsd")


class AgraUuidNormalizationTest(unittest.TestCase):
    """Pin the harness UUID gap and the rewrite that closes it.

    This is the load-bearing assumption behind AgraHarnessRoundTripTest, so
    it is checked against the schema itself rather than assumed.  It needs
    only the harness and xmlschema -- no C++ toolchain -- so it runs
    wherever those import."""

    @classmethod
    def setUpClass(cls):
        cls.generator = _load_harness_generator(_AGRA_XSD)
        try:
            import xmlschema
        except ImportError as exc:
            raise unittest.SkipTest(f"xmlschema missing: {exc}")
        cls.uuid_type = xmlschema.XMLSchema(str(_AGRA_XSD)).types[
            "UniversallyUniqueIdentifierType"]

    def test_raw_harness_uuids_are_schema_invalid_and_rewrites_are_valid(self):
        raw_total = 0
        for root in _P2_ROOTS:
            seen = []
            _normalize_agra_uuids(self.generator.generate_minimal(root), seen)
            self.assertTrue(
                seen, f"{root}: expected at least one harness UUID field")
            raw_total += len(seen)
            for raw in seen:
                self.assertFalse(
                    self.uuid_type.is_valid(raw),
                    f"{root}: harness UUID {raw!r} is unexpectedly valid -- "
                    "if the harness gained facet awareness, drop the rewrite")
                self.assertTrue(
                    self.uuid_type.is_valid(raw.replace("-", "").upper()),
                    f"{root}: rewritten UUID from {raw!r} is not valid "
                    "against UniversallyUniqueIdentifierType")
        self.assertEqual(raw_total, 53)

    def test_rewrite_only_touches_hyphenated_uuid_fields(self):
        already_wire_form = "0123456789ABCDEF0123456789ABCDEF"
        doc = {
            "UUID": already_wire_form,
            "DescriptiveLabel": "1c1a4f2e-0000-4000-8000-000000000000",
            "Nested": [{"UUID": "1c1a4f2e-0000-4000-8000-000000000000"}],
        }
        out = _normalize_agra_uuids(doc)
        self.assertEqual(out["UUID"], already_wire_form)
        self.assertEqual(out["DescriptiveLabel"],
                         "1c1a4f2e-0000-4000-8000-000000000000")
        self.assertEqual(out["Nested"][0]["UUID"],
                         "1C1A4F2E000040008000000000000000")


def _build_p2_driver() -> Path:
    """Generate P2 bindings and compile the round-trip driver, once per run.

    Built from the interaction overlay rather than the bare data-model tree,
    because the overlay is the contract the SDK actually packages: it is the
    only input that carries binding_metadata.json, so it is the only one
    whose codec has a schema identity to check.  The data model underneath is
    a byte-identical copy of P2_TREE's (pim/test_agra_p2_seam.py guards that),
    so the round trips below test the same codec either way -- this way they
    test the artifact that ships.

    Slower than the P1 driver: the A-GRA closure is 1192 messages."""
    if "p2_exe" in _DRIVER_DIR:
        return _DRIVER_DIR["p2_exe"]
    nlohmann = _require_toolchain()
    if not P2_CONTRACT.is_dir():
        raise unittest.SkipTest("P2 contract overlay is absent")
    import atexit
    tmp = Path(tempfile.mkdtemp(prefix="oms_json_p2_smoke_"))
    atexit.register(shutil.rmtree, tmp, ignore_errors=True)
    repo = PIM_DIR.parents[2]
    driver = PIM_DIR / "test_oms_json_gen_fixtures" / "p2_smoke_driver.cpp"
    gen = tmp / "gen"
    subprocess.run(
        [sys.executable, str(PIM_DIR / "generate_bindings.py"),
         str(P2_CONTRACT), str(gen), "--languages", "cpp",
         "--backends", "oms_json"],
        check=True, capture_output=True)
    exe = tmp / "p2_driver"
    compile_cmd = [
        "g++", "-std=c++17", "-O1", "-o", str(exe), str(driver),
        str(gen / "oms_json" / "cpp" /
            "pyramid_data_model_agra_oms_json_codec_plugin.cpp"),
        str(gen / "pyramid_data_model_agra_cabi_marshal.cpp"),
        f"-I{gen}", f"-I{nlohmann}",
        f"-I{repo / 'subprojects' / 'PCL' / 'include'}",
        f"-I{repo / 'subprojects' / 'PYRAMID' / 'core' / 'external'}",
    ]
    if os.name == "nt":
        # Same COFF section limit the P2 object compile hits: the full
        # closure exceeds what the default MinGW object format permits.
        compile_cmd.append("-Wa,-mbig-obj")
    started = time.perf_counter()
    build = subprocess.run(compile_cmd, capture_output=True, text=True)
    if build.returncode != 0:
        raise AssertionError(
            f"P2 driver build failed after {time.perf_counter() - started:.1f}s"
            ":\n" + build.stderr[-8000:])
    print(f"P2 round-trip driver build: {time.perf_counter() - started:.1f}s")
    _DRIVER_DIR["p2_exe"] = exe
    return exe


class AgraHarnessRoundTripTest(unittest.TestCase):
    """Plan step 4, the P2 half of D4(a): schema-derived A-GRA instances
    through the generated P2 codec.

    For every root in the p2_agra_planning_core profile, the
    independently-authored la-cal-harness generator produces a minimal
    instance from the pinned A-GRA 5.0a XSD; UUIDs are rewritten to the
    drop's hexBinary wire form (see ``_normalize_agra_uuids``); the
    generated codec must decode and re-encode it to a semantically
    identical document.  As with P1, equality is simultaneously a
    decode-fidelity, encode-fidelity, and schema-shape check, because the
    input comes from a foreign implementation of the same schema.

    Opt-in: OMS_JSON_COMPILE_SMOKE=1 plus a C++17 toolchain, nlohmann, and
    an importable harness."""

    @classmethod
    def setUpClass(cls):
        cls.generator = _load_harness_generator(_AGRA_XSD)
        cls.exe = _build_p2_driver()

    def _roundtrip(self, root: str):
        doc = _normalize_agra_uuids(
            _strip_type_annotations(self.generator.generate_minimal(root)))
        run = subprocess.run(
            [str(self.exe), "--roundtrip", root],
            input=json.dumps(doc), capture_output=True, text=True)
        self.assertEqual(
            run.returncode, 0,
            f"{root}: {run.stderr}\ninput: {json.dumps(doc)[:2000]}")
        self.assertEqual(json.loads(run.stdout), doc, root)

    def test_unknown_root_fails_closed(self):
        run = subprocess.run(
            [str(self.exe), "--roundtrip", "NotAnAgraRoot"],
            input="{}", capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)

    def test_malformed_input_fails_closed(self):
        run = subprocess.run(
            [str(self.exe), "--roundtrip", "MA_Task"],
            input="{not json", capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)


def _add_p2_roundtrip_cases():
    """One test method per profile root, so a failure names the root."""
    for root in _P2_ROOTS:
        def case(self, root=root):
            self._roundtrip(root)
        case.__name__ = f"test_{root.lower()}"
        case.__doc__ = f"Harness round trip for the {root} root."
        setattr(AgraHarnessRoundTripTest, case.__name__, case)


_add_p2_roundtrip_cases()


class SchemaIdentitySourceTest(unittest.TestCase):
    """Plan step 5: a profile-specific codec knows which drop it was built
    for.  These checks need no toolchain, so they run everywhere.

    The identity is emitted only for a contract that carries
    binding_metadata.json.  That is not an optimization -- it is what keeps
    the frozen seam golden byte-identical (plan D4(b)), the same discipline
    the UUID validators follow: emit a rule only where the tree uses it."""

    def _plugin_source(self, tree: Path) -> str:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            subprocess.run(
                [sys.executable, str(PIM_DIR / "generate_bindings.py"),
                 str(tree), str(out), "--languages", "cpp",
                 "--backends", "oms_json"],
                check=True, capture_output=True)
            sources = list((out / "oms_json" / "cpp").glob(
                "*_oms_json_codec_plugin.cpp"))
            self.assertEqual(len(sources), 1, f"{tree.name}: {sources}")
            return sources[0].read_text(encoding="utf-8")

    def test_the_agra_contract_emits_its_identity_and_the_check(self):
        source = self._plugin_source(P2_CONTRACT)
        identity = json.loads(
            (P2_CONTRACT / "binding_metadata.json").read_text(
                encoding="utf-8"))
        for key, value in identity.items():
            self.assertIn(f'{{"{key}","{value}"}}', source)
        self.assertIn("identity_admits(config_json) ? &codec : nullptr",
                      source)

    def test_a_contract_without_metadata_keeps_the_unconditional_entry(self):
        source = self._plugin_source(P1_TREE)
        self.assertNotIn("kIdentity", source)
        self.assertNotIn("identity_admits", source)
        self.assertIn(
            "const pcl_codec_t* pcl_codec_plugin_entry(const char*) "
            "{ return &codec; }", source)


class SchemaIdentityRuntimeTest(unittest.TestCase):
    """Plan step 5, the runtime half: the A-GRA codec's PCL entry point
    admits or refuses a loader configuration by its schema identity.

    The entry contract makes config_json opaque and plugin-specific, so this
    is the seam where a codec can decline to serve the wrong drop before any
    traffic reaches it -- SchemaDropMismatchTest covers the message-level
    refusals that catch what gets past the loader.

    Opt-in: same gates as the round-trip tests."""

    @classmethod
    def setUpClass(cls):
        cls.exe = _build_p2_driver()
        cls.identity = json.loads(
            (P2_CONTRACT / "binding_metadata.json").read_text(
                encoding="utf-8"))

    def _entry(self, config: str) -> str:
        run = subprocess.run([str(self.exe), "--identity", config],
                             capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        return run.stdout.strip()

    def test_no_configuration_is_admitted(self):
        # The PCL entry contract documents NULL as "no configuration": the
        # loader asserts nothing, so there is nothing to contradict.
        self.assertEqual(self._entry("null"), "ADMITTED")
        self.assertEqual(self._entry("{}"), "ADMITTED")

    def test_the_matching_drop_is_admitted(self):
        self.assertEqual(self._entry(json.dumps(self.identity)), "ADMITTED")

    def test_each_identity_key_is_checked_on_its_own(self):
        for key, value in self.identity.items():
            with self.subTest(key=key):
                self.assertEqual(self._entry(json.dumps({key: value})),
                                 "ADMITTED")
                self.assertEqual(
                    self._entry(json.dumps({key: value + "-wrong"})),
                    "REFUSED", f"{key} is not actually checked")

    def test_the_uci_drop_is_refused(self):
        # The concrete confusion this exists to prevent: an SDK consumer
        # pointing a UCI 2.5 deployment at the A-GRA codec.
        self.assertEqual(
            self._entry(json.dumps({"drop": "uci_2_5_0",
                                    "schema_version": "002.5.0"})),
            "REFUSED")

    def test_one_wrong_key_refuses_even_when_the_others_match(self):
        config = dict(self.identity)
        config["schema_version"] = "005.0a.ASK-99999999-deadbee"
        self.assertEqual(self._entry(json.dumps(config)), "REFUSED")

    def test_unparseable_configuration_is_refused(self):
        # A loader that meant to pin the drop but typed the config wrong must
        # not silently get an unchecked codec.
        self.assertEqual(self._entry("{not json"), "REFUSED")
        self.assertEqual(self._entry('"a string, not an object"'), "REFUSED")

    def test_keys_the_codec_has_no_identity_for_are_left_alone(self):
        self.assertEqual(self._entry(json.dumps({"transport": "lacal"})),
                         "ADMITTED")


class SchemaDropMismatchTest(unittest.TestCase):
    """Plan step 4's schema-drop mismatch negatives: each generated codec
    must fail closed when handed the other drop's traffic.

    The two drops overlap enough for this to be a real deployment risk
    rather than a theoretical one.  A-GRA 5.0a and UCI 2.5 share element
    names down to the message envelope (``MessageData``, ``MessageHeader``,
    ``SecurityInformation``), so a peer that publishes the wrong drop's
    content on the right topic produces a document that looks plausible
    until something checks it.  Two independent things must catch it:

    1. the UUID lexical form, which differs between the drops (hexBinary in
       A-GRA, RFC-4122 hyphenated in UCI).  These two cases are the runtime
       inverse of the defect the P2 round trip found -- before the fix the
       A-GRA codec enforced the UCI form, so the wrong-drop document was
       the one it would have accepted;
    2. the message body, whose required elements differ, so ``j.at`` fails
       on the first element the other drop does not carry.

    Both directions are covered, because a codec that refuses foreign
    traffic but emits it is just as broken.

    Opt-in: same gates as the round-trip tests (OMS_JSON_COMPILE_SMOKE=1,
    a C++17 toolchain, nlohmann, and an importable harness)."""

    @classmethod
    def setUpClass(cls):
        cls.uci_generator = _load_harness_generator(_UCI_XSD)
        cls.agra_generator = _load_harness_generator(_AGRA_XSD)
        cls.p1_exe = _build_driver()
        cls.p2_exe = _build_p2_driver()

    def _uci_doc(self):
        return _strip_type_annotations(
            self.uci_generator.generate_minimal("ActionCommand"))

    def _agra_doc(self):
        return _strip_type_annotations(
            self.agra_generator.generate_minimal("MA_Task"))

    def _assert_carries_uuids(self, doc, label):
        """Guard against a vacuous pass: a document with no UUID in it
        cannot demonstrate anything about UUID forms."""
        seen = []
        _normalize_agra_uuids(doc, seen)
        self.assertTrue(seen, f"{label} carries no UUID -- test is vacuous")

    def _assert_refused(self, exe, root, doc, why):
        run = subprocess.run(
            [str(exe), "--roundtrip", root],
            input=json.dumps(doc), capture_output=True, text=True)
        self.assertNotEqual(
            run.returncode, 0,
            f"{why}: the codec accepted it\ninput: {json.dumps(doc)[:2000]}")
        # The refusal must come from the codec's decode of a root it knows,
        # not from the driver failing to find the root or from a crash --
        # either of those would pass the returncode check while proving
        # nothing about drop mismatch.
        self.assertIn(
            f"FAIL: decode {root}", run.stderr,
            f"{why}: expected the codec to refuse it on decode, but the "
            f"driver exited {run.returncode} saying: {run.stderr!r}")

    def test_a_uci_form_uuid_is_refused_by_the_agra_codec(self):
        # The harness emits hyphenated UUIDs whatever the schema says, so
        # its raw A-GRA output *is* an otherwise well-shaped A-GRA document
        # carrying UCI-form UUIDs.  Skipping _normalize_agra_uuids is the
        # whole fixture.
        doc = self._agra_doc()
        self._assert_carries_uuids(doc, "the MA_Task instance")
        self._assert_refused(self.p2_exe, "MA_Task", doc,
                             "A-GRA document with UCI-form UUIDs")

    def test_an_agra_form_uuid_is_refused_by_the_uci_codec(self):
        raw = self._uci_doc()
        self._assert_carries_uuids(raw, "the ActionCommand instance")
        doc = _normalize_agra_uuids(raw)
        self._assert_refused(self.p1_exe, "ActionCommand", doc,
                             "UCI document with A-GRA-form UUIDs")

    def test_uci_content_is_refused_on_an_agra_root(self):
        # UUIDs normalized to the A-GRA form on purpose, so the refusal
        # comes from the message body rather than the UUID check.
        doc = _normalize_agra_uuids(self._uci_doc())
        self._assert_refused(self.p2_exe, "MA_Task", doc,
                             "UCI ActionCommand body on the MA_Task root")

    def test_agra_content_is_refused_on_a_uci_root(self):
        doc = self._agra_doc()  # left in the harness's UCI-form UUIDs
        self._assert_refused(self.p1_exe, "ActionCommand", doc,
                             "A-GRA MA_Task body on the ActionCommand root")


class AdaCompileParityTest(unittest.TestCase):
    """Opt-in Ada parity gates (same OMS_JSON_COMPILE_SMOKE=1 switch):

    1. the generated P1 Ada OMS-JSON encoder object-compiles (gnatmake,
       -gnat2020) over the full 515-message tree;
    2. an Ada driver building the same ActionCommandStatusMT value as the
       C++ self-test prints wire JSON **byte-identical** to the checked-in
       parity golden (which the C++ driver also matches) -- the static
       key sorting reproduces nlohmann's object ordering exactly.

    Demonstrated green 2026-07-11 (GNAT 13.3)."""

    @classmethod
    def setUpClass(cls):
        if os.environ.get("OMS_JSON_COMPILE_SMOKE") != "1":
            raise unittest.SkipTest("set OMS_JSON_COMPILE_SMOKE=1 to run")
        if shutil.which("gnatmake") is None:
            raise unittest.SkipTest("gnatmake not available")

    def test_p1_ada_encoder_compiles_and_matches_the_parity_golden(self):
        golden = (PIM_DIR / "test_oms_json_gen_fixtures" /
                  "p1_parity_golden.json").read_text(encoding="utf-8").strip()
        driver = PIM_DIR / "test_oms_json_gen_fixtures" / \
            "p1_ada_parity_driver.adb"
        with tempfile.TemporaryDirectory() as tmp:
            gen = Path(tmp) / "gen"
            subprocess.run(
                [sys.executable, str(PIM_DIR / "generate_bindings.py"),
                 str(P1_TREE), str(gen), "--languages", "ada",
                 "--backends", "oms_json"],
                check=True, capture_output=True)
            shutil.copy(driver, gen / driver.name)
            build = subprocess.run(
                ["gnatmake", "-q", "-gnat2020", "-I.", "-Ioms_json/ada",
                 driver.name],
                cwd=gen, capture_output=True, text=True)
            self.assertEqual(build.returncode, 0,
                             build.stdout + build.stderr)
            run = subprocess.run([str(gen / "p1_ada_parity_driver")],
                                 capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertEqual(run.stdout.strip(), golden,
                             "Ada wire output drifted from the parity "
                             "golden (C++ and Ada must stay byte-identical)")

    def test_cpp_driver_matches_the_same_golden(self):
        golden = (PIM_DIR / "test_oms_json_gen_fixtures" /
                  "p1_parity_golden.json").read_text(encoding="utf-8").strip()
        try:
            exe = _build_driver()
        except unittest.SkipTest:
            self.skipTest("C++ toolchain gates not met")
        run = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertEqual(run.stdout.splitlines()[0], golden)


class P2AdaCompileParityTest(unittest.TestCase):
    """Plan step 4, the P2 twin of AdaCompileParityTest: the deterministic
    golden and the C++/Ada wire-parity gate for the A-GRA profile.

    Both drivers build the same MA_MissionPlanCommandStatusMT value and must
    print wire JSON byte-identical to ``p2_parity_golden.json``.  What this
    adds over the P1 golden is the drop difference: every UUID in the A-GRA
    value is hexBinary (32 hex digits, no hyphens), so the golden would move
    if either emitter ever reverted to the UCI 2.5 hyphenated form.

    What the golden does and does not prove: it pins the bytes and the
    agreement between the two emitters, not that the document is
    schema-valid.  Schema fidelity is AgraHarnessRoundTripTest's job, where
    the instances come from a foreign implementation of the same XSD.

    Opt-in: OMS_JSON_COMPILE_SMOKE=1, plus gnatmake for the Ada half and a
    C++17 toolchain with nlohmann for the C++ half."""

    GOLDEN = PIM_DIR / "test_oms_json_gen_fixtures" / "p2_parity_golden.json"

    @classmethod
    def setUpClass(cls):
        if os.environ.get("OMS_JSON_COMPILE_SMOKE") != "1":
            raise unittest.SkipTest("set OMS_JSON_COMPILE_SMOKE=1 to run")
        if not P2_TREE.is_dir():
            raise unittest.SkipTest("P2 generated tree is absent")

    def test_p2_ada_encoder_compiles_and_matches_the_parity_golden(self):
        if shutil.which("gnatmake") is None:
            self.skipTest("gnatmake not available")
        golden = self.GOLDEN.read_text(encoding="utf-8").strip()
        driver = (PIM_DIR / "test_oms_json_gen_fixtures" /
                  "p2_ada_parity_driver.adb")
        with tempfile.TemporaryDirectory() as tmp:
            gen = Path(tmp) / "gen"
            subprocess.run(
                [sys.executable, str(PIM_DIR / "generate_bindings.py"),
                 str(P2_TREE), str(gen), "--languages", "ada",
                 "--backends", "oms_json"],
                check=True, capture_output=True)
            shutil.copy(driver, gen / driver.name)
            started = time.perf_counter()
            build = subprocess.run(
                ["gnatmake", "-q", "-gnat2020", "-I.", "-Ioms_json/ada",
                 driver.name],
                cwd=gen, capture_output=True, text=True)
            self.assertEqual(build.returncode, 0,
                             f"P2 Ada build failed after "
                             f"{time.perf_counter() - started:.1f}s:\n"
                             + build.stdout + build.stderr)
            run = subprocess.run([str(gen / "p2_ada_parity_driver")],
                                 capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertEqual(run.stdout.strip(), golden,
                             "A-GRA Ada wire output drifted from the parity "
                             "golden (C++ and Ada must stay byte-identical)")

    def test_p2_cpp_driver_matches_the_same_golden(self):
        golden = self.GOLDEN.read_text(encoding="utf-8").strip()
        try:
            exe = _build_p2_driver()
        except unittest.SkipTest:
            self.skipTest("C++ toolchain gates not met")
        run = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertIn("RT_OK", run.stdout)
        self.assertEqual(run.stdout.splitlines()[0], golden)

    def test_the_golden_carries_the_agra_uuid_form(self):
        """A cheap, toolchain-free guard on the fixture itself, so a golden
        recaptured from a regressed emitter cannot pass unnoticed."""
        doc = json.loads(self.GOLDEN.read_text(encoding="utf-8"))
        uuids = []

        def collect(node):
            if isinstance(node, dict):
                for key, value in node.items():
                    if key == "UUID" and isinstance(value, str):
                        uuids.append(value)
                    else:
                        collect(value)
            elif isinstance(node, list):
                for value in node:
                    collect(value)

        collect(doc)
        self.assertTrue(uuids, "golden carries no UUID to check")
        for value in uuids:
            self.assertRegex(value, r"^[0-9A-Fa-f]{32}$",
                             "A-GRA UUIDs are hexBinary length 16")


if __name__ == "__main__":
    unittest.main()
