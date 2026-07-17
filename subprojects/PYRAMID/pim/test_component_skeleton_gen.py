"""Regression tests for generated component skeletons and scaffolds."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from binding_contract import build_contract  # noqa: E402
from proto_parser import ProtoTypeIndex, parse_proto_tree  # noqa: E402
from ada import component_skeleton_gen as ada_skeleton_gen  # noqa: E402
from cpp import component_skeleton_gen as cpp_skeleton_gen  # noqa: E402


PIM_DIR = Path(__file__).resolve().parent
GENERATOR = PIM_DIR / "generate_bindings.py"
PROTO_DIR = PIM_DIR / "test" / "pyramid"
BASELINE_DIR = PIM_DIR / "test_component_skeleton_baseline"
COMPONENTS = "pim_osprey.sensors,pim_osprey.sensor_products"


def _tree_bytes(root: Path) -> dict[str, bytes]:
    return {
        path.relative_to(root).as_posix(): path.read_bytes()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def _run_generator(output_dir: Path, scaffold_dir: Path | None = None,
                   component_skeletons: bool = True):
    command = [
        sys.executable,
        str(GENERATOR),
        str(PROTO_DIR),
        str(output_dir),
        "--languages",
        "cpp,ada",
        "--backends",
        "json",
    ]
    if component_skeletons:
        command.extend([
            "--component-skeletons",
            "--components",
            COMPONENTS,
        ])
    if scaffold_dir is not None:
        command.extend(["--scaffold-dir", str(scaffold_dir)])
    return subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
    )


class EmitterLockstepTest(unittest.TestCase):
    """The C++ and Ada emitters must agree on ports and handler slots."""

    def test_cpp_and_ada_emitters_agree_on_names(self):
        files = parse_proto_tree(PROTO_DIR)
        contract = build_contract(files, "pyramid")
        index = ProtoTypeIndex(files)
        for group in contract.components:
            cpp = cpp_skeleton_gen.CppComponentSkeletonGenerator(
                group, files, index, contract.naming_policy)
            ada = ada_skeleton_gen.AdaComponentSkeletonGenerator(
                group, files, index, contract.naming_policy)
            self.assertEqual(
                cpp.port_names(), ada.port_names(), group.key)
            self.assertEqual(
                cpp.slot_names(), ada.slot_names(), group.key)


class ComponentSkeletonGenerationTest(unittest.TestCase):
    def test_golden_manifest_and_regeneration_idempotence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output_dir = root / "generated"
            scaffold_dir = root / "scaffold"
            _run_generator(output_dir, scaffold_dir)

            golden = _tree_bytes(BASELINE_DIR)
            generated = {
                name: (output_dir / name).read_bytes()
                for name in golden
            }
            self.assertEqual(generated, golden)

            manifest_path = output_dir / "binding_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(
                sorted(manifest["component_skeletons"]),
                sorted(golden),
            )
            self.assertEqual(
                manifest["component_skeleton_sources"],
                [
                    "pyramid_component_pim_osprey_sensor_products_skeleton.cpp",
                    "pyramid_component_pim_osprey_sensors_skeleton.cpp",
                ],
            )
            manifest_text = manifest_path.read_text(encoding="utf-8")
            self.assertNotIn(str(scaffold_dir), manifest_text)
            self.assertNotIn("scaffold/", manifest_text)

            before = {
                name: (output_dir / name).read_bytes()
                for name in golden
            }
            sentinel_path = (
                scaffold_dir
                / "pim_osprey_sensors"
                / "src"
                / "sensors_component.cpp"
            )
            sentinel = b"// user-owned sentinel\n"
            sentinel_path.write_bytes(sentinel)
            deleted_path = (
                scaffold_dir
                / "pim_osprey_sensors"
                / "configs"
                / "linux"
                / "tcp"
                / "sensors.ports"
            )
            deleted_path.unlink()

            _run_generator(output_dir, scaffold_dir)

            after = {
                name: (output_dir / name).read_bytes()
                for name in golden
            }
            self.assertEqual(after, before)
            self.assertEqual(sentinel_path.read_bytes(), sentinel)
            self.assertTrue(deleted_path.is_file())

    def test_default_generation_emits_no_skeleton_artifacts_or_roles(self):
        with tempfile.TemporaryDirectory() as temporary:
            output_dir = Path(temporary) / "generated"
            _run_generator(output_dir, component_skeletons=False)

            self.assertFalse(list(output_dir.rglob("pyramid_component_*")))
            manifest = json.loads(
                (output_dir / "binding_manifest.json").read_text(
                    encoding="utf-8"))
            self.assertNotIn("component_skeletons", manifest)
            self.assertNotIn("component_skeleton_sources", manifest)

    def test_scaffold_directory_must_be_outside_output_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output_dir = root / "generated"
            result = subprocess.run(
                [
                    sys.executable,
                    str(GENERATOR),
                    str(PROTO_DIR),
                    str(output_dir),
                    "--languages",
                    "cpp",
                    "--backends",
                    "json",
                    "--component-skeletons",
                    "--scaffold-dir",
                    str(output_dir / "scaffold"),
                ],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(
                "--scaffold-dir must be outside output_dir",
                result.stderr,
            )


if __name__ == "__main__":
    unittest.main()
