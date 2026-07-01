#!/usr/bin/env python3
"""Generated binding manifest tests."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from test_generic_binding_contract import PYRAMID_CPP_JSON_FILENAMES


REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
GENERATOR = PYRAMID_ROOT / "pim" / "generate_bindings.py"


TELEMETRY_PROTO = """syntax = "proto3";
package example.telemetry;
message Pose { double x = 1; double y = 2; }
service Telemetry { rpc GetPose(Pose) returns (Pose); }
"""


ARTIFACT_ROLES = (
    "types",
    "json_codecs",
    "cabi",
    "services",
    "flatbuffers_schemas",
)


def _run_generator(proto_dir: Path, out_dir: Path, *extra: str) -> None:
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            str(proto_dir),
            str(out_dir),
            "--languages",
            "cpp",
            "--backends",
            "json",
            *extra,
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _load_manifest(out_dir: Path) -> dict:
    manifest_path = out_dir / "binding_manifest.json"
    assert manifest_path.exists()
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def _assert_manifest_artifacts_exist(out_dir: Path, manifest: dict) -> None:
    for role in ARTIFACT_ROLES:
        for rel_path in manifest[role]:
            assert (out_dir / rel_path).is_file(), rel_path
    for plugin in manifest["codec_plugins"]:
        assert (out_dir / plugin["source"]).is_file(), plugin["source"]


@pytest.fixture()
def telemetry_proto_dir(tmp_path: Path) -> Path:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "telemetry.proto").write_text(
        TELEMETRY_PROTO, encoding="utf-8")
    return proto_dir


def test_generic_manifest_lists_generated_cpp_json_artifacts(
    telemetry_proto_dir: Path,
    tmp_path: Path,
) -> None:
    out_dir = tmp_path / "out"

    _run_generator(
        telemetry_proto_dir,
        out_dir,
        "--contract-layout",
        "generic",
    )

    manifest = _load_manifest(out_dir)

    assert manifest["layout"] == "generic"
    assert "example_telemetry_types.hpp" in manifest["types"]
    assert "example_telemetry_codec.cpp" in manifest["json_codecs"]
    assert "example_telemetry_services.cpp" in manifest["services"]
    _assert_manifest_artifacts_exist(out_dir, manifest)


def test_pyramid_manifest_lists_existing_cpp_json_roles(tmp_path: Path) -> None:
    out_dir = tmp_path / "pyramid_out"
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"

    _run_generator(proto_dir, out_dir)

    manifest = _load_manifest(out_dir)

    assert manifest["layout"] == "pyramid"
    assert "pyramid_data_model_types.hpp" in manifest["types"]
    assert (
        "pyramid_services_tactical_objects_provided.cpp"
        in manifest["services"]
    )
    assert any(
        plugin["content_type"] == "application/json"
        and plugin["source"]
        == "pyramid_services_tactical_objects_json_codec_plugin.cpp"
        for plugin in manifest["codec_plugins"]
    )
    _assert_manifest_artifacts_exist(out_dir, manifest)


def test_pyramid_manifest_only_adds_manifest_to_locked_outputs(
    tmp_path: Path,
) -> None:
    out_dir = tmp_path / "pyramid_out"
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"

    _run_generator(proto_dir, out_dir)

    generated = {path.name for path in out_dir.iterdir() if path.is_file()}

    assert "binding_manifest.json" in generated
    assert PYRAMID_CPP_JSON_FILENAMES.issubset(generated)
    assert generated - {"binding_manifest.json"} == PYRAMID_CPP_JSON_FILENAMES
