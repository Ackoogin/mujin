#!/usr/bin/env python3
"""The CMake manifest helper resolves generator manifests into source lists.

Runs the standalone `cmake -P` test against freshly generated manifests for
both the pyramid and generic contract layouts, so the manifest->build contract
is covered by the Python test suite.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
GENERATOR = PYRAMID_ROOT / "pim" / "generate_bindings.py"
CMAKE_TEST = PYRAMID_ROOT / "cmake" / "tests" / "test_pyramid_manifest.cmake"
CMAKE_SOURCES_TEST = (
    PYRAMID_ROOT / "cmake" / "tests" / "test_pyramid_binding_sources.cmake"
)

pytestmark = pytest.mark.skipif(
    shutil.which("cmake") is None, reason="cmake not available"
)


def _generate(proto_dir: Path, out_dir: Path, *extra: str) -> None:
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            str(proto_dir),
            str(out_dir),
            "--languages",
            "cpp",
            "--backends",
            "json,flatbuffers",
            *extra,
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _run_cmake_script(script: Path, out_dir: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            "cmake",
            f"-DMANIFEST={out_dir / 'binding_manifest.json'}",
            f"-DBINDINGS_DIR={out_dir}",
            "-P",
            str(script),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def _run_cmake_helper_test(out_dir: Path) -> subprocess.CompletedProcess:
    return _run_cmake_script(CMAKE_TEST, out_dir)


def test_cmake_helper_reads_pyramid_manifest(tmp_path: Path) -> None:
    out_dir = tmp_path / "out"
    _generate(PYRAMID_ROOT / "proto" / "pyramid", out_dir)
    result = _run_cmake_helper_test(out_dir)
    assert result.returncode == 0, result.stdout
    assert "layout=pyramid" in result.stdout


def test_cmake_helper_reads_generic_manifest(tmp_path: Path) -> None:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "telemetry.proto").write_text(
        'syntax = "proto3";\n'
        "package example.telemetry;\n"
        "message Pose { double x = 1; }\n"
        "service Telemetry { rpc GetPose(Pose) returns (Pose); }\n",
        encoding="utf-8",
    )
    out_dir = tmp_path / "out"
    _generate(proto_dir, out_dir, "--contract-layout", "generic")
    result = _run_cmake_helper_test(out_dir)
    assert result.returncode == 0, result.stdout
    assert "layout=generic" in result.stdout


def test_binding_sources_selection_glob_vs_manifest(tmp_path: Path) -> None:
    """The glob/manifest source-selection wrapper picks the right list and
    falls back to globs for roles absent from the manifest."""
    out_dir = tmp_path / "out"
    _generate(PYRAMID_ROOT / "proto" / "pyramid", out_dir)
    result = _run_cmake_script(CMAKE_SOURCES_TEST, out_dir)
    assert result.returncode == 0, result.stdout
    assert "pyramid_binding_sources test OK" in result.stdout
