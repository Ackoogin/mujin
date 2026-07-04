#!/usr/bin/env python3
"""Generic-layout Ada type/codec generation for arbitrary proto packages."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
GENERATOR = PYRAMID_ROOT / "pim" / "generate_bindings.py"
GNATCOLL_SRC = (
    PYRAMID_ROOT / "core" / "external" / "gnatcoll-core" / "core" / "src"
)
GENERIC_CONSUMED_PARENT_STUBS = {
    "pyramid-components-pim_osprey-sensors-services-consumed.ads":
        "Pyramid.Components.Pim_Osprey.Sensors.Services.Consumed",
    "pyramid-components-pim_osprey-tactical_objects-services-consumed.ads":
        "Pyramid.Components.Pim_Osprey.Tactical_Objects.Services.Consumed",
    "pyramid-components-pim_seaspray-sensors-services-consumed.ads":
        "Pyramid.Components.Pim_Seaspray.Sensors.Services.Consumed",
}
GENERIC_CONSUMED_CHILD_SERVICES = [
    "pyramid-components-pim_osprey-sensors-services-consumed-services.ads",
    "pyramid-components-pim_osprey-tactical_objects-services-consumed-services.ads",
    "pyramid-components-pim_seaspray-sensors-services-consumed-services.ads",
]


def _run(proto_dir: Path, out_dir: Path, *extra: str) -> None:
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            str(proto_dir),
            str(out_dir),
            "--languages",
            "ada",
            "--backends",
            "json",
            *extra,
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_generic_ada_types_and_codec_for_arbitrary_package(tmp_path: Path) -> None:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "telemetry.proto").write_text(
        'syntax = "proto3";\n'
        "package example.telemetry;\n"
        "message Pose { double x = 1; double y = 2; }\n"
        "service Telemetry { rpc GetPose(Pose) returns (Pose); }\n",
        encoding="utf-8",
    )
    out_dir = tmp_path / "out"
    _run(proto_dir, out_dir, "--contract-layout", "generic")

    names = {p.name for p in out_dir.iterdir() if p.is_file()}
    assert "example-telemetry-types.ads" in names
    assert "example-telemetry-types_codec.ads" in names
    assert "example-telemetry-types_codec.adb" in names

    # Role-neutral service facade for the (non-CRUD) Telemetry service.
    assert "example-telemetry-services.ads" in names
    facade = (out_dir / "example-telemetry-services.ads").read_text(
        encoding="utf-8")
    assert "package Example.Telemetry.Services is" in facade
    assert "Handle_Telemetry_Get_Pose" in facade
    assert "access function (Request : Pose) return Pose" in facade

    all_text = "\n".join(
        p.read_text(encoding="utf-8")
        for p in out_dir.iterdir()
        if p.suffix in (".ads", ".adb")
    )
    # Package derives from the proto package, with no Pyramid.* root.
    assert "package Example.Telemetry.Types is" in all_text
    assert "Pyramid" not in all_text
    assert "pyramid" not in "\n".join(names).lower()


def test_generic_ada_emits_consumed_parent_stubs_for_child_services(
    tmp_path: Path,
) -> None:
    out_dir = tmp_path / "out"
    _run(PYRAMID_ROOT / "pim" / "test", out_dir, "--contract-layout", "generic")

    for file_name, package_name in GENERIC_CONSUMED_PARENT_STUBS.items():
        path = out_dir / file_name
        assert path.exists()
        text = path.read_text(encoding="utf-8")
        assert f"package {package_name} is" in text
        assert f"end {package_name};" in text


@pytest.mark.skipif(
    (shutil.which("gnatgcc") or shutil.which("gcc")) is None,
    reason="GNAT gcc driver not available",
)
def test_generic_ada_consumed_child_services_object_compile(
    tmp_path: Path,
) -> None:
    out_dir = tmp_path / "out"
    _run(PYRAMID_ROOT / "pim" / "test", out_dir, "--contract-layout", "generic")

    # The PIM fixture references google.protobuf.Empty but does not generate
    # google.protobuf types; provide the minimal external specs needed to
    # object-compile the generated service children and catch missing parents.
    (out_dir / "google.ads").write_text(
        "package Google is\nend Google;\n",
        encoding="utf-8",
    )
    (out_dir / "google-protobuf.ads").write_text(
        "package Google.Protobuf is\nend Google.Protobuf;\n",
        encoding="utf-8",
    )
    (out_dir / "google-protobuf-types.ads").write_text(
        "package Google.Protobuf.Types is\n"
        "   type Empty is null record;\n"
        "end Google.Protobuf.Types;\n",
        encoding="utf-8",
    )

    compiler = shutil.which("gnatgcc") or shutil.which("gcc")
    for file_name in GENERIC_CONSUMED_CHILD_SERVICES:
        result = subprocess.run(
            [
                compiler,
                "-c",
                "-gnat2020",
                "-I.",
                file_name,
            ],
            cwd=out_dir,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        assert result.returncode == 0, result.stdout


@pytest.mark.skipif(
    shutil.which("gnatmake") is None or not GNATCOLL_SRC.is_dir(),
    reason="gnatmake or in-tree GNATCOLL not available",
)
def test_generic_ada_compiles_under_gnat(tmp_path: Path) -> None:
    """Generated generic Ada (types + codec + role-neutral facade) passes a
    GNAT semantic compile, including a non-CRUD RPC."""
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "telemetry.proto").write_text(
        'syntax = "proto3";\n'
        "package example.telemetry;\n"
        "message Pose { double x = 1; double y = 2; }\n"
        "message PoseRequest { Pose pose = 1; }\n"
        "service Telemetry {\n"
        "  rpc GetPose(PoseRequest) returns (Pose);\n"
        "  rpc Reset(Pose) returns (Pose);\n"
        "}\n",
        encoding="utf-8",
    )
    out_dir = tmp_path / "out"
    _run(proto_dir, out_dir, "--contract-layout", "generic")

    result = subprocess.run(
        [
            "gnatmake",
            "-gnatc",
            "-gnat2012",
            "-I.",
            f"-I{GNATCOLL_SRC}",
            "example-telemetry-services.ads",
            "example-telemetry-types_codec.ads",
        ],
        cwd=out_dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    assert result.returncode == 0, result.stdout


def test_pyramid_ada_still_uses_pyramid_packages(tmp_path: Path) -> None:
    out_dir = tmp_path / "out"
    _run(PYRAMID_ROOT / "proto" / "pyramid", out_dir)
    names = {p.name for p in out_dir.iterdir() if p.is_file()}
    # Compatibility layout keeps the Pyramid.Data_Model.* Ada packages.
    assert any(n.startswith("pyramid-data_model-") for n in names)
    tactical = out_dir / "pyramid-data_model-tactical-types.ads"
    assert tactical.exists()
    assert "package Pyramid.Data_Model.Tactical.Types is" in tactical.read_text(
        encoding="utf-8"
    )
