#!/usr/bin/env python3
"""Generic binding contract and C++ JSON generation tests."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
PIM_DIR = PYRAMID_ROOT / "pim"
GENERATOR = PIM_DIR / "generate_bindings.py"

sys.path.insert(0, str(PIM_DIR))

from binding_contract import build_contract  # noqa: E402
from proto_parser import parse_proto_tree  # noqa: E402


TELEMETRY_PROTO = """syntax = "proto3";
package example.telemetry;
message Pose { double x = 1; double y = 2; }
service Telemetry { rpc GetPose(Pose) returns (Pose); }
"""


PYRAMID_CPP_JSON_FILENAMES = {
    "pyramid_data_model_autonomy_cabi.h",
    "pyramid_data_model_autonomy_cabi_marshal.cpp",
    "pyramid_data_model_autonomy_cabi_marshal.hpp",
    "pyramid_data_model_autonomy_codec.cpp",
    "pyramid_data_model_autonomy_codec.hpp",
    "pyramid_data_model_autonomy_types.hpp",
    "pyramid_data_model_base_cabi.h",
    "pyramid_data_model_base_cabi_marshal.cpp",
    "pyramid_data_model_base_cabi_marshal.hpp",
    "pyramid_data_model_base_codec.cpp",
    "pyramid_data_model_base_codec.hpp",
    "pyramid_data_model_base_types.hpp",
    "pyramid_data_model_common_cabi.h",
    "pyramid_data_model_common_cabi_marshal.cpp",
    "pyramid_data_model_common_cabi_marshal.hpp",
    "pyramid_data_model_common_codec.cpp",
    "pyramid_data_model_common_codec.hpp",
    "pyramid_data_model_common_types.hpp",
    "pyramid_data_model_radar_cabi.h",
    "pyramid_data_model_radar_cabi_marshal.cpp",
    "pyramid_data_model_radar_cabi_marshal.hpp",
    "pyramid_data_model_radar_codec.cpp",
    "pyramid_data_model_radar_codec.hpp",
    "pyramid_data_model_radar_types.hpp",
    "pyramid_data_model_sensorproducts_cabi.h",
    "pyramid_data_model_sensorproducts_cabi_marshal.cpp",
    "pyramid_data_model_sensorproducts_cabi_marshal.hpp",
    "pyramid_data_model_sensorproducts_codec.cpp",
    "pyramid_data_model_sensorproducts_codec.hpp",
    "pyramid_data_model_sensorproducts_types.hpp",
    "pyramid_data_model_sensors_cabi.h",
    "pyramid_data_model_sensors_cabi_marshal.cpp",
    "pyramid_data_model_sensors_cabi_marshal.hpp",
    "pyramid_data_model_sensors_codec.cpp",
    "pyramid_data_model_sensors_codec.hpp",
    "pyramid_data_model_sensors_types.hpp",
    "pyramid_data_model_tactical_cabi.h",
    "pyramid_data_model_tactical_cabi_marshal.cpp",
    "pyramid_data_model_tactical_cabi_marshal.hpp",
    "pyramid_data_model_tactical_codec.cpp",
    "pyramid_data_model_tactical_codec.hpp",
    "pyramid_data_model_tactical_types.hpp",
    "pyramid_data_model_types.hpp",
    "pyramid_datamodel_cabi.h",
    "pyramid_services_autonomy_backend_json_codec_plugin.cpp",
    "pyramid_services_autonomy_backend_provided.cpp",
    "pyramid_services_autonomy_backend_provided.hpp",
    "pyramid_services_autonomy_backend_provided_components.hpp",
    "pyramid_services_sensor_data_interpretation_consumed.cpp",
    "pyramid_services_sensor_data_interpretation_consumed.hpp",
    "pyramid_services_sensor_data_interpretation_consumed_components.hpp",
    "pyramid_services_sensor_data_interpretation_json_codec_plugin.cpp",
    "pyramid_services_sensor_data_interpretation_provided.cpp",
    "pyramid_services_sensor_data_interpretation_provided.hpp",
    "pyramid_services_sensor_data_interpretation_provided_components.hpp",
    "pyramid_services_tactical_objects_consumed.cpp",
    "pyramid_services_tactical_objects_consumed.hpp",
    "pyramid_services_tactical_objects_consumed_components.hpp",
    "pyramid_services_tactical_objects_json_codec_plugin.cpp",
    "pyramid_services_tactical_objects_provided.cpp",
    "pyramid_services_tactical_objects_provided.hpp",
    "pyramid_services_tactical_objects_provided_components.hpp",
}


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


@pytest.fixture()
def telemetry_proto_dir(tmp_path: Path) -> Path:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "telemetry.proto").write_text(
        TELEMETRY_PROTO, encoding="utf-8")
    return proto_dir


def test_generic_cpp_json_generation_for_arbitrary_package(
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

    generated = {path.name for path in out_dir.iterdir()}
    assert "example_telemetry_types.hpp" in generated
    assert "example_telemetry_codec.hpp" in generated
    assert "example_telemetry_codec.cpp" in generated
    assert "example_telemetry_services.hpp" in generated
    assert "example_telemetry_services.cpp" in generated

    all_text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted(out_dir.iterdir())
        if path.is_file()
    )
    assert "namespace example::telemetry" in all_text
    assert "namespace pyramid" not in all_text
    assert "pyramid" not in "\n".join(generated).lower()
    assert "pyramid" not in all_text.lower()
    assert "class ServiceHandler" in all_text
    assert "kSvcTelemetryGetPose" in all_text
    assert "handleTelemetryGetPose" in all_text
    assert "Pose" in all_text


def test_contract_classifies_by_content_and_computes_service_closure(
    tmp_path: Path,
) -> None:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "types.proto").write_text(
        """syntax = "proto3";
package example.telemetry;
message Pose { double x = 1; double y = 2; }
""",
        encoding="utf-8",
    )
    (proto_dir / "service.proto").write_text(
        """syntax = "proto3";
package example.telemetry;
service Telemetry { rpc GetPose(Pose) returns (Pose); }
""",
        encoding="utf-8",
    )
    (proto_dir / "wrapper.proto").write_text(
        """syntax = "proto3";
package example.telemetry.wrapper;
message PoseRequest { example.telemetry.Pose pose = 1; }
service WrappedTelemetry {
  rpc GetPose(PoseRequest) returns (example.telemetry.Pose);
}
""",
        encoding="utf-8",
    )

    files = parse_proto_tree(proto_dir)
    contract = build_contract(files, layout="generic")

    type_packages = {pf.package for pf in contract.type_modules}
    service_packages = {pf.package for pf in contract.service_modules}
    wrapper_packages = {pf.package for pf in contract.wrapper_modules}

    assert type_packages == {
        "example.telemetry",
        "example.telemetry.wrapper",
    }
    assert service_packages == {
        "example.telemetry",
        "example.telemetry.wrapper",
    }
    assert wrapper_packages == {"example.telemetry.wrapper"}
    assert "example.telemetry.Pose" in contract.schema_ids

    closure = contract.service_type_closures[
        "example.telemetry.Telemetry"].types
    assert "example.telemetry.Pose" in closure

    wrapper_closure = contract.service_type_closures[
        "example.telemetry.wrapper.WrappedTelemetry"].types
    assert "example.telemetry.Pose" in wrapper_closure
    assert "example.telemetry.wrapper.PoseRequest" in wrapper_closure


def test_default_pyramid_layout_cpp_json_regression(tmp_path: Path) -> None:
    out_dir = tmp_path / "pyramid_out"
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"

    _run_generator(proto_dir, out_dir)

    generated = {path.name for path in out_dir.iterdir() if path.is_file()}
    assert "binding_manifest.json" in generated
    assert generated - {"binding_manifest.json"} == PYRAMID_CPP_JSON_FILENAMES

    umbrella = (out_dir / "pyramid_data_model_types.hpp").read_text(
        encoding="utf-8")
    tactical = (
        out_dir / "pyramid_services_tactical_objects_provided.hpp"
    ).read_text(encoding="utf-8")

    assert "pyramid::domain_model" in umbrella
    assert "namespace pyramid::components::tactical_objects::services::provided" in tactical
    assert any(
        name.startswith("pyramid_services_tactical_objects_provided")
        for name in generated
    )
