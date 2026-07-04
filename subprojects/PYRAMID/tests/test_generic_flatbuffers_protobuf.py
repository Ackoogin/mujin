#!/usr/bin/env python3
"""Generic FlatBuffers/Protobuf backend binding tests."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
GENERATOR = PYRAMID_ROOT / "pim" / "generate_bindings.py"


TELEMETRY_PROTO = """syntax = "proto3";
package example.telemetry;

message Pose {
  double x = 1;
  double y = 2;
}

service Telemetry {
  rpc GetPose(Pose) returns (Pose);
}
"""


PYRAMID_FLATBUFFERS_CPP_FILENAMES = {
    "pyramid_data_model_autonomy.fbs",
    "pyramid_data_model_autonomy_flatbuffers_codec.hpp",
    "pyramid_data_model_base.fbs",
    "pyramid_data_model_base_flatbuffers_codec.hpp",
    "pyramid_data_model_common.fbs",
    "pyramid_data_model_common_flatbuffers_codec.hpp",
    "pyramid_data_model_radar.fbs",
    "pyramid_data_model_radar_flatbuffers_codec.hpp",
    "pyramid_data_model_sensorproducts.fbs",
    "pyramid_data_model_sensorproducts_flatbuffers_codec.hpp",
    "pyramid_data_model_sensors.fbs",
    "pyramid_data_model_sensors_flatbuffers_codec.hpp",
    "pyramid_data_model_tactical.fbs",
    "pyramid_data_model_tactical_flatbuffers_codec.hpp",
    "pyramid_services_autonomy_backend.fbs",
    "pyramid_services_autonomy_backend_flatbuffers_codec.cpp",
    "pyramid_services_autonomy_backend_flatbuffers_codec.hpp",
    "pyramid_services_sensor_data_interpretation.fbs",
    "pyramid_services_sensor_data_interpretation_flatbuffers_codec.cpp",
    "pyramid_services_sensor_data_interpretation_flatbuffers_codec.hpp",
    "pyramid_services_tactical_objects.fbs",
    "pyramid_services_tactical_objects_flatbuffers_codec.cpp",
    "pyramid_services_tactical_objects_flatbuffers_codec.hpp",
}


PYRAMID_PROTOBUF_CPP_FILENAMES = {
    "pyramid_data_model_autonomy_protobuf_codec.hpp",
    "pyramid_data_model_base_protobuf_codec.hpp",
    "pyramid_data_model_common_protobuf_codec.hpp",
    "pyramid_data_model_sensorproducts_protobuf_codec.hpp",
    "pyramid_data_model_sensors_protobuf_codec.hpp",
    "pyramid_data_model_tactical_protobuf_codec.hpp",
}


def _write_telemetry_proto(tmp_path: Path) -> Path:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "telemetry.proto").write_text(
        TELEMETRY_PROTO, encoding="utf-8")
    return proto_dir


def _run_generator(
    proto_dir: Path,
    out_dir: Path,
    backend: str,
    layout: str = "generic",
) -> None:
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            str(proto_dir),
            str(out_dir),
            "--languages",
            "cpp",
            "--backends",
            backend,
            "--contract-layout",
            layout,
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _generated_text(out_dir: Path) -> str:
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted(out_dir.rglob("*"))
        if path.is_file()
    )


def _manifest(out_dir: Path) -> dict:
    return json.loads(
        (out_dir / "binding_manifest.json").read_text(encoding="utf-8"))


def test_generic_flatbuffers_generation_has_no_pyramid_leak(
    tmp_path: Path,
) -> None:
    proto_dir = _write_telemetry_proto(tmp_path)
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "flatbuffers")

    cpp_dir = out_dir / "flatbuffers" / "cpp"
    assert (cpp_dir / "example_telemetry.fbs").exists()
    assert (cpp_dir / "example_telemetry_flatbuffers_codec.hpp").exists()
    assert (cpp_dir / "example_telemetry_services.fbs").exists()
    assert (
        cpp_dir / "example_telemetry_services_flatbuffers_codec.cpp"
    ).exists()
    assert "flatbuffers/cpp/example_telemetry.fbs" in (
        _manifest(out_dir)["flatbuffers_schemas"])

    text = _generated_text(out_dir)
    assert "namespace example.telemetry;" in text
    assert "namespace example::telemetry::flatbuffers_codec" in text
    assert "example::telemetry::Pose" in text
    assert "pyramid_data_model_types.hpp" not in text
    assert "pyramid" not in text.lower()


def test_generic_protobuf_wrapper_uses_import_root_relative_include(
    tmp_path: Path,
) -> None:
    proto_dir = _write_telemetry_proto(tmp_path)
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "protobuf")

    wrapper = (
        out_dir / "protobuf" / "cpp" /
        "example_telemetry_protobuf_codec.hpp"
    )
    assert wrapper.exists()
    text = wrapper.read_text(encoding="utf-8")

    assert '#include "telemetry.pb.h"' in text
    assert "namespace example::telemetry::protobuf_codec" in text
    assert "example::telemetry::Pose" in text
    assert "pyramid/" not in text
    assert "pyramid" not in text.lower()
    assert _manifest(out_dir)["protobuf_protos"] == ["telemetry.proto"]


def test_pyramid_flatbuffers_and_protobuf_outputs_keep_legacy_names(
    tmp_path: Path,
) -> None:
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"
    flat_out = tmp_path / "flat"
    pb_out = tmp_path / "pb"

    _run_generator(proto_dir, flat_out, "flatbuffers", layout="pyramid")
    _run_generator(proto_dir, pb_out, "protobuf", layout="pyramid")

    flat_files = {
        path.name
        for path in (flat_out / "flatbuffers" / "cpp").iterdir()
        if path.is_file()
    }
    pb_files = {
        path.name
        for path in (pb_out / "protobuf" / "cpp").iterdir()
        if path.is_file()
    }

    assert flat_files == PYRAMID_FLATBUFFERS_CPP_FILENAMES
    assert pb_files == PYRAMID_PROTOBUF_CPP_FILENAMES

    tactical_hpp = (
        flat_out / "flatbuffers" / "cpp" /
        "pyramid_services_tactical_objects_flatbuffers_codec.hpp"
    ).read_text(encoding="utf-8")
    tactical_fbs = (
        flat_out / "flatbuffers" / "cpp" /
        "pyramid_services_tactical_objects.fbs"
    ).read_text(encoding="utf-8")
    base_pb = (
        pb_out / "protobuf" / "cpp" /
        "pyramid_data_model_base_protobuf_codec.hpp"
    ).read_text(encoding="utf-8")

    assert "namespace pyramid::services::tactical_objects::flatbuffers_codec" in tactical_hpp
    assert '#include "pyramid_data_model_types.hpp"' in tactical_hpp
    assert "namespace data_model = pyramid::domain_model;" in tactical_hpp
    assert "namespace pyramid.services.tactical_objects;" in tactical_fbs
    assert '#include "pyramid/data_model/pyramid.data_model.base.pb.h"' in base_pb
    assert "namespace pyramid::data_model::base::protobuf_codec" in base_pb


def test_pim_flatbuffers_json_bridge_uses_full_namespace_for_nested_data_model(
    tmp_path: Path,
) -> None:
    proto_dir = PYRAMID_ROOT / "pim" / "test"
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "flatbuffers", layout="pyramid")

    bridge_cpp = (
        out_dir / "flatbuffers" / "cpp" /
        "pyramid_services_pim_osprey_sensor_products_flatbuffers_codec.cpp"
    )
    assert bridge_cpp.exists()
    text = bridge_cpp.read_text(encoding="utf-8")

    nested_namespace = (
        "pyramid::domain_model::common_pim_components::authorisation"
    )
    assert (
        '#include "pyramid_data_model_common_pim_components_authorisation_codec.hpp"'
        in text
    )
    assert '#include "pyramid_data_model_common_pim_components_codec.hpp"' not in text
    assert f"{nested_namespace}::fromJson" in text
    assert f"{nested_namespace}::toJson" in text
    assert f"static_cast<{nested_namespace}::AUTRequirement*>(nullptr)" in text
    assert "pyramid::domain_model::common_pim_components::fromJson" not in text
    assert "pyramid::domain_model::common_pim_components::toJson" not in text
