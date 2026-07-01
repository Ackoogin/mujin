#!/usr/bin/env python3
"""Generic gRPC/ROS2 transport backend binding tests."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
GENERATOR = PYRAMID_ROOT / "pim" / "generate_bindings.py"


TYPES_PROTO = """syntax = "proto3";
package example.telemetry;

message Pose {
  double x = 1;
  double y = 2;
}
"""


SERVICE_PROTO = """syntax = "proto3";
package example.telemetry;

import "types.proto";

message PoseRequest {
  string entity_id = 1;
}

service Telemetry {
  rpc GetPose(PoseRequest) returns (Pose);
}
"""


PYRAMID_ROS2_CPP_FILENAMES = {
    "pyramid_components_autonomy_backend_services_provided_ros2_transport.cpp",
    "pyramid_components_autonomy_backend_services_provided_ros2_transport.hpp",
    "pyramid_components_sensor_data_interpretation_services_consumed_ros2_transport.cpp",
    "pyramid_components_sensor_data_interpretation_services_consumed_ros2_transport.hpp",
    "pyramid_components_sensor_data_interpretation_services_provided_ros2_transport.cpp",
    "pyramid_components_sensor_data_interpretation_services_provided_ros2_transport.hpp",
    "pyramid_components_tactical_objects_services_consumed_ros2_transport.cpp",
    "pyramid_components_tactical_objects_services_consumed_ros2_transport.hpp",
    "pyramid_components_tactical_objects_services_provided_ros2_transport.cpp",
    "pyramid_components_tactical_objects_services_provided_ros2_transport.hpp",
    "pyramid_ros2_transport_support.cpp",
    "pyramid_ros2_transport_support.hpp",
}


PYRAMID_GRPC_CPP_FILENAMES = {
    "pyramid_components_autonomy_backend_services_provided_grpc_transport.cpp",
    "pyramid_components_autonomy_backend_services_provided_grpc_transport.hpp",
    "pyramid_components_sensor_data_interpretation_services_consumed_grpc_transport.cpp",
    "pyramid_components_sensor_data_interpretation_services_consumed_grpc_transport.hpp",
    "pyramid_components_sensor_data_interpretation_services_provided_grpc_transport.cpp",
    "pyramid_components_sensor_data_interpretation_services_provided_grpc_transport.hpp",
    "pyramid_components_tactical_objects_services_consumed_grpc_transport.cpp",
    "pyramid_components_tactical_objects_services_consumed_grpc_transport.hpp",
    "pyramid_components_tactical_objects_services_provided_grpc_transport.cpp",
    "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp",
    "pyramid_grpc_plugin_aggregator.cpp",
    "pyramid_grpc_plugin_aggregator.hpp",
}


def _write_telemetry_proto(tmp_path: Path) -> Path:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "types.proto").write_text(TYPES_PROTO, encoding="utf-8")
    (proto_dir / "service.proto").write_text(SERVICE_PROTO, encoding="utf-8")
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


def _cpp_filenames(out_dir: Path, backend: str) -> set[str]:
    return {
        path.name
        for path in (out_dir / backend / "cpp").iterdir()
        if path.is_file()
    }


def test_generic_ros2_generates_flat_namespace_service_transport(
    tmp_path: Path,
) -> None:
    proto_dir = _write_telemetry_proto(tmp_path)
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "ros2")

    cpp_dir = out_dir / "ros2" / "cpp"
    assert (cpp_dir / "example_telemetry_ros2_transport_support.hpp").exists()
    assert (cpp_dir / "example_telemetry_ros2_transport_support.cpp").exists()
    assert (cpp_dir / "example_telemetry_services_ros2_transport.hpp").exists()
    assert (cpp_dir / "example_telemetry_services_ros2_transport.cpp").exists()
    assert (out_dir / "ros2" / "idl" / "srv" / "TelemetryGetPose.srv").exists()

    text = _generated_text(out_dir)
    assert "namespace example::telemetry::transport::ros2" in text
    assert "namespace example::telemetry::ros2_transport" in text
    assert '#include "example_telemetry_ros2_transport_support.hpp"' in text
    assert "example_telemetry_ros2/PclEnvelope" in text
    assert "pyramid" not in text.lower()


def test_generic_grpc_generates_flat_namespace_service_transport(
    tmp_path: Path,
) -> None:
    proto_dir = _write_telemetry_proto(tmp_path)
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "grpc")

    cpp_dir = out_dir / "grpc" / "cpp"
    assert (cpp_dir / "example_telemetry_services_grpc_transport.hpp").exists()
    assert (cpp_dir / "example_telemetry_services_grpc_transport.cpp").exists()
    assert (cpp_dir / "example_telemetry_grpc_plugin_aggregator.hpp").exists()
    assert (cpp_dir / "example_telemetry_grpc_plugin_aggregator.cpp").exists()

    text = _generated_text(out_dir)
    assert '#include "service.pb.h"' in text
    assert '#include "service.grpc.pb.h"' in text
    assert "namespace example::telemetry::grpc_transport" in text
    assert "example_telemetry_grpc_plugin_server_start" in text
    assert "::example::telemetry::PoseRequest" in text
    assert "pyramid" not in text.lower()
    assert _manifest(out_dir)["grpc_service_protos"] == ["service.proto"]


def test_pyramid_ros2_output_keeps_legacy_names_and_symbols(
    tmp_path: Path,
) -> None:
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "ros2", layout="pyramid")

    assert _cpp_filenames(out_dir, "ros2") == PYRAMID_ROS2_CPP_FILENAMES

    support = (
        out_dir / "ros2" / "cpp" / "pyramid_ros2_transport_support.hpp"
    ).read_text(encoding="utf-8")
    facade = (
        out_dir / "ros2" / "cpp" /
        "pyramid_components_tactical_objects_services_provided_ros2_transport.hpp"
    ).read_text(encoding="utf-8")

    assert "namespace pyramid::transport::ros2" in support
    assert 'kEnvelopeType = "pyramid_ros2/PclEnvelope"' in support
    assert '#include "pyramid_ros2_transport_support.hpp"' in facade
    assert "pyramid::transport::ros2::Adapter" in facade


def test_pyramid_grpc_output_keeps_legacy_names_and_symbols(
    tmp_path: Path,
) -> None:
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "grpc", layout="pyramid")

    assert _cpp_filenames(out_dir, "grpc") == PYRAMID_GRPC_CPP_FILENAMES

    aggregator = (
        out_dir / "grpc" / "cpp" / "pyramid_grpc_plugin_aggregator.hpp"
    ).read_text(encoding="utf-8")
    facade = (
        out_dir / "grpc" / "cpp" /
        "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"
    ).read_text(encoding="utf-8")

    assert "typedef struct pyramid_grpc_plugin_server" in aggregator
    assert "pyramid_grpc_plugin_server_start" in aggregator
    assert "pyramid_grpc_plugin_client_invoke_unary" in aggregator
    assert "namespace pyramid::components::tactical_objects::services::provided::grpc_transport" in facade
    assert '#include "pyramid/components/pyramid.components.tactical_objects.services.provided.grpc.pb.h"' in facade
