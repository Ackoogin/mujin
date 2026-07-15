#!/usr/bin/env python3
"""Generic FlatBuffers/Protobuf backend binding tests."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
GENERATOR = PYRAMID_ROOT / "pim" / "generate_bindings.py"


def _find_flatc():
    """The flatc binary, or None. FLATC wins; otherwise take one from a
    configured build tree, then PATH."""
    env = os.environ.get("FLATC")
    if env and Path(env).is_file():
        return Path(env)
    name = "flatc.exe" if os.name == "nt" else "flatc"
    for build in sorted(REPO_ROOT.glob("build*")):
        deps = build / "_deps"
        if not deps.is_dir():
            continue
        for found in deps.rglob(name):
            # rglob also matches CMake's flatc.dir/ and similar directories.
            if found.is_file():
                return found
    which = shutil.which("flatc")
    return Path(which) if which else None


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
    # Generated local-struct <-> protobuf service codecs (C1: retired the
    # hand-maintained src/protobuf_support/ tactical_objects bridge; the
    # generator now emits a codec per service package).
    "pyramid_services_autonomy_backend_protobuf_codec.hpp",
    "pyramid_services_autonomy_backend_protobuf_codec.cpp",
    "pyramid_services_sensor_data_interpretation_protobuf_codec.hpp",
    "pyramid_services_sensor_data_interpretation_protobuf_codec.cpp",
    "pyramid_services_tactical_objects_protobuf_codec.hpp",
    "pyramid_services_tactical_objects_protobuf_codec.cpp",
}


# The shapes an XSD-derived contract produces that the FlatBuffers backend
# used to get wrong. Every one of these was a real defect, found when the
# A-GRA 5.0a P2 contract first reached this backend:
#
#   * an enum-typed arm (Marking.government_identifier) -- a union member must
#     be a table, so this needs a wrapper;
#   * two arms of the same message type (Trigger.primary/secondary) -- an
#     unnamed arm contributes its type name to the union's implicit enum, so
#     these collide;
#   * two messages whose oneofs share a name (both "choice", which is what
#     xsd2proto names every xs:choice) -- keying the union on that name alone
#     gives both messages the same union;
#   * a bytes field (Identifier.uuid, from xs:hexBinary) -- projected as
#     [ubyte] it becomes a std::vector<uint8_t> on the FlatBuffers side and no
#     longer assigns to the generated struct's std::string.
CHOICE_PROTO = """syntax = "proto3";
package example.choice;

enum GovernmentEnum {
  GOVERNMENT_ENUM_UNSPECIFIED = 0;
  GOVERNMENT_ENUM_USA = 1;
}

message TaxonomyType {
  string label = 1;
}

message Identifier {
  bytes uuid = 1;
}

message Marking {
  oneof choice {
    GovernmentEnum government_identifier = 1;
    string nato_special_word = 2;
  }
}

message Trigger {
  oneof choice {
    TaxonomyType primary = 1;
    TaxonomyType secondary = 2;
  }
}
"""


def _write_telemetry_proto(tmp_path: Path) -> Path:
    proto_dir = tmp_path / "proto"
    proto_dir.mkdir()
    (proto_dir / "telemetry.proto").write_text(
        TELEMETRY_PROTO, encoding="utf-8")
    return proto_dir


def _write_choice_proto(tmp_path: Path) -> Path:
    proto_dir = tmp_path / "choice_proto"
    proto_dir.mkdir()
    (proto_dir / "choice.proto").write_text(CHOICE_PROTO, encoding="utf-8")
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


def test_pyramid_protobuf_service_codec_is_generated_with_common_namespaces(
    tmp_path: Path,
) -> None:
    """C1: the tactical_objects service protobuf codec (formerly the checked-in
    src/protobuf_support/ bridge) is now generated on the pyramid compat layout,
    referencing the current sub-namespaced data-model types
    (pyramid::domain_model::common::*), not the stale flat namespace."""
    out_dir = tmp_path / "out"
    _run_generator(PYRAMID_ROOT / "proto" / "pyramid", out_dir, "protobuf",
                   layout="pyramid")

    hpp = out_dir / "protobuf" / "cpp" / (
        "pyramid_services_tactical_objects_protobuf_codec.hpp")
    cpp = out_dir / "protobuf" / "cpp" / (
        "pyramid_services_tactical_objects_protobuf_codec.cpp")
    assert hpp.exists() and cpp.exists()

    hpp_text = hpp.read_text(encoding="utf-8")
    cpp_text = cpp.read_text(encoding="utf-8")
    assert "namespace pyramid::services::tactical_objects::protobuf_codec" in hpp_text
    assert "toBinary(const pyramid::domain_model::common::GeodeticPosition&" in hpp_text
    assert "fromBinaryGeodeticPosition(" in hpp_text
    assert "SerializeToString" in cpp_text and "ParseFromArray" in cpp_text
    # Must use the current sub-namespaced types, not the stale flat spelling.
    assert "pyramid::domain_model::GeodeticPosition" not in hpp_text


def _fbs_text(out_dir: Path) -> str:
    schemas = list((out_dir / "flatbuffers" / "cpp").glob("*.fbs"))
    assert len(schemas) == 1, schemas
    return schemas[0].read_text(encoding="utf-8")


def test_flatbuffers_union_arms_are_tables_named_per_message_and_field(
    tmp_path: Path,
) -> None:
    """A FlatBuffers union may only hold tables, its members' names must be
    unique within it, and it belongs to one message's oneof.

    Regression bar for three defects the A-GRA 5.0a P2 contract exposed; see
    CHOICE_PROTO above for what each shape stands for."""
    out_dir = tmp_path / "out"
    _run_generator(_write_choice_proto(tmp_path), out_dir, "flatbuffers")
    fbs = _fbs_text(out_dir)

    # One union per (message, oneof), not one per oneof name.
    assert "union MarkingChoiceUnion {" in fbs
    assert "union TriggerChoiceUnion {" in fbs
    assert "union ChoiceUnion {" not in fbs
    assert "choice:MarkingChoiceUnion;" in fbs
    assert "choice:TriggerChoiceUnion;" in fbs

    # The enum arm is wrapped in a table; the bare enum never appears as a
    # union member, which is what flatc rejects.
    assert ("table MarkingChoiceUnionGovernmentIdentifierValue {\n"
            "  value:GovernmentEnum;\n}") in fbs
    assert "GovernmentIdentifier:MarkingChoiceUnionGovernmentIdentifierValue" \
        in fbs
    assert "\n  GovernmentEnum" not in fbs

    # Two arms of the same type stay distinct, because arms are named after
    # their proto field rather than their type.
    assert "Primary:TaxonomyType" in fbs
    assert "Secondary:TaxonomyType" in fbs


def test_flatbuffers_projects_bytes_as_a_string(tmp_path: Path) -> None:
    """A proto bytes field is text in this repository (xsd2proto emits it only
    for xs:hexBinary/xs:base64Binary, and the generated C++ type is
    std::string), so the FlatBuffers side must be a string too. Projected as
    [ubyte] it becomes a std::vector<uint8_t>, and the generated codec's
    assignment between the two no longer compiles."""
    out_dir = tmp_path / "out"
    _run_generator(_write_choice_proto(tmp_path), out_dir, "flatbuffers")
    fbs = _fbs_text(out_dir)

    assert "table Identifier {\n  uuid:string;\n}" in fbs
    assert "[ubyte]" not in fbs


def test_flatbuffers_choice_schema_compiles_with_flatc(tmp_path: Path) -> None:
    """The generated schema is accepted by the real flatc, which is the only
    authority on what a union may contain. Skips where flatc is absent."""
    flatc = _find_flatc()
    if flatc is None:
        pytest.skip("flatc not found; build FlatBuffers or set FLATC")
    out_dir = tmp_path / "out"
    _run_generator(_write_choice_proto(tmp_path), out_dir, "flatbuffers")
    schema_dir = out_dir / "flatbuffers" / "cpp"
    result = subprocess.run(
        [str(flatc), "--cpp", "-o", str(tmp_path / "hdr"),
         "-I", str(schema_dir), str(next(schema_dir.glob("*.fbs")))],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
