#!/usr/bin/env python3
"""Check that generated service facades only expose selected dependencies."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def _run_generator(generator: Path, proto_dir: Path, out_dir: Path,
                   language: str, backends: str) -> None:
    subprocess.run(
        [
            sys.executable,
            str(generator),
            str(proto_dir),
            str(out_dir),
            "--languages",
            language,
            "--backends",
            backends,
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )


def _read(files: list[Path]) -> str:
    return "\n".join(p.read_text(encoding="utf-8") for p in files)


def _cpp_service_files(out_dir: Path) -> list[Path]:
    return sorted(out_dir.glob("pyramid_services_*.hpp")) + sorted(
        out_dir.glob("pyramid_services_*.cpp")
    )


def _ada_service_files(out_dir: Path) -> list[Path]:
    return sorted(out_dir.glob("pyramid-services-*.ads")) + sorted(
        out_dir.glob("pyramid-services-*.adb")
    )


def _assert_absent(label: str, text: str, patterns: list[str]) -> None:
    for pattern in patterns:
        if re.search(pattern, text):
            raise AssertionError(f"{label}: unexpected generated dependency marker {pattern!r}")


def _assert_present(label: str, text: str, patterns: list[str]) -> None:
    for pattern in patterns:
        if not re.search(pattern, text):
            raise AssertionError(f"{label}: missing expected generated marker {pattern!r}")


def _case(generator: Path, proto_dir: Path, language: str,
          backends: str) -> Path:
    out_dir = Path(tempfile.mkdtemp(prefix=f"pyramid_{language}_{backends.replace(',', '_')}_"))
    _run_generator(generator, proto_dir, out_dir, language, backends)
    return out_dir


def check_cpp(generator: Path, proto_dir: Path) -> None:
    out_dir = _case(generator, proto_dir, "cpp", "json")
    text = _read(_cpp_service_files(out_dir))
    _assert_absent(
        "cpp json",
        text,
        [
            r"FlatBuffers",
            r"flatbuffers",
            r"Protobuf",
            r"protobuf",
            r"kFlatBuffersContentType",
            r"kProtobufContentType",
            r"__has_include",
            r"PYRAMID_HAVE_SERVICE",
        ],
    )
    if (out_dir / "grpc").exists() or (out_dir / "ros2").exists():
        raise AssertionError("cpp json: unselected transport directory was generated")
    shutil.rmtree(out_dir)

    out_dir = _case(generator, proto_dir, "cpp", "json,ros2")
    service_text = _read(_cpp_service_files(out_dir))
    _assert_absent(
        "cpp json,ros2 service facade",
        service_text,
        [
            r"FlatBuffers",
            r"flatbuffers",
            r"Protobuf",
            r"protobuf",
            r"kFlatBuffersContentType",
            r"kProtobufContentType",
            r"ServiceBinder",
        ],
    )
    _assert_present(
        "cpp json,ros2 service facade",
        service_text,
        [
            r"bindRos2",
            r"pyramid::transport::ros2::Adapter",
            r"bindUnaryServiceIngress",
        ],
    )
    ros2_files = sorted((out_dir / "ros2").rglob("*")) if (out_dir / "ros2").exists() else []
    if ros2_files:
        raise AssertionError("cpp json,ros2: parallel ROS2 transport files were generated")
    shutil.rmtree(out_dir)

    out_dir = _case(generator, proto_dir, "cpp", "json,flatbuffers")
    text = _read(_cpp_service_files(out_dir))
    _assert_present("cpp json,flatbuffers", text, [r"flatbuffers", r"kFlatBuffersContentType"])
    _assert_absent(
        "cpp json,flatbuffers",
        text,
        [r"Protobuf", r"protobuf", r"kProtobufContentType", r"__has_include", r"PYRAMID_HAVE_SERVICE"],
    )
    shutil.rmtree(out_dir)

    out_dir = _case(generator, proto_dir, "cpp", "json,protobuf")
    text = _read(_cpp_service_files(out_dir))
    _assert_present("cpp json,protobuf", text, [r"protobuf", r"kProtobufContentType"])
    _assert_absent(
        "cpp json,protobuf",
        text,
        [r"FlatBuffers", r"flatbuffers", r"kFlatBuffersContentType", r"__has_include", r"PYRAMID_HAVE_SERVICE"],
    )
    shutil.rmtree(out_dir)


def check_ada(generator: Path, proto_dir: Path) -> None:
    out_dir = _case(generator, proto_dir, "ada", "json")
    text = _read(_ada_service_files(out_dir))
    _assert_absent(
        "ada json",
        text,
        [r"Grpc_Content_Type", r"Configure_Grpc", r"GRPC_Transport"],
    )
    if (out_dir / "grpc").exists() or (out_dir / "ros2").exists():
        raise AssertionError("ada json: unselected transport directory was generated")
    shutil.rmtree(out_dir)

    out_dir = _case(generator, proto_dir, "ada", "json,ros2")
    service_text = _read(_ada_service_files(out_dir))
    _assert_absent(
        "ada json,ros2 service facade",
        service_text,
        [r"ROS2_Transport", r"Grpc_Content_Type", r"Configure_Grpc"],
    )
    _assert_present(
        "ada json,ros2 service facade",
        service_text,
        [
            r"Ros2_Transport_Content_Type",
            r"/pyramid/service/",
            r"/pyramid/stream/",
        ],
    )
    ros2_specs = sorted((out_dir / "ros2").rglob("*")) if (out_dir / "ros2").exists() else []
    if ros2_specs:
        raise AssertionError("ada json,ros2: parallel ROS2 transport specs were generated")
    shutil.rmtree(out_dir)

    out_dir = _case(generator, proto_dir, "ada", "json,grpc")
    service_text = _read(_ada_service_files(out_dir))
    _assert_present("ada json,grpc", service_text, [r"Grpc_Content_Type", r"Configure_Grpc"])
    _assert_absent(
        "ada json,grpc service facade",
        service_text,
        [r"function\s+Invoke_Create_Requirement\s*\n\s*\(\s*Channel"],
    )

    grpc_specs = sorted((out_dir / "grpc" / "ada").glob("*-grpc_transport.ads"))
    spec_text = _read(grpc_specs)
    _assert_absent(
        "ada grpc public transport specs",
        spec_text,
        [r"_Json", r"chars_ptr", r"Interfaces\.C\.Strings"],
    )
    shutil.rmtree(out_dir)


def check_facade_tests(pyramid_root: Path) -> None:
    ada_interop = pyramid_root / "tests" / "ada" / "ada_grpc_cpp_interop_e2e.adb"
    script = pyramid_root / "scripts" / "test_ada_grpc_cpp_interop_e2e.bat"

    ada_text = ada_interop.read_text(encoding="utf-8")
    _assert_present(
        "ada grpc interop facade test",
        ada_text,
        [
            r"Provided\.Configure_Grpc_Library",
            r"Provided\.Configure_Grpc_Channel",
            r"Provided\.Invoke_Create_Requirement",
            r"Content_Type\s*=>\s*Provided\.Grpc_Content_Type",
        ],
    )
    _assert_absent(
        "ada grpc interop facade test",
        ada_text,
        [
            r"_Json",
            r"grpc_provided_",
            r"GNATCOLL",
            r"\bLoad\b",
            r"\bLookup\b",
        ],
    )

    script_text = script.read_text(encoding="utf-8")
    _assert_present(
        "ada grpc interop driver",
        script_text,
        [r"generated service facade", r"--dll", r"--address"],
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--generator", required=True, type=Path)
    parser.add_argument("--proto-dir", required=True, type=Path)
    args = parser.parse_args()

    check_cpp(args.generator, args.proto_dir)
    check_ada(args.generator, args.proto_dir)
    check_facade_tests(args.generator.resolve().parents[1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
