#!/usr/bin/env python3
"""ROS2 typed codec plugin generation tests."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
GENERATOR = PYRAMID_ROOT / "pim" / "generate_bindings.py"


def _run_generator(proto_dir: Path, out_dir: Path, backends: str) -> None:
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            str(proto_dir),
            str(out_dir),
            "--languages",
            "cpp",
            "--backends",
            backends,
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _manifest(out_dir: Path) -> dict:
    return json.loads(
        (out_dir / "binding_manifest.json").read_text(encoding="utf-8"))


def test_pyramid_ros2_typed_codec_plugin_is_emitted_and_ament_wired(
    tmp_path: Path,
) -> None:
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "json,ros2")

    plugin = (
        out_dir / "ros2" / "cpp" /
        "pyramid_services_tactical_objects_ros2_codec_plugin.cpp"
    )
    assert plugin.exists()
    text = plugin.read_text(encoding="utf-8")

    assert "Content-Type: application/ros2" in text
    assert '#include "pyramid_ros2_codec.hpp"' in text
    assert "namespace wire_codec = pyramid::ros2_codec;" in text
    assert '"application/ros2"' in text
    assert "wire_codec::toBinary(native)" in text
    assert "wire_codec::fromBinaryObjectDetail(" in text
    assert "pcl_codec_plugin_entry" in text

    codec = out_dir / "ros2" / "codec" / "pyramid_ros2_codec.hpp"
    codec_text = codec.read_text(encoding="utf-8")
    assert "fromBinaryObjectMatchArray(" in codec_text

    ros2_plugins = [
        plugin
        for plugin in _manifest(out_dir)["codec_plugins"]
        if plugin["content_type"] == "application/ros2"
    ]
    assert {
        plugin["source"]
        for plugin in ros2_plugins
    } == {
        "ros2/cpp/pyramid_services_autonomy_backend_ros2_codec_plugin.cpp",
        "ros2/cpp/pyramid_services_sensor_data_interpretation_ros2_codec_plugin.cpp",
        "ros2/cpp/pyramid_services_tactical_objects_ros2_codec_plugin.cpp",
    }

    ament_cmake = (PYRAMID_ROOT / "ros2" / "CMakeLists.txt").read_text(
        encoding="utf-8")
    assert "PYRAMID_ROS2_CODEC_PLUGIN_SRCS" in ament_cmake
    assert "pyramid_codec_ros2_" in ament_cmake
    assert "ament_target_dependencies(${PYRAMID_ROS2_CODEC_PLUGIN_TARGET}" in ament_cmake
    assert "pyramid_msgs" in ament_cmake

    top_cmake = (PYRAMID_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "pyramid_codec_ros2_" not in top_cmake
    assert "_ros2_codec_plugin\\\\.cpp$" in top_cmake


def test_pyramid_ros2_typed_codec_plugin_is_absent_without_ros2_backend(
    tmp_path: Path,
) -> None:
    proto_dir = PYRAMID_ROOT / "proto" / "pyramid"
    out_dir = tmp_path / "out"

    _run_generator(proto_dir, out_dir, "json")

    assert not list(out_dir.rglob("*_ros2_codec_plugin.cpp"))
    assert all(
        plugin["content_type"] != "application/ros2"
        for plugin in _manifest(out_dir)["codec_plugins"]
    )
