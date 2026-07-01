#!/usr/bin/env python3
"""Topic generation is explicit-metadata driven, not domain-hardcoded."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PYRAMID_ROOT = REPO_ROOT / "subprojects" / "PYRAMID"
PIM_DIR = PYRAMID_ROOT / "pim"
GENERATOR = PIM_DIR / "generate_bindings.py"

sys.path.insert(0, str(PIM_DIR))

import standard_topics as st  # noqa: E402


def test_default_metadata_reproduces_tactical_topics() -> None:
    """The pyramid_compat default reproduces the exact prior topic surface."""
    subscribe, publish = st.topics_for_service(
        "pyramid.components.tactical_objects.services.provided",
        is_provided=True,
    )
    assert subscribe == {
        "entity_matches": "standard.entity_matches",
        "evidence_requirements": "standard.evidence_requirements",
    }
    assert publish == {}

    subscribe_c, publish_c = st.topics_for_service(
        "pyramid.components.tactical_objects.services.consumed",
        is_provided=False,
    )
    assert subscribe_c == {}
    assert publish_c == {"object_evidence": "standard.object_evidence"}

    spec = st.topic_spec("entity_matches")
    assert spec.full_type == "pyramid.data_model.tactical.ObjectMatch"
    assert spec.is_array is True
    assert spec.cpp_payload_type == "std::vector<ObjectMatch>"


def test_empty_metadata_yields_no_topics_even_for_tactical_package() -> None:
    """With empty/generic metadata, no package -- including a tactical one --
    produces topics. Selection is metadata-driven, not domain-branched."""
    for package in (
        "pyramid.components.tactical_objects.services.provided",
        "example.telemetry",
        "vendor.api.telemetry.v1",
    ):
        assert st.topics_for_service(
            package, True, metadata=st.EMPTY_METADATA
        ) == ({}, {})
        assert st.topics_for_service(
            package, False, metadata=st.EMPTY_METADATA
        ) == ({}, {})


def test_standard_topics_logic_has_no_hardcoded_domain_strings() -> None:
    """The reusable module logic must not literally branch on domain names.

    The Tactical Objects topic table lives in the JSON data file, not in the
    module's Python logic.
    """
    source = (PIM_DIR / "standard_topics.py").read_text(encoding="utf-8")
    # The domain topic data (wire names, payload types) must not be baked into
    # the module logic -- it lives in the JSON metadata file.
    assert "pyramid.data_model.tactical" not in source
    assert "standard.entity_matches" not in source
    # The only permissible mention of a domain name is the compat metadata
    # file path constant.
    for line in source.splitlines():
        if "tactical_objects" in line:
            assert ".json" in line, f"domain string leaked into logic: {line!r}"

    # The tactical set is instead sourced from the JSON metadata file.
    meta = st.load_topic_metadata(
        PIM_DIR / "topic_metadata" / "tactical_objects_topics.json"
    )
    subscribe, _ = meta.topics_for_service(
        "pyramid.components.tactical_objects.services.provided", True)
    assert subscribe.get("entity_matches") == "standard.entity_matches"


def test_pyramid_layout_still_emits_standard_topic_wire_names(
    tmp_path: Path,
) -> None:
    """End-to-end: pyramid layout still bakes the standard topic wire names
    into the tactical objects provided service file."""
    out_dir = tmp_path / "out"
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            str(PYRAMID_ROOT / "proto" / "pyramid"),
            str(out_dir),
            "--languages",
            "cpp",
            "--backends",
            "json",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    provided = (
        out_dir / "pyramid_services_tactical_objects_provided.hpp"
    ).read_text(encoding="utf-8")
    assert "standard.entity_matches" in provided
    assert "standard.evidence_requirements" in provided


def test_generic_layout_service_emits_no_topics(tmp_path: Path) -> None:
    """A generic service package produces no standard-topic helpers."""
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
            "--contract-layout",
            "generic",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    all_text = "\n".join(
        p.read_text(encoding="utf-8")
        for p in out_dir.iterdir()
        if p.is_file()
    )
    assert "standard." not in all_text
    assert not re.search(r"\btopic\b", all_text, re.IGNORECASE) or \
        "standard." not in all_text
