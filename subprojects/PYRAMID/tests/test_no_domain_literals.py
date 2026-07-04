#!/usr/bin/env python3
"""Source guard: the base/common domain type-map literals are confined to the
PYRAMID compat policy, not duplicated across the language naming modules.

This locks in C3 (PYRAMID TODO): ``BASE_TYPE_MAP`` lives behind
``PyramidCompatNamingPolicy`` as the single source of truth, and the generic
(non-PYRAMID) code path carries no such compat domain literals.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PIM_DIR = REPO_ROOT / "subprojects" / "PYRAMID" / "pim"

sys.path.insert(0, str(PIM_DIR))

from binding_contract import (  # noqa: E402
    PyramidCompatNamingPolicy,
    GenericNamingPolicy,
)

# The map key literals only ever existed as base-type-map entries.
MAP_KEY_LITERALS = [
    "pyramid.data_model.base.Identifier",
    "pyramid.data_model.common.Ack",
]

# Modules that previously each hardcoded their own copy of the map.
CONSUMER_MODULES = [
    PIM_DIR / "cpp" / "naming.py",
    PIM_DIR / "ada" / "naming.py",
    PIM_DIR / "backends" / "grpc_backend.py",
]


def test_base_type_map_is_single_sourced_on_the_compat_policy() -> None:
    """The compat policy owns the canonical base/common short-name mapping."""
    policy_map = PyramidCompatNamingPolicy.base_type_map
    # Semantic anchor, not a rename: every value equals the type's last segment.
    assert policy_map
    for full_type, short in policy_map.items():
        assert short == full_type.split(".")[-1]
    # The generic layout has no compat domain mapping at all.
    assert GenericNamingPolicy.base_type_map == {}


def test_consumer_modules_do_not_redefine_the_domain_map() -> None:
    """cpp/ada/grpc naming modules must consume the map, not redefine it: the
    quoted domain key literals appear only in the compat policy."""
    for module in CONSUMER_MODULES:
        source = module.read_text(encoding="utf-8")
        for literal in MAP_KEY_LITERALS:
            for quote in ("'", '"'):
                assert f"{quote}{literal}{quote}" not in source, (
                    f"domain map literal {literal!r} leaked into "
                    f"{module.name}; it belongs to the compat policy"
                )


def test_compat_policy_still_holds_the_literals() -> None:
    """Guard against the map being deleted rather than relocated."""
    source = (PIM_DIR / "binding_contract.py").read_text(encoding="utf-8")
    for literal in MAP_KEY_LITERALS:
        assert literal in source
