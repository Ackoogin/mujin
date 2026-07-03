#!/usr/bin/env python3
"""Differential tests for the snake-casing helper variants.

The generator refactor (phase 2) merged the casing helpers that were
copy-pasted across cpp_codegen / ada_codegen into proto_parser: those were
textually identical, and the alias assertions below keep them that way.

standard_topics._camel_to_snake is deliberately NOT merged with
proto_parser.camel_to_lower_snake.  They diverge on acronym runs
('ABCDef' -> 'abcdef' vs 'abc_def') and underscored names
('State_Service' -> 'state__service' vs 'state_service').  That divergence
is harmless only because each variant sees disjoint inputs:

  * standard_topics._camel_to_snake processes only the legacy catalog
    payload type names, where the two variants agree (pinned below);
  * contract-derived BindingTopic payload names go through the regex
    variant, and many of them (HDIDependencyRequirement, ...) would change
    spelling under the catalog variant (also pinned below).
"""

from __future__ import annotations

import sys
from pathlib import Path

PYRAMID_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYRAMID_ROOT / 'pim'))

import proto_parser  # noqa: E402
from ada import naming as ada_naming  # noqa: E402
from cpp import json_codec_gen as cpp_json_codec_gen  # noqa: E402
from cpp import naming as cpp_naming  # noqa: E402
from cpp import service_header_gen as cpp_header_gen  # noqa: E402
import standard_topics  # noqa: E402
from binding_contract import topics_for_proto_file  # noqa: E402
from proto_parser import parse_proto_tree  # noqa: E402

PROTO_TREES = [PYRAMID_ROOT / 'proto', PYRAMID_ROOT / 'pim' / 'test']


def _catalog_payload_names() -> set[str]:
    """Payload type names the legacy catalog variant actually processes."""
    meta = standard_topics.default_metadata()
    return {spec.short_type for spec in meta.specs.values()}


def _contract_payload_names() -> set[str]:
    """Payload type names of contract-derived topics from both trees."""
    names: set[str] = set()
    for root in PROTO_TREES:
        for pf in parse_proto_tree(root):
            sub_specs, pub_specs = topics_for_proto_file(pf)
            for specs in (sub_specs, pub_specs):
                names.update(spec.short_type for spec in specs.values())
    return names


def test_merged_helpers_are_the_proto_parser_ones():
    # The codegen-local names must be aliases, not copies.
    assert cpp_naming._camel_to_lower_snake is proto_parser.camel_to_lower_snake
    assert cpp_header_gen._snake_to_pascal is proto_parser.snake_to_pascal
    assert cpp_json_codec_gen._lc_first is proto_parser.lc_first
    assert ada_naming._camel_to_snake is proto_parser.camel_to_snake
    assert ada_naming._camel_to_lower_snake is proto_parser.camel_to_lower_snake


def test_variants_agree_on_catalog_payload_names():
    """On its real input domain the catalog variant matches the regex one."""
    names = _catalog_payload_names()
    assert names, 'legacy catalog unexpectedly empty'
    divergent = [
        n for n in sorted(names)
        if standard_topics._camel_to_snake(n)
        != proto_parser.camel_to_lower_snake(n)
    ]
    assert divergent == []


def test_variants_diverge_on_contract_payload_names():
    """Documents WHY the two variants must stay separate: merging the
    catalog variant into the regex one (or vice versa) would respell
    contract-derived payload identifiers."""
    names = _contract_payload_names()
    assert len(names) >= 20, 'contract topic corpus unexpectedly small'
    divergent = [
        n for n in sorted(names)
        if standard_topics._camel_to_snake(n)
        != proto_parser.camel_to_lower_snake(n)
    ]
    assert divergent, 'variants now agree everywhere; reconsider merging'
    # Spot-pin the shape of the divergence.
    assert standard_topics._camel_to_snake('ABCDef') == 'abcdef'
    assert proto_parser.camel_to_lower_snake('ABCDef') == 'abc_def'
