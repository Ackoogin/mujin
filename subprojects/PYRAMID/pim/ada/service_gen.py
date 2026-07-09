#!/usr/bin/env python3
"""Ada service binding generator (core driver).

Split verbatim from ada_codegen.py (generator refactor plan, phase 4).
"""

import sys
from pathlib import Path
from typing import List, Tuple

from proto_parser import ProtoRpc, parse_proto
from binding_contract import TopicSpecResolver
from .naming import (
    _crud_rpcs,
    _pkg_name_from_proto,
    _collect_type_pkgs,
    _collect_codec_pkgs,
    _ensure_parent_packages,
)
from .service_body_gen import BodyEmitterMixin
from .service_spec_gen import SpecEmitterMixin
from .interaction_facade_gen import InteractionFacadeSpecMixin


class AdaServiceGenerator(InteractionFacadeSpecMixin, SpecEmitterMixin, BodyEmitterMixin):

    def __init__(self, proto_input: str, enabled_backends=None,
                 topic_resolver: TopicSpecResolver = None):
        self._proto_input = Path(proto_input)
        self._enabled_backends = set(enabled_backends or ['json'])
        self._topics = topic_resolver or TopicSpecResolver()

    def generate(self, output_dir: str):
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        proto_files: List[Path] = []
        if self._proto_input.is_dir():
            proto_files = list(self._proto_input.rglob('*.proto'))
        elif self._proto_input.is_file():
            proto_files = [self._proto_input]
        else:
            print(f'ERROR: {self._proto_input} is not a file or directory', file=sys.stderr)
            sys.exit(1)

        generated_pkgs: List[str] = []
        for pf in proto_files:
            parsed = parse_proto(pf)
            all_rpcs: List[Tuple[str, ProtoRpc]] = []
            for svc in parsed.services:
                for rpc in _crud_rpcs(svc):
                    all_rpcs.append((svc.name, rpc))

            if not all_rpcs:
                continue

            # Compute needed type/codec packages once for both spec and body.
            type_pkgs  = _collect_type_pkgs(pf, all_rpcs, self._topics)
            codec_pkgs = _collect_codec_pkgs(type_pkgs)

            pkg_name = _pkg_name_from_proto(parsed)
            ads_path = output_path / (pkg_name.lower().replace('.', '-') + '.ads')
            adb_path = output_path / (pkg_name.lower().replace('.', '-') + '.adb')

            self._write_spec(ads_path, pkg_name, parsed, all_rpcs, type_pkgs, pf)
            self._write_body(adb_path, pkg_name, parsed, all_rpcs,
                             type_pkgs, codec_pkgs, pf)
            generated_pkgs.append(pkg_name)
            print(f'  Generated {pkg_name}')

        _ensure_parent_packages(output_path, generated_pkgs)

