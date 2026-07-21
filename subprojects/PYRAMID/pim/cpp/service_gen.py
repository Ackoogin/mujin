#!/usr/bin/env python3
"""PCL-aligned C++ service binding generator (core driver).

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

import sys
from pathlib import Path
from typing import Dict, List, Tuple

from proto_parser import (
    parse_proto,
    parse_proto_tree,
    ProtoFile,
    ProtoRpc,
)
from proto_resolve import _DATA_MODEL_PROTO_ROOT
from binding_contract import (NamingPolicy, TopicSpecResolver)
from .naming import (
    _crud_rpcs,
    _DATA_MODEL_TYPES_HEADER,
    _DEFAULT_NAMING_POLICY,
    _namespace_from_proto,
    _service_group_key,
    _find_proto_root,
)
from .components_gen import ComponentsFacadeEmitterMixin
from .interaction_facade_gen import InteractionFacadeEmitterMixin
from .codec_plugin_gen import CodecPluginEmitterMixin
from .generic_gen import GenericServiceEmitterMixin
from .service_header_gen import ServiceHeaderEmitterMixin
from .service_impl_gen import ServiceImplEmitterMixin


class CppServiceGenerator(
        ServiceHeaderEmitterMixin,
        ServiceImplEmitterMixin,
        ComponentsFacadeEmitterMixin,
        InteractionFacadeEmitterMixin,
        CodecPluginEmitterMixin,
        GenericServiceEmitterMixin,
):
    _dm_proto_cache: Dict[Path, List[ProtoFile]] = {}

    def __init__(self, proto_input: str, enabled_backends=None,
                 naming_policy: NamingPolicy = None,
                 topic_resolver: TopicSpecResolver = None,
                 emit_native: bool = False):
        self._proto_input = Path(proto_input)
        self._enabled_backends = set(enabled_backends or [
            'json', 'flatbuffers', 'protobuf',
        ])
        self._naming = naming_policy or _DEFAULT_NAMING_POLICY
        self._topics = topic_resolver or TopicSpecResolver()
        # When True, emit the opt-in high-efficiency in-process (Tier-A native)
        # publish/subscribe fast path alongside the default serialized path.
        # Off by default so generated output is byte-for-byte identical to the
        # serialized-only baseline (the standing regression bar). See
        # doc/plans/PYRAMID/high_efficiency_process_bindings_plan.md.
        self._emit_native = emit_native

    def _has_backend(self, name: str) -> bool:
        return name in self._enabled_backends

    def _service_protobuf_codec_available(self, svc_base_ns: str) -> bool:
        """Return whether the protobuf backend will emit a service bridge."""
        del svc_base_ns
        return self._has_backend('protobuf') or self._has_backend('grpc')

    def _discover_data_model_proto_files(self) -> List[ProtoFile]:
        """Find parsed data-model protos for service JSON codec imports.

        Support both historical folder layouts like `proto/pyramid/data_model/*`
        and flattened namespace-style filenames under a shared `proto` root.
        """
        roots: List[Path] = []
        if self._proto_input.is_dir():
            roots.append(self._proto_input)
        else:
            proto_root = next(
                (parent for parent in [self._proto_input.parent, *self._proto_input.parents]
                 if parent.name.lower() == 'proto'),
                None,
            )
            if proto_root is not None:
                roots.append(proto_root)
            roots.extend([self._proto_input.parent, *self._proto_input.parents])

        seen: set[Path] = set()
        for root in roots:
            try:
                resolved = root.resolve()
            except OSError:
                resolved = root
            if resolved in seen or not root.exists() or not root.is_dir():
                continue
            seen.add(resolved)
            if resolved not in self._dm_proto_cache:
                try:
                    files = parse_proto_tree(root)
                except OSError:
                    files = []
                self._dm_proto_cache[resolved] = [
                    pf for pf in files
                    if pf.package == _DATA_MODEL_PROTO_ROOT
                    or pf.package.startswith(_DATA_MODEL_PROTO_ROOT + '.')
                ]
            dm_files = self._dm_proto_cache[resolved]
            if dm_files:
                return dm_files
        return []

    def _discover_all_proto_files(self) -> List[ProtoFile]:
        root = _find_proto_root(self._proto_input)
        if root is None or not root.exists() or not root.is_dir():
            return []
        try:
            return parse_proto_tree(root)
        except OSError:
            return []

    def _contract_service_files(self, package: str) -> Tuple[str, List[ProtoFile]]:
        base_package = _service_group_key(package)
        if not base_package:
            return '', []
        files = [
            pf for pf in self._discover_all_proto_files()
            if pf.services and _service_group_key(pf.package) == base_package
        ]
        return base_package, files

    def generate(self, output_dir: str):
        if self._naming.layout == 'generic':
            self._generate_generic(output_dir)
            return

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        proto_files: List[Path] = []
        if self._proto_input.is_dir():
            proto_files = list(self._proto_input.rglob('*.proto'))
        elif self._proto_input.is_file():
            proto_files = [self._proto_input]
        else:
            print(f'ERROR: {self._proto_input} is not a file or directory',
                  file=sys.stderr)
            sys.exit(1)

        for pf in proto_files:
            parsed = parse_proto(pf)
            all_rpcs: List[Tuple[str, ProtoRpc]] = []
            for svc in parsed.services:
                for rpc in _crud_rpcs(svc):
                    all_rpcs.append((svc.name, rpc))

            if not all_rpcs:
                continue

            full_ns, file_prefix, _svc_base_ns, types_ns = _namespace_from_proto(parsed)
            types_header = _DATA_MODEL_TYPES_HEADER

            hpp_path = output_path / (file_prefix + '.hpp')
            cpp_path = output_path / (file_prefix + '.cpp')

            self._write_header(hpp_path, full_ns, types_ns, types_header,
                               parsed, all_rpcs)
            self._write_impl(cpp_path, file_prefix, full_ns, types_ns,
                             parsed, all_rpcs)
            components_path = output_path / (file_prefix + '_components.hpp')
            self._write_components_header(
                components_path, file_prefix, full_ns, parsed, all_rpcs)
            self._write_codec_plugins(output_path, parsed)
            print(f'  Generated {full_ns}')
