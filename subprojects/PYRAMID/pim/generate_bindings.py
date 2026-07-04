#!/usr/bin/env python3
"""
Unified Codec Binding Generator

Parses .proto files and generates all codec/transport backends (JSON,
FlatBuffers, Protobuf, gRPC, ROS2) for all target languages (C++, Ada) in a
single invocation.

No hardcoded knowledge of specific data models -- everything is derived from
the proto IDL as the single source of truth.

Usage:
    # Generate all backends for all languages
    python generate_bindings.py proto/pyramid/ output/

    # Generate specific backends only
    python generate_bindings.py proto/pyramid/ output/ --backends json,flatbuffers

    # Generate for specific languages only
    python generate_bindings.py proto/pyramid/ output/ --languages cpp

    # List available backends
    python generate_bindings.py --list-backends
"""

import argparse
import json
import re
import sys
from pathlib import Path

# Ensure pim/ is on sys.path so imports resolve
sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import (
    parse_proto, parse_proto_tree, ProtoTypeIndex, ProtoMessage,
)
from binding_contract import TopicSpecResolver, build_contract
import codec_backends
import cabi_codegen
import ada_cabi_codegen
from ada import codec_gen as ada_codec_gen
from ada import generic_service_gen as ada_generic_service_gen
from ada import service_gen as ada_service_gen
from ada import types_gen as ada_types_gen
from cpp import json_codec_gen as cpp_json_codec_gen
from cpp import service_gen as cpp_service_gen
from cpp import types_gen as cpp_types_gen

# Importing the backends package auto-registers all backends
import backends  # noqa: F401


_MANIFEST_NAME = 'binding_manifest.json'
_BINDING_EXCLUDED_PACKAGES = {
    'pyramid.options',
}


class BindingArtifactManifest:
    """Accumulates generated binding artifact paths by build role."""

    def __init__(self, layout: str, proto_dir: Path, output_dir: Path,
                 proto_files):
        self._output_dir = output_dir
        self._proto_import_root = _infer_proto_import_root(
            proto_dir, proto_files)
        self._data = {
            'layout': layout,
            'proto_import_roots': [
                _display_path(self._proto_import_root),
            ],
            'proto_files': [
                _relative_proto_path(pf.path, self._proto_import_root)
                for pf in proto_files
            ],
            'types': [],
            'json_codecs': [],
            'cabi': [],
            'services': [],
            'topics': [],
            'endpoint_requirements': [],
            'codec_plugins': [],
            'flatbuffers_schemas': [],
            'protobuf_service_codecs': [],
            'protobuf_protos': [],
            'grpc_service_protos': [],
            # Build-source roles: populated by a post-generation scan
            # (populate_build_source_roles) so manifest mode selects exactly the
            # sources the CMake glob sites do. Kept distinct from the semantic
            # roles above so each maps 1:1 to a source-selection site.
            'service_json_codecs': [],
            'service_marshal': [],
            'flatbuffers_service_codecs': [],
            'grpc_transport': [],
            'grpc_plugin_aggregator': [],
            'ros2_transport': [],
            'marshal_modules': [],
        }
        self._seen = {
            key: set()
            for key, value in self._data.items()
            if isinstance(value, list)
        }
        self._seen['codec_plugins'] = set()
        for key in ('proto_import_roots', 'proto_files'):
            self._seen[key].update(self._data[key])

    def add_paths(self, role: str, paths) -> None:
        for path in paths:
            rel = self._relative_output_path(path)
            if rel == _MANIFEST_NAME or rel in self._seen[role]:
                continue
            self._data[role].append(rel)
            self._seen[role].add(rel)

    def add_codec_plugins(self, paths) -> None:
        for path in paths:
            rel = self._relative_output_path(path)
            if rel == _MANIFEST_NAME or rel in self._seen['codec_plugins']:
                continue
            self._data['codec_plugins'].append({
                'target': Path(rel).stem,
                'source': rel,
                'content_type': _codec_plugin_content_type(Path(rel).name),
            })
            self._seen['codec_plugins'].add(rel)

    def add_selected_backend_protos(self, backend_names, proto_files) -> None:
        selected = set(backend_names)
        rel_proto_files = [
            _relative_proto_path(pf.path, self._proto_import_root)
            for pf in proto_files
        ]
        if 'protobuf' in selected:
            type_protos = [
                rel
                for rel, pf in zip(rel_proto_files, proto_files)
                if pf.messages or pf.enums
            ]
            self._add_values('protobuf_protos', type_protos)
        if 'grpc' in selected:
            service_protos = [
                rel
                for rel, pf in zip(rel_proto_files, proto_files)
                if pf.services
            ]
            self._add_values('grpc_service_protos', service_protos)

    def add_contract_topics(self, contract) -> None:
        entries = []
        for service_key, topics in sorted(contract.service_topics.items()):
            for topic in topics:
                entries.append({
                    'service': service_key,
                    'rpc': topic.rpc_name,
                    'direction': topic.direction,
                    'pattern': topic.pattern,
                    'key': topic.key,
                    'wire_name': topic.wire_name,
                    'payload_type': topic.full_type,
                    'qos': topic.qos,
                })
        self._data['topics'] = entries
        self._seen['topics'] = {
            f"{entry['service']}:{entry['rpc']}:{entry['direction']}:{entry['wire_name']}"
            for entry in entries
        }

    def add_endpoint_requirements(self, contract) -> None:
        entries = []
        for req in contract.endpoint_requirements:
            entry = {
                'endpoint_name': req.endpoint_name,
                'kind': req.kind,
                'capability': req.capability,
                'reliability': req.reliability,
                'source': req.source,
                'qos': req.qos,
            }
            if req.service_name:
                entry['service_name'] = req.service_name
            if req.rpc_name:
                entry['rpc_name'] = req.rpc_name
            if req.topic_key:
                entry['topic_key'] = req.topic_key
            if req.topic_wire_name:
                entry['topic_wire_name'] = req.topic_wire_name
            entries.append(entry)
        self._data['endpoint_requirements'] = entries
        self._seen['endpoint_requirements'] = {
            f"{entry['source']}:{entry['endpoint_name']}:{entry['kind']}"
            for entry in entries
        }

    def record_generated(self, role: str, callback) -> None:
        before = _output_file_snapshot(self._output_dir)
        callback()
        after = _output_file_snapshot(self._output_dir)
        self.add_paths(role, _changed_output_files(before, after))

    def record_service_generation(self, callback) -> None:
        before = _output_file_snapshot(self._output_dir)
        callback()
        after = _output_file_snapshot(self._output_dir)
        service_paths = []
        plugin_paths = []
        for path in _changed_output_files(before, after):
            if path.name.endswith('_codec_plugin.cpp'):
                plugin_paths.append(path)
            else:
                service_paths.append(path)
        self.add_paths('services', service_paths)
        self.add_codec_plugins(plugin_paths)

    def record_backend_outputs(self, backend_name: str, paths) -> None:
        if backend_name == 'flatbuffers':
            self.add_paths(
                'flatbuffers_schemas',
                [path for path in paths if Path(path).suffix == '.fbs'],
            )
        if backend_name == 'protobuf':
            self.add_paths(
                'protobuf_service_codecs',
                [
                    path for path in paths
                    if '_services_' in Path(path).name
                    and (
                        Path(path).name.endswith('_protobuf_codec.cpp')
                        or Path(path).name.endswith('_protobuf_codec.hpp')
                    )
                ],
            )

    def write(self) -> None:
        self._output_dir.mkdir(parents=True, exist_ok=True)
        path = self._output_dir / _MANIFEST_NAME
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(self._data, f, indent=2)
            f.write('\n')

    @property
    def proto_import_root(self) -> Path:
        return self._proto_import_root

    def _add_values(self, role: str, values) -> None:
        for value in values:
            if value in self._seen[role]:
                continue
            self._data[role].append(value)
            self._seen[role].add(value)

    def _relative_output_path(self, path) -> str:
        path = Path(path)
        try:
            rel = path.relative_to(self._output_dir)
        except ValueError:
            rel = path
        return rel.as_posix()

    def _scan_generated(self, subdir: str, name_pattern: str):
        """Relative posix paths of files under output_dir/subdir matching
        name_pattern, searched recursively -- mirroring the CMake
        `file(GLOB_RECURSE ${DIR}/subdir/name_pattern)` the build sites use."""
        base = self._output_dir / subdir if subdir else self._output_dir
        if not base.is_dir():
            return []
        return sorted(
            self._relative_output_path(p)
            for p in base.rglob(name_pattern)
            if p.is_file())

    def populate_build_source_roles(self) -> None:
        """Populate the build-source roles by scanning the emitted tree with the
        exact patterns the CMake source-selection sites glob, so `manifest`
        mode selects the same sources as `glob` mode. Called once after all
        generation, before write()."""
        self._data['service_json_codecs'] = self._scan_generated(
            '', 'pyramid_components_*_codec.cpp')
        self._data['service_marshal'] = self._scan_generated(
            '', 'pyramid_components_*_cabi_marshal.cpp')
        self._data['flatbuffers_service_codecs'] = self._scan_generated(
            'flatbuffers/cpp', 'pyramid_services_*_flatbuffers_codec.cpp')
        self._data['grpc_transport'] = self._scan_generated(
            'grpc/cpp', '*_grpc_transport.cpp')
        self._data['grpc_plugin_aggregator'] = self._scan_generated(
            'grpc/cpp', 'pyramid_grpc_plugin_aggregator.cpp')
        self._data['ros2_transport'] = [
            rel for rel in self._scan_generated('ros2/cpp', '*.cpp')
            if not rel.endswith('_ros2_codec_plugin.cpp')]
        # Per-module marshal identity for the churn-isolation loop, so CMake does
        # not re-parse the module name out of the filename.
        modules = []
        for rel in self._scan_generated(
                '', 'pyramid_data_model_*_cabi_marshal.cpp'):
            match = re.match(
                r'^pyramid_data_model_(.+)_cabi_marshal\.cpp$', Path(rel).name)
            if match:
                modules.append({'module': match.group(1), 'source': rel})
        self._data['marshal_modules'] = modules


def _display_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _infer_proto_import_root(proto_dir: Path, proto_files) -> Path:
    packages = [pf.package for pf in proto_files if pf.package]
    first_segments = {pkg.split('.')[0] for pkg in packages}
    if len(first_segments) == 1 and proto_dir.name == next(iter(first_segments)):
        return proto_dir.parent
    return proto_dir


def _relative_proto_path(path: Path, proto_import_root: Path) -> str:
    try:
        return path.relative_to(proto_import_root).as_posix()
    except ValueError:
        return path.as_posix()


def _output_file_snapshot(output_dir: Path):
    if not output_dir.exists():
        return {}
    return {
        path: (path.stat().st_mtime_ns, path.stat().st_size)
        for path in output_dir.rglob('*')
        if path.is_file() and path.name != _MANIFEST_NAME
    }


def _changed_output_files(before, after):
    return sorted(
        path for path, stat in after.items()
        if before.get(path) != stat
    )


def _codec_plugin_content_type(filename: str) -> str:
    if '_flatbuffers_' in filename:
        return 'application/flatbuffers'
    if '_protobuf_' in filename:
        return 'application/protobuf'
    if '_ros2_' in filename:
        return 'application/ros2'
    return 'application/json'


def _discover_data_model_files(proto_dir: Path):
    return [
        pf for pf in parse_proto_tree(proto_dir)
        if _is_binding_proto(pf) and pf.package.startswith('pyramid.data_model')
    ]


def _discover_service_message_files(proto_dir: Path):
    """Service protos that declare local wrapper messages (oneof request/
    response payloads). These messages are canonically homed in their own
    component namespace (Component-NS) and need the same struct/codec/cabi
    treatment as data-model types. A synthetic ``Empty`` is appended where an
    RPC uses google.protobuf.Empty so the generated facade resolves it."""
    files = []
    for pf in parse_proto_tree(proto_dir):
        if not _is_binding_proto(pf):
            continue
        if '.services.' not in f'.{pf.package}.':
            continue
        uses_empty = any(
            'google.protobuf.Empty' in (rpc.request_type, rpc.response_type)
            for svc in pf.services for rpc in svc.rpcs
        )
        if uses_empty and not any(m.name == 'Empty' for m in pf.messages):
            pf.messages.append(ProtoMessage(name='Empty'))
        # A service that only references google.protobuf.Empty (no local oneof
        # wrappers) still needs the synthesized Empty homed in its namespace.
        if not pf.messages:
            continue
        files.append(pf)
    return files


def _is_binding_proto(pf) -> bool:
    """True for application contract protos that should produce bindings.

    Compiler-extension protos such as pyramid.options are parsed so method
    options can be captured, but they are not part of the generated SDK surface.
    """
    return pf.package not in _BINDING_EXCLUDED_PACKAGES


def _generate_json_cpp(proto_dir: Path, output_dir: Path,
                       enabled_backends=None, contract_layout: str = 'pyramid',
                       proto_files=None,
                       manifest: BindingArtifactManifest = None,
                       contract=None) -> int:
    total = 0
    # One topic-spec resolver per generation run, shared by every service
    # generator in the run (mirrors the pre-refactor module-global cache).
    topic_resolver = TopicSpecResolver()
    if contract_layout == 'generic':
        parsed_files = list(proto_files) if proto_files is not None else parse_proto_tree(proto_dir)
        contract = contract or build_contract(parsed_files, layout='generic')
        policy = contract.naming_policy
        type_files = contract.type_modules
        if type_files:
            gen = cpp_types_gen.CppTypesGenerator(
                type_files, naming_policy=policy)
            if manifest:
                manifest.record_generated(
                    'types', lambda: gen.generate(str(output_dir)))
            else:
                gen.generate(str(output_dir))
            total += 1
            type_index = ProtoTypeIndex(type_files)
            for pf in type_files:
                gen = cpp_json_codec_gen.CppDataModelCodecGenerator(
                    pf, type_index, naming_policy=policy)
                if manifest:
                    manifest.record_generated(
                        'json_codecs',
                        lambda gen=gen: gen.generate(str(output_dir)),
                    )
                else:
                    gen.generate(str(output_dir))
                total += 1
        for pf in contract.service_modules:
            gen = cpp_service_gen.CppServiceGenerator(
                str(pf.path),
                enabled_backends=enabled_backends,
                naming_policy=policy,
                topic_resolver=topic_resolver,
            )
            if manifest:
                manifest.record_service_generation(
                    lambda gen=gen: gen.generate(str(output_dir)))
            else:
                gen.generate(str(output_dir))
            total += 1
        return total

    dm_files = _discover_data_model_files(proto_dir)
    if dm_files:
        gen = cpp_types_gen.CppTypesGenerator(dm_files)
        if manifest:
            manifest.record_generated(
                'types', lambda: gen.generate(str(output_dir)))
        else:
            gen.generate(str(output_dir))
        total += 1
        dm_index = ProtoTypeIndex(dm_files)
        for pf in dm_files:
            gen = cpp_json_codec_gen.CppDataModelCodecGenerator(pf, dm_index)
            if manifest:
                manifest.record_generated(
                    'json_codecs',
                    lambda gen=gen: gen.generate(str(output_dir)),
                )
            else:
                gen.generate(str(output_dir))
            total += 1
        gen = cabi_codegen.CabiTypesGenerator(dm_files)
        if manifest:
            manifest.record_generated(
                'cabi', lambda: gen.generate(str(output_dir)))
        else:
            gen.generate(str(output_dir))
        gen = cabi_codegen.CabiMarshalGenerator(dm_files)
        if manifest:
            manifest.record_generated(
                'cabi', lambda: gen.generate(str(output_dir)))
        else:
            gen.generate(str(output_dir))
        total += 2

        # Service-local wrapper messages (Component-NS): emit struct + JSON
        # codec + C-ABI types/marshal in each component's own namespace, using a
        # combined index so wrapper field types (data-model payloads) resolve.
        svc_files = _discover_service_message_files(proto_dir)
        if svc_files:
            combined = dm_files + svc_files
            combined_index = ProtoTypeIndex(combined)
            types_gen = cpp_types_gen.CppTypesGenerator(combined)
            cabi_types_gen = cabi_codegen.CabiTypesGenerator(combined)
            cabi_marshal_gen = cabi_codegen.CabiMarshalGenerator(combined)
            for spf in svc_files:
                if manifest:
                    manifest.record_generated(
                        'types',
                        lambda spf=spf: types_gen.write_file(
                            spf, str(output_dir)),
                    )
                else:
                    types_gen.write_file(spf, str(output_dir))
                gen = cpp_json_codec_gen.CppDataModelCodecGenerator(
                    spf, combined_index)
                if manifest:
                    manifest.record_generated(
                        'json_codecs',
                        lambda gen=gen: gen.generate(str(output_dir)),
                    )
                else:
                    gen.generate(str(output_dir))
                if manifest:
                    manifest.record_generated(
                        'cabi',
                        lambda spf=spf: cabi_types_gen.write_file(
                            spf, str(output_dir)),
                    )
                    manifest.record_generated(
                        'cabi',
                        lambda spf=spf: cabi_marshal_gen.write_file(
                            spf, str(output_dir)),
                    )
                else:
                    cabi_types_gen.write_file(spf, str(output_dir))
                    cabi_marshal_gen.write_file(spf, str(output_dir))
                total += 4

    for proto_path in sorted(proto_dir.rglob('*.proto')):
        parsed = parse_proto(proto_path)
        if not parsed.services or '.services.' not in f'.{parsed.package}.':
            continue
        gen = cpp_service_gen.CppServiceGenerator(
            str(proto_path),
            enabled_backends=enabled_backends,
            topic_resolver=topic_resolver,
        )
        if manifest:
            manifest.record_service_generation(
                lambda gen=gen: gen.generate(str(output_dir)))
        else:
            gen.generate(str(output_dir))
        total += 1
    return total


def _generate_json_ada(proto_dir: Path, output_dir: Path,
                       enabled_backends=None, contract_layout: str = 'pyramid',
                       proto_files=None, contract=None) -> int:
    total = 0
    # One topic-spec resolver per generation run (see _generate_json_cpp).
    topic_resolver = TopicSpecResolver()
    if contract_layout == 'generic':
        parsed_files = (list(proto_files) if proto_files is not None
                        else parse_proto_tree(proto_dir))
        contract = contract or build_contract(parsed_files, layout='generic')
        type_files = contract.type_modules
        if type_files:
            # Ada type/codec package names are already derived generically from
            # the proto package (example.telemetry -> Example.Telemetry.Types),
            # so the existing generators work for arbitrary packages.
            ada_types_gen.AdaTypesGenerator(type_files).generate(str(output_dir))
            total += 1
            type_index = ProtoTypeIndex(type_files)
            for pf in type_files:
                ada_codec_gen.AdaDataModelCodecGenerator(
                    pf, type_index).generate(str(output_dir))
                total += 1
        for pf in contract.service_modules:
            ada_generic_service_gen.AdaGenericServiceGenerator(
                pf, enabled_backends=enabled_backends,
            ).generate(str(output_dir))
            total += 1
        return total

    dm_files = _discover_data_model_files(proto_dir)
    if dm_files:
        gen = ada_types_gen.AdaTypesGenerator(dm_files)
        gen.generate(str(output_dir))
        total += 1
        ada_cabi_codegen.AdaCabiGenerator(dm_files).generate(str(output_dir))
        total += 1
        dm_index = ProtoTypeIndex(dm_files)
        for pf in dm_files:
            ada_codec_gen.AdaDataModelCodecGenerator(pf, dm_index).generate(str(output_dir))
            total += 1

        # Service-local wrapper messages (Component-NS): emit native types +
        # JSON codec + C-ABI mirror/marshal in each component's own namespace,
        # using a combined index so wrapper field types (data-model payloads)
        # resolve. Mirrors the C++ Defect-B routing in _generate_json_cpp.
        svc_files = _discover_service_message_files(proto_dir)
        if svc_files:
            combined = dm_files + svc_files
            combined_index = ProtoTypeIndex(combined)
            types_gen = ada_types_gen.AdaTypesGenerator(combined)
            cabi_gen = ada_cabi_codegen.AdaCabiGenerator(combined)
            for spf in svc_files:
                types_gen.write_file(spf, str(output_dir))
                ada_codec_gen.AdaDataModelCodecGenerator(
                    spf, combined_index).generate(str(output_dir))
                cabi_gen.write_file(spf, str(output_dir))
                total += 3

    for proto_path in sorted(proto_dir.rglob('*.proto')):
        parsed = parse_proto(proto_path)
        if not parsed.services or '.services.' not in f'.{parsed.package}.':
            continue
        ada_service_gen.AdaServiceGenerator(
            str(proto_path),
            enabled_backends=enabled_backends,
            topic_resolver=topic_resolver,
        ).generate(str(output_dir))
        total += 1
    return total


def main():
    parser = argparse.ArgumentParser(
        description='Generate codec bindings from proto IDL files.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        'proto_dir', nargs='?',
        help='Root directory containing .proto files',
    )
    parser.add_argument(
        'output_dir', nargs='?',
        help='Output directory for generated files',
    )
    parser.add_argument(
        '--backends', '-b',
        help='Comma-separated list of backends (default: all). '
             'Available: json, flatbuffers, protobuf, grpc, ros2',
    )
    parser.add_argument(
        '--languages', '-l',
        help='Comma-separated list of languages (default: all). '
             'Available: cpp, ada',
    )
    parser.add_argument(
        '--list-backends', action='store_true',
        help='List all registered backends and exit',
    )
    parser.add_argument(
        '--contract-layout',
        choices=('pyramid', 'generic'),
        default='pyramid',
        help='Binding contract layout (default: pyramid)',
    )

    args = parser.parse_args()

    if args.list_backends:
        print('Registered codec backends:')
        for name, backend in codec_backends.all_backends().items():
            print(f'  {name:<15} content-type: {backend.content_type}')
        return

    if not args.proto_dir or not args.output_dir:
        parser.error('proto_dir and output_dir are required')

    proto_dir = Path(args.proto_dir)
    output_dir = Path(args.output_dir)

    if not proto_dir.exists():
        print(f'ERROR: {proto_dir} does not exist', file=sys.stderr)
        sys.exit(1)

    # Parse all proto files
    print(f'Parsing proto files from {proto_dir}...')
    parsed_proto_files = parse_proto_tree(proto_dir)
    proto_files = [pf for pf in parsed_proto_files if _is_binding_proto(pf)]
    index = ProtoTypeIndex(proto_files)
    contract = build_contract(proto_files, layout=args.contract_layout)

    n_msgs = len(index.all_messages())
    n_enums = len(index.all_enums())
    n_svcs = sum(len(pf.services) for pf in proto_files)
    print(f'  Found {len(proto_files)} proto files: '
          f'{n_msgs} messages, {n_enums} enums, {n_svcs} services')

    # Determine backends and languages
    backend_names = (args.backends.split(',') if args.backends
                     else list(codec_backends.all_backends().keys()))
    langs = set(args.languages.split(',') if args.languages else ['cpp', 'ada'])

    # Generate
    print(f'\nGenerating backends: {", ".join(backend_names)}')
    print(f'Languages: {", ".join(sorted(langs))}')
    print()

    manifest = BindingArtifactManifest(
        args.contract_layout, proto_dir, output_dir, proto_files)
    manifest.add_selected_backend_protos(backend_names, proto_files)
    manifest.add_contract_topics(contract)
    manifest.add_endpoint_requirements(contract)

    total = 0
    remaining_backends = list(backend_names)
    if 'json' in remaining_backends:
        print('  json: generating unified data-model and service bindings')
        if 'cpp' in langs:
            total += _generate_json_cpp(
                proto_dir,
                output_dir,
                enabled_backends=backend_names,
                contract_layout=args.contract_layout,
                proto_files=proto_files,
                manifest=manifest,
                contract=contract,
            )
        if 'ada' in langs:
            total += _generate_json_ada(
                proto_dir,
                output_dir,
                enabled_backends=backend_names,
                contract_layout=args.contract_layout,
                proto_files=proto_files,
                contract=contract,
            )
        remaining_backends.remove('json')

    if remaining_backends:
        results = codec_backends.generate_all(
            index, output_dir,
            languages=list(langs),
            backends=remaining_backends,
            naming_policy=contract.naming_policy,
            proto_import_root=manifest.proto_import_root,
            contract=contract,
        )
        for backend_name, files in results.items():
            print(f'  {backend_name}: {len(files)} files generated')
            for fp in files:
                print(f'    {fp}')
            manifest.record_backend_outputs(backend_name, files)
            total += len(files)

    manifest.populate_build_source_roles()
    manifest.write()
    print(f'\nDone -- {total} files generated in {output_dir}/')


if __name__ == '__main__':
    main()
