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
import sys
from pathlib import Path

# Ensure pim/ is on sys.path so imports resolve
sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import parse_proto_tree, ProtoTypeIndex
import codec_backends
import cpp_codegen
import ada_codegen

# Importing the backends package auto-registers all backends
import backends  # noqa: F401


def _generate_json_cpp(proto_dir: Path, output_dir: Path,
                       enabled_backends=None) -> int:
    total = 0
    data_model_dir = proto_dir / 'pyramid' / 'data_model'
    if data_model_dir.exists():
        gen = cpp_codegen.CppTypesGenerator(data_model_dir)
        gen.generate(str(output_dir))
        total += 1
        dm_files = parse_proto_tree(data_model_dir)
        dm_index = ProtoTypeIndex(dm_files)
        for pf in dm_files:
            cpp_codegen.CppDataModelCodecGenerator(pf, dm_index).generate(str(output_dir))
            total += 1

    for proto_path in sorted(proto_dir.rglob('*.proto')):
        parsed = cpp_codegen.parse_proto(proto_path)
        if not parsed.services or '.services.' not in f'.{parsed.package}.':
            continue
        cpp_codegen.CppServiceGenerator(
            str(proto_path),
            enabled_backends=enabled_backends,
        ).generate(str(output_dir))
        total += 1
    return total


def _generate_json_ada(proto_dir: Path, output_dir: Path,
                       enabled_backends=None) -> int:
    total = 0
    data_model_dir = proto_dir / 'pyramid' / 'data_model'
    if data_model_dir.exists():
        gen = ada_codegen.AdaTypesGenerator(data_model_dir)
        gen.generate(str(output_dir))
        total += 1
        dm_files = parse_proto_tree(data_model_dir)
        dm_index = ProtoTypeIndex(dm_files)
        for pf in dm_files:
            ada_codegen.AdaDataModelCodecGenerator(pf, dm_index).generate(str(output_dir))
            total += 1

    for proto_path in sorted(proto_dir.rglob('*.proto')):
        parsed = ada_codegen.parse_proto(proto_path)
        if not parsed.services or '.services.' not in f'.{parsed.package}.':
            continue
        ada_codegen.AdaServiceGenerator(
            str(proto_path),
            enabled_backends=enabled_backends,
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
    proto_files = parse_proto_tree(proto_dir)
    index = ProtoTypeIndex(proto_files)

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

    total = 0
    remaining_backends = list(backend_names)
    if 'json' in remaining_backends:
        print('  json: generating unified data-model and service bindings')
        if 'cpp' in langs:
            total += _generate_json_cpp(
                proto_dir,
                output_dir,
                enabled_backends=backend_names,
            )
        if 'ada' in langs:
            total += _generate_json_ada(
                proto_dir,
                output_dir,
                enabled_backends=backend_names,
            )
        remaining_backends.remove('json')

    if remaining_backends:
        results = codec_backends.generate_all(
            index, output_dir,
            languages=list(langs),
            backends=remaining_backends,
        )
        for backend_name, files in results.items():
            print(f'  {backend_name}: {len(files)} files generated')
            for fp in files:
                print(f'    {fp}')
            total += len(files)

    print(f'\nDone -- {total} files generated in {output_dir}/')


if __name__ == '__main__':
    main()
