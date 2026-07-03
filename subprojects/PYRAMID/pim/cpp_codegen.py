#!/usr/bin/env python3
"""
C++ Service Stub Generator
Generates PCL-aligned EntityActions service stubs from a .proto IDL file.

Each rpc in a proto service block produces a handle<Op><Entity> virtual method
matching the EntityActions CRUD contract:

  CreateXxx(Xxx)            returns (Identifier)   -> handleCreateXxx
  ReadXxx(XxxQuery)         returns (stream Xxx)   -> handleReadXxx
  UpdateXxx(Xxx)            returns (Ack)           -> handleUpdateXxx
  DeleteXxx(Identifier)     returns (Ack)           -> handleDeleteXxx

A dispatch() function is generated as the single integration point for any
transport (PCL, socket, shared memory, etc.) -- it routes an incoming
ServiceChannel to the correct typed handler.

Generated file names and namespaces are derived entirely from the proto file.

Service wire-name constants, JSON builder functions (provided only), and PCL
binding functions (subscribe*, invoke*, publish*) are generated for standard
pyramid protocol interaction.

Architecture: component logic > service binding (this layer) > PCL

Usage:
    python cpp_service_generator.py <file.proto> <output_dir>
    python cpp_service_generator.py <proto_dir/>  <output_dir>
"""

# Re-export shim: the implementation lives in the pim/cpp/ package
# (generator refactor plan, phases 3+5).  This module now carries only the
# externally consumed surface (pinned by tests/test_codegen_export_surface.py)
# and is kept one release for SDK consumers; new code should import from
# cpp.* / proto_resolve / proto_parser directly.

import sys
from pathlib import Path

from proto_parser import (  # noqa: F401
    parse_proto,
    parse_proto_tree,
    ProtoTypeIndex,
)
from proto_resolve import (  # noqa: F401
    _DATA_MODEL_PROTO_ROOT,
    _field_with_type,
    _is_proto_enum_type,
    _is_proto_message_type,
    _package_for_proto_type,
    _proto_type_fqn,
    _resolve_enum,
    _resolve_message,
)
from cpp.naming import (  # noqa: F401
    _CPP_SCALAR_MAP,
    _DATA_MODEL_TYPES_HEADER,
    _DATA_MODEL_TYPES_NS,
    _cpp_ns_for_proto_package,
    _cpp_ns_for_proto_type_package,
)
from cpp.types_gen import (  # noqa: F401
    CppTypesGenerator,
    find_scalar_wrappers,
)
from cpp.json_codec_gen import CppDataModelCodecGenerator  # noqa: F401
from cpp.service_gen import CppServiceGenerator  # noqa: F401


def main():
    if len(sys.argv) < 2:
        print('Usage: python cpp_service_generator.py'
              ' <file.proto|proto_dir> <output_dir>')
        print('       python cpp_service_generator.py --codec <file.proto> <output_dir>')
        print('       python cpp_service_generator.py --types <data_model_dir>'
              ' <output_dir>')
        sys.exit(1)

    if sys.argv[1] == '--types':
        if len(sys.argv) < 4:
            print('Usage: python cpp_service_generator.py --types'
                  ' <data_model_dir> <output_dir>')
            print('  e.g. --types proto/pyramid/data_model build/generated/pyramid_cpp_bindings')
            sys.exit(1)
        gen = CppTypesGenerator(Path(sys.argv[2]))
        gen.generate(sys.argv[3])
        print('\n\u2713 C++ types generated')
        return

    if sys.argv[1] == '--codec':
        if len(sys.argv) < 4:
            print('Usage: python cpp_service_generator.py --codec'
                  ' <file.proto|data_model_dir> <output_dir>')
            sys.exit(1)
        codec_path = Path(sys.argv[2])
        if codec_path.is_dir():
            # Data model dir: generate one proto-driven codec per .proto file
            proto_files = parse_proto_tree(codec_path)
            index = ProtoTypeIndex(proto_files)
            for pf in proto_files:
                gen = CppDataModelCodecGenerator(pf, index)
                gen.generate(sys.argv[3])
            print('\n\u2713 C++ data model codecs generated')
        else:
            print('ERROR: service-local bridge JSON codecs were removed; use generate_bindings.py for proto-native generation',
                  file=sys.stderr)
            sys.exit(1)
        return

    if len(sys.argv) < 3:
        print('Usage: python cpp_service_generator.py'
              ' <file.proto|proto_dir> <output_dir>')
        sys.exit(1)

    gen = CppServiceGenerator(sys.argv[1])
    gen.generate(sys.argv[2])
    print('\n\u2713 C++ services generated')


if __name__ == '__main__':
    main()
