#!/usr/bin/env python3
"""
Ada Service Stub Generator
Generates PCL-aligned EntityActions service stubs from a .proto IDL file.

Each rpc in a proto service block produces a Handle_<Op>_<Entity> procedure
matching the EntityActions CRUD contract:

  CreateXxx(Xxx)            returns (Identifier)   -> Handle_Create_Xxx
  ReadXxx(XxxQuery)         returns (stream Xxx)   -> Handle_Read_Xxx
  UpdateXxx(Xxx)            returns (Ack)           -> Handle_Update_Xxx
  DeleteXxx(Identifier)     returns (Ack)           -> Handle_Delete_Xxx

A Dispatch procedure is generated as the single integration point for any
transport (PCL, socket, shared memory, etc.) -- it routes an incoming
operation+channel to the correct typed handler.

Generated packages reference the types package derived from the proto file.

Service wire-name constants and PCL binding procedures
(Subscribe_*, Invoke_*, Publish_*) are generated for standard PYRAMID protocol
interaction. JSON serialisation/deserialisation is delegated to the generated
proto-native data-model codec packages.

Architecture: component logic > service binding (this layer) > pcl

Usage:
    python ada_service_generator.py <file.proto> <output_dir>
    python ada_service_generator.py <proto_dir/>  <output_dir>
"""

# Re-export shim: the implementation lives in the pim/ada/ package
# (generator refactor plan, phases 4+5).  This module now carries only the
# externally consumed surface (pinned by tests/test_codegen_export_surface.py)
# and is kept one release for SDK consumers; new code should import from
# ada.* / proto_resolve / proto_parser directly.

import sys
from pathlib import Path

from proto_parser import (  # noqa: F401
    parse_proto,
    parse_proto_tree,
    ProtoTypeIndex,
)
from ada.naming import (  # noqa: F401
    _ADA_SCALAR_MAP,
    _ada_array_name_for_repeated,
    _ada_cabi_pkg_from_proto_pkg,
    _ada_field_name,
    _ada_name,
    _ada_pkg_from_proto_pkg,
    _ada_pkg_segment,
    _ensure_parent_packages,
    _proto_pkg_of_type,
)
from ada.generic_service_gen import AdaGenericServiceGenerator  # noqa: F401
from ada.service_gen import AdaServiceGenerator  # noqa: F401
from ada.types_gen import AdaTypesGenerator  # noqa: F401
from ada.codec_gen import AdaDataModelCodecGenerator  # noqa: F401


def main():
    if len(sys.argv) < 2:
        print('Usage: python ada_service_generator.py <file.proto|proto_dir> <output_dir>')
        print('       python ada_service_generator.py --codec <file.proto> <output_dir>')
        print('       python ada_service_generator.py --types <data_model_dir>'
              ' <output_dir>')
        sys.exit(1)

    if sys.argv[1] == '--types':
        if len(sys.argv) < 4:
            print('Usage: python ada_service_generator.py --types'
                  ' <data_model_dir> <output_dir>')
            print('  e.g. --types proto/pyramid/data_model build/generated/pyramid_ada_bindings')
            sys.exit(1)
        gen = AdaTypesGenerator(Path(sys.argv[2]))
        gen.generate(sys.argv[3])
        print('\n\u2713 Ada types generated')
        return

    if sys.argv[1] == '--codec':
        if len(sys.argv) < 4:
            print('Usage: python ada_service_generator.py --codec'
                  ' <file.proto|data_model_dir> <output_dir>')
            sys.exit(1)
        codec_path = Path(sys.argv[2])
        if codec_path.is_dir():
            proto_files = parse_proto_tree(codec_path)
            index = ProtoTypeIndex(proto_files)
            for pf in proto_files:
                gen = AdaDataModelCodecGenerator(pf, index)
                gen.generate(sys.argv[3])
            print('\n\u2713 Ada data model codecs generated')
        else:
            print('ERROR: service-local bridge JSON codecs were removed; use generate_bindings.py for proto-native generation',
                  file=sys.stderr)
            sys.exit(1)
        return

    if len(sys.argv) < 3:
        print('Usage: python ada_service_generator.py <file.proto|proto_dir> <output_dir>')
        sys.exit(1)

    gen = AdaServiceGenerator(sys.argv[1])
    gen.generate(sys.argv[2])
    print('\n\u2713 Ada services generated')


if __name__ == '__main__':
    main()
