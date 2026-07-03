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

# This module is now a re-export shim: the implementation lives in the
# pim/ada/ package (generator refactor plan, phase 4).  Importers should
# migrate to the new module paths; this surface is kept one release for
# SDK consumers (tests/test_codegen_export_surface.py pins it).

import sys
from pathlib import Path

from proto_parser import (  # noqa: F401
    parse_proto_tree, ProtoTypeIndex, ProtoMessage, ProtoEnum,
    ProtoFile, ProtoRpc, ProtoService,
    screaming_to_pascal, camel_to_snake, _PROTO_SCALARS,
    parse_proto,
    camel_to_snake as _camel_to_snake,
    camel_to_lower_snake as _camel_to_lower_snake,
)
from ada.naming import (  # noqa: F401
    OP_PREFIXES,
    BASE_TYPE_MAP,
    _short_type,
    _proto_type_to_ada,
    _service_wire_prefix,
    _service_ada_prefix,
    _duplicate_rpc_names,
    _rpc_ada_base,
    _rpc_ada_handler,
    _rpc_ada_channel,
    _rpc_ada_invoke_name,
    _rpc_ada_decode_response_name,
    _rpc_ada_svc_const,
    _rpc_ada_handler_field,
    _rpc_ada_callback_name,
    _rpc_op,
    _crud_rpcs,
    _rpc_wire_name,
    _ada_req_type,
    _ada_rsp_type,
    _pkg_name_from_proto,
    _generic_pkg_name_from_proto,
    _is_provided,
    _DATA_MODEL_TYPES_PKGS,
    _DATA_MODEL_TYPES_PKG,
    _DM_CODEC_PKG_FOR_TYPE_PKG,
    _proto_pkg_of_type,
    _ada_pkg_from_proto_pkg,
    _find_proto_root,
    _collect_type_pkgs,
    _collect_codec_pkgs,
    _data_model_msg_pkgs,
    _applicable_topics,
    _types_pkg_from_proto,
    _flatbuffers_codec_pkg_from_proto,
    _grpc_transport_pkg_from_proto,
    _ada_cabi_pkg_from_proto_pkg,
    _ada_cabi_type_name,
    _service_wrapper_pf,
    _binding_proto_files,
    _collect_array_schema_bindings,
    _flatbuffers_func_suffix_for_type,
    _flatbuffers_func_suffix_for_stream,
    _ensure_parent_packages,
    _ADA_SCALAR_MAP,
    _ADA_DEFAULTS,
    _ADA_UNIT_FIELD_NAMES,
    _ADA_RESERVED_WORDS,
    _ada_name,
    _ada_array_name_for_repeated,
    _ada_pkg_segment,
    _ada_field_name,
    _common_ada_pkg,
)
from ada.generic_service_gen import AdaGenericServiceGenerator  # noqa: F401
from ada.service_gen import AdaServiceGenerator  # noqa: F401
from ada.service_body_gen import (  # noqa: F401
    _collect_cabi_message_bindings,
    _collect_alias_schema_bindings,
)
from ada.types_gen import AdaTypesGenerator, _common_ada_pkg  # noqa: F401
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
