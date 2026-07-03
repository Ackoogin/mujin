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

# This module is now a re-export shim: the implementation lives in the
# pim/cpp/ package (generator refactor plan, phase 3).  Importers should
# migrate to the new module paths; this surface is kept one release for
# SDK consumers (tests/test_codegen_export_surface.py pins it).

import sys
from pathlib import Path

from proto_parser import (  # noqa: F401
    parse_proto,
    parse_proto_tree, ProtoTypeIndex, ProtoMessage, ProtoEnum, ProtoField,
    ProtoFile, ProtoRpc, ProtoService,
    screaming_to_pascal, _PROTO_SCALARS,
    camel_to_snake as _camel_to_snake,
    camel_to_lower_snake as _camel_to_lower_snake,
    snake_to_pascal as _snake_to_pascal,
    lc_first as _lc_first,
)
from proto_resolve import (  # noqa: F401
    _DATA_MODEL_PROTO_ROOT,
    _data_model_package_for_type,
    _enum_matches,
    _field_with_type,
    _is_proto_enum_type,
    _is_proto_message_type,
    _message_matches,
    _package_for_proto_type,
    _proto_type_fqn,
    _qualified_package_for_type,
    _resolve_enum,
    _resolve_message,
)
from cpp.naming import (  # noqa: F401
    OP_PREFIXES,
    BASE_TYPE_MAP,
    _SEP,
    _DEFAULT_CONTENT_TYPE,
    _ALIAS_FIELD_NAMES,
    _CPP_SCALAR_MAP,
    _CPP_DEFAULTS,
    _LITERAL_CPP_TYPES,
    _FORCED_ALIASES,
    _UNIT_FIELD_NAMES,
    _STRUCT_CONSTANTS,
    _singularize,
    _topic_key_to_phrase,
    _cpp_qos_reliability_expr,
    _short_type,
    _mapped_type,
    _service_wire_prefix,
    _service_cpp_prefix,
    _duplicate_rpc_names,
    _rpc_symbol_base,
    _rpc_handler_name,
    _rpc_enum_value,
    _rpc_service_const,
    _rpc_invoke_func,
    _rpc_decode_response_func,
    _rpc_stream_handler_name,
    _rpc_encode_stream_frame_func,
    _rpc_decode_stream_frame_func,
    _rpc_send_stream_frame_func,
    _rpc_invoke_stream_func,
    _rpc_op,
    _crud_rpcs,
    _rpc_wire_name,
    _cpp_req_type,
    _cpp_rsp_type,
    _DATA_MODEL_TYPES_NS,
    _DATA_MODEL_TYPES_HEADER,
    _DEFAULT_NAMING_POLICY,
    _cpp_ns_for_proto_package,
    _cpp_ns_for_proto_type_package,
    _legacy_service_namespace,
    _namespace_from_proto,
    _is_provided,
    _c_struct_for_type,
    _native_namespace_for_type,
    _service_codec_imports,
    _service_group_key,
    _service_contract_names,
    _json_codec_namespace_for_type,
    _json_codec_header_for_type,
    _alias_cpp_types,
    _find_proto_root,
)
from cpp.types_gen import (  # noqa: F401
    find_scalar_wrappers,
    _common_cpp_ns,
    CppTypesGenerator,
)
from cpp.json_codec_gen import (  # noqa: F401
    _CPP_INTEGRAL_SCALARS,
    CppDataModelCodecGenerator,
)
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
