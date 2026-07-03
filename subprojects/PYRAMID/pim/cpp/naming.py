#!/usr/bin/env python3
"""C++ naming, namespace and rpc-symbol derivation helpers.

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from proto_parser import (
    ProtoTypeIndex,
    ProtoFile,
    ProtoRpc,
    ProtoService,
    camel_to_lower_snake as _camel_to_lower_snake,
)
from proto_resolve import (
    _data_model_package_for_type,
    _qualified_package_for_type,
)
from binding_contract import (
    BindingTopic,
    PyramidCompatNamingPolicy,
    TopicSpecResolver,
)


# -- EntityActions operation set -----------------------------------------------

OP_PREFIXES = ['Create', 'Read', 'Update', 'Delete', 'Cancel']

# Base-type short names from pyramid.data_model.base.* and common.*
BASE_TYPE_MAP = {
    'pyramid.data_model.base.Identifier': 'Identifier',
    'pyramid.data_model.base.Query':      'Query',
    'pyramid.data_model.base.Ack':        'Ack',
    'pyramid.data_model.common.Query':    'Query',
    'pyramid.data_model.common.Ack':      'Ack',
}

# Standard topic names -- tactical_objects bridge-facing topics aligned
# directly to canonical proto-derived PYRAMID payloads.

_SEP = '// ' + '-' * 75

# Default content type -- used when no port-level override is provided.
# Generated code accepts content_type as a parameter so components can
# configure per-port codecs at pcl_container_add_* time.
_DEFAULT_CONTENT_TYPE = 'application/json'

_ALIAS_FIELD_NAMES = frozenset({
    'value', 'radians', 'meters', 'meters_per_second', 'seconds',
    'kilograms', 'kelvin', 'pascals', 'hertz',
})

# -- C++ type-mapping tables ---------------------------------------------------

_CPP_SCALAR_MAP: Dict[str, str] = {
    'double': 'double', 'float': 'float',
    'int32': 'int32_t', 'int64': 'int64_t',
    'uint32': 'uint32_t', 'uint64': 'uint64_t',
    'sint32': 'int32_t', 'sint64': 'int64_t',
    'fixed32': 'uint32_t', 'fixed64': 'uint64_t',
    'sfixed32': 'int32_t', 'sfixed64': 'int64_t',
    'bool': 'bool', 'string': 'std::string', 'bytes': 'std::string',
}

_CPP_DEFAULTS: Dict[str, str] = {
    'double': '0.0', 'float': '0.0f',
    'int32_t': '0', 'int64_t': '0', 'uint32_t': '0', 'uint64_t': '0',
    'bool': 'false', 'std::string': '{}',
}

# C++ field types that keep a struct a literal type (so its named constants can
# stay constexpr). std::string and aggregate types are deliberately excluded.
_LITERAL_CPP_TYPES: frozenset = frozenset({
    'double', 'float', 'int32_t', 'int64_t', 'uint32_t', 'uint64_t', 'bool',
})

# Messages whose single scalar field makes them transparent wrappers.
# Identifier is special-cased to std::string; Timestamp to double (epoch s).
_FORCED_ALIASES: Dict[str, str] = {
    'Timestamp': 'double',  # google.protobuf.Timestamp wrapper -> epoch seconds
}

# Single-field message is a scalar wrapper only when the field name signals
# a physical unit or a generic "value" placeholder -- NOT domain names like
# "success" (Ack) which carry independent meaning as a struct.
_UNIT_FIELD_NAMES = frozenset({
    'value', 'radians', 'meters', 'meters_per_second', 'seconds',
    'kilograms', 'kelvin', 'pascals', 'hertz',
})

# After emitting a struct, optionally emit named constants { name: initialiser }.
_STRUCT_CONSTANTS: Dict[str, List[Tuple[str, str]]] = {
    'Ack': [
        ('kAckOk',   'Ack{ true  }'),
        ('kAckFail', 'Ack{ false }'),
    ],
}


# -- Proto parser --------------------------------------------------------------

def _singularize(word: str) -> str:
    """Naive singularization: matches->match, requirements->requirement."""
    if word.endswith('ies'):
        return word[:-3] + 'y'
    if word.endswith('es') and len(word) > 3:
        return word[:-2]
    if word.endswith('s') and len(word) > 1 and not word.endswith('ss'):
        return word[:-1]
    return word


def _topic_key_to_phrase(key: str) -> str:
    """entity_matches -> entity-match (singular, hyphenated)."""
    words = key.split('_')
    words[-1] = _singularize(words[-1])
    return '-'.join(words)


def _cpp_qos_reliability_expr(spec: BindingTopic) -> str:
    floor = spec.reliability_floor
    if floor == 'reliable':
        return 'PCL_QOS_RELIABILITY_RELIABLE'
    if floor == 'best_effort':
        return 'PCL_QOS_RELIABILITY_BEST_EFFORT'
    return 'PCL_QOS_RELIABILITY_UNSPECIFIED'


def _short_type(full_type: str) -> str:
    """Raw last segment, no BASE_TYPE_MAP substitution.

    pyramid.data_model.tactical.ObjectMatch -> ObjectMatch
    """
    return full_type.split('.')[-1]


def _mapped_type(full_type: str) -> str:
    """Apply BASE_TYPE_MAP, then take last segment.

    pyramid.data_model.common.Capability -> Identifier (via BASE_TYPE_MAP)
    pyramid.data_model.tactical.ObjectMatch -> ObjectMatch
    """
    if full_type in BASE_TYPE_MAP:
        return BASE_TYPE_MAP[full_type]
    return full_type.split('.')[-1]


def _service_wire_prefix(service_name: str) -> str:
    """Object_Of_Interest_Service -> object_of_interest."""
    name = service_name
    if name.endswith('_Service'):
        name = name[:-len('_Service')]
    return _camel_to_lower_snake(name)


def _service_cpp_prefix(service_name: str) -> str:
    """Data_Provision_Dependency_Service -> DataProvisionDependency."""
    name = service_name
    if name.endswith('_Service'):
        name = name[:-len('_Service')]
    return ''.join(part.capitalize() for part in name.split('_') if part)


def _duplicate_rpc_names(all_rpcs: List[tuple[str, 'ProtoRpc']]) -> set[str]:
    counts: Dict[str, int] = {}
    for _svc_name, rpc in all_rpcs:
        counts[rpc.name] = counts.get(rpc.name, 0) + 1
    return {name for name, count in counts.items() if count > 1}


def _rpc_symbol_base(svc_name: str, rpc: 'ProtoRpc',
                     duplicate_rpc_names: set[str]) -> str:
    del duplicate_rpc_names
    return _service_cpp_prefix(svc_name) + rpc.name


def _rpc_handler_name(svc_name: str, rpc: 'ProtoRpc',
                      duplicate_rpc_names: set[str]) -> str:
    return f'handle{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_enum_value(svc_name: str, rpc: 'ProtoRpc',
                    duplicate_rpc_names: set[str]) -> str:
    return _rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)


def _rpc_service_const(svc_name: str, rpc: 'ProtoRpc',
                       duplicate_rpc_names: set[str]) -> str:
    return f'kSvc{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_invoke_func(svc_name: str, rpc: 'ProtoRpc',
                     duplicate_rpc_names: set[str]) -> str:
    return f'invoke{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_decode_response_func(svc_name: str, rpc: 'ProtoRpc',
                              duplicate_rpc_names: set[str]) -> str:
    return f'decode{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}Response'


def _rpc_stream_handler_name(svc_name: str, rpc: 'ProtoRpc',
                             duplicate_rpc_names: set[str]) -> str:
    return f'stream{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_encode_stream_frame_func(svc_name: str, rpc: 'ProtoRpc',
                                  duplicate_rpc_names: set[str]) -> str:
    return f'encode{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}StreamFrame'


def _rpc_decode_stream_frame_func(svc_name: str, rpc: 'ProtoRpc',
                                  duplicate_rpc_names: set[str]) -> str:
    return f'decode{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}StreamFrame'


def _rpc_send_stream_frame_func(svc_name: str, rpc: 'ProtoRpc',
                                duplicate_rpc_names: set[str]) -> str:
    return f'send{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}StreamFrame'


def _rpc_invoke_stream_func(svc_name: str, rpc: 'ProtoRpc',
                            duplicate_rpc_names: set[str]) -> str:
    return f'invoke{_rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)}Stream'


# -- Proto model ---------------------------------------------------------------

def _rpc_op(rpc: ProtoRpc) -> Optional[str]:
    """EntityActions CRUD operation prefix of an rpc, or None if non-CRUD."""
    for prefix in OP_PREFIXES:
        if rpc.name.startswith(prefix):
            return prefix
    return None


def _crud_rpcs(svc: ProtoService) -> List[ProtoRpc]:
    """The CRUD-named rpcs of a service (the EntityActions binding surface)."""
    return [r for r in svc.rpcs if _rpc_op(r) is not None]


def _rpc_wire_name(rpc: ProtoRpc) -> str:
    """create_requirement (rpc wire-name segment)."""
    return _camel_to_lower_snake(rpc.name)


def _cpp_req_type(rpc: ProtoRpc) -> str:
    """C++ request type after BASE_TYPE_MAP mapping."""
    return _mapped_type(rpc.request_type)


def _cpp_rsp_type(rpc: ProtoRpc) -> str:
    """C++ response type: std::vector<X> for streaming, X otherwise."""
    base = _mapped_type(rpc.response_type)
    if rpc.server_streaming:
        return f'std::vector<{base}>'
    return base


# -- C++ namespace / file name derivation -------------------------------------

_DATA_MODEL_TYPES_NS = 'pyramid::domain_model'
_DATA_MODEL_TYPES_HEADER = 'pyramid_data_model_types.hpp'
_DEFAULT_NAMING_POLICY = PyramidCompatNamingPolicy()


def _cpp_ns_for_proto_package(package: str) -> str:
    return _DEFAULT_NAMING_POLICY.cpp_namespace_for_package(package)


def _cpp_ns_for_proto_type_package(package: str) -> str:
    return _DEFAULT_NAMING_POLICY.cpp_type_namespace_for_package(package)

def _legacy_service_namespace(package: str) -> Tuple[str, str]:
    """Return the legacy service namespace base and generated file prefix.

    Keep these stable so existing generated codec file names and checked-in
    service protobuf codec headers continue to resolve.
    """
    return _DEFAULT_NAMING_POLICY.legacy_service_namespace(package)


def _namespace_from_proto(proto_file: ProtoFile) -> Tuple[str, str, str, str]:
    """Return (full_namespace, file_prefix, svc_base_ns, types_namespace).

    pyramid.components.tactical_objects.services.provided
      -> full_ns    : pyramid::components::tactical_objects::services::provided
      -> file_prefix: pyramid_services_tactical_objects_provided
      -> svc_base_ns: pyramid::services::tactical_objects
      -> types_ns   : pyramid::domain_model
    """
    full_ns = _DEFAULT_NAMING_POLICY.service_namespace(proto_file.package)
    svc_base_ns, file_prefix = _legacy_service_namespace(proto_file.package)

    return full_ns, file_prefix, svc_base_ns, _DATA_MODEL_TYPES_NS


def _is_provided(proto_file: ProtoFile) -> bool:
    return 'provided' in proto_file.package.lower()


def _c_struct_for_type(full_type: str) -> str:
    """Package-qualified C-ABI struct symbol for a proto type, matching
    cabi_codegen._c_struct_name (and the Ada generator)."""
    package = _qualified_package_for_type(full_type)
    short = full_type.split('.')[-1]
    if package:
        return f'{package.replace(".", "_")}_{short}_c'
    return f'pyramid_{short}_c'


def _native_namespace_for_type(full_type: str) -> str:
    """C++ namespace where the *plain struct* for a proto type is referenced.

    Data-model types are referenced flat in pyramid::domain_model (the umbrella
    header re-exports each sub-namespace into it). Service-local wrapper
    messages live in their component's own service namespace -- the canonical
    Component-NS home shared with the protobuf/flatbuffers/grpc/ros2 backends.
    """
    package = _qualified_package_for_type(full_type)
    if not package:
        return _DATA_MODEL_TYPES_NS
    return _cpp_ns_for_proto_package(package)


def _service_codec_imports(
        data_model_files: List[ProtoFile],
        all_rpcs: List[Tuple[str, ProtoRpc]],
        topic_keys: List[str],
        topics: TopicSpecResolver,
) -> List[Tuple[str, str]]:
    packages_with_messages = {
        pf.package for pf in data_model_files
        if pf.messages
    }
    packages_with_messages.discard('pyramid.data_model.base')
    needed_packages = set()
    for _svc_name, rpc in all_rpcs:
        for full_type in (rpc.request_type, rpc.response_type):
            package = _data_model_package_for_type(full_type)
            if package in packages_with_messages:
                needed_packages.add(package)
    for key in topic_keys:
        package = _data_model_package_for_type(topics.spec(key).full_type)
        if package in packages_with_messages:
            needed_packages.add(package)
    return [
        (_cpp_ns_for_proto_package(package),
         f'{package.replace(".", "_")}_codec.hpp')
        for package in sorted(needed_packages)
    ]


def _service_group_key(package: str) -> Optional[str]:
    """Return the role-independent service contract package key."""
    if '.services.' not in f'.{package}.':
        return None
    parts = [p for p in package.split('.') if p]
    if parts and parts[-1].lower() in ('provided', 'consumed'):
        parts = parts[:-1]
    return '.'.join(parts)


def _service_contract_names(base_package: str) -> Tuple[str, str]:
    """Return (file_base, cpp_base_namespace) for a service contract."""
    parts = [p for p in base_package.split('.') if p]
    skip = {'pyramid', 'components', 'services', 'data_model', 'base'}
    meaningful = [p for p in parts if p.lower() not in skip]
    ns_parts = ['pyramid', 'services'] + [p.lower() for p in meaningful]
    return '_'.join(ns_parts), '::'.join(ns_parts)


def _json_codec_namespace_for_type(full_type: str) -> str:
    """Return the generated JSON codec C++ namespace for a proto type."""
    package = _qualified_package_for_type(full_type)
    if not package:
        return _DATA_MODEL_TYPES_NS
    return _cpp_ns_for_proto_package(package)


def _json_codec_header_for_type(full_type: str) -> str:
    package = _qualified_package_for_type(full_type)
    if not package:
        return ''
    return f'{package.replace(".", "_")}_codec.hpp'


def _alias_cpp_types(index: ProtoTypeIndex) -> Dict[str, str]:
    """Mirror CppTypesGenerator scalar-wrapper aliases."""
    aliases: Dict[str, str] = dict(_FORCED_ALIASES)
    for msg in index.all_messages():
        fields = msg.all_fields()
        if len(fields) == 1 and not fields[0].is_repeated:
            field = fields[0]
            if field.type in _CPP_SCALAR_MAP and field.name in _ALIAS_FIELD_NAMES:
                aliases[msg.name] = _CPP_SCALAR_MAP[field.type]
    return aliases


def _find_proto_root(proto_input: Path) -> Optional[Path]:
    if proto_input.is_dir():
        return proto_input
    for parent in [proto_input.parent, *proto_input.parents]:
        # The package tree root is conventionally a 'proto' dir, but any
        # directory holding the 'pyramid' package root works too (e.g. the new
        # 'pim/test' layout), so both data-model and component protos are seen.
        if parent.name.lower() == 'proto' or (parent / 'pyramid').is_dir():
            return parent
    return proto_input.parent if proto_input.parent.exists() else None


