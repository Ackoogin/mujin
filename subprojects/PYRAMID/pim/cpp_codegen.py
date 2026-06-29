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

import sys
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from proto_parser import (
    parse_proto_tree, ProtoTypeIndex, ProtoMessage, ProtoEnum, ProtoField,
    screaming_to_pascal, _PROTO_SCALARS,
)
from standard_topics import topic_spec, topics_for_service


# -- EntityActions operation set -----------------------------------------------

OP_PREFIXES = ['Create', 'Read', 'Update', 'Delete']

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

# -- Proto parser --------------------------------------------------------------

def _strip_comments(text: str) -> str:
    """Remove // line comments and /* */ block comments."""
    text = re.sub(r'//[^\n]*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text


def _camel_to_snake(name: str) -> str:
    """TacticalObject -> Tactical_Object (Ada identifier style)."""
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    s = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s)
    return s


def _camel_to_lower_snake(name: str) -> str:
    """ObjectOfInterest -> object_of_interest (wire-format style)."""
    return _camel_to_snake(name).lower()


def _snake_to_pascal(name: str) -> str:
    """entity_matches -> EntityMatches."""
    return ''.join(w.capitalize() for w in name.split('_'))


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

class ProtoRpc:
    """One rpc entry extracted from a proto service block."""

    def __init__(self, name: str, req: str, rsp: str, streaming: bool):
        self.name = name
        self.req = req
        self.rsp = rsp
        self.streaming = streaming

        self.op: Optional[str] = None
        self.entity: Optional[str] = None
        for prefix in OP_PREFIXES:
            if name.startswith(prefix):
                self.op = prefix
                self.entity = name[len(prefix):]
                break

    # -- C++ naming ------------------------------------------------------------

    @property
    def cpp_handler(self) -> str:
        """handleCreateRequirement."""
        return f'handle{self.name}'

    @property
    def cpp_enum_value(self) -> str:
        """CreateRequirement (ServiceChannel enum value)."""
        return self.name

    @property
    def cpp_svc_const(self) -> str:
        """kSvcCreateRequirement."""
        return f'kSvc{self.name}'

    @property
    def cpp_invoke_func(self) -> str:
        """invokeCreateRequirement."""
        return f'invoke{self.name}'

    @property
    def cpp_decode_response_func(self) -> str:
        """decodeCreateRequirementResponse."""
        return f'decode{self.name}Response'

    @property
    def cpp_req_type(self) -> str:
        """C++ request type after BASE_TYPE_MAP mapping."""
        return _mapped_type(self.req)

    @property
    def cpp_rsp_type(self) -> str:
        """C++ response type: std::vector<X> for streaming, X otherwise."""
        base = _mapped_type(self.rsp)
        if self.streaming:
            return f'std::vector<{base}>'
        return base

    @property
    def wire_name(self) -> str:
        """create_requirement (rpc wire-name segment)."""
        return _camel_to_lower_snake(self.name)

    @property
    def raw_req_type(self) -> str:
        """Raw request type name (no BASE_TYPE_MAP), for using declarations."""
        return _short_type(self.req)

    @property
    def raw_rsp_type(self) -> str:
        """Raw response type name (no BASE_TYPE_MAP), for using declarations."""
        return _short_type(self.rsp)


class ProtoService:
    def __init__(self, name: str, rpcs: List[ProtoRpc]):
        self.name = name
        self.rpcs = [r for r in rpcs if r.op is not None]

    @property
    def wire_prefix(self) -> str:
        return _service_wire_prefix(self.name)


class ProtoFile:
    def __init__(self, package: str, services: List[ProtoService]):
        self.package = package
        self.services = services


def parse_proto(proto_path: Path) -> ProtoFile:
    text = _strip_comments(proto_path.read_text(encoding='utf-8'))

    pkg_match = re.search(r'\bpackage\s+([\w.]+)\s*;', text)
    package = pkg_match.group(1) if pkg_match else ''

    services: List[ProtoService] = []
    for svc_match in re.finditer(r'\bservice\s+(\w+)\s*\{([^}]*)\}', text, re.DOTALL):
        svc_name = svc_match.group(1)
        svc_body = svc_match.group(2)

        rpcs: List[ProtoRpc] = []
        for rpc_match in re.finditer(
                r'\brpc\s+(\w+)\s*\(\s*([\w.]+)\s*\)\s*returns\s*\(\s*(stream\s+)?([\w.]+)\s*\)',
                svc_body):
            rpc_name = rpc_match.group(1)
            req_type = rpc_match.group(2)
            streaming = bool(rpc_match.group(3))
            rsp_type = rpc_match.group(4)
            rpcs.append(ProtoRpc(rpc_name, req_type, rsp_type, streaming))

        services.append(ProtoService(svc_name, rpcs))

    return ProtoFile(package, services)


# -- C++ namespace / file name derivation -------------------------------------

_DATA_MODEL_PROTO_ROOT = 'pyramid.data_model'
_DATA_MODEL_TYPES_NS = 'pyramid::domain_model'
_DATA_MODEL_TYPES_HEADER = 'pyramid_data_model_types.hpp'


def _cpp_ns_for_proto_package(package: str) -> str:
    if package == _DATA_MODEL_PROTO_ROOT:
        return _DATA_MODEL_TYPES_NS
    if package.startswith(_DATA_MODEL_PROTO_ROOT + '.'):
        suffix = package[len(_DATA_MODEL_PROTO_ROOT) + 1:]
        return _DATA_MODEL_TYPES_NS + '::' + suffix.replace('.', '::')
    return package.replace('.', '::')


def _cpp_ns_for_proto_type_package(package: str) -> str:
    return _cpp_ns_for_proto_package(package)

def _legacy_service_namespace(package: str) -> Tuple[str, str]:
    """Return the legacy service namespace base and generated file prefix.

    Keep these stable so existing generated codec file names and checked-in
    service protobuf codec headers continue to resolve.
    """
    parts = package.split('.')

    last = parts[-1].lower()
    suffix = None
    if last in ('provided', 'consumed'):
        suffix = last
        parts = parts[:-1]

    skip = {'pyramid', 'components', 'data_model', 'base', 'services'}
    meaningful = [p for p in parts if p.lower() not in skip]

    ns_parts = ['pyramid', 'services'] + [p.lower() for p in meaningful]
    suffix = suffix or 'provided'
    return '::'.join(ns_parts), '_'.join(ns_parts + [suffix])


def _namespace_from_proto(proto_file: ProtoFile) -> Tuple[str, str, str, str]:
    """Return (full_namespace, file_prefix, svc_base_ns, types_namespace).

    pyramid.components.tactical_objects.services.provided
      -> full_ns    : pyramid::components::tactical_objects::services::provided
      -> file_prefix: pyramid_services_tactical_objects_provided
      -> svc_base_ns: pyramid::services::tactical_objects
      -> types_ns   : pyramid::domain_model
    """
    parts = proto_file.package.split('.')

    last = parts[-1].lower()
    suffix = None
    if last in ('provided', 'consumed'):
        suffix = last
        parts = parts[:-1]

    suffix = suffix or 'provided'
    full_ns = '::'.join(parts + [suffix])
    svc_base_ns, file_prefix = _legacy_service_namespace(proto_file.package)

    return full_ns, file_prefix, svc_base_ns, _DATA_MODEL_TYPES_NS


def _is_provided(proto_file: ProtoFile) -> bool:
    return 'provided' in proto_file.package.lower()


def _topics_for_proto(
        parsed: 'ProtoFile', is_provided: bool
) -> Tuple[Dict[str, str], Dict[str, str]]:
    """Return (sub_topics, pub_topics) for the service, based on its package."""
    return topics_for_service(parsed.package, is_provided)


def _data_model_package_for_type(full_type: str) -> str:
    if not full_type.startswith(_DATA_MODEL_PROTO_ROOT + '.'):
        return ''
    if '.' not in full_type:
        return ''
    return full_type.rsplit('.', 1)[0]


def _qualified_package_for_type(full_type: str) -> str:
    """Package of any fully-qualified proto type (data-model or component).

    Unlike _data_model_package_for_type this does not restrict to the
    pyramid.data_model root, so service-local wrapper messages (canonically
    homed in their component package) also resolve.
    """
    if '.' not in full_type:
        return ''
    return full_type.rsplit('.', 1)[0]


def _message_matches(index: ProtoTypeIndex,
                     short_name: str) -> List[Tuple[str, ProtoMessage]]:
    result = []
    for pf in index.files:
        for msg in pf.messages:
            if msg.name == short_name:
                result.append((pf.package, msg))
    return result


def _enum_matches(index: ProtoTypeIndex,
                  short_name: str) -> List[Tuple[str, ProtoEnum]]:
    result = []
    for pf in index.files:
        for enum in pf.enums:
            if enum.name == short_name:
                result.append((pf.package, enum))
    return result


def _resolve_message(index: ProtoTypeIndex, type_name: str,
                     current_pkg: str = '') -> Tuple[Optional[ProtoMessage], str]:
    """Resolve a proto message with package-aware bare-name handling.

    Bare proto names are scoped to the current package first. If there is no
    current-package match, a bare name is accepted only when it is unique in the
    index. This keeps duplicate short names from resolving to an arbitrary
    package.
    """
    if not type_name or type_name in _PROTO_SCALARS:
        return None, ''
    if type_name.startswith('google.'):
        return None, ''
    if '.' in type_name:
        return index.resolve_message(type_name), type_name.rsplit('.', 1)[0]
    if current_pkg:
        fqn = f'{current_pkg}.{type_name}'
        msg = index.resolve_message(fqn)
        if msg is not None:
            return msg, current_pkg
    matches = _message_matches(index, type_name)
    if len(matches) == 1:
        return matches[0][1], matches[0][0]
    return None, ''


def _resolve_enum(index: ProtoTypeIndex, type_name: str,
                  current_pkg: str = '') -> Tuple[Optional[ProtoEnum], str]:
    if not type_name or type_name in _PROTO_SCALARS:
        return None, ''
    if type_name.startswith('google.'):
        return None, ''
    if '.' in type_name:
        return index.resolve_enum(type_name), type_name.rsplit('.', 1)[0]
    if current_pkg:
        fqn = f'{current_pkg}.{type_name}'
        enum = index.resolve_enum(fqn)
        if enum is not None:
            return enum, current_pkg
    matches = _enum_matches(index, type_name)
    if len(matches) == 1:
        return matches[0][1], matches[0][0]
    return None, ''


def _proto_type_fqn(index: ProtoTypeIndex, type_name: str,
                    current_pkg: str = '') -> str:
    if not type_name or type_name in _PROTO_SCALARS:
        return type_name
    if type_name.startswith('google.'):
        return type_name
    if '.' in type_name:
        return type_name
    msg, pkg = _resolve_message(index, type_name, current_pkg)
    if msg is not None and pkg:
        return f'{pkg}.{msg.name}'
    enum, pkg = _resolve_enum(index, type_name, current_pkg)
    if enum is not None and pkg:
        return f'{pkg}.{enum.name}'
    return ''


def _package_for_proto_type(index: ProtoTypeIndex, type_name: str,
                            current_pkg: str = '') -> str:
    if '.' in type_name and not type_name.startswith('google.'):
        return type_name.rsplit('.', 1)[0]
    fqn = _proto_type_fqn(index, type_name, current_pkg)
    if '.' in fqn and not fqn.startswith('google.'):
        return fqn.rsplit('.', 1)[0]
    return ''


def _is_proto_message_type(index: ProtoTypeIndex, type_name: str,
                           current_pkg: str = '') -> bool:
    fqn = _proto_type_fqn(index, type_name, current_pkg)
    return bool(fqn and index.is_message_type(fqn))


def _is_proto_enum_type(index: ProtoTypeIndex, type_name: str,
                        current_pkg: str = '') -> bool:
    fqn = _proto_type_fqn(index, type_name, current_pkg)
    return bool(fqn and index.is_enum_type(fqn))


def _field_with_type(field: ProtoField, field_type: str) -> ProtoField:
    return ProtoField(
        name=field.name,
        type=field_type or field.type,
        number=field.number,
        label=field.label,
        oneof_group=field.oneof_group,
    )


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
) -> List[Tuple[str, str]]:
    packages_with_messages = {
        pf.package for pf in data_model_files
        if pf.messages
    }
    packages_with_messages.discard('pyramid.data_model.base')
    needed_packages = set()
    for _svc_name, rpc in all_rpcs:
        for full_type in (rpc.req, rpc.rsp):
            package = _data_model_package_for_type(full_type)
            if package in packages_with_messages:
                needed_packages.add(package)
    for key in topic_keys:
        package = _data_model_package_for_type(topic_spec(key).full_type)
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


# -- Code generation -----------------------------------------------------------

class CppServiceGenerator:
    _dm_proto_cache: Dict[Path, List[ProtoFile]] = {}

    def __init__(self, proto_input: str, enabled_backends=None):
        self._proto_input = Path(proto_input)
        self._enabled_backends = set(enabled_backends or [
            'json', 'flatbuffers', 'protobuf',
        ])

    def _has_backend(self, name: str) -> bool:
        return name in self._enabled_backends

    def _service_protobuf_codec_available(self, svc_base_ns: str) -> bool:
        """Return whether a local-struct service protobuf codec exists.

        The current protobuf backend emits data-model protobuf wrappers.  The
        generated service facade, however, needs local-struct <-> protobuf
        conversion helpers. Those exist today as source support for
        tactical_objects; do not emit service protobuf dispatch code for service
        packages that do not have that bridge.
        """
        header = '_'.join(svc_base_ns.split('::')) + '_protobuf_codec.hpp'
        candidates = []
        for parent in [self._proto_input, *self._proto_input.parents]:
            if parent.is_file():
                parent = parent.parent
            candidates.append(parent / 'src' / 'protobuf_support' / header)
        return any(path.exists() for path in candidates)

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
                for rpc in svc.rpcs:
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

    def _collect_contract_codec_types(
            self, parsed: ProtoFile,
    ) -> Tuple[str, str, List[Tuple[str, str, bool, str]]]:
        """Return contract file base, namespace, and root schema types.

        Type tuples are (short_name, full_type, is_alias, alias_cpp_type).
        """
        base_package, service_files = self._contract_service_files(parsed.package)
        if not base_package:
            return '', '', []
        if not service_files:
            return '', '', []
        service_files = [
            pf for pf in service_files
            if pf.package == parsed.package
        ]
        if not service_files:
            return '', '', []

        dm_files = [
            pf for pf in self._discover_all_proto_files()
            if pf.package == _DATA_MODEL_PROTO_ROOT
            or pf.package.startswith(_DATA_MODEL_PROTO_ROOT + '.')
        ]
        indexed_files = dm_files + [
            pf for pf in service_files
            if pf not in dm_files
        ]
        index = ProtoTypeIndex(indexed_files)
        aliases = _alias_cpp_types(index)
        file_base, cpp_base_ns = _service_contract_names(base_package)
        root_types: List[Tuple[str, str]] = []

        for pf in service_files:
            for svc in pf.services:
                for rpc in svc.rpcs:
                    root_types.append((rpc.request_type, pf.package))
                    root_types.append((rpc.response_type, pf.package))
            is_provided = 'provided' in pf.package.lower()
            sub_topics, pub_topics = _topics_for_proto(pf, is_provided)
            topic_keys = list(sub_topics.keys()) + list(pub_topics.keys())
            for key in topic_keys:
                root_types.append((topic_spec(key).full_type, ''))

        result: List[Tuple[str, str, bool, str, bool]] = []
        seen_schema_ids = set()
        for type_name, current_pkg in root_types:
            short = type_name.split('.')[-1]
            if short in seen_schema_ids:
                continue

            msg, msg_pkg = _resolve_message(index, type_name, current_pkg)
            if msg is None and short not in aliases:
                continue

            full_type = type_name
            if msg is not None and '.' not in type_name:
                full_type = f'{msg_pkg}.{msg.name}' if msg_pkg else type_name

            result.append((short, full_type, short in aliases,
                           aliases.get(short, ''), False))
            seen_schema_ids.add(short)

        for pf in service_files:
            for svc in pf.services:
                for rpc in svc.rpcs:
                    if not (getattr(rpc, 'streaming', False)
                            or getattr(rpc, 'server_streaming', False)):
                        continue
                    elem_short = rpc.response_type.split('.')[-1]
                    array_schema = elem_short + 'Array'
                    if array_schema in seen_schema_ids:
                        continue
                    msg, msg_pkg = _resolve_message(
                        index, rpc.response_type, pf.package)
                    if msg is None:
                        continue
                    full_type = rpc.response_type
                    if '.' not in full_type:
                        full_type = f'{msg_pkg}.{msg.name}' if msg_pkg else full_type
                    result.append((array_schema, full_type, False, '', True))
                    seen_schema_ids.add(array_schema)

        return file_base, cpp_base_ns, result

    def _write_codec_plugins(self, output_path: Path, parsed: ProtoFile) -> None:
        file_base, cpp_base_ns, codec_types = self._collect_contract_codec_types(parsed)
        if not file_base or not codec_types:
            return

        json_path = output_path / (file_base + '_json_codec_plugin.cpp')
        self._write_codec_plugin_impl(
            json_path,
            file_base,
            cpp_base_ns,
            'json',
            'application/json',
            codec_types,
        )

        if self._has_backend('flatbuffers'):
            flatbuffers_path = output_path / (
                file_base + '_flatbuffers_codec_plugin.cpp')
            self._write_codec_plugin_impl(
                flatbuffers_path,
                file_base,
                cpp_base_ns,
                'flatbuffers',
                'application/flatbuffers',
                codec_types,
            )

        # Only emit a protobuf codec plugin for service packages that have the
        # local-struct <-> protobuf bridge (the checked-in service protobuf
        # codec). This mirrors the facade's protobuf gating in
        # _service_protobuf_codec_available; today only tactical_objects has it.
        if (self._has_backend('protobuf')
                and self._service_protobuf_codec_available(cpp_base_ns)):
            protobuf_path = output_path / (
                file_base + '_protobuf_codec_plugin.cpp')
            self._write_codec_plugin_impl(
                protobuf_path,
                file_base,
                cpp_base_ns,
                'protobuf',
                'application/protobuf',
                codec_types,
            )

    def _write_codec_plugin_impl(
            self,
            path: Path,
            file_base: str,
            cpp_base_ns: str,
            backend: str,
            content_type: str,
            codec_types: List[Tuple[str, str, bool, str]],
    ) -> None:
        is_json = backend == 'json'
        json_headers = sorted({
            _json_codec_header_for_type(full_type)
            for _short, full_type, is_alias, _alias_cpp, _is_array in codec_types
            if not is_alias and _json_codec_header_for_type(full_type)
        })
        cabi_marshal_headers = sorted({
            '.'.join(full_type.split('.')[:-1]).replace('.', '_')
            + '_cabi_marshal.hpp'
            for _short, full_type, is_alias, _alias_cpp, _is_array in codec_types
            if not is_alias and '.' in full_type
        })
        # flatbuffers and protobuf are both binary backends with the same
        # native<->wire surface (wire_codec::toBinary(native) /
        # fromBinary<Short>(data, size)); only the namespace and header differ.
        if backend == 'protobuf':
            wire_codec_ns = cpp_base_ns + '::protobuf_codec'
            # The protobuf service bridge header ships as source support, which
            # pyramid_protobuf_support exposes as an include dir, so it is
            # included bare (unlike the flatbuffers codec header, which is
            # generated into the build bindings dir under flatbuffers/cpp).
            wire_codec_header = file_base + '_protobuf_codec.hpp'
        else:
            wire_codec_ns = cpp_base_ns + '::flatbuffers_codec'
            wire_codec_header = (
                'flatbuffers/cpp/' + file_base + '_flatbuffers_codec.hpp')

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated PCL codec plugin\n')
            f.write(f'// Backend: {backend} | Content-Type: {content_type}\n\n')
            f.write('#include "pyramid_data_model_types.hpp"\n')
            if is_json:
                for header in json_headers:
                    f.write(f'#include "{header}"\n')
                f.write('#include <nlohmann/json.hpp>\n')
            else:
                f.write(f'#include "{wire_codec_header}"\n')
            # C-ABI marshalling: the plugin receives/returns frozen C structs and
            # marshals to/from the native type before invoking the wire codec.
            for header in cabi_marshal_headers:
                f.write(f'#include "{header}"\n')
            f.write('\n')
            f.write('extern "C" {\n')
            f.write('#include <pcl/pcl_codec.h>\n')
            f.write('#include "pyramid_datamodel_cabi.h"\n')
            f.write('}\n\n')
            f.write('#include <cstdlib>\n')
            f.write('#include <cstring>\n')
            f.write('#include <limits>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write('#if defined(_WIN32)\n')
            f.write('#  define PCL_CODEC_PLUGIN_EXPORT __declspec(dllexport)\n')
            f.write('#elif defined(__GNUC__) || defined(__clang__)\n')
            f.write('#  define PCL_CODEC_PLUGIN_EXPORT __attribute__((visibility("default")))\n')
            f.write('#else\n')
            f.write('#  define PCL_CODEC_PLUGIN_EXPORT\n')
            f.write('#endif\n\n')
            f.write('namespace {\n\n')
            f.write('namespace data_model = pyramid::domain_model;\n')
            if not is_json:
                f.write(f'namespace wire_codec = {wire_codec_ns};\n')
            f.write('\n')
            f.write('pcl_status_t assign_payload(const std::string& payload,\n')
            f.write('                            const char* content_type,\n')
            f.write('                            pcl_msg_t* out_msg)\n')
            f.write('{\n')
            f.write('    if (!out_msg) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    if (payload.size() > std::numeric_limits<uint32_t>::max()) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    void* copy = nullptr;\n')
            f.write('    if (!payload.empty()) {\n')
            f.write('        copy = std::malloc(payload.size());\n')
            f.write('        if (!copy) {\n')
            f.write('            return PCL_ERR_NOMEM;\n')
            f.write('        }\n')
            f.write('        std::memcpy(copy, payload.data(), payload.size());\n')
            f.write('    }\n')
            f.write('    out_msg->data = copy;\n')
            f.write('    out_msg->size = static_cast<uint32_t>(payload.size());\n')
            f.write('    out_msg->type_name = content_type;\n')
            f.write('    return PCL_OK;\n')
            f.write('}\n\n')
            if is_json:
                f.write('template <class T>\n')
                f.write('std::string scalar_to_json(const T& value)\n')
                f.write('{\n')
                f.write('    return nlohmann::json(value).dump();\n')
                f.write('}\n\n')
                f.write('template <class T>\n')
                f.write('T scalar_from_json(const std::string& payload)\n')
                f.write('{\n')
                f.write('    return nlohmann::json::parse(payload).get<T>();\n')
                f.write('}\n\n')
            f.write('} // namespace\n\n')

            f.write('extern "C" {\n\n')
            f.write('static pcl_status_t plugin_encode(void*       codec_ctx,\n')
            f.write('                                  const char* schema_id,\n')
            f.write('                                  const void* value,\n')
            f.write('                                  pcl_msg_t*  out_msg)\n')
            f.write('{\n')
            f.write('    (void)codec_ctx;\n')
            f.write('    if (!schema_id || !value || !out_msg) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    try {\n')
            for short, full_type, is_alias, _alias_cpp, is_array in codec_types:
                if is_alias:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    if _alias_cpp == 'std::string':
                        f.write('            const auto* cs = static_cast<const pyramid_str_t*>(value);\n')
                        f.write('            const char* data = cs && cs->ptr ? cs->ptr : "";\n')
                        f.write(f'            {cpp_type} native(data, cs ? cs->len : 0u);\n')
                    else:
                        f.write(f'            const auto* native = static_cast<const {cpp_type}*>(value);\n')
                        f.write('            if (!native) {\n')
                        f.write('                return PCL_ERR_INVALID;\n')
                        f.write('            }\n')
                    if is_json:
                        if _alias_cpp == 'std::string':
                            f.write('            return assign_payload(scalar_to_json(native),\n')
                        else:
                            f.write('            return assign_payload(scalar_to_json(*native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    else:
                        if _alias_cpp == 'std::string':
                            f.write('            return assign_payload(wire_codec::toBinary(native),\n')
                        else:
                            f.write('            return assign_payload(wire_codec::toBinary(*native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    f.write('        }\n')
                elif is_array:
                    elem_short = short[:-len('Array')]
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{elem_short}'
                    c_struct = _c_struct_for_type(full_type)
                    codec_ns = _json_codec_namespace_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write('            const auto* slice = static_cast<const pyramid_slice_t*>(value);\n')
                    f.write('            if (!slice || (!slice->ptr && slice->len != 0u)) {\n')
                    f.write('                return PCL_ERR_INVALID;\n')
                    f.write('            }\n')
                    f.write(f'            const auto* values = static_cast<const {c_struct}*>(slice->ptr);\n')
                    f.write(f'            std::vector<{cpp_type}> native;\n')
                    f.write('            native.reserve(slice->len);\n')
                    f.write('            for (uint32_t i = 0; i < slice->len; ++i) {\n')
                    f.write(f'                {cpp_type} item;\n')
                    f.write('                pyramid::cabi::from_c(&values[i], item);\n')
                    f.write('                native.push_back(std::move(item));\n')
                    f.write('            }\n')
                    if is_json:
                        f.write('            nlohmann::json arr = nlohmann::json::array();\n')
                        f.write('            for (const auto& item : native) {\n')
                        f.write(f'                arr.push_back(nlohmann::json::parse({codec_ns}::toJson(item)));\n')
                        f.write('            }\n')
                        f.write('            return assign_payload(arr.dump(),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    else:
                        f.write('            return assign_payload(wire_codec::toBinary(native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    f.write('        }\n')
                else:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    c_struct = _c_struct_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            const auto* cs = static_cast<const {c_struct}*>(value);\n')
                    f.write(f'            {cpp_type} native;\n')
                    f.write('            pyramid::cabi::from_c(cs, native);\n')
                    if is_json:
                        codec_ns = _json_codec_namespace_for_type(full_type)
                        f.write(f'            return assign_payload({codec_ns}::toJson(native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    else:
                        f.write('            return assign_payload(wire_codec::toBinary(native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    f.write('        }\n')
            f.write('    } catch (...) {\n')
            f.write('        return PCL_ERR_CALLBACK;\n')
            f.write('    }\n')
            f.write('    return PCL_ERR_NOT_FOUND;\n')
            f.write('}\n\n')

            f.write('static pcl_status_t plugin_decode(void*            codec_ctx,\n')
            f.write('                                  const char*      schema_id,\n')
            f.write('                                  const pcl_msg_t* msg,\n')
            f.write('                                  void*            out_value)\n')
            f.write('{\n')
            f.write('    (void)codec_ctx;\n')
            f.write('    if (!schema_id || !msg || (!msg->data && msg->size != 0) || !out_value) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    try {\n')
            f.write('        const std::string payload = msg->data\n')
            f.write('            ? std::string(static_cast<const char*>(msg->data), msg->size)\n')
            f.write('            : std::string();\n')
            for short, full_type, is_alias, _alias_cpp, is_array in codec_types:
                if is_alias:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            {cpp_type} native;\n')
                    if is_json:
                        f.write(f'            native = scalar_from_json<{cpp_type}>(payload);\n')
                    else:
                        f.write(f'            native = wire_codec::fromBinary{short}(\n')
                        f.write('                msg->data, msg->size);\n')
                    if _alias_cpp == 'std::string':
                        f.write('            auto* cs = static_cast<pyramid_str_t*>(out_value);\n')
                        f.write('            cs->ptr = nullptr;\n')
                        f.write('            cs->len = 0u;\n')
                        f.write('            if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                        f.write('                return PCL_ERR_INVALID;\n')
                        f.write('            }\n')
                        f.write('            if (!native.empty()) {\n')
                        f.write('                void* copy = std::malloc(native.size());\n')
                        f.write('                if (!copy) {\n')
                        f.write('                    return PCL_ERR_NOMEM;\n')
                        f.write('                }\n')
                        f.write('                std::memcpy(copy, native.data(), native.size());\n')
                        f.write('                cs->ptr = static_cast<const char*>(copy);\n')
                        f.write('                cs->len = static_cast<uint32_t>(native.size());\n')
                        f.write('            }\n')
                    else:
                        f.write(f'            *static_cast<{cpp_type}*>(out_value) = native;\n')
                    f.write('            return PCL_OK;\n')
                    f.write('        }\n')
                elif is_array:
                    elem_short = short[:-len('Array')]
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{elem_short}'
                    c_struct = _c_struct_for_type(full_type)
                    codec_ns = _json_codec_namespace_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            std::vector<{cpp_type}> native;\n')
                    if is_json:
                        f.write('            const auto arr = nlohmann::json::parse(payload);\n')
                        f.write('            if (!arr.is_array()) {\n')
                        f.write('                return PCL_ERR_INVALID;\n')
                        f.write('            }\n')
                        f.write('            native.reserve(arr.size());\n')
                        f.write('            for (const auto& item : arr) {\n')
                        f.write(f'                native.push_back({codec_ns}::fromJson(\n')
                        f.write(f'                    item.dump(), static_cast<{cpp_type}*>(nullptr)));\n')
                        f.write('            }\n')
                    else:
                        f.write(f'            native = wire_codec::fromBinary{short}(\n')
                        f.write('                msg->data, msg->size);\n')
                    f.write('            if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                    f.write('                return PCL_ERR_INVALID;\n')
                    f.write('            }\n')
                    f.write('            auto* slice = static_cast<pyramid_slice_t*>(out_value);\n')
                    f.write('            slice->ptr = nullptr;\n')
                    f.write('            slice->len = 0u;\n')
                    f.write('            if (!native.empty()) {\n')
                    f.write(f'                auto* values = static_cast<{c_struct}*>(\n')
                    f.write(f'                    std::calloc(native.size(), sizeof({c_struct})));\n')
                    f.write('                if (!values) {\n')
                    f.write('                    return PCL_ERR_NOMEM;\n')
                    f.write('                }\n')
                    f.write('                for (std::size_t i = 0; i < native.size(); ++i) {\n')
                    f.write('                    pyramid::cabi::to_c(native[i], &values[i]);\n')
                    f.write('                }\n')
                    f.write('                slice->ptr = values;\n')
                    f.write('                slice->len = static_cast<uint32_t>(native.size());\n')
                    f.write('            }\n')
                    f.write('            return PCL_OK;\n')
                    f.write('        }\n')
                else:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    c_struct = _c_struct_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            {cpp_type} native;\n')
                    if is_json:
                        codec_ns = _json_codec_namespace_for_type(full_type)
                        f.write(f'            native = {codec_ns}::fromJson(\n')
                        f.write(f'                payload, static_cast<{cpp_type}*>(nullptr));\n')
                    else:
                        f.write(f'            native = wire_codec::fromBinary{short}(\n')
                        f.write('                msg->data, msg->size);\n')
                    f.write(f'            auto* cs = static_cast<{c_struct}*>(out_value);\n')
                    f.write('            pyramid::cabi::to_c(native, cs);\n')
                    f.write('            return PCL_OK;\n')
                    f.write('        }\n')
            f.write('    } catch (...) {\n')
            f.write('        return PCL_ERR_CALLBACK;\n')
            f.write('    }\n')
            f.write('    return PCL_ERR_NOT_FOUND;\n')
            f.write('}\n\n')

            f.write('static void plugin_free_msg(void* codec_ctx, pcl_msg_t* msg)\n')
            f.write('{\n')
            f.write('    (void)codec_ctx;\n')
            f.write('    if (!msg) {\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    std::free(const_cast<void*>(msg->data));\n')
            f.write('    msg->data = nullptr;\n')
            f.write('    msg->size = 0u;\n')
            f.write('    msg->type_name = nullptr;\n')
            f.write('}\n\n')

            f.write('static pcl_codec_t k_codec = {\n')
            f.write('    PCL_CODEC_ABI_VERSION,\n')
            f.write(f'    "{content_type}",\n')
            f.write('    plugin_encode,\n')
            f.write('    plugin_decode,\n')
            f.write('    plugin_free_msg,\n')
            f.write('    nullptr\n')
            f.write('};\n\n')
            f.write('// Opaque, plugin-specific configuration threaded through the loader.\n')
            f.write('// Stored here and exposed via codec_ctx so encode/decode can honor it.\n')
            f.write('static std::string k_config_json;\n\n')
            f.write('PCL_CODEC_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(\n')
            f.write('    const char* config_json)\n')
            f.write('{\n')
            f.write('    k_config_json = config_json ? config_json : "";\n')
            f.write('    k_codec.codec_ctx = k_config_json.empty() ? nullptr : &k_config_json;\n')
            f.write('    return &k_codec;\n')
            f.write('}\n\n')
            f.write('} // extern "C"\n')

    # -- Header (.hpp) ---------------------------------------------------------

    def _write_header(self, path: Path, full_ns: str, types_ns: str,
                      types_header: str, parsed: ProtoFile,
                      all_rpcs: List[Tuple[str, ProtoRpc]]):
        is_provided = _is_provided(parsed)
        has_grpc = self._has_backend('grpc')
        has_ros2 = self._has_backend('ros2')
        legacy_svc_base_ns, _legacy_prefix = _legacy_service_namespace(parsed.package)
        legacy_role = parsed.package.split('.')[-1] if parsed.package else 'provided'
        legacy_full_ns = legacy_svc_base_ns + '::' + legacy_role
        sub_topics, pub_topics = _topics_for_proto(parsed, is_provided)
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)
        topic_set = all_topics
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)

        # Collect raw type names (no BASE_TYPE_MAP) for using declarations
        # Map each referenced short type name to its proto FQN so the using
        # declarations can be qualified by the type's real (sub-)namespace
        # instead of relying on a flat re-export.
        raw_type_fqn: Dict[str, str] = {}
        for _, rpc in all_rpcs:
            raw_type_fqn.setdefault(_short_type(rpc.req), rpc.req)
            raw_type_fqn.setdefault(_short_type(rpc.rsp), rpc.rsp)
        for key in topic_set:
            spec = topic_spec(key)
            raw_type_fqn.setdefault(spec.short_type, spec.full_type)
        raw_types = sorted(raw_type_fqn)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            # File-level comment block
            f.write('// Auto-generated service binding header\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n')
            f.write('//\n')
            f.write('// Architecture: component logic > service binding (this) > PCL\n')
            f.write('//\n')
            f.write('// This header provides:\n')
            f.write('//   1. Wire-name constants and topic constants\n')
            f.write('//   2. EntityActions handler base class'
                    ' (ServiceHandler \u2014 override Handle*)\n')
            f.write('//   3. PCL binding functions (subscribe*, publish*, invoke*)\n')
            f.write('//   4. Content-type support metadata and msgToString utility\n')
            f.write('#pragma once\n\n')

            # Includes
            f.write(f'#include "{types_header}"\n')
            # Service-local wrapper messages (and Empty) live in this contract's
            # own component types header.
            has_local_types = any(
                _qualified_package_for_type(raw_type_fqn.get(t, '')) == parsed.package
                or raw_type_fqn.get(t, '') == 'google.protobuf.Empty'
                for t in raw_types
            )
            if has_local_types:
                f.write(f'#include "{parsed.package.replace(".", "_")}_types.hpp"\n')
            f.write('\n')
            f.write('#include <pcl/pcl_container.h>\n')
            f.write('#include <pcl/pcl_executor.h>\n')
            f.write('#include <pcl/pcl_transport.h>\n')
            f.write('#include <pcl/pcl_types.h>\n\n')
            if has_ros2:
                f.write('#include "ros2/cpp/pyramid_ros2_transport_support.hpp"\n\n')
            if has_grpc:
                f.write('#include <memory>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')

            # Namespace open
            f.write(f'namespace {full_ns} {{\n\n')

            # ---- Content-type constants --------------------------------------
            f.write(_SEP + '\n')
            f.write('// Content-type constants and support metadata\n')
            f.write(_SEP + '\n\n')
            f.write('constexpr const char* kJsonContentType = "application/json";\n')
            if self._has_backend('flatbuffers'):
                f.write('constexpr const char* kFlatBuffersContentType = "application/flatbuffers";\n')
            if self._has_backend('protobuf') or has_grpc:
                f.write('constexpr const char* kProtobufContentType = "application/protobuf";\n')
            f.write('\n')
            f.write('bool supportsContentType(const char* content_type);\n')
            f.write('std::vector<const char*> supportedContentTypes();\n\n')

            # ---- Service wire-name constants ----------------------------------
            f.write(_SEP + '\n')
            f.write('// Service wire-name constants (generated from proto)\n')
            f.write(_SEP + '\n\n')

            # Align '=' across all constant declarations
            const_names = [
                _rpc_service_const(svc.name, rpc, duplicate_rpc_names)
                for svc in parsed.services
                for rpc in svc.rpcs
            ]
            max_const = max((len(n) for n in const_names), default=0)

            for svc in parsed.services:
                for rpc in svc.rpcs:
                    wire = f'{svc.wire_prefix}.{rpc.wire_name}'
                    service_const = _rpc_service_const(
                        svc.name, rpc, duplicate_rpc_names)
                    pad = max_const - len(service_const)
                    f.write(f'constexpr const char* {service_const}'
                            f'{" " * pad}  = "{wire}";\n')
            f.write('\n')

            # ---- Topic constants ---------------------------------------------
            if topic_set:
                f.write(_SEP + '\n')
                f.write('// Standard topic name constants\n')
                f.write(_SEP + '\n\n')
                topic_consts = [f'kTopic{_snake_to_pascal(k)}' for k in topic_set]
                max_topic = max((len(n) for n in topic_consts), default=0)
                # Only pad when there are multiple constants to align
                multi = len(topic_set) > 1
                for key, wire in topic_set.items():
                    cname = f'kTopic{_snake_to_pascal(key)}'
                    if multi:
                        pad = max_topic - len(cname)
                        f.write(f'constexpr const char* {cname}'
                                f'{" " * pad}  = "{wire}";\n')
                    else:
                        f.write(f'constexpr const char* {cname} = "{wire}";\n')
                f.write('\n')

            # ---- ServiceChannel enum -----------------------------------------
            f.write(_SEP + '\n')
            f.write('// Service channel discriminant\n')
            f.write(_SEP + '\n\n')
            f.write('enum class ServiceChannel {\n')
            for svc_name, rpc in all_rpcs:
                f.write(
                    f'    {_rpc_enum_value(svc_name, rpc, duplicate_rpc_names)},\n')
            f.write('};\n\n')

            # ---- msgToString -------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// PCL message utility\n')
            f.write(_SEP + '\n\n')
            f.write('/// \\brief Convert a raw PCL message buffer to a std::string.\n')
            f.write('std::string msgToString(const void* data, unsigned size);\n\n')

            # ---- ServiceHandler base class -----------------------------------
            f.write(_SEP + '\n')
            f.write('// EntityActions handler base class\n')
            f.write('//\n')
            f.write('// Subclass and override the handle* methods to implement'
                    ' business logic.\n')
            f.write('// Default implementations return empty / null values'
                    ' (stub behaviour).\n')
            f.write(_SEP + '\n\n')

            # Data-model RPC/topic types are imported from their package
            # sub-namespace. Service-local wrapper messages and Empty are
            # defined in this contract's own namespace (== full_ns, supplied by
            # the included component types header) and need no using-declaration.
            for t in raw_types:
                fqn = raw_type_fqn.get(t, '')
                pkg = _qualified_package_for_type(fqn)
                if (pkg == _DATA_MODEL_PROTO_ROOT
                        or pkg.startswith(_DATA_MODEL_PROTO_ROOT + '.')):
                    f.write(f'using {_cpp_ns_for_proto_package(pkg)}::{t};\n')
            f.write('\n')

            f.write('class ServiceHandler {\n')
            f.write('public:\n')
            f.write('    virtual ~ServiceHandler() = default;\n')

            current_svc = None
            for i, (svc_name, rpc) in enumerate(all_rpcs):
                if svc_name != current_svc:
                    f.write(f'\n    // {svc_name}\n')
                    current_svc = svc_name
                f.write(f'    virtual {rpc.cpp_rsp_type}\n')
                handler_name = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                f.write(
                    f'    {handler_name}(const {rpc.cpp_req_type}& request);\n')
                if rpc.streaming:
                    stream_name = _rpc_stream_handler_name(
                        svc_name, rpc, duplicate_rpc_names)
                    f.write('\n')
                    f.write('    /// \\brief Begin an asynchronous stream for this RPC.\n')
                    f.write('    ///\n')
                    f.write('    /// Override this for true server streaming. Store stream_context,\n')
                    f.write('    /// return PCL_STREAMING, then emit frames with send*StreamFrame().\n')
                    f.write('    virtual pcl_status_t\n')
                    f.write(f'    {stream_name}(const {rpc.cpp_req_type}& request,\n')
                    f.write('    ' + ' ' * len(stream_name)
                            + ' pcl_stream_context_t* stream_context,\n')
                    f.write('    ' + ' ' * len(stream_name)
                            + ' const char* content_type);\n')
                # Blank line after method unless: last method, or next method
                # is a new service section (its leading \n already provides spacing)
                is_last = i == len(all_rpcs) - 1
                next_svc_change = (not is_last
                                   and all_rpcs[i + 1][0] != svc_name)
                if not is_last and not next_svc_change:
                    f.write('\n')

            f.write('};\n\n')

            # ---- PCL binding functions ---------------------------------------
            f.write(_SEP + '\n')
            f.write('// PCL binding functions \u2014'
                    ' Subscribe / Publish / Invoke (typed)\n')
            f.write(_SEP + '\n\n')

            # Subscribe helpers (unchanged -- PCL registration is content_type
            # aware but not typed, since the callback receives raw pcl_msg_t)
            for key, _wire in topic_set.items():
                pascal = _snake_to_pascal(key)
                fname = f'subscribe{pascal}'
                cname = f'kTopic{pascal}'
                col = len(f'pcl_port_t* {fname}(')
                sp = ' ' * col
                brief = (f'/// \\brief Subscribe to'
                         f' {_topic_key_to_phrase(key)} publications on'
                         f' {cname}.')
                if len(brief) > 80:
                    split = brief.rfind(f' {cname}.')
                    f.write(brief[:split] + '\n')
                    f.write(f'///        {cname}.\n')
                else:
                    f.write(brief + '\n')
                f.write(f'pcl_port_t* {fname}(pcl_container_t*  container,\n')
                f.write(f'{sp}pcl_sub_callback_t callback,\n')
                f.write(f'{sp}void*             user_data = nullptr,\n')
                f.write(f'{sp}const char*       content_type'
                        f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')

            # Typed publish helpers.  These are generated for every topic in
            # both provider and consumer namespaces so components on either side
            # can use the generated binding as their only codec facade.
            for key, _wire in topic_set.items():
                pascal = _snake_to_pascal(key)
                fname = f'publish{pascal}'
                cname = f'kTopic{pascal}'
                col = 13 + len(fname) + 1
                sp = ' ' * col
                spec = topic_spec(key)
                wire_decl_t = spec.cpp_payload_type
                f.write(f'/// \\brief Publish a typed message on'
                        f' {cname}.\n')
                f.write('///\n')
                f.write(f'/// \\p publisher must be the pcl_port_t*'
                        f' returned by addPublisher for\n')
                f.write(f'/// {cname}, obtained during on_configure.\n')
                f.write(f'pcl_status_t {fname}'
                        f'(pcl_port_t*        publisher,\n')
                f.write(f'{sp}const {wire_decl_t}& payload,\n')
                f.write(f'{sp}const char*        content_type'
                        f' = "{_DEFAULT_CONTENT_TYPE}");\n')
                f.write('\n')
                f.write(f'pcl_status_t {fname}'
                        f'(pcl_port_t*        publisher,\n')
                f.write(f'{sp}const std::string& payload,\n')
                f.write(f'{sp}const char*'
                        f'        content_type'
                        f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')

                encode_name = f'encode{pascal}'
                col = len(f'bool {encode_name}(')
                sp = ' ' * col
                f.write(f'/// \\brief Encode a typed message for {cname}.\n')
                f.write(f'bool {encode_name}(const {wire_decl_t}& payload,\n')
                f.write(f'{sp}const char*        content_type,\n')
                f.write(f'{sp}std::string*       out);\n\n')

                decode_name = f'decode{pascal}'
                col = len(f'bool {decode_name}(')
                sp = ' ' * col
                f.write(f'/// \\brief Decode a PCL message from {cname}.\n')
                f.write(f'bool {decode_name}(const pcl_msg_t* msg,\n')
                f.write(f'{sp}{wire_decl_t}* out);\n\n')

            # Typed invoke helpers
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    wire_full = f'{svc.wire_prefix}.{rpc.wire_name}'
                    rsp_decl_t = rpc.cpp_rsp_type
                    decode_func = _rpc_decode_response_func(
                        svc.name, rpc, duplicate_rpc_names)
                    invoke_func = _rpc_invoke_func(
                        svc.name, rpc, duplicate_rpc_names)
                    decode_col = len(f'bool {decode_func}(')
                    decode_sp = ' ' * decode_col
                    f.write(f'/// \\brief Decode a response from {wire_full}.\n')
                    f.write(f'bool {decode_func}'
                            f'(const pcl_msg_t* msg,\n')
                    f.write(f'{decode_sp}{rsp_decl_t}* out);\n\n')
                    if rpc.streaming:
                        frame_t = rpc.cpp_rsp_type[len('std::vector<'):-1]
                        encode_frame = _rpc_encode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        decode_frame = _rpc_decode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        send_frame = _rpc_send_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        f.write(f'/// \\brief Encode one stream frame for {wire_full}.\n')
                        col = len(f'bool {encode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {encode_frame}(const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type,\n')
                        f.write(f'{sp}std::string*       out);\n\n')
                        f.write(f'/// \\brief Decode one stream frame from {wire_full}.\n')
                        col = len(f'bool {decode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {decode_frame}(const pcl_msg_t* msg,\n')
                        f.write(f'{sp}{frame_t}* out);\n\n')
                        f.write(f'/// \\brief Send one typed stream frame for {wire_full}.\n')
                        col = 13 + len(send_frame) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {send_frame}'
                                f'(pcl_stream_context_t* stream_context,\n')
                        f.write(f'{sp}const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type'
                                f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')
                    req_decl_t = rpc.cpp_req_type
                    col = 13 + len(invoke_func) + 1
                    sp = ' ' * col
                    f.write(f'/// \\brief Invoke {wire_full}'
                            f' (typed, serialisation handled internally).\n')
                    f.write(f'///\n')
                    f.write(f'/// Uses the configured endpoint route, or the legacy\n')
                    f.write(f'/// executor transport fallback when no route is supplied.\n')
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}pcl_resp_cb_fn_t'
                            f'        callback,\n')
                    f.write(f'{sp}void*'
                            f'                   user_data'
                            f' = nullptr,\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route'
                            f' = nullptr,\n')
                    f.write(f'{sp}const char*       content_type'
                            f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')
                    f.write(f'/// \\brief Invoke {wire_full} and ignore the async response.\n')
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}const char*       content_type'
                            f' = "{_DEFAULT_CONTENT_TYPE}",\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route'
                            f' = nullptr);\n\n')
                    if rpc.streaming:
                        invoke_stream = _rpc_invoke_stream_func(
                            svc.name, rpc, duplicate_rpc_names)
                        col = 13 + len(invoke_stream) + 1
                        sp = ' ' * col
                        f.write(f'/// \\brief Invoke {wire_full} as an asynchronous stream.\n')
                        f.write(f'pcl_status_t {invoke_stream}'
                                f'(pcl_executor_t* executor,\n')
                        f.write(f'{sp}const {req_decl_t}&'
                                f'{" " * max(1, 22 - len(req_decl_t))}'
                                f'request,\n')
                        f.write(f'{sp}pcl_stream_msg_fn_t'
                                f'   callback,\n')
                        f.write(f'{sp}void*'
                                f'                   user_data'
                                f' = nullptr,\n')
                        f.write(f'{sp}pcl_stream_context_t** out_context'
                                f' = nullptr,\n')
                        f.write(f'{sp}const pcl_endpoint_route_t* route'
                                f' = nullptr,\n')
                        f.write(f'{sp}const char*       content_type'
                                f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')

            if has_grpc:
                f.write(_SEP + '\n')
                f.write('// gRPC binding startup hook\n')
                f.write(_SEP + '\n\n')
                f.write('class GrpcServer {\n')
                f.write('public:\n')
                f.write('    GrpcServer();\n')
                f.write('    GrpcServer(GrpcServer&&) noexcept;\n')
                f.write('    GrpcServer& operator=(GrpcServer&&) noexcept;\n')
                f.write('    GrpcServer(const GrpcServer&) = delete;\n')
                f.write('    GrpcServer& operator=(const GrpcServer&) = delete;\n')
                f.write('    ~GrpcServer();\n\n')
                f.write('    bool started() const;\n')
                f.write('    explicit operator bool() const { return started(); }\n')
                f.write('    void wait();\n')
                f.write('    void shutdown();\n\n')
                f.write('private:\n')
                f.write('    struct Impl;\n')
                f.write('    explicit GrpcServer(std::unique_ptr<Impl> impl);\n')
                f.write('    std::unique_ptr<Impl> impl_;\n')
                f.write('    friend GrpcServer buildGrpcServer(const std::string& listen_address,\n')
                f.write('                                      pcl_executor_t* executor);\n')
                f.write('};\n\n')
                f.write('/// \\brief Start generated gRPC ingress endpoints on a PCL executor.\n')
                f.write('GrpcServer buildGrpcServer(const std::string& listen_address,\n')
                f.write('                           pcl_executor_t* executor);\n\n')

            if has_ros2:
                f.write(_SEP + '\n')
                f.write('// ROS2 binding startup hook\n')
                f.write(_SEP + '\n\n')
                f.write('/// \\brief Bind generated ROS2 ingress endpoints to the executor.\n')
                f.write('inline void bindRos2(pyramid::transport::ros2::Adapter& adapter,\n')
                f.write('                     pcl_executor_t* executor)\n')
                f.write('{\n')
                if sub_topics:
                    for key in sub_topics:
                        cname = f'kTopic{_snake_to_pascal(key)}'
                        f.write('    pyramid::transport::ros2::bindTopicIngress'
                                f'(adapter, executor, {cname});\n')
                for svc in parsed.services:
                    for rpc in svc.rpcs:
                        bind_func = ('bindStreamServiceIngress'
                                     if rpc.streaming
                                     else 'bindUnaryServiceIngress')
                        service_const = _rpc_service_const(
                            svc.name, rpc, duplicate_rpc_names)
                        f.write(f'    pyramid::transport::ros2::{bind_func}'
                                f'(adapter, executor, {service_const});\n')
                f.write('}\n\n')

            # ---- dispatch() --------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Dispatch -- deserialises request, calls handler,'
                    ' serialises response.\n')
            f.write('//\n')
            f.write('// Response buffer is heap-allocated via std::malloc;'
                    ' caller frees with std::free.\n')
            f.write(_SEP + '\n\n')
            f.write('void dispatch(ServiceHandler& handler,\n')
            f.write('              ServiceChannel  channel,\n')
            f.write('              const void*     request_buf,\n')
            f.write('              size_t          request_size,\n')
            f.write('              const char*     content_type,\n')
            f.write('              void**          response_buf,\n')
            f.write('              size_t*         response_size);\n\n')
            f.write('inline void dispatch(ServiceHandler& handler,\n')
            f.write('                     ServiceChannel  channel,\n')
            f.write('                     const void*     request_buf,\n')
            f.write('                     size_t          request_size,\n')
            f.write('                     void**          response_buf,\n')
            f.write('                     size_t*         response_size)\n')
            f.write('{\n')
            f.write(f'    dispatch(handler, channel, request_buf, request_size, "{_DEFAULT_CONTENT_TYPE}", response_buf, response_size);\n')
            f.write('}\n\n')
            f.write('/// \\brief Dispatch a server-streaming service request.\n')
            f.write('pcl_status_t dispatchStream(ServiceHandler& handler,\n')
            f.write('                            ServiceChannel  channel,\n')
            f.write('                            const void*     request_buf,\n')
            f.write('                            size_t          request_size,\n')
            f.write('                            const char*     content_type,\n')
            f.write('                            pcl_stream_context_t* stream_context);\n\n')
            f.write('inline pcl_status_t dispatchStream(ServiceHandler& handler,\n')
            f.write('                                   ServiceChannel  channel,\n')
            f.write('                                   const void*     request_buf,\n')
            f.write('                                   size_t          request_size,\n')
            f.write('                                   pcl_stream_context_t* stream_context)\n')
            f.write('{\n')
            f.write(f'    return dispatchStream(handler, channel, request_buf, request_size, "{_DEFAULT_CONTENT_TYPE}", stream_context);\n')
            f.write('}\n\n')

            # Namespace close
            f.write(f'}} // namespace {full_ns}\n')
            if legacy_full_ns != full_ns:
                legacy_parts = legacy_full_ns.split('::')
                alias_name = legacy_parts[-1]
                alias_parent = '::'.join(legacy_parts[:-1])
                f.write('\n')
                f.write('// Legacy service namespace alias for existing callers.\n')
                f.write(f'namespace {alias_parent} {{\n')
                f.write(f'namespace {alias_name} = ::{full_ns};\n')
                f.write(f'}} // namespace {alias_parent}\n')

    # -- Implementation (.cpp) -------------------------------------------------

    def _write_impl(self, path: Path, file_prefix: str, full_ns: str,
                    types_ns: str, parsed: ProtoFile,
                    all_rpcs: List[Tuple[str, ProtoRpc]]):
        is_provided = _is_provided(parsed)
        svc_base_ns = _namespace_from_proto(parsed)[2]
        has_flatbuffers = self._has_backend('flatbuffers')
        has_protobuf = (
            (self._has_backend('protobuf') or self._has_backend('grpc'))
            and self._service_protobuf_codec_available(svc_base_ns)
        )
        flatbuffers_codec_ns = svc_base_ns + '::flatbuffers_codec'
        flatbuffers_codec_header = 'flatbuffers/cpp/' + '_'.join(svc_base_ns.split('::')) + '_flatbuffers_codec.hpp'
        protobuf_codec_ns = svc_base_ns + '::protobuf_codec'
        protobuf_codec_header = '_'.join(svc_base_ns.split('::')) + '_protobuf_codec.hpp'
        sub_topics, pub_topics = _topics_for_proto(parsed, is_provided)
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)
        topic_set = all_topics
        hpp_name = file_prefix + '.hpp'
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)

        data_model_files = self._discover_data_model_proto_files()
        codec_imports = _service_codec_imports(
            data_model_files,
            all_rpcs,
            list(topic_set.keys()),
        )
        dm_codec_nss = [ns for ns, _header in codec_imports]
        dm_codec_headers = [header for _ns, header in codec_imports]

        # C-ABI typed-value boundary: schema types this contract marshals
        # through the runtime codec registry, and the cabi marshalling headers
        # for the data-model modules in their dependency closure.
        _cc_base, _cc_ns, cabi_codec_types = \
            self._collect_contract_codec_types(parsed)
        cabi_marshal_headers = sorted({
            '.'.join(full_type.split('.')[:-1]).replace('.', '_')
            + '_cabi_marshal.hpp'
            for _short, full_type, is_alias, _alias_cpp, _is_array in cabi_codec_types
            if not is_alias and '.' in full_type
        })

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            # File-level comment block
            f.write('// Auto-generated service binding implementation\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n\n')

            # Includes
            f.write(f'#include "{hpp_name}"\n\n')
            # C-ABI marshalling for the typed-value plugin boundary
            for ch in cabi_marshal_headers:
                f.write(f'#include "{ch}"\n')
            f.write('\n')
            f.write('extern "C" {\n')
            f.write('#include <pcl/pcl_codec.h>\n')
            f.write('#include <pcl/pcl_codec_registry.h>\n')
            f.write('#include <pcl/pcl_container.h>\n')
            f.write('#include <pcl/pcl_executor.h>\n')
            f.write('#include <pcl/pcl_transport.h>\n')
            f.write('#include "pyramid_datamodel_cabi.h"\n')
            f.write('}\n')
            f.write('\n#include <cstdlib>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <cstring>\n')
            f.write('#include <limits>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')

            # Namespace open
            f.write(f'namespace {full_ns} {{\n\n')

            f.write('\n')

            # ---- msgToString -------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// PCL message utility\n')
            f.write(_SEP + '\n\n')
            f.write('std::string msgToString(const void* data, unsigned size) {\n')
            f.write('    return std::string(static_cast<const char*>(data), size);\n')
            f.write('}\n\n')

            # ---- ServiceHandler stubs ----------------------------------------
            f.write(_SEP + '\n')
            f.write('// ServiceHandler \u2014 default stub implementations\n')
            f.write(_SEP + '\n\n')

            for svc_name, rpc in all_rpcs:
                rsp_t = rpc.cpp_rsp_type
                req_t = rpc.cpp_req_type
                handler_name = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)

                f.write(f'{rsp_t}\n')
                f.write(f'ServiceHandler::{handler_name}'
                        f'(const {req_t}& /*request*/) {{\n')

                if rsp_t == 'Ack':
                    f.write(f'    return {types_ns}::kAckOk;\n')
                else:
                    f.write('    return {};\n')

                f.write('}\n\n')

                if rpc.streaming:
                    stream_name = _rpc_stream_handler_name(
                        svc_name, rpc, duplicate_rpc_names)
                    f.write('pcl_status_t\n')
                    f.write(f'ServiceHandler::{stream_name}'
                            f'(const {req_t}& /*request*/,\n')
                    f.write('    pcl_stream_context_t* /*stream_context*/,\n')
                    f.write('    const char* /*content_type*/) {\n')
                    f.write('    return PCL_ERR_INVALID;\n')
                    f.write('}\n\n')

            # ---- Internal PCL helpers (anonymous namespace) ------------------
            f.write(_SEP + '\n')
            f.write('// Internal PCL helpers\n')
            f.write(_SEP + '\n\n')
            f.write('namespace {\n\n')

            # C-ABI typed-value boundary: marshal the native value to its frozen
            # C struct (to_c), hand the C struct to the registry codec, then free.
            # Decode is symmetric: the plugin fills the C representation, we
            # move it into the native value and free any C-owned storage.
            non_alias_cabi = [
                (short, full_type)
                for short, full_type, is_alias, _alias_cpp, is_array in cabi_codec_types
                if not is_alias and not is_array
            ]
            alias_cabi = [
                (short, _alias_cpp)
                for short, _full_type, is_alias, _alias_cpp, _is_array in cabi_codec_types
                if is_alias
            ]
            array_cabi = [
                (short, full_type)
                for short, full_type, is_alias, _alias_cpp, is_array in cabi_codec_types
                if is_array
            ]

            f.write('static int pyramid_cabi_encode(const pcl_codec_t* c,\n')
            f.write('                               const char* schema_id,\n')
            f.write('                               const void* value,\n')
            f.write('                               pcl_msg_t* out_msg)\n')
            f.write('{\n')
            for short, alias_cpp in alias_cabi:
                native = f'{_DATA_MODEL_TYPES_NS}::{short}'
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                if alias_cpp == 'std::string':
                    f.write(f'        const auto& native = *static_cast<const {native}*>(value);\n')
                    f.write('        if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                    f.write('            return -1;\n')
                    f.write('        }\n')
                    f.write('        pyramid_str_t cs;\n')
                    f.write('        cs.ptr = native.data();\n')
                    f.write('        cs.len = static_cast<uint32_t>(native.size());\n')
                    f.write('        const pcl_status_t rc =\n')
                    f.write('            c->encode(c->codec_ctx, schema_id, &cs, out_msg);\n')
                else:
                    f.write('        const pcl_status_t rc =\n')
                    f.write('            c->encode(c->codec_ctx, schema_id, value, out_msg);\n')
                f.write('        return rc == PCL_OK ? 1 : -1;\n')
                f.write('    }\n')
            for short, _full in non_alias_cabi:
                native = f'{_native_namespace_for_type(_full)}::{short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write(f'        {c_struct} cs;\n')
                f.write(f'        pyramid::cabi::to_c(\n')
                f.write(f'            *static_cast<const {native}*>(value), &cs);\n')
                f.write('        const pcl_status_t rc =\n')
                f.write('            c->encode(c->codec_ctx, schema_id, &cs, out_msg);\n')
                f.write(f'        {c_struct}_free(&cs);\n')
                f.write('        return rc == PCL_OK ? 1 : -1;\n')
                f.write('    }\n')
            for short, _full in array_cabi:
                elem_short = short[:-len('Array')]
                native = f'{_native_namespace_for_type(_full)}::{elem_short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write(f'        const auto& native = *static_cast<const std::vector<{native}>*>(value);\n')
                f.write('        if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                f.write('            return -1;\n')
                f.write('        }\n')
                f.write(f'        std::vector<{c_struct}> values(native.size());\n')
                f.write('        if (!values.empty()) {\n')
                f.write('            std::memset(values.data(), 0, values.size() * sizeof(values[0]));\n')
                f.write('        }\n')
                f.write('        for (std::size_t i = 0; i < native.size(); ++i) {\n')
                f.write('            pyramid::cabi::to_c(native[i], &values[i]);\n')
                f.write('        }\n')
                f.write('        pyramid_slice_t slice;\n')
                f.write('        slice.ptr = values.empty() ? nullptr : values.data();\n')
                f.write('        slice.len = static_cast<uint32_t>(values.size());\n')
                f.write('        const pcl_status_t rc =\n')
                f.write('            c->encode(c->codec_ctx, schema_id, &slice, out_msg);\n')
                f.write('        for (auto& item : values) {\n')
                f.write(f'            {c_struct}_free(&item);\n')
                f.write('        }\n')
                f.write('        return rc == PCL_OK ? 1 : -1;\n')
                f.write('    }\n')
            f.write('    (void)c; (void)value; (void)out_msg;\n')
            f.write('    return 0;\n')
            f.write('}\n\n')

            f.write('static int pyramid_cabi_decode(const pcl_codec_t* c,\n')
            f.write('                               const char* schema_id,\n')
            f.write('                               const pcl_msg_t* msg,\n')
            f.write('                               void* out_value)\n')
            f.write('{\n')
            for short, alias_cpp in alias_cabi:
                native = f'{_DATA_MODEL_TYPES_NS}::{short}'
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                if alias_cpp == 'std::string':
                    f.write('        pyramid_str_t cs;\n')
                    f.write('        std::memset(&cs, 0, sizeof(cs));\n')
                    f.write('        if (c->decode(c->codec_ctx, schema_id, msg, &cs)\n')
                    f.write('                != PCL_OK) {\n')
                    f.write('            return -1;\n')
                    f.write('        }\n')
                    f.write(f'        auto* native = static_cast<{native}*>(out_value);\n')
                    f.write('        native->assign(cs.ptr ? cs.ptr : "", cs.len);\n')
                    f.write('        std::free(const_cast<char*>(cs.ptr));\n')
                else:
                    f.write('        if (c->decode(c->codec_ctx, schema_id, msg, out_value)\n')
                    f.write('                != PCL_OK) {\n')
                    f.write('            return -1;\n')
                    f.write('        }\n')
                f.write('        return 1;\n')
                f.write('    }\n')
            for short, _full in non_alias_cabi:
                native = f'{_native_namespace_for_type(_full)}::{short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write(f'        {c_struct} cs;\n')
                f.write('        std::memset(&cs, 0, sizeof(cs));\n')
                f.write('        if (c->decode(c->codec_ctx, schema_id, msg, &cs)\n')
                f.write('                != PCL_OK) {\n')
                f.write(f'            {c_struct}_free(&cs);\n')
                f.write('            return -1;\n')
                f.write('        }\n')
                f.write(f'        pyramid::cabi::from_c(\n')
                f.write(f'            &cs, *static_cast<{native}*>(out_value));\n')
                f.write(f'        {c_struct}_free(&cs);\n')
                f.write('        return 1;\n')
                f.write('    }\n')
            for short, _full in array_cabi:
                elem_short = short[:-len('Array')]
                native = f'{_native_namespace_for_type(_full)}::{elem_short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write('        pyramid_slice_t slice;\n')
                f.write('        std::memset(&slice, 0, sizeof(slice));\n')
                f.write('        if (c->decode(c->codec_ctx, schema_id, msg, &slice)\n')
                f.write('                != PCL_OK) {\n')
                f.write('            return -1;\n')
                f.write('        }\n')
                f.write(f'        auto* native = static_cast<std::vector<{native}>*>(out_value);\n')
                f.write('        native->clear();\n')
                f.write('        native->reserve(slice.len);\n')
                f.write(f'        auto* values = static_cast<{c_struct}*>(const_cast<void*>(slice.ptr));\n')
                f.write('        for (uint32_t i = 0; i < slice.len; ++i) {\n')
                f.write(f'            {native} item;\n')
                f.write('            pyramid::cabi::from_c(&values[i], item);\n')
                f.write('            native->push_back(std::move(item));\n')
                f.write(f'            {c_struct}_free(&values[i]);\n')
                f.write('        }\n')
                f.write('        std::free(values);\n')
                f.write('        return 1;\n')
                f.write('    }\n')
            f.write('    (void)c; (void)msg; (void)out_value;\n')
            f.write('    return 0;\n')
            f.write('}\n\n')

            # A single process (e.g. a PYRAMID bridge) may load codec plugins for
            # several components, all registered under the same content_type. We
            # iterate the codecs registered for the content_type and try each by
            # schema_id until one handles it; if none do (or the schema is a
            # scalar alias), we return -1 so the caller uses the static codec.
            f.write('static int pyramid_try_registry_encode(const char* content_type,\n')
            f.write('                                       const char* schema_id,\n')
            f.write('                                       const void* value,\n')
            f.write('                                       std::string* out)\n')
            f.write('{\n')
            f.write('    pcl_codec_registry_t* reg = pcl_codec_registry_default();\n')
            f.write('    for (uint32_t i = 0; ; ++i) {\n')
            f.write('        const pcl_codec_t* c =\n')
            f.write('            pcl_codec_registry_get_at(reg, content_type, i);\n')
            f.write('        if (!c) {\n')
            f.write('            break;\n')
            f.write('        }\n')
            f.write('        if (!c->encode) {\n')
            f.write('            continue;\n')
            f.write('        }\n')
            f.write('        pcl_msg_t m;\n')
            f.write('        m.data = nullptr;\n')
            f.write('        m.size = 0;\n')
            f.write('        m.type_name = nullptr;\n')
            f.write('        const int r = pyramid_cabi_encode(c, schema_id, value, &m);\n')
            f.write('        if (r == 1) {\n')
            f.write('            if (m.data && m.size != 0) {\n')
            f.write('                out->assign(static_cast<const char*>(m.data), m.size);\n')
            f.write('            } else {\n')
            f.write('                out->clear();\n')
            f.write('            }\n')
            f.write('            if (c->free_msg) {\n')
            f.write('                c->free_msg(c->codec_ctx, &m);\n')
            f.write('            }\n')
            f.write('            return 1;\n')
            f.write('        }\n')
            f.write('        if (r == 0) {\n')
            f.write('            return -1;\n')
            f.write('        }\n')
            f.write('    }\n')
            f.write('    return -1;\n')
            f.write('}\n\n')

            f.write('static int pyramid_try_registry_decode(const pcl_msg_t* msg,\n')
            f.write('                                       const char* schema_id,\n')
            f.write('                                       void* out_value)\n')
            f.write('{\n')
            f.write('    if (!msg) {\n')
            f.write('        return -1;\n')
            f.write('    }\n')
            f.write('    pcl_codec_registry_t* reg = pcl_codec_registry_default();\n')
            f.write('    for (uint32_t i = 0; ; ++i) {\n')
            f.write('        const pcl_codec_t* c =\n')
            f.write('            pcl_codec_registry_get_at(reg, msg->type_name, i);\n')
            f.write('        if (!c) {\n')
            f.write('            break;\n')
            f.write('        }\n')
            f.write('        if (!c->decode) {\n')
            f.write('            continue;\n')
            f.write('        }\n')
            f.write('        const int r =\n')
            f.write('            pyramid_cabi_decode(c, schema_id, msg, out_value);\n')
            f.write('        if (r == 1) {\n')
            f.write('            return 1;\n')
            f.write('        }\n')
            f.write('        if (r == 0) {\n')
            f.write('            return -1;\n')
            f.write('        }\n')
            f.write('    }\n')
            f.write('    return -1;\n')
            f.write('}\n\n')

            f.write('void ignore_async_response(const pcl_msg_t*, void*) {}\n\n')

            f.write('pcl_status_t invoke_async('
                    'pcl_executor_t* executor,\n')
            f.write('                           const char*'
                    '             service_name,\n')
            f.write('                           const std::string&'
                    '      payload,\n')
            f.write('                           pcl_resp_cb_fn_t'
                    '        callback,\n')
            f.write('                           void*'
                    '                   user_data,\n')
            f.write('                           const pcl_endpoint_route_t*'
                    ' route,\n')
            f.write('                           const char*             content_type)\n')
            f.write('{\n')
            f.write('    pcl_msg_t msg{};\n')
            f.write('    msg.data      = payload.data();\n')
            f.write('    msg.size      = static_cast<uint32_t>'
                    '(payload.size());\n')
            f.write('    msg.type_name = content_type;\n')
            f.write('    if (route) {\n')
            f.write('        const pcl_status_t route_rc ='
                    ' pcl_executor_set_endpoint_route(executor, route);\n')
            f.write('        if (route_rc != PCL_OK) {\n')
            f.write('            return route_rc;\n')
            f.write('        }\n')
            f.write('    }\n')
            f.write('    return pcl_executor_invoke_async(\n')
            f.write('        executor, service_name, &msg,'
                    ' callback, user_data);\n')
            f.write('}\n\n')

            f.write('} // namespace\n\n')

            # ---- Content-type metadata ---------------------------------------
            f.write(_SEP + '\n')
            f.write('// Content-type support metadata\n')
            f.write(_SEP + '\n\n')
            f.write('std::vector<const char*> supportedContentTypes()\n')
            f.write('{\n')
            f.write('    std::vector<const char*> result;\n')
            f.write('    pcl_codec_registry_t* reg = pcl_codec_registry_default();\n')
            f.write('    if (pcl_codec_registry_get(reg, kJsonContentType) != nullptr) {\n')
            f.write('        result.push_back(kJsonContentType);\n')
            f.write('    }\n')
            if has_flatbuffers:
                f.write('    if (pcl_codec_registry_get(reg, kFlatBuffersContentType) != nullptr) {\n')
                f.write('        result.push_back(kFlatBuffersContentType);\n')
                f.write('    }\n')
            if has_protobuf:
                f.write('    if (pcl_codec_registry_get(reg, kProtobufContentType) != nullptr) {\n')
                f.write('        result.push_back(kProtobufContentType);\n')
                f.write('    }\n')
            f.write('    return result;\n')
            f.write('}\n\n')
            f.write('bool supportsContentType(const char* content_type)\n')
            f.write('{\n')
            f.write('    return content_type\n')
            f.write('        && pcl_codec_registry_get(pcl_codec_registry_default(), content_type)\n')
            f.write('            != nullptr;\n')
            f.write('}\n\n')

            # Subscribe wrappers (unchanged)
            n_topics = len(topic_set)
            if n_topics:
                f.write(_SEP + '\n')
                label = 'PCL subscribe wrapper' + ('s' if n_topics > 1 else '')
                f.write(f'// {label}\n')
                f.write(_SEP + '\n\n')

                for key, _wire in topic_set.items():
                    pascal = _snake_to_pascal(key)
                    fname = f'subscribe{pascal}'
                    cname = f'kTopic{pascal}'
                    col = len(f'pcl_port_t* {fname}(')
                    sp = ' ' * col
                    f.write(f'pcl_port_t* {fname}(pcl_container_t*   container,\n')
                    f.write(f'{sp}pcl_sub_callback_t  callback,\n')
                    f.write(f'{sp}void*              user_data,\n')
                    f.write(f'{sp}const char*        content_type)\n')
                    f.write('{\n')
                    f.write('    return pcl_container_add_subscriber(container,\n')
                    f.write(f'                                 {cname},\n')
                    f.write('                                 content_type,\n')
                    f.write('                                 callback,\n')
                    f.write('                                 user_data);\n')
                    f.write('}\n\n')

            # Typed response decoders -- hide content-type details from
            # component response callbacks.
            f.write(_SEP + '\n')
            f.write('// Typed response decode wrappers\n')
            f.write(_SEP + '\n\n')
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    rsp_decl_t = rpc.cpp_rsp_type
                    rsp_raw_t = rpc.raw_rsp_type
                    decode_func = _rpc_decode_response_func(
                        svc.name, rpc, duplicate_rpc_names)
                    col = len(f'bool {decode_func}(')
                    sp = ' ' * col
                    f.write(f'bool {decode_func}'
                            f'(const pcl_msg_t* msg,\n')
                    f.write(f'{sp}{rsp_decl_t}* out)\n')
                    f.write('{\n')
                    f.write('    if (!msg || !msg->data || msg->size == 0 || !out) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    if rpc.streaming:
                        f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{rsp_raw_t}Array", out); if (r == 1) return true; }}\n')
                    else:
                        f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{rsp_raw_t}", out); if (r == 1) return true; }}\n')
                    f.write('    return false;\n')
                    f.write('}\n\n')

                    if rpc.streaming:
                        frame_t = rpc.cpp_rsp_type[len('std::vector<'):-1]
                        encode_frame = _rpc_encode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        decode_frame = _rpc_decode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        send_frame = _rpc_send_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)

                        col = len(f'bool {encode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {encode_frame}(const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type,\n')
                        f.write(f'{sp}std::string*       out)\n')
                        f.write('{\n')
                        f.write('    if (!out) {\n')
                        f.write('        return false;\n')
                        f.write('    }\n')
                        f.write(f'    {{ int r = pyramid_try_registry_encode(content_type, "{rpc.raw_rsp_type}", &payload, out); if (r == 1) return true; }}\n')
                        f.write('    return false;\n')
                        f.write('}\n\n')

                        col = len(f'bool {decode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {decode_frame}(const pcl_msg_t* msg,\n')
                        f.write(f'{sp}{frame_t}* out)\n')
                        f.write('{\n')
                        f.write('    if (!msg || !msg->data || msg->size == 0 || !out) {\n')
                        f.write('        return false;\n')
                        f.write('    }\n')
                        f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{rpc.raw_rsp_type}", out); if (r == 1) return true; }}\n')
                        f.write('    return false;\n')
                        f.write('}\n\n')

                        col = 13 + len(send_frame) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {send_frame}'
                                f'(pcl_stream_context_t* stream_context,\n')
                        f.write(f'{sp}const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type)\n')
                        f.write('{\n')
                        f.write('    std::string wire_payload;\n')
                        f.write(f'    if (!{encode_frame}(payload, content_type, &wire_payload)) {{\n')
                        f.write('        return PCL_ERR_INVALID;\n')
                        f.write('    }\n')
                        f.write('    pcl_msg_t msg{};\n')
                        f.write('    msg.data = wire_payload.data();\n')
                        f.write('    msg.size = static_cast<uint32_t>(wire_payload.size());\n')
                        f.write('    msg.type_name = content_type;\n')
                        f.write('    return pcl_stream_send(stream_context, &msg);\n')
                        f.write('}\n\n')

            # Typed invoke wrappers -- serialize request, then PCL
            f.write(_SEP + '\n')
            f.write('// Typed invoke wrappers \u2014 serialise and dispatch via executor transport\n')
            f.write(_SEP + '\n\n')
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    req_decl_t = rpc.cpp_req_type
                    invoke_func = _rpc_invoke_func(
                        svc.name, rpc, duplicate_rpc_names)
                    service_const = _rpc_service_const(
                        svc.name, rpc, duplicate_rpc_names)
                    col = 13 + len(invoke_func) + 1
                    sp = ' ' * col
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}pcl_resp_cb_fn_t'
                            f'        callback,\n')
                    f.write(f'{sp}void*'
                            f'                   user_data,\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route,\n')
                    f.write(f'{sp}const char*       content_type)\n')
                    f.write('{\n')
                    f.write('    std::string payload;\n')
                    f.write(f'    if (pyramid_try_registry_encode(content_type, "{rpc.raw_req_type}", &request, &payload) != 1) {{\n')
                    f.write('        return PCL_ERR_NOT_FOUND;\n')
                    f.write('    }\n')
                    f.write('    return invoke_async(executor,'
                            f' {service_const},'
                            f' payload, callback, user_data, route, content_type);\n')
                    f.write('}\n\n')
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}const char*       content_type,\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route)\n')
                    f.write('{\n')
                    f.write(f'    return {invoke_func}'
                            '(executor, request, ignore_async_response,\n')
                    f.write('                         nullptr, route, content_type);\n')
                    f.write('}\n\n')
                    if rpc.streaming:
                        invoke_stream = _rpc_invoke_stream_func(
                            svc.name, rpc, duplicate_rpc_names)
                        col = 13 + len(invoke_stream) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {invoke_stream}'
                                f'(pcl_executor_t* executor,\n')
                        f.write(f'{sp}const {req_decl_t}&'
                                f'{" " * max(1, 22 - len(req_decl_t))}'
                                f'request,\n')
                        f.write(f'{sp}pcl_stream_msg_fn_t'
                                f'   callback,\n')
                        f.write(f'{sp}void*'
                                f'                   user_data,\n')
                        f.write(f'{sp}pcl_stream_context_t** out_context,\n')
                        f.write(f'{sp}const pcl_endpoint_route_t* route,\n')
                        f.write(f'{sp}const char*       content_type)\n')
                        f.write('{\n')
                        f.write('    std::string payload;\n')
                        f.write(f'    if (pyramid_try_registry_encode(content_type, "{rpc.raw_req_type}", &request, &payload) != 1) {{\n')
                        f.write('        return PCL_ERR_NOT_FOUND;\n')
                        f.write('    }\n')
                        f.write('    if (route) {\n')
                        f.write('        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);\n')
                        f.write('        if (route_rc != PCL_OK) {\n')
                        f.write('            return route_rc;\n')
                        f.write('        }\n')
                        f.write('    }\n')
                        f.write('    pcl_msg_t msg{};\n')
                        f.write('    msg.data = payload.data();\n')
                        f.write('    msg.size = static_cast<uint32_t>(payload.size());\n')
                        f.write('    msg.type_name = content_type;\n')
                        f.write(f'    return pcl_executor_invoke_stream(executor, {service_const},\n')
                        f.write('                                      &msg, callback, user_data,\n')
                        f.write('                                      out_context);\n')
                        f.write('}\n\n')
            # Publish and topic decode wrappers
            n_pubs = len(topic_set)
            if n_pubs:
                f.write(_SEP + '\n')
                label = 'PCL topic wrapper' + ('s' if n_pubs > 1 else '')
                f.write(f'// {label}\n')
                f.write(_SEP + '\n\n')
                for key, _wire in topic_set.items():
                    pascal = _snake_to_pascal(key)
                    fname = f'publish{pascal}'
                    decode_name = f'decode{pascal}'
                    encode_name = f'encode{pascal}'
                    spec = topic_spec(key)
                    col = len(f'bool {encode_name}(')
                    sp = ' ' * col
                    f.write(f'bool {encode_name}(const {spec.cpp_payload_type}& payload,\n')
                    f.write(f'{sp}const char*        content_type,\n')
                    f.write(f'{sp}std::string*       out)\n')
                    f.write('{\n')
                    f.write('    if (!out) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    topic_schema = spec.short_type + ('Array' if spec.is_array else '')
                    f.write(f'    {{ int r = pyramid_try_registry_encode(content_type, "{topic_schema}", &payload, out); if (r == 1) return true; }}\n')
                    f.write('    return false;\n')
                    f.write('}\n\n')

                    col = 13 + len(fname) + 1
                    sp = ' ' * col
                    f.write(f'pcl_status_t {fname}'
                            f'(pcl_port_t*        publisher,\n')
                    f.write(f'{sp}const {spec.cpp_payload_type}& payload,\n')
                    f.write(f'{sp}const char*        content_type)\n')
                    f.write('{\n')
                    f.write('    std::string wire_payload;\n')
                    f.write(f'    if (!{encode_name}(payload, content_type, &wire_payload)) {{\n')
                    f.write('        return PCL_ERR_INVALID;\n')
                    f.write('    }\n')
                    f.write(f'    return {fname}(publisher, wire_payload, content_type);\n')
                    f.write('}\n\n')
                    f.write(f'pcl_status_t {fname}'
                            f'(pcl_port_t*        publisher,\n')
                    f.write(f'{sp}const std::string& payload,\n')
                    f.write(f'{sp}const char*        content_type)\n')
                    f.write('{\n')
                    f.write('    pcl_msg_t msg{};\n')
                    f.write('    msg.data      = payload.data();\n')
                    f.write('    msg.size      = static_cast<uint32_t>'
                            '(payload.size());\n')
                    f.write('    msg.type_name = content_type;\n')
                    f.write('    return pcl_port_publish(publisher,'
                            ' &msg);\n')
                    f.write('}\n\n')

                    col = len(f'bool {decode_name}(')
                    sp = ' ' * col
                    f.write(f'bool {decode_name}(const pcl_msg_t* msg,\n')
                    f.write(f'{sp}{spec.cpp_payload_type}* out)\n')
                    f.write('{\n')
                    f.write('    if (!msg || !msg->data || msg->size == 0 || !out) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{topic_schema}", out); if (r == 1) return true; }}\n')
                    f.write('    return false;\n')
                    f.write('}\n\n')

            # ---- dispatch() --------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Dispatch \u2014 deserialise request, call handler,'
                    ' serialise response\n')
            f.write(_SEP + '\n\n')
            f.write('void dispatch(ServiceHandler& handler,\n')
            f.write('              ServiceChannel  channel,\n')
            f.write('              const void*     request_buf,\n')
            f.write('              size_t          request_size,\n')
            f.write('              const char*     content_type,\n')
            f.write('              void**          response_buf,\n')
            f.write('              size_t*         response_size)\n')
            f.write('{\n')
            f.write('    if (!response_buf || !response_size) {\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    *response_buf = nullptr;\n')
            f.write('    *response_size = 0;\n')
            f.write('    pcl_msg_t req_msg{};\n')
            f.write('    req_msg.data = request_buf;\n')
            f.write('    req_msg.size = static_cast<uint32_t>(request_size);\n')
            f.write('    req_msg.type_name = content_type;\n')
            f.write('    std::string rsp_payload;\n\n')
            f.write('    try {\n')
            f.write('    switch (channel) {\n')

            for svc_name, rpc in all_rpcs:
                enum_val = _rpc_enum_value(svc_name, rpc, duplicate_rpc_names)
                handler_fn = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                req_t = rpc.cpp_req_type
                rsp_t = rpc.cpp_rsp_type

                f.write(f'    case ServiceChannel::{enum_val}: {{\n')
                f.write(f'        {req_t} req;\n')
                f.write(f'        if (pyramid_try_registry_decode(&req_msg, "{rpc.raw_req_type}", &req) != 1) {{\n')
                f.write('            break;\n')
                f.write('        }\n')
                f.write(f'        auto rsp = handler.{handler_fn}(req);\n')
                if rsp_t.startswith('std::vector<'):
                    f.write(f'        if (pyramid_try_registry_encode(content_type, "{rpc.raw_rsp_type}Array", &rsp, &rsp_payload) != 1) {{\n')
                    f.write('            break;\n')
                    f.write('        }\n')
                else:
                    f.write(f'        if (pyramid_try_registry_encode(content_type, "{rpc.raw_rsp_type}", &rsp, &rsp_payload) != 1) {{\n')
                    f.write('            break;\n')
                    f.write('        }\n')
                f.write('        break;\n')
                f.write('    }\n')

            f.write('    }\n')
            f.write('    } catch (...) {\n')
            f.write('        *response_buf = nullptr;\n')
            f.write('        *response_size = 0;\n')
            f.write('        return;\n')
            f.write('    }\n\n')

            # Copy to heap buffer
            f.write('    if (!rsp_payload.empty()) {\n')
            f.write('        *response_size = rsp_payload.size();\n')
            f.write('        *response_buf = std::malloc(rsp_payload.size());\n')
            f.write('        std::memcpy(*response_buf, rsp_payload.data(),'
                    ' rsp_payload.size());\n')
            f.write('    } else {\n')
            f.write('        *response_buf = nullptr;\n')
            f.write('        *response_size = 0;\n')
            f.write('    }\n')
            f.write('}\n\n')

            # ---- dispatchStream() -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Stream dispatch \u2014 deserialise request and open stream\n')
            f.write(_SEP + '\n\n')
            f.write('pcl_status_t dispatchStream(ServiceHandler& handler,\n')
            f.write('                            ServiceChannel  channel,\n')
            f.write('                            const void*     request_buf,\n')
            f.write('                            size_t          request_size,\n')
            f.write('                            const char*     content_type,\n')
            f.write('                            pcl_stream_context_t* stream_context)\n')
            f.write('{\n')
            f.write('    pcl_msg_t req_msg{};\n')
            f.write('    req_msg.data = request_buf;\n')
            f.write('    req_msg.size = static_cast<uint32_t>(request_size);\n')
            f.write('    req_msg.type_name = content_type;\n\n')
            f.write('    try {\n')
            f.write('    switch (channel) {\n')

            for svc_name, rpc in all_rpcs:
                if not rpc.streaming:
                    continue
                enum_val = _rpc_enum_value(svc_name, rpc, duplicate_rpc_names)
                stream_fn = _rpc_stream_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                req_t = rpc.cpp_req_type
                f.write(f'    case ServiceChannel::{enum_val}: {{\n')
                f.write(f'        {req_t} req;\n')
                f.write(f'        if (pyramid_try_registry_decode(&req_msg, "{rpc.raw_req_type}", &req) != 1) {{\n')
                f.write('            return PCL_ERR_NOT_FOUND;\n')
                f.write('        }\n')
                f.write(f'        return handler.{stream_fn}(req, stream_context, content_type);\n')
                f.write('    }\n')

            f.write('    }\n')
            f.write('    } catch (...) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    return PCL_ERR_INVALID;\n')
            f.write('}\n\n')

            # Namespace close
            f.write(f'}} // namespace {full_ns}\n')

    # -- Component-shaped facade header (.hpp, header-only) --------------------
    #
    # Emits ProvidedHandler / ProvidedService / ConsumedService, layered on
    # top of the existing typed invoke/dispatch/encode/decode primitives. The
    # generated classes are *service bindings*, not pcl::Component subclasses:
    # users compose them as members of their own component and call bind() on
    # the provided side from on_configure().

    def _write_components_header(self, path: Path, file_prefix: str,
                                  full_ns: str, parsed: ProtoFile,
                                  all_rpcs: List[Tuple[str, ProtoRpc]]):
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)
        hpp_name = file_prefix + '.hpp'
        is_provided = _is_provided(parsed)
        sub_topics, _pub_topics = _topics_for_proto(parsed, is_provided)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated component facade for the service binding.\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n')
            f.write('//\n')
            f.write('// Layers on top of the low-level invoke/dispatch primitives in\n')
            f.write(f'// {hpp_name}. Components written against this header do not need to\n')
            f.write('// see pcl_msg_t, stream contexts, or response buffer ownership.\n')
            f.write('#pragma once\n\n')

            f.write(f'#include "{hpp_name}"\n\n')
            f.write('#include <pcl/component.hpp>\n')
            f.write('#include <pcl/executor.hpp>\n\n')
            f.write('#include <cstdlib>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <functional>\n')
            f.write('#include <future>\n')
            f.write('#include <list>\n')
            f.write('#include <memory>\n')
            f.write('#include <stdexcept>\n')
            f.write('#include <string>\n')
            f.write('#include <string_view>\n')
            f.write('#include <utility>\n')
            f.write('#include <vector>\n\n')

            f.write(f'namespace {full_ns} {{\n\n')

            # ---- Result<T> -------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Result<T> -- typed result + pcl_status_t for async API.\n')
            f.write(_SEP + '\n\n')
            f.write('template <class T>\n')
            f.write('struct Result {\n')
            f.write('    pcl_status_t status = PCL_OK;\n')
            f.write('    T            value{};\n\n')
            f.write('    bool ok() const { return status == PCL_OK; }\n')
            f.write('    explicit operator bool() const { return ok(); }\n')
            f.write('};\n\n')

            # ---- StreamWriter<T> -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// StreamWriter<T> -- typed write handle the server-streaming handler\n')
            f.write('// receives. The handler may keep it alive across ticks and call send()\n')
            f.write('// when new frames are available, then end(). Dropping the writer without\n')
            f.write('// calling end() aborts the stream with PCL_ERR_STATE.\n')
            f.write(_SEP + '\n\n')

            f.write('template <class T>\n')
            f.write('class StreamWriter {\n')
            f.write('public:\n')
            f.write('    using SendFn = pcl_status_t (*)(pcl_stream_context_t*,\n')
            f.write('                                    const T&,\n')
            f.write('                                    const char*);\n\n')
            f.write('    /// \\brief Live writer: send() goes on the wire via pcl_stream_send.\n')
            f.write('    StreamWriter(pcl_stream_context_t* ctx,\n')
            f.write('                 std::string content_type,\n')
            f.write('                 SendFn send_fn)\n')
            f.write('        : ctx_(ctx),\n')
            f.write('          content_type_(std::move(content_type)),\n')
            f.write('          send_fn_(send_fn) {}\n\n')
            f.write('    /// \\brief Collecting writer: send() appends to \\p sink. Used when the\n')
            f.write('    ///        binding bridges a streaming-RPC handler to a unary caller.\n')
            f.write('    static StreamWriter collecting(std::vector<T>* sink) {\n')
            f.write('        StreamWriter w;\n')
            f.write('        w.sink_ = sink;\n')
            f.write('        return w;\n')
            f.write('    }\n\n')
            f.write('    StreamWriter() = default;\n')
            f.write('    StreamWriter(const StreamWriter&) = delete;\n')
            f.write('    StreamWriter& operator=(const StreamWriter&) = delete;\n')
            f.write('    StreamWriter(StreamWriter&& o) noexcept { *this = std::move(o); }\n')
            f.write('    StreamWriter& operator=(StreamWriter&& o) noexcept {\n')
            f.write('        if (this != &o) {\n')
            f.write('            abortIfLive();\n')
            f.write('            ctx_          = o.ctx_;          o.ctx_      = nullptr;\n')
            f.write('            content_type_ = std::move(o.content_type_);\n')
            f.write('            send_fn_      = o.send_fn_;      o.send_fn_  = nullptr;\n')
            f.write('            sink_         = o.sink_;         o.sink_     = nullptr;\n')
            f.write('        }\n')
            f.write('        return *this;\n')
            f.write('    }\n')
            f.write('    ~StreamWriter() { abortIfLive(); }\n\n')

            f.write('    /// \\brief Emit one typed frame. Returns PCL_OK on success.\n')
            f.write('    pcl_status_t send(const T& frame) {\n')
            f.write('        if (sink_)   { sink_->push_back(frame); return PCL_OK; }\n')
            f.write('        if (!ctx_ || !send_fn_) return PCL_ERR_STATE;\n')
            f.write('        return send_fn_(ctx_, frame, content_type_.c_str());\n')
            f.write('    }\n\n')

            f.write('    /// \\brief End the stream. \\p status == PCL_OK clean-ends, otherwise\n')
            f.write('    ///        aborts with the given status. Idempotent.\n')
            f.write('    pcl_status_t end(pcl_status_t status = PCL_OK) {\n')
            f.write('        if (sink_) { sink_ = nullptr; return PCL_OK; }\n')
            f.write('        if (!ctx_) return PCL_OK;\n')
            f.write('        pcl_stream_context_t* c = ctx_;\n')
            f.write('        ctx_ = nullptr;\n')
            f.write('        send_fn_ = nullptr;\n')
            f.write('        if (status == PCL_OK) return pcl_stream_end(c);\n')
            f.write('        return pcl_stream_abort(c, status);\n')
            f.write('    }\n\n')

            f.write('    /// \\brief True if the client has asked to cancel the stream.\n')
            f.write('    ///        Servers emitting long streams should poll this and stop.\n')
            f.write('    bool cancelled() const {\n')
            f.write('        return ctx_ && pcl_stream_is_cancelled(ctx_);\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Live state: an open wire stream the handler must end().\n')
            f.write('    ///        False for collecting writers and after end() has run.\n')
            f.write('    bool live() const { return ctx_ != nullptr; }\n\n')

            f.write('private:\n')
            f.write('    void abortIfLive() {\n')
            f.write('        if (!ctx_) return;\n')
            f.write('        pcl_stream_context_t* c = ctx_;\n')
            f.write('        ctx_ = nullptr;\n')
            f.write('        send_fn_ = nullptr;\n')
            f.write('        (void)pcl_stream_abort(c, PCL_ERR_STATE);\n')
            f.write('    }\n\n')

            f.write('    pcl_stream_context_t* ctx_ = nullptr;\n')
            f.write('    std::string           content_type_;\n')
            f.write('    SendFn                send_fn_ = nullptr;\n')
            f.write('    std::vector<T>*       sink_ = nullptr;\n')
            f.write('};\n\n')

            # ---- ProvidedHandler -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// ProvidedHandler -- typed callbacks the component author overrides.\n')
            f.write('//\n')
            f.write('// One on<Name> method per RPC. Unary RPCs return a typed value. Server-\n')
            f.write('// streaming RPCs take a StreamWriter<Frame> the handler keeps and pumps\n')
            f.write('// frames to over time; it must call writer.end() when the stream is\n')
            f.write('// complete (the writer auto-aborts if dropped without end()).\n')
            f.write(_SEP + '\n\n')

            f.write('class ProvidedHandler {\n')
            f.write('public:\n')
            f.write('    virtual ~ProvidedHandler() = default;\n')

            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'\n    // {svc_name}\n')
                    current_svc = svc_name
                on_name = 'on' + _rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)
                if rpc.streaming:
                    frame_t = rpc.cpp_rsp_type[len('std::vector<'):-1]
                    f.write('    virtual void\n')
                    f.write(f'    {on_name}(const {rpc.cpp_req_type}& /*request*/,\n')
                    f.write(f'        StreamWriter<{frame_t}> writer) {{\n')
                    f.write('        // Default: empty stream. Override to emit frames + end.\n')
                    f.write('        writer.end();\n')
                    f.write('    }\n')
                else:
                    f.write(f'    virtual {rpc.cpp_rsp_type}\n')
                    f.write(f'    {on_name}(const {rpc.cpp_req_type}& /*request*/) {{\n')
                    f.write('        return {};\n')
                    f.write('    }\n')
            f.write('};\n\n')

            # ---- ProvidedService -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// ProvidedService -- attach to your pcl::Component to host this package\'s\n')
            f.write('// RPCs. Owns the ServiceHandler-bridge adapter and the per-channel binding\n')
            f.write('// storage (response buffer, content type). Lifecycle of any open streams is\n')
            f.write('// owned by the user via StreamWriter::end().\n')
            f.write('//\n')
            f.write('// Usage: construct as a member of your component, call bind() from\n')
            f.write('// on_configure(); optionally restrict callers with routeAllRemote().\n')
            f.write(_SEP + '\n\n')

            f.write('class ProvidedService {\n')
            f.write('public:\n')
            f.write('    ProvidedService(pcl::Component& host,\n')
            f.write('                    pcl::Executor& executor,\n')
            f.write('                    ProvidedHandler& handler,\n')
            f.write('                    std::string content_type = kJsonContentType)\n')
            f.write('        : host_(&host),\n')
            f.write('          executor_(&executor),\n')
            f.write('          handler_(&handler),\n')
            f.write('          content_type_(std::move(content_type)),\n')
            f.write('          bridge_(*this) {}\n\n')

            f.write('    /// \\brief Owning constructor. \\p handler must not be null;\n')
            f.write('    ///        std::invalid_argument is thrown otherwise.\n')
            f.write('    ProvidedService(pcl::Component& host,\n')
            f.write('                    pcl::Executor& executor,\n')
            f.write('                    std::unique_ptr<ProvidedHandler> handler,\n')
            f.write('                    std::string content_type = kJsonContentType)\n')
            f.write('        : ProvidedService(host, executor,\n')
            f.write('                          requireHandler(handler.get()),\n')
            f.write('                          std::move(content_type)) {\n')
            f.write('        owned_handler_ = std::move(handler);\n')
            f.write('    }\n\n')

            f.write('    ProvidedService(const ProvidedService&) = delete;\n')
            f.write('    ProvidedService& operator=(const ProvidedService&) = delete;\n')
            f.write('    ProvidedService(ProvidedService&&) = delete;\n')
            f.write('    ProvidedService& operator=(ProvidedService&&) = delete;\n\n')

            f.write('    /// \\brief Install the service ports on the host component. Call from\n')
            f.write('    ///        the host\'s on_configure().\n')
            f.write('    pcl_status_t bind() {\n')
            f.write('        if (!supportsContentType(content_type_.c_str())) {\n')
            f.write('            return PCL_ERR_INVALID;\n')
            f.write('        }\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                enum_val = _rpc_enum_value(
                    svc_name, rpc, duplicate_rpc_names)
                if rpc.streaming:
                    f.write(f'        if (!addStreamBinding({svc_const}, ServiceChannel::{enum_val})) return PCL_ERR_NOMEM;\n')
                else:
                    f.write(f'        if (!addUnaryBinding({svc_const}, ServiceChannel::{enum_val})) return PCL_ERR_NOMEM;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Restrict every advertised service to a single peer.\n')
            f.write('    pcl_status_t routeAllRemote(std::string_view peer_id) {\n')
            f.write('        for (const auto& port : ports_) {\n')
            f.write('            const pcl_status_t rc = port.routeRemote(peer_id);\n')
            f.write('            if (rc != PCL_OK) return rc;\n')
            f.write('        }\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('private:\n')
            f.write('    static ProvidedHandler& requireHandler(ProvidedHandler* h) {\n')
            f.write('        if (!h) throw std::invalid_argument("ProvidedHandler must not be null");\n')
            f.write('        return *h;\n')
            f.write('    }\n\n')
            f.write('    // Adapter that satisfies the existing ServiceHandler ABI by\n')
            f.write('    // forwarding typed requests to the user-supplied ProvidedHandler.\n')
            f.write('    class Bridge final : public ServiceHandler {\n')
            f.write('    public:\n')
            f.write('        explicit Bridge(ProvidedService& owner) : owner_(&owner) {}\n\n')

            for svc_name, rpc in all_rpcs:
                on_name = 'on' + _rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)
                handler_name = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                if rpc.streaming:
                    frame_t = rpc.cpp_rsp_type[len('std::vector<'):-1]
                    send_frame = _rpc_send_stream_frame_func(
                        svc_name, rpc, duplicate_rpc_names)
                    stream_name = _rpc_stream_handler_name(
                        svc_name, rpc, duplicate_rpc_names)

                    # Unary path on a streaming RPC: forward to the user's
                    # streaming handler with a collecting writer and return
                    # the accumulated vector. The codec dispatcher still
                    # exercises this when the wire-level invoke arrives via
                    # the unary path (e.g. legacy clients).
                    f.write(f'        {rpc.cpp_rsp_type}\n')
                    f.write(f'        {handler_name}(const {rpc.cpp_req_type}& request) override {{\n')
                    f.write(f'            std::vector<{frame_t}> collected;\n')
                    f.write(f'            auto writer = StreamWriter<{frame_t}>::collecting(&collected);\n')
                    f.write(f'            owner_->handler_->{on_name}(request, std::move(writer));\n')
                    f.write('            return collected;\n')
                    f.write('        }\n')

                    # Streaming path: hand the user a live writer over the
                    # PCL stream context. The user owns end(); if they drop
                    # the writer without ending, the destructor aborts.
                    f.write('        pcl_status_t\n')
                    f.write(f'        {stream_name}(const {rpc.cpp_req_type}& request,\n')
                    f.write('                                    pcl_stream_context_t* stream_context,\n')
                    f.write('                                    const char* content_type) override {\n')
                    f.write(f'            StreamWriter<{frame_t}> writer{{\n')
                    f.write('                stream_context,\n')
                    f.write('                content_type ? content_type : kJsonContentType,\n')
                    f.write(f'                &{send_frame}\n')
                    f.write('            };\n')
                    f.write(f'            owner_->handler_->{on_name}(request, std::move(writer));\n')
                    f.write('            return PCL_STREAMING;\n')
                    f.write('        }\n')
                else:
                    f.write(f'        {rpc.cpp_rsp_type}\n')
                    f.write(f'        {handler_name}(const {rpc.cpp_req_type}& request) override {{\n')
                    f.write(f'            return owner_->handler_->{on_name}(request);\n')
                    f.write('        }\n')
            f.write('    private:\n')
            f.write('        ProvidedService* owner_;\n')
            f.write('    };\n\n')

            f.write('    struct UnaryBinding {\n')
            f.write('        Bridge*         bridge = nullptr;\n')
            f.write('        ServiceChannel  channel = ServiceChannel{};\n')
            f.write('        std::string     content_type;\n')
            f.write('        std::string     response_storage;\n')
            f.write('    };\n\n')
            f.write('    struct StreamBinding {\n')
            f.write('        Bridge*         bridge = nullptr;\n')
            f.write('        ServiceChannel  channel = ServiceChannel{};\n')
            f.write('        std::string     content_type;\n')
            f.write('    };\n\n')

            f.write('    static pcl_status_t unaryDispatch(pcl_container_t* /*self*/,\n')
            f.write('                                       const pcl_msg_t*  request,\n')
            f.write('                                       pcl_msg_t*        response,\n')
            f.write('                                       pcl_svc_context_t* /*ctx*/,\n')
            f.write('                                       void*             user_data) {\n')
            f.write('        auto* b = static_cast<UnaryBinding*>(user_data);\n')
            f.write('        if (!b || !b->bridge || !response) return PCL_ERR_INVALID;\n')
            f.write('        void*  buf = nullptr;\n')
            f.write('        size_t sz = 0;\n')
            f.write('        const char* ct = (request && request->type_name)\n')
            f.write('                             ? request->type_name\n')
            f.write('                             : b->content_type.c_str();\n')
            f.write('        dispatch(*b->bridge, b->channel,\n')
            f.write('                 request ? request->data : nullptr,\n')
            f.write('                 request ? request->size : 0u,\n')
            f.write('                 ct, &buf, &sz);\n')
            f.write('        b->response_storage.clear();\n')
            f.write('        if (buf && sz > 0u) {\n')
            f.write('            b->response_storage.assign(static_cast<const char*>(buf), sz);\n')
            f.write('            std::free(buf);\n')
            f.write('        }\n')
            f.write('        response->data = b->response_storage.empty() ? nullptr\n')
            f.write('                                                     : b->response_storage.data();\n')
            f.write('        response->size = static_cast<uint32_t>(b->response_storage.size());\n')
            f.write('        response->type_name = ct;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    static pcl_status_t streamDispatch(pcl_container_t* /*self*/,\n')
            f.write('                                        const pcl_msg_t*       request,\n')
            f.write('                                        pcl_stream_context_t*  stream_context,\n')
            f.write('                                        void*                  user_data) {\n')
            f.write('        auto* b = static_cast<StreamBinding*>(user_data);\n')
            f.write('        if (!b || !b->bridge) return PCL_ERR_INVALID;\n')
            f.write('        const char* ct = (request && request->type_name)\n')
            f.write('                             ? request->type_name\n')
            f.write('                             : b->content_type.c_str();\n')
            f.write('        return dispatchStream(*b->bridge, b->channel,\n')
            f.write('                              request ? request->data : nullptr,\n')
            f.write('                              request ? request->size : 0u,\n')
            f.write('                              ct, stream_context);\n')
            f.write('    }\n\n')

            f.write('    bool addUnaryBinding(const char* service_name, ServiceChannel channel) {\n')
            f.write('        unary_bindings_.push_back(UnaryBinding{&bridge_, channel, content_type_, {}});\n')
            f.write('        UnaryBinding& binding = unary_bindings_.back();\n')
            f.write('        pcl::Port port = host_->addService(service_name, content_type_.c_str(),\n')
            f.write('                                           &ProvidedService::unaryDispatch, &binding);\n')
            f.write('        if (!port) { unary_bindings_.pop_back(); return false; }\n')
            f.write('        ports_.push_back(port);\n')
            f.write('        return true;\n')
            f.write('    }\n\n')

            f.write('    bool addStreamBinding(const char* service_name, ServiceChannel channel) {\n')
            f.write('        stream_bindings_.push_back(StreamBinding{&bridge_, channel, content_type_});\n')
            f.write('        StreamBinding& binding = stream_bindings_.back();\n')
            f.write('        pcl::Port port = host_->addStreamService(service_name, content_type_.c_str(),\n')
            f.write('                                                 &ProvidedService::streamDispatch, &binding);\n')
            f.write('        if (!port) { stream_bindings_.pop_back(); return false; }\n')
            f.write('        ports_.push_back(port);\n')
            f.write('        return true;\n')
            f.write('    }\n\n')

            f.write('    pcl::Component*                   host_     = nullptr;\n')
            f.write('    pcl::Executor*                    executor_ = nullptr;\n')
            f.write('    ProvidedHandler*                  handler_  = nullptr;\n')
            f.write('    std::unique_ptr<ProvidedHandler>  owned_handler_;\n')
            f.write('    std::string                       content_type_;\n')
            f.write('    Bridge                            bridge_;\n')
            f.write('    std::list<UnaryBinding>           unary_bindings_;\n')
            f.write('    std::list<StreamBinding>          stream_bindings_;\n')
            f.write('    std::vector<pcl::Port>            ports_;\n')
            f.write('};\n\n')

            # ---- ConsumedService -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// ConsumedService -- attach to your pcl::Component to call this package\'s\n')
            f.write('// RPCs. Per-RPC entry points are async-shaped:\n')
            f.write('//   * Unary RPCs return std::future<Result<T>>.\n')
            f.write('//   * Streaming RPCs return a StreamHandle and deliver frames via the\n')
            f.write('//     supplied on_frame/on_end callbacks (both fire on the executor\n')
            f.write('//     thread; together they cover the stream lifetime).\n')
            f.write('//\n')
            f.write('// Usage: construct as a member of your component; call routeAllRemote()\n')
            f.write('// (or routeAllLocal()) once the executor transport is up.\n')
            f.write(_SEP + '\n\n')

            f.write('/// \\brief Move-only handle for an in-flight server-streaming RPC.\n')
            f.write('///\n')
            f.write('/// Returned by every <op>Streaming() entry point. Call cancel() to stop\n')
            f.write('/// receiving frames -- the on_frame callback is suppressed from then on,\n')
            f.write('/// the underlying executor stream context is cancelled (the transport may\n')
            f.write('/// notify the server depending on its cancel support), and on_end fires\n')
            f.write('/// with PCL_ERR_CANCELLED once the framework finalises the stream.\n')
            f.write('/// Destroying or overwriting the handle also cancels, but suppresses\n')
            f.write('/// later user callbacks because their captures may be going away.\n')
            f.write('class StreamHandle {\n')
            f.write('public:\n')
            f.write('    StreamHandle() = default;\n')
            f.write('    StreamHandle(StreamHandle&& o) noexcept\n')
            f.write('        : cancel_fn_(std::move(o.cancel_fn_)) {\n')
            f.write('        o.cancel_fn_ = {};\n')
            f.write('    }\n')
            f.write('    StreamHandle& operator=(StreamHandle&& o) noexcept {\n')
            f.write('        if (this != &o) {\n')
            f.write('            cancelImpl(false);\n')
            f.write('            cancel_fn_ = std::move(o.cancel_fn_);\n')
            f.write('            o.cancel_fn_ = {};\n')
            f.write('        }\n')
            f.write('        return *this;\n')
            f.write('    }\n')
            f.write('    StreamHandle(const StreamHandle&) = delete;\n')
            f.write('    StreamHandle& operator=(const StreamHandle&) = delete;\n\n')
            f.write('    ~StreamHandle() { cancelImpl(false); }\n\n')
            f.write('    bool valid() const { return static_cast<bool>(cancel_fn_); }\n')
            f.write('    explicit operator bool() const { return valid(); }\n\n')
            f.write('    /// \\brief Cancel the stream. Idempotent; subsequent calls no-op.\n')
            f.write('    void cancel() {\n')
            f.write('        cancelImpl(true);\n')
            f.write('    }\n\n')
            f.write('private:\n')
            f.write('    void cancelImpl(bool notify_end) {\n')
            f.write('        if (cancel_fn_) {\n')
            f.write('            cancel_fn_(notify_end);\n')
            f.write('            cancel_fn_ = {};\n')
            f.write('        }\n')
            f.write('    }\n\n')
            f.write('    friend class ConsumedService;\n')
            f.write('    explicit StreamHandle(std::function<void(bool)> cancel_fn)\n')
            f.write('        : cancel_fn_(std::move(cancel_fn)) {}\n')
            f.write('    std::function<void(bool)> cancel_fn_;\n')
            f.write('};\n\n')

            f.write('class ConsumedService {\n')
            f.write('public:\n')
            f.write('    ConsumedService(pcl::Component& host,\n')
            f.write('                    pcl::Executor& executor,\n')
            f.write('                    std::string content_type = kJsonContentType)\n')
            f.write('        : host_(&host),\n')
            f.write('          executor_(&executor),\n')
            f.write('          content_type_(std::move(content_type)) {}\n\n')

            f.write('    ConsumedService(const ConsumedService&) = delete;\n')
            f.write('    ConsumedService& operator=(const ConsumedService&) = delete;\n')
            f.write('    ConsumedService(ConsumedService&&) = delete;\n')
            f.write('    ConsumedService& operator=(ConsumedService&&) = delete;\n\n')

            f.write('    /// \\brief Route every consumed endpoint to the executor\'s\n')
            f.write('    ///        default transport. The transport itself picks the peer\n')
            f.write('    ///        (e.g. shared-memory bus discovers the unique provider).\n')
            f.write('    pcl_status_t routeAllRemote() {\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                f.write('        if (auto rc = executor_->setEndpointRoute(\n')
                f.write(f'                {svc_const}, PCL_ENDPOINT_CONSUMED,\n')
                f.write('                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Route every consumed endpoint to a named peer transport\n')
            f.write('    ///        previously registered via executor.registerTransport().\n')
            f.write('    pcl_status_t routeAllRemote(std::string_view peer_id) {\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                f.write('        if (auto rc = executor_->routeRemote(\n')
                f.write(f'                {svc_const}, peer_id); rc != PCL_OK) return rc;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    pcl_status_t routeAllLocal() {\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                f.write('        if (auto rc = executor_->routeLocal(\n')
                f.write(f'                {svc_const}); rc != PCL_OK) return rc;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            for key in sub_topics:
                pascal = _snake_to_pascal(key)
                fname = f'subscribe{pascal}'
                trampoline = f'trampoline{pascal}'
                decode_name = f'decode{pascal}'
                payload_t = topic_spec(key).cpp_payload_type
                f.write(f'    pcl_port_t* {fname}(\n')
                f.write(f'        std::function<void(const {payload_t}&)> on_message) {{\n')
                f.write('        auto callback =\n')
                f.write(f'            std::make_shared<std::function<void(const {payload_t}&)>>(\n')
                f.write('                std::move(on_message));\n')
                f.write('        topic_callbacks_.push_back(callback);\n')
                f.write('        pcl_port_t* port =\n')
                f.write(f'            ::{full_ns}::{fname}(\n')
                f.write(f'                host_->handle(), &ConsumedService::{trampoline},\n')
                f.write('                callback.get(), content_type_.c_str());\n')
                f.write('        if (!port) {\n')
                f.write('            topic_callbacks_.pop_back();\n')
                f.write('        }\n')
                f.write('        return port;\n')
                f.write('    }\n\n')

            for svc_name, rpc in all_rpcs:
                base = _rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)
                async_name = _lc_first(base) + 'Async'
                invoke_fn = _rpc_invoke_func(
                    svc_name, rpc, duplicate_rpc_names)
                req_t = rpc.cpp_req_type
                if rpc.streaming:
                    frame_t = rpc.cpp_rsp_type[len('std::vector<'):-1]
                    invoke_stream = _rpc_invoke_stream_func(
                        svc_name, rpc, duplicate_rpc_names)
                    decode_frame = _rpc_decode_stream_frame_func(
                        svc_name, rpc, duplicate_rpc_names)
                    streaming_name = _lc_first(base) + 'Streaming'

                    # Push-mode streaming variant -- callbacks fire on the
                    # executor thread and together cover the stream lifetime.
                    f.write('    StreamHandle\n')
                    f.write(f'    {streaming_name}(const {req_t}& request,\n')
                    f.write(f'                std::function<void(const {frame_t}&)> on_frame,\n')
                    f.write('                std::function<void(pcl_status_t)> on_end = {}) {\n')
                    f.write(f'        auto push = std::make_shared<StreamPushState<{frame_t}>>();\n')
                    f.write('        push->on_frame  = std::move(on_frame);\n')
                    f.write('        push->on_end    = std::move(on_end);\n')
                    f.write(f'        push->decoder   = [](const pcl_msg_t* msg, {frame_t}* out) {{\n')
                    f.write(f'            return {decode_frame}(msg, out);\n')
                    f.write('        };\n')
                    f.write(f'        auto holder = std::make_unique<StreamPushHolder<{frame_t}>>(\n')
                    f.write(f'            StreamPushHolder<{frame_t}>{{push}});\n')
                    f.write('        pcl_stream_context_t* ctx_handle = nullptr;\n')
                    f.write(f'        const pcl_status_t rc = {invoke_stream}(\n')
                    f.write('            executor_->handle(), request,\n')
                    f.write(f'            &StreamPushState<{frame_t}>::trampoline, holder.get(),\n')
                    f.write('            &ctx_handle, nullptr, content_type_.c_str());\n')
                    f.write('        if (rc != PCL_OK && rc != PCL_STREAMING) {\n')
                    f.write('            return StreamHandle{};\n')
                    f.write('        }\n')
                    f.write('        (void)holder.release();\n')
                    f.write('        return StreamHandle{[push, ctx_handle](bool notify_end) {\n')
                    f.write('            push->cancelled = true;\n')
                    f.write('            if (!notify_end) {\n')
                    f.write('                push->on_frame = {};\n')
                    f.write('                push->on_end = {};\n')
                    f.write('            }\n')
                    f.write('            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);\n')
                    f.write('        }};\n')
                    f.write('    }\n\n')
                else:
                    rsp_t = rpc.cpp_rsp_type
                    decode_resp = _rpc_decode_response_func(
                        svc_name, rpc, duplicate_rpc_names)
                    f.write(f'    std::future<Result<{rsp_t}>>\n')
                    f.write(f'    {async_name}(const {req_t}& request) {{\n')
                    f.write(f'        auto state = std::make_shared<UnaryState<{rsp_t}>>();\n')
                    f.write('        auto future = state->promise.get_future();\n')
                    f.write('        state->decoder = [](const pcl_msg_t* msg, '
                            f'{rsp_t}* out) {{\n')
                    f.write(f'            return {decode_resp}(msg, out);\n')
                    f.write('        };\n')
                    f.write(f'        auto holder = std::make_unique<UnaryHolder<{rsp_t}>>(UnaryHolder<{rsp_t}>{{state}});\n')
                    f.write(f'        const pcl_status_t rc = {invoke_fn}(\n')
                    f.write('            executor_->handle(), request,\n')
                    f.write(f'            &UnaryState<{rsp_t}>::trampoline,\n')
                    f.write('            holder.get(),\n')
                    f.write('            nullptr, content_type_.c_str());\n')
                    f.write('        if (rc == PCL_OK) {\n')
                    f.write('            (void)holder.release();\n')
                    f.write('        } else {\n')
                    f.write('            state->promise.set_value({rc, {}});\n')
                    f.write('        }\n')
                    f.write('        return future;\n')
                    f.write('    }\n\n')

            # State templates -------------------------------------------------
            f.write('private:\n')
            f.write('    template <class T>\n')
            f.write('    struct UnaryState {\n')
            f.write('        std::promise<Result<T>>                       promise;\n')
            f.write('        std::function<bool(const pcl_msg_t*, T*)>     decoder;\n\n')
            f.write('        static void trampoline(const pcl_msg_t* msg, void* user_data);\n')
            f.write('    };\n\n')
            f.write('    template <class T>\n')
            f.write('    struct UnaryHolderT { std::shared_ptr<UnaryState<T>> state; };\n')
            f.write('    template <class T> using UnaryHolder = UnaryHolderT<T>;\n\n')

            f.write('    template <class T>\n')
            f.write('    struct StreamPushState {\n')
            f.write('        std::function<void(const T&)>                 on_frame;\n')
            f.write('        std::function<void(pcl_status_t)>             on_end;\n')
            f.write('        std::function<bool(const pcl_msg_t*, T*)>     decoder;\n')
            f.write('        bool                                          cancelled = false;\n\n')
            f.write('        bool                                          closed = false;\n\n')
            f.write('        static void trampoline(const pcl_msg_t* msg,\n')
            f.write('                                bool             end,\n')
            f.write('                                pcl_status_t     status,\n')
            f.write('                                void*            user_data);\n')
            f.write('    };\n\n')
            f.write('    template <class T>\n')
            f.write('    struct StreamPushHolderT { std::shared_ptr<StreamPushState<T>> state; };\n')
            f.write('    template <class T> using StreamPushHolder = StreamPushHolderT<T>;\n\n')

            for key in sub_topics:
                pascal = _snake_to_pascal(key)
                trampoline = f'trampoline{pascal}'
                decode_name = f'decode{pascal}'
                payload_t = topic_spec(key).cpp_payload_type
                f.write(f'    static void {trampoline}(pcl_container_t*,\n')
                f.write('                           const pcl_msg_t* msg,\n')
                f.write('                           void* user_data) {\n')
                f.write('        auto* callback =\n')
                f.write(f'            static_cast<std::function<void(const {payload_t}&)>*>(\n')
                f.write('                user_data);\n')
                f.write('        if (!callback || !*callback) return;\n')
                f.write(f'        {payload_t} payload{{}};\n')
                f.write(f'        if ({decode_name}(msg, &payload)) {{\n')
                f.write('            (*callback)(payload);\n')
                f.write('        }\n')
                f.write('    }\n\n')

            f.write('    pcl::Component* host_     = nullptr;\n')
            f.write('    pcl::Executor*  executor_ = nullptr;\n')
            f.write('    std::string     content_type_;\n')
            if sub_topics:
                f.write('    std::vector<std::shared_ptr<void>> topic_callbacks_;\n')
            f.write('};\n\n')

            # Template member-fn definitions outside the class --------------
            f.write('template <class T>\n')
            f.write('inline void ConsumedService::UnaryState<T>::trampoline(\n')
            f.write('        const pcl_msg_t* msg, void* user_data) {\n')
            f.write('    std::unique_ptr<ConsumedService::UnaryHolder<T>> holder(\n')
            f.write('        static_cast<ConsumedService::UnaryHolder<T>*>(user_data));\n')
            f.write('    if (!holder || !holder->state) return;\n')
            f.write('    auto& state = *holder->state;\n')
            f.write('    Result<T> result{};\n')
            f.write('    if (state.decoder && state.decoder(msg, &result.value)) {\n')
            f.write('        result.status = PCL_OK;\n')
            f.write('    } else {\n')
            f.write('        result.status = PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    state.promise.set_value(std::move(result));\n')
            f.write('}\n\n')

            f.write('template <class T>\n')
            f.write('inline void ConsumedService::StreamPushState<T>::trampoline(\n')
            f.write('        const pcl_msg_t* msg, bool end, pcl_status_t status,\n')
            f.write('        void* user_data) {\n')
            f.write('    auto* holder = static_cast<ConsumedService::StreamPushHolder<T>*>(\n')
            f.write('        user_data);\n')
            f.write('    if (!holder || !holder->state) {\n')
            f.write('        if (end) delete holder;\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    auto& state = *holder->state;\n')
            f.write('    if (end) {\n')
            f.write('        state.closed = true;\n')
            f.write('        const pcl_status_t end_status = state.cancelled\n')
            f.write('            ? PCL_ERR_CANCELLED\n')
            f.write('            : status;\n')
            f.write('        if (state.on_end) state.on_end(end_status);\n')
            f.write('        delete holder;\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    if (state.cancelled || status != PCL_OK) return;\n')
            f.write('    T frame{};\n')
            f.write('    if (state.decoder && state.decoder(msg, &frame) && state.on_frame) {\n')
            f.write('        state.on_frame(frame);\n')
            f.write('    }\n')
            f.write('}\n\n')

            f.write(f'}} // namespace {full_ns}\n')


def _lc_first(s: str) -> str:
    """Lowercase first character: CreateRequirementRequest -> createRequirementRequest."""
    if not s:
        return s
    return s[0].lower() + s[1:]


# ---------------------------------------------------------------------------
# Types header generator
# ---------------------------------------------------------------------------

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


def find_scalar_wrappers(index: ProtoTypeIndex) -> Dict[str, str]:
    """Return {message_name: cpp_scalar_type} for transparent wrapper messages."""
    aliases: Dict[str, str] = dict(_FORCED_ALIASES)
    for msg in index.all_messages():
        fields = msg.all_fields()
        if len(fields) == 1 and not fields[0].is_repeated:
            ft = fields[0].type
            fn = fields[0].name
            if ft in _CPP_SCALAR_MAP and fn in _UNIT_FIELD_NAMES:
                aliases[msg.name] = _CPP_SCALAR_MAP[ft]
    return aliases


def _common_cpp_ns(index: ProtoTypeIndex) -> str:
    """Derive C++ namespace from common package prefix of data model protos.

    e.g. pyramid.data_model.base, pyramid.data_model.common -> pyramid::domain_model
    """
    pkgs = [pf.package for pf in index.files if pf.package]
    if not pkgs:
        return 'data_model'
    if all(pkg == _DATA_MODEL_PROTO_ROOT or
           pkg.startswith(_DATA_MODEL_PROTO_ROOT + '.')
           for pkg in pkgs):
        return _DATA_MODEL_TYPES_NS
    parts = [p.split('.') for p in pkgs]
    common = parts[0]
    for p in parts[1:]:
        common = [a for a, b in zip(common, p) if a == b]
    return '::'.join(common)


class CppTypesGenerator:
    """Generates ``{prefix}_types.hpp`` from data model proto files.

    Scalar wrapper messages (single-field message whose sole field is a proto
    scalar) are emitted as ``using`` aliases; all other messages become
    ``struct``s.  Enums are always emitted as ``enum class``.

    The namespace and output file name are derived automatically from the
    common package prefix of the data model proto files.

    Usage::
        gen = CppTypesGenerator(data_model_dir)
        gen.generate(output_dir)
    """

    def __init__(self, data_model_source):
        if isinstance(data_model_source, Path):
            proto_files = parse_proto_tree(data_model_source)
        else:
            proto_files = list(data_model_source)
        self._index = ProtoTypeIndex(proto_files)
        self._data_model_dir = data_model_source if isinstance(data_model_source, Path) else None
        self._ns = _common_cpp_ns(self._index)
        self._prefix = '_'.join(self._ns.split('::'))
        self._aliases = find_scalar_wrappers(self._index)

    # -- public ----------------------------------------------------------------

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        for pf in self._index.files:
            prefix = pf.package.replace('.', '_')
            hpp = out / (prefix + '_types.hpp')
            self._write_hpp_for_file(hpp, pf)
            print(f'  Generated {pf.package.replace(".", "::")} (types)')
        # Umbrella header: pyramid_data_model_types.hpp -> includes all + re-exports
        umbrella = out / _DATA_MODEL_TYPES_HEADER
        self._write_umbrella_hpp(umbrella)
        print(f'  Generated {self._ns} (umbrella)')

    def write_file(self, pf, output_dir: str) -> None:
        """Emit a single per-file types header (no umbrella).

        Used for service-local wrapper messages, which are homed in their own
        component namespace and must NOT be re-exported into domain_model. The
        instance must be constructed with an index that also contains the data
        model files so wrapper field types resolve.
        """
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        prefix = pf.package.replace('.', '_')
        self._write_hpp_for_file(out / (prefix + '_types.hpp'), pf)
        print(f'  Generated {pf.package.replace(".", "::")} (types)')

    # -- internal --------------------------------------------------------------

    def _package_of_type(self, name: str, current_pkg: str = '') -> str:
        """Return the proto package that defines a type."""
        return _package_for_proto_type(self._index, name, current_pkg)

    def _cpp_field_type(self, field_type: str, repeated: bool,
                         current_pkg: str = '') -> str:
        """Resolve a proto field type to a C++ type string.

        When current_pkg is given, cross-package references are fully qualified.
        Both FQN (dotted) and short-name types are qualified when necessary.
        """
        short = field_type.split('.')[-1]
        if field_type in _CPP_SCALAR_MAP:
            base = _CPP_SCALAR_MAP[field_type]
        elif short in self._aliases:
            base = self._aliases[short]           # always a scalar -- no namespace
        elif '.' in field_type and not field_type.startswith('google.'):
            # Fully-qualified proto type
            pkg = '.'.join(field_type.split('.')[:-1])
            if current_pkg and pkg != current_pkg:
                base = _cpp_ns_for_proto_type_package(pkg) + '::' + short
            else:
                base = short
        else:
            # Short name -- look up its package for cross-package qualification
            fqn = _proto_type_fqn(self._index, field_type, current_pkg)
            if fqn and (self._index.is_enum_type(fqn)
                        or self._index.is_message_type(fqn)):
                pkg = fqn.rsplit('.', 1)[0]
                resolved_short = fqn.split('.')[-1]
                if current_pkg and pkg and pkg != current_pkg:
                    base = (_cpp_ns_for_proto_type_package(pkg)
                            + '::' + resolved_short)
                else:
                    base = resolved_short
            else:
                base = short
        return f'std::vector<{base}>' if repeated else base

    def _cpp_default(self, cpp_type: str, field_type: str,
                     current_pkg: str = '') -> str:
        """Default initialiser for a C++ field."""
        short = field_type.split('.')[-1]
        if cpp_type.startswith('std::vector'):
            return '{}'
        if cpp_type in _CPP_DEFAULTS:
            return _CPP_DEFAULTS[cpp_type]
        enum, enum_pkg = _resolve_enum(self._index, field_type, current_pkg)
        if enum is not None:
            if enum and enum.values:
                suf = enum.suffix_of(enum.values[0].name)
                lit = screaming_to_pascal(suf) if suf else enum.values[0].name
                if '.' in field_type and not field_type.startswith('google.'):
                    pkg = '.'.join(field_type.split('.')[:-1])
                    if current_pkg and pkg != current_pkg:
                        return f'{_cpp_ns_for_proto_type_package(pkg)}::{short}::{lit}'
                else:
                    pkg = enum_pkg
                    if current_pkg and pkg and pkg != current_pkg:
                        return f'{_cpp_ns_for_proto_type_package(pkg)}::{field_type}::{lit}'
                return f'{short}::{lit}'
        return '{}'

    def _toposort(self, messages: List[ProtoMessage]) -> List[ProtoMessage]:
        """Return messages in dependency order (dependencies first)."""
        names = {m.name for m in messages}
        by_name = {m.name: m for m in messages}
        deps: Dict[str, set] = {}
        for m in messages:
            d = set()
            for f in m.all_fields():
                short = f.type.split('.')[-1]
                if short in names and short != m.name and short not in self._aliases:
                    d.add(short)
            deps[m.name] = d

        order: List[str] = []
        visited: set = set()

        def visit(name: str) -> None:
            if name in visited:
                return
            visited.add(name)
            for dep in deps.get(name, set()):
                visit(dep)
            order.append(name)

        for m in messages:
            visit(m.name)
        return [by_name[n] for n in order]

    def _write_enum(self, f, enum: ProtoEnum) -> None:
        f.write(f'enum class {enum.name} : int {{\n')
        for v in enum.values:
            suf = enum.suffix_of(v.name)
            lit = screaming_to_pascal(suf) if suf else v.name
            f.write(f'    {lit} = {v.number},\n')
        f.write('};\n\n')

    def _inline_base_fields(self, msg: ProtoMessage, current_pkg: str = ''):
        """Yield (field, comment) for all non-base fields, inlining any 'base'
        fields by expanding their sub-fields directly into the parent struct.

        Only one level of inlining is done; nested 'base' fields are kept as-is.
        """
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                short = field.type.split('.')[-1]
                base_msg, base_pkg = _resolve_message(
                    self._index, field.type, current_pkg)
                if base_msg and base_msg.name not in self._aliases:
                    # inline the base's own fields with collision renaming
                    for bf in base_msg.fields:
                        name = bf.name
                        if name in own_names:
                            name = short.lower() + '_' + name
                        bf_type = _proto_type_fqn(
                            self._index, bf.type, base_pkg) or bf.type
                        yield (_field_with_type(bf, bf_type), name,
                               f'  // from {base_msg.name}')
                    continue
            yield field, field.name, ''

    def _write_struct(self, f, msg: ProtoMessage,
                      current_pkg: str = '') -> None:
        f.write(f'struct {msg.name} {{\n')
        for field, fname, comment in self._inline_base_fields(msg, current_pkg):
            base_cpp = self._cpp_field_type(field.type, False, current_pkg)
            if field.is_repeated:
                cpp_type = f'std::vector<{base_cpp}>'
                default = '{}'
                opt = ''
            elif field.is_optional and base_cpp not in ('std::string',):
                cpp_type = f'tl::optional<{base_cpp}>'
                default = ''
                opt = '  // optional'
            else:
                cpp_type = base_cpp
                default = self._cpp_default(base_cpp, field.type, current_pkg)
                opt = '  // optional' if field.is_optional else ''
            assign = f' = {default}' if default else ''
            f.write(f'    {cpp_type} {fname}{assign};{comment}{opt}\n')
        for oo in msg.oneofs:
            f.write(f'    // oneof {oo.name}\n')
            for field in oo.fields:
                cpp_type = self._cpp_field_type(field.type, False, current_pkg)
                f.write(f'    tl::optional<{cpp_type}> {field.name};\n')
        f.write('};\n')
        struct_constants = _STRUCT_CONSTANTS.get(msg.name, [])
        if struct_constants:
            # A struct is a literal type only if every field is itself literal.
            # std::string / std::vector / tl::optional / message fields all make
            # it non-literal, so its named constants cannot be constexpr -- emit
            # them as inline const instead (e.g. Ack gained a string identifier).
            is_literal = not msg.oneofs and all(
                not f.is_repeated
                and self._cpp_field_type(f.type, False, current_pkg)
                in _LITERAL_CPP_TYPES
                for f in msg.all_fields()
            )
            qualifier = 'constexpr' if is_literal else 'inline const'
            for const_name, init in struct_constants:
                body = init[len(msg.name):] if init.startswith(msg.name) else (
                    '{ ' + init + ' }')
                f.write(qualifier + ' ' + msg.name + ' ' + const_name + body + ';\n')
        f.write('\n')

    def _includes_for_file(self, pf) -> List[str]:
        """Map proto imports to #include lines for sibling data model types headers."""
        result = []
        for imp in pf.imports:
            if imp.startswith('google/'):
                continue
            pkg = imp.replace('/', '.').removesuffix('.proto')
            imp_stem = Path(imp).stem
            for indexed_pf in self._index.files:
                if indexed_pf.package == pkg or indexed_pf.path.stem == imp_stem:
                    result.append(f'#include "{indexed_pf.package.replace(".", "_")}_types.hpp"')
                    break
        return result

    def _write_hpp_for_file(self, path: Path, pf) -> None:
        ns = _cpp_ns_for_proto_package(pf.package)
        current_pkg = pf.package
        includes = self._includes_for_file(pf)
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated types header\n')
            f.write(f'// Generated from: {pf.path.name}'
                    f' by generate_bindings.py (types)\n')
            f.write(f'// Namespace: {ns}\n')
            f.write('#pragma once\n\n')
            f.write('#include <cstdint>\n')
            f.write('#include <tl/optional.hpp>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n')
            for inc in includes:
                f.write(inc + '\n')
            f.write(f'\nnamespace {ns} {{\n\n')

            # using aliases for scalar wrappers defined in this file
            for msg in pf.messages:
                if msg.name in self._aliases:
                    f.write(f'using {msg.name} = {self._aliases[msg.name]};\n')
            f.write('\n')

            # enums defined in this file
            for enum in pf.enums:
                self._write_enum(f, enum)

            # structs defined in this file (toposorted within this file)
            for msg in self._toposort(non_alias):
                self._write_struct(f, msg, current_pkg=current_pkg)

            f.write(f'}} // namespace {ns}\n')

    def _write_umbrella_hpp(self, path: Path) -> None:
        """Umbrella header that includes all per-file headers and re-exports
        everything into the common domain-model namespace."""
        per_file_headers = sorted(
            pf.package.replace('.', '_') + '_types.hpp'
            for pf in self._index.files
        )
        # A short name shared by more than one package cannot be re-exported
        # flat into the root (it would be ambiguous). Such types must be
        # referenced through their package sub-namespace
        # (e.g. domain_model::pim_osprey::sensor_products::SPRRequest); the
        # generated structs/codecs already qualify cross-package references.
        name_packages: Dict[str, set] = {}
        for pf in self._index.files:
            for msg in pf.messages:
                name_packages.setdefault(msg.name, set()).add(pf.package)
            for enum in pf.enums:
                name_packages.setdefault(enum.name, set()).add(pf.package)
        unique_names = {n for n, pkgs in name_packages.items() if len(pkgs) == 1}

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated umbrella types header\n')
            f.write('// Includes all data model type headers and re-exports\n')
            f.write(f'// uniquely-named types into namespace {_DATA_MODEL_TYPES_NS}.\n')
            f.write('// Names shared across packages are intentionally NOT flattened\n')
            f.write('// and must be used via their package sub-namespace.\n')
            f.write('#pragma once\n\n')
            for h in per_file_headers:
                f.write(f'#include "{h}"\n')
            f.write('\n// Re-export uniquely-named sub-namespace types into the\n')
            f.write('// common namespace so generated bindings can use a single root.\n')
            f.write(f'namespace {self._ns} {{\n')
            for pf in self._index.files:
                ns = _cpp_ns_for_proto_package(pf.package)
                if ns == self._ns:
                    continue
                for msg in pf.messages:
                    if msg.name in unique_names:
                        f.write(f'using {ns}::{msg.name};\n')
                        for const_name, _init in _STRUCT_CONSTANTS.get(
                                msg.name, []):
                            f.write(f'using {ns}::{const_name};\n')
                for enum in pf.enums:
                    if enum.name in unique_names:
                        f.write(f'using {ns}::{enum.name};\n')
            f.write(f'}} // namespace {self._ns}\n')


# ---------------------------------------------------------------------------
# Data model JSON codec generator (proto-driven, per file)
# ---------------------------------------------------------------------------

_CPP_INTEGRAL_SCALARS = frozenset({
    'double', 'float', 'int32_t', 'int64_t', 'uint32_t', 'uint64_t', 'bool',
})


class CppDataModelCodecGenerator:
    """Generates ``{pkg}_codec.hpp/cpp`` from a single data model proto file.

    For each message and enum defined in the proto, emits nlohmann::json-based
    toJson / fromJson helpers in the same namespace as the types.

    Usage::
        proto_files = parse_proto_tree(data_model_dir)
        index = ProtoTypeIndex(proto_files)
        for pf in proto_files:
            gen = CppDataModelCodecGenerator(pf, index)
            gen.generate(output_dir)
    """

    def __init__(self, pf, index: 'ProtoTypeIndex'):
        self._pf = pf
        self._index = index
        self._ns = _cpp_ns_for_proto_package(pf.package)
        self._prefix = pf.package.replace('.', '_')
        self._types_header = self._prefix + '_types.hpp'
        self._hpp_name = self._prefix + '_codec.hpp'
        self._cpp_name = self._prefix + '_codec.cpp'
        # Build alias map (scalar wrappers) -- same logic as CppTypesGenerator
        self._aliases: Dict[str, str] = dict(_FORCED_ALIASES)
        for msg in self._index.all_messages():
            fields = msg.all_fields()
            if len(fields) == 1 and not fields[0].is_repeated:
                ft = fields[0].type
                fn = fields[0].name
                if ft in _CPP_SCALAR_MAP and fn in _UNIT_FIELD_NAMES:
                    self._aliases[msg.name] = _CPP_SCALAR_MAP[ft]

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        self._write_header(out / self._hpp_name)
        self._write_impl(out / self._cpp_name)
        print(f'  Generated {self._ns} (codec)')

    # ------------------------------------------------------------------ header

    def _write_header(self, path: Path) -> None:
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated data model JSON codec header\n')
            f.write(f'// Generated from: {self._pf.path.name}'
                    f' by generate_bindings.py (codec)\n')
            f.write(f'// Namespace: {self._ns}\n')
            f.write('#pragma once\n\n')
            f.write(f'#include "{self._types_header}"\n')
            f.write('#include <string>\n\n')
            f.write(f'namespace {self._ns} {{\n\n')

            # Enum converters
            if self._pf.enums:
                f.write('// Enum string converters\n')
                for enum in self._pf.enums:
                    t = enum.name
                    fn = _lc_first(t)
                    f.write(f'inline std::string toString({t} v) {{\n')
                    f.write('    switch (v) {\n')
                    for v in enum.values:
                        suf = enum.suffix_of(v.name)
                        lit = screaming_to_pascal(suf) if suf else v.name
                        f.write(f'        case {t}::{lit}: return "{v.name}";\n')
                    first_suf = enum.suffix_of(enum.values[0].name)
                    first_lit = (
                        screaming_to_pascal(first_suf)
                        if first_suf else enum.values[0].name)
                    f.write('    }\n')
                    f.write(f'    return "{enum.values[0].name}";\n')
                    f.write('}\n')
                    f.write(f'inline {t} {fn}FromString(const std::string& s) {{\n')
                    for v in enum.values:
                        suf = enum.suffix_of(v.name)
                        lit = screaming_to_pascal(suf) if suf else v.name
                        f.write(f'    if (s == "{v.name}") return {t}::{lit};\n')
                    f.write(f'    return {t}::{first_lit};\n')
                    f.write('}\n')
                f.write('\n')

            alias_names = set(self._aliases.keys())
            structs = [m for m in self._pf.messages if m.name not in alias_names]

            # Struct codec declarations
            if structs:
                f.write('// JSON codec\n')
                for msg in structs:
                    f.write(f'std::string toJson(const {msg.name}& msg);\n')
                    f.write(f'{msg.name} fromJson(const std::string& s,'
                            f' {msg.name}* /*tag*/ = nullptr);\n')
                f.write('\n')

            f.write(f'}} // namespace {self._ns}\n')

    # ------------------------------------------------------------------ impl

    def _write_impl(self, path: Path) -> None:
        alias_names = set(self._aliases.keys())
        structs = [m for m in self._pf.messages if m.name not in alias_names]
        current_pkg = self._pf.package

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated data model JSON codec implementation\n')
            f.write(f'// Namespace: {self._ns}\n\n')
            f.write(f'#include "{self._hpp_name}"\n\n')
            f.write('#include <nlohmann/json.hpp>\n\n')
            # Include codec headers for imported packages (for nested toJson calls)
            included_codec_pkgs = set()
            for imp in self._pf.imports:
                if imp.startswith('google/'):
                    continue
                pkg = imp.replace('/', '.').removesuffix('.proto')
                imp_stem = Path(imp).stem
                for indexed_pf in self._index.files:
                    if indexed_pf.package == pkg or indexed_pf.path.stem == imp_stem:
                        f.write(f'#include "{indexed_pf.package.replace(".", "_")}_codec.hpp"\n')
                        included_codec_pkgs.add(indexed_pf.package)
                        break
            for msg in structs:
                codec_fields = list(self._inline_base_fields(msg, current_pkg))
                codec_fields.extend(
                    (field, field.name, '')
                    for oo in msg.oneofs
                    for field in oo.fields
                )
                for field, _fname, _comment in codec_fields:
                    short = field.type.split('.')[-1]
                    if short in self._aliases:
                        continue
                    if not (
                            _is_proto_message_type(self._index, field.type,
                                                   current_pkg)
                            or _is_proto_enum_type(self._index, field.type,
                                                   current_pkg)):
                        continue
                    pkg = _package_for_proto_type(
                        self._index, field.type, current_pkg)
                    if (pkg and pkg != current_pkg
                            and pkg not in included_codec_pkgs):
                        f.write(f'#include "{pkg.replace(".", "_")}_codec.hpp"\n')
                        included_codec_pkgs.add(pkg)
            f.write(f'\nnamespace {self._ns} {{\n\n')

            # Struct codec implementations
            for msg in structs:
                self._write_to_json(f, msg, current_pkg)
                self._write_from_json(f, msg, current_pkg)

            f.write(f'}} // namespace {self._ns}\n')

    def _package_of_type(self, name: str, current_pkg: str = '') -> str:
        """Return the proto package that defines a type."""
        return _package_for_proto_type(self._index, name, current_pkg)

    def _qualify(self, type_name: str, current_pkg: str) -> str:
        """Return qualified C++ name for type_name relative to current_pkg."""
        short = type_name.split('.')[-1]
        if '.' in type_name and not type_name.startswith('google.'):
            pkg = '.'.join(type_name.split('.')[:-1])
        else:
            pkg = self._package_of_type(type_name, current_pkg)
        if pkg and pkg != current_pkg:
            return _cpp_ns_for_proto_type_package(pkg) + '::' + short
        return short

    def _field_info(self, field, fname: str, current_pkg: str):
        """Return (base_cpp_type, is_struct, is_enum, is_alias) for a field."""
        short = field.type.split('.')[-1]
        if field.type in _CPP_SCALAR_MAP:
            return _CPP_SCALAR_MAP[field.type], False, False, False
        if short in self._aliases:
            return self._aliases[short], False, False, True
        if _is_proto_enum_type(self._index, field.type, current_pkg):
            return self._qualify(field.type, current_pkg), False, True, False
        if _is_proto_message_type(self._index, field.type, current_pkg):
            if short in self._aliases:
                return self._aliases[short], False, False, True
            return self._qualify(field.type, current_pkg), True, False, False
        return short, False, False, False

    def _codec_ns_prefix(self, base_cpp: str) -> str:
        return base_cpp.rsplit('::', 1)[0] + '::' if '::' in base_cpp else ''

    def _write_to_json(self, f, msg, current_pkg: str) -> None:
        f.write(f'std::string toJson(const {msg.name}& msg) {{\n')
        f.write('    nlohmann::json obj;\n')
        for field, fname, _ in self._inline_base_fields(msg, current_pkg):
            base_cpp, is_struct, is_enum, _is_alias = self._field_info(
                field, fname, current_pkg)
            if field.is_repeated:
                f.write(f'    {{\n')
                f.write(f'        nlohmann::json arr = nlohmann::json::array();\n')
                f.write(f'        for (const auto& v : msg.{fname}) {{\n')
                if is_enum:
                    ns_tok = base_cpp.rsplit('::', 1)[0] + '::' if '::' in base_cpp else ''
                    fn_name = _lc_first(base_cpp.split('::')[-1]) + 'FromString'
                    f.write(f'            arr.push_back('
                            f'{ns_tok if ns_tok else ""}toString(v));\n')
                elif is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'            arr.push_back('
                            f'nlohmann::json::parse({ns_pre}toJson(v)));\n')
                else:
                    f.write(f'            arr.push_back(v);\n')
                f.write(f'        }}\n')
                f.write(f'        obj["{fname}"] = arr;\n')
                f.write(f'    }}\n')
            elif field.is_optional and base_cpp not in ('std::string',):
                f.write(f'    if (msg.{fname}.has_value()) {{\n')
                if is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'        obj["{fname}"] = nlohmann::json::parse('
                            f'{ns_pre}toJson(msg.{fname}.value()));\n')
                elif is_enum:
                    f.write(f'        obj["{fname}"] = toString(msg.{fname}.value());\n')
                else:
                    f.write(f'        obj["{fname}"] = msg.{fname}.value();\n')
                f.write(f'    }}\n')
            else:
                if is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'    obj["{fname}"] = nlohmann::json::parse('
                            f'{ns_pre}toJson(msg.{fname}));\n')
                elif is_enum:
                    f.write(f'    obj["{fname}"] = toString(msg.{fname});\n')
                else:
                    f.write(f'    obj["{fname}"] = msg.{fname};\n')
        for oo in msg.oneofs:
            for field in oo.fields:
                base_cpp, is_struct, is_enum, _ = self._field_info(
                    field, field.name, current_pkg)
                f.write(f'    if (msg.{field.name}.has_value()) {{\n')
                if is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'        obj["{field.name}"] = nlohmann::json::parse('
                            f'{ns_pre}toJson(msg.{field.name}.value()));\n')
                elif is_enum:
                    f.write(f'        obj["{field.name}"] = toString('
                            f'msg.{field.name}.value());\n')
                else:
                    f.write(f'        obj["{field.name}"] = msg.{field.name}.value();\n')
                f.write(f'    }}\n')
        f.write('    return obj.dump();\n')
        f.write(f'}}\n\n')

    def _write_from_json(self, f, msg, current_pkg: str) -> None:
        f.write(f'{msg.name} fromJson(const std::string& s,'
                f' {msg.name}* /*tag*/) {{\n')
        f.write('    auto j = nlohmann::json::parse(s);\n')
        f.write(f'    {msg.name} msg;\n')
        for field, fname, _ in self._inline_base_fields(msg, current_pkg):
            base_cpp, is_struct, is_enum, _is_alias = self._field_info(
                field, fname, current_pkg)
            short_type = base_cpp.split('::')[-1]
            if field.is_repeated:
                f.write(f'    if (j.contains("{fname}")) {{\n')
                f.write(f'        for (const auto& v : j["{fname}"]) {{\n')
                if is_enum:
                    fn_name = (_lc_first(short_type) + 'FromString')
                    ns_pre = (base_cpp.rsplit('::', 1)[0] + '::'
                              if '::' in base_cpp else '')
                    f.write(f'            msg.{fname}.push_back('
                            f'{ns_pre}{fn_name}(v.get<std::string>()));\n')
                elif is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'            msg.{fname}.push_back('
                            f'{ns_pre}fromJson(v.dump(), '
                            f'static_cast<{base_cpp}*>(nullptr)));\n')
                else:
                    f.write(f'            msg.{fname}.push_back(v.get<{base_cpp}>());\n')
                f.write(f'        }}\n')
                f.write(f'    }}\n')
            elif field.is_optional and base_cpp not in ('std::string',):
                f.write(f'    if (j.contains("{fname}")) {{\n')
                if is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'        msg.{fname} = {ns_pre}fromJson('
                            f'j["{fname}"].dump(),'
                            f' static_cast<{base_cpp}*>(nullptr));\n')
                elif is_enum:
                    fn_name = _lc_first(short_type) + 'FromString'
                    ns_pre = (base_cpp.rsplit('::', 1)[0] + '::'
                              if '::' in base_cpp else '')
                    f.write(f'        msg.{fname} = {ns_pre}{fn_name}('
                            f'j["{fname}"].get<std::string>());\n')
                else:
                    f.write(f'        msg.{fname} = j["{fname}"].get<{base_cpp}>();\n')
                f.write(f'    }}\n')
            else:
                if is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'    if (j.contains("{fname}")) msg.{fname} = {ns_pre}fromJson('
                            f'j["{fname}"].dump(),'
                            f' static_cast<{base_cpp}*>(nullptr));\n')
                elif is_enum:
                    fn_name = _lc_first(short_type) + 'FromString'
                    ns_pre = (base_cpp.rsplit('::', 1)[0] + '::'
                              if '::' in base_cpp else '')
                    f.write(f'    if (j.contains("{fname}")) msg.{fname} = '
                            f'{ns_pre}{fn_name}(j["{fname}"].get<std::string>());\n')
                elif base_cpp == 'std::string':
                    f.write(f'    if (j.contains("{fname}")) msg.{fname} = '
                            f'j["{fname}"].get<std::string>();\n')
                else:
                    f.write(f'    if (j.contains("{fname}")) msg.{fname} = '
                            f'j["{fname}"].get<{base_cpp}>();\n')
        for oo in msg.oneofs:
            for field in oo.fields:
                base_cpp, is_struct, is_enum, _ = self._field_info(
                    field, field.name, current_pkg)
                short_type = base_cpp.split('::')[-1]
                f.write(f'    if (j.contains("{field.name}")) {{\n')
                if is_struct:
                    ns_pre = self._codec_ns_prefix(base_cpp)
                    f.write(f'        msg.{field.name} = {ns_pre}fromJson('
                            f'j["{field.name}"].dump(),'
                            f' static_cast<{base_cpp}*>(nullptr));\n')
                elif is_enum:
                    fn_name = _lc_first(short_type) + 'FromString'
                    ns_pre = (base_cpp.rsplit('::', 1)[0] + '::'
                              if '::' in base_cpp else '')
                    f.write(f'        msg.{field.name} = {ns_pre}{fn_name}('
                            f'j["{field.name}"].get<std::string>());\n')
                else:
                    f.write(f'        msg.{field.name} = '
                            f'j["{field.name}"].get<{base_cpp}>();\n')
                f.write(f'    }}\n')
        f.write('    return msg;\n')
        f.write(f'}}\n\n')

    def _inline_base_fields(self, msg, current_pkg: str = ''):
        """Mirror CppTypesGenerator._inline_base_fields for codec use."""
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                short = field.type.split('.')[-1]
                base_msg, base_pkg = _resolve_message(
                    self._index, field.type, current_pkg)
                if base_msg and base_msg.name not in self._aliases:
                    for bf in base_msg.fields:
                        name = bf.name
                        if name in own_names:
                            name = short.lower() + '_' + name
                        bf_type = _proto_type_fqn(
                            self._index, bf.type, base_pkg) or bf.type
                        yield _field_with_type(bf, bf_type), name, ''
                    continue
            yield field, field.name, ''


# ---------------------------------------------------------------------------


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
