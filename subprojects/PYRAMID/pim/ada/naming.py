#!/usr/bin/env python3
"""Ada naming, package derivation and binding-collection helpers.

Split verbatim from ada_codegen.py (generator refactor plan, phase 4).
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from proto_parser import (
    parse_proto_tree,
    ProtoTypeIndex,
    ProtoMessage,
    ProtoFile,
    ProtoRpc,
    ProtoService,
    camel_to_snake,
    parse_proto,
    camel_to_snake as _camel_to_snake,
    camel_to_lower_snake as _camel_to_lower_snake,
)
from binding_contract import TopicSpecResolver, PyramidCompatNamingPolicy
from cabi_codegen import _c_struct_name


# -- EntityActions operation set -----------------------------------------------

OP_PREFIXES = ['Create', 'Read', 'Update', 'Delete', 'Cancel']

# Base-type short names from pyramid.data_model.base.* and common.*.
# The domain literals live in the PYRAMID compat policy (single source of
# truth); this module just consumes them (see C3 in the PYRAMID TODO).
BASE_TYPE_MAP = PyramidCompatNamingPolicy.base_type_map

# Wire-name service prefix derived from proto service name
# e.g. Object_Of_Interest_Service -> "object_of_interest"
# RPC CreateRequirement -> "create_requirement"
# Full wire name: "object_of_interest.create_requirement"

# Standard topic names -- tactical_objects bridge-facing topics aligned
# directly to canonical proto-derived PYRAMID payloads.


# -- Proto parser --------------------------------------------------------------

def _short_type(full_type: str) -> str:
    """pyramid.data_model.base.Identifier -> Identifier."""
    if full_type in BASE_TYPE_MAP:
        return BASE_TYPE_MAP[full_type]
    return full_type.split('.')[-1]


def _proto_type_to_ada(full_type: str) -> str:
    """Map a proto type to its Ada equivalent in Tactical_Objects_Types."""
    short = _short_type(full_type)
    # Apply CamelCase -> Ada_Style conversion for compound names
    return _camel_to_snake(short)


def _service_wire_prefix(service_name: str) -> str:
    """Object_Of_Interest_Service -> object_of_interest."""
    # Remove _Service suffix if present
    name = service_name
    if name.endswith('_Service'):
        name = name[:-len('_Service')]
    return _camel_to_lower_snake(name)


def _service_ada_prefix(service_name: str) -> str:
    """Data_Provision_Dependency_Service -> Data_Provision_Dependency."""
    name = service_name
    if name.endswith('_Service'):
        name = name[:-len('_Service')]
    return _camel_to_snake(name)


def _duplicate_rpc_names(all_rpcs: List[Tuple[str, 'ProtoRpc']]) -> set[str]:
    counts: Dict[str, int] = {}
    for _svc_name, rpc in all_rpcs:
        counts[rpc.name] = counts.get(rpc.name, 0) + 1
    return {name for name, count in counts.items() if count > 1}


def _rpc_ada_base(svc_name: str, rpc: 'ProtoRpc',
                  duplicate_rpc_names: set[str]) -> str:
    del duplicate_rpc_names
    rpc_name = _camel_to_snake(rpc.name)
    return f'{_service_ada_prefix(svc_name)}_{rpc_name}'


def _rpc_ada_handler(svc_name: str, rpc: 'ProtoRpc',
                     duplicate_rpc_names: set[str]) -> str:
    return f'Handle_{_rpc_ada_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_ada_channel(svc_name: str, rpc: 'ProtoRpc',
                     duplicate_rpc_names: set[str]) -> str:
    return f'Ch_{_rpc_ada_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_ada_invoke_name(svc_name: str, rpc: 'ProtoRpc',
                         duplicate_rpc_names: set[str]) -> str:
    return f'Invoke_{_rpc_ada_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_ada_decode_response_name(svc_name: str, rpc: 'ProtoRpc',
                                  duplicate_rpc_names: set[str]) -> str:
    return f'Decode_{_rpc_ada_base(svc_name, rpc, duplicate_rpc_names)}_Response'


def _rpc_ada_svc_const(svc_name: str, rpc: 'ProtoRpc',
                       duplicate_rpc_names: set[str]) -> str:
    return f'Svc_{_rpc_ada_base(svc_name, rpc, duplicate_rpc_names)}'


def _rpc_ada_handler_field(svc_name: str, rpc: 'ProtoRpc',
                           duplicate_rpc_names: set[str]) -> str:
    return _rpc_ada_handler(svc_name, rpc, duplicate_rpc_names).replace(
        'Handle_', 'On_')


def _rpc_ada_callback_name(svc_name: str, rpc: 'ProtoRpc',
                           duplicate_rpc_names: set[str]) -> str:
    return f'Service_{_rpc_ada_base(svc_name, rpc, duplicate_rpc_names)}'


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
    """create_requirement (rpc part of wire name)."""
    return _camel_to_lower_snake(rpc.name)


def _ada_req_type(rpc: ProtoRpc) -> str:
    return _proto_type_to_ada(rpc.request_type)


def _ada_rsp_type(rpc: ProtoRpc) -> str:
    base = _proto_type_to_ada(rpc.response_type)
    if rpc.server_streaming:
        return f'{base}_Array'
    return base


# -- Ada package name derivation -----------------------------------------------

def _pkg_name_from_proto(proto_file: ProtoFile) -> str:
    parts = proto_file.package.split('.')

    last = parts[-1].lower()
    explicit_suffix = None
    if last in ('provided', 'consumed'):
        explicit_suffix = _ada_pkg_segment(parts[-1])
        parts = parts[:-1]

    skip = {'pyramid', 'components', 'data_model', 'base', 'services'}
    meaningful = [p for p in parts if p.lower() not in skip]

    ada_parts = ['Pyramid', 'Services']
    for p in meaningful:
        ada_parts.append(_ada_pkg_segment(p))

    ada_parts.append(explicit_suffix if explicit_suffix else 'Provided')
    return '.'.join(ada_parts)


def _generic_pkg_name_from_proto(proto_file: ProtoFile) -> str:
    """Generic (non-PYRAMID) Ada service package name.

    Derives the Ada package flat from the proto package, with no reserved
    ``Pyramid.Services`` root or ``provided``/``consumed`` suffix, so services
    declared directly in a main namespace stay there:

      example.telemetry -> Example.Telemetry.Services
    """
    parts = proto_file.package.split('.')
    ada_parts = [_ada_pkg_segment(p) for p in parts if p]
    ada_parts.append('Services')
    return '.'.join(ada_parts)


def _is_provided(proto_file: ProtoFile) -> bool:
    return 'provided' in proto_file.package.lower()


# All data model type packages, ordered base -> common -> tactical so that
# `use` clauses in dependent order work correctly.
_DATA_MODEL_TYPES_PKGS = [
    'Pyramid.Data_Model.Base.Types',
    'Pyramid.Data_Model.Common.Types',
    'Pyramid.Data_Model.Tactical.Types',
]

# Primary (most specific) package -- kept for codec / single-package references.
_DATA_MODEL_TYPES_PKG = _DATA_MODEL_TYPES_PKGS[-1]

# Data model codec packages: Ada type pkg -> Ada codec pkg.
# No base-types codec; Identifier/Ack/Query use hardcoded Ada stdlib paths.
_DM_CODEC_PKG_FOR_TYPE_PKG: Dict[str, str] = {
    'Pyramid.Data_Model.Common.Types':   'Pyramid.Data_Model.Common.Types_Codec',
    'Pyramid.Data_Model.Tactical.Types': 'Pyramid.Data_Model.Tactical.Types_Codec',
}


def _proto_pkg_of_type(fqn: str) -> Optional[str]:
    """Extract the proto package from a fully-qualified type name.

    'pyramid.data_model.tactical.TacticalObject' -> 'pyramid.data_model.tactical'
    'TacticalObject'                              -> None  (unqualified)
    """
    if '.' not in fqn:
        return None
    return fqn.rsplit('.', 1)[0]


def _ada_pkg_from_proto_pkg(proto_pkg: str) -> str:
    """Convert a proto package name to its Ada type package name.

    Uses the same convention as AdaTypesGenerator._ada_pkg_for_file():
      pyramid.data_model.tactical -> Pyramid.Data_Model.Tactical.Types
    """
    parts = proto_pkg.split('.')
    ada_parts = [_ada_pkg_segment(seg) for seg in parts]
    return '.'.join(ada_parts + ['Types'])


def _find_proto_root(proto_path: Path) -> Optional[Path]:
    """Walk up from *proto_path* to find the proto root directory.

    The root is the first ancestor directory that contains a subdirectory
    matching the first segment of a proto import (typically 'pyramid').
    Returns None if not found.
    """
    parent = proto_path.parent
    while parent != parent.parent:
        if (parent / 'pyramid').is_dir():
            return parent
        parent = parent.parent
    return None


def _collect_type_pkgs(proto_path: Path,
                       all_rpcs: List[Tuple[str, 'ProtoRpc']],
                       resolver: TopicSpecResolver) -> List[str]:
    """Return all Ada type packages required by the service binding.

    Traces two layers:
      1. The proto package of each RPC request/response type (from the FQN).
      2. The proto packages of FIELDS within those message types, so that
         types used by the request/response records are also visible.

    Uses proto_parser to load the service proto and its imports for step 2;
    degrades gracefully (step-1 results only) when proto files can't be found.

    Returns packages in a deterministic order: known data-model packages first
    (in _DATA_MODEL_TYPES_PKGS order), then any additional packages sorted.
    """
    needed: set = set()
    rpc_types = {rpc.request_type for _, rpc in all_rpcs} | {rpc.response_type for _, rpc in all_rpcs}

    # Step 1 -- direct packages from fully-qualified type names in the RPC signature.
    for t in rpc_types:
        pkg = _proto_pkg_of_type(t)
        if pkg and not pkg.startswith('google.'):
            needed.add(_ada_pkg_from_proto_pkg(pkg))

    # Step 2 -- load proto + imports; trace each RPC message's field types too.
    try:
        svc_pf = parse_proto(proto_path)
        loaded = [svc_pf]

        proto_root = _find_proto_root(proto_path)
        if proto_root is not None:
            for imp in svc_pf.imports:
                if imp.startswith('google/'):
                    continue
                imp_path = proto_root / imp
                if imp_path.exists():
                    try:
                        loaded.append(parse_proto(imp_path))
                    except Exception:
                        pass

        index = ProtoTypeIndex(loaded)

        # The service's own proto may declare local wrapper messages (oneof
        # request/response payloads, plus a synthesized Empty) that RPCs
        # reference by bare name. Those types live in the service's Component-NS
        # types package, so the binding must with/use it. Use the same wrapper
        # detection (incl. Empty synthesis) as the generator so message-less
        # Empty-only services are covered too.
        wrapper_pf = _service_wrapper_pf(proto_path)
        if wrapper_pf is not None:
            needed.add(_ada_pkg_from_proto_pkg(wrapper_pf.package))

        # Standard-topic payloads need their declaring package withed too, even
        # when no RPC references them; absent payloads are filtered out.
        _, _, topic_pkgs = _applicable_topics(
            svc_pf, _is_provided(svc_pf), proto_path, resolver)
        needed.update(topic_pkgs)

        for t in rpc_types:
            # Try FQN first, fall back to short name.
            msg = index.resolve_message(t) or index.resolve_message(t.split('.')[-1])
            if msg is None:
                continue
            for fld in msg.all_fields():
                if fld.is_scalar or fld.is_map:
                    continue
                fld_pkg = _proto_pkg_of_type(fld.type)
                if fld_pkg and not fld_pkg.startswith('google.'):
                    needed.add(_ada_pkg_from_proto_pkg(fld_pkg))
    except Exception:
        pass  # Step-1 results are still valid.

    # Return in a stable order: known data-model packages first (ordered),
    # then any additional packages (alphabetically sorted).
    known_ordered = [p for p in _DATA_MODEL_TYPES_PKGS if p in needed]
    extra = sorted(p for p in needed if p not in set(_DATA_MODEL_TYPES_PKGS))
    return known_ordered + extra


def _collect_codec_pkgs(type_pkgs: List[str]) -> List[str]:
    """Return codec packages needed for the body, derived from type packages.

    The spec already walks imported/request/response field types to find the
    correct Ada *type* packages.  The body needs the matching codec packages so
    `From_Json` / `To_Json` calls resolve in the same namespaces.

    For known data-model packages we preserve the explicit mapping.  For any
    additional type package discovered by `_collect_type_pkgs`, derive the
    codec package name using the normal paired-package convention:

      Some_Package_Types -> Some_Package_Types_Codec
    """
    needed: List[str] = []
    for pkg in type_pkgs:
        codec_pkg = _DM_CODEC_PKG_FOR_TYPE_PKG.get(pkg)
        if codec_pkg is None:
            if pkg in _DATA_MODEL_TYPES_PKGS:
                # Keep existing special handling for built-in base types, which
                # do not require an imported codec package in generated service
                # bodies.
                continue
            codec_pkg = f'{pkg}_Codec'
        if codec_pkg not in needed:
            needed.append(codec_pkg)
    return needed


def _data_model_msg_pkgs(proto_path: Path) -> Dict[str, str]:
    """Map each data-model message short name to its proto package.

    Used to resolve standard-topic payload types (whose hardcoded FQNs target a
    canonical catalog) against the proto set actually being generated."""
    proto_root = _find_proto_root(proto_path)
    if proto_root is None:
        return {}
    try:
        files = [
            pf for pf in parse_proto_tree(proto_root)
            if pf.package.startswith('pyramid.data_model')
        ]
    except Exception:
        return {}
    result: Dict[str, str] = {}
    for pf in files:
        for msg in pf.messages:
            result.setdefault(msg.name, pf.package)
    return result


def _applicable_topics(
        parsed: 'ProtoFile', is_provided: bool, proto_path: Path,
        resolver: TopicSpecResolver,
) -> Tuple[Dict[str, str], Dict[str, str], List[str]]:
    """Filter standard topics to those whose payload type exists in the data
    model being generated, and return the Ada type packages those payloads need.

    Standard topics carry hardcoded payload FQNs (a canonical catalog). When a
    proto set does not define a topic's payload message (e.g. the PIM test tree
    has no ``ObjectEvidenceRequirement``), that topic's binding helpers cannot
    compile, so drop it. For payloads that do resolve, return the declaring
    package so the binding withs it even when no RPC references it directly.
    """
    sub_topics, pub_topics = resolver.topics_for_proto(parsed, is_provided)
    if not sub_topics and not pub_topics:
        return {}, {}, []
    msg_pkgs = _data_model_msg_pkgs(proto_path)
    type_pkgs: set = set()

    def keep(topics: Dict[str, str]) -> Dict[str, str]:
        out: Dict[str, str] = {}
        for key, wire in topics.items():
            short = resolver.spec(key).short_type
            pkg = msg_pkgs.get(short)
            if pkg is None:
                continue  # payload type absent from this proto set -- skip topic
            type_pkgs.add(_ada_pkg_from_proto_pkg(pkg))
            out[key] = wire
        return out

    return keep(sub_topics), keep(pub_topics), sorted(type_pkgs)


def _types_pkg_from_proto(proto_file: 'ProtoFile') -> str:
    """Return the primary Ada types package (tactical -- includes all types)."""
    return _DATA_MODEL_TYPES_PKG


def _flatbuffers_codec_pkg_from_proto(proto_file: ProtoFile) -> str:
    """Derive Ada FlatBuffers codec package name from proto package."""
    parts = proto_file.package.split('.')
    if parts and parts[-1].lower() in ('provided', 'consumed'):
        parts = parts[:-1]
    skip = {'pyramid', 'components', 'data_model', 'base', 'services'}
    meaningful = [p for p in parts if p.lower() not in skip]
    ada_parts = ['Pyramid', 'Services']
    for p in meaningful:
        ada_parts.append(_ada_pkg_segment(p))
    ada_parts.append('Flatbuffers_Codec')
    return '.'.join(ada_parts)


def _grpc_transport_pkg_from_proto(proto_file: ProtoFile) -> str:
    """Derive the generated Ada gRPC transport package for this service proto."""
    pkg_parts = [_ada_pkg_segment(p) for p in proto_file.package.split('.') if p]
    return '.'.join(pkg_parts) + '.GRPC_Transport'


def _ada_cabi_pkg_from_proto_pkg(proto_pkg: str) -> str:
    parts = proto_pkg.split('.')
    ada_parts = [_ada_pkg_segment(seg) for seg in parts]
    return '.'.join(ada_parts + ['Cabi'])


def _ada_cabi_type_name(message_name: str, package: str = '') -> str:
    """Ada C-ABI mirror record name, e.g. ``Pyramid_Data_Model_Common_Ack_C``.

    Mirrors ada_cabi_codegen._cabi_type_name (kept here to avoid a circular
    import) so the service bindings reference the same package-qualified C-ABI
    symbols the cabi generator emits (Decision 3 in the PIM handover)."""
    return _ada_name(_c_struct_name(message_name, package).removesuffix('_c')) + '_C'


def _service_wrapper_pf(proto_path: Path):
    """Parse the service proto and return its ProtoFile if it declares local
    wrapper messages (Component-NS oneof payloads), synthesizing an ``Empty``
    message where an RPC uses ``google.protobuf.Empty`` -- mirroring
    generate_bindings._discover_service_message_files so the marshalling
    bindings match the wrapper types/codec/cabi actually generated."""
    try:
        spf = parse_proto(proto_path)
    except Exception:
        return None
    if '.services.' not in f'.{spf.package}.':
        return None
    uses_empty = any(
        'google.protobuf.Empty' in (rpc.request_type, rpc.response_type)
        for svc in spf.services for rpc in svc.rpcs
    )
    if uses_empty and not any(m.name == 'Empty' for m in spf.messages):
        spf.messages.append(ProtoMessage(name='Empty'))
    if not spf.messages:
        return None
    return spf


def _binding_proto_files(proto_path: Path):
    """Data-model proto files plus the service's own wrapper file (if any).

    The wrapper messages are homed in their Component-NS and need the same
    native/codec/C-ABI marshalling treatment as data-model types."""
    proto_root = _find_proto_root(proto_path)
    if proto_root is None:
        return []
    try:
        files = [
            pf for pf in parse_proto_tree(proto_root)
            if pf.package.startswith('pyramid.data_model')
        ]
    except Exception:
        return []
    wrapper_pf = _service_wrapper_pf(proto_path)
    if wrapper_pf is not None:
        files.append(wrapper_pf)
    return files


def _collect_array_schema_bindings(
        cabi_bindings: List[Tuple[str, str, str, str]],
        all_rpcs: List[Tuple[str, 'ProtoRpc']],
        sub_topics: Dict[str, str],
        pub_topics: Dict[str, str],
        resolver: TopicSpecResolver) -> List[Tuple[str, str, str, str, str]]:
    """Return array payload bindings used by this facade.

    Each tuple is (array_schema_id, ada_array_type, ada_element_type,
    cabi_element_type, cabi_pkg).  Array schemas use the C++/plugin spelling
    ``<ProtoShortName>Array`` while Ada array types use ``<Ada_Element>_Array``.
    """
    by_schema = {schema: (native, cabi, pkg)
                 for schema, native, cabi, pkg in cabi_bindings}
    result: List[Tuple[str, str, str, str, str]] = []
    seen: set[str] = set()

    def add(short_type: str) -> None:
        if short_type in seen or short_type not in by_schema:
            return
        native, cabi, pkg = by_schema[short_type]
        result.append((short_type + 'Array',
                       native + '_Array',
                       native,
                       cabi,
                       pkg))
        seen.add(short_type)

    for key in list(sub_topics.keys()) + list(pub_topics.keys()):
        spec = resolver.spec(key)
        if spec.is_array:
            add(spec.short_type)

    for _svc_name, rpc in all_rpcs:
        if rpc.server_streaming:
            add(_short_type(rpc.response_type))

    return result


def _flatbuffers_func_suffix_for_type(full_type: str) -> str:
    """Return the Ada suffix used by the generated FlatBuffers bridge package."""
    return _camel_to_snake(_short_type(full_type))


def _flatbuffers_func_suffix_for_stream(full_type: str) -> str:
    """Return the Ada suffix for a streamed response array."""
    return _camel_to_snake(_short_type(full_type) + 'Array')


# -- Parent package stubs ------------------------------------------------------

def _ensure_parent_packages(output_dir: Path, pkg_names: List[str]) -> None:
    """Generate empty parent package specs required by Ada child packages.

    For a package like Pyramid.Services.Tactical_Objects.Provided, Ada requires
    that Pyramid, Pyramid.Services, and Pyramid.Services.Tactical_Objects each
    have a corresponding .ads file.  This function collects all such ancestors
    from *pkg_names* and writes trivial specs for any that don't already exist
    in *output_dir*.
    """
    needed: dict[str, None] = {}          # ordered set via dict
    for pkg in pkg_names:
        parts = pkg.split('.')
        for i in range(1, len(parts)):    # skip full name, keep ancestors
            ancestor = '.'.join(parts[:i])
            needed[ancestor] = None

    for ancestor in needed:
        file_base = ancestor.lower().replace('.', '-')
        ads = output_dir / (file_base + '.ads')
        if ads.exists():
            continue
        with open(ads, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'--  Auto-generated parent package spec (empty)\n')
            f.write(f'package {ancestor} is\n')
            f.write(f'end {ancestor};\n')
        print(f'  Generated parent spec {ancestor}')


# ---------------------------------------------------------------------------
# Ada types package generator
# ---------------------------------------------------------------------------

_ADA_SCALAR_MAP: Dict[str, str] = {
    'double': 'Long_Float', 'float': 'Float',
    'int32': 'Integer', 'int64': 'Long_Integer',
    'uint32': 'Natural', 'uint64': 'Long_Integer',
    'sint32': 'Integer', 'sint64': 'Long_Integer',
    'fixed32': 'Natural', 'fixed64': 'Long_Integer',
    'sfixed32': 'Integer', 'sfixed64': 'Long_Integer',
    'bool': 'Boolean', 'string': 'Unbounded_String', 'bytes': 'Unbounded_String',
}

_ADA_DEFAULTS: Dict[str, str] = {
    'Long_Float': '0.0', 'Float': '0.0',
    'Integer': '0', 'Long_Integer': '0', 'Natural': '0',
    'Boolean': 'False', 'Unbounded_String': 'Null_Unbounded_String',
}

_ADA_UNIT_FIELD_NAMES = frozenset({
    'value', 'radians', 'meters', 'meters_per_second', 'seconds',
})

_ADA_RESERVED_WORDS = frozenset({
    'abort', 'abs', 'abstract', 'accept', 'access', 'aliased', 'all', 'and',
    'array', 'at', 'begin', 'body', 'case', 'constant', 'declare', 'delay',
    'delta', 'digits', 'do', 'else', 'elsif', 'end', 'entry', 'exception',
    'exit', 'for', 'function', 'generic', 'goto', 'if', 'in', 'interface',
    'is', 'limited', 'loop', 'mod', 'new', 'not', 'null', 'of', 'or',
    'others', 'out', 'overriding', 'package', 'pragma', 'private',
    'procedure', 'protected', 'raise', 'range', 'record', 'rem', 'renames',
    'requeue', 'return', 'reverse', 'select', 'separate', 'subtype',
    'synchronized', 'tagged', 'task', 'terminate', 'then', 'type', 'until',
    'use', 'when', 'while', 'with', 'xor',
})


def _ada_name(cpp_or_proto_name: str) -> str:
    """Convert CamelCase or snake_case proto name to Ada Title_Case."""
    s = camel_to_snake(cpp_or_proto_name)   # CamelCase -> Camel_Case
    return '_'.join(w.capitalize() for w in s.split('_'))


def _ada_array_name_for_repeated(field_type: str, field_name: str) -> str:
    """Return the native Ada array type name for a repeated field."""
    del field_name
    return _ada_name(field_type.split('.')[-1]) + '_Array'


def _ada_pkg_segment(proto_segment: str) -> str:
    """Return a legal Ada package identifier segment for a proto package part."""
    name = _ada_name(proto_segment)
    if not name:
        return 'Pkg'
    if name.lower() in _ADA_RESERVED_WORDS:
        return name + '_Pkg'
    return name


def _ada_field_name(proto_name: str, ada_type: Optional[str] = None) -> str:
    """Return a legal Ada record/component identifier for a proto field name."""
    name = _ada_name(proto_name)
    if name.lower() in _ADA_RESERVED_WORDS:
        name = name + '_Field'
    if ada_type is not None and name == ada_type:
        name = 'Val_' + name
    return name


def _common_ada_pkg(index: ProtoTypeIndex) -> str:
    """Derive Ada package name from common package prefix of data model protos.

    e.g. pyramid.data_model.base, pyramid.data_model.common
      -> Pyramid_Data_Model_Types
    """
    pkgs = [pf.package for pf in index.files if pf.package]
    if not pkgs:
        return 'Data_Model_Types'
    parts = [p.split('.') for p in pkgs]
    common = parts[0]
    for p in parts[1:]:
        common = [a for a, b in zip(common, p) if a == b]
    ada_parts = [_ada_pkg_segment(seg) for seg in common]
    return '_'.join(ada_parts) + '_Types'


