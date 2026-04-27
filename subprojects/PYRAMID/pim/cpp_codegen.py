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
    parse_proto_tree, ProtoTypeIndex, ProtoMessage, ProtoEnum,
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

_DATA_MODEL_TYPES_NS = 'pyramid::data_model'

def _namespace_from_proto(proto_file: ProtoFile) -> Tuple[str, str, str, str]:
    """Return (full_namespace, file_prefix, svc_base_ns, types_namespace).

    pyramid.components.tactical_objects.services.provided
      -> full_ns    : pyramid::services::tactical_objects::provided
      -> file_prefix: pyramid_services_tactical_objects_provided
      -> svc_base_ns: pyramid::services::tactical_objects
      -> types_ns   : pyramid::data_model
    """
    parts = proto_file.package.split('.')

    last = parts[-1].lower()
    suffix = None
    if last in ('provided', 'consumed'):
        suffix = last
        parts = parts[:-1]

    skip = {'pyramid', 'components', 'data_model', 'base', 'services'}
    meaningful = [p for p in parts if p.lower() not in skip]

    ns_parts = ['pyramid', 'services'] + [p.lower() for p in meaningful]
    suffix = suffix or 'provided'

    svc_base_ns = '::'.join(ns_parts)
    full_ns = svc_base_ns + '::' + suffix
    file_prefix = '_'.join(ns_parts + [suffix])

    return full_ns, file_prefix, svc_base_ns, _DATA_MODEL_TYPES_NS


def _is_provided(proto_file: ProtoFile) -> bool:
    return 'provided' in proto_file.package.lower()


def _topics_for_proto(
        parsed: 'ProtoFile', is_provided: bool
) -> Tuple[Dict[str, str], Dict[str, str]]:
    """Return (sub_topics, pub_topics) for the service, based on its package."""
    return topics_for_service(parsed.package, is_provided)


# -- Code generation -----------------------------------------------------------

class CppServiceGenerator:

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
        conversion helpers.  Those exist today as checked-in service codecs for
        tactical_objects; do not emit service protobuf dispatch code for service
        packages that do not have that bridge.
        """
        header = '_'.join(svc_base_ns.split('::')) + '_protobuf_codec.hpp'
        candidates = []
        for parent in [self._proto_input, *self._proto_input.parents]:
            if parent.is_file():
                parent = parent.parent
            candidates.append(parent / 'bindings' / 'protobuf' / 'cpp' / header)
        return any(path.exists() for path in candidates)

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
            types_header = '_'.join(types_ns.split('::')) + '_types.hpp'

            hpp_path = output_path / (file_prefix + '.hpp')
            cpp_path = output_path / (file_prefix + '.cpp')

            self._write_header(hpp_path, full_ns, types_ns, types_header,
                               parsed, all_rpcs)
            self._write_impl(cpp_path, file_prefix, full_ns, types_ns,
                             parsed, all_rpcs)
            print(f'  Generated {full_ns}')

    # -- Header (.hpp) ---------------------------------------------------------

    def _write_header(self, path: Path, full_ns: str, types_ns: str,
                      types_header: str, parsed: ProtoFile,
                      all_rpcs: List[Tuple[str, ProtoRpc]]):
        is_provided = _is_provided(parsed)
        has_grpc = self._has_backend('grpc')
        has_ros2 = self._has_backend('ros2')
        sub_topics, pub_topics = _topics_for_proto(parsed, is_provided)
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)
        topic_set = all_topics

        # Collect raw type names (no BASE_TYPE_MAP) for using declarations
        raw_types = sorted({
            t
            for _, rpc in all_rpcs
            for t in (rpc.raw_req_type, rpc.raw_rsp_type)
        } | {
            topic_spec(key).short_type
            for key in topic_set
        })

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
            f.write(f'#include "{types_header}"\n\n')
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
            const_names = [rpc.cpp_svc_const
                           for svc in parsed.services
                           for rpc in svc.rpcs]
            max_const = max((len(n) for n in const_names), default=0)

            for svc in parsed.services:
                for rpc in svc.rpcs:
                    wire = f'{svc.wire_prefix}.{rpc.wire_name}'
                    pad = max_const - len(rpc.cpp_svc_const)
                    f.write(f'constexpr const char* {rpc.cpp_svc_const}'
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
            for _, rpc in all_rpcs:
                f.write(f'    {rpc.cpp_enum_value},\n')
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

            for t in raw_types:
                f.write(f'using {types_ns}::{t};\n')
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
                f.write(f'    {rpc.cpp_handler}(const {rpc.cpp_req_type}& request);\n')
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
                    decode_col = len(f'bool {rpc.cpp_decode_response_func}(')
                    decode_sp = ' ' * decode_col
                    f.write(f'/// \\brief Decode a response from {wire_full}.\n')
                    f.write(f'bool {rpc.cpp_decode_response_func}'
                            f'(const pcl_msg_t* msg,\n')
                    f.write(f'{decode_sp}{rsp_decl_t}* out);\n\n')
                    req_decl_t = rpc.cpp_req_type
                    col = 13 + len(rpc.cpp_invoke_func) + 1
                    sp = ' ' * col
                    f.write(f'/// \\brief Invoke {wire_full}'
                            f' (typed, serialisation handled internally).\n')
                    f.write(f'///\n')
                    f.write(f'/// Uses the configured endpoint route, or the legacy\n')
                    f.write(f'/// executor transport fallback when no route is supplied.\n')
                    f.write(f'pcl_status_t {rpc.cpp_invoke_func}'
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
                    f.write(f'pcl_status_t {rpc.cpp_invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}const char*       content_type'
                            f' = "{_DEFAULT_CONTENT_TYPE}",\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route'
                            f' = nullptr);\n\n')

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
                        f.write(f'    pyramid::transport::ros2::{bind_func}'
                                f'(adapter, executor, {rpc.cpp_svc_const});\n')
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

            # Namespace close
            f.write(f'}} // namespace {full_ns}\n')

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

        # Determine which data model codec namespaces to use. Service request
        # and response payloads can reference any data-model package, so bring
        # every generated data-model codec into scope rather than hard-coding
        # the current Tactical Objects subset.
        dm_codec_nss = []
        dm_codec_headers = []
        if self._proto_input.is_dir():
            data_model_files = [
                parse_proto(p) for p in self._proto_input.rglob('*.proto')
            ]
        else:
            data_model_files = []
            for parent in self._proto_input.parents:
                candidate = parent / 'pyramid' / 'data_model'
                if candidate.exists():
                    data_model_files = [
                        parse_proto(p) for p in candidate.rglob('*.proto')
                    ]
                    break
            if not data_model_files:
                data_model_files = [parsed]
        for indexed_pf in data_model_files:
            if not indexed_pf.package.startswith('pyramid.data_model'):
                continue
            if indexed_pf.package == 'pyramid.data_model.base':
                continue
            dm_codec_nss.append(indexed_pf.package.replace('.', '::'))
            dm_codec_headers.append(
                f'{indexed_pf.package.replace(".", "_")}_codec.hpp')

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            # File-level comment block
            f.write('// Auto-generated service binding implementation\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n\n')

            # Includes
            f.write(f'#include "{hpp_name}"\n\n')
            if has_flatbuffers:
                f.write(f'#include "{flatbuffers_codec_header}"\n')
            if has_protobuf:
                f.write(f'#include "{protobuf_codec_header}"\n')
            # Data model codec headers -- for serialisation inside
            # invoke/publish/dispatch
            for ch in dm_codec_headers:
                f.write(f'#include "{ch}"\n')
            f.write('\n')
            f.write('#include <pcl/pcl_container.h>\n')
            f.write('#include <pcl/pcl_executor.h>\n')
            f.write('#include <pcl/pcl_transport.h>\n')
            f.write('\n#include <cstdlib>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <cstring>\n')
            f.write('#include <nlohmann/json.hpp>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')

            # Namespace open
            f.write(f'namespace {full_ns} {{\n\n')

            # Bring codec functions into scope for unqualified calls
            f.write('// Bring data model codec functions into scope\n')
            for ns in dm_codec_nss:
                f.write(f'using {ns}::toJson;\n')
                f.write(f'using {ns}::fromJson;\n')
            if has_flatbuffers:
                f.write(f'namespace flatbuffers_codec = {flatbuffers_codec_ns};\n')
            if has_protobuf:
                f.write(f'namespace protobuf_codec = {protobuf_codec_ns};\n')
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

            for _svc_name, rpc in all_rpcs:
                rsp_t = rpc.cpp_rsp_type
                req_t = rpc.cpp_req_type

                f.write(f'{rsp_t}\n')
                f.write(f'ServiceHandler::{rpc.cpp_handler}'
                        f'(const {req_t}& /*request*/) {{\n')

                if rsp_t == 'Ack':
                    f.write(f'    return {types_ns}::kAckOk;\n')
                else:
                    f.write('    return {};\n')

                f.write('}\n\n')

            # ---- Internal PCL helpers (anonymous namespace) ------------------
            f.write(_SEP + '\n')
            f.write('// Internal PCL helpers\n')
            f.write(_SEP + '\n\n')
            f.write('namespace {\n\n')

            f.write('bool is_json_content_type(const char* content_type)\n')
            f.write('{\n')
            f.write('    return !content_type || std::strcmp(content_type, kJsonContentType) == 0;\n')
            f.write('}\n\n')

            if has_flatbuffers:
                f.write('bool is_flatbuffers_content_type(const char* content_type)\n')
                f.write('{\n')
                f.write('    return content_type && std::strcmp(content_type, kFlatBuffersContentType) == 0;\n')
                f.write('}\n\n')

            if has_protobuf:
                f.write('bool is_protobuf_content_type(const char* content_type)\n')
                f.write('{\n')
                f.write('    return content_type && std::strcmp(content_type, kProtobufContentType) == 0;\n')
                f.write('}\n\n')

            f.write('std::string json_request_body(const void* data, size_t size)\n')
            f.write('{\n')
            f.write('    if (!data && size != 0) return {};\n')
            f.write('    return std::string(static_cast<const char*>(data), size);\n')
            f.write('}\n\n')

            f.write('std::string encode_identifier_payload(const Identifier& value)\n')
            f.write('{\n')
            f.write('    return nlohmann::json(value).dump();\n')
            f.write('}\n\n')

            f.write('Identifier decode_identifier_payload(const std::string& payload)\n')
            f.write('{\n')
            f.write('    if (payload.empty()) {\n')
            f.write('        return {};\n')
            f.write('    }\n')
            f.write('    try {\n')
            f.write('        auto j = nlohmann::json::parse(payload);\n')
            f.write('        if (j.is_string()) {\n')
            f.write('            return j.get<std::string>();\n')
            f.write('        }\n')
            f.write('        if (j.is_object() && j.contains("uuid") && j["uuid"].is_string()) {\n')
            f.write('            return j["uuid"].get<std::string>();\n')
            f.write('        }\n')
            f.write('    } catch (...) {\n')
            f.write('    }\n')
            f.write('    return payload;\n')
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
            f.write('    std::vector<const char*> result{kJsonContentType};\n')
            if has_flatbuffers:
                f.write('    result.push_back(kFlatBuffersContentType);\n')
            if has_protobuf:
                f.write('    result.push_back(kProtobufContentType);\n')
            f.write('    return result;\n')
            f.write('}\n\n')
            f.write('bool supportsContentType(const char* content_type)\n')
            f.write('{\n')
            f.write('    if (is_json_content_type(content_type)) {\n')
            f.write('        return true;\n')
            f.write('    }\n')
            if has_flatbuffers:
                f.write('    if (is_flatbuffers_content_type(content_type)) {\n')
                f.write('        return true;\n')
                f.write('    }\n')
            if has_protobuf:
                f.write('    if (is_protobuf_content_type(content_type)) {\n')
                f.write('        return true;\n')
                f.write('    }\n')
            f.write('    return false;\n')
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
                    col = len(f'bool {rpc.cpp_decode_response_func}(')
                    sp = ' ' * col
                    f.write(f'bool {rpc.cpp_decode_response_func}'
                            f'(const pcl_msg_t* msg,\n')
                    f.write(f'{sp}{rsp_decl_t}* out)\n')
                    f.write('{\n')
                    f.write('    if (!msg || !msg->data || msg->size == 0 || !out) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    f.write('    try {\n')
                    f.write('        if (!is_json_content_type(msg->type_name)) {\n')
                    if has_flatbuffers:
                        f.write('            if (is_flatbuffers_content_type(msg->type_name)) {\n')
                        if rpc.streaming:
                            f.write(f'                *out = flatbuffers_codec::fromBinary{rsp_raw_t}Array(msg->data, msg->size);\n')
                        else:
                            f.write(f'                *out = flatbuffers_codec::fromBinary{rsp_raw_t}(msg->data, msg->size);\n')
                        f.write('                return true;\n')
                        f.write('            }\n')
                    if has_protobuf:
                        f.write('            if (is_protobuf_content_type(msg->type_name)) {\n')
                        if rpc.streaming:
                            f.write(f'                *out = protobuf_codec::fromBinary{rsp_raw_t}Array(msg->data, msg->size);\n')
                        else:
                            f.write(f'                *out = protobuf_codec::fromBinary{rsp_raw_t}(msg->data, msg->size);\n')
                        f.write('                return true;\n')
                        f.write('            }\n')
                    f.write('            return false;\n')
                    f.write('        }\n')
                    f.write('        const std::string payload = msgToString(msg->data, msg->size);\n')
                    if rpc.streaming:
                        f.write('        const auto arr = nlohmann::json::parse(payload);\n')
                        f.write('        out->clear();\n')
                        f.write('        for (const auto& item : arr) {\n')
                        if rsp_decl_t == 'std::vector<Identifier>':
                            f.write('            out->push_back(decode_identifier_payload(item.dump()));\n')
                        else:
                            f.write(f'            out->push_back(fromJson(item.dump(), static_cast<{rsp_raw_t}*>(nullptr)));\n')
                        f.write('        }\n')
                    else:
                        if rsp_decl_t == 'Identifier':
                            f.write('        *out = decode_identifier_payload(payload);\n')
                        else:
                            f.write(f'        *out = fromJson(payload, static_cast<{rsp_raw_t}*>(nullptr));\n')
                    f.write('        return true;\n')
                    f.write('    } catch (...) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    f.write('}\n\n')

            # Typed invoke wrappers -- serialize request, then PCL
            f.write(_SEP + '\n')
            f.write('// Typed invoke wrappers \u2014 serialise and dispatch via executor transport\n')
            f.write(_SEP + '\n\n')
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    req_decl_t = rpc.cpp_req_type
                    col = 13 + len(rpc.cpp_invoke_func) + 1
                    sp = ' ' * col
                    f.write(f'pcl_status_t {rpc.cpp_invoke_func}'
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
                    f.write('    if (is_json_content_type(content_type)) {\n')
                    if req_decl_t == 'Identifier':
                        f.write('        payload = encode_identifier_payload(request);\n')
                    else:
                        f.write('        payload = toJson(request);\n')
                    if has_flatbuffers:
                        f.write('    } else if (is_flatbuffers_content_type(content_type)) {\n')
                        f.write('        payload = flatbuffers_codec::toBinary(request);\n')
                    if has_protobuf:
                        f.write('    } else if (is_protobuf_content_type(content_type)) {\n')
                        f.write('        payload = protobuf_codec::toBinary(request);\n')
                    f.write('    } else {\n')
                    f.write('        return PCL_ERR_INVALID;\n')
                    f.write('    }\n')
                    f.write('    return invoke_async(executor,'
                            f' {rpc.cpp_svc_const},'
                            f' payload, callback, user_data, route, content_type);\n')
                    f.write('}\n\n')
                    f.write(f'pcl_status_t {rpc.cpp_invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}const char*       content_type,\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route)\n')
                    f.write('{\n')
                    f.write(f'    return {rpc.cpp_invoke_func}'
                            '(executor, request, ignore_async_response,\n')
                    f.write('                         nullptr, route, content_type);\n')
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
                    f.write('    std::string wire_payload;\n')
                    f.write('    if (is_json_content_type(content_type)) {\n')
                    if spec.is_array:
                        f.write('        wire_payload = "[";\n')
                        f.write('        bool first = true;\n')
                        f.write('        for (const auto& item : payload) {\n')
                        f.write('            if (!first) wire_payload += ",";\n')
                        f.write('            first = false;\n')
                        f.write('            wire_payload += toJson(item);\n')
                        f.write('        }\n')
                        f.write('        wire_payload += "]";\n')
                    else:
                        if spec.short_type == 'Identifier':
                            f.write('        wire_payload = encode_identifier_payload(payload);\n')
                        else:
                            f.write('        wire_payload = toJson(payload);\n')
                    if has_flatbuffers:
                        f.write('    } else if (is_flatbuffers_content_type(content_type)) {\n')
                        f.write('        wire_payload = flatbuffers_codec::toBinary(payload);\n')
                    if has_protobuf:
                        f.write('    } else if (is_protobuf_content_type(content_type)) {\n')
                        f.write('        wire_payload = protobuf_codec::toBinary(payload);\n')
                    f.write('    } else {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    f.write('    *out = wire_payload;\n')
                    f.write('    return true;\n')
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
                    f.write('    try {\n')
                    f.write('        if (!is_json_content_type(msg->type_name)) {\n')
                    if has_flatbuffers:
                        f.write('            if (is_flatbuffers_content_type(msg->type_name)) {\n')
                        if spec.is_array:
                            f.write(f'                *out = flatbuffers_codec::fromBinary{spec.short_type}Array(msg->data, msg->size);\n')
                        else:
                            f.write(f'                *out = flatbuffers_codec::fromBinary{spec.short_type}(msg->data, msg->size);\n')
                        f.write('                return true;\n')
                        f.write('            }\n')
                    if has_protobuf:
                        f.write('            if (is_protobuf_content_type(msg->type_name)) {\n')
                        if spec.is_array:
                            f.write(f'                *out = protobuf_codec::fromBinary{spec.short_type}Array(msg->data, msg->size);\n')
                        else:
                            f.write(f'                *out = protobuf_codec::fromBinary{spec.short_type}(msg->data, msg->size);\n')
                        f.write('                return true;\n')
                        f.write('            }\n')
                    f.write('            return false;\n')
                    f.write('        }\n')
                    f.write('        const std::string payload = msgToString(msg->data, msg->size);\n')
                    if spec.is_array:
                        f.write('        const auto arr = nlohmann::json::parse(payload);\n')
                        f.write('        out->clear();\n')
                        f.write('        for (const auto& item : arr) {\n')
                        if spec.short_type == 'Identifier':
                            f.write('            out->push_back(decode_identifier_payload(item.dump()));\n')
                        else:
                            f.write(f'            out->push_back(fromJson(item.dump(), static_cast<{spec.short_type}*>(nullptr)));\n')
                        f.write('        }\n')
                    else:
                        if spec.short_type == 'Identifier':
                            f.write('        *out = decode_identifier_payload(payload);\n')
                        else:
                            f.write(f'        *out = fromJson(payload, static_cast<{spec.short_type}*>(nullptr));\n')
                    f.write('        return true;\n')
                    f.write('    } catch (...) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
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
            f.write('    std::string req_str;\n')
            f.write('    std::string rsp_payload;\n\n')
            f.write('    bool rsp_is_binary = false;\n\n')
            f.write('    if (is_json_content_type(content_type)) {\n')
            f.write('        req_str = json_request_body(request_buf, request_size);\n')
            f.write('        if (request_size != 0 && req_str.empty()) {\n')
            f.write('            *response_buf = nullptr;\n')
            f.write('            *response_size = 0;\n')
            f.write('            return;\n')
            f.write('        }\n')
            if has_flatbuffers:
                f.write('    } else if (is_flatbuffers_content_type(content_type)) {\n')
            if has_protobuf:
                f.write('    } else if (is_protobuf_content_type(content_type)) {\n')
            f.write('    } else {\n')
            f.write('        *response_buf = nullptr;\n')
            f.write('        *response_size = 0;\n')
            f.write('        return;\n')
            f.write('    }\n\n')
            f.write('    try {\n')
            f.write('    switch (channel) {\n')

            for _, rpc in all_rpcs:
                enum_val = rpc.cpp_enum_value
                handler_fn = rpc.cpp_handler
                req_t = rpc.cpp_req_type
                rsp_t = rpc.cpp_rsp_type

                f.write(f'    case ServiceChannel::{enum_val}: {{\n')
                # Deserialize request
                if req_t == 'Identifier':
                    f.write(f'        {req_t} req;\n')
                    f.write('        if (is_json_content_type(content_type))\n')
                    f.write('            req = decode_identifier_payload(req_str);\n')
                    if has_flatbuffers:
                        f.write('        else if (is_flatbuffers_content_type(content_type))\n')
                        f.write(f'            req = flatbuffers_codec::fromBinary{req_t}(request_buf, request_size);\n')
                    if has_protobuf:
                        f.write('        else if (is_protobuf_content_type(content_type))\n')
                        f.write(f'            req = protobuf_codec::fromBinary{req_t}(request_buf, request_size);\n')
                    f.write('        else\n')
                    f.write('            break;\n')
                else:
                    f.write(f'        {req_t} req;\n')
                    f.write('        if (is_json_content_type(content_type))\n')
                    f.write(f'            req = fromJson(req_str, static_cast<{req_t}*>(nullptr));\n')
                    if has_flatbuffers:
                        f.write('        else if (is_flatbuffers_content_type(content_type))\n')
                        f.write(f'            req = flatbuffers_codec::fromBinary{req_t}(request_buf, request_size);\n')
                    if has_protobuf:
                        f.write('        else if (is_protobuf_content_type(content_type))\n')
                        f.write(f'            req = protobuf_codec::fromBinary{req_t}(request_buf, request_size);\n')
                    f.write('        else\n')
                    f.write('            break;\n')
                f.write(f'        auto rsp = handler.{handler_fn}(req);\n')

                # Serialize response
                if rsp_t.startswith('std::vector<'):
                    inner = rsp_t[len('std::vector<'):-1]
                    f.write('        if (is_json_content_type(content_type)) {\n')
                    f.write('            rsp_payload = "[";\n')
                    f.write('            for (size_t i = 0; i < rsp.size(); ++i) {\n')
                    f.write('                if (i > 0) rsp_payload += ",";\n')
                    if inner == 'Identifier':
                        f.write('                rsp_payload += encode_identifier_payload(rsp[i]);\n')
                    else:
                        f.write('                rsp_payload += toJson(rsp[i]);\n')
                    f.write('            }\n')
                    f.write('            rsp_payload += "]";\n')
                    if has_flatbuffers:
                        f.write('        } else if (is_flatbuffers_content_type(content_type)) {\n')
                        f.write('            rsp_payload = flatbuffers_codec::toBinary(rsp);\n')
                        f.write('            rsp_is_binary = true;\n')
                    if has_protobuf:
                        f.write('        } else if (is_protobuf_content_type(content_type)) {\n')
                        f.write('            rsp_payload = protobuf_codec::toBinary(rsp);\n')
                        f.write('            rsp_is_binary = true;\n')
                    f.write('        } else {\n')
                    f.write('            break;\n')
                    f.write('        }\n')
                elif rsp_t == 'Identifier':
                    f.write('        if (is_json_content_type(content_type)) {\n')
                    f.write('            rsp_payload = encode_identifier_payload(rsp);\n')
                    if has_flatbuffers:
                        f.write('        } else if (is_flatbuffers_content_type(content_type)) {\n')
                        f.write('            rsp_payload = flatbuffers_codec::toBinary(rsp);\n')
                        f.write('            rsp_is_binary = true;\n')
                    if has_protobuf:
                        f.write('        } else if (is_protobuf_content_type(content_type)) {\n')
                        f.write('            rsp_payload = protobuf_codec::toBinary(rsp);\n')
                        f.write('            rsp_is_binary = true;\n')
                    f.write('        } else {\n')
                    f.write('            break;\n')
                    f.write('        }\n')
                else:
                    f.write('        if (is_json_content_type(content_type)) {\n')
                    f.write('            rsp_payload = toJson(rsp);\n')
                    if has_flatbuffers:
                        f.write('        } else if (is_flatbuffers_content_type(content_type)) {\n')
                        f.write('            rsp_payload = flatbuffers_codec::toBinary(rsp);\n')
                        f.write('            rsp_is_binary = true;\n')
                    if has_protobuf:
                        f.write('        } else if (is_protobuf_content_type(content_type)) {\n')
                        f.write('            rsp_payload = protobuf_codec::toBinary(rsp);\n')
                        f.write('            rsp_is_binary = true;\n')
                    f.write('        } else {\n')
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

            f.write('    (void) rsp_is_binary;\n\n')

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

            # Namespace close
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


def _common_cpp_ns(index: ProtoTypeIndex) -> str:
    """Derive C++ namespace from common package prefix of data model protos.

    e.g. pyramid.data_model.base, pyramid.data_model.common -> pyramid::data_model
    """
    pkgs = [pf.package for pf in index.files if pf.package]
    if not pkgs:
        return 'data_model'
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

    def __init__(self, data_model_dir: Path):
        proto_files = parse_proto_tree(data_model_dir)
        self._index = ProtoTypeIndex(proto_files)
        self._data_model_dir = data_model_dir
        self._ns = _common_cpp_ns(self._index)
        self._prefix = '_'.join(self._ns.split('::'))
        self._aliases = self._find_scalar_wrappers()

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
        umbrella = out / (self._prefix + '_types.hpp')
        self._write_umbrella_hpp(umbrella)
        print(f'  Generated {self._ns} (umbrella)')

    # -- internal --------------------------------------------------------------

    def _find_scalar_wrappers(self) -> Dict[str, str]:
        """Return {message_name: cpp_scalar_type} for transparent wrappers.

        Only messages whose single field has a unit-style name (e.g. "radians",
        "meters", "value") are inlined as aliases.  Domain fields like "success"
        are left as structs so their enclosing type retains its semantics.
        """
        aliases: Dict[str, str] = dict(_FORCED_ALIASES)
        for msg in self._index.all_messages():
            fields = msg.all_fields()
            if len(fields) == 1 and not fields[0].is_repeated:
                ft = fields[0].type
                fn = fields[0].name
                if ft in _CPP_SCALAR_MAP and fn in _UNIT_FIELD_NAMES:
                    aliases[msg.name] = _CPP_SCALAR_MAP[ft]
        return aliases

    def _package_of_type(self, name: str) -> str:
        """Return the proto package that defines a type (short name lookup)."""
        short = name.split('.')[-1]
        for pf in self._index.files:
            for msg in pf.messages:
                if msg.name == short:
                    return pf.package
            for enum in pf.enums:
                if enum.name == short:
                    return pf.package
        return ''

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
                base = pkg.replace('.', '::') + '::' + short
            else:
                base = short
        elif (self._index.is_enum_type(field_type)
              or self._index.is_message_type(field_type)):
            # Short name -- look up its package for cross-package qualification
            pkg = self._package_of_type(field_type)
            if current_pkg and pkg and pkg != current_pkg:
                base = pkg.replace('.', '::') + '::' + field_type
            else:
                base = field_type
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
        if self._index.is_enum_type(field_type) or self._index.is_enum_type(short):
            enum = (self._index.resolve_enum(field_type)
                    or self._index.resolve_enum(short))
            if enum and enum.values:
                suf = enum.suffix_of(enum.values[0].name)
                lit = screaming_to_pascal(suf) if suf else enum.values[0].name
                if '.' in field_type and not field_type.startswith('google.'):
                    pkg = '.'.join(field_type.split('.')[:-1])
                    if current_pkg and pkg != current_pkg:
                        return f'{pkg.replace(".", "::")}::{short}::{lit}'
                else:
                    pkg = self._package_of_type(field_type)
                    if current_pkg and pkg and pkg != current_pkg:
                        return f'{pkg.replace(".", "::")}::{field_type}::{lit}'
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

    def _inline_base_fields(self, msg: ProtoMessage):
        """Yield (field, comment) for all non-base fields, inlining any 'base'
        fields by expanding their sub-fields directly into the parent struct.

        Only one level of inlining is done; nested 'base' fields are kept as-is.
        """
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                short = field.type.split('.')[-1]
                base_msg = self._index.resolve_message(field.type) or \
                           self._index.resolve_message(short)
                if base_msg and base_msg.name not in self._aliases:
                    # inline the base's own fields with collision renaming
                    for bf in base_msg.fields:
                        name = bf.name
                        if name in own_names:
                            name = short.lower() + '_' + name
                        yield bf, name, f'  // from {base_msg.name}'
                    continue
            yield field, field.name, ''

    def _write_struct(self, f, msg: ProtoMessage,
                      current_pkg: str = '') -> None:
        f.write(f'struct {msg.name} {{\n')
        for field, fname, comment in self._inline_base_fields(msg):
            base_cpp = self._cpp_field_type(field.type, False, current_pkg)
            if field.is_repeated:
                cpp_type = f'std::vector<{base_cpp}>'
                default = '{}'
                opt = ''
            elif field.is_optional and base_cpp not in ('std::string',):
                cpp_type = f'std::optional<{base_cpp}>'
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
                f.write(f'    std::optional<{cpp_type}> {field.name};\n')
        f.write('};\n')
        for const_name, init in _STRUCT_CONSTANTS.get(msg.name, []):
            body = init[len(msg.name):] if init.startswith(msg.name) else (
                '{ ' + init + ' }')
            f.write('constexpr ' + msg.name + ' ' + const_name + body + ';\n')
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
        ns = pf.package.replace('.', '::')
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
            f.write('#include <optional>\n')
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
        everything into the common pyramid::data_model namespace for backward
        compatibility with code that uses that single namespace."""
        per_file_headers = sorted(
            pf.package.replace('.', '_') + '_types.hpp'
            for pf in self._index.files
        )
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated umbrella types header\n')
            f.write('// Includes all data model type headers and re-exports\n')
            f.write('// their contents into namespace pyramid::data_model.\n')
            f.write('#pragma once\n\n')
            for h in per_file_headers:
                f.write(f'#include "{h}"\n')
            f.write('\n// Re-export all sub-namespace types into the common namespace\n')
            f.write('// so existing code using pyramid::data_model::T continues to work.\n')
            f.write(f'namespace {self._ns} {{\n')
            for pf in self._index.files:
                ns = pf.package.replace('.', '::')
                if ns != self._ns:
                    f.write(f'using namespace {ns};\n')
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
        self._ns = pf.package.replace('.', '::')
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
                    f.write(f'std::string toString({t} v);\n')
                    f.write(f'{t} {fn}FromString(const std::string& s);\n')
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
            for imp in self._pf.imports:
                if imp.startswith('google/'):
                    continue
                pkg = imp.replace('/', '.').removesuffix('.proto')
                imp_stem = Path(imp).stem
                for indexed_pf in self._index.files:
                    if indexed_pf.package == pkg or indexed_pf.path.stem == imp_stem:
                        f.write(f'#include "{indexed_pf.package.replace(".", "_")}_codec.hpp"\n')
                        break
            f.write(f'\nnamespace {self._ns} {{\n\n')

            # Enum converters
            for enum in self._pf.enums:
                t = enum.name
                fn = _lc_first(t)
                f.write(f'std::string toString({t} v) {{\n')
                f.write(f'    switch (v) {{\n')
                for v in enum.values:
                    suf = enum.suffix_of(v.name)
                    lit = screaming_to_pascal(suf) if suf else v.name
                    f.write(f'        case {t}::{lit}: return "{v.name}";\n')
                f.write(f'    }}\n')
                first_suf = enum.suffix_of(enum.values[0].name)
                first_lit = screaming_to_pascal(first_suf) if first_suf else enum.values[0].name
                f.write(f'    return "{enum.values[0].name}";\n')
                f.write(f'}}\n\n')

                f.write(f'{t} {fn}FromString(const std::string& s) {{\n')
                for v in enum.values:
                    suf = enum.suffix_of(v.name)
                    lit = screaming_to_pascal(suf) if suf else v.name
                    f.write(f'    if (s == "{v.name}") return {t}::{lit};\n')
                first_suf = enum.suffix_of(enum.values[0].name)
                first_lit = screaming_to_pascal(first_suf) if first_suf else enum.values[0].name
                f.write(f'    return {t}::{first_lit};\n')
                f.write(f'}}\n\n')

            # Struct codec implementations
            for msg in structs:
                self._write_to_json(f, msg, current_pkg)
                self._write_from_json(f, msg, current_pkg)

            f.write(f'}} // namespace {self._ns}\n')

    def _package_of_type(self, name: str) -> str:
        """Return the proto package that defines a type."""
        short = name.split('.')[-1]
        for pf in self._index.files:
            for msg in pf.messages:
                if msg.name == short:
                    return pf.package
            for enum in pf.enums:
                if enum.name == short:
                    return pf.package
        return ''

    def _qualify(self, type_name: str, current_pkg: str) -> str:
        """Return qualified C++ name for type_name relative to current_pkg."""
        short = type_name.split('.')[-1]
        if '.' in type_name and not type_name.startswith('google.'):
            pkg = '.'.join(type_name.split('.')[:-1])
        else:
            pkg = self._package_of_type(type_name)
        if pkg and pkg != current_pkg:
            return pkg.replace('.', '::') + '::' + short
        return short

    def _field_info(self, field, fname: str, current_pkg: str):
        """Return (base_cpp_type, is_struct, is_enum, is_alias) for a field."""
        short = field.type.split('.')[-1]
        if field.type in _CPP_SCALAR_MAP:
            return _CPP_SCALAR_MAP[field.type], False, False, False
        if short in self._aliases:
            return self._aliases[short], False, False, True
        if self._index.is_enum_type(field.type) or self._index.is_enum_type(short):
            return self._qualify(field.type, current_pkg), False, True, False
        if self._index.is_message_type(field.type) or self._index.is_message_type(short):
            if short in self._aliases:
                return self._aliases[short], False, False, True
            return self._qualify(field.type, current_pkg), True, False, False
        return short, False, False, False

    def _write_to_json(self, f, msg, current_pkg: str) -> None:
        f.write(f'std::string toJson(const {msg.name}& msg) {{\n')
        f.write('    nlohmann::json obj;\n')
        for field, fname, _ in self._inline_base_fields(msg):
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
                    f.write(f'            arr.push_back('
                            f'nlohmann::json::parse(toJson(v)));\n')
                else:
                    f.write(f'            arr.push_back(v);\n')
                f.write(f'        }}\n')
                f.write(f'        obj["{fname}"] = arr;\n')
                f.write(f'    }}\n')
            elif field.is_optional and base_cpp not in ('std::string',):
                f.write(f'    if (msg.{fname}.has_value()) {{\n')
                if is_struct:
                    f.write(f'        obj["{fname}"] = nlohmann::json::parse('
                            f'toJson(msg.{fname}.value()));\n')
                elif is_enum:
                    f.write(f'        obj["{fname}"] = toString(msg.{fname}.value());\n')
                else:
                    f.write(f'        obj["{fname}"] = msg.{fname}.value();\n')
                f.write(f'    }}\n')
            else:
                if is_struct:
                    f.write(f'    obj["{fname}"] = nlohmann::json::parse('
                            f'toJson(msg.{fname}));\n')
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
                    f.write(f'        obj["{field.name}"] = nlohmann::json::parse('
                            f'toJson(msg.{field.name}.value()));\n')
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
        for field, fname, _ in self._inline_base_fields(msg):
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
                    f.write(f'            msg.{fname}.push_back('
                            f'fromJson(v.dump(), '
                            f'static_cast<{base_cpp}*>(nullptr)));\n')
                else:
                    f.write(f'            msg.{fname}.push_back(v.get<{base_cpp}>());\n')
                f.write(f'        }}\n')
                f.write(f'    }}\n')
            elif field.is_optional and base_cpp not in ('std::string',):
                f.write(f'    if (j.contains("{fname}")) {{\n')
                if is_struct:
                    f.write(f'        msg.{fname} = fromJson('
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
                    f.write(f'    if (j.contains("{fname}")) msg.{fname} = fromJson('
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
                    f.write(f'        msg.{field.name} = fromJson('
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

    def _inline_base_fields(self, msg):
        """Mirror CppTypesGenerator._inline_base_fields for codec use."""
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                short = field.type.split('.')[-1]
                base_msg = (self._index.resolve_message(field.type)
                            or self._index.resolve_message(short))
                if base_msg and base_msg.name not in self._aliases:
                    for bf in base_msg.fields:
                        name = bf.name
                        if name in own_names:
                            name = short.lower() + '_' + name
                        yield bf, name, ''
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
            print('  e.g. --types proto/pyramid/data_model bindings/cpp/generated')
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
