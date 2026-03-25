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

    # Also generate the JSON codec files:
    python cpp_service_generator.py --codec <file.proto> <output_dir>
"""

import sys
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import json_schema as schema
from proto_parser import (
    parse_proto_tree, ProtoTypeIndex, ProtoMessage, ProtoEnum,
    screaming_to_pascal, _PROTO_SCALARS,
)


# -- EntityActions operation set -----------------------------------------------

OP_PREFIXES = ['Create', 'Read', 'Update', 'Delete']

# Base-type short names from pyramid.data_model.base.* and common.*
BASE_TYPE_MAP = {
    'pyramid.data_model.base.Identifier': 'Identifier',
    'pyramid.data_model.base.Query':      'Query',
    'pyramid.data_model.base.Ack':        'Ack',
    'pyramid.data_model.common.Query':    'Query',
    'pyramid.data_model.common.Ack':      'Ack',
    'pyramid.data_model.common.Capability': 'Identifier',
}

# Standard topic names — sourced from json_schema.py as canonical authority.
# Provided side: client subscribes to these (server publishes).
SUBSCRIBE_TOPICS = schema.SUBSCRIBE_TOPICS   # entity_matches, evidence_requirements
# Consumed side: client publishes to these (server subscribes).
PUBLISH_TOPICS   = schema.PUBLISH_TOPICS     # object_evidence

_SEP = '// ' + '-' * 75

# Default content type — used when no port-level override is provided.
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


# -- Code generation -----------------------------------------------------------

class CppServiceGenerator:

    def __init__(self, proto_input: str):
        self._proto_input = Path(proto_input)

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
        sub_topics = SUBSCRIBE_TOPICS if is_provided else {}
        pub_topics = {} if is_provided else PUBLISH_TOPICS
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)
        topic_set = all_topics

        # Collect raw type names (no BASE_TYPE_MAP) for using declarations
        raw_types = sorted({
            t
            for _, rpc in all_rpcs
            for t in (rpc.raw_req_type, rpc.raw_rsp_type)
        })

        with open(path, 'w') as f:
            # File-level comment block
            f.write('// Auto-generated service binding header\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by cpp_service_generator\n')
            f.write(f'// Namespace: {full_ns}\n')
            f.write('//\n')
            f.write('// Architecture: component logic > service binding (this) > PCL\n')
            f.write('//\n')
            f.write('// This header provides:\n')
            f.write('//   1. Wire-name constants and topic constants\n')
            f.write('//   2. EntityActions handler base class'
                    ' (ServiceHandler \u2014 override Handle*)\n')
            if is_provided:
                f.write('//   3. PCL binding functions (subscribe*, invoke*)\n')
                f.write('//   4. msgToString utility for PCL message payloads\n')
            else:
                f.write('//   3. PCL binding functions (subscribe*, publish*)\n')
                f.write('//   4. msgToString utility for PCL message payloads\n')
            f.write('#pragma once\n\n')

            # Includes
            f.write(f'#include "{types_header}"\n\n')
            f.write('#include <pcl/pcl_container.h>\n')
            if is_provided:
                f.write('#include <pcl/pcl_transport_socket.h>\n')
            else:
                f.write('#include <pcl/pcl_executor.h>\n')
                f.write('#include <pcl/pcl_transport_socket.h>\n')
            f.write('#include <pcl/pcl_types.h>\n\n')
            f.write('#include <string>\n')
            if is_provided:
                f.write('#include <string_view>\n')
            f.write('#include <vector>\n\n')

            # Namespace open
            f.write(f'namespace {full_ns} {{\n\n')

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
            if is_provided:
                f.write('// PCL binding functions \u2014 Subscribe / Invoke wrappers\n')
            else:
                f.write('// PCL binding functions \u2014 Subscribe / Publish wrappers\n')
            f.write(_SEP + '\n\n')

            # Subscribe helpers
            for key, _wire in topic_set.items():
                pascal = _snake_to_pascal(key)
                fname = f'subscribe{pascal}'
                cname = f'kTopic{pascal}'
                # Align params to column after opening '('
                col = len(f'void {fname}(')
                sp = ' ' * col
                brief = (f'/// \\brief Subscribe to'
                         f' {_topic_key_to_phrase(key)} publications on'
                         f' {cname}.')
                if len(brief) > 80:
                    # Wrap at the constant name
                    split = brief.rfind(f' {cname}.')
                    f.write(brief[:split] + '\n')
                    f.write(f'///        {cname}.\n')
                else:
                    f.write(brief + '\n')
                f.write(f'void {fname}(pcl_container_t*  container,\n')
                f.write(f'{sp}pcl_sub_callback_t callback,\n')
                f.write(f'{sp}void*             user_data = nullptr,\n')
                f.write(f'{sp}const char*       content_type = "application/json");\n\n')

            if is_provided:
                # Invoke helpers
                for svc in parsed.services:
                    for rpc in svc.rpcs:
                        wire_full = f'{svc.wire_prefix}.{rpc.wire_name}'
                        # Align params: 'pcl_status_t ' (13) + fname + '(' (1)
                        col = 13 + len(rpc.cpp_invoke_func) + 1
                        sp = ' ' * col
                        f.write(f'/// \\brief Asynchronously invoke {wire_full}.\n')
                        f.write(f'pcl_status_t {rpc.cpp_invoke_func}'
                                f'(pcl_socket_transport_t* transport,\n')
                        f.write(f'{sp}const std::string&      request,\n')
                        f.write(f'{sp}pcl_resp_cb_fn_t        callback,\n')
                        f.write(f'{sp}void*                   user_data'
                                f' = nullptr,\n')
                        f.write(f'{sp}const char*             content_type'
                                f' = "application/json");\n\n')
            else:
                # Publish helpers
                for key, _wire in topic_set.items():
                    pascal = _snake_to_pascal(key)
                    fname = f'publish{pascal}'
                    cname = f'kTopic{pascal}'
                    # Align params: 'pcl_status_t ' (13) + fname + '(' (1)
                    col = 13 + len(fname) + 1
                    sp = ' ' * col
                    f.write(f'/// \\brief Publish an'
                            f' {key.replace("_", "-")} payload on {cname}.\n')
                    f.write('///\n')
                    f.write(f'/// \\p publisher must be the pcl_port_t* returned'
                            f' by addPublisher for\n')
                    f.write(f'/// {cname}, obtained during on_configure.\n')
                    f.write(f'pcl_status_t {fname}(pcl_port_t*        publisher,\n')
                    f.write(f'{sp}const std::string& payload,\n')
                    f.write(f'{sp}const char*        content_type'
                            f' = "application/json");\n\n')

            # ---- dispatch() --------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Transport dispatch point\n')
            f.write('//\n')
            f.write('// Routes a raw request buffer to the appropriate handler.\n')
            f.write('// Response buffer is heap-allocated; caller is responsible'
                    ' for freeing it.\n')
            f.write(_SEP + '\n\n')
            f.write('void dispatch(ServiceChannel channel,\n')
            f.write('              const void*    request_buf,\n')
            f.write('              size_t         request_size,\n')
            f.write('              void**         response_buf,\n')
            f.write('              size_t*        response_size);\n\n')

            # Namespace close
            f.write(f'}} // namespace {full_ns}\n')

    # -- Implementation (.cpp) -------------------------------------------------

    def _write_impl(self, path: Path, file_prefix: str, full_ns: str,
                    types_ns: str, parsed: ProtoFile,
                    all_rpcs: List[Tuple[str, ProtoRpc]]):
        is_provided = _is_provided(parsed)
        sub_topics = SUBSCRIBE_TOPICS if is_provided else {}
        pub_topics = {} if is_provided else PUBLISH_TOPICS
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)
        topic_set = all_topics
        hpp_name = file_prefix + '.hpp'

        with open(path, 'w') as f:
            # File-level comment block
            f.write('// Auto-generated service binding implementation\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by cpp_service_generator\n')
            f.write(f'// Namespace: {full_ns}\n\n')

            # Includes
            f.write(f'#include "{hpp_name}"\n\n')
            f.write('#include <pcl/pcl_container.h>\n')
            if is_provided:
                f.write('#include <pcl/pcl_transport_socket.h>\n')
            f.write('\n#include <string>\n')
            f.write('#include <vector>\n\n')

            # Namespace open
            f.write(f'namespace {full_ns} {{\n\n')

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

            # ---- PCL binding helpers -----------------------------------------
            if is_provided:
                # Anonymous namespace with generic invoke_async helper
                f.write(_SEP + '\n')
                f.write('// PCL binding helpers\n')
                f.write(_SEP + '\n\n')
                f.write('namespace {\n\n')
                f.write('/// \\brief Generic invoke helper \u2014 builds a pcl_msg_t and'
                        ' dispatches via\n')
                f.write('///        pcl_socket_transport_invoke_remote_async.\n')
                f.write('pcl_status_t invoke_async(pcl_socket_transport_t* transport,\n')
                f.write('                           const char*'
                        '             service_name,\n')
                f.write('                           const std::string&'
                        '      request,\n')
                f.write('                           pcl_resp_cb_fn_t'
                        '        callback,\n')
                f.write('                           void*'
                        '                   user_data,\n')
                f.write('                           const char*'
                        '             content_type)\n')
                f.write('{\n')
                f.write('    pcl_msg_t msg{};\n')
                f.write('    msg.data      = request.data();\n')
                f.write('    msg.size      = static_cast<uint32_t>(request.size());\n')
                f.write('    msg.type_name = content_type;\n')
                f.write('    return pcl_socket_transport_invoke_remote_async(\n')
                f.write('        transport, service_name, &msg, callback, user_data);\n')
                f.write('}\n\n')
                f.write('} // namespace\n\n')

            # Subscribe wrappers
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
                    col = len(f'void {fname}(')
                    sp = ' ' * col
                    f.write(f'void {fname}(pcl_container_t*   container,\n')
                    f.write(f'{sp}pcl_sub_callback_t  callback,\n')
                    f.write(f'{sp}void*              user_data,\n')
                    f.write(f'{sp}const char*        content_type)\n')
                    f.write('{\n')
                    f.write('    pcl_container_add_subscriber(container,\n')
                    f.write(f'                                 {cname},\n')
                    f.write('                                 content_type,\n')
                    f.write('                                 callback,\n')
                    f.write('                                 user_data);\n')
                    f.write('}\n\n')

            if is_provided:
                # Invoke wrappers
                f.write(_SEP + '\n')
                f.write('// PCL invoke wrappers\n')
                f.write(_SEP + '\n\n')
                for svc in parsed.services:
                    for rpc in svc.rpcs:
                        col = 13 + len(rpc.cpp_invoke_func) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {rpc.cpp_invoke_func}'
                                f'(pcl_socket_transport_t* transport,\n')
                        f.write(f'{sp}const std::string&      request,\n')
                        f.write(f'{sp}pcl_resp_cb_fn_t        callback,\n')
                        f.write(f'{sp}void*                   user_data,\n')
                        f.write(f'{sp}const char*             content_type)\n')
                        f.write('{\n')
                        f.write(f'    return invoke_async(transport,'
                                f' {rpc.cpp_svc_const}, request, callback,'
                                f' user_data, content_type);\n')
                        f.write('}\n\n')
            else:
                # Publish wrappers
                n_pubs = len(topic_set)
                if n_pubs:
                    f.write(_SEP + '\n')
                    label = 'PCL publish wrapper' + ('s' if n_pubs > 1 else '')
                    f.write(f'// {label}\n')
                    f.write(_SEP + '\n\n')
                    for key, _wire in topic_set.items():
                        pascal = _snake_to_pascal(key)
                        fname = f'publish{pascal}'
                        col = 13 + len(fname) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {fname}(pcl_port_t*        publisher,\n')
                        f.write(f'{sp}const std::string& payload,\n')
                        f.write(f'{sp}const char*        content_type)\n')
                        f.write('{\n')
                        f.write('    pcl_msg_t msg{};\n')
                        f.write('    msg.data      = payload.data();\n')
                        f.write('    msg.size      = static_cast<uint32_t>(payload.size());\n')
                        f.write('    msg.type_name = content_type;\n')
                        f.write('    return pcl_port_publish(publisher, &msg);\n')
                        f.write('}\n\n')

            # ---- dispatch() --------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Dispatch \u2014 routes raw buffer to the appropriate handler stub\n')
            f.write(_SEP + '\n\n')
            f.write('void dispatch(ServiceChannel channel,\n')
            f.write('              const void*    /*request_buf*/,\n')
            f.write('              size_t         /*request_size*/,\n')
            f.write('              void**         response_buf,\n')
            f.write('              size_t*        response_size)\n')
            f.write('{\n')
            f.write('    *response_buf  = nullptr;\n')
            f.write('    *response_size = 0;\n\n')
            f.write('    switch (channel) {\n')
            for _, rpc in all_rpcs:
                f.write(f'        case ServiceChannel::{rpc.cpp_enum_value}:\n')
                f.write(f'            // TODO: deserialise request, call'
                        f' handler, serialise response\n')
                f.write('            break;\n')
            f.write('    }\n')
            f.write('}\n\n')

            # Namespace close
            f.write(f'}} // namespace {full_ns}\n')


class CppJsonCodecGenerator:
    """Generates the canonical JSON ser/de header+impl from json_schema.py.

    Output file names and namespaces are derived from the supplied .proto file.
    Types from the parent namespace are brought in via  using namespace.

    Usage:
        gen = CppJsonCodecGenerator(proto_file)
        gen.generate('/path/to/output_dir')
    """

    def __init__(self, proto_file: ProtoFile):
        _, _, svc_base_ns, types_ns = _namespace_from_proto(proto_file)
        self.TYPES_NS = types_ns
        self.NS       = svc_base_ns + '::json_codec'
        svc_prefix    = '_'.join(svc_base_ns.split('::'))
        self.HPP      = svc_prefix + '_json_codec.hpp'
        self.CPP      = svc_prefix + '_json_codec.cpp'
        self._types_header = '_'.join(types_ns.split('::')) + '_types.hpp'

    def generate(self, output_dir: str):
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        self._write_header(out / self.HPP)
        self._write_impl(out / self.CPP)
        print(f'  Generated {self.NS}')

    # -- Header ----------------------------------------------------------------

    def _write_header(self, path: Path):
        ek = schema.FieldKind

        with open(path, 'w') as f:
            f.write('// Auto-generated JSON codec header\n')
            f.write(f'// Namespace: {self.NS}\n')
            f.write('// Generated by cpp_service_generator.py (CppJsonCodecGenerator)\n')
            f.write('// Schema source: pim/json_schema.py\n')
            f.write('//\n')
            f.write('// Canonical JSON wire format for the pyramid standard bridge protocol.\n')
            f.write('// Each message schema maps directly to a proto message type:\n')
            f.write('//\n')
            for msg in schema.ALL_SCHEMAS:
                f.write(f'//   {msg.cpp_name:<34} {msg.wire_description}\n')
            f.write('//\n')
            f.write('// Architecture: component logic > JsonCodec > service binding > PCL\n')
            f.write('#pragma once\n\n')
            f.write(f'#include "{self._types_header}"\n\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write(f'namespace {self.NS} {{\n\n')
            f.write(f'using namespace {self.TYPES_NS};\n\n')

            # -- Message struct declarations ------------------------------------
            f.write(_SEP + '\n')
            f.write('// Wire message structs\n')
            f.write(_SEP + '\n\n')
            for msg in schema.ALL_SCHEMAS:
                f.write(f'// {msg.wire_description}\n')
                f.write(f'struct {msg.cpp_name} {{\n')
                for fld in msg.fields:
                    comment = f'  // {fld.description}' if fld.description else ''
                    opt_tag = '' if fld.required else '  // optional'
                    tag = comment or opt_tag
                    f.write(f'    {fld.cpp_type} {fld.name} = {fld.cpp_default};{tag}\n')
                f.write('};\n\n')

            # Array alias for entity matches
            f.write('using EntityMatchArray = std::vector<EntityMatch>;\n\n')

            # -- Serialisation -------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Serialisation (toJson)\n')
            f.write(_SEP + '\n\n')
            for msg in schema.ALL_SCHEMAS:
                f.write(f'std::string toJson(const {msg.cpp_name}& msg);\n')
            f.write('\n')

            # -- Deserialisation -----------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Deserialisation (fromJson)\n')
            f.write(_SEP + '\n\n')
            for msg in schema.ALL_SCHEMAS:
                f.write(f'{msg.cpp_name} {_lc_first(msg.cpp_name)}FromJson'
                        f'(const std::string& s);\n')
            f.write('\n')
            f.write('// Deserialise standard.entity_matches JSON array\n')
            f.write('EntityMatchArray entityMatchesFromJson(const std::string& s);\n\n')

            # -- Enum string converters ----------------------------------------
            f.write(_SEP + '\n')
            f.write('// Enum string converters\n')
            f.write(_SEP + '\n\n')
            for _kind, spec in schema.ENUM_SPECS.items():
                t = spec.cpp_type
                fn = _lc_first(t)
                f.write(f'std::string toString({t} v);\n')
                f.write(f'{t} {fn}FromString(const std::string& s);\n')
            f.write('\n')

            f.write(f'}} // namespace {self.NS}\n')

    # -- Implementation --------------------------------------------------------

    def _write_impl(self, path: Path):
        ek = schema.FieldKind

        with open(path, 'w') as f:
            f.write('// Auto-generated JSON codec implementation\n')
            f.write(f'// Namespace: {self.NS}\n\n')
            f.write(f'#include "{self.HPP}"\n\n')
            f.write('#include <nlohmann/json.hpp>\n\n')
            f.write(f'namespace {self.NS} {{\n\n')

            # -- Enum string converters ----------------------------------------
            f.write(_SEP + '\n')
            f.write('// Enum string converters\n')
            f.write(_SEP + '\n\n')
            for _kind, spec in schema.ENUM_SPECS.items():
                t   = spec.cpp_type
                fn  = _lc_first(t)
                tbl = spec.table()

                # toString
                f.write(f'std::string toString({t} v) {{\n')
                f.write(f'    switch (v) {{\n')
                for proto_str, _ada, cpp_lit, _ord in tbl:
                    f.write(f'        case {t}::{cpp_lit}: return "{proto_str}";\n')
                f.write(f'    }}\n')
                f.write(f'    return "{tbl[0][0]}";\n')  # fallback = first (Unspecified)
                f.write(f'}}\n\n')

                # fromString
                f.write(f'{t} {fn}FromString(const std::string& s) {{\n')
                for proto_str, _ada, cpp_lit, _ord in tbl:
                    f.write(f'    if (s == "{proto_str}") return {t}::{cpp_lit};\n')
                f.write(f'    return {t}::{spec.default_cpp};\n')
                f.write(f'}}\n\n')

            # -- toJson implementations ----------------------------------------
            f.write(_SEP + '\n')
            f.write('// Serialisation (toJson)\n')
            f.write(_SEP + '\n\n')
            for msg in schema.ALL_SCHEMAS:
                f.write(f'std::string toJson(const {msg.cpp_name}& msg) {{\n')
                f.write(f'    nlohmann::json obj;\n')
                for fld in msg.fields:
                    jkey = fld.name
                    if fld.is_enum:
                        spec = schema.ENUM_SPECS[fld.kind]
                        fn   = _lc_first(spec.cpp_type)
                        if fld.required:
                            f.write(f'    obj["{jkey}"] = toString(msg.{jkey});\n')
                        else:
                            f.write(f'    if (msg.{jkey} != {fld.cpp_default})'
                                    f' obj["{jkey}"] = toString(msg.{jkey});\n')
                    elif fld.kind in (ek.STRING, ek.IDENTIFIER):
                        if fld.required:
                            f.write(f'    obj["{jkey}"] = msg.{jkey};\n')
                        else:
                            f.write(f'    if (!msg.{jkey}.empty())'
                                    f' obj["{jkey}"] = msg.{jkey};\n')
                    elif fld.kind == ek.DOUBLE:
                        if fld.required:
                            f.write(f'    obj["{jkey}"] = msg.{jkey};\n')
                        else:
                            f.write(f'    if (msg.{jkey} != 0.0)'
                                    f' obj["{jkey}"] = msg.{jkey};\n')
                    elif fld.kind == ek.BOOL:
                        f.write(f'    obj["{jkey}"] = msg.{jkey};\n')
                f.write(f'    return obj.dump();\n')
                f.write(f'}}\n\n')

            # -- fromJson implementations ---------------------------------------
            f.write(_SEP + '\n')
            f.write('// Deserialisation (fromJson)\n')
            f.write(_SEP + '\n\n')
            for msg in schema.ALL_SCHEMAS:
                fname = f'{_lc_first(msg.cpp_name)}FromJson'
                f.write(f'{msg.cpp_name} {fname}(const std::string& s) {{\n')
                f.write(f'    {msg.cpp_name} result;\n')
                f.write(f'    try {{\n')
                f.write(f'        auto j = nlohmann::json::parse(s);\n')
                for fld in msg.fields:
                    jkey = fld.name
                    if fld.is_enum:
                        spec = schema.ENUM_SPECS[fld.kind]
                        fn   = _lc_first(spec.cpp_type)
                        f.write(f'        if (j.contains("{jkey}"))\n')
                        f.write(f'            result.{jkey} = {fn}FromString('
                                f'j["{jkey}"].get<std::string>());\n')
                    elif fld.kind in (ek.STRING, ek.IDENTIFIER):
                        f.write(f'        if (j.contains("{jkey}"))\n')
                        f.write(f'            result.{jkey} = '
                                f'j["{jkey}"].get<std::string>();\n')
                    elif fld.kind == ek.DOUBLE:
                        f.write(f'        if (j.contains("{jkey}"))\n')
                        f.write(f'            result.{jkey} = '
                                f'j["{jkey}"].get<double>();\n')
                    elif fld.kind == ek.BOOL:
                        f.write(f'        if (j.contains("{jkey}"))\n')
                        f.write(f'            result.{jkey} = '
                                f'j["{jkey}"].get<bool>();\n')
                f.write(f'    }} catch (...) {{}}\n')
                f.write(f'    return result;\n')
                f.write(f'}}\n\n')

            # -- entityMatchesFromJson -----------------------------------------
            f.write(_SEP + '\n')
            f.write('// Array deserialisation\n')
            f.write(_SEP + '\n\n')
            f.write('EntityMatchArray entityMatchesFromJson(const std::string& s) {\n')
            f.write('    EntityMatchArray result;\n')
            f.write('    try {\n')
            f.write('        auto arr = nlohmann::json::parse(s);\n')
            f.write('        if (!arr.is_array()) return result;\n')
            f.write('        result.reserve(arr.size());\n')
            f.write('        for (const auto& elem : arr) {\n')
            f.write('            EntityMatch m;\n')
            for fld in schema.ENTITY_MATCH.fields:
                jkey = fld.name
                if fld.is_enum:
                    spec = schema.ENUM_SPECS[fld.kind]
                    fn   = _lc_first(spec.cpp_type)
                    f.write(f'            if (elem.contains("{jkey}"))\n')
                    f.write(f'                m.{jkey} = {fn}FromString('
                            f'elem["{jkey}"].get<std::string>());\n')
                elif fld.kind in (ek.STRING, ek.IDENTIFIER):
                    f.write(f'            if (elem.contains("{jkey}"))\n')
                    f.write(f'                m.{jkey} = '
                            f'elem["{jkey}"].get<std::string>();\n')
                elif fld.kind == ek.DOUBLE:
                    f.write(f'            if (elem.contains("{jkey}"))\n')
                    f.write(f'                m.{jkey} = '
                            f'elem["{jkey}"].get<double>();\n')
            f.write('            result.push_back(std::move(m));\n')
            f.write('        }\n')
            f.write('    } catch (...) {}\n')
            f.write('    return result;\n')
            f.write('}\n\n')

            f.write(f'}} // namespace {self.NS}\n')


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
    'Timestamp': 'double',  # google.protobuf.Timestamp wrapper → epoch seconds
}

# Single-field message is a scalar wrapper only when the field name signals
# a physical unit or a generic "value" placeholder — NOT domain names like
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
        # Umbrella header: pyramid_data_model_types.hpp → includes all + re-exports
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
            base = self._aliases[short]           # always a scalar — no namespace
        elif '.' in field_type and not field_type.startswith('google.'):
            # Fully-qualified proto type
            pkg = '.'.join(field_type.split('.')[:-1])
            if current_pkg and pkg != current_pkg:
                base = pkg.replace('.', '::') + '::' + short
            else:
                base = short
        elif (self._index.is_enum_type(field_type)
              or self._index.is_message_type(field_type)):
            # Short name — look up its package for cross-package qualification
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
            for indexed_pf in self._index.files:
                if indexed_pf.package == pkg:
                    result.append(f'#include "{pkg.replace(".", "_")}_types.hpp"')
                    break
        return result

    def _write_hpp_for_file(self, path: Path, pf) -> None:
        ns = pf.package.replace('.', '::')
        current_pkg = pf.package
        includes = self._includes_for_file(pf)
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]

        with open(path, 'w') as f:
            f.write('// Auto-generated types header\n')
            f.write(f'// Generated from: {pf.path.name}'
                    f' by cpp_service_generator.py --types\n')
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
        with open(path, 'w') as f:
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
        # Build alias map (scalar wrappers) — same logic as CppTypesGenerator
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
        with open(path, 'w') as f:
            f.write('// Auto-generated data model JSON codec header\n')
            f.write(f'// Generated from: {self._pf.path.name}'
                    f' by cpp_service_generator.py --codec\n')
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

        with open(path, 'w') as f:
            f.write('// Auto-generated data model JSON codec implementation\n')
            f.write(f'// Namespace: {self._ns}\n\n')
            f.write(f'#include "{self._hpp_name}"\n\n')
            f.write('#include <nlohmann/json.hpp>\n\n')
            # Include codec headers for imported packages (for nested toJson calls)
            for imp in self._pf.imports:
                if imp.startswith('google/'):
                    continue
                pkg = imp.replace('/', '.').removesuffix('.proto')
                for indexed_pf in self._index.files:
                    if indexed_pf.package == pkg:
                        f.write(f'#include "{pkg.replace(".", "_")}_codec.hpp"\n')
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
            print('  e.g. --types proto/pyramid/data_model examples/cpp/generated')
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
            parsed = parse_proto(codec_path)
            gen = CppJsonCodecGenerator(parsed)
            gen.generate(sys.argv[3])
            print('\n\u2713 C++ JSON codec generated')
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
