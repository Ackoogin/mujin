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

Generated files reference a <prefix>_types.hpp for C++ type definitions.

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
from typing import List, Optional, Tuple


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

# Standard topic names by convention (same as Ada generator)
STANDARD_TOPICS = {
    'provided': {
        'entity_matches': 'standard.entity_matches',
        'evidence_requirements': 'standard.evidence_requirements',
    },
    'consumed': {
        'object_evidence': 'standard.object_evidence',
    },
}

_SEP = '// ' + '-' * 75


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

def _namespace_from_proto(proto_file: ProtoFile) -> Tuple[str, str, str]:
    """Return (full_namespace, file_prefix, types_namespace).

    pyramid.components.tactical_objects.services.provided
      -> full_ns    : pyramid::services::tactical_objects::provided
      -> file_prefix: pyramid_services_tactical_objects_provided
      -> types_ns   : pyramid::services::tactical_objects
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

    types_ns = '::'.join(ns_parts)
    full_ns = types_ns + '::' + suffix
    file_prefix = '_'.join(ns_parts + [suffix])

    return full_ns, file_prefix, types_ns


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

            full_ns, file_prefix, types_ns = _namespace_from_proto(parsed)
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
        topic_set = STANDARD_TOPICS.get(
            'provided' if is_provided else 'consumed', {})

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
                f.write('//   3. JSON builder functions (nlohmann::json / string)\n')
                f.write('//   4. PCL binding functions (subscribe*, invoke*)\n')
                f.write('//   5. msgToString utility for PCL message payloads\n')
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

            # ---- JSON builder functions (provided only) ----------------------
            if is_provided:
                f.write(_SEP + '\n')
                f.write('// JSON builder functions\n')
                f.write(_SEP + '\n\n')
                f.write('/// \\brief Build a JSON object for an interest/evidence'
                        ' requirement.\n')
                f.write('///\n')
                f.write('/// Produces: {"policy":..., "identity":..., "dimension":...,\n')
                f.write('///            "min_lat_rad":..., "max_lat_rad":...,\n')
                f.write('///            "min_lon_rad":..., "max_lon_rad":...}\n')
                f.write('std::string buildStandardRequirementJson(\n')
                f.write('    std::string_view policy,\n')
                f.write('    std::string_view identity,\n')
                f.write('    std::string_view dimension  = "",\n')
                f.write('    double min_lat_rad          = 0.0,\n')
                f.write('    double max_lat_rad          = 0.0,\n')
                f.write('    double min_lon_rad          = 0.0,\n')
                f.write('    double max_lon_rad          = 0.0);\n')
                f.write('\n')
                f.write('/// \\brief Build a JSON object for an observation evidence'
                        ' report.\n')
                f.write('///\n')
                f.write('/// Produces: {"identity":..., "dimension":...,\n')
                f.write('///            "latitude_rad":..., "longitude_rad":...,\n')
                f.write('///            "confidence":..., "observed_at":...}\n')
                f.write('std::string buildStandardEvidenceJson(\n')
                f.write('    std::string_view identity,\n')
                f.write('    std::string_view dimension,\n')
                f.write('    double lat_rad,\n')
                f.write('    double lon_rad,\n')
                f.write('    double confidence,\n')
                f.write('    double observed_at = 0.5);\n\n')

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
                f.write(f'{sp}void*             user_data = nullptr);\n\n')

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
                                f' = nullptr);\n\n')
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
                    f.write(f'{sp}const std::string& payload);\n\n')

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
        topic_set = STANDARD_TOPICS.get(
            'provided' if is_provided else 'consumed', {})
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
                f.write('\n#include <nlohmann/json.hpp>\n')
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
                    f.write('    return kAckOk;\n')
                else:
                    f.write('    return {};\n')

                f.write('}\n\n')

            # ---- JSON builders (provided only) -------------------------------
            if is_provided:
                f.write(_SEP + '\n')
                f.write('// JSON builder: buildStandardRequirementJson\n')
                f.write(_SEP + '\n\n')
                f.write('std::string buildStandardRequirementJson(\n')
                f.write('    std::string_view policy,\n')
                f.write('    std::string_view identity,\n')
                f.write('    std::string_view dimension,\n')
                f.write('    double min_lat_rad,\n')
                f.write('    double max_lat_rad,\n')
                f.write('    double min_lon_rad,\n')
                f.write('    double max_lon_rad)\n')
                f.write('{\n')
                f.write('    nlohmann::json obj;\n')
                f.write('    obj["policy"]      = std::string(policy);\n')
                f.write('    obj["identity"]    = std::string(identity);\n')
                f.write('    if (!dimension.empty()) {\n')
                f.write('        obj["dimension"] = std::string(dimension);\n')
                f.write('    }\n')
                f.write('    obj["min_lat_rad"] = min_lat_rad;\n')
                f.write('    obj["max_lat_rad"] = max_lat_rad;\n')
                f.write('    obj["min_lon_rad"] = min_lon_rad;\n')
                f.write('    obj["max_lon_rad"] = max_lon_rad;\n')
                f.write('    return obj.dump();\n')
                f.write('}\n\n')

                f.write(_SEP + '\n')
                f.write('// JSON builder: buildStandardEvidenceJson\n')
                f.write(_SEP + '\n\n')
                f.write('std::string buildStandardEvidenceJson(\n')
                f.write('    std::string_view identity,\n')
                f.write('    std::string_view dimension,\n')
                f.write('    double lat_rad,\n')
                f.write('    double lon_rad,\n')
                f.write('    double confidence,\n')
                f.write('    double observed_at)\n')
                f.write('{\n')
                f.write('    nlohmann::json obj;\n')
                f.write('    obj["identity"]      = std::string(identity);\n')
                f.write('    obj["dimension"]     = std::string(dimension);\n')
                f.write('    obj["latitude_rad"]  = lat_rad;\n')
                f.write('    obj["longitude_rad"] = lon_rad;\n')
                f.write('    obj["confidence"]    = confidence;\n')
                f.write('    obj["observed_at"]   = observed_at;\n')
                f.write('    return obj.dump();\n')
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
                        '                   user_data)\n')
                f.write('{\n')
                f.write('    pcl_msg_t msg{};\n')
                f.write('    msg.data      = request.data();\n')
                f.write('    msg.size      = static_cast<uint32_t>(request.size());\n')
                f.write('    msg.type_name = "application/json";\n')
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
                    f.write(f'{sp}void*              user_data)\n')
                    f.write('{\n')
                    f.write('    pcl_container_add_subscriber(container,\n')
                    f.write(f'                                 {cname},\n')
                    f.write('                                 "application/json",\n')
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
                        f.write(f'{sp}void*                   user_data)\n')
                        f.write('{\n')
                        f.write(f'    return invoke_async(transport,'
                                f' {rpc.cpp_svc_const}, request, callback,'
                                f' user_data);\n')
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
                        f.write(f'{sp}const std::string& payload)\n')
                        f.write('{\n')
                        f.write('    pcl_msg_t msg{};\n')
                        f.write('    msg.data      = payload.data();\n')
                        f.write('    msg.size      = static_cast<uint32_t>(payload.size());\n')
                        f.write('    msg.type_name = "application/json";\n')
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


def main():
    if len(sys.argv) < 3:
        print('Usage: python cpp_service_generator.py'
              ' <file.proto|proto_dir> <output_dir>')
        sys.exit(1)

    gen = CppServiceGenerator(sys.argv[1])
    gen.generate(sys.argv[2])
    print('\n\u2713 C++ services generated')


if __name__ == '__main__':
    main()
