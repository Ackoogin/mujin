#!/usr/bin/env python3
"""
FlatBuffers Codec Backend

Generates:
  1. .fbs schema files from proto definitions (proto message -> FlatBuffers table)
  2. C++ wrapper headers for proto-native flatbuffer artefacts
  3. Proto-native service FlatBuffers codec artefacts

Service-wire codecs are generated against real `flatc` output.  The service
runtime still exposes a generic `wrapPayload` / `unwrapPayload` surface for
domain-shaped RPCs, but that wrapper is itself a proper FlatBuffers table
(`JsonPayload`) rather than the old ad hoc "PWFB" prefix envelope.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import codec_backends
from proto_parser import (
    ProtoTypeIndex, ProtoFile, ProtoField,
    camel_to_snake, _PROTO_SCALARS,
)


_TACTICAL_SERVICE_PACKAGES = frozenset({
    'pyramid.components.tactical_objects.services.provided',
    'pyramid.components.tactical_objects.services.consumed',
})

_SERVICE_FILE_BASE = 'pyramid_services_tactical_objects'
_SERVICE_GENERATED_HEADER = _SERVICE_FILE_BASE + '_generated.h'
_SERVICE_CODEC_NS = 'pyramid::services::tactical_objects::flatbuffers_codec'
_SERVICE_ADA_CODEC_PKG = 'Pyramid.Services.Tactical_Objects.Flatbuffers_Codec'
_SERVICE_ADA_CODEC_FILE = 'pyramid-services-tactical_objects-flatbuffers_codec'
_ALIAS_FIELD_NAMES = frozenset({
    'value', 'radians', 'meters', 'meters_per_second', 'seconds',
})


@dataclass
class FlatFieldSpec:
    index: int
    name: str
    kind: str
    type_name: str
    proto_scalar: str = ''
    repeated: bool = False
    generated_optional: bool = False
    generated_presence: bool = False
    oneof: bool = False


@dataclass
class FlatMessageSpec:
    name: str
    cpp_type: str
    ada_type: str
    full_type: str
    fields: List[FlatFieldSpec]
    json_codec_ns: str
    is_alias_root: bool = False
    alias_scalar: str = ''
    alias_field_name: str = 'value'


@dataclass
class FlatArraySpec:
    name: str
    holder_name: str
    element_kind: str
    element_type_name: str
    element_cpp_type: str
    element_ada_type: str
    element_full_type: str
    json_codec_ns: str
    element_proto_scalar: str = ''


@dataclass
class ServiceCodecGroup:
    base_package: str
    file_base: str
    fbs_namespace: str
    cpp_codec_ns: str
    cpp_base_ns: str
    wire_header: str
    generated_header: str
    ada_codec_pkg: str
    ada_codec_file: str
    message_specs: List[FlatMessageSpec]
    array_specs: List[FlatArraySpec]
    enum_specs: List


def _service_group_key(package: str) -> Optional[str]:
    if '.services.' not in f'.{package}.':
        return None
    parts = [p for p in package.split('.') if p]
    if parts and parts[-1].lower() in ('provided', 'consumed'):
        parts = parts[:-1]
    return '.'.join(parts)


def _service_group_names(base_package: str) -> Tuple[str, str, str, str, str]:
    parts = [p for p in base_package.split('.') if p]
    skip = {'pyramid', 'components', 'services', 'data_model', 'base'}
    meaningful = [p for p in parts if p.lower() not in skip]
    fbs_namespace = '.'.join(['pyramid', 'services'] + [p.lower() for p in meaningful])
    cpp_base_ns = fbs_namespace.replace('.', '::')
    file_base = '_'.join(['pyramid', 'services'] + [p.lower() for p in meaningful])
    ada_parts = ['Pyramid', 'Services']
    for part in meaningful:
        ada_parts.append('_'.join(w.capitalize() for w in part.split('_')))
    ada_codec_pkg = '.'.join(ada_parts + ['Flatbuffers_Codec'])
    ada_codec_file = ada_codec_pkg.lower().replace('.', '-')
    return fbs_namespace, cpp_base_ns, file_base, ada_codec_pkg, ada_codec_file


def _ada_pkg_from_proto_pkg(proto_pkg: str) -> str:
    parts = [p for p in proto_pkg.split('.') if p]
    ada_parts = ['_'.join(w.capitalize() for w in seg.split('_')) for seg in parts]
    return '.'.join(ada_parts)


def _ada_types_pkg_for_full_type(full_type: str) -> str:
    proto_pkg = full_type.rsplit('.', 1)[0] if '.' in full_type else 'pyramid.data_model.base'
    return _ada_pkg_from_proto_pkg(proto_pkg) + '.Types'


def _service_proto_groups(index: ProtoTypeIndex) -> List[Tuple[str, List[ProtoFile]]]:
    grouped: Dict[str, List[ProtoFile]] = {}
    for pf in index.files:
        if not pf.services:
            continue
        key = _service_group_key(pf.package)
        if not key:
            continue
        grouped.setdefault(key, []).append(pf)
    return sorted(grouped.items(), key=lambda item: item[0])


def _alias_info(index: ProtoTypeIndex) -> Dict[str, Tuple[str, str]]:
    aliases: Dict[str, Tuple[str, str]] = {
        'Timestamp': ('double', 'value'),
    }
    for pf in index.files:
        for msg in pf.messages:
            fields = msg.all_fields()
            if len(fields) != 1 or fields[0].is_repeated:
                continue
            fld = fields[0]
            if fld.type in _PROTO_SCALARS and fld.name in _ALIAS_FIELD_NAMES:
                aliases[msg.name] = (fld.type, fld.name)
    return aliases


def _resolve_message(index: ProtoTypeIndex, type_name: str):
    return index.resolve_message(type_name) or index.resolve_message(type_name.split('.')[-1])


def _resolve_enum(index: ProtoTypeIndex, type_name: str):
    return index.resolve_enum(type_name) or index.resolve_enum(type_name.split('.')[-1])


def _inline_service_fields(
        msg, index: ProtoTypeIndex, aliases: Dict[str, Tuple[str, str]]
) -> Iterable[Tuple[ProtoField, str, bool]]:
    own_names = {f.name for f in msg.fields if f.name != 'base'}
    for field in msg.fields:
        if field.name == 'base' and not field.is_repeated:
            short = field.type.split('.')[-1]
            base_msg = _resolve_message(index, field.type)
            if base_msg and short not in aliases:
                for bf in base_msg.fields:
                    name = bf.name
                    if name in own_names:
                        name = short.lower() + '_' + name
                    yield bf, name, False
                continue
        yield field, field.name, False
    for oneof in msg.oneofs:
        for field in oneof.fields:
            yield field, field.name, True


def _field_kind_and_type(
        field: ProtoField,
        index: ProtoTypeIndex,
        aliases: Dict[str, Tuple[str, str]],
) -> Tuple[str, str, str]:
    short = field.type.split('.')[-1]
    if field.type in _FBS_SCALAR_MAP:
        kind = 'string' if field.type == 'string' else 'scalar'
        return kind, _FBS_SCALAR_MAP[field.type], field.type
    if short in aliases:
        proto_scalar, _ = aliases[short]
        kind = 'string' if proto_scalar == 'string' else 'scalar'
        return kind, _FBS_SCALAR_MAP[proto_scalar], proto_scalar
    if _resolve_enum(index, field.type):
        return 'enum', short, ''
    return 'message', short, ''


def _json_codec_namespace_for_type(full_type: str) -> str:
    parts = [part for part in full_type.split('.') if part]
    try:
        data_model_index = parts.index('data_model')
    except ValueError:
        return 'common'
    namespace_index = data_model_index + 1
    if namespace_index >= len(parts) - 1:
        return 'common'
    return parts[namespace_index]


def _is_generated_string_like(
        kind: str, proto_scalar: str,
) -> bool:
    return kind == 'string' or proto_scalar in ('string', 'bytes')


def _message_spec_from_proto(
        msg,
        full_type: str,
        index: ProtoTypeIndex,
        aliases: Dict[str, Tuple[str, str]],
) -> FlatMessageSpec:
    fields: List[FlatFieldSpec] = []
    field_index = 0
    for field, name, oneof in _inline_service_fields(msg, index, aliases):
        kind, type_name, proto_scalar = _field_kind_and_type(field, index, aliases)
        string_like = _is_generated_string_like(kind, proto_scalar)
        generated_optional = bool(oneof or field.is_optional)
        generated_presence = False
        if generated_optional:
            if kind == 'message':
                generated_presence = False
            elif string_like:
                generated_optional = bool(oneof)
            else:
                generated_presence = True
        fields.append(FlatFieldSpec(
            index=field_index,
            name=name,
            kind=kind,
            type_name=type_name,
            proto_scalar=proto_scalar,
            repeated=field.is_repeated,
            generated_optional=generated_optional,
            generated_presence=generated_presence,
            oneof=oneof,
        ))
        field_index += 1 + (1 if generated_presence else 0)
    return FlatMessageSpec(
        name=msg.name,
        cpp_type=f'pyramid::domain_model::{msg.name}',
        ada_type=camel_to_snake(msg.name),
        full_type=full_type,
        fields=fields,
        json_codec_ns=_json_codec_namespace_for_type(full_type),
    )


def _alias_root_spec(short_name: str, full_type: str,
                     aliases: Dict[str, Tuple[str, str]]) -> FlatMessageSpec:
    proto_scalar, field_name = aliases[short_name]
    kind = 'string' if proto_scalar == 'string' else 'scalar'
    type_name = _FBS_SCALAR_MAP[proto_scalar]
    table_name = f'{short_name}Value'
    return FlatMessageSpec(
        name=table_name,
        cpp_type=f'pyramid::domain_model::{short_name}',
        ada_type=camel_to_snake(short_name),
        full_type=full_type,
        fields=[FlatFieldSpec(
            index=0,
            name=field_name,
            kind=kind,
            type_name=type_name,
            proto_scalar=proto_scalar,
        )],
        json_codec_ns=_json_codec_namespace_for_type(full_type),
        is_alias_root=True,
        alias_scalar=proto_scalar,
        alias_field_name=field_name,
    )


def _collect_service_group(index: ProtoTypeIndex, base_package: str, service_files: Sequence[ProtoFile]) -> ServiceCodecGroup:
    aliases = _alias_info(index)
    root_types: List[str] = []
    stream_types: List[str] = []
    for pf in service_files:
        for svc in pf.services:
            for rpc in svc.rpcs:
                root_types.append(rpc.request_type)
                root_types.append(rpc.response_type)
                if rpc.server_streaming:
                    stream_types.append(rpc.response_type)

    reachable_message_ids: set = set()
    reachable_enum_ids: set = set()
    root_aliases: set = set()
    pending = list(root_types)
    seen_types = set()
    while pending:
        type_name = pending.pop()
        if type_name in seen_types:
            continue
        seen_types.add(type_name)
        short = type_name.split('.')[-1]
        if short in aliases:
            root_aliases.add(short)
            continue
        enum = _resolve_enum(index, type_name)
        if enum:
            reachable_enum_ids.add(id(enum))
            continue
        msg = _resolve_message(index, type_name)
        if msg is None:
            continue
        if id(msg) in reachable_message_ids:
            continue
        reachable_message_ids.add(id(msg))
        for field, _name, _oneof in _inline_service_fields(msg, index, aliases):
            kind, _type_name, _proto_scalar = _field_kind_and_type(field, index, aliases)
            if kind == 'enum':
                enum = _resolve_enum(index, field.type)
                if enum:
                    reachable_enum_ids.add(id(enum))
            elif kind == 'message':
                pending.append(field.type)

    ordered_enums = []
    ordered_messages = []
    for pf in index.files:
        for enum in pf.enums:
            if id(enum) in reachable_enum_ids:
                ordered_enums.append(enum)
        for msg in pf.messages:
            if id(msg) in reachable_message_ids:
                full_type = f'{pf.package}.{msg.name}' if pf.package else msg.name
                ordered_messages.append(_message_spec_from_proto(msg, full_type, index, aliases))

    spec_by_name = {spec.name: spec for spec in ordered_messages}
    remaining = list(ordered_messages)
    ordered_messages = []
    while remaining:
        progressed = False
        for spec in list(remaining):
            deps = {
                field.type_name for field in spec.fields
                if field.kind == 'message' and field.type_name in spec_by_name
            }
            if deps.issubset({done.name for done in ordered_messages}):
                ordered_messages.append(spec)
                remaining.remove(spec)
                progressed = True
        if not progressed:
            ordered_messages.extend(remaining)
            break

    for alias_name in sorted(root_aliases):
        alias_full_type = next((t for t in root_types if t.split('.')[-1] == alias_name), alias_name)
        ordered_messages.append(_alias_root_spec(alias_name, alias_full_type, aliases))

    stream_specs: List[FlatArraySpec] = []
    seen_arrays = set()
    for type_name in stream_types:
        short = type_name.split('.')[-1]
        if short in seen_arrays:
            continue
        seen_arrays.add(short)
        if short in aliases:
            proto_scalar, _field_name = aliases[short]
            stream_specs.append(FlatArraySpec(
                name=f'{short}Array',
                holder_name=f'{short}ArrayHolder',
                element_kind='string' if proto_scalar == 'string' else 'scalar',
                element_type_name=_FBS_SCALAR_MAP[proto_scalar],
                element_cpp_type=f'pyramid::domain_model::{short}',
                element_ada_type=camel_to_snake(short),
                element_full_type=type_name,
                json_codec_ns=_json_codec_namespace_for_type(type_name),
                element_proto_scalar=proto_scalar,
            ))
        else:
            stream_specs.append(FlatArraySpec(
                name=f'{short}Array',
                holder_name=f'{short}ArrayHolder',
                element_kind='message',
                element_type_name=short,
                element_cpp_type=f'pyramid::domain_model::{short}',
                element_ada_type=camel_to_snake(short),
                element_full_type=type_name,
                json_codec_ns=_json_codec_namespace_for_type(type_name),
            ))

    fbs_namespace, cpp_base_ns, file_base, ada_codec_pkg, ada_codec_file = _service_group_names(base_package)
    return ServiceCodecGroup(
        base_package=base_package,
        file_base=file_base,
        fbs_namespace=fbs_namespace,
        cpp_codec_ns=cpp_base_ns + '::flatbuffers_codec',
        cpp_base_ns=cpp_base_ns,
        wire_header=file_base + '_wire_types.hpp',
        generated_header=file_base + '_generated.h',
        ada_codec_pkg=ada_codec_pkg,
        ada_codec_file=ada_codec_file,
        message_specs=ordered_messages,
        array_specs=stream_specs,
        enum_specs=ordered_enums,
    )


# -- FlatBuffers type mapping -------------------------------------------------

_FBS_SCALAR_MAP = {
    'double': 'double', 'float': 'float',
    'int32': 'int', 'int64': 'long',
    'uint32': 'uint', 'uint64': 'ulong',
    'sint32': 'int', 'sint64': 'long',
    'fixed32': 'uint', 'fixed64': 'ulong',
    'sfixed32': 'int', 'sfixed64': 'long',
    'bool': 'bool', 'string': 'string', 'bytes': '[ubyte]',
}


def _fbs_type_name(type_name: str, current_package: str) -> str:
    if type_name in _FBS_SCALAR_MAP:
        return _FBS_SCALAR_MAP[type_name]
    if type_name == 'google.protobuf.Timestamp':
        return 'double'
    if '.' not in type_name:
        return type_name
    package_name, short_name = type_name.rsplit('.', 1)
    if package_name == current_package:
        return short_name
    return type_name


def _fbs_type(field: ProtoField, index: ProtoTypeIndex, current_package: str = '') -> str:
    """Map a proto field to its FlatBuffers schema type."""
    base = _fbs_type_name(field.type, current_package)

    if field.is_repeated:
        return f'[{base}]'
    return base


def _flatbuffers_union_name(oneof_name: str) -> str:
    return ''.join(w.capitalize() for w in oneof_name.split('_')) + 'Union'


def _flatbuffers_oneof_wrapper_name(msg_name: str, oneof_name: str, field_name: str) -> str:
    field_part = ''.join(w.capitalize() for w in field_name.split('_'))
    return f'{msg_name}{_flatbuffers_union_name(oneof_name)}{field_part}Value'


def _has_tactical_service_wire(index: ProtoTypeIndex) -> bool:
    return any(pf.package in _TACTICAL_SERVICE_PACKAGES for pf in index.files)


def _schema_type_for_field(field: FlatFieldSpec) -> str:
    base = field.type_name
    if field.repeated:
        return f'[{base}]'
    return base


def _schema_type_for_array(spec: FlatArraySpec) -> str:
    return f'[{spec.element_type_name}]'


class FlatBuffersBackend(codec_backends.CodecBackend):

    @property
    def name(self) -> str:
        return 'flatbuffers'

    @property
    def content_type(self) -> str:
        return 'application/flatbuffers'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated: List[Path] = []

        for pf in index.files:
            if not pf.messages and not pf.enums:
                continue

            pkg_parts = [p for p in pf.package.split('.') if p]
            file_base = '_'.join(pkg_parts)

            fbs_path = output_dir / (file_base + '.fbs')
            self._write_fbs_schema(fbs_path, pf, index)
            generated.append(fbs_path)

            hpp_path = output_dir / (file_base + '_flatbuffers_codec.hpp')
            self._write_cpp_wrapper(hpp_path, pf)
            generated.append(hpp_path)

        for base_package, files in _service_proto_groups(index):
            group = _collect_service_group(index, base_package, files)
            service_fbs = output_dir / (group.file_base + '.fbs')
            service_hpp = output_dir / (group.file_base + '_flatbuffers_codec.hpp')
            service_cpp = output_dir / (group.file_base + '_flatbuffers_codec.cpp')
            self._write_service_fbs_schema(service_fbs, group)
            self._write_service_cpp_header(service_hpp, group)
            self._write_service_cpp_impl(service_cpp, group)
            generated.extend([service_fbs, service_hpp, service_cpp])

        return generated

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        for pf in index.files:
            if not pf.messages and not pf.enums:
                continue

            pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
            file_base = '_'.join(p.lower() for p in pkg_parts)

            spec_path = output_dir / (file_base + '-flatbuffers_codec.ads')
            self._write_ada_spec(spec_path, pf)
            generated.append(spec_path)

        for base_package, files in _service_proto_groups(index):
            group = _collect_service_group(index, base_package, files)
            spec_path = output_dir / (group.ada_codec_file + '.ads')
            body_path = output_dir / (group.ada_codec_file + '.adb')
            self._write_service_ada_spec(spec_path, group)
            self._write_service_ada_body(body_path, group)
            generated.extend([spec_path, body_path])

        return generated

    # -- Proto-native .fbs schema generation ---------------------------------

    def _write_fbs_schema(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'// Auto-generated FlatBuffers schema from {pf.path.name}\n')
            f.write('// Do not edit -- regenerate from proto source\n\n')

            for imported in pf.imports:
                if not imported.endswith('.proto') or imported.startswith('google/protobuf/'):
                    continue
                include_stem = Path(imported).stem
                if not include_stem.startswith('pyramid.'):
                    include_stem = f'pyramid.{include_stem}'
                include_base = include_stem.replace('.', '_')
                f.write(f'include "{include_base}.fbs";\n')
            if pf.imports:
                filtered_imports = [
                    imported for imported in pf.imports
                    if imported.endswith('.proto') and not imported.startswith('google/protobuf/')
                ]
                if filtered_imports:
                    f.write('\n')

            if pf.package:
                f.write(f'namespace {pf.package};\n\n')

            for enum in pf.enums:
                max_val = max((v.number for v in enum.values), default=0)
                int_type = 'byte' if max_val < 128 else 'short' if max_val < 32768 else 'int'
                f.write(f'enum {enum.name} : {int_type} {{\n')
                for i, v in enumerate(enum.values):
                    suffix = enum.suffix_of(v.name)
                    fbs_name = suffix if suffix else v.name
                    comma = ',' if i < len(enum.values) - 1 else ''
                    f.write(f'  {fbs_name} = {v.number}{comma}\n')
                f.write('}\n\n')

            emitted_oneof_wrappers = set()
            for msg in pf.messages:
                for oo in msg.oneofs:
                    for field in oo.fields:
                        if field.type not in _FBS_SCALAR_MAP:
                            continue
                        wrapper_name = _flatbuffers_oneof_wrapper_name(msg.name, oo.name, field.name)
                        if wrapper_name in emitted_oneof_wrappers:
                            continue
                        emitted_oneof_wrappers.add(wrapper_name)
                        f.write(f'table {wrapper_name} {{\n')
                        f.write(f'  value:{_fbs_type(field, index, pf.package)};\n')
                        f.write('}\n\n')

            emitted_unions = set()
            for msg in pf.messages:
                for oo in msg.oneofs:
                    union_name = _flatbuffers_union_name(oo.name)
                    if union_name in emitted_unions:
                        continue
                    emitted_unions.add(union_name)
                    f.write(f'union {union_name} {{\n')
                    for i, field in enumerate(oo.fields):
                        member_type = _fbs_type_name(field.type, pf.package)
                        if field.type in _FBS_SCALAR_MAP:
                            member_type = _flatbuffers_oneof_wrapper_name(msg.name, oo.name, field.name)
                        comma = ',' if i < len(oo.fields) - 1 else ''
                        f.write(f'  {member_type}{comma}\n')
                    f.write('}\n\n')

            for msg in pf.messages:
                f.write(f'table {msg.name} {{\n')
                for fld in msg.fields:
                    f.write(f'  {fld.name}:{_fbs_type(fld, index, pf.package)};\n')
                for oo in msg.oneofs:
                    union_name = _flatbuffers_union_name(oo.name)
                    f.write(f'  {oo.name}:{union_name};\n')
                f.write('}\n\n')

    def _write_cpp_wrapper(self, path: Path, pf: ProtoFile):
        pkg_parts = [p for p in pf.package.split('.') if p]
        ns = '::'.join(pkg_parts) + '::flatbuffers_codec'
        fbs_base = '_'.join(pkg_parts)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated FlatBuffers PCL codec -- do not edit\n')
            f.write(f'// Backend: flatbuffers | Namespace: {ns}\n')
            f.write('#pragma once\n\n')
            f.write(f'// Include the flatc-generated header (run flatc on {fbs_base}.fbs)\n')
            f.write(f'#include "{fbs_base}_generated.h"\n\n')
            f.write('#include <flatbuffers/flatbuffers.h>\n')
            f.write('#include <cstddef>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <string>\n\n')
            f.write(f'namespace {ns} {{\n\n')
            f.write('static constexpr const char* kContentType = "application/flatbuffers";\n\n')

            for msg in pf.messages:
                f.write(f'inline std::string toBinary(const {msg.name}T& obj) {{\n')
                f.write('    flatbuffers::FlatBufferBuilder builder(1024);\n')
                f.write(f'    auto offset = {msg.name}::Pack(builder, &obj);\n')
                f.write('    builder.Finish(offset);\n')
                f.write('    return std::string(\n')
                f.write('        reinterpret_cast<const char*>(builder.GetBufferPointer()),\n')
                f.write('        builder.GetSize());\n')
                f.write('}\n\n')

                f.write(f'inline {msg.name}T fromBinary{msg.name}(const void* data, size_t size) {{\n')
                f.write(f'    (void) size;\n')
                f.write(f'    {msg.name}T result;\n')
                f.write(f'    auto* fb = flatbuffers::GetRoot<{msg.name}>(data);\n')
                f.write('    if (fb) fb->UnPackTo(&result);\n')
                f.write('    return result;\n')
                f.write('}\n\n')

                f.write(f'inline {msg.name}T fromBinary{msg.name}(const std::string& s) {{\n')
                f.write(f'    return fromBinary{msg.name}(s.data(), s.size());\n')
                f.write('}\n\n')

            f.write(f'}} // namespace {ns}\n')

    # -- Tactical service-wire generation ------------------------------------

    def _write_service_fbs_schema(self, path: Path, group: ServiceCodecGroup):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated service FlatBuffers schema\n')
            f.write(f'// Generated from proto service closure for {group.base_package}\n')
            f.write('\n')
            f.write(f'namespace {group.fbs_namespace};\n\n')

            for enum in group.enum_specs:
                f.write(f'enum {enum.name} : int {{\n')
                for i, value in enumerate(enum.values):
                    suffix = enum.suffix_of(value.name)
                    lit = suffix if suffix else value.name
                    comma = ',' if i < len(enum.values) - 1 else ''
                    f.write(f'  {lit} = {value.number}{comma}\n')
                f.write('}\n\n')

            for msg in group.message_specs:
                f.write(f'table {msg.name} {{\n')
                for field in msg.fields:
                    if field.generated_presence:
                        f.write(f'  has_{field.name}:bool;\n')
                    f.write(f'  {field.name}:{_schema_type_for_field(field)};\n')
                f.write('}\n\n')

            for array_spec in group.array_specs:
                f.write(f'table {array_spec.holder_name} {{\n')
                f.write(f'  items:{_schema_type_for_array(array_spec)};\n')
                f.write('}\n\n')

    def _write_service_cpp_header(self, path: Path, group: ServiceCodecGroup):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated service FlatBuffers codec\n')
            f.write(f'// Backend: flatbuffers | Namespace: {group.cpp_codec_ns}\n')
            f.write(f'// Generated from proto service closure for {group.base_package}\n')
            f.write('#pragma once\n\n')
            f.write('#include "pyramid_data_model_types.hpp"\n')
            f.write(f'#include "{group.generated_header}"\n\n')
            f.write('#include <flatbuffers/flatbuffers.h>\n')
            f.write('#include <cstddef>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write(f'namespace {group.cpp_codec_ns} {{\n\n')
            f.write('namespace data_model = pyramid::domain_model;\n')
            f.write('\n')
            f.write('static constexpr const char* kContentType = "application/flatbuffers";\n\n')

            for msg in group.message_specs:
                short = msg.cpp_type.split('::')[-1]
                f.write(f'std::string toBinary(const {msg.cpp_type}& msg);\n')
                f.write(f'{msg.cpp_type} fromBinary{short}(const void* data, size_t size);\n')
                f.write(f'inline {msg.cpp_type} fromBinary{short}(const std::string& s) {{\n')
                f.write(f'    return fromBinary{short}(s.data(), s.size());\n')
                f.write('}\n\n')

            for array_spec in group.array_specs:
                f.write(f'std::string toBinary(const std::vector<{array_spec.element_cpp_type}>& msg);\n')
                f.write(f'std::vector<{array_spec.element_cpp_type}> fromBinary{array_spec.name}(const void* data, size_t size);\n')
                f.write(f'inline std::vector<{array_spec.element_cpp_type}> fromBinary{array_spec.name}(const std::string& s) {{\n')
                f.write(f'    return fromBinary{array_spec.name}(s.data(), s.size());\n')
                f.write('}\n\n')

            f.write(f'}} // namespace {group.cpp_codec_ns}\n')

    def _write_service_cpp_impl(self, path: Path, group: ServiceCodecGroup):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated service FlatBuffers codec\n')
            f.write(f'#include "{group.file_base}_flatbuffers_codec.hpp"\n\n')
            codec_namespaces = {
                spec.json_codec_ns
                for spec in group.message_specs
                if spec.json_codec_ns not in ('base', 'wire')
            }
            codec_namespaces.update(
                spec.json_codec_ns
                for spec in group.array_specs
                if spec.json_codec_ns not in ('base', 'wire')
            )
            for codec_ns in sorted(codec_namespaces):
                f.write(f'#include "pyramid_data_model_{codec_ns}_codec.hpp"\n')
            f.write('#include <cstdlib>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <cstring>\n')
            f.write('#include <memory>\n')
            f.write('#include <nlohmann/json.hpp>\n')
            f.write('#include <stdexcept>\n')
            f.write('#include <utility>\n\n')
            f.write(f'namespace {group.cpp_codec_ns} {{\n\n')
            f.write(f'namespace fbs = {group.fbs_namespace.replace(".", "::")};\n\n')
            f.write('namespace {\n\n')
            f.write('template <typename OffsetT>\n')
            f.write('std::string finish_buffer(flatbuffers::FlatBufferBuilder& builder, OffsetT root) {\n')
            f.write('    builder.Finish(root);\n')
            f.write('    return std::string(\n')
            f.write('        reinterpret_cast<const char*>(builder.GetBufferPointer()),\n')
            f.write('        builder.GetSize());\n')
            f.write('}\n\n')
            f.write('template <typename TableT>\n')
            f.write('const TableT* verified_root(const void* data, size_t size, const char* type_name) {\n')
            f.write('    if (data == nullptr || size == 0) {\n')
            f.write('        throw std::runtime_error(std::string("empty ") + type_name + " flatbuffer");\n')
            f.write('    }\n')
            f.write('    auto* bytes = reinterpret_cast<const std::uint8_t*>(data);\n')
            f.write('    flatbuffers::Verifier verifier(bytes, size);\n')
            f.write('    if (!verifier.VerifyBuffer<TableT>()) {\n')
            f.write('        throw std::runtime_error(std::string("invalid ") + type_name + " flatbuffer");\n')
            f.write('    }\n')
            f.write('    return flatbuffers::GetRoot<TableT>(bytes);\n')
            f.write('}\n\n')
            self._emit_service_cpp_proto_helpers(f, group)
            f.write('} // namespace\n\n')
            self._emit_service_cpp_proto_exports(f, group)
            f.write(f'}} // namespace {group.cpp_codec_ns}\n')
            self._emit_service_cpp_json_bridge_exports(f, group)

    def _emit_service_cpp_proto_helpers(self, f, group: ServiceCodecGroup):
        for msg in group.message_specs:
            short = msg.cpp_type.split('::')[-1]
            if msg.is_alias_root:
                f.write(f'fbs::{msg.name}T to_fb(const {msg.cpp_type}& msg) {{\n')
                f.write(f'    fbs::{msg.name}T out{{}};\n')
                f.write(f'    out.{msg.alias_field_name} = msg;\n')
                f.write('    return out;\n')
                f.write('}\n\n')
                f.write(f'{msg.cpp_type} from_fb(const fbs::{msg.name}T& msg, {msg.cpp_type}* /*tag*/) {{\n')
                f.write(f'    return msg.{msg.alias_field_name};\n')
                f.write('}\n\n')
                continue

            f.write(f'fbs::{msg.name}T to_fb(const {msg.cpp_type}& msg) {{\n')
            f.write(f'    fbs::{msg.name}T out{{}};\n')
            for field in msg.fields:
                member = f'msg.{field.name}'
                if field.repeated:
                    if field.kind == 'message':
                        f.write(f'    out.{field.name}.reserve({member}.size());\n')
                        f.write(f'    for (const auto& item : {member}) {{\n')
                        f.write(f'        out.{field.name}.emplace_back(std::make_unique<fbs::{field.type_name}T>(to_fb(item)));\n')
                        f.write('    }\n')
                    elif field.kind == 'enum':
                        f.write(f'    out.{field.name}.reserve({member}.size());\n')
                        f.write(f'    for (const auto& item : {member}) {{\n')
                        f.write(f'        out.{field.name}.push_back(static_cast<fbs::{field.type_name}>(item));\n')
                        f.write('    }\n')
                    else:
                        f.write(f'    out.{field.name} = {member};\n')
                elif field.generated_presence:
                    f.write(f'    out.has_{field.name} = {member}.has_value();\n')
                    f.write(f'    if ({member}.has_value()) {{\n')
                    if field.kind == 'enum':
                        f.write(f'        out.{field.name} = static_cast<fbs::{field.type_name}>({member}.value());\n')
                    else:
                        f.write(f'        out.{field.name} = {member}.value();\n')
                    f.write('    }\n')
                elif field.generated_optional and field.kind == 'message':
                    f.write(f'    if ({member}.has_value()) {{\n')
                    f.write(f'        out.{field.name} = std::make_unique<fbs::{field.type_name}T>(to_fb({member}.value()));\n')
                    f.write('    }\n')
                elif field.generated_optional:
                    f.write(f'    if ({member}.has_value()) {{\n')
                    f.write(f'        out.{field.name} = {member}.value();\n')
                    f.write('    }\n')
                elif field.kind == 'message':
                    f.write(f'    out.{field.name} = std::make_unique<fbs::{field.type_name}T>(to_fb({member}));\n')
                elif field.kind == 'enum':
                    f.write(f'    out.{field.name} = static_cast<fbs::{field.type_name}>({member});\n')
                else:
                    f.write(f'    out.{field.name} = {member};\n')
            f.write('    return out;\n')
            f.write('}\n\n')

            f.write(f'{msg.cpp_type} from_fb(const fbs::{msg.name}T& msg, {msg.cpp_type}* /*tag*/) {{\n')
            f.write(f'    {msg.cpp_type} out{{}};\n')
            for field in msg.fields:
                member = f'msg.{field.name}'
                if field.repeated:
                    if field.kind == 'message':
                        f.write(f'    out.{field.name}.reserve({member}.size());\n')
                        f.write(f'    for (const auto& item : {member}) {{\n')
                        f.write('        if (item) {\n')
                        f.write(f'            out.{field.name}.push_back(from_fb(*item, static_cast<pyramid::domain_model::{field.type_name}*>(nullptr)));\n')
                        f.write('        }\n')
                        f.write('    }\n')
                    elif field.kind == 'enum':
                        f.write(f'    out.{field.name}.reserve({member}.size());\n')
                        f.write(f'    for (const auto& item : {member}) {{\n')
                        f.write(f'        out.{field.name}.push_back(static_cast<pyramid::domain_model::{field.type_name}>(item));\n')
                        f.write('    }\n')
                    else:
                        f.write(f'    out.{field.name} = {member};\n')
                elif field.generated_presence:
                    f.write(f'    if (msg.has_{field.name}) {{\n')
                    if field.kind == 'enum':
                        f.write(f'        out.{field.name} = static_cast<pyramid::domain_model::{field.type_name}>({member});\n')
                    else:
                        f.write(f'        out.{field.name} = {member};\n')
                    f.write('    }\n')
                elif field.generated_optional and field.kind == 'message':
                    f.write(f'    if ({member}) {{\n')
                    f.write(f'        out.{field.name} = from_fb(*{member}, static_cast<pyramid::domain_model::{field.type_name}*>(nullptr));\n')
                    f.write('    }\n')
                elif field.generated_optional:
                    f.write(f'    if (!{member}.empty()) {{\n')
                    f.write(f'        out.{field.name} = {member};\n')
                    f.write('    }\n')
                elif field.kind == 'message':
                    f.write(f'    if ({member}) out.{field.name} = from_fb(*{member}, static_cast<pyramid::domain_model::{field.type_name}*>(nullptr));\n')
                elif field.kind == 'enum':
                    f.write(f'    out.{field.name} = static_cast<pyramid::domain_model::{field.type_name}>({member});\n')
                else:
                    f.write(f'    out.{field.name} = {member};\n')
            f.write('    return out;\n')
            f.write('}\n\n')

        for array_spec in group.array_specs:
            f.write(f'fbs::{array_spec.holder_name}T to_fb(const std::vector<{array_spec.element_cpp_type}>& msg) {{\n')
            f.write(f'    fbs::{array_spec.holder_name}T out{{}};\n')
            if array_spec.element_kind == 'message':
                short = array_spec.element_cpp_type.split('::')[-1]
                f.write('    out.items.reserve(msg.size());\n')
                f.write('    for (const auto& item : msg) {\n')
                f.write(f'        out.items.emplace_back(std::make_unique<fbs::{short}T>(to_fb(item)));\n')
                f.write('    }\n')
            else:
                f.write('    out.items = msg;\n')
            f.write('    return out;\n')
            f.write('}\n\n')

            f.write(f'std::vector<{array_spec.element_cpp_type}> from_fb(const fbs::{array_spec.holder_name}T& msg) {{\n')
            f.write(f'    std::vector<{array_spec.element_cpp_type}> out{{}};\n')
            if array_spec.element_kind == 'message':
                short = array_spec.element_cpp_type.split('::')[-1]
                f.write('    out.reserve(msg.items.size());\n')
                f.write('    for (const auto& item : msg.items) {\n')
                f.write('        if (item) {\n')
                f.write(f'            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::{short}*>(nullptr)));\n')
                f.write('        }\n')
                f.write('    }\n')
            else:
                f.write('    out = msg.items;\n')
            f.write('    return out;\n')
            f.write('}\n\n')

    def _emit_service_cpp_proto_exports(self, f, group: ServiceCodecGroup):
        for msg in group.message_specs:
            short = msg.cpp_type.split('::')[-1]
            f.write(f'std::string toBinary(const {msg.cpp_type}& msg) {{\n')
            f.write('    flatbuffers::FlatBufferBuilder builder(256);\n')
            f.write('    auto object = to_fb(msg);\n')
            f.write(f'    return finish_buffer(builder, fbs::{msg.name}::Pack(builder, &object));\n')
            f.write('}\n\n')
            f.write(f'{msg.cpp_type} fromBinary{short}(const void* data, size_t size) {{\n')
            f.write(f'    auto* root = verified_root<fbs::{msg.name}>(data, size, "{short}");\n')
            f.write(f'    fbs::{msg.name}T object{{}};\n')
            f.write('    root->UnPackTo(&object);\n')
            f.write(f'    return from_fb(object, static_cast<{msg.cpp_type}*>(nullptr));\n')
            f.write('}\n\n')

        for array_spec in group.array_specs:
            f.write(f'std::string toBinary(const std::vector<{array_spec.element_cpp_type}>& msg) {{\n')
            f.write('    flatbuffers::FlatBufferBuilder builder(256);\n')
            f.write('    auto object = to_fb(msg);\n')
            f.write(f'    return finish_buffer(builder, fbs::{array_spec.holder_name}::Pack(builder, &object));\n')
            f.write('}\n\n')
            f.write(f'std::vector<{array_spec.element_cpp_type}> fromBinary{array_spec.name}(const void* data, size_t size) {{\n')
            f.write(f'    auto* root = verified_root<fbs::{array_spec.holder_name}>(data, size, "{array_spec.name}");\n')
            f.write(f'    fbs::{array_spec.holder_name}T object{{}};\n')
            f.write('    root->UnPackTo(&object);\n')
            f.write('    return from_fb(object);\n')
            f.write('}\n\n')

    def _ffi_symbol(self, group: ServiceCodecGroup, short_name: str, direction: str) -> str:
        return f'{group.file_base}_{short_name}_{direction}_json'

    def _cpp_json_encode_expr(self, group: ServiceCodecGroup, json_ns: str, cpp_type: str, expr: str) -> str:
        if json_ns == 'base':
            return f'nlohmann::json({expr}).dump()'
        if json_ns == 'wire':
            return f'{group.cpp_base_ns}::json_codec::toJson({expr})'
        return f'pyramid::domain_model::{json_ns}::toJson({expr})'

    def _cpp_json_decode_expr(self, group: ServiceCodecGroup, json_ns: str, cpp_type: str, expr: str) -> str:
        if json_ns == 'base':
            return f'nlohmann::json::parse({expr}).get<{cpp_type}>()'
        if json_ns == 'wire':
            short = cpp_type.split('::')[-1]
            snake = short[0].lower() + short[1:]
            return f'{group.cpp_base_ns}::json_codec::{snake}FromJson({expr})'
        return f'pyramid::domain_model::{json_ns}::fromJson({expr}, static_cast<{cpp_type}*>(nullptr))'

    def _emit_service_cpp_json_bridge_exports(self, f, group: ServiceCodecGroup):
        f.write('\nextern "C" {\n\n')
        f.write(f'void {group.file_base}_free_buffer(void* data) {{\n')
        f.write('    std::free(data);\n')
        f.write('}\n\n')

        for msg in group.message_specs:
            short = msg.cpp_type.split('::')[-1]
            to_name = self._ffi_symbol(group, short, 'to_flatbuffer')
            from_name = self._ffi_symbol(group, short, 'from_flatbuffer')
            decode_expr = self._cpp_json_decode_expr(group, msg.json_codec_ns, msg.cpp_type, 'std::string(json ? json : "")')
            encode_expr = self._cpp_json_encode_expr(group, msg.json_codec_ns, msg.cpp_type, 'value')
            f.write(f'void* {to_name}(const char* json, size_t* size_out) {{\n')
            f.write('    if (size_out) *size_out = 0;\n')
            f.write('    try {\n')
            f.write(f'        auto value = {decode_expr};\n')
            f.write(f'        auto payload = {group.cpp_codec_ns}::toBinary(value);\n')
            f.write('        if (size_out) *size_out = payload.size();\n')
            f.write('        if (payload.empty()) return nullptr;\n')
            f.write('        void* out = std::malloc(payload.size());\n')
            f.write('        if (!out) return nullptr;\n')
            f.write('        std::memcpy(out, payload.data(), payload.size());\n')
            f.write('        return out;\n')
            f.write('    } catch (...) {\n')
            f.write('        return nullptr;\n')
            f.write('    }\n')
            f.write('}\n\n')
            f.write(f'char* {from_name}(const void* data, size_t size) {{\n')
            f.write('    try {\n')
            f.write(f'        auto value = {group.cpp_codec_ns}::fromBinary{short}(data, size);\n')
            f.write(f'        auto json = {encode_expr};\n')
            f.write('        char* out = static_cast<char*>(std::malloc(json.size() + 1));\n')
            f.write('        if (!out) return nullptr;\n')
            f.write('        std::memcpy(out, json.c_str(), json.size() + 1);\n')
            f.write('        return out;\n')
            f.write('    } catch (...) {\n')
            f.write('        return nullptr;\n')
            f.write('    }\n')
            f.write('}\n\n')

        for array_spec in group.array_specs:
            short = array_spec.name
            to_name = self._ffi_symbol(group, short, 'to_flatbuffer')
            from_name = self._ffi_symbol(group, short, 'from_flatbuffer')
            f.write(f'void* {to_name}(const char* json, size_t* size_out) {{\n')
            f.write('    if (size_out) *size_out = 0;\n')
            f.write('    try {\n')
            f.write('        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));\n')
            f.write(f'        std::vector<{array_spec.element_cpp_type}> values;\n')
            f.write('        if (arr.is_array()) {\n')
            f.write('            values.reserve(arr.size());\n')
            f.write('            for (const auto& item : arr) {\n')
            decode_expr = self._cpp_json_decode_expr(group, array_spec.json_codec_ns, array_spec.element_cpp_type, 'item.dump()')
            f.write(f'                values.push_back({decode_expr});\n')
            f.write('            }\n')
            f.write('        }\n')
            f.write(f'        auto payload = {group.cpp_codec_ns}::toBinary(values);\n')
            f.write('        if (size_out) *size_out = payload.size();\n')
            f.write('        if (payload.empty()) return nullptr;\n')
            f.write('        void* out = std::malloc(payload.size());\n')
            f.write('        if (!out) return nullptr;\n')
            f.write('        std::memcpy(out, payload.data(), payload.size());\n')
            f.write('        return out;\n')
            f.write('    } catch (...) {\n')
            f.write('        return nullptr;\n')
            f.write('    }\n')
            f.write('}\n\n')
            f.write(f'char* {from_name}(const void* data, size_t size) {{\n')
            f.write('    try {\n')
            f.write(f'        auto values = {group.cpp_codec_ns}::fromBinary{short}(data, size);\n')
            f.write('        nlohmann::json arr = nlohmann::json::array();\n')
            f.write('        for (const auto& item : values) {\n')
            encode_expr = self._cpp_json_encode_expr(group, array_spec.json_codec_ns, array_spec.element_cpp_type, 'item')
            f.write(f'            arr.push_back(nlohmann::json::parse({encode_expr}));\n')
            f.write('        }\n')
            f.write('        auto json = arr.dump();\n')
            f.write('        char* out = static_cast<char*>(std::malloc(json.size() + 1));\n')
            f.write('        if (!out) return nullptr;\n')
            f.write('        std::memcpy(out, json.c_str(), json.size() + 1);\n')
            f.write('        return out;\n')
            f.write('    } catch (...) {\n')
            f.write('        return nullptr;\n')
            f.write('    }\n')
            f.write('}\n\n')

        f.write('} // extern "C"\n')

    # -- Ada thin binding -----------------------------------------------------

    def _write_ada_spec(self, path: Path, pf: ProtoFile):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.Flatbuffers_Codec'

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated FlatBuffers codec spec -- do not edit\n')
            f.write(f'--  Backend: flatbuffers | Package: {pkg_name}\n')
            f.write('--\n')
            f.write('--  This package provides thin bindings to the C++ FlatBuffers codec.\n')
            f.write('--  Actual ser/de is performed via C interop (Import pragma).\n\n')
            f.write('with Interfaces.C; use Interfaces.C;\n')
            f.write('with System;\n\n')
            f.write(f'package {pkg_name} is\n\n')
            f.write('   Content_Type : constant String := "application/flatbuffers";\n\n')

            for msg in pf.messages:
                ada_name = camel_to_snake(msg.name)
                f.write('   function To_Binary (Msg : System.Address)\n')
                f.write('     return Interfaces.C.Strings.chars_ptr\n')
                f.write('     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{msg.name}_to_flatbuffer";\n\n')
                f.write(f'   function From_Binary_{ada_name}\n')
                f.write('     (Data : System.Address; Size : Interfaces.C.size_t)\n')
                f.write('     return System.Address\n')
                f.write('     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{msg.name}_from_flatbuffer";\n\n')

            f.write(f'end {pkg_name};\n')

    def _write_service_ada_spec(self, path: Path, group: ServiceCodecGroup):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated service FlatBuffers codec\n')
            f.write('--  Backend: flatbuffers\n')
            f.write(f'--  Generated from proto service closure for {group.base_package}\n\n')
            type_withs = {self._ada_type_pkg_for_message(msg) for msg in group.message_specs}
            for pkg in sorted(type_withs):
                f.write(f'with {pkg};\n')
            if type_withs:
                f.write('\n')
            f.write(f'package {group.ada_codec_pkg} is\n')
            f.write('   Content_Type : constant String := "application/flatbuffers";\n\n')

            for msg in group.message_specs:
                suffix = camel_to_snake(msg.cpp_type.split('::')[-1])
                type_ref = self._ada_type_ref_for_message(msg)
                f.write(f'   function To_Binary_{suffix} (Json : String) return String;\n')
                f.write(f'   function To_Binary_{suffix} (Msg : {type_ref}) return String;\n')
                f.write(f'   function From_Binary_{suffix} (Payload : String) return String;\n\n')
                f.write(f'   function From_Binary_{suffix}\n')
                f.write(f'     (Payload : String; Tag : access {type_ref}) return {type_ref};\n\n')

            for array_spec in group.array_specs:
                suffix = camel_to_snake(array_spec.name)
                f.write(f'   function To_Binary_{suffix} (Json : String) return String;\n')
                f.write(f'   function From_Binary_{suffix} (Payload : String) return String;\n\n')

            f.write(f'end {group.ada_codec_pkg};\n')

    def _write_service_ada_body(self, path: Path, group: ServiceCodecGroup):
        specs: List[Tuple[str, str]] = []
        for msg in group.message_specs:
            short = msg.cpp_type.split('::')[-1]
            specs.append((camel_to_snake(short), short))
        for array_spec in group.array_specs:
            specs.append((camel_to_snake(array_spec.name), array_spec.name))

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated service FlatBuffers codec\n\n')
            f.write('with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;\n')
            f.write('with Ada.Unchecked_Conversion;\n')
            f.write('with GNATCOLL.JSON;  use GNATCOLL.JSON;\n')
            f.write('with Interfaces.C;\n')
            f.write('with Interfaces.C.Strings;\n')
            f.write('with System;\n')
            codec_withs = {self._ada_codec_pkg_for_message(msg) for msg in group.message_specs
                           if self._ada_codec_pkg_for_message(msg)}
            for pkg in sorted(codec_withs):
                f.write(f'with {pkg};\n')
            f.write('\n')
            f.write(f'package body {group.ada_codec_pkg} is\n')
            f.write('   use type Interfaces.C.size_t;\n')
            f.write('   use type Interfaces.C.Strings.chars_ptr;\n')
            f.write('   use type System.Address;\n\n')
            f.write('   function To_Address is new\n')
            f.write('     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);\n\n')
            f.write('   function Copy_From_Buffer\n')
            f.write('     (Data : System.Address; Size : Interfaces.C.size_t) return String\n')
            f.write('   is\n')
            f.write('      type Char_Array is array (1 .. Natural (Size)) of Character;\n')
            f.write('      pragma Pack (Char_Array);\n')
            f.write('   begin\n')
            f.write('      if Data = System.Null_Address or else Size = 0 then\n')
            f.write('         return "";\n')
            f.write('      end if;\n')
            f.write('\n')
            f.write('      declare\n')
            f.write('         Chars : Char_Array;\n')
            f.write("         for Chars'Address use Data;\n")
            f.write('         pragma Import (Ada, Chars);\n')
            f.write('      begin\n')
            f.write('         return String (Chars);\n')
            f.write('      end;\n')
            f.write('   end Copy_From_Buffer;\n\n')
            f.write('   procedure Free_Buffer (Data : System.Address)\n')
            f.write('     with Import, Convention => C,\n')
            f.write(f'          External_Name => "{group.file_base}_free_buffer";\n\n')

            for ada_suffix, short_name in specs:
                to_symbol = self._ffi_symbol(group, short_name, 'to_flatbuffer')
                from_symbol = self._ffi_symbol(group, short_name, 'from_flatbuffer')
                imported_to = f'Imported_To_Binary_{ada_suffix}'
                imported_from = f'Imported_From_Binary_{ada_suffix}'
                f.write(f'   function {imported_to}\n')
                f.write('     (Json     : Interfaces.C.Strings.chars_ptr;\n')
                f.write('      Size_Out : access Interfaces.C.size_t) return System.Address\n')
                f.write('     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{to_symbol}";\n\n')
                f.write(f'   function {imported_from}\n')
                f.write('     (Data : System.Address; Size : Interfaces.C.size_t)\n')
                f.write('      return Interfaces.C.Strings.chars_ptr\n')
                f.write('     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{from_symbol}";\n\n')

            for ada_suffix, _short_name in specs:
                imported_to = f'Imported_To_Binary_{ada_suffix}'
                imported_from = f'Imported_From_Binary_{ada_suffix}'
                f.write(f'   function To_Binary_{ada_suffix} (Json : String) return String is\n')
                f.write('      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);\n')
                f.write('      Size_Out : aliased Interfaces.C.size_t := 0;\n')
                f.write(f'      Data     : System.Address := {imported_to} (Json_C, Size_Out\'Access);\n')
                f.write('   begin\n')
                f.write('      Interfaces.C.Strings.Free (Json_C);\n')
                f.write('      if Data = System.Null_Address then\n')
                f.write(f'         raise Constraint_Error with "FlatBuffers encode failed for {ada_suffix}";\n')
                f.write('      end if;\n')
                f.write('\n')
                f.write('      declare\n')
                f.write('         Result : constant String := Copy_From_Buffer (Data, Size_Out);\n')
                f.write('      begin\n')
                f.write('         Free_Buffer (Data);\n')
                f.write('         return Result;\n')
                f.write('      end;\n')
                f.write(f'   end To_Binary_{ada_suffix};\n\n')
                f.write(f'   function From_Binary_{ada_suffix} (Payload : String) return String is\n')
                f.write('      Payload_Bytes : aliased constant String := Payload;\n')
                f.write(f'      Json_C : Interfaces.C.Strings.chars_ptr := {imported_from}\n')
                f.write('        ((if Payload_Bytes\'Length = 0\n')
                f.write('          then System.Null_Address\n')
                f.write('          else Payload_Bytes (Payload_Bytes\'First)\'Address),\n')
                f.write('         Interfaces.C.size_t (Payload_Bytes\'Length));\n')
                f.write('   begin\n')
                f.write('      if Json_C = Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'         raise Constraint_Error with "FlatBuffers decode failed for {ada_suffix}";\n')
                f.write('      end if;\n')
                f.write('\n')
                f.write('      declare\n')
                f.write('         Result : constant String := Interfaces.C.Strings.Value (Json_C);\n')
                f.write('      begin\n')
                f.write('         Free_Buffer (To_Address (Json_C));\n')
                f.write('         return Result;\n')
                f.write('      end;\n')
                f.write(f'   end From_Binary_{ada_suffix};\n\n')

            for msg in group.message_specs:
                suffix = camel_to_snake(msg.cpp_type.split('::')[-1])
                type_ref = self._ada_type_ref_for_message(msg)
                encode_expr = self._ada_json_encode_expr_for_message(msg, 'Msg')
                f.write(f'   function To_Binary_{suffix} (Msg : {type_ref}) return String is\n')
                f.write('   begin\n')
                f.write(f'      return To_Binary_{suffix} ({encode_expr});\n')
                f.write(f'   end To_Binary_{suffix};\n\n')
                f.write(f'   function From_Binary_{suffix}\n')
                f.write(f'     (Payload : String; Tag : access {type_ref}) return {type_ref}\n')
                f.write('   is\n')
                f.write('      pragma Unreferenced (Tag);\n')
                f.write(f'      Json   : constant String := From_Binary_{suffix} (Payload);\n')
                f.write(f'      Result : {type_ref};\n')
                f.write('   begin\n')
                if msg.full_type.endswith('.Identifier'):
                    f.write('      declare\n')
                    f.write('         Value : constant JSON_Value := Read (Json);\n')
                    f.write("         Text  : constant String := String'(UTF8_String'(Get (Value)));\n")
                    f.write('      begin\n')
                    f.write('         Result := To_Unbounded_String (Text);\n')
                    f.write('      end;\n')
                else:
                    decode_stmt = self._ada_json_decode_stmt_for_message(msg, 'Json', 'Result')
                    f.write(f'      {decode_stmt}\n')
                f.write('      return Result;\n')
                f.write(f'   end From_Binary_{suffix};\n\n')

            f.write(f'end {group.ada_codec_pkg};\n')

    def _ada_type_pkg_for_message(self, msg: FlatMessageSpec) -> str:
        return _ada_types_pkg_for_full_type(msg.full_type)

    def _ada_type_ref_for_message(self, msg: FlatMessageSpec) -> str:
        return f'{self._ada_type_pkg_for_message(msg)}.{msg.ada_type}'

    def _ada_codec_pkg_for_message(self, msg: FlatMessageSpec) -> str:
        if msg.full_type.endswith('.Identifier'):
            return ''
        type_pkg = self._ada_type_pkg_for_message(msg)
        if type_pkg.endswith('.Base.Types'):
            return ''
        return type_pkg + '_Codec'

    def _ada_json_encode_expr_for_message(self, msg: FlatMessageSpec, accessor: str) -> str:
        if msg.full_type.endswith('.Identifier'):
            return (
                'String\'(Write (Create (UTF8_String\'('
                f'Ada.Strings.Unbounded.To_String ({accessor})'
                '))))'
            )
        codec_pkg = self._ada_codec_pkg_for_message(msg)
        return f'{codec_pkg}.To_Json ({accessor})'

    def _ada_json_decode_stmt_for_message(self, msg: FlatMessageSpec, json_var: str, result_var: str) -> str:
        codec_pkg = self._ada_codec_pkg_for_message(msg)
        return f'{result_var} := {codec_pkg}.From_Json ({json_var}, null);'


codec_backends.register(FlatBuffersBackend())
