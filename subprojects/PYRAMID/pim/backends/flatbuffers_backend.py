#!/usr/bin/env python3
"""
FlatBuffers Codec Backend

Generates:
  1. .fbs schema files from proto definitions (proto message -> FlatBuffers table)
  2. C++ wrapper headers for proto-native flatbuffer artefacts
  3. Tactical service-wire FlatBuffers codec artefacts from json_schema.py

Service-wire codecs are generated against real `flatc` output.  The service
runtime still exposes a generic `wrapPayload` / `unwrapPayload` surface for
domain-shaped RPCs, but that wrapper is itself a proper FlatBuffers table
(`JsonPayload`) rather than the old ad hoc "PWFB" prefix envelope.
"""

from pathlib import Path
from typing import Iterable, List

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import codec_backends
import json_schema as service_schema
from proto_parser import (
    ProtoTypeIndex, ProtoFile, ProtoField,
    camel_to_snake, _PROTO_SCALARS,
)


_TACTICAL_SERVICE_PACKAGES = frozenset({
    'pyramid.components.tactical_objects.services.provided',
    'pyramid.components.tactical_objects.services.consumed',
})

_SERVICE_FILE_BASE = 'pyramid_services_tactical_objects'
_SERVICE_WIRE_HEADER = 'pyramid_services_tactical_objects_wire_types.hpp'
_SERVICE_GENERATED_HEADER = _SERVICE_FILE_BASE + '_generated.h'
_SERVICE_CODEC_NS = 'pyramid::services::tactical_objects::flatbuffers_codec'
_SERVICE_ADA_CODEC_PKG = 'Pyramid.Services.Tactical_Objects.Flatbuffers_Codec'
_SERVICE_ADA_CODEC_FILE = 'pyramid-services-tactical_objects-flatbuffers_codec'


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


def _fbs_type(field: ProtoField, index: ProtoTypeIndex) -> str:
    """Map a proto field to its FlatBuffers schema type."""
    if field.type in _FBS_SCALAR_MAP:
        base = _FBS_SCALAR_MAP[field.type]
    else:
        base = field.short_type

    if field.is_repeated:
        return f'[{base}]'
    return base


def _has_tactical_service_wire(index: ProtoTypeIndex) -> bool:
    return any(pf.package in _TACTICAL_SERVICE_PACKAGES for pf in index.files)


def _service_enum_specs() -> Iterable[service_schema.EnumSpec]:
    seen = set()
    for spec in service_schema.ENUM_SPECS.values():
        if spec.cpp_type in seen:
            continue
        seen.add(spec.cpp_type)
        yield spec


def _service_field_fbs_type(field: service_schema.Field) -> str:
    if field.kind in service_schema.ENUM_SPECS:
        return service_schema.ENUM_SPECS[field.kind].cpp_type
    return {
        service_schema.FieldKind.STRING: 'string',
        service_schema.FieldKind.IDENTIFIER: 'string',
        service_schema.FieldKind.DOUBLE: 'double',
        service_schema.FieldKind.BOOL: 'bool',
    }[field.kind]


def _service_encode_stmt(field: service_schema.Field) -> str:
    member = f'msg.{field.name}'
    if field.kind in service_schema.ENUM_SPECS:
        return f'w.i32(static_cast<std::int32_t>({member}));'
    if field.kind in (service_schema.FieldKind.STRING,
                      service_schema.FieldKind.IDENTIFIER):
        return f'w.str({member});'
    if field.kind == service_schema.FieldKind.DOUBLE:
        return f'w.f64({member});'
    return f'w.u8({member});'


def _service_decode_stmt(field: service_schema.Field) -> str:
    member = f'msg.{field.name}'
    if field.kind in service_schema.ENUM_SPECS:
        enum_type = f'pyramid::data_model::{field.cpp_type}'
        return (
            'if (!r.i32(tmp_i32)) return false;\n'
            f'    {member} = static_cast<{enum_type}>(tmp_i32);'
        )
    if field.kind in (service_schema.FieldKind.STRING,
                      service_schema.FieldKind.IDENTIFIER):
        return f'if (!r.str({member})) return false;'
    if field.kind == service_schema.FieldKind.DOUBLE:
        return f'if (!r.f64({member})) return false;'
    return f'if (!r.u8({member})) return false;'


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

        if _has_tactical_service_wire(index):
            service_fbs = output_dir / (_SERVICE_FILE_BASE + '.fbs')
            service_hpp = output_dir / (_SERVICE_FILE_BASE + '_flatbuffers_codec.hpp')
            service_cpp = output_dir / (_SERVICE_FILE_BASE + '_flatbuffers_codec.cpp')
            self._write_service_fbs_schema(service_fbs)
            self._write_service_cpp_header(service_hpp)
            self._write_service_cpp_impl(service_cpp)
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

        if _has_tactical_service_wire(index):
            spec_path = output_dir / (_SERVICE_ADA_CODEC_FILE + '.ads')
            body_path = output_dir / (_SERVICE_ADA_CODEC_FILE + '.adb')
            self._write_service_ada_spec(spec_path)
            self._write_service_ada_body(body_path)
            generated.extend([spec_path, body_path])

        return generated

    # -- Proto-native .fbs schema generation ---------------------------------

    def _write_fbs_schema(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'// Auto-generated FlatBuffers schema from {pf.path.name}\n')
            f.write('// Do not edit — regenerate from proto source\n\n')

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

            emitted_unions = set()
            for msg in pf.messages:
                for oo in msg.oneofs:
                    union_name = ''.join(w.capitalize() for w in oo.name.split('_')) + 'Union'
                    if union_name in emitted_unions:
                        continue
                    emitted_unions.add(union_name)
                    f.write(f'union {union_name} {{\n')
                    for i, field in enumerate(oo.fields):
                        comma = ',' if i < len(oo.fields) - 1 else ''
                        f.write(f'  {field.short_type}{comma}\n')
                    f.write('}\n\n')

            for msg in pf.messages:
                f.write(f'table {msg.name} {{\n')
                for fld in msg.fields:
                    f.write(f'  {fld.name}:{_fbs_type(fld, index)};\n')
                for oo in msg.oneofs:
                    union_name = ''.join(w.capitalize() for w in oo.name.split('_')) + 'Union'
                    f.write(f'  {oo.name}:{union_name};\n')
                f.write('}\n\n')

    def _write_cpp_wrapper(self, path: Path, pf: ProtoFile):
        pkg_parts = [p for p in pf.package.split('.') if p]
        ns = '::'.join(pkg_parts) + '::flatbuffers_codec'
        fbs_base = '_'.join(pkg_parts)

        with open(path, 'w', encoding='utf-8') as f:
            f.write('// Auto-generated FlatBuffers PCL codec — do not edit\n')
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

    def _write_service_fbs_schema(self, path: Path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('// Auto-generated tactical service FlatBuffers schema\n')
            f.write('// Generated from pim/json_schema.py\n\n')
            f.write('namespace pyramid.services.tactical_objects;\n\n')

            for spec in _service_enum_specs():
                entries = spec.table()
                max_val = max((ordinal for *_rest, ordinal in entries), default=0)
                int_type = 'byte' if max_val < 128 else 'short' if max_val < 32768 else 'int'
                f.write(f'enum {spec.cpp_type} : {int_type} {{\n')
                for i, (_proto, _ada, cpp_lit, ordinal) in enumerate(entries):
                    comma = ',' if i < len(entries) - 1 else ''
                    f.write(f'  {cpp_lit} = {ordinal}{comma}\n')
                f.write('}\n\n')

            for schema_def in service_schema.ALL_SCHEMAS:
                f.write(f'table {schema_def.cpp_name} {{\n')
                for field in schema_def.fields:
                    f.write(f'  {field.name}:{_service_field_fbs_type(field)};\n')
                f.write('}\n\n')
                if schema_def.is_array_response:
                    f.write(f'table {schema_def.cpp_name}ArrayHolder {{\n')
                    f.write(f'  items:[{schema_def.cpp_name}];\n')
                    f.write('}\n\n')

            f.write('table JsonPayload {\n')
            f.write('  payload:string;\n')
            f.write('}\n')

    def _write_service_cpp_header(self, path: Path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('// Auto-generated tactical service FlatBuffers codec\n')
            f.write(f'// Backend: flatbuffers | Namespace: {_SERVICE_CODEC_NS}\n')
            f.write('// Generated from pim/json_schema.py\n')
            f.write('#pragma once\n\n')
            f.write(f'#include "{_SERVICE_WIRE_HEADER}"\n\n')
            f.write(f'#include "{_SERVICE_GENERATED_HEADER}"\n\n')
            f.write('#include <flatbuffers/flatbuffers.h>\n')
            f.write('#include <cstddef>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write(f'namespace {_SERVICE_CODEC_NS} {{\n\n')
            f.write('namespace wire_types = pyramid::services::tactical_objects::wire_types;\n\n')
            f.write('static constexpr const char* kContentType = "application/flatbuffers";\n\n')
            f.write('std::string wrapPayload(const std::string& payload);\n')
            f.write('std::string unwrapPayload(const void* data, size_t size);\n')
            f.write('inline std::string unwrapPayload(const std::string& payload) {\n')
            f.write('    return unwrapPayload(payload.data(), payload.size());\n')
            f.write('}\n\n')

            for schema_def in service_schema.ALL_SCHEMAS:
                cpp_name = schema_def.cpp_name
                f.write(f'std::string toBinary(const wire_types::{cpp_name}& msg);\n')
                f.write(f'wire_types::{cpp_name} fromBinary{cpp_name}(const void* data, size_t size);\n')
                f.write(f'inline wire_types::{cpp_name} fromBinary{cpp_name}(const std::string& s) {{\n')
                f.write(f'    return fromBinary{cpp_name}(s.data(), s.size());\n')
                f.write('}\n\n')
                if schema_def.is_array_response:
                    alias_name = cpp_name + 'Array'
                    f.write(f'std::string toBinary(const wire_types::{alias_name}& msg);\n')
                    f.write(f'wire_types::{alias_name} fromBinary{alias_name}(const void* data, size_t size);\n')
                    f.write(f'inline wire_types::{alias_name} fromBinary{alias_name}(const std::string& s) {{\n')
                    f.write(f'    return fromBinary{alias_name}(s.data(), s.size());\n')
                    f.write('}\n\n')

            f.write(f'}} // namespace {_SERVICE_CODEC_NS}\n')

    def _write_service_cpp_impl(self, path: Path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('// Auto-generated tactical service FlatBuffers codec\n')
            f.write(f'#include "{_SERVICE_FILE_BASE}_flatbuffers_codec.hpp"\n\n')
            f.write('#include <memory>\n')
            f.write('#include <stdexcept>\n')
            f.write('#include <utility>\n\n')
            f.write(f'namespace {_SERVICE_CODEC_NS} {{\n\n')
            f.write('namespace fbs = pyramid::services::tactical_objects;\n\n')
            f.write('namespace {\n\n')
            f.write('template <typename OffsetT>\n')
            f.write('std::string finish_buffer(flatbuffers::FlatBufferBuilder& builder, OffsetT root) {\n')
            f.write('    builder.Finish(root);\n')
            f.write('    return std::string(\n')
            f.write('        reinterpret_cast<const char*>(builder.GetBufferPointer()),\n')
            f.write('        builder.GetSize());\n')
            f.write('}\n\n')

            for schema_def in service_schema.ALL_SCHEMAS:
                cpp_name = schema_def.cpp_name
                f.write(f'fbs::{cpp_name}T to_fb(const wire_types::{cpp_name}& msg) {{\n')
                f.write(f'    fbs::{cpp_name}T out{{}};\n')
                for field in schema_def.fields:
                    if field.kind in service_schema.ENUM_SPECS:
                        f.write(f'    out.{field.name} = static_cast<fbs::{field.cpp_type}>(msg.{field.name});\n')
                    else:
                        f.write(f'    out.{field.name} = msg.{field.name};\n')
                f.write('    return out;\n')
                f.write('}\n\n')

                f.write(f'wire_types::{cpp_name} from_fb(const fbs::{cpp_name}T& msg) {{\n')
                f.write(f'    wire_types::{cpp_name} out{{}};\n')
                for field in schema_def.fields:
                    if field.kind in service_schema.ENUM_SPECS:
                        f.write(f'    out.{field.name} = static_cast<pyramid::data_model::{field.cpp_type}>(msg.{field.name});\n')
                    else:
                        f.write(f'    out.{field.name} = msg.{field.name};\n')
                f.write('    return out;\n')
                f.write('}\n\n')

                if schema_def.is_array_response:
                    alias_name = cpp_name + 'Array'
                    holder_name = cpp_name + 'ArrayHolder'
                    f.write(f'fbs::{holder_name}T to_fb(const wire_types::{alias_name}& msg) {{\n')
                    f.write(f'    fbs::{holder_name}T out{{}};\n')
                    f.write('    out.items.reserve(msg.size());\n')
                    f.write('    for (const auto& item : msg) {\n')
                    f.write(f'        out.items.emplace_back(std::make_unique<fbs::{cpp_name}T>(to_fb(item)));\n')
                    f.write('    }\n')
                    f.write('    return out;\n')
                    f.write('}\n\n')

                    f.write(f'wire_types::{alias_name} from_fb(const fbs::{holder_name}T& msg) {{\n')
                    f.write(f'    wire_types::{alias_name} out{{}};\n')
                    f.write('    out.reserve(msg.items.size());\n')
                    f.write('    for (const auto& item : msg.items) {\n')
                    f.write('        if (item) {\n')
                    f.write('            out.push_back(from_fb(*item));\n')
                    f.write('        }\n')
                    f.write('    }\n')
                    f.write('    return out;\n')
                    f.write('}\n\n')

            f.write('} // namespace\n\n')

            f.write('std::string wrapPayload(const std::string& payload) {\n')
            f.write('    flatbuffers::FlatBufferBuilder builder(\n')
            f.write('        static_cast<uint32_t>(payload.size() + 64u));\n')
            f.write('    auto payload_offset = builder.CreateString(payload);\n')
            f.write('    fbs::JsonPayloadBuilder root(builder);\n')
            f.write('    root.add_payload(payload_offset);\n')
            f.write('    return finish_buffer(builder, root.Finish());\n')
            f.write('}\n\n')

            f.write('std::string unwrapPayload(const void* data, size_t size) {\n')
            f.write('    if (data == nullptr || size == 0) {\n')
            f.write('        return {};\n')
            f.write('    }\n')
            f.write('    auto* root = flatbuffers::GetRoot<fbs::JsonPayload>(data);\n')
            f.write('    if (!root || !root->payload()) {\n')
            f.write('        throw std::runtime_error("invalid JsonPayload flatbuffer");\n')
            f.write('    }\n')
            f.write('    return root->payload()->str();\n')
            f.write('}\n\n')

            for schema_def in service_schema.ALL_SCHEMAS:
                cpp_name = schema_def.cpp_name
                f.write(f'std::string toBinary(const wire_types::{cpp_name}& msg) {{\n')
                f.write('    flatbuffers::FlatBufferBuilder builder(256);\n')
                f.write(f'    auto object = to_fb(msg);\n')
                f.write(f'    return finish_buffer(builder, fbs::{cpp_name}::Pack(builder, &object));\n')
                f.write('}\n\n')

                f.write(f'wire_types::{cpp_name} fromBinary{cpp_name}(const void* data, size_t size) {{\n')
                f.write('    if (data == nullptr || size == 0) {\n')
                f.write(f'        throw std::runtime_error("empty {cpp_name} flatbuffer");\n')
                f.write('    }\n')
                f.write(f'    auto* root = flatbuffers::GetRoot<fbs::{cpp_name}>(data);\n')
                f.write('    if (!root) {\n')
                f.write(f'        throw std::runtime_error("invalid {cpp_name} flatbuffer");\n')
                f.write('    }\n')
                f.write(f'    fbs::{cpp_name}T object{{}};\n')
                f.write('    root->UnPackTo(&object);\n')
                f.write('    return from_fb(object);\n')
                f.write('}\n\n')

                if schema_def.is_array_response:
                    alias_name = cpp_name + 'Array'
                    holder_name = cpp_name + 'ArrayHolder'
                    f.write(f'std::string toBinary(const wire_types::{alias_name}& msg) {{\n')
                    f.write('    flatbuffers::FlatBufferBuilder builder(256);\n')
                    f.write('    auto object = to_fb(msg);\n')
                    f.write(f'    return finish_buffer(builder, fbs::{holder_name}::Pack(builder, &object));\n')
                    f.write('}\n\n')

                    f.write(f'wire_types::{alias_name} fromBinary{alias_name}(const void* data, size_t size) {{\n')
                    f.write('    if (data == nullptr || size == 0) {\n')
                    f.write(f'        throw std::runtime_error("empty {alias_name} flatbuffer");\n')
                    f.write('    }\n')
                    f.write(f'    auto* root = flatbuffers::GetRoot<fbs::{holder_name}>(data);\n')
                    f.write('    if (!root) {\n')
                    f.write(f'        throw std::runtime_error("invalid {alias_name} flatbuffer");\n')
                    f.write('    }\n')
                    f.write(f'    fbs::{holder_name}T object{{}};\n')
                    f.write('    root->UnPackTo(&object);\n')
                    f.write('    return from_fb(object);\n')
                    f.write('}\n\n')

            f.write(f'}} // namespace {_SERVICE_CODEC_NS}\n')

    # -- Ada thin binding -----------------------------------------------------

    def _write_ada_spec(self, path: Path, pf: ProtoFile):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.Flatbuffers_Codec'

        with open(path, 'w', encoding='utf-8') as f:
            f.write('--  Auto-generated FlatBuffers codec spec — do not edit\n')
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

    def _write_service_ada_spec(self, path: Path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('--  Auto-generated tactical service FlatBuffers codec\n')
            f.write('--  Backend: flatbuffers\n')
            f.write('--  Generated from pim/json_schema.py\n\n')
            f.write(f'package {_SERVICE_ADA_CODEC_PKG} is\n')
            f.write('   Content_Type : constant String := "application/flatbuffers";\n\n')
            f.write('   function Encode_Payload (Payload : String) return String;\n')
            f.write('   function Decode_Payload (Payload : String) return String;\n')
            f.write(f'end {_SERVICE_ADA_CODEC_PKG};\n')

    def _write_service_ada_body(self, path: Path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('--  Auto-generated tactical service FlatBuffers codec\n\n')
            f.write(f'package body {_SERVICE_ADA_CODEC_PKG} is\n')
            f.write('   Magic : constant String := "PWFB";\n\n')
            f.write('   function Encode_Payload (Payload : String) return String is\n')
            f.write('   begin\n')
            f.write('      return Magic & Payload;\n')
            f.write('   end Encode_Payload;\n\n')
            f.write('   function Decode_Payload (Payload : String) return String is\n')
            f.write('   begin\n')
            f.write("      if Payload'Length < Magic'Length\n")
            f.write("        or else Payload (Payload'First .. Payload'First + Magic'Length - 1) /= Magic\n")
            f.write('      then\n')
            f.write('         raise Constraint_Error with "Invalid tactical flatbuffers payload";\n')
            f.write('      end if;\n\n')
            f.write("      return Payload (Payload'First + Magic'Length .. Payload'Last);\n")
            f.write('   end Decode_Payload;\n')
            f.write(f'end {_SERVICE_ADA_CODEC_PKG};\n')


codec_backends.register(FlatBuffersBackend())
