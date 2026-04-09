#!/usr/bin/env python3
"""
FlatBuffers Codec Backend

Generates:
  1. .fbs schema files from proto definitions (proto message -> FlatBuffers table)
  2. C++ wrapper headers for proto-native flatbuffer artefacts
  3. Tactical service-wire FlatBuffers codec artefacts from json_schema.py

The proto-native wrappers are still thin surfaces over `flatc` artefacts.
For service wire types we generate a self-contained binary codec today so the
service-binding runtime can dispatch cleanly without adding a new build-time
FlatBuffers dependency midstream.  The generated `.fbs` schema remains the
canonical target layout for a future `flatc`-driven implementation.
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

    def _write_service_cpp_header(self, path: Path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('// Auto-generated tactical service FlatBuffers codec\n')
            f.write(f'// Backend: flatbuffers | Namespace: {_SERVICE_CODEC_NS}\n')
            f.write('// Generated from pim/json_schema.py\n')
            f.write('#pragma once\n\n')
            f.write(f'#include "{_SERVICE_WIRE_HEADER}"\n\n')
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
            f.write('#include <cstdint>\n')
            f.write('#include <cstring>\n')
            f.write('#include <stdexcept>\n')
            f.write('#include <utility>\n\n')
            f.write(f'namespace {_SERVICE_CODEC_NS} {{\n\n')
            f.write('namespace {\n\n')
            f.write('constexpr std::uint32_t kMagic = 0x42465750u;  // "PWFB"\n\n')
            f.write('enum class MessageTag : std::uint32_t {\n')
            f.write('    PayloadEnvelope = 0,\n')
            tag_id = 1
            for schema_def in service_schema.ALL_SCHEMAS:
                f.write(f'    {schema_def.cpp_name} = {tag_id},\n')
                tag_id += 1
                if schema_def.is_array_response:
                    f.write(f'    {schema_def.cpp_name}Array = {tag_id},\n')
                    tag_id += 1
            f.write('};\n\n')

            f.write('struct Writer {\n')
            f.write('    std::string data;\n\n')
            f.write('    void raw(const void* ptr, size_t size) {\n')
            f.write('        data.append(static_cast<const char*>(ptr), size);\n')
            f.write('    }\n\n')
            f.write('    void u8(bool value) {\n')
            f.write('        const std::uint8_t raw_value = value ? 1u : 0u;\n')
            f.write('        raw(&raw_value, sizeof(raw_value));\n')
            f.write('    }\n\n')
            f.write('    void u32(std::uint32_t value) { raw(&value, sizeof(value)); }\n')
            f.write('    void i32(std::int32_t value) { raw(&value, sizeof(value)); }\n')
            f.write('    void f64(double value) { raw(&value, sizeof(value)); }\n\n')
            f.write('    void str(const std::string& value) {\n')
            f.write('        u32(static_cast<std::uint32_t>(value.size()));\n')
            f.write('        raw(value.data(), value.size());\n')
            f.write('    }\n')
            f.write('};\n\n')

            f.write('struct Reader {\n')
            f.write('    const char* data = nullptr;\n')
            f.write('    size_t size = 0;\n')
            f.write('    size_t offset = 0;\n\n')
            f.write('    bool raw(void* dst, size_t n) {\n')
            f.write('        if (offset + n > size) return false;\n')
            f.write('        std::memcpy(dst, data + offset, n);\n')
            f.write('        offset += n;\n')
            f.write('        return true;\n')
            f.write('    }\n\n')
            f.write('    bool u8(bool& value) {\n')
            f.write('        std::uint8_t raw_value = 0;\n')
            f.write('        if (!raw(&raw_value, sizeof(raw_value))) return false;\n')
            f.write('        value = raw_value != 0;\n')
            f.write('        return true;\n')
            f.write('    }\n\n')
            f.write('    bool u32(std::uint32_t& value) { return raw(&value, sizeof(value)); }\n')
            f.write('    bool i32(std::int32_t& value) { return raw(&value, sizeof(value)); }\n')
            f.write('    bool f64(double& value) { return raw(&value, sizeof(value)); }\n\n')
            f.write('    bool str(std::string& value) {\n')
            f.write('        std::uint32_t len = 0;\n')
            f.write('        if (!u32(len)) return false;\n')
            f.write('        if (offset + len > size) return false;\n')
            f.write('        value.assign(data + offset, len);\n')
            f.write('        offset += len;\n')
            f.write('        return true;\n')
            f.write('    }\n')
            f.write('};\n\n')

            f.write('void beginMessage(Writer& w, MessageTag tag) {\n')
            f.write('    w.u32(kMagic);\n')
            f.write('    w.u32(static_cast<std::uint32_t>(tag));\n')
            f.write('}\n\n')

            f.write('bool beginMessage(Reader& r, MessageTag expected) {\n')
            f.write('    std::uint32_t magic = 0;\n')
            f.write('    std::uint32_t tag = 0;\n')
            f.write('    return r.u32(magic) && r.u32(tag)\n')
            f.write('        && magic == kMagic\n')
            f.write('        && tag == static_cast<std::uint32_t>(expected);\n')
            f.write('}\n\n')

            for schema_def in service_schema.ALL_SCHEMAS:
                cpp_name = schema_def.cpp_name
                f.write(f'void encode{cpp_name}(Writer& w, const wire_types::{cpp_name}& msg) {{\n')
                for field in schema_def.fields:
                    f.write(f'    {_service_encode_stmt(field)}\n')
                f.write('}\n\n')

                f.write(f'bool decode{cpp_name}(Reader& r, wire_types::{cpp_name}& msg) {{\n')
                f.write('    std::int32_t tmp_i32 = 0;\n')
                for field in schema_def.fields:
                    stmt = _service_decode_stmt(field).splitlines()
                    for line in stmt:
                        f.write(f'    {line}\n')
                f.write('    return true;\n')
                f.write('}\n\n')

                if schema_def.is_array_response:
                    alias_name = cpp_name + 'Array'
                    f.write(f'void encode{alias_name}(Writer& w, const wire_types::{alias_name}& msg) {{\n')
                    f.write('    w.u32(static_cast<std::uint32_t>(msg.size()));\n')
                    f.write('    for (const auto& item : msg) {\n')
                    f.write(f'        encode{cpp_name}(w, item);\n')
                    f.write('    }\n')
                    f.write('}\n\n')

                    f.write(f'bool decode{alias_name}(Reader& r, wire_types::{alias_name}& msg) {{\n')
                    f.write('    std::uint32_t count = 0;\n')
                    f.write('    if (!r.u32(count)) return false;\n')
                    f.write('    msg.clear();\n')
                    f.write('    msg.reserve(count);\n')
                    f.write('    for (std::uint32_t i = 0; i < count; ++i) {\n')
                    f.write(f'        wire_types::{cpp_name} item{{}};\n')
                    f.write(f'        if (!decode{cpp_name}(r, item)) return false;\n')
                    f.write('        msg.push_back(std::move(item));\n')
                    f.write('    }\n')
                    f.write('    return true;\n')
                    f.write('}\n\n')

            f.write('template <typename T>\n')
            f.write('T finishRead(const void* data, size_t size, MessageTag expected,\n')
            f.write('             bool (*decode_fn)(Reader&, T&)) {\n')
            f.write('    Reader r{static_cast<const char*>(data), size, 0};\n')
            f.write('    if (!beginMessage(r, expected))\n')
            f.write('        throw std::runtime_error("invalid flatbuffers payload header");\n')
            f.write('    T msg{};\n')
            f.write('    if (!decode_fn(r, msg) || r.offset != r.size)\n')
            f.write('        throw std::runtime_error("invalid flatbuffers payload body");\n')
            f.write('    return msg;\n')
            f.write('}\n\n')
            f.write('} // namespace\n\n')

            f.write('std::string wrapPayload(const std::string& payload) {\n')
            f.write('    return std::string("PWFB", 4) + payload;\n')
            f.write('}\n\n')

            f.write('std::string unwrapPayload(const void* data, size_t size) {\n')
            f.write('    if (data == nullptr || size == 0) {\n')
            f.write('        return {};\n')
            f.write('    }\n')
            f.write('    const auto* bytes = static_cast<const char*>(data);\n')
            f.write('    if (size < 4 || std::memcmp(bytes, "PWFB", 4) != 0) {\n')
            f.write('        throw std::runtime_error("invalid flatbuffers payload envelope");\n')
            f.write('    }\n')
            f.write('    return std::string(bytes + 4, size - 4);\n')
            f.write('}\n\n')

            for schema_def in service_schema.ALL_SCHEMAS:
                cpp_name = schema_def.cpp_name
                f.write(f'std::string toBinary(const wire_types::{cpp_name}& msg) {{\n')
                f.write('    Writer w;\n')
                f.write(f'    beginMessage(w, MessageTag::{cpp_name});\n')
                f.write(f'    encode{cpp_name}(w, msg);\n')
                f.write('    return w.data;\n')
                f.write('}\n\n')

                f.write(f'wire_types::{cpp_name} fromBinary{cpp_name}(const void* data, size_t size) {{\n')
                f.write(f'    return finishRead<wire_types::{cpp_name}>(data, size,\n')
                f.write(f'        MessageTag::{cpp_name}, decode{cpp_name});\n')
                f.write('}\n\n')

                if schema_def.is_array_response:
                    alias_name = cpp_name + 'Array'
                    f.write(f'std::string toBinary(const wire_types::{alias_name}& msg) {{\n')
                    f.write('    Writer w;\n')
                    f.write(f'    beginMessage(w, MessageTag::{alias_name});\n')
                    f.write(f'    encode{alias_name}(w, msg);\n')
                    f.write('    return w.data;\n')
                    f.write('}\n\n')

                    f.write(f'wire_types::{alias_name} fromBinary{alias_name}(const void* data, size_t size) {{\n')
                    f.write(f'    return finishRead<wire_types::{alias_name}>(data, size,\n')
                    f.write(f'        MessageTag::{alias_name}, decode{alias_name});\n')
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
