#!/usr/bin/env python3
"""
FlatBuffers Codec Backend

Generates:
  1. .fbs schema files from proto definitions (proto message -> FlatBuffers table)
  2. C++ PCL wrapper code that uses flatbuffers::GetRoot / CreateT for ser/de
  3. Ada thin binding wrappers (via C interop to flatbuffers C API)

FlatBuffers advantages over JSON:
  - Zero-copy reads (no parsing/unpacking step)
  - Schema-compiled — type-safe generated code
  - Header-only C++ runtime (~50KB)
  - Deterministic binary layout for reproducibility
"""

from pathlib import Path
from typing import List

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from proto_parser import (
    ProtoTypeIndex, ProtoFile, ProtoMessage, ProtoEnum, ProtoField,
    camel_to_snake, camel_to_lower_snake, _PROTO_SCALARS,
)
import codec_backends


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

# C++ type mapping for flatbuffers wrapper code
_CPP_SCALAR_MAP = {
    'double': 'double', 'float': 'float',
    'int32': 'int32_t', 'int64': 'int64_t',
    'uint32': 'uint32_t', 'uint64': 'uint64_t',
    'sint32': 'int32_t', 'sint64': 'int64_t',
    'bool': 'bool', 'string': 'std::string', 'bytes': 'std::string',
}


def _fbs_type(field: ProtoField, index: ProtoTypeIndex) -> str:
    """Map a proto field to its FlatBuffers schema type."""
    if field.type in _FBS_SCALAR_MAP:
        base = _FBS_SCALAR_MAP[field.type]
    else:
        base = field.short_type  # enum or table name

    if field.is_repeated:
        return f'[{base}]'
    return base


class FlatBuffersBackend(codec_backends.CodecBackend):

    @property
    def name(self) -> str:
        return 'flatbuffers'

    @property
    def content_type(self) -> str:
        return 'application/flatbuffers'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        for pf in index.files:
            if not pf.messages and not pf.enums:
                continue

            pkg_parts = [p for p in pf.package.split('.') if p]
            file_base = '_'.join(pkg_parts)

            # 1. Generate .fbs schema
            fbs_path = output_dir / (file_base + '.fbs')
            self._write_fbs_schema(fbs_path, pf, index)
            generated.append(fbs_path)

            # 2. Generate C++ PCL wrapper header
            hpp_path = output_dir / (file_base + '_flatbuffers_codec.hpp')
            self._write_cpp_wrapper(hpp_path, pf, index)
            generated.append(hpp_path)

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
            self._write_ada_spec(spec_path, pf, index)
            generated.append(spec_path)

        return generated

    # -- .fbs schema generation ------------------------------------------------

    def _write_fbs_schema(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        """Generate a .fbs schema file from proto definitions."""
        with open(path, 'w') as f:
            f.write(f'// Auto-generated FlatBuffers schema from {pf.path.name}\n')
            f.write(f'// Do not edit — regenerate from proto source\n\n')

            # Namespace
            if pf.package:
                ns = '.'.join(pf.package.split('.'))
                f.write(f'namespace {ns};\n\n')

            # Enums (FlatBuffers enums must have an underlying type)
            for enum in pf.enums:
                # Determine smallest integer type that fits
                max_val = max((v.number for v in enum.values), default=0)
                int_type = 'byte' if max_val < 128 else 'short' if max_val < 32768 else 'int'
                f.write(f'enum {enum.name} : {int_type} {{\n')
                for i, v in enumerate(enum.values):
                    suffix = enum.suffix_of(v.name)
                    fbs_name = suffix if suffix else v.name
                    comma = ',' if i < len(enum.values) - 1 else ''
                    f.write(f'  {fbs_name} = {v.number}{comma}\n')
                f.write(f'}}\n\n')

            # Oneof groups become FlatBuffers unions (deduplicated by name)
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
                    f.write(f'}}\n\n')

            # Messages become tables
            for msg in pf.messages:
                f.write(f'table {msg.name} {{\n')
                for fld in msg.fields:  # non-oneof fields
                    fbs_t = _fbs_type(fld, index)
                    f.write(f'  {fld.name}:{fbs_t};\n')
                for oo in msg.oneofs:
                    union_name = ''.join(w.capitalize() for w in oo.name.split('_')) + 'Union'
                    f.write(f'  {oo.name}:{union_name};\n')
                f.write(f'}}\n\n')

    # -- C++ PCL wrapper -------------------------------------------------------

    def _write_cpp_wrapper(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        """Generate C++ header with toBinary/fromBinary wrappers for PCL."""
        pkg_parts = [p for p in pf.package.split('.') if p]
        ns = '::'.join(pkg_parts) + '::flatbuffers_codec'
        fbs_base = '_'.join(pkg_parts)

        with open(path, 'w') as f:
            f.write(f'// Auto-generated FlatBuffers PCL codec — do not edit\n')
            f.write(f'// Backend: flatbuffers | Namespace: {ns}\n')
            f.write(f'#pragma once\n\n')
            f.write(f'// Include the flatc-generated header (run flatc on {fbs_base}.fbs)\n')
            f.write(f'#include "{fbs_base}_generated.h"\n\n')
            f.write(f'#include <flatbuffers/flatbuffers.h>\n')
            f.write(f'#include <string>\n')
            f.write(f'#include <vector>\n')
            f.write(f'#include <cstdint>\n\n')
            f.write(f'namespace {ns} {{\n\n')
            f.write(f'static constexpr const char* kContentType = "application/flatbuffers";\n\n')

            for msg in pf.messages:
                # toBinary
                f.write(f'/// Serialise {msg.name} to a FlatBuffer binary blob.\n')
                f.write(f'inline std::string toBinary(const {msg.name}T& obj) {{\n')
                f.write(f'    flatbuffers::FlatBufferBuilder builder(1024);\n')
                f.write(f'    auto offset = {msg.name}::Pack(builder, &obj);\n')
                f.write(f'    builder.Finish(offset);\n')
                f.write(f'    return std::string(reinterpret_cast<const char*>(\n')
                f.write(f'        builder.GetBufferPointer()), builder.GetSize());\n')
                f.write(f'}}\n\n')

                # fromBinary
                f.write(f'/// Deserialise {msg.name} from a FlatBuffer binary blob.\n')
                f.write(f'inline {msg.name}T fromBinary{msg.name}(const void* data, size_t size) {{\n')
                f.write(f'    {msg.name}T result;\n')
                f.write(f'    auto* fb = flatbuffers::GetRoot<{msg.name}>(data);\n')
                f.write(f'    if (fb) fb->UnPackTo(&result);\n')
                f.write(f'    return result;\n')
                f.write(f'}}\n\n')

                # fromBinary (string overload)
                f.write(f'inline {msg.name}T fromBinary{msg.name}(const std::string& s) {{\n')
                f.write(f'    return fromBinary{msg.name}(s.data(), s.size());\n')
                f.write(f'}}\n\n')

            f.write(f'}} // namespace {ns}\n')

    # -- Ada spec (thin binding) -----------------------------------------------

    def _write_ada_spec(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.Flatbuffers_Codec'

        with open(path, 'w') as f:
            f.write(f'--  Auto-generated FlatBuffers codec spec — do not edit\n')
            f.write(f'--  Backend: flatbuffers | Package: {pkg_name}\n')
            f.write(f'--\n')
            f.write(f'--  This package provides thin bindings to the C++ FlatBuffers codec.\n')
            f.write(f'--  Actual ser/de is performed via C interop (Import pragma).\n\n')
            f.write(f'with Interfaces.C; use Interfaces.C;\n')
            f.write(f'with System;\n\n')
            f.write(f'package {pkg_name} is\n\n')
            f.write(f'   Content_Type : constant String := "application/flatbuffers";\n\n')

            for msg in pf.messages:
                ada_name = camel_to_snake(msg.name)
                f.write(f'   function To_Binary (Msg : System.Address) return Interfaces.C.Strings.chars_ptr\n')
                f.write(f'     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{msg.name}_to_flatbuffer";\n\n')
                f.write(f'   function From_Binary_{ada_name} (Data : System.Address; Size : Interfaces.C.size_t)\n')
                f.write(f'     return System.Address\n')
                f.write(f'     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{msg.name}_from_flatbuffer";\n\n')

            f.write(f'end {pkg_name};\n')


# -- Register ------------------------------------------------------------------

codec_backends.register(FlatBuffersBackend())
