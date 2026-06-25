#!/usr/bin/env python3
"""
C-ABI data model and C++ marshalling code generator.

Emits frozen C structs plus to_c/from_c/_free marshalling between native
pyramid::domain_model types and those C structs.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple

from proto_parser import (
    ProtoField,
    ProtoMessage,
    ProtoTypeIndex,
    parse_proto_tree,
)
from cpp_codegen import (
    _CPP_SCALAR_MAP,
    _DATA_MODEL_PROTO_ROOT,
    _DATA_MODEL_TYPES_HEADER,
    _DATA_MODEL_TYPES_NS,
    _cpp_ns_for_proto_package,
    _cpp_ns_for_proto_type_package,
    find_scalar_wrappers,
)

_CABI_UMBRELLA = 'pyramid_datamodel_cabi.h'

_C_SCALAR_MAP: Dict[str, str] = {
    'double': 'double',
    'float': 'float',
    'int32': 'int32_t',
    'int64': 'int64_t',
    'uint32': 'uint32_t',
    'uint64': 'uint64_t',
    'sint32': 'int32_t',
    'sint64': 'int64_t',
    'fixed32': 'uint32_t',
    'fixed64': 'uint64_t',
    'sfixed32': 'int32_t',
    'sfixed64': 'int64_t',
    'bool': 'uint8_t',
    'string': 'pyramid_str_t',
    'bytes': 'pyramid_str_t',
}


def _c_scalar_for_cpp(cpp_scalar: str) -> str:
    if cpp_scalar == 'bool':
        return 'uint8_t'
    if cpp_scalar == 'std::string':
        return 'pyramid_str_t'
    return cpp_scalar


def _guard_from_stem(stem: str) -> str:
    return stem.upper().replace('.', '_') + '_H'


def _c_struct_name(message_name: str) -> str:
    return f'pyramid_{message_name}_c'


def _package_of_type(index: ProtoTypeIndex, name: str) -> str:
    short = name.split('.')[-1]
    for pf in index.files:
        for msg in pf.messages:
            if msg.name == short:
                return pf.package
        for enum in pf.enums:
            if enum.name == short:
                return pf.package
    return ''


def _includes_for_file(index: ProtoTypeIndex, pf, suffix: str) -> List[str]:
    result = []
    for imp in pf.imports:
        if imp.startswith('google/'):
            continue
        pkg = imp.replace('/', '.').removesuffix('.proto')
        imp_stem = Path(imp).stem
        for indexed_pf in index.files:
            if indexed_pf.package == pkg or indexed_pf.path.stem == imp_stem:
                result.append(
                    f'#include "{indexed_pf.package.replace(".", "_")}{suffix}"'
                )
                break
    return result


def _toposort(
    index: ProtoTypeIndex,
    aliases: Dict[str, str],
    messages: List[ProtoMessage],
) -> List[ProtoMessage]:
    names = {m.name for m in messages}
    by_name = {m.name: m for m in messages}
    deps: Dict[str, set] = {}
    for m in messages:
        d = set()
        for f in m.all_fields():
            short = f.type.split('.')[-1]
            if short in names and short != m.name and short not in aliases:
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


def _inline_base_fields(
    index: ProtoTypeIndex,
    aliases: Dict[str, str],
    msg: ProtoMessage,
) -> Iterator[Tuple[ProtoField, str, str]]:
    own_names = {f.name for f in msg.fields if f.name != 'base'}
    for field in msg.fields:
        if field.name == 'base' and not field.is_repeated:
            short = field.type.split('.')[-1]
            base_msg = index.resolve_message(field.type) or \
                index.resolve_message(short)
            if base_msg and base_msg.name not in aliases:
                for bf in base_msg.fields:
                    name = bf.name
                    if name in own_names:
                        name = short.lower() + '_' + name
                    yield bf, name, f'  /* from {base_msg.name} */'
                continue
        yield field, field.name, ''


@dataclass
class CabiMember:
    name: str
    kind: str
    c_type: str
    native_cpp_type: str
    proto_field: ProtoField
    comment: str = ''
    is_oneof: bool = False

    @property
    def has_flag(self) -> bool:
        return self.kind.startswith('opt_') or self.is_oneof

    @property
    def is_repeated(self) -> bool:
        return self.kind.startswith('repeated_')


def _native_cpp_field_type(
    field_type: str,
    repeated: bool,
    aliases: Dict[str, str],
    index: ProtoTypeIndex,
    current_pkg: str,
) -> str:
    short = field_type.split('.')[-1]
    if field_type in _CPP_SCALAR_MAP:
        base = _CPP_SCALAR_MAP[field_type]
    elif short in aliases:
        base = aliases[short]
    elif '.' in field_type and not field_type.startswith('google.'):
        pkg = '.'.join(field_type.split('.')[:-1])
        if current_pkg and pkg != current_pkg:
            base = _cpp_ns_for_proto_type_package(pkg) + '::' + short
        else:
            base = short
    elif index.is_enum_type(field_type) or index.is_message_type(field_type):
        pkg = _package_of_type(index, field_type)
        if current_pkg and pkg and pkg != current_pkg:
            base = _cpp_ns_for_proto_type_package(pkg) + '::' + field_type
        else:
            base = field_type
    else:
        base = short
    return f'std::vector<{base}>' if repeated else base


def c_type_for(
    field_type: str,
    aliases: Dict[str, str],
    index: ProtoTypeIndex,
    current_pkg: str = '',
) -> str:
    short = field_type.split('.')[-1]
    if field_type in _C_SCALAR_MAP:
        return _C_SCALAR_MAP[field_type]
    if short in aliases:
        return _c_scalar_for_cpp(aliases[short])
    if index.is_enum_type(field_type) or index.is_enum_type(short):
        return 'int32_t'
    if index.is_message_type(field_type) or index.is_message_type(short):
        return _c_struct_name(short)
    return 'int32_t'


def _is_string_alias(field_type: str, aliases: Dict[str, str]) -> bool:
    short = field_type.split('.')[-1]
    return short in aliases and aliases[short] == 'std::string'


def _classify_member(
    field: ProtoField,
    fname: str,
    is_oneof: bool,
    aliases: Dict[str, str],
    index: ProtoTypeIndex,
    current_pkg: str,
) -> CabiMember:
    ft = field.type
    short = ft.split('.')[-1]
    is_alias = short in aliases

    if field.is_repeated:
        if ft in ('string', 'bytes') or _is_string_alias(ft, aliases):
            kind = 'repeated_string'
            c_base = 'pyramid_str_t'
        elif ft == 'bool' or (is_alias and aliases[short] == 'bool'):
            kind = 'repeated_bool'
            c_base = 'uint8_t'
        elif ft in _CPP_SCALAR_MAP or is_alias:
            kind = 'repeated_scalar'
            c_base = c_type_for(ft, aliases, index, current_pkg)
        elif index.is_enum_type(ft) or index.is_enum_type(short):
            kind = 'repeated_enum'
            c_base = 'int32_t'
        else:
            kind = 'repeated_message'
            c_base = c_type_for(ft, aliases, index, current_pkg)
        native = _native_cpp_field_type(ft, True, aliases, index, current_pkg)
        return CabiMember(fname, kind, c_base, native, field, is_oneof=is_oneof)

    if is_alias:
        c_base = _c_scalar_for_cpp(aliases[short])
        native_base = aliases[short]
    else:
        c_base = c_type_for(ft, aliases, index, current_pkg)
        native_base = _native_cpp_field_type(
            ft, False, aliases, index, current_pkg)

    optional = field.is_optional or is_oneof
    # Oneof string fields are tl::optional<std::string> in native; plain
    # optional strings stay std::string and are handled as 'string' below.
    if is_oneof and native_base == 'std::string':
        return CabiMember(
            fname, 'opt_string', 'pyramid_str_t',
            'tl::optional<std::string>', field, is_oneof=is_oneof)
    if optional and native_base != 'std::string':
        if is_alias or ft in _CPP_SCALAR_MAP:
            if native_base == 'bool':
                kind = 'opt_bool'
            else:
                kind = 'opt_scalar'
        elif index.is_message_type(ft) or index.is_message_type(short):
            kind = 'opt_message'
        elif index.is_enum_type(ft) or index.is_enum_type(short):
            kind = 'opt_enum'
        else:
            kind = 'opt_scalar'
        native = f'tl::optional<{native_base}>'
        return CabiMember(fname, kind, c_base, native, field, is_oneof=is_oneof)

    if is_alias or ft in _CPP_SCALAR_MAP:
        if native_base == 'bool' or ft == 'bool':
            kind = 'bool'
        elif native_base == 'std::string' or ft in ('string', 'bytes'):
            kind = 'string'
        else:
            kind = 'scalar'
    elif index.is_enum_type(ft) or index.is_enum_type(short):
        kind = 'enum'
    elif index.is_message_type(ft) or index.is_message_type(short):
        kind = 'message'
    else:
        kind = 'scalar'
    return CabiMember(fname, kind, c_base, native_base, field, is_oneof=is_oneof)


def iter_cabi_members(
    index: ProtoTypeIndex,
    aliases: Dict[str, str],
    msg: ProtoMessage,
    current_pkg: str,
) -> Iterator[CabiMember]:
    for field, fname, comment in _inline_base_fields(index, aliases, msg):
        member = _classify_member(
            field, fname, False, aliases, index, current_pkg)
        member.comment = comment
        yield member
    for oo in msg.oneofs:
        for field in oo.fields:
            member = _classify_member(
                field, field.name, True, aliases, index, current_pkg)
            yield member


def _emit_c_struct_field(member: CabiMember) -> str:
    if member.is_repeated:
        return f'  pyramid_slice_t {member.name};'
    if member.has_flag:
        return f'  uint8_t has_{member.name};\n  {member.c_type} {member.name};'
    return f'  {member.c_type} {member.name};'


class CabiTypesGenerator:
    """Emits pure C struct headers for the data model."""

    def __init__(self, data_model_source):
        if isinstance(data_model_source, Path):
            proto_files = parse_proto_tree(data_model_source)
        else:
            proto_files = list(data_model_source)
        self._index = ProtoTypeIndex(proto_files)
        self._aliases = find_scalar_wrappers(self._index)

    def c_type_for(self, proto_field_type: str, current_pkg: str = '') -> str:
        return c_type_for(
            proto_field_type, self._aliases, self._index, current_pkg)

    def iter_message_members(
        self, msg: ProtoMessage, current_pkg: str
    ) -> Iterator[CabiMember]:
        yield from iter_cabi_members(
            self._index, self._aliases, msg, current_pkg)

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        self._write_umbrella(out / _CABI_UMBRELLA)
        for pf in self._index.files:
            stem = pf.package.replace('.', '_')
            self._write_cabi_header(out / f'{stem}_cabi.h', pf)
            print(f'  Generated {pf.package.replace(".", "::")} (cabi)')

    def _write_umbrella(self, path: Path) -> None:
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('#ifndef PYRAMID_DATAMODEL_CABI_H\n')
            f.write('#define PYRAMID_DATAMODEL_CABI_H\n')
            f.write('#include <stdint.h>\n')
            f.write('#ifdef __cplusplus\n')
            f.write('extern "C" {\n')
            f.write('#endif\n')
            f.write('#define PYRAMID_DATAMODEL_ABI_VERSION 1u\n')
            f.write('typedef struct { const char* ptr; uint32_t len; } pyramid_str_t;\n')
            f.write('typedef struct { const void* ptr; uint32_t len; } pyramid_slice_t;\n')
            f.write('#ifdef __cplusplus\n')
            f.write('}\n')
            f.write('#endif\n')
            f.write('#endif\n')

    def _write_cabi_header(self, path: Path, pf) -> None:
        stem = pf.package.replace('.', '_')
        guard = _guard_from_stem(stem + '_cabi')
        current_pkg = pf.package
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]
        includes = _includes_for_file(self._index, pf, '_cabi.h')

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'#ifndef {guard}\n')
            f.write(f'#define {guard}\n\n')
            f.write('#include <stdint.h>\n')
            f.write(f'#include "{_CABI_UMBRELLA}"\n')
            for inc in includes:
                f.write(inc + '\n')
            f.write('\n#ifdef __cplusplus\n')
            f.write('extern "C" {\n')
            f.write('#endif\n\n')

            for msg in _toposort(self._index, self._aliases, non_alias):
                f.write(f'typedef struct {_c_struct_name(msg.name)} {{\n')
                for member in iter_cabi_members(
                        self._index, self._aliases, msg, current_pkg):
                    line = _emit_c_struct_field(member)
                    if member.comment:
                        line += member.comment
                    f.write(line + '\n')
                f.write(f'}} {_c_struct_name(msg.name)};\n\n')

            for msg in non_alias:
                f.write(
                    f'void {_c_struct_name(msg.name)}_free'
                    f'({_c_struct_name(msg.name)}* value);\n')

            f.write('\n#ifdef __cplusplus\n')
            f.write('}\n')
            f.write('#endif\n\n')
            f.write(f'#endif /* {guard} */\n')


def _strip_template_wrapper(cpp_type: str, prefix: str) -> str:
    if cpp_type.startswith(prefix):
        return cpp_type[len(prefix):-1]
    return cpp_type


_PRIMITIVE_CPP_TYPES = frozenset({
    'double', 'float', 'bool', 'int32_t', 'int64_t', 'uint32_t', 'uint64_t',
    'std::string',
})


def _qualify_native_type(cpp_type: str) -> str:
    inner = _strip_template_wrapper(cpp_type, 'tl::optional<')
    inner = _strip_template_wrapper(inner, 'std::vector<')
    if '::' in inner or inner in _PRIMITIVE_CPP_TYPES:
        return cpp_type
    qualified = f'{_DATA_MODEL_TYPES_NS}::{inner}'
    if cpp_type == inner:
        return qualified
    return cpp_type.replace(inner, qualified)


class CabiMarshalGenerator:
    """Emits C++ marshalling between native types and C-ABI structs."""

    def __init__(self, data_model_source):
        if isinstance(data_model_source, Path):
            proto_files = parse_proto_tree(data_model_source)
        else:
            proto_files = list(data_model_source)
        self._index = ProtoTypeIndex(proto_files)
        self._aliases = find_scalar_wrappers(self._index)

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        for pf in self._index.files:
            stem = pf.package.replace('.', '_')
            self._write_marshal_hpp(out / f'{stem}_cabi_marshal.hpp', pf)
            self._write_marshal_cpp(out / f'{stem}_cabi_marshal.cpp', pf)
            print(f'  Generated {pf.package.replace(".", "::")} (cabi marshal)')

    def _native_type(self, message_name: str) -> str:
        return f'{_DATA_MODEL_TYPES_NS}::{message_name}'

    def _marshal_includes_for_file(self, pf) -> Tuple[List[str], List[str]]:
        cabi_includes = _includes_for_file(self._index, pf, '_cabi.h')
        marshal_includes = _includes_for_file(
            self._index, pf, '_cabi_marshal.hpp')
        return cabi_includes, marshal_includes

    def _write_marshal_hpp(self, path: Path, pf) -> None:
        stem = pf.package.replace('.', '_')
        current_pkg = pf.package
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]
        _, marshal_includes = self._marshal_includes_for_file(pf)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('#pragma once\n\n')
            f.write(f'#include "{_DATA_MODEL_TYPES_HEADER}"\n')
            f.write(f'#include "{stem}_cabi.h"\n')
            for inc in marshal_includes:
                f.write(inc + '\n')
            f.write('\nnamespace pyramid::cabi {\n\n')
            for msg in non_alias:
                c_name = _c_struct_name(msg.name)
                native = self._native_type(msg.name)
                f.write(f'void to_c(const {native}& in, {c_name}* out);\n')
                f.write(
                    f'void from_c(const {c_name}* in, {native}& out);\n\n')
            f.write('} // namespace pyramid::cabi\n')

    def _write_marshal_cpp(self, path: Path, pf) -> None:
        stem = pf.package.replace('.', '_')
        current_pkg = pf.package
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated C-ABI marshalling\n')
            f.write('// Ownership: to_c malloc-deep-copies all variable-length data into\n')
            f.write('// the C struct; from_c deep-copies into native values; _free releases\n')
            f.write('// everything allocated for the C struct. One full copy per direction.\n\n')
            f.write(f'#include "{stem}_cabi_marshal.hpp"\n')
            f.write('#include <cstdlib>\n')
            f.write('#include <cstring>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write('namespace {\n\n')
            f.write('void dup_str(pyramid_str_t& out, const std::string& in) {\n')
            f.write('  if (in.empty()) {\n')
            f.write('    out.ptr = nullptr;\n')
            f.write('    out.len = 0;\n')
            f.write('    return;\n')
            f.write('  }\n')
            f.write('  out.len = static_cast<uint32_t>(in.size());\n')
            f.write('  out.ptr = static_cast<const char*>(std::malloc(out.len));\n')
            f.write('  std::memcpy(const_cast<char*>(out.ptr), in.data(), out.len);\n')
            f.write('}\n\n')
            f.write('void free_str(pyramid_str_t& s) {\n')
            f.write('  if (s.ptr) {\n')
            f.write('    std::free(const_cast<char*>(s.ptr));\n')
            f.write('    s.ptr = nullptr;\n')
            f.write('    s.len = 0;\n')
            f.write('  }\n')
            f.write('}\n\n')
            f.write('} // namespace\n\n')
            f.write('namespace pyramid::cabi {\n\n')

            for msg in non_alias:
                self._write_to_c(f, msg, current_pkg)
                self._write_from_c(f, msg, current_pkg)
                self._write_free(f, msg, current_pkg)

            f.write('} // namespace pyramid::cabi\n\n')
            f.write('extern "C" {\n\n')
            for msg in non_alias:
                c_name = _c_struct_name(msg.name)
                f.write(
                    f'void {c_name}_free({c_name}* value) {{\n')
                f.write(
                    f'  pyramid::cabi::_free_{msg.name}(value);\n')
                f.write('}\n\n')
            f.write('} // extern "C"\n')

    def _write_to_c_member(
        self, f, member: CabiMember, msg_name: str, indent: str
    ) -> None:
        n = member.name
        if member.kind == 'scalar':
            f.write(f'{indent}out->{n} = in.{n};\n')
        elif member.kind == 'bool':
            f.write(f'{indent}out->{n} = in.{n} ? 1u : 0u;\n')
        elif member.kind == 'string':
            f.write(f'{indent}dup_str(out->{n}, in.{n});\n')
        elif member.kind == 'enum':
            f.write(
                f'{indent}out->{n} = static_cast<int32_t>(in.{n});\n')
        elif member.kind == 'message':
            f.write(
                f'{indent}to_c(in.{n}, &out->{n});\n')
        elif member.kind == 'repeated_scalar':
            f.write(f'{indent}{{\n')
            f.write(f'{indent}  const auto count = in.{n}.size();\n')
            f.write(f'{indent}  if (count > 0) {{\n')
            f.write(
                f'{indent}    auto* arr = static_cast<{member.c_type}*>'
                f'(std::malloc(count * sizeof({member.c_type})));\n')
            f.write(f'{indent}    for (size_t i = 0; i < count; ++i) {{\n')
            f.write(f'{indent}      arr[i] = in.{n}[i];\n')
            f.write(f'{indent}    }}\n')
            f.write(f'{indent}    out->{n}.ptr = arr;\n')
            f.write(
                f'{indent}    out->{n}.len = static_cast<uint32_t>(count);\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_bool':
            f.write(f'{indent}{{\n')
            f.write(f'{indent}  const auto count = in.{n}.size();\n')
            f.write(f'{indent}  if (count > 0) {{\n')
            f.write(
                f'{indent}    auto* arr = static_cast<uint8_t*>'
                f'(std::malloc(count * sizeof(uint8_t)));\n')
            f.write(f'{indent}    for (size_t i = 0; i < count; ++i) {{\n')
            f.write(
                f'{indent}      arr[i] = in.{n}[i] ? 1u : 0u;\n')
            f.write(f'{indent}    }}\n')
            f.write(f'{indent}    out->{n}.ptr = arr;\n')
            f.write(
                f'{indent}    out->{n}.len = static_cast<uint32_t>(count);\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_string':
            f.write(f'{indent}{{\n')
            f.write(f'{indent}  const auto count = in.{n}.size();\n')
            f.write(f'{indent}  if (count > 0) {{\n')
            f.write(
                f'{indent}    auto* arr = static_cast<pyramid_str_t*>'
                f'(std::malloc(count * sizeof(pyramid_str_t)));\n')
            f.write(f'{indent}    for (size_t i = 0; i < count; ++i) {{\n')
            f.write(f'{indent}      dup_str(arr[i], in.{n}[i]);\n')
            f.write(f'{indent}    }}\n')
            f.write(f'{indent}    out->{n}.ptr = arr;\n')
            f.write(
                f'{indent}    out->{n}.len = static_cast<uint32_t>(count);\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_enum':
            f.write(f'{indent}{{\n')
            f.write(f'{indent}  const auto count = in.{n}.size();\n')
            f.write(f'{indent}  if (count > 0) {{\n')
            f.write(
                f'{indent}    auto* arr = static_cast<int32_t*>'
                f'(std::malloc(count * sizeof(int32_t)));\n')
            f.write(f'{indent}    for (size_t i = 0; i < count; ++i) {{\n')
            f.write(
                f'{indent}      arr[i] = static_cast<int32_t>(in.{n}[i]);\n')
            f.write(f'{indent}    }}\n')
            f.write(f'{indent}    out->{n}.ptr = arr;\n')
            f.write(
                f'{indent}    out->{n}.len = static_cast<uint32_t>(count);\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_message':
            elem_c = member.c_type
            f.write(f'{indent}{{\n')
            f.write(f'{indent}  const auto count = in.{n}.size();\n')
            f.write(f'{indent}  if (count > 0) {{\n')
            f.write(
                f'{indent}    auto* arr = static_cast<{elem_c}*>'
                f'(std::malloc(count * sizeof({elem_c})));\n')
            f.write(f'{indent}    for (size_t i = 0; i < count; ++i) {{\n')
            f.write(f'{indent}      to_c(in.{n}[i], &arr[i]);\n')
            f.write(f'{indent}    }}\n')
            f.write(f'{indent}    out->{n}.ptr = arr;\n')
            f.write(
                f'{indent}    out->{n}.len = static_cast<uint32_t>(count);\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind in ('opt_scalar', 'opt_enum', 'opt_bool'):
            f.write(
                f'{indent}out->has_{n} = in.{n}.has_value() ? 1u : 0u;\n')
            f.write(f'{indent}if (in.{n}) {{\n')
            if member.kind == 'opt_bool':
                f.write(
                    f'{indent}  out->{n} = (*in.{n}) ? 1u : 0u;\n')
            elif member.kind == 'opt_enum':
                f.write(
                    f'{indent}  out->{n} = static_cast<int32_t>(*in.{n});\n')
            else:
                f.write(f'{indent}  out->{n} = *in.{n};\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'opt_message':
            f.write(
                f'{indent}out->has_{n} = in.{n}.has_value() ? 1u : 0u;\n')
            f.write(f'{indent}if (in.{n}) {{\n')
            f.write(f'{indent}  to_c(*in.{n}, &out->{n});\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'opt_string':
            f.write(
                f'{indent}out->has_{n} = in.{n}.has_value() ? 1u : 0u;\n')
            f.write(f'{indent}if (in.{n}) {{\n')
            f.write(f'{indent}  dup_str(out->{n}, *in.{n});\n')
            f.write(f'{indent}}}\n')

    def _write_from_c_member(
        self, f, member: CabiMember, indent: str
    ) -> None:
        n = member.name
        if member.kind == 'scalar':
            f.write(f'{indent}out.{n} = in->{n};\n')
        elif member.kind == 'bool':
            f.write(f'{indent}out.{n} = in->{n} != 0;\n')
        elif member.kind == 'string':
            f.write(f'{indent}if (in->{n}.ptr && in->{n}.len > 0) {{\n')
            f.write(
                f'{indent}  out.{n}.assign(in->{n}.ptr, in->{n}.len);\n')
            f.write(f'{indent}}} else {{\n')
            f.write(f'{indent}  out.{n}.clear();\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'enum':
            enum_type = _qualify_native_type(member.native_cpp_type)
            f.write(
                f'{indent}out.{n} = static_cast<{enum_type}>(in->{n});\n')
        elif member.kind == 'message':
            f.write(f'{indent}from_c(&in->{n}, out.{n});\n')
        elif member.kind == 'repeated_scalar':
            f.write(f'{indent}out.{n}.clear();\n')
            f.write(f'{indent}if (in->{n}.ptr && in->{n}.len > 0) {{\n')
            f.write(
                f'{indent}  const auto* arr = '
                f'static_cast<const {member.c_type}*>(in->{n}.ptr);\n')
            f.write(f'{indent}  out.{n}.reserve(in->{n}.len);\n')
            f.write(f'{indent}  for (uint32_t i = 0; i < in->{n}.len; ++i) {{\n')
            f.write(f'{indent}    out.{n}.push_back(arr[i]);\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_bool':
            f.write(f'{indent}out.{n}.clear();\n')
            f.write(f'{indent}if (in->{n}.ptr && in->{n}.len > 0) {{\n')
            f.write(
                f'{indent}  const auto* arr = '
                f'static_cast<const uint8_t*>(in->{n}.ptr);\n')
            f.write(f'{indent}  out.{n}.reserve(in->{n}.len);\n')
            f.write(f'{indent}  for (uint32_t i = 0; i < in->{n}.len; ++i) {{\n')
            f.write(f'{indent}    out.{n}.push_back(arr[i] != 0);\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_string':
            f.write(f'{indent}out.{n}.clear();\n')
            f.write(f'{indent}if (in->{n}.ptr && in->{n}.len > 0) {{\n')
            f.write(
                f'{indent}  const auto* arr = '
                f'static_cast<const pyramid_str_t*>(in->{n}.ptr);\n')
            f.write(f'{indent}  out.{n}.reserve(in->{n}.len);\n')
            f.write(f'{indent}  for (uint32_t i = 0; i < in->{n}.len; ++i) {{\n')
            f.write(f'{indent}    if (arr[i].ptr && arr[i].len > 0) {{\n')
            f.write(
                f'{indent}      out.{n}.emplace_back(arr[i].ptr, arr[i].len);\n')
            f.write(f'{indent}    }} else {{\n')
            f.write(f'{indent}      out.{n}.emplace_back();\n')
            f.write(f'{indent}    }}\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_enum':
            enum_elem = _qualify_native_type(
                _strip_template_wrapper(member.native_cpp_type, 'std::vector<'))
            f.write(f'{indent}out.{n}.clear();\n')
            f.write(f'{indent}if (in->{n}.ptr && in->{n}.len > 0) {{\n')
            f.write(
                f'{indent}  const auto* arr = '
                f'static_cast<const int32_t*>(in->{n}.ptr);\n')
            f.write(f'{indent}  out.{n}.reserve(in->{n}.len);\n')
            f.write(f'{indent}  for (uint32_t i = 0; i < in->{n}.len; ++i) {{\n')
            f.write(
                f'{indent}    out.{n}.push_back(static_cast<{enum_elem}>(arr[i]));\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'repeated_message':
            elem_native = _qualify_native_type(
                _strip_template_wrapper(member.native_cpp_type, 'std::vector<'))
            elem_c = member.c_type
            f.write(f'{indent}out.{n}.clear();\n')
            f.write(f'{indent}if (in->{n}.ptr && in->{n}.len > 0) {{\n')
            f.write(
                f'{indent}  const auto* arr = '
                f'static_cast<const {elem_c}*>(in->{n}.ptr);\n')
            f.write(f'{indent}  out.{n}.reserve(in->{n}.len);\n')
            f.write(f'{indent}  for (uint32_t i = 0; i < in->{n}.len; ++i) {{\n')
            f.write(f'{indent}    {elem_native} elem{{}};\n')
            f.write(f'{indent}    from_c(&arr[i], elem);\n')
            f.write(f'{indent}    out.{n}.push_back(std::move(elem));\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}}\n')
        elif member.kind in ('opt_scalar', 'opt_enum', 'opt_bool'):
            f.write(f'{indent}if (in->has_{n}) {{\n')
            if member.kind == 'opt_bool':
                f.write(
                    f'{indent}  out.{n} = in->{n} != 0;\n')
            elif member.kind == 'opt_enum':
                enum_type = _qualify_native_type(
                    _strip_template_wrapper(
                        member.native_cpp_type, 'tl::optional<'))
                f.write(
                    f'{indent}  out.{n} = static_cast<{enum_type}>(in->{n});\n')
            else:
                f.write(f'{indent}  out.{n} = in->{n};\n')
            f.write(f'{indent}}} else {{\n')
            f.write(f'{indent}  out.{n} = tl::nullopt;\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'opt_message':
            native = _qualify_native_type(
                _strip_template_wrapper(member.native_cpp_type, 'tl::optional<'))
            f.write(f'{indent}if (in->has_{n}) {{\n')
            f.write(f'{indent}  out.{n}.emplace();\n')
            f.write(f'{indent}  from_c(&in->{n}, *out.{n});\n')
            f.write(f'{indent}}} else {{\n')
            f.write(f'{indent}  out.{n} = tl::nullopt;\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'opt_string':
            f.write(f'{indent}if (in->has_{n}) {{\n')
            f.write(f'{indent}  if (in->{n}.ptr && in->{n}.len > 0) {{\n')
            f.write(
                f'{indent}    out.{n} = std::string(in->{n}.ptr, in->{n}.len);\n')
            f.write(f'{indent}  }} else {{\n')
            f.write(f'{indent}    out.{n} = std::string();\n')
            f.write(f'{indent}  }}\n')
            f.write(f'{indent}}} else {{\n')
            f.write(f'{indent}  out.{n} = tl::nullopt;\n')
            f.write(f'{indent}}}\n')

    def _write_free_member(
        self, f, member: CabiMember, ptr: str, indent: str
    ) -> None:
        n = member.name
        if member.kind == 'string':
            f.write(f'{indent}free_str({ptr}->{n});\n')
        elif member.kind == 'opt_string':
            f.write(f'{indent}free_str({ptr}->{n});\n')
            f.write(f'{indent}{ptr}->has_{n} = 0;\n')
        elif member.kind == 'message':
            short = member.proto_field.type.split('.')[-1]
            f.write(
                f'{indent}{_c_struct_name(short)}_free(&{ptr}->{n});\n')
        elif member.is_repeated:
            f.write(f'{indent}if ({ptr}->{n}.ptr) {{\n')
            if member.kind == 'repeated_string':
                f.write(
                    f'{indent}  auto* arr = static_cast<pyramid_str_t*>'
                    f'(const_cast<void*>({ptr}->{n}.ptr));\n')
                f.write(
                    f'{indent}  for (uint32_t i = 0; i < {ptr}->{n}.len; ++i) {{\n')
                f.write(f'{indent}    free_str(arr[i]);\n')
                f.write(f'{indent}  }}\n')
            elif member.kind == 'repeated_message':
                elem_c = member.c_type
                short = member.proto_field.type.split('.')[-1]
                f.write(
                    f'{indent}  auto* arr = static_cast<{elem_c}*>'
                    f'(const_cast<void*>({ptr}->{n}.ptr));\n')
                f.write(
                    f'{indent}  for (uint32_t i = 0; i < {ptr}->{n}.len; ++i) {{\n')
                f.write(
                    f'{indent}    {_c_struct_name(short)}_free(&arr[i]);\n')
                f.write(f'{indent}  }}\n')
            f.write(
                f'{indent}  std::free(const_cast<void*>({ptr}->{n}.ptr));\n')
            f.write(f'{indent}  {ptr}->{n}.ptr = nullptr;\n')
            f.write(f'{indent}  {ptr}->{n}.len = 0;\n')
            f.write(f'{indent}}}\n')
        elif member.kind == 'opt_message':
            f.write(f'{indent}if ({ptr}->has_{n}) {{\n')
            short = member.proto_field.type.split('.')[-1]
            f.write(
                f'{indent}  {_c_struct_name(short)}_free(&{ptr}->{n});\n')
            f.write(f'{indent}  {ptr}->has_{n} = 0;\n')
            f.write(f'{indent}}}\n')

    def _write_to_c(self, f, msg: ProtoMessage, current_pkg: str) -> None:
        c_name = _c_struct_name(msg.name)
        native = self._native_type(msg.name)
        f.write(f'void to_c(const {native}& in, {c_name}* out) {{\n')
        f.write('  std::memset(out, 0, sizeof(*out));\n')
        for member in iter_cabi_members(
                self._index, self._aliases, msg, current_pkg):
            self._write_to_c_member(f, member, msg.name, '  ')
        f.write('}\n\n')

    def _write_from_c(self, f, msg: ProtoMessage, current_pkg: str) -> None:
        c_name = _c_struct_name(msg.name)
        native = self._native_type(msg.name)
        f.write(f'void from_c(const {c_name}* in, {native}& out) {{\n')
        for member in iter_cabi_members(
                self._index, self._aliases, msg, current_pkg):
            self._write_from_c_member(f, member, '  ')
        f.write('}\n\n')

    def _write_free(self, f, msg: ProtoMessage, current_pkg: str) -> None:
        c_name = _c_struct_name(msg.name)
        f.write(f'void _free_{msg.name}({c_name}* value) {{\n')
        f.write('  if (!value) {\n')
        f.write('    return;\n')
        f.write('  }\n')
        for member in iter_cabi_members(
                self._index, self._aliases, msg, current_pkg):
            self._write_free_member(f, member, 'value', '  ')
        f.write('}\n\n')
