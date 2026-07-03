#!/usr/bin/env python3
"""C++ data-model JSON codec generator (proto-driven, per file).

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

from pathlib import Path
from typing import Dict

from proto_parser import (
    ProtoTypeIndex,
    screaming_to_pascal,
    lc_first as _lc_first,
)
from proto_resolve import (
    _field_with_type,
    _is_proto_enum_type,
    _is_proto_message_type,
    _package_for_proto_type,
    _proto_type_fqn,
    _resolve_message,
)
from binding_contract import NamingPolicy
from .naming import (
    _CPP_SCALAR_MAP,
    _FORCED_ALIASES,
    _UNIT_FIELD_NAMES,
    _DEFAULT_NAMING_POLICY,
)


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

    def __init__(self, pf, index: 'ProtoTypeIndex',
                 naming_policy: NamingPolicy = None):
        self._pf = pf
        self._index = index
        self._naming = naming_policy or _DEFAULT_NAMING_POLICY
        self._ns = self._naming.cpp_namespace_for_package(pf.package)
        self._prefix = pf.package.replace('.', '_')
        self._types_header = self._naming.type_header_for_package(pf.package)
        self._hpp_name = self._naming.codec_header_for_package(pf.package)
        self._cpp_name = self._naming.codec_source_for_package(pf.package)
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
                        header = self._naming.codec_header_for_package(
                            indexed_pf.package)
                        f.write(f'#include "{header}"\n')
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
                        f.write(
                            f'#include "{self._naming.codec_header_for_package(pkg)}"\n')
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
            return self._naming.cpp_type_namespace_for_package(pkg) + '::' + short
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


