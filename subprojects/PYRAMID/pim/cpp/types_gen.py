#!/usr/bin/env python3
"""C++ plain-struct types header generator.

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

from pathlib import Path
from typing import Dict, List

from proto_parser import (
    parse_proto_tree,
    ProtoTypeIndex,
    ProtoMessage,
    ProtoEnum,
    screaming_to_pascal,
)
from proto_resolve import (
    _DATA_MODEL_PROTO_ROOT,
    _field_with_type,
    _package_for_proto_type,
    _proto_type_fqn,
    _resolve_enum,
    _resolve_message,
)
from binding_contract import NamingPolicy
from .naming import (
    _CPP_SCALAR_MAP,
    _CPP_DEFAULTS,
    _LITERAL_CPP_TYPES,
    _FORCED_ALIASES,
    _UNIT_FIELD_NAMES,
    _STRUCT_CONSTANTS,
    _DATA_MODEL_TYPES_NS,
    _DEFAULT_NAMING_POLICY,
)


def find_scalar_wrappers(index: ProtoTypeIndex) -> Dict[str, str]:
    """Return {message_name: cpp_scalar_type} for transparent wrapper messages."""
    aliases: Dict[str, str] = dict(_FORCED_ALIASES)
    for msg in index.all_messages():
        fields = msg.all_fields()
        if len(fields) == 1 and not fields[0].is_repeated:
            ft = fields[0].type
            fn = fields[0].name
            if ft in _CPP_SCALAR_MAP and fn in _UNIT_FIELD_NAMES:
                aliases[msg.name] = _CPP_SCALAR_MAP[ft]
    return aliases


def _common_cpp_ns(index: ProtoTypeIndex) -> str:
    """Derive C++ namespace from common package prefix of data model protos.

    e.g. pyramid.data_model.base, pyramid.data_model.common -> pyramid::domain_model
    """
    pkgs = [pf.package for pf in index.files if pf.package]
    if not pkgs:
        return 'data_model'
    if all(pkg == _DATA_MODEL_PROTO_ROOT or
           pkg.startswith(_DATA_MODEL_PROTO_ROOT + '.')
           for pkg in pkgs):
        return _DATA_MODEL_TYPES_NS
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

    def __init__(self, data_model_source, naming_policy: NamingPolicy = None):
        if isinstance(data_model_source, Path):
            proto_files = parse_proto_tree(data_model_source)
        else:
            proto_files = list(data_model_source)
        self._naming = naming_policy or _DEFAULT_NAMING_POLICY
        self._index = ProtoTypeIndex(proto_files)
        self._data_model_dir = data_model_source if isinstance(data_model_source, Path) else None
        self._ns = _common_cpp_ns(self._index)
        self._prefix = '_'.join(self._ns.split('::'))
        self._aliases = find_scalar_wrappers(self._index)

    # -- public ----------------------------------------------------------------

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        for pf in self._index.files:
            hpp = out / self._naming.type_header_for_package(pf.package)
            self._write_hpp_for_file(hpp, pf)
            print(f'  Generated {pf.package.replace(".", "::")} (types)')
        umbrella_name = self._naming.umbrella_types_header(self._index.files)
        if umbrella_name:
            umbrella = out / umbrella_name
            self._write_umbrella_hpp(umbrella)
            print(f'  Generated {self._ns} (umbrella)')

    def write_file(self, pf, output_dir: str) -> None:
        """Emit a single per-file types header (no umbrella).

        Used for service-local wrapper messages, which are homed in their own
        component namespace and must NOT be re-exported into domain_model. The
        instance must be constructed with an index that also contains the data
        model files so wrapper field types resolve.
        """
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        self._write_hpp_for_file(
            out / self._naming.type_header_for_package(pf.package), pf)
        print(f'  Generated {pf.package.replace(".", "::")} (types)')

    # -- internal --------------------------------------------------------------

    @staticmethod
    def _write_documentation(f, lines: List[str], indent: str = '') -> None:
        """Emit source documentation as a Doxygen ``///`` block."""
        if not lines:
            return
        for index, line in enumerate(lines):
            prefix = '/// \\brief ' if index == 0 else '/// '
            f.write(f'{indent}{prefix}{line}\n')

    def _package_of_type(self, name: str, current_pkg: str = '') -> str:
        """Return the proto package that defines a type."""
        return _package_for_proto_type(self._index, name, current_pkg)

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
                base = self._naming.cpp_type_namespace_for_package(pkg) + '::' + short
            else:
                base = short
        else:
            # Short name -- look up its package for cross-package qualification
            fqn = _proto_type_fqn(self._index, field_type, current_pkg)
            if fqn and (self._index.is_enum_type(fqn)
                        or self._index.is_message_type(fqn)):
                pkg = fqn.rsplit('.', 1)[0]
                resolved_short = fqn.split('.')[-1]
                if current_pkg and pkg and pkg != current_pkg:
                    base = (self._naming.cpp_type_namespace_for_package(pkg)
                            + '::' + resolved_short)
                else:
                    base = resolved_short
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
        enum, enum_pkg = _resolve_enum(self._index, field_type, current_pkg)
        if enum is not None:
            if enum and enum.values:
                suf = enum.suffix_of(enum.values[0].name)
                lit = screaming_to_pascal(suf) if suf else enum.values[0].name
                if '.' in field_type and not field_type.startswith('google.'):
                    pkg = '.'.join(field_type.split('.')[:-1])
                    if current_pkg and pkg != current_pkg:
                        return f'{self._naming.cpp_type_namespace_for_package(pkg)}::{short}::{lit}'
                else:
                    pkg = enum_pkg
                    if current_pkg and pkg and pkg != current_pkg:
                        return f'{self._naming.cpp_type_namespace_for_package(pkg)}::{field_type}::{lit}'
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
            # Sorted so output order is independent of hash randomisation.
            for dep in sorted(deps.get(name, ())):
                visit(dep)
            order.append(name)

        for m in messages:
            visit(m.name)
        return [by_name[n] for n in order]

    def _write_enum(self, f, enum: ProtoEnum) -> None:
        self._write_documentation(f, enum.documentation)
        f.write(f'enum class {enum.name} : int {{\n')
        for v in enum.values:
            suf = enum.suffix_of(v.name)
            lit = screaming_to_pascal(suf) if suf else v.name
            self._write_documentation(f, v.documentation, '    ')
            f.write(f'    {lit} = {v.number},\n')
        f.write('};\n\n')

    def _inline_base_fields(self, msg: ProtoMessage, current_pkg: str = ''):
        """Yield (field, comment) for all non-base fields, inlining any 'base'
        fields by expanding their sub-fields directly into the parent struct.

        Only one level of inlining is done; nested 'base' fields are kept as-is.
        """
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                short = field.type.split('.')[-1]
                base_msg, base_pkg = _resolve_message(
                    self._index, field.type, current_pkg)
                if base_msg and base_msg.name not in self._aliases:
                    # inline the base's own fields with collision renaming
                    for bf in base_msg.fields:
                        name = bf.name
                        if name in own_names:
                            name = short.lower() + '_' + name
                        bf_type = _proto_type_fqn(
                            self._index, bf.type, base_pkg) or bf.type
                        yield (_field_with_type(bf, bf_type), name,
                               f'  // from {base_msg.name}')
                    continue
            yield field, field.name, ''

    def _write_struct(self, f, msg: ProtoMessage,
                      current_pkg: str = '') -> None:
        self._write_documentation(f, msg.documentation)
        f.write(f'struct {msg.name} {{\n')
        for field, fname, comment in self._inline_base_fields(msg, current_pkg):
            self._write_documentation(f, field.documentation, '    ')
            base_cpp = self._cpp_field_type(field.type, False, current_pkg)
            if field.is_repeated:
                cpp_type = f'std::vector<{base_cpp}>'
                default = '{}'
                opt = ''
            elif field.is_optional and base_cpp not in ('std::string',):
                cpp_type = f'tl::optional<{base_cpp}>'
                default = ''
                opt = '  // optional'
            else:
                cpp_type = base_cpp
                default = self._cpp_default(base_cpp, field.type, current_pkg)
                opt = '  // optional' if field.is_optional else ''
            assign = f' = {default}' if default else ''
            f.write(f'    {cpp_type} {fname}{assign};{comment}{opt}\n')
        for oo in msg.oneofs:
            self._write_documentation(f, oo.documentation, '    ')
            f.write(f'    // oneof {oo.name}\n')
            for field in oo.fields:
                self._write_documentation(f, field.documentation, '    ')
                cpp_type = self._cpp_field_type(field.type, False, current_pkg)
                f.write(f'    tl::optional<{cpp_type}> {field.name};\n')
        f.write('};\n')
        struct_constants = _STRUCT_CONSTANTS.get(msg.name, [])
        if struct_constants:
            # A struct is a literal type only if every field is itself literal.
            # std::string / std::vector / tl::optional / message fields all make
            # it non-literal, so its named constants cannot be constexpr -- emit
            # them as inline const instead (e.g. Ack gained a string identifier).
            is_literal = not msg.oneofs and all(
                not f.is_repeated
                and self._cpp_field_type(f.type, False, current_pkg)
                in _LITERAL_CPP_TYPES
                for f in msg.all_fields()
            )
            qualifier = 'constexpr' if is_literal else 'inline const'
            for const_name, init in struct_constants:
                body = init[len(msg.name):] if init.startswith(msg.name) else (
                    '{ ' + init + ' }')
                f.write(qualifier + ' ' + msg.name + ' ' + const_name + body + ';\n')
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
                    result.append(
                        f'#include "{self._naming.type_header_for_package(indexed_pf.package)}"')
                    break
        return result

    def _write_hpp_for_file(self, path: Path, pf) -> None:
        ns = self._naming.cpp_namespace_for_package(pf.package)
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
            f.write('#include <tl/optional.hpp>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n')
            for inc in includes:
                f.write(inc + '\n')
            f.write(f'\nnamespace {ns} {{\n\n')

            # using aliases for scalar wrappers defined in this file
            for msg in pf.messages:
                if msg.name in self._aliases:
                    self._write_documentation(f, msg.documentation)
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
        everything into the common domain-model namespace."""
        per_file_headers = sorted(
            self._naming.type_header_for_package(pf.package)
            for pf in self._index.files
        )
        # A short name shared by more than one package cannot be re-exported
        # flat into the root (it would be ambiguous). Such types must be
        # referenced through their package sub-namespace
        # (e.g. domain_model::pim_osprey::sensor_products::SPRRequest); the
        # generated structs/codecs already qualify cross-package references.
        name_packages: Dict[str, set] = {}
        for pf in self._index.files:
            for msg in pf.messages:
                name_packages.setdefault(msg.name, set()).add(pf.package)
            for enum in pf.enums:
                name_packages.setdefault(enum.name, set()).add(pf.package)
        unique_names = {n for n, pkgs in name_packages.items() if len(pkgs) == 1}

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated umbrella types header\n')
            f.write('// Includes all data model type headers and re-exports\n')
            f.write(f'// uniquely-named types into namespace {_DATA_MODEL_TYPES_NS}.\n')
            f.write('// Names shared across packages are intentionally NOT flattened\n')
            f.write('// and must be used via their package sub-namespace.\n')
            f.write('#pragma once\n\n')
            for h in per_file_headers:
                f.write(f'#include "{h}"\n')
            f.write('\n// Re-export uniquely-named sub-namespace types into the\n')
            f.write('// common namespace so generated bindings can use a single root.\n')
            f.write(f'namespace {self._ns} {{\n')
            for pf in self._index.files:
                ns = self._naming.cpp_namespace_for_package(pf.package)
                if ns == self._ns:
                    continue
                for msg in pf.messages:
                    if msg.name in unique_names:
                        f.write(f'using {ns}::{msg.name};\n')
                        for const_name, _init in _STRUCT_CONSTANTS.get(
                                msg.name, []):
                            f.write(f'using {ns}::{const_name};\n')
                for enum in pf.enums:
                    if enum.name in unique_names:
                        f.write(f'using {ns}::{enum.name};\n')
            f.write(f'}} // namespace {self._ns}\n')
