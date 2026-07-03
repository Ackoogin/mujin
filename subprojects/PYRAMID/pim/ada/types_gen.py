#!/usr/bin/env python3
"""Ada plain-record types package generator.

Split verbatim from ada_codegen.py (generator refactor plan, phase 4).
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from proto_parser import (
    parse_proto_tree,
    ProtoTypeIndex,
    ProtoMessage,
    ProtoEnum,
    screaming_to_pascal,
)
from proto_resolve import _field_with_type, _proto_type_fqn, _resolve_message
from .naming import (
    _proto_pkg_of_type,
    _ada_pkg_from_proto_pkg,
    _ensure_parent_packages,
    _ADA_SCALAR_MAP,
    _ADA_DEFAULTS,
    _ADA_UNIT_FIELD_NAMES,
    _ada_name,
    _ada_array_name_for_repeated,
    _ada_pkg_segment,
    _ada_field_name,
    _common_ada_pkg,
)


class AdaTypesGenerator:
    """Generates ``{Prefix}_Types.ads`` from data model proto files.

    Scalar wrapper messages are emitted as subtypes; all other messages become
    record types.  Enums are emitted with a ``{Last_Word}_`` prefix on each
    literal.

    The Ada package name and output file name are derived automatically from
    the common package prefix of the data model proto files.

    Usage::
        gen = AdaTypesGenerator(data_model_dir)
        gen.generate(output_dir)
    """

    def __init__(self, data_model_source):
        if isinstance(data_model_source, Path):
            proto_files = parse_proto_tree(data_model_source)
        else:
            proto_files = list(data_model_source)
        self._index = ProtoTypeIndex(proto_files)
        self._data_model_dir = data_model_source if isinstance(data_model_source, Path) else None
        self._ada_pkg = _common_ada_pkg(self._index)
        self._file_base = self._ada_pkg.lower().replace('.', '-')
        self._aliases = self._find_scalar_wrappers()

    # -- public ----------------------------------------------------------------

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        generated_pkgs: List[str] = []
        for pf in self._index.files:
            ada_pkg = self._ada_pkg_for_file(pf)
            file_base = ada_pkg.lower().replace('.', '-')
            ads = out / (file_base + '.ads')
            self._write_ads_for_file(ads, pf, ada_pkg)
            generated_pkgs.append(ada_pkg)
            print(f'  Generated {ada_pkg}')
        _ensure_parent_packages(out, generated_pkgs)

    def write_file(self, pf, output_dir: str) -> None:
        """Emit the native types package for a single proto file (used for
        service-local wrapper messages homed in their Component-NS)."""
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        ada_pkg = self._ada_pkg_for_file(pf)
        file_base = ada_pkg.lower().replace('.', '-')
        self._write_ads_for_file(out / (file_base + '.ads'), pf, ada_pkg)
        _ensure_parent_packages(out, [ada_pkg])
        print(f'  Generated {ada_pkg}')

    @staticmethod
    def _ada_pkg_for_file(pf) -> str:
        """Derive Ada package name from a proto file's package.

        pyramid.data_model.common -> Pyramid.Data_Model.Common.Types
        """
        parts = pf.package.split('.')
        ada_parts = [_ada_pkg_segment(seg) for seg in parts]
        return '.'.join(ada_parts + ['Types'])

    def _with_clauses_for_file(self, pf) -> List[str]:
        """Return Ada 'with Pkg; use Pkg;' lines for imports from the index."""
        result = []
        seen = set()

        def add(ada_pkg: str) -> None:
            if ada_pkg not in seen and ada_pkg != self._ada_pkg_for_file(pf):
                seen.add(ada_pkg)
                result.append(f'with {ada_pkg};  use {ada_pkg};')

        for imp in pf.imports:
            if imp.startswith('google/'):
                continue
            pkg = imp.replace('/', '.').removesuffix('.proto')
            imp_stem = Path(imp).stem
            for indexed_pf in self._index.files:
                if indexed_pf.package == pkg or indexed_pf.path.stem == imp_stem:
                    add(self._ada_pkg_for_file(indexed_pf))
                    break

        for msg in pf.messages:
            for field, _ in self._inline_base_fields(msg, pf.package):
                proto_pkg = _proto_pkg_of_type(field.type)
                if (proto_pkg and proto_pkg != pf.package
                        and not proto_pkg.startswith('google.')):
                    add(_ada_pkg_from_proto_pkg(proto_pkg))
            for oneof in msg.oneofs:
                for field in oneof.fields:
                    proto_pkg = _proto_pkg_of_type(field.type)
                    if (proto_pkg and proto_pkg != pf.package
                            and not proto_pkg.startswith('google.')):
                        add(_ada_pkg_from_proto_pkg(proto_pkg))
        return result

    # -- internal --------------------------------------------------------------

    def _ada_pkg_for_type(self, short_name: str) -> Optional[str]:
        """Return the Ada package name that declares a proto message or enum
        by its short (unqualified) name."""
        for pf in self._index.files:
            for msg in pf.messages:
                if msg.name == short_name:
                    return self._ada_pkg_for_file(pf)
            for enum in pf.enums:
                if enum.name == short_name:
                    return self._ada_pkg_for_file(pf)
        return None

    def _shadow_subtypes_for_file(self, pf) -> Dict[str, str]:
        """Return {ada_type_name -> qualified_ada_type} for regular record
        fields where the Ada field name equals the Ada type name, which causes
        Ada's 'component cannot be used before end of record declaration' error."""
        result: Dict[str, str] = {}
        alias_names = set(self._aliases.keys())
        for msg in pf.messages:
            if msg.name in alias_names:
                continue
            for field, fname in self._inline_base_fields(msg, pf.package):
                ada_type, arr = self._ada_field_type(
                    field.type, field.is_repeated, fname, pf.package)
                ada_fname = _ada_field_name(fname, ada_type)
                if arr is None and ada_fname == ada_type:
                    short = field.type.split('.')[-1]
                    pkg = self._ada_pkg_for_type(short)
                    if pkg:
                        result[ada_type] = f'{pkg}.{ada_type}'
        return result

    def _find_scalar_wrappers(self) -> Dict[str, str]:
        aliases: Dict[str, str] = {'Timestamp': 'Long_Float'}
        for msg in self._index.all_messages():
            fields = msg.all_fields()
            if len(fields) == 1 and not fields[0].is_repeated:
                ft = fields[0].type
                fn = fields[0].name
                if ft in _ADA_SCALAR_MAP and fn in _ADA_UNIT_FIELD_NAMES:
                    aliases[msg.name] = _ADA_SCALAR_MAP[ft]
        return aliases

    def _ada_base_type(self, field_type: str,
                       current_pkg: str = '') -> str:
        """Resolve a proto type to its Ada element type name."""
        short = field_type.split('.')[-1]
        proto_pkg = _proto_pkg_of_type(field_type)
        if field_type in _ADA_SCALAR_MAP:
            return _ADA_SCALAR_MAP[field_type]
        if short in self._aliases:
            return self._aliases[short]
        if self._index.is_enum_type(field_type) or self._index.is_enum_type(short):
            if proto_pkg and proto_pkg != current_pkg:
                return f'{_ada_pkg_from_proto_pkg(proto_pkg)}.{_ada_name(short)}'
            return _ada_name(short)
        if self._index.is_message_type(field_type) or self._index.is_message_type(short):
            if short in self._aliases:
                return self._aliases[short]
            if proto_pkg and proto_pkg != current_pkg:
                return f'{_ada_pkg_from_proto_pkg(proto_pkg)}.{_ada_name(short)}'
            return _ada_name(short)
        return _ada_name(short)

    def _ada_field_type(self, field_type: str, repeated: bool,
                        field_name: str,
                        current_pkg: str = '') -> Tuple[str, Optional[str]]:
        """Return (ada_type_string, array_type_name_or_None).

        For repeated fields the Ada type used in record components is the
        access type (``Foo_Array_Acc``) so that unconstrained arrays can
        appear inside records.
        """
        base = self._ada_base_type(field_type, current_pkg)
        if repeated:
            arr = _ada_array_name_for_repeated(field_type, field_name)
            return (arr + '_Acc', arr)
        return (base, None)

    def _ada_default(self, ada_type: str, field_type: str) -> Optional[str]:
        """Return Ada default expression, or None if the type has component defaults."""
        short = field_type.split('.')[-1]
        if ada_type in _ADA_DEFAULTS:
            return _ADA_DEFAULTS[ada_type]
        if self._index.is_enum_type(field_type) or self._index.is_enum_type(short):
            enum = (self._index.resolve_enum(field_type)
                    or self._index.resolve_enum(short))
            if enum and enum.values:
                suf = enum.suffix_of(enum.values[0].name)
                val = screaming_to_pascal(suf) if suf else enum.values[0].name
                ada_enum = _ada_name(short)
                prefix = ada_enum.split('_')[-1] + '_'
                return prefix + val
        # Record types rely on their own component defaults -- no initialiser needed
        return None

    def _toposort(self, messages: List[ProtoMessage]) -> List[ProtoMessage]:
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

    def _inline_base_fields(self, msg: ProtoMessage,
                            current_pkg: str = ''):
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
                        bf_type = (
                            _proto_type_fqn(self._index, bf.type, base_pkg)
                            or bf.type
                        )
                        yield _field_with_type(bf, bf_type), name
                    continue
            yield field, field.name

    def _write_enum(self, f, enum: ProtoEnum) -> None:
        ada_name = _ada_name(enum.name)
        prefix = ada_name.split('_')[-1] + '_'
        f.write(f'   type {ada_name} is\n     (')
        vals = []
        for v in enum.values:
            suf = enum.suffix_of(v.name)
            lit = screaming_to_pascal(suf) if suf else v.name
            vals.append(prefix + lit)
        f.write((',\n      ').join(vals))
        f.write(');\n\n')

    def _arrays_for_msg(self, msg: ProtoMessage,
                        current_pkg: str) -> List[Tuple[str, str]]:
        """Return [(array_type_name, element_ada_type)] needed by this message."""
        seen: set = set()
        result = []
        for field, fname in self._inline_base_fields(msg, current_pkg):
            if field.is_repeated:
                arr = _ada_array_name_for_repeated(field.type, fname)
                if arr not in seen:
                    seen.add(arr)
                    result.append(
                        (arr, self._ada_base_type(field.type, current_pkg)))
        for oo in msg.oneofs:
            for fld in oo.fields:
                if fld.is_repeated:
                    arr = _ada_array_name_for_repeated(fld.type, fld.name)
                    if arr not in seen:
                        seen.add(arr)
                        result.append(
                            (arr, self._ada_base_type(fld.type, current_pkg)))
        return result

    def _write_record(self, f, msg: ProtoMessage,
                      shadow_subtypes: Optional[Dict[str, str]] = None,
                      current_pkg: str = '') -> None:
        ada_name = _ada_name(msg.name)
        shadow = shadow_subtypes or {}
        regular_fields = list(self._inline_base_fields(msg, current_pkg))
        f.write(f'   type {ada_name} is record\n')
        if not regular_fields and not msg.oneofs:
            f.write('      Padding : Boolean := False;\n')
        for field, fname in regular_fields:
            ada_type, arr = self._ada_field_type(
                field.type, field.is_repeated, fname, current_pkg)
            ada_fname = _ada_field_name(fname, ada_type)
            if arr:
                # Access-to-array fields default to null (no allocation)
                f.write(f'      {ada_fname} : {ada_type} := null;\n')
            elif ada_fname == ada_type and ada_type in shadow:
                # Use Base_ prefix to avoid component name shadowing the type name
                dflt = self._ada_default(ada_type, field.type)
                init = f' := {dflt}' if dflt else ''
                f.write(f'      {ada_fname} : Base_{ada_type}{init};\n')
            elif field.is_optional and _ADA_SCALAR_MAP.get(field.type, '') not in (
                    'Unbounded_String', ''):
                f.write(f'      Has_{ada_fname} : Boolean := False;\n')
                dflt = self._ada_default(ada_type, field.type)
                init = f' := {dflt}' if dflt else ''
                f.write(f'      {ada_fname} : {ada_type}{init};\n')
            else:
                dflt = self._ada_default(ada_type, field.type)
                init = f' := {dflt}' if dflt else ''
                f.write(f'      {ada_fname} : {ada_type}{init};\n')
        for oo in msg.oneofs:
            f.write(f'      --  oneof {oo.name}\n')
            for fld in oo.fields:
                ada_type, _ = self._ada_field_type(
                    fld.type, False, fld.name, current_pkg)
                ada_fname = _ada_field_name(fld.name, ada_type)
                f.write(f'      Has_{ada_fname} : Boolean := False;\n')
                dflt = self._ada_default(ada_type, fld.type)
                init = f' := {dflt}' if dflt else ''
                f.write(f'      {ada_fname} : {ada_type}{init};\n')
        f.write(f'   end record;\n\n')

    def _write_ads_for_file(self, path: Path, pf, ada_pkg: str) -> None:
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]
        sorted_msgs = self._toposort(non_alias)
        with_clauses = self._with_clauses_for_file(pf)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated types specification\n')
            f.write(f'--  Generated from: {pf.path.name}'
                    f' by generate_bindings.py (types)\n')
            f.write(f'--  Package: {ada_pkg}\n\n')
            f.write('with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;\n')
            for wc in with_clauses:
                f.write(wc + '\n')
            f.write(f'\npackage {ada_pkg} is\n\n')

            # Subtypes for scalar wrappers defined in this file
            for msg in pf.messages:
                if msg.name in self._aliases:
                    ada_n = _ada_name(msg.name)
                    f.write(f'   subtype {ada_n} is {self._aliases[msg.name]};\n')
            f.write('\n')

            # Enums defined in this file
            for enum in pf.enums:
                self._write_enum(f, enum)

            # Array types for scalar/enum elements
            emitted_arrays: set = set()
            all_enum_names = {_ada_name(e.name) for e in self._index.all_enums()}
            local_msg_names = {_ada_name(m.name) for m in non_alias}
            scalar_types = set(_ADA_SCALAR_MAP.values())
            for msg in sorted_msgs:
                for arr_name, elem_type in self._arrays_for_msg(msg, pf.package):
                    if arr_name not in emitted_arrays and (
                            elem_type in scalar_types
                            or elem_type in all_enum_names
                            or elem_type == 'Unbounded_String'
                            or elem_type not in local_msg_names):
                        f.write(f'   type {arr_name} is'
                                f' array (Positive range <>) of {elem_type};\n')
                        f.write(f'   type {arr_name}_Acc is'
                                f' access all {arr_name};\n')
                        emitted_arrays.add(arr_name)
            if emitted_arrays:
                f.write('\n')

            # Subtypes for fields whose Ada name equals their Ada type name
            shadow_subtypes = self._shadow_subtypes_for_file(pf)
            for ada_type, qualified in sorted(shadow_subtypes.items()):
                f.write(f'   subtype Base_{ada_type} is {qualified};\n')
            if shadow_subtypes:
                f.write('\n')

            # Records interleaved with their array type dependencies
            for msg in sorted_msgs:
                ada_msg_name = _ada_name(msg.name)
                self._write_record(f, msg, shadow_subtypes, pf.package)
                for later_msg in sorted_msgs:
                    for arr_name, elem_type in self._arrays_for_msg(
                            later_msg, pf.package):
                        if arr_name not in emitted_arrays and elem_type == ada_msg_name:
                            f.write(f'   type {arr_name} is'
                                    f' array (Positive range <>) of {elem_type};\n')
                            f.write(f'   type {arr_name}_Acc is'
                                    f' access all {arr_name};\n')
                            emitted_arrays.add(arr_name)
                            f.write('\n')

            f.write(f'end {ada_pkg};\n')


