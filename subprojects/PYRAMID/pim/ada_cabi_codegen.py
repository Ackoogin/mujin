#!/usr/bin/env python3
"""
Ada C-ABI data-model marshalling generator.

Emits Ada records that mirror the generated package-qualified C structs
and Ada marshalling between the generated native Ada records and those C
structs.  Variable-length data allocated by ``To_C`` uses C ``malloc`` so the
shared C++ ``<package>_<Type>_c_free`` routines can release it.
"""

from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple

from ada_codegen import (
    AdaTypesGenerator,
    _ADA_SCALAR_MAP,
    _ada_array_name_for_repeated,
    _ada_field_name,
    _ada_pkg_from_proto_pkg,
    _ada_name,
    _ada_pkg_segment,
    _proto_pkg_of_type,
    _ada_cabi_pkg_from_proto_pkg,
    _ensure_parent_packages,
)
from cabi_codegen import CabiMember, iter_cabi_members, _c_struct_name
from cpp_codegen import find_scalar_wrappers
from proto_parser import ProtoMessage, ProtoTypeIndex, parse_proto_tree


def _ada_cabi_pkg_for_file(pf) -> str:
    parts = pf.package.split('.')
    ada_parts = [_ada_pkg_segment(seg) for seg in parts]
    return '.'.join(ada_parts + ['Cabi'])


def _cabi_type_name_from_c(c_type: str) -> str:
    stem = c_type.removesuffix('_c')
    return _ada_name(stem) + '_C'


def _cabi_type_name(message_name: str, package: str = '') -> str:
    return _cabi_type_name_from_c(_c_struct_name(message_name, package))


def _c_field_name(name: str) -> str:
    return _ada_field_name(name)


def _has_field_name(name: str) -> str:
    return 'Has_' + _ada_name(name)


def _native_type_name(message_name: str, package: str = '') -> str:
    name = _ada_name(message_name)
    if package:
        return f'{_ada_pkg_from_proto_pkg(package)}.{name}'
    return name


def _free_proc_name(message_name: str, package: str = '') -> str:
    c_name = _c_struct_name(message_name, package).removesuffix('_c')
    return 'Free_' + _ada_name(c_name)


def _slice_array_name(member: CabiMember) -> str:
    return _ada_array_name_for_repeated(member.proto_field.type, member.name)


_C_SCALAR_TO_ADA: Dict[str, str] = {
    'double': 'Interfaces.C.double',
    'float': 'Interfaces.C.C_float',
    'int32_t': 'Interfaces.C.int',
    'int64_t': 'Interfaces.C.long',
    'uint8_t': 'Interfaces.C.unsigned_char',
    'uint32_t': 'Interfaces.C.unsigned',
    'uint64_t': 'Interfaces.C.unsigned_long',
}


class AdaCabiGenerator:
    """Generate Ada C-ABI mirror and marshalling packages."""

    def __init__(self, data_model_source):
        if isinstance(data_model_source, Path):
            proto_files = parse_proto_tree(data_model_source)
        else:
            proto_files = list(data_model_source)
        self._index = ProtoTypeIndex(proto_files)
        self._aliases = find_scalar_wrappers(self._index)
        self._ada_types = AdaTypesGenerator(proto_files)

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        generated_pkgs: List[str] = []
        for pf in self._index.files:
            pkg = _ada_cabi_pkg_for_file(pf)
            file_base = pkg.lower().replace('.', '-')
            self._write_spec(out / (file_base + '.ads'), pf, pkg)
            self._write_body(out / (file_base + '.adb'), pf, pkg)
            generated_pkgs.append(pkg)
            print(f'  Generated {pkg}')
        _ensure_parent_packages(out, generated_pkgs)

    def write_file(self, pf, output_dir: str) -> None:
        """Emit the C-ABI mirror+marshal packages for a single proto file (used
        for service-local wrapper messages homed in their Component-NS)."""
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        pkg = _ada_cabi_pkg_for_file(pf)
        file_base = pkg.lower().replace('.', '-')
        self._write_spec(out / (file_base + '.ads'), pf, pkg)
        self._write_body(out / (file_base + '.adb'), pf, pkg)
        _ensure_parent_packages(out, [pkg])
        print(f'  Generated {pkg}')

    def _with_clauses_for_file(self, pf, suffix: str,
                               use_clause: bool = True) -> List[str]:
        result = []
        seen = set()

        def add(ada_pkg: str) -> None:
            if ada_pkg in seen:
                return
            seen.add(ada_pkg)
            line = f'with {ada_pkg};'
            if use_clause:
                line += f'  use {ada_pkg};'
            result.append(line)

        for imp in pf.imports:
            if imp.startswith('google/'):
                continue
            pkg = imp.replace('/', '.').removesuffix('.proto')
            imp_stem = Path(imp).stem
            for indexed_pf in self._index.files:
                if indexed_pf.package == pkg or indexed_pf.path.stem == imp_stem:
                    if suffix == 'types':
                        ada_pkg = AdaTypesGenerator._ada_pkg_for_file(indexed_pf)
                    else:
                        ada_pkg = _ada_cabi_pkg_for_file(indexed_pf)
                    add(ada_pkg)
                    break

        for msg in pf.messages:
            if msg.name in self._aliases:
                continue
            for member in self._members(msg, pf.package):
                proto_pkg = _proto_pkg_of_type(member.proto_field.type)
                if (not proto_pkg or proto_pkg == pf.package
                        or proto_pkg.startswith('google.')):
                    continue
                if suffix == 'types':
                    add(_ada_pkg_from_proto_pkg(proto_pkg))
                else:
                    add(_ada_cabi_pkg_from_proto_pkg(proto_pkg))
        return result

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
            for dep in deps.get(name, set()):
                visit(dep)
            order.append(name)

        for m in messages:
            visit(m.name)
        return [by_name[n] for n in order]

    def _members(self, msg: ProtoMessage, current_pkg: str) -> Iterator[CabiMember]:
        yield from iter_cabi_members(
            self._index, self._aliases, msg, current_pkg)

    def _ada_c_type(self, c_type: str) -> str:
        if c_type == 'pyramid_str_t':
            return 'Pyramid_Str_T'
        if c_type == 'pyramid_slice_t':
            return 'Pyramid_Slice_T'
        if c_type.startswith('pyramid_') and c_type.endswith('_c'):
            return _cabi_type_name_from_c(c_type)
        return _C_SCALAR_TO_ADA.get(c_type, 'Interfaces.C.int')

    def _native_field(self, member: CabiMember,
                      current_pkg: str = '') -> str:
        if member.is_repeated:
            ada_type = (
                _ada_array_name_for_repeated(
                    member.proto_field.type, member.name) + '_Acc'
            )
        else:
            ada_type = self._ada_native_type(member, current_pkg)
        return _ada_field_name(member.name, ada_type)

    def _ada_native_type(self, member: CabiMember,
                         current_pkg: str = '') -> str:
        field_type = member.proto_field.type
        short = field_type.split('.')[-1]
        proto_pkg = _proto_pkg_of_type(field_type)
        if field_type in _ADA_SCALAR_MAP:
            return _ADA_SCALAR_MAP[field_type]
        if short in self._ada_types._aliases:
            return self._ada_types._aliases[short]
        if self._index.is_enum_type(field_type) or self._index.is_enum_type(short):
            if proto_pkg and proto_pkg != current_pkg:
                return f'{_ada_pkg_from_proto_pkg(proto_pkg)}.{_ada_name(short)}'
            return _ada_name(short)
        if self._index.is_message_type(field_type) or self._index.is_message_type(short):
            if proto_pkg and proto_pkg != current_pkg:
                return f'{_ada_pkg_from_proto_pkg(proto_pkg)}.{_ada_name(short)}'
            return _ada_name(short)
        return _ada_name(short)

    def _native_has_field(self, member: CabiMember,
                          current_pkg: str = '') -> Optional[str]:
        if member.is_oneof:
            return 'Has_' + self._native_field(member, current_pkg)
        if (member.proto_field.is_optional
                and member.proto_field.type in _ADA_SCALAR_MAP
                and _ADA_SCALAR_MAP[member.proto_field.type] not in (
                    'Unbounded_String', '')):
            return 'Has_' + self._native_field(member, current_pkg)
        return None

    def _enum_expr_to_c(self, expr: str) -> str:
        return f'Interfaces.C.int ({expr}\'Pos ({expr}))'

    def _write_spec(self, path: Path, pf, pkg: str) -> None:
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]
        sorted_msgs = self._toposort(non_alias)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated Ada C-ABI mirror specification\n')
            f.write(f'--  Generated from: {pf.path.name}'
                    f' by generate_bindings.py (ada cabi)\n')
            f.write(f'--  Package: {pkg}\n\n')
            f.write('with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;\n')
            f.write('with Interfaces.C;\n')
            f.write('with Interfaces.C.Strings;\n')
            f.write('with System;\n')
            f.write(f'with {AdaTypesGenerator._ada_pkg_for_file(pf)};'
                    f'  use {AdaTypesGenerator._ada_pkg_for_file(pf)};\n')
            for wc in self._with_clauses_for_file(pf, 'types'):
                f.write(wc + '\n')
            for wc in self._with_clauses_for_file(pf, 'cabi'):
                f.write(wc + '\n')
            f.write(f'\npackage {pkg} is\n\n')
            f.write('   pragma Elaborate_Body;\n\n')
            f.write('   type Pyramid_Str_T is record\n')
            f.write('      Ptr : Interfaces.C.Strings.chars_ptr :=\n')
            f.write('        Interfaces.C.Strings.Null_Ptr;\n')
            f.write('      Len : Interfaces.C.unsigned := 0;\n')
            f.write('   end record;\n')
            f.write('   pragma Convention (C, Pyramid_Str_T);\n\n')
            f.write('   type Pyramid_Slice_T is record\n')
            f.write('      Ptr : System.Address := System.Null_Address;\n')
            f.write('      Len : Interfaces.C.unsigned := 0;\n')
            f.write('   end record;\n')
            f.write('   pragma Convention (C, Pyramid_Slice_T);\n\n')

            for msg in sorted_msgs:
                c_name = _cabi_type_name(msg.name, pf.package)
                f.write(f'   type {c_name} is record\n')
                members = list(self._members(msg, pf.package))
                if not members:
                    f.write('      Padding : Interfaces.C.unsigned_char := 0;\n')
                for member in members:
                    if member.is_repeated:
                        f.write(f'      {_c_field_name(member.name)} : Pyramid_Slice_T;\n')
                    else:
                        if member.has_flag:
                            f.write(f'      {_has_field_name(member.name)} : '
                                    'Interfaces.C.unsigned_char := 0;\n')
                        f.write(f'      {_c_field_name(member.name)} : '
                                f'{self._ada_c_type(member.c_type)};\n')
                f.write('   end record;\n')
                f.write(f'   pragma Convention (C, {c_name});\n\n')

            for msg in non_alias:
                c_name = _cabi_type_name(msg.name, pf.package)
                native = _native_type_name(msg.name, pf.package)
                free_name = _c_struct_name(msg.name, pf.package) + '_free'
                f.write(f'   procedure To_C\n')
                f.write(f'     (In_Value  : {native};\n')
                f.write(f'      Out_Value : out {c_name});\n')
                f.write(f'   procedure From_C\n')
                f.write(f'     (In_Value  : {c_name};\n')
                f.write(f'      Out_Value : out {native});\n')
                proc_name = _free_proc_name(msg.name, pf.package)
                f.write(f'   procedure {proc_name} (Value : access {c_name});\n')
                f.write(f'   pragma Import (C, {proc_name}, "{free_name}");\n\n')

            f.write(f'end {pkg};\n')

    def _write_body(self, path: Path, pf, pkg: str) -> None:
        alias_names = set(self._aliases.keys())
        non_alias = [m for m in pf.messages if m.name not in alias_names]

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated Ada C-ABI marshalling body\n')
            f.write(f'--  Package body: {pkg}\n\n')
            f.write('with Ada.Unchecked_Conversion;\n')
            f.write('with Interfaces.C.Strings;\n')
            f.write('with System;\n\n')
            f.write(f'package body {pkg} is\n')
            f.write('   use type Interfaces.C.unsigned;\n')
            f.write('   use type Interfaces.C.unsigned_char;\n')
            f.write('   use type Interfaces.C.Strings.chars_ptr;\n')
            f.write('   use type System.Address;\n\n')
            f.write('   function Malloc (Size : Interfaces.C.size_t)\n')
            f.write('     return System.Address;\n')
            f.write('   pragma Import (C, Malloc, "malloc");\n\n')
            f.write('   function To_Chars_Ptr is new Ada.Unchecked_Conversion\n')
            f.write('     (System.Address, Interfaces.C.Strings.chars_ptr);\n\n')
            f.write('   function To_Address is new Ada.Unchecked_Conversion\n')
            f.write('     (Interfaces.C.Strings.chars_ptr, System.Address);\n\n')
            f.write('   function Bytes_For\n')
            f.write('     (Count : Natural;\n')
            f.write('      Element_Bits : Natural) return Interfaces.C.size_t\n')
            f.write('   is\n')
            f.write('   begin\n')
            f.write('      return Interfaces.C.size_t\n')
            f.write('        (Count * (Element_Bits / System.Storage_Unit));\n')
            f.write('   end Bytes_For;\n\n')
            f.write('   function To_Ada_String (S : Pyramid_Str_T) return String is\n')
            f.write('   begin\n')
            f.write('      if S.Ptr = Interfaces.C.Strings.Null_Ptr or else S.Len = 0 then\n')
            f.write('         return "";\n')
            f.write('      end if;\n')
            f.write('      return Interfaces.C.Strings.Value\n')
            f.write('        (S.Ptr, Interfaces.C.size_t (S.Len));\n')
            f.write('   end To_Ada_String;\n\n')
            f.write('   procedure Dup_Str (Out_Value : out Pyramid_Str_T; S : String) is\n')
            f.write('   begin\n')
            f.write('      Out_Value := (Ptr => Interfaces.C.Strings.Null_Ptr, Len => 0);\n')
            f.write("      if S'Length = 0 then\n")
            f.write('         return;\n')
            f.write('      end if;\n')
            f.write("      Out_Value.Len := Interfaces.C.unsigned (S'Length);\n")
            f.write('      Out_Value.Ptr := To_Chars_Ptr\n')
            f.write('        (Malloc (Interfaces.C.size_t (Out_Value.Len)));\n')
            f.write('      declare\n')
            f.write("         type Char_Array is array (Positive range 1 .. S'Length)\n")
            f.write('           of Interfaces.C.char;\n')
            f.write('         pragma Convention (C, Char_Array);\n')
            f.write('         Chars : Char_Array;\n')
            f.write("         for Chars'Address use To_Address (Out_Value.Ptr);\n")
            f.write('         pragma Import (Ada, Chars);\n')
            f.write('      begin\n')
            f.write("         for I in S'Range loop\n")
            f.write("            Chars (I - S'First + 1) :=\n")
            f.write("              Interfaces.C.char'Val (Character'Pos (S (I)));\n")
            f.write('         end loop;\n')
            f.write('      end;\n')
            f.write('   end Dup_Str;\n\n')

            for msg in non_alias:
                self._write_to_c(f, msg, pf.package)
                self._write_from_c(f, msg, pf.package)

            f.write(f'end {pkg};\n')

    def _write_to_c(self, f, msg: ProtoMessage, current_pkg: str) -> None:
        native = _native_type_name(msg.name, current_pkg)
        c_name = _cabi_type_name(msg.name, current_pkg)
        f.write(f'   procedure To_C\n')
        f.write(f'     (In_Value  : {native};\n')
        f.write(f'      Out_Value : out {c_name})\n')
        f.write('   is\n')
        f.write('   begin\n')
        f.write('      Out_Value := (others => <>);\n')
        for member in self._members(msg, current_pkg):
            self._emit_to_c_member(f, member, current_pkg)
        f.write('   end To_C;\n\n')

    def _write_from_c(self, f, msg: ProtoMessage, current_pkg: str) -> None:
        native = _native_type_name(msg.name, current_pkg)
        c_name = _cabi_type_name(msg.name, current_pkg)
        f.write(f'   procedure From_C\n')
        f.write(f'     (In_Value  : {c_name};\n')
        f.write(f'      Out_Value : out {native})\n')
        f.write('   is\n')
        f.write('   begin\n')
        f.write('      Out_Value := (others => <>);\n')
        for member in self._members(msg, current_pkg):
            self._emit_from_c_member(f, member, current_pkg)
        f.write('   end From_C;\n\n')

    def _emit_to_c_member(self, f, member: CabiMember,
                          current_pkg: str) -> None:
        c_name = _c_field_name(member.name)
        native_name = self._native_field(member, current_pkg)
        native_has = self._native_has_field(member, current_pkg)
        c_has = _has_field_name(member.name)

        if member.has_flag:
            if native_has:
                f.write(f'      Out_Value.{c_has} :=\n')
                f.write(f'        (if In_Value.{native_has} then 1 else 0);\n')
            else:
                f.write(f'      Out_Value.{c_has} := 1;\n')

        if member.kind in ('scalar', 'opt_scalar'):
            f.write(f'      Out_Value.{c_name} := '
                    f'{self._scalar_to_c_expr(member, f"In_Value.{native_name}")};\n')
        elif member.kind in ('bool', 'opt_bool'):
            f.write(f'      Out_Value.{c_name} :=\n')
            f.write(f'        (if In_Value.{native_name} then 1 else 0);\n')
        elif member.kind in ('string', 'opt_string'):
            f.write(f'      Dup_Str (Out_Value.{c_name}, To_String (In_Value.{native_name}));\n')
        elif member.kind in ('enum', 'opt_enum'):
            f.write(f'      Out_Value.{c_name} := Interfaces.C.int\n')
            f.write(f'        ({self._ada_native_type(member, current_pkg)}\'Pos (In_Value.{native_name}));\n')
        elif member.kind in ('message', 'opt_message'):
            f.write(f'      To_C (In_Value.{native_name}, Out_Value.{c_name});\n')
        elif member.is_repeated:
            self._emit_to_c_repeated(
                f, member, c_name, native_name, current_pkg)

    def _emit_from_c_member(self, f, member: CabiMember,
                            current_pkg: str) -> None:
        c_name = _c_field_name(member.name)
        native_name = self._native_field(member, current_pkg)
        native_has = self._native_has_field(member, current_pkg)
        c_has = _has_field_name(member.name)

        if native_has:
            f.write(f'      Out_Value.{native_has} := In_Value.{c_has} /= 0;\n')
            f.write(f'      if Out_Value.{native_has} then\n')
            indent = '         '
        elif member.has_flag:
            f.write(f'      if In_Value.{c_has} /= 0 then\n')
            indent = '         '
        else:
            indent = '      '

        if member.kind in ('scalar', 'opt_scalar'):
            f.write(f'{indent}Out_Value.{native_name} := '
                    f'{self._scalar_from_c_expr(member, f"In_Value.{c_name}", current_pkg)};\n')
        elif member.kind in ('bool', 'opt_bool'):
            f.write(f'{indent}Out_Value.{native_name} := In_Value.{c_name} /= 0;\n')
        elif member.kind in ('string', 'opt_string'):
            f.write(f'{indent}Out_Value.{native_name} :=\n')
            f.write(f'{indent}  To_Unbounded_String (To_Ada_String (In_Value.{c_name}));\n')
        elif member.kind in ('enum', 'opt_enum'):
            f.write(f'{indent}Out_Value.{native_name} := {self._ada_native_type(member, current_pkg)}\'Val\n')
            f.write(f'{indent}  (Integer (In_Value.{c_name}));\n')
        elif member.kind in ('message', 'opt_message'):
            f.write(f'{indent}From_C (In_Value.{c_name}, Out_Value.{native_name});\n')
        elif member.is_repeated:
            self._emit_from_c_repeated(
                f, member, c_name, native_name, indent, current_pkg)

        if member.has_flag:
            f.write('      end if;\n')

    def _repeated_c_elem_type(self, member: CabiMember) -> str:
        if member.kind == 'repeated_string':
            return 'Pyramid_Str_T'
        if member.kind == 'repeated_bool':
            return 'Interfaces.C.unsigned_char'
        if member.kind == 'repeated_enum':
            return 'Interfaces.C.int'
        if member.kind == 'repeated_message':
            return self._ada_c_type(member.c_type)
        return self._ada_c_type(member.c_type)

    def _scalar_to_c_expr(self, member: CabiMember, native_expr: str) -> str:
        return f'{self._ada_c_type(member.c_type)} ({native_expr})'

    def _scalar_from_c_expr(self, member: CabiMember, c_expr: str,
                            current_pkg: str = '') -> str:
        native = self._ada_native_type(member, current_pkg)
        return f'{native} ({c_expr})'

    def _to_c_repeated_assignment(self, member: CabiMember,
                                  arr_expr: str, native_expr: str,
                                  current_pkg: str) -> str:
        if member.kind == 'repeated_string':
            return f'Dup_Str ({arr_expr}, To_String ({native_expr}));'
        if member.kind == 'repeated_bool':
            return f'{arr_expr} := (if {native_expr} then 1 else 0);'
        if member.kind == 'repeated_enum':
            return (f'{arr_expr} := Interfaces.C.int'
                    f' ({self._ada_native_type(member, current_pkg)}'
                    f'\'Pos ({native_expr}));')
        if member.kind == 'repeated_message':
            return f'To_C ({native_expr}, {arr_expr});'
        if member.c_type == 'double':
            return f'{arr_expr} := Interfaces.C.double ({native_expr});'
        if member.c_type == 'int32_t':
            return f'{arr_expr} := Interfaces.C.int ({native_expr});'
        return f'{arr_expr} := {self._repeated_c_elem_type(member)} ({native_expr});'

    def _from_c_repeated_assignment(self, member: CabiMember,
                                    native_expr: str, arr_expr: str,
                                    current_pkg: str) -> str:
        if member.kind == 'repeated_string':
            return (f'{native_expr} := To_Unbounded_String'
                    f' (To_Ada_String ({arr_expr}));')
        if member.kind == 'repeated_bool':
            return f'{native_expr} := {arr_expr} /= 0;'
        if member.kind == 'repeated_enum':
            return (f'{native_expr} := {self._ada_native_type(member, current_pkg)}\'Val'
                    f' (Integer ({arr_expr}));')
        if member.kind == 'repeated_message':
            return f'From_C ({arr_expr}, {native_expr});'
        if member.c_type in ('double', 'float'):
            return f'{native_expr} := Long_Float ({arr_expr});'
        return f'{native_expr} := Integer ({arr_expr});'

    def _emit_to_c_repeated(self, f, member: CabiMember,
                            c_name: str, native_name: str,
                            current_pkg: str) -> None:
        elem_type = self._repeated_c_elem_type(member)
        arr_type = _slice_array_name(member) + '_C_Array'
        f.write('      declare\n')
        f.write('         Count : constant Natural :=\n')
        f.write(f'           (if In_Value.{native_name} = null then 0\n')
        f.write(f'            else In_Value.{native_name}.all\'Length);\n')
        f.write('      begin\n')
        f.write('         if Count > 0 then\n')
        f.write(f'            Out_Value.{c_name}.Ptr :=\n')
        f.write(f'              Malloc (Bytes_For (Count, {elem_type}\'Object_Size));\n')
        f.write(f'            Out_Value.{c_name}.Len := Interfaces.C.unsigned (Count);\n')
        f.write('            declare\n')
        f.write(f'               type {arr_type} is array (Positive range 1 .. Count)\n')
        f.write(f'                 of {elem_type};\n')
        f.write(f'               pragma Convention (C, {arr_type});\n')
        f.write(f'               Arr : {arr_type};\n')
        f.write(f"               for Arr'Address use Out_Value.{c_name}.Ptr;\n")
        f.write('               pragma Import (Ada, Arr);\n')
        f.write('            begin\n')
        f.write('               for I in 1 .. Count loop\n')
        stmt = self._to_c_repeated_assignment(
            member, 'Arr (I)', f'In_Value.{native_name} (I)', current_pkg)
        f.write(f'                  {stmt}\n')
        f.write('               end loop;\n')
        f.write('            end;\n')
        f.write('         end if;\n')
        f.write('      end;\n')

    def _emit_from_c_repeated(self, f, member: CabiMember, c_name: str,
                              native_name: str, indent: str,
                              current_pkg: str) -> None:
        elem_type = self._repeated_c_elem_type(member)
        arr_type = _slice_array_name(member)
        native_arr_type = f'{_ada_pkg_from_proto_pkg(current_pkg)}.{arr_type}'
        c_arr_type = arr_type + '_C_Array'
        f.write(f'{indent}if In_Value.{c_name}.Ptr /= System.Null_Address\n')
        f.write(f'{indent}  and then In_Value.{c_name}.Len > 0\n')
        f.write(f'{indent}then\n')
        f.write(f'{indent}   declare\n')
        f.write(f'{indent}      Count : constant Natural := Natural (In_Value.{c_name}.Len);\n')
        f.write(f'{indent}      type {c_arr_type} is array (Positive range 1 .. Count)\n')
        f.write(f'{indent}        of {elem_type};\n')
        f.write(f'{indent}      pragma Convention (C, {c_arr_type});\n')
        f.write(f'{indent}      Arr : {c_arr_type};\n')
        f.write(f"{indent}      for Arr'Address use In_Value.{c_name}.Ptr;\n")
        f.write(f'{indent}      pragma Import (Ada, Arr);\n')
        f.write(f'{indent}   begin\n')
        f.write(f'{indent}      Out_Value.{native_name} := new {native_arr_type} (1 .. Count);\n')
        f.write(f'{indent}      for I in 1 .. Count loop\n')
        stmt = self._from_c_repeated_assignment(
            member, f'Out_Value.{native_name} (I)', 'Arr (I)', current_pkg)
        f.write(f'{indent}         {stmt}\n')
        f.write(f'{indent}      end loop;\n')
        f.write(f'{indent}   end;\n')
        f.write(f'{indent}else\n')
        f.write(f'{indent}   Out_Value.{native_name} := null;\n')
        f.write(f'{indent}end if;\n')
