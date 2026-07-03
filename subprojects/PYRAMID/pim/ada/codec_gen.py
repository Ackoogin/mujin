#!/usr/bin/env python3
"""Ada data-model JSON codec generator (proto-driven, per file).

Split verbatim from ada_codegen.py (generator refactor plan, phase 4).
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from proto_parser import ProtoTypeIndex, screaming_to_pascal, camel_to_snake
from proto_resolve import (
    _field_with_type, _proto_type_fqn, _resolve_message,
    _package_for_proto_type,
)
from .naming import (
    _proto_pkg_of_type,
    _ada_pkg_from_proto_pkg,
    _ensure_parent_packages,
    _ADA_SCALAR_MAP,
    _ADA_UNIT_FIELD_NAMES,
    _ada_name,
    _ada_array_name_for_repeated,
    _ada_field_name,
)
from .types_gen import AdaTypesGenerator


class AdaDataModelCodecGenerator:
    """Generates ``{pkg}_codec.ads/adb`` from a single data model proto file.

    For each message and enum in the proto, emits JSON To_Json / From_Json
    functions in the same Ada package as the types.

    Usage::
        proto_files = parse_proto_tree(data_model_dir)
        index = ProtoTypeIndex(proto_files)
        for pf in proto_files:
            gen = AdaDataModelCodecGenerator(pf, index)
            gen.generate(output_dir)
    """

    def __init__(self, pf, index: 'ProtoTypeIndex'):
        self._pf = pf
        self._index = index
        self._ada_pkg = AdaTypesGenerator._ada_pkg_for_file(pf) + '_Codec'
        self._types_pkg = AdaTypesGenerator._ada_pkg_for_file(pf)
        file_base = self._ada_pkg.lower().replace('.', '-')
        self._ads_name = file_base + '.ads'
        self._adb_name = file_base + '.adb'
        # Build map: message name -> codec package name (for qualified calls)
        self._msg_to_codec: Dict[str, str] = {}
        # Build map: enum name -> codec package name (for qualified calls)
        self._enum_to_codec: Dict[str, str] = {}
        # Build map: proto package -> codec package name. Short-name maps are
        # ambiguous in the duplicate-heavy PIM tree (many packages reuse names
        # like ``Generic_Component``); resolve foreign codec calls by the field's
        # declaring proto package instead, falling back to the short-name maps.
        self._pkg_to_codec: Dict[str, str] = {}
        for other_pf in self._index.files:
            codec_pkg = AdaTypesGenerator._ada_pkg_for_file(other_pf) + '_Codec'
            self._pkg_to_codec[other_pf.package] = codec_pkg
            for m in other_pf.messages:
                self._msg_to_codec[m.name] = codec_pkg
            for e in other_pf.enums:
                self._enum_to_codec[e.name] = codec_pkg
        # Build alias map (scalar wrappers)
        self._aliases: Dict[str, str] = {'Timestamp': 'Long_Float'}
        for msg in self._index.all_messages():
            fields = msg.all_fields()
            if len(fields) == 1 and not fields[0].is_repeated:
                ft = fields[0].type
                fn = fields[0].name
                if ft in _ADA_SCALAR_MAP and fn in _ADA_UNIT_FIELD_NAMES:
                    self._aliases[msg.name] = _ADA_SCALAR_MAP[ft]

    def generate(self, output_dir: str) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        if not self._has_generated_content():
            print(f'  Skipped empty codec package {self._ada_pkg}')
            return
        self._write_spec(out / self._ads_name)
        self._write_body(out / self._adb_name)
        _ensure_parent_packages(out, [self._ada_pkg])
        print(f'  Generated {self._ada_pkg}')

    def _has_generated_content(self) -> bool:
        """Return True when this proto file needs a codec package.

        We only need to emit a ``*_Codec`` package when the file contributes at
        least one real public codec/helper declaration:

        - enum string converters for enums defined in the file
        - To_Json / From_Json routines for non-alias messages defined in the file

        Files that contain only scalar-wrapper aliases, services, or no
        messages/enums at all produce empty packages today; skip those entirely.
        """
        alias_names = set(self._aliases.keys())
        has_structs = any(m.name not in alias_names for m in self._pf.messages)
        has_enums = bool(self._pf.enums)
        return has_structs or has_enums

    def _inline_base_fields(self, msg):
        """Expand 'base' fields inline, same logic as AdaTypesGenerator."""
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                short = field.type.split('.')[-1]
                base_msg, base_pkg = _resolve_message(
                    self._index, field.type, self._pf.package)
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

    def _type_ref(self, ada_n: str) -> str:
        """Return a fully-qualified reference to a message/enum type declared in
        this codec's Types package.

        Bare type names collide in several ways in the duplicate-heavy PIM tree:
        a message whose name equals an enclosing package segment (e.g.
        ``Authorisation`` in ``...Authorisation.Types_Codec``) resolves to the
        parent *package*; a generated enum named ``Boolean`` is hidden by the
        predefined ``Standard.Boolean``. Qualifying every own-package type with
        its Types package sidesteps all of these unambiguously.
        """
        return f'{self._types_pkg}.{ada_n}'

    # ------------------------------------------------------------------ spec

    def _write_spec(self, path: Path) -> None:
        alias_names = set(self._aliases.keys())
        structs = [m for m in self._pf.messages if m.name not in alias_names]

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated data model JSON codec specification\n')
            f.write(f'--  Generated from: {self._pf.path.name}'
                    f' by generate_bindings.py (codec)\n')
            f.write(f'--  Package: {self._ada_pkg}\n\n')
            f.write('with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;\n')
            f.write(f'with {self._types_pkg};  use {self._types_pkg};\n')
            f.write(f'\npackage {self._ada_pkg} is\n\n')

            # Enum converters
            for enum in self._pf.enums:
                ada_n = _ada_name(enum.name)
                ref = self._type_ref(ada_n)
                f.write(f'   function To_String (V : {ref}) return String;\n')
                f.write(f'   function {ada_n}_From_String (S : String) return {ref};\n')
            if self._pf.enums:
                f.write('\n')

            # Struct codec declarations
            for msg in structs:
                ada_n = _ada_name(msg.name)
                ref = self._type_ref(ada_n)
                f.write(f'   function To_Json (Msg : {ref}) return String;\n')
                f.write(f'   function From_Json (S : String;'
                        f' Tag : access {ref}) return {ref};\n')
            if structs:
                f.write('\n')

            f.write(f'end {self._ada_pkg};\n')

    # ---------------------------------------------------------------- helpers

    def _field_codec_pkg(self, fld, short_map: Dict[str, str]) -> str:
        """FQN-aware codec package for a message/enum field.

        Resolve the field's declaring proto package (current package first for
        bare names, then a unique global match) and map it to its codec package.
        Falls back to the short-name map when the package cannot be resolved
        (e.g. a same-package reference the index could not pin down).
        """
        pkg = _package_for_proto_type(self._index, fld.type, self._pf.package)
        if pkg and pkg in self._pkg_to_codec:
            return self._pkg_to_codec[pkg]
        return short_map.get(fld.type.split('.')[-1], self._ada_pkg)

    def _qualified_to_json(self, fld, accessor: str) -> str:
        """Return a qualified To_Json call for a nested message field.

        If the message type lives in a different codec package, qualify with
        that package name to avoid Ada overload ambiguity.
        """
        codec_pkg = self._field_codec_pkg(fld, self._msg_to_codec)
        if codec_pkg != self._ada_pkg:
            return f'{codec_pkg}.To_Json ({accessor})'
        return f'To_Json ({accessor})'

    def _string_to_json_expr(self, accessor: str) -> str:
        """Return an Ada expression that emits a JSON-quoted string."""
        return f'"""" & Ada.Strings.Unbounded.To_String ({accessor}) & """"'

    def _enum_to_json_expr(self, fld, accessor: str) -> str:
        """Return an Ada expression that emits a JSON-quoted enum value."""
        enum_pkg = self._field_codec_pkg(fld, self._enum_to_codec)
        qual = f'{enum_pkg}.' if enum_pkg != self._ada_pkg else ''
        return f'"""" & {qual}To_String ({accessor}) & """"'

    def _foreign_codec_deps(self) -> List[str]:
        """Return sorted list of foreign codec package names that this
        file's messages reference (for ``with`` clauses)."""
        deps: set = set()
        alias_names = set(self._aliases.keys())

        def add_field(fld) -> None:
            short = fld.type.split('.')[-1]
            if short in self._aliases:
                return
            if self._is_field_message(fld):
                codec = self._field_codec_pkg(fld, self._msg_to_codec)
            elif (self._index.is_enum_type(fld.type) or
                  self._index.is_enum_type(short)):
                codec = self._field_codec_pkg(fld, self._enum_to_codec)
            else:
                return
            if codec and codec != self._ada_pkg:
                deps.add(codec)

        for msg in self._pf.messages:
            if msg.name in alias_names:
                continue
            for fld, fname in self._inline_base_fields(msg):
                add_field(fld)
            # oneof variants are separate from msg.fields and also emit foreign
            # To_Json/From_Json calls, so their codec packages must be withed.
            for oneof in msg.oneofs:
                for fld in oneof.fields:
                    add_field(fld)
        return sorted(deps)

    def _ada_base_type(self, field_type: str) -> str:
        # Mirror AdaTypesGenerator._ada_base_type so that the component name
        # decision in _ada_field_name (which adds a ``Val_`` prefix when the
        # field name equals its type name) matches the record actually emitted
        # by the types generator. In particular, cross-package message/enum
        # types are package-qualified there, which keeps the bare field name
        # from colliding with the (now qualified) type name.
        current_pkg = self._pf.package
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
                        field_name: str) -> Tuple[str, Optional[str]]:
        base = self._ada_base_type(field_type)
        if repeated:
            arr = _ada_array_name_for_repeated(field_type, field_name)
            return (arr + '_Acc', arr)
        return (base, None)

    # -- field-level serialisation helpers ------------------------------------

    def _to_json_expr(self, fld, ada_fname: str) -> str:
        """Return an Ada expression that serialises a single scalar/enum field
        value to its JSON text representation."""
        if fld.type == 'bool':
            return f'(if Msg.{ada_fname} then "true" else "false")'
        if fld.type in ('int32', 'int64', 'sint32', 'sint64',
                         'uint32', 'uint64', 'fixed32', 'fixed64',
                         'sfixed32', 'sfixed64'):
            ada_int_type = _ADA_SCALAR_MAP[fld.type]
            return f"{ada_int_type}'Image (Msg.{ada_fname})"
        if fld.type in ('float', 'double'):
            return f"Long_Float'Image (Msg.{ada_fname})"
        if fld.type == 'string':
            return self._string_to_json_expr(f'Msg.{ada_fname}')
        short = fld.type.split('.')[-1]
        if self._index.is_enum_type(fld.type) or self._index.is_enum_type(short):
            return self._enum_to_json_expr(fld, f'Msg.{ada_fname}')
        # Alias (unit type collapsed to scalar) -- dispatch on target type
        if short in self._aliases:
            ada_target = self._aliases[short]
            if ada_target == 'Unbounded_String':
                return self._string_to_json_expr(f'Msg.{ada_fname}')
            elif ada_target in ('Integer', 'Long_Integer', 'Natural'):
                return f"{ada_target}'Image (Msg.{ada_fname})"
            elif ada_target == 'Boolean':
                return f'(if Msg.{ada_fname} then "true" else "false")'
            else:
                return f"Long_Float'Image (Msg.{ada_fname})"
        return self._string_to_json_expr(f'Msg.{ada_fname}')

    def _from_json_stmts(self, f, fld, ada_fname: str, wire: str,
                          indent: str = '      ') -> None:
        """Emit Ada statements that deserialise a single scalar/enum field
        from a GNATCOLL.JSON JSON_Value *J* into Result.<ada_fname>."""
        short = fld.type.split('.')[-1]
        ada_target = self._aliases.get(short, '')

        if fld.type == 'bool':
            f.write(f'{indent}if Has_Field (J, "{wire}") then\n')
            f.write(f'{indent}   declare\n')
            f.write(f'{indent}      Val : constant JSON_Value := Get (J, "{wire}");\n')
            f.write(f'{indent}   begin\n')
            f.write(f'{indent}      Result.{ada_fname} := Get (Val);\n')
            f.write(f'{indent}   end;\n')
            f.write(f'{indent}end if;\n')
        elif fld.type == 'string' or ada_target == 'Unbounded_String':
            f.write(f'{indent}if Has_Field (J, "{wire}") then\n')
            f.write(f'{indent}   declare\n')
            f.write(f'{indent}      Val : constant JSON_Value := Get (J, "{wire}");\n')
            f.write(f'{indent}      Str : constant String := Get (Val);\n')
            f.write(f'{indent}   begin\n')
            f.write(f'{indent}      Result.{ada_fname} := To_Unbounded_String (Str);\n')
            f.write(f'{indent}   end;\n')
            f.write(f'{indent}end if;\n')
        elif fld.type in ('float', 'double') or ada_target in (
                'Long_Float', 'Float'):
            # Numeric float
            if fld.type in ('float', 'double') or ada_target == 'Long_Float':
                f.write(f'{indent}if Has_Field (J, "{wire}") then\n')
                f.write(f'{indent}   Result.{ada_fname} := Get_Long_Float (Get (J, "{wire}"));\n')
                f.write(f'{indent}end if;\n')
            elif ada_target == 'Float':
                f.write(f'{indent}if Has_Field (J, "{wire}") then\n')
                f.write(f'{indent}   Result.{ada_fname} := Float (Get_Long_Float (Get (J, "{wire}")));\n')
                f.write(f'{indent}end if;\n')
        elif fld.type in ('int32', 'int64', 'sint32', 'sint64',
                           'uint32', 'uint64', 'fixed32', 'fixed64',
                           'sfixed32', 'sfixed64') or ada_target in (
                           'Integer', 'Long_Integer', 'Natural'):
            int_type = _ADA_SCALAR_MAP.get(fld.type, ada_target or 'Integer')
            f.write(f'{indent}if Has_Field (J, "{wire}") then\n')
            f.write(f'{indent}   Result.{ada_fname} := {int_type} (Get_Long_Float (Get (J, "{wire}")));\n')
            f.write(f'{indent}end if;\n')
        elif (self._index.is_enum_type(fld.type) or
              self._index.is_enum_type(short)):
            enum_ada = _ada_name(short)
            enum_pkg = self._field_codec_pkg(fld, self._enum_to_codec)
            qual = f'{enum_pkg}.' if enum_pkg != self._ada_pkg else ''
            f.write(f'{indent}if Has_Field (J, "{wire}") then\n')
            f.write(f'{indent}   declare\n')
            f.write(f'{indent}      Val : constant JSON_Value := Get (J, "{wire}");\n')
            f.write(f'{indent}      Str : constant String := Get (Val);\n')
            f.write(f'{indent}   begin\n')
            f.write(f'{indent}      Result.{ada_fname} := {qual}{enum_ada}_From_String (Str);\n')
            f.write(f'{indent}   end;\n')
            f.write(f'{indent}end if;\n')

    def _is_field_scalar_or_enum(self, fld) -> bool:
        """Return True if *fld* is a scalar, enum or alias (unit type)."""
        if fld.type in _ADA_SCALAR_MAP:
            return True
        short = fld.type.split('.')[-1]
        if short in self._aliases:
            return True
        if self._index.is_enum_type(fld.type) or self._index.is_enum_type(short):
            return True
        return False

    def _is_field_message(self, fld) -> bool:
        short = fld.type.split('.')[-1]
        return (self._index.is_message_type(fld.type) or
                self._index.is_message_type(short))

    # -- codec writers ---------------------------------------------------------

    def _write_codec(self, f, ada_n: str, msg) -> None:
        """Generate To_Json / From_Json for any message type, dispatching
        between the simple string-concat approach for scalar-only records
        and the full implementation for complex records."""
        fields = list(self._inline_base_fields(msg))
        has_complex = bool(msg.oneofs)
        if not has_complex:
            for fld, fname in fields:
                if fld.is_repeated or fld.oneof_group:
                    has_complex = True
                    break
                if self._is_field_message(fld):
                    has_complex = True
                    break

        if has_complex:
            self._write_complex_codec(f, ada_n, msg, fields)
        else:
            # All fields are scalar/enum/alias -- use simple concat
            simple_fields = [fld for fld, _ in fields]
            self._write_simple_codec(f, ada_n, simple_fields)

    def _write_simple_codec(self, f, ada_n: str, fields) -> None:
        """Generate working To_Json / From_Json for scalar-only records."""
        ref = self._type_ref(ada_n)
        # -- To_Json --
        f.write(f'   function To_Json (Msg : {ref}) return String is\n')
        if not fields:
            f.write('      pragma Unreferenced (Msg);\n')
        f.write(f'   begin\n')
        if not fields:
            f.write('      return "{}";\n')
        else:
            f.write(f'      return "{{" &\n')
            parts = []
            for fld in fields:
                wire = camel_to_snake(fld.name)
                ada_type, _ = self._ada_field_type(
                    fld.type, fld.is_repeated, fld.name)
                ada_fname = _ada_field_name(fld.name, ada_type)
                expr = self._to_json_expr(fld, ada_fname)
                parts.append(f'        """{wire}"":" & {expr}')
            f.write(' &\n        "," &\n'.join(parts))
            f.write(' &\n')
            f.write(f'        "}}";\n')
        f.write(f'   end To_Json;\n\n')

        # -- From_Json --
        f.write(f'   function From_Json'
                f' (S : String; Tag : access {ref})'
                f' return {ref} is\n')
        f.write(f'      pragma Unreferenced (Tag);\n')
        f.write(f'      J      : constant JSON_Value := Read (S);\n')
        f.write(f'      Result : {ref};\n')
        f.write(f'   begin\n')
        for fld in fields:
            wire = camel_to_snake(fld.name)
            ada_type, _ = self._ada_field_type(fld.type, fld.is_repeated, fld.name)
            ada_fname = _ada_field_name(fld.name, ada_type)
            self._from_json_stmts(f, fld, ada_fname, wire)
        f.write(f'      return Result;\n')
        f.write(f'   exception\n')
        f.write(f'      when others => return Result;\n')
        f.write(f'   end From_Json;\n\n')

    def _write_complex_codec(self, f, ada_n: str, msg, fields) -> None:
        """Generate To_Json / From_Json for records with nested messages,
        repeated fields, oneofs and aliases."""
        ref = self._type_ref(ada_n)

        # Collect oneof groups from msg.oneofs (they are separate from fields)
        oneof_groups: dict = {}  # group_name -> [(fld, fname)]
        for oo in msg.oneofs:
            for fld in oo.fields:
                oneof_groups.setdefault(oo.name, []).append((fld, fld.name))
        # Regular fields -- filter out any that happen to have oneof_group set
        regular_fields = []
        for fld, fname in fields:
            if fld.oneof_group:
                oneof_groups.setdefault(fld.oneof_group, []).append((fld, fname))
            else:
                regular_fields.append((fld, fname))

        # ---- To_Json ----
        f.write(f'   function To_Json (Msg : {ref}) return String is\n')
        f.write(f'      Result : Unbounded_String := To_Unbounded_String ("{{"')
        f.write(f');\n')
        f.write(f'      First  : Boolean := True;\n')
        f.write(f'      procedure Comma is\n')
        f.write(f'      begin\n')
        f.write(f'         if First then First := False;\n')
        f.write(f'         else Append (Result, ","); end if;\n')
        f.write(f'      end Comma;\n')
        f.write(f'   begin\n')

        for fld, fname in regular_fields:
            wire = camel_to_snake(fname)
            ada_type, _ = self._ada_field_type(fld.type, fld.is_repeated, fname)
            ada_fname = _ada_field_name(fname, ada_type)
            self._emit_to_json_field(f, fld, ada_fname, wire)

        # Oneofs
        for group, variants in oneof_groups.items():
            for fld, fname in variants:
                wire = camel_to_snake(fname)
                oo_short = fld.type.split('.')[-1]
                oo_ada_type = self._ada_base_type(fld.type)
                ada_fname = _ada_field_name(fname, oo_ada_type)
                has_flag = 'Has_' + ada_fname
                # Only emit the variant that is set
                f.write(f'      if Msg.{has_flag} then\n')
                f.write(f'         Comma;\n')
                if self._is_field_message(fld) and oo_short not in self._aliases:
                    qcall = self._qualified_to_json(fld, f'Msg.{ada_fname}')
                else:
                    qcall = self._to_json_expr(fld, ada_fname)
                f.write(f'         Append (Result, """{wire}"":" & {qcall});\n')
                f.write(f'      end if;\n')

        f.write(f'      Append (Result, "}}");\n')
        f.write(f'      return To_String (Result);\n')
        f.write(f'   end To_Json;\n\n')

        # ---- From_Json ----
        f.write(f'   function From_Json'
                f' (S : String; Tag : access {ref})'
                f' return {ref} is\n')
        f.write(f'      pragma Unreferenced (Tag);\n')
        f.write(f'      J      : constant JSON_Value := Read (S);\n')
        f.write(f'      Result : {ref};\n')
        f.write(f'   begin\n')

        for fld, fname in regular_fields:
            wire = camel_to_snake(fname)
            ada_type, _ = self._ada_field_type(fld.type, fld.is_repeated, fname)
            ada_fname = _ada_field_name(fname, ada_type)
            self._emit_from_json_field(f, fld, ada_fname, wire)

        for group, variants in oneof_groups.items():
            for fld, fname in variants:
                wire = camel_to_snake(fname)
                oo_short = fld.type.split('.')[-1]
                oo_ada_type = self._ada_base_type(fld.type)
                ada_fname = _ada_field_name(fname, oo_ada_type)
                has_flag = 'Has_' + ada_fname
                short = fld.type.split('.')[-1]
                f.write(f'      if Has_Field (J, "{wire}") then\n')
                f.write(f'         Result.{has_flag} := True;\n')
                if self._is_field_message(fld) and short not in self._aliases:
                    qpkg = self._field_codec_pkg(fld, self._msg_to_codec)
                    qual = f'{qpkg}.' if qpkg != self._ada_pkg else ''
                    f.write(f'         declare\n')
                    f.write(f'            Sub : constant String := Write (Get (J, "{wire}"));\n')
                    f.write(f'         begin\n')
                    f.write(f'            Result.{ada_fname} := {qual}From_Json (Sub, null);\n')
                    f.write(f'         end;\n')
                else:
                    self._from_json_stmts(f, fld, ada_fname, wire,
                                          indent='         ')
                f.write(f'      end if;\n')

        f.write(f'      return Result;\n')
        f.write(f'   exception\n')
        f.write(f'      when others => return Result;\n')
        f.write(f'   end From_Json;\n\n')

    def _emit_to_json_field(self, f, fld, ada_fname: str, wire: str) -> None:
        """Emit To_Json append statements for a single field."""
        short = fld.type.split('.')[-1]

        if fld.is_repeated:
            # Repeated field -- emit JSON array
            elem_is_scalar = (fld.type in _ADA_SCALAR_MAP or
                              short in self._aliases)
            elem_is_enum = (self._index.is_enum_type(fld.type) or
                            self._index.is_enum_type(short))
            f.write(f'      if Msg.{ada_fname} /= null then\n')
            f.write(f'         Comma;\n')
            f.write(f'         Append (Result, """{wire}"":[");\n')
            f.write(f'         for I in Msg.{ada_fname}\'Range loop\n')
            f.write(f'            if I > Msg.{ada_fname}\'First then\n')
            f.write(f'               Append (Result, ",");\n')
            f.write(f'            end if;\n')
            if elem_is_enum:
                expr = self._enum_to_json_expr(fld, f'Msg.{ada_fname} (I)')
                f.write(f'            Append (Result, {expr});\n')
            elif elem_is_scalar:
                ada_target = self._aliases.get(short, '')
                if fld.type == 'string' or ada_target == 'Unbounded_String':
                    expr = self._string_to_json_expr(f'Msg.{ada_fname} (I)')
                    f.write(f'            Append (Result, {expr});\n')
                elif fld.type == 'bool' or ada_target == 'Boolean':
                    f.write(f'            Append (Result, (if Msg.{ada_fname} (I) then "true" else "false"));\n')
                elif ada_target in ('Integer', 'Long_Integer', 'Natural'):
                    f.write(f'            Append (Result, {ada_target}\'Image (Msg.{ada_fname} (I)));\n')
                elif fld.type in ('float', 'double') or short in self._aliases:
                    f.write(f'            Append (Result, Long_Float\'Image (Msg.{ada_fname} (I)));\n')
                else:
                    f.write(f'            Append (Result, Integer\'Image (Msg.{ada_fname} (I)));\n')
            else:
                # Nested message element -- qualify to avoid ambiguity
                qcall = self._qualified_to_json(fld, f'Msg.{ada_fname} (I)')
                f.write(f'            Append (Result, {qcall});\n')
            f.write(f'         end loop;\n')
            f.write(f'         Append (Result, "]");\n')
            f.write(f'      end if;\n')
        elif self._is_field_message(fld) and short not in self._aliases:
            # Nested message -- delegate with qualified call
            qcall = self._qualified_to_json(fld, f'Msg.{ada_fname}')
            f.write(f'      Comma;\n')
            f.write(f'      Append (Result, """{wire}"":" & {qcall});\n')
        else:
            # Scalar / enum / alias
            expr = self._to_json_expr(fld, ada_fname)
            f.write(f'      Comma;\n')
            f.write(f'      Append (Result, """{wire}"":" & {expr});\n')

    def _emit_from_json_field(self, f, fld, ada_fname: str, wire: str) -> None:
        """Emit From_Json parse statements for a single field."""
        short = fld.type.split('.')[-1]

        if fld.is_repeated:
            # Repeated field -- iterate JSON array, allocate Ada array
            arr_type = _ada_array_name_for_repeated(fld.type, fld.name)
            elem_is_msg = (self._is_field_message(fld) and
                           short not in self._aliases)
            elem_is_enum = (self._index.is_enum_type(fld.type) or
                            self._index.is_enum_type(short))
            ada_target = self._aliases.get(short, '')

            f.write(f'      if Has_Field (J, "{wire}") then\n')
            f.write(f'         declare\n')
            f.write(f'            Arr_Val : constant JSON_Value := Get (J, "{wire}");\n')
            f.write(f'            Arr : constant JSON_Array := Get (Arr_Val);\n')
            f.write(f'            Len : constant Natural := Length (Arr);\n')
            f.write(f'         begin\n')
            f.write(f'            if Len > 0 then\n')
            f.write(f'               Result.{ada_fname} := new {arr_type} (1 .. Len);\n')
            f.write(f'               for I in 1 .. Len loop\n')
            if elem_is_msg:
                qpkg = self._field_codec_pkg(fld, self._msg_to_codec)
                qual = f'{qpkg}.' if qpkg != self._ada_pkg else ''
                f.write(f'                  declare\n')
                f.write(f'                     Elem : constant JSON_Value := Get (Arr, I);\n')
                f.write(f'                     Sub : constant String := Write (Elem);\n')
                f.write(f'                  begin\n')
                f.write(f'                     Result.{ada_fname} (I) := {qual}From_Json (Sub, null);\n')
                f.write(f'                  end;\n')
            elif elem_is_enum:
                enum_ada = _ada_name(short)
                enum_pkg = self._enum_to_codec.get(short, self._ada_pkg)
                qual = f'{enum_pkg}.' if enum_pkg != self._ada_pkg else ''
                f.write(f'                  declare\n')
                f.write(f'                     Elem : constant JSON_Value := Get (Arr, I);\n')
                f.write(f'                     Str : constant String := Get (Elem);\n')
                f.write(f'                  begin\n')
                f.write(f'                     Result.{ada_fname} (I) := {qual}{enum_ada}_From_String (Str);\n')
                f.write(f'                  end;\n')
            elif fld.type == 'string' or ada_target == 'Unbounded_String':
                f.write(f'                  declare\n')
                f.write(f'                     Elem : constant JSON_Value := Get (Arr, I);\n')
                f.write(f'                     Str : constant String := Get (Elem);\n')
                f.write(f'                  begin\n')
                f.write(f'                     Result.{ada_fname} (I) := To_Unbounded_String (Str);\n')
                f.write(f'                  end;\n')
            elif fld.type == 'bool' or ada_target == 'Boolean':
                f.write(f'                  declare\n')
                f.write(f'                     Elem : constant JSON_Value := Get (Arr, I);\n')
                f.write(f'                  begin\n')
                f.write(f'                     Result.{ada_fname} (I) := Get (Elem);\n')
                f.write(f'                  end;\n')
            elif ada_target in ('Integer', 'Long_Integer', 'Natural'):
                f.write(f'                  declare\n')
                f.write(f'                     Elem : constant JSON_Value := Get (Arr, I);\n')
                f.write(f'                  begin\n')
                f.write(f'                     Result.{ada_fname} (I) := {ada_target} (Get_Long_Float (Elem));\n')
                f.write(f'                  end;\n')
            else:
                # Default to float
                f.write(f'                  declare\n')
                f.write(f'                     Elem : constant JSON_Value := Get (Arr, I);\n')
                f.write(f'                  begin\n')
                f.write(f'                     Result.{ada_fname} (I) := Get_Long_Float (Elem);\n')
                f.write(f'                  end;\n')
            f.write(f'               end loop;\n')
            f.write(f'            end if;\n')
            f.write(f'         end;\n')
            f.write(f'      end if;\n')
        elif self._is_field_message(fld) and short not in self._aliases:
            # Nested message -- extract sub-object, serialise to string, call From_Json
            qpkg = self._field_codec_pkg(fld, self._msg_to_codec)
            qual = f'{qpkg}.' if qpkg != self._ada_pkg else ''
            f.write(f'      if Has_Field (J, "{wire}") then\n')
            f.write(f'         declare\n')
            f.write(f'            Sub : constant String := Write (Get (J, "{wire}"));\n')
            f.write(f'         begin\n')
            f.write(f'            Result.{ada_fname} := {qual}From_Json (Sub, null);\n')
            f.write(f'         end;\n')
            f.write(f'      end if;\n')
        else:
            self._from_json_stmts(f, fld, ada_fname, wire)

    # ------------------------------------------------------------------ body

    def _write_body(self, path: Path) -> None:
        alias_names = set(self._aliases.keys())
        structs = [m for m in self._pf.messages if m.name not in alias_names]

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated data model JSON codec body\n')
            f.write(f'--  Package: {self._ada_pkg}\n\n')
            f.write('with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;\n')
            f.write('with GNATCOLL.JSON;  use GNATCOLL.JSON;\n')
            # Import foreign codec packages for cross-file To_Json calls
            for dep in self._foreign_codec_deps():
                f.write(f'with {dep};\n')
            f.write('pragma Warnings (Off);\n\n')
            f.write(f'package body {self._ada_pkg} is\n\n')

            # Enum converters
            for enum in self._pf.enums:
                ada_n = _ada_name(enum.name)
                ref = self._type_ref(ada_n)
                prefix = ada_n.split('_')[-1] + '_'

                f.write(f'   function To_String (V : {ref}) return String is\n')
                f.write(f'   begin\n')
                f.write(f'      case V is\n')
                for v in enum.values:
                    suf = enum.suffix_of(v.name)
                    lit = screaming_to_pascal(suf) if suf else v.name
                    f.write(f'         when {prefix}{lit} => return "{v.name}";\n')
                f.write(f'      end case;\n')
                f.write(f'   end To_String;\n\n')

                f.write(f'   function {ada_n}_From_String (S : String) return {ref} is\n')
                f.write(f'   begin\n')
                for v in enum.values:
                    suf = enum.suffix_of(v.name)
                    lit = screaming_to_pascal(suf) if suf else v.name
                    f.write(f'      if S = "{v.name}" then return {prefix}{lit}; end if;\n')
                first_suf = enum.suffix_of(enum.values[0].name)
                first_lit = (screaming_to_pascal(first_suf)
                             if first_suf else enum.values[0].name)
                f.write(f'      return {prefix}{first_lit};\n')
                f.write(f'   end {ada_n}_From_String;\n\n')

            # Struct codecs -- generate working serialisation for all types
            for msg in structs:
                ada_n = _ada_name(msg.name)
                self._write_codec(f, ada_n, msg)

            f.write(f'end {self._ada_pkg};\n')


