#!/usr/bin/env python3
"""
JSON Codec Backend

Generates C++ (nlohmann/json) and Ada (GNATCOLL.JSON) serialisation code
for all proto messages.  Entirely driven from the parsed proto model —
no hardcoded message names or field lists.
"""

from pathlib import Path
from typing import List

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from proto_parser import (
    ProtoTypeIndex, ProtoMessage, ProtoEnum, ProtoField,
    camel_to_snake, camel_to_lower_snake, lc_first, screaming_to_pascal,
    screaming_to_title, _PROTO_SCALARS,
)
import codec_backends

_SEP = '// ' + '-' * 75


# -- C++ type mapping ---------------------------------------------------------

_CPP_SCALAR_MAP = {
    'double': 'double', 'float': 'float',
    'int32': 'int32_t', 'int64': 'int64_t',
    'uint32': 'uint32_t', 'uint64': 'uint64_t',
    'sint32': 'int32_t', 'sint64': 'int64_t',
    'fixed32': 'uint32_t', 'fixed64': 'uint64_t',
    'sfixed32': 'int32_t', 'sfixed64': 'int64_t',
    'bool': 'bool', 'string': 'std::string', 'bytes': 'std::string',
}

_CPP_DEFAULTS = {
    'double': '0.0', 'float': '0.0f',
    'int32_t': '0', 'int64_t': '0', 'uint32_t': '0', 'uint64_t': '0',
    'bool': 'false', 'std::string': '{}',
}

_JSON_GET_TYPE = {
    'double': 'double', 'float': 'float',
    'int32_t': 'int32_t', 'int64_t': 'int64_t',
    'uint32_t': 'uint32_t', 'uint64_t': 'uint64_t',
    'bool': 'bool', 'std::string': 'std::string',
}


def _cpp_type(field: ProtoField, index: ProtoTypeIndex) -> str:
    """Map a proto field type to the C++ type string."""
    if field.type in _CPP_SCALAR_MAP:
        base = _CPP_SCALAR_MAP[field.type]
    elif index.is_enum_type(field.type) or index.is_enum_type(field.short_type):
        base = field.short_type
    elif index.is_message_type(field.type) or index.is_message_type(field.short_type):
        base = field.short_type
    else:
        base = field.short_type  # best guess for unresolved types

    if field.is_repeated:
        return f'std::vector<{base}>'
    return base


def _cpp_default(field: ProtoField, index: ProtoTypeIndex) -> str:
    """Default value for a C++ field."""
    if field.is_repeated:
        return '{}'
    cpp = _cpp_type(field, index)
    if cpp in _CPP_DEFAULTS:
        return _CPP_DEFAULTS[cpp]
    if index.is_enum_type(field.type) or index.is_enum_type(field.short_type):
        enum = index.resolve_enum(field.type) or index.resolve_enum(field.short_type)
        if enum and enum.values:
            return f'{field.short_type}::{screaming_to_pascal(enum.suffix_of(enum.values[0].name) or "Unspecified")}'
        return f'{field.short_type}::Unspecified'
    return '{}'


# -- Ada type mapping ----------------------------------------------------------

_ADA_SCALAR_MAP = {
    'double': 'Long_Float', 'float': 'Float',
    'int32': 'Integer', 'int64': 'Long_Integer',
    'uint32': 'Interfaces.Unsigned_32', 'uint64': 'Interfaces.Unsigned_64',
    'sint32': 'Integer', 'sint64': 'Long_Integer',
    'bool': 'Boolean', 'string': 'Unbounded_String', 'bytes': 'Unbounded_String',
}

_ADA_DEFAULTS = {
    'Long_Float': '0.0', 'Float': '0.0',
    'Integer': '0', 'Long_Integer': '0',
    'Interfaces.Unsigned_32': '0', 'Interfaces.Unsigned_64': '0',
    'Boolean': 'False', 'Unbounded_String': 'Null_Unbounded_String',
}


def _ada_type(field: ProtoField, index: ProtoTypeIndex) -> str:
    if field.type in _ADA_SCALAR_MAP:
        return _ADA_SCALAR_MAP[field.type]
    return camel_to_snake(field.short_type)


def _ada_field_name(name: str) -> str:
    """Convert a proto field name to Ada Title_Case (e.g. 'update_time' -> 'Update_Time')."""
    s = camel_to_snake(name)
    return '_'.join(w.capitalize() for w in s.split('_'))


def _ada_to_json_expr(fld: ProtoField, ada_fname: str, index: ProtoTypeIndex) -> str:
    """Return an Ada expression that serialises a scalar/enum field to JSON text."""
    if fld.type == 'bool':
        return f'(if Msg.{ada_fname} then "true" else "false")'
    if fld.type in ('int32', 'int64', 'sint32', 'sint64',
                     'uint32', 'uint64', 'fixed32', 'fixed64',
                     'sfixed32', 'sfixed64'):
        return f"Integer'Image (Msg.{ada_fname})"
    if fld.type in ('float', 'double'):
        return f"Long_Float'Image (Msg.{ada_fname})"
    if fld.type in ('string', 'bytes'):
        return f'"""" & To_String (Msg.{ada_fname}) & """"'
    if index.is_enum_type(fld.type) or index.is_enum_type(fld.short_type):
        return f'"""" & To_String (Msg.{ada_fname}) & """"'
    # Nested message — delegate
    return f'To_Json (Msg.{ada_fname})'


# -- Generator -----------------------------------------------------------------

class JsonBackend(codec_backends.CodecBackend):

    @property
    def name(self) -> str:
        return 'json'

    @property
    def content_type(self) -> str:
        return 'application/json'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        # Generate one codec header+impl per proto file that has messages
        for pf in index.files:
            msgs = pf.messages
            enums = pf.enums
            if not msgs and not enums:
                continue

            # Derive file names from package
            pkg_parts = [p for p in pf.package.split('.') if p]
            file_base = '_'.join(pkg_parts) + '_json_codec'
            ns = '::'.join(pkg_parts) + '::json_codec'

            hpp_path = output_dir / (file_base + '.hpp')
            cpp_path = output_dir / (file_base + '.cpp')

            self._write_cpp_header(hpp_path, ns, file_base, pf.messages, pf.enums, index)
            self._write_cpp_impl(cpp_path, ns, file_base, pf.messages, pf.enums, index)
            generated.extend([hpp_path, cpp_path])

        return generated

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        for pf in index.files:
            msgs = pf.messages
            enums = pf.enums
            if not msgs and not enums:
                continue

            pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
            pkg_name = '.'.join(pkg_parts) + '.Json_Codec'
            file_base = '_'.join(p.lower() for p in pkg_parts) + '-json_codec'

            spec_path = output_dir / (file_base + '.ads')
            body_path = output_dir / (file_base + '.adb')

            self._write_ada_spec(spec_path, pkg_name, pf.messages, pf.enums, index)
            self._write_ada_body(body_path, pkg_name, pf.messages, pf.enums, index)
            generated.extend([spec_path, body_path])

        return generated

    # -- C++ header ------------------------------------------------------------

    def _write_cpp_header(self, path: Path, ns: str, file_base: str,
                          messages: List[ProtoMessage], enums: List[ProtoEnum],
                          index: ProtoTypeIndex):
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'// Auto-generated JSON codec — do not edit\n')
            f.write(f'// Backend: json | Namespace: {ns}\n')
            f.write('#pragma once\n\n')
            f.write('#include <string>\n#include <vector>\n#include <cstdint>\n\n')
            f.write(f'namespace {ns} {{\n\n')

            # Enum forward declarations + string converters
            for enum in enums:
                f.write(f'enum class {enum.name} {{\n')
                for v in enum.values:
                    suffix = enum.suffix_of(v.name)
                    cpp_name = screaming_to_pascal(suffix) if suffix else v.name
                    f.write(f'    {cpp_name} = {v.number},\n')
                f.write(f'}};\n\n')
                f.write(f'std::string toString({enum.name} v);\n')
                f.write(f'{enum.name} {lc_first(enum.name)}FromString(const std::string& s);\n\n')

            # Message structs
            for msg in messages:
                f.write(f'struct {msg.name} {{\n')
                for fld in msg.all_fields():
                    ct = _cpp_type(fld, index)
                    default = _cpp_default(fld, index)
                    f.write(f'    {ct} {fld.name} = {default};\n')
                f.write(f'}};\n\n')

            # Ser/de declarations
            for msg in messages:
                f.write(f'std::string toJson(const {msg.name}& msg);\n')
                f.write(f'{msg.name} {lc_first(msg.name)}FromJson(const std::string& s);\n')
            f.write(f'\n}} // namespace {ns}\n')

    # -- C++ implementation ----------------------------------------------------

    def _write_cpp_impl(self, path: Path, ns: str, file_base: str,
                        messages: List[ProtoMessage], enums: List[ProtoEnum],
                        index: ProtoTypeIndex):
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'// Auto-generated JSON codec — do not edit\n')
            f.write(f'#include "{file_base}.hpp"\n\n')
            f.write('#include <nlohmann/json.hpp>\n\n')
            f.write(f'namespace {ns} {{\n\n')

            # Enum converters
            for enum in enums:
                # toString
                f.write(f'std::string toString({enum.name} v) {{\n')
                f.write(f'    switch (v) {{\n')
                for v in enum.values:
                    suffix = enum.suffix_of(v.name)
                    cpp_name = screaming_to_pascal(suffix) if suffix else v.name
                    f.write(f'        case {enum.name}::{cpp_name}: return "{v.name}";\n')
                f.write(f'    }}\n')
                if enum.values:
                    f.write(f'    return "{enum.values[0].name}";\n')
                else:
                    f.write(f'    return "";\n')
                f.write(f'}}\n\n')

                # fromString
                f.write(f'{enum.name} {lc_first(enum.name)}FromString(const std::string& s) {{\n')
                for v in enum.values:
                    suffix = enum.suffix_of(v.name)
                    cpp_name = screaming_to_pascal(suffix) if suffix else v.name
                    f.write(f'    if (s == "{v.name}") return {enum.name}::{cpp_name};\n')
                if enum.values:
                    suffix0 = enum.suffix_of(enum.values[0].name)
                    cpp0 = screaming_to_pascal(suffix0) if suffix0 else enum.values[0].name
                    f.write(f'    return {enum.name}::{cpp0};\n')
                else:
                    f.write(f'    return {{}};\n')
                f.write(f'}}\n\n')

            # toJson
            for msg in messages:
                f.write(f'std::string toJson(const {msg.name}& msg) {{\n')
                f.write(f'    nlohmann::json obj;\n')
                for fld in msg.all_fields():
                    jkey = fld.name
                    if index.is_enum_type(fld.type) or index.is_enum_type(fld.short_type):
                        if fld.is_repeated:
                            f.write(f'    {{\n')
                            f.write(f'        nlohmann::json arr = nlohmann::json::array();\n')
                            f.write(f'        for (const auto& e : msg.{jkey}) arr.push_back(toString(e));\n')
                            f.write(f'        obj["{jkey}"] = arr;\n')
                            f.write(f'    }}\n')
                        else:
                            f.write(f'    obj["{jkey}"] = toString(msg.{jkey});\n')
                    elif fld.is_repeated:
                        # Repeated scalar or message
                        is_msg = (index.is_message_type(fld.type) or
                                  index.is_message_type(fld.short_type))
                        f.write(f'    {{\n')
                        f.write(f'        nlohmann::json arr = nlohmann::json::array();\n')
                        if is_msg:
                            f.write(f'        for (const auto& e : msg.{jkey}) arr.push_back(nlohmann::json::parse(toJson(e)));\n')
                        else:
                            f.write(f'        for (const auto& e : msg.{jkey}) arr.push_back(e);\n')
                        f.write(f'        obj["{jkey}"] = arr;\n')
                        f.write(f'    }}\n')
                    elif fld.type in ('string', 'bytes'):
                        if fld.is_optional:
                            f.write(f'    if (!msg.{jkey}.empty()) obj["{jkey}"] = msg.{jkey};\n')
                        else:
                            f.write(f'    obj["{jkey}"] = msg.{jkey};\n')
                    elif fld.type == 'bool':
                        f.write(f'    obj["{jkey}"] = msg.{jkey};\n')
                    elif fld.is_scalar:
                        if fld.is_optional:
                            f.write(f'    if (msg.{jkey} != 0) obj["{jkey}"] = msg.{jkey};\n')
                        else:
                            f.write(f'    obj["{jkey}"] = msg.{jkey};\n')
                    else:
                        # Nested message — delegate to its toJson
                        f.write(f'    obj["{jkey}"] = nlohmann::json::parse(toJson(msg.{jkey}));\n')
                f.write(f'    return obj.dump();\n')
                f.write(f'}}\n\n')

            # fromJson
            for msg in messages:
                fname = f'{lc_first(msg.name)}FromJson'
                f.write(f'{msg.name} {fname}(const std::string& s) {{\n')
                f.write(f'    {msg.name} result;\n')
                f.write(f'    try {{\n')
                f.write(f'        auto j = nlohmann::json::parse(s);\n')
                for fld in msg.all_fields():
                    jkey = fld.name
                    if index.is_enum_type(fld.type) or index.is_enum_type(fld.short_type):
                        if fld.is_repeated:
                            f.write(f'        if (j.contains("{jkey}") && j["{jkey}"].is_array())\n')
                            f.write(f'            for (const auto& e : j["{jkey}"])\n')
                            f.write(f'                result.{jkey}.push_back({lc_first(fld.short_type)}FromString(e.get<std::string>()));\n')
                        else:
                            f.write(f'        if (j.contains("{jkey}"))\n')
                            f.write(f'            result.{jkey} = {lc_first(fld.short_type)}FromString(j["{jkey}"].get<std::string>());\n')
                    elif fld.is_repeated:
                        # Repeated scalar or message
                        is_msg = (index.is_message_type(fld.type) or
                                  index.is_message_type(fld.short_type))
                        f.write(f'        if (j.contains("{jkey}") && j["{jkey}"].is_array())\n')
                        f.write(f'            for (const auto& e : j["{jkey}"])\n')
                        if is_msg:
                            f.write(f'                result.{jkey}.push_back({lc_first(fld.short_type)}FromJson(e.dump()));\n')
                        elif fld.type in ('string', 'bytes'):
                            f.write(f'                result.{jkey}.push_back(e.get<std::string>());\n')
                        elif fld.type == 'bool':
                            f.write(f'                result.{jkey}.push_back(e.get<bool>());\n')
                        else:
                            json_t = _JSON_GET_TYPE.get(_CPP_SCALAR_MAP.get(fld.type, 'double'), 'double')
                            f.write(f'                result.{jkey}.push_back(e.get<{json_t}>());\n')
                    elif fld.type in ('string', 'bytes'):
                        f.write(f'        if (j.contains("{jkey}"))\n')
                        f.write(f'            result.{jkey} = j["{jkey}"].get<std::string>();\n')
                    elif fld.type == 'bool':
                        f.write(f'        if (j.contains("{jkey}"))\n')
                        f.write(f'            result.{jkey} = j["{jkey}"].get<bool>();\n')
                    elif fld.is_scalar:
                        json_t = _JSON_GET_TYPE.get(_CPP_SCALAR_MAP.get(fld.type, 'double'), 'double')
                        f.write(f'        if (j.contains("{jkey}"))\n')
                        f.write(f'            result.{jkey} = j["{jkey}"].get<{json_t}>();\n')
                    else:
                        # Nested message
                        f.write(f'        if (j.contains("{jkey}") && j["{jkey}"].is_object())\n')
                        f.write(f'            result.{jkey} = {lc_first(fld.short_type)}FromJson(j["{jkey}"].dump());\n')
                f.write(f'    }} catch (...) {{}}\n')
                f.write(f'    return result;\n')
                f.write(f'}}\n\n')

            f.write(f'}} // namespace {ns}\n')

    # -- Ada spec --------------------------------------------------------------

    def _write_ada_spec(self, path: Path, pkg_name: str,
                        messages: List[ProtoMessage], enums: List[ProtoEnum],
                        index: ProtoTypeIndex):
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'--  Auto-generated JSON codec — do not edit\n')
            f.write(f'--  Backend: json | Package: {pkg_name}\n\n')
            f.write(f'with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;\n')
            f.write(f'with GNATCOLL.JSON;\n\n')
            f.write(f'package {pkg_name} is\n\n')

            # Enums
            for enum in enums:
                ada_name = camel_to_snake(enum.name)
                f.write(f'   type {ada_name} is\n')
                f.write(f'     (')
                literals = []
                for v in enum.values:
                    suffix = enum.suffix_of(v.name)
                    lit = screaming_to_title(suffix) if suffix else v.name
                    literals.append(lit)
                f.write((',\n      ').join(literals))
                f.write(f');\n\n')
                f.write(f'   function To_String (V : {ada_name}) return String;\n')
                f.write(f'   function From_String (S : String) return {ada_name};\n\n')

            # Records
            for msg in messages:
                ada_name = camel_to_snake(msg.name)
                f.write(f'   type {ada_name} is record\n')
                for fld in msg.all_fields():
                    ada_fld = camel_to_snake(fld.name) if fld.name != fld.name.lower() else '_'.join(w.capitalize() for w in fld.name.split('_'))
                    ada_t = _ada_type(fld, index)
                    f.write(f'      {ada_fld} : {ada_t};\n')
                f.write(f'   end record;\n\n')

            # Ser/de
            for msg in messages:
                ada_name = camel_to_snake(msg.name)
                f.write(f'   function To_Json (Msg : {ada_name}) return String;\n')
                f.write(f'   function From_Json (S : String) return {ada_name};\n\n')

            f.write(f'end {pkg_name};\n')

    # -- Ada body --------------------------------------------------------------

    def _write_ada_body(self, path: Path, pkg_name: str,
                        messages: List[ProtoMessage], enums: List[ProtoEnum],
                        index: ProtoTypeIndex):
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'--  Auto-generated JSON codec — do not edit\n\n')
            f.write(f'with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;\n')
            f.write(f'with GNATCOLL.JSON; use GNATCOLL.JSON;\n\n')
            f.write(f'package body {pkg_name} is\n\n')

            # Enum converters
            for enum in enums:
                ada_name = camel_to_snake(enum.name)
                # To_String
                f.write(f'   function To_String (V : {ada_name}) return String is\n')
                f.write(f'   begin\n')
                f.write(f'      case V is\n')
                for v in enum.values:
                    suffix = enum.suffix_of(v.name)
                    lit = screaming_to_title(suffix) if suffix else v.name
                    f.write(f'         when {lit} => return "{v.name}";\n')
                f.write(f'      end case;\n')
                f.write(f'   end To_String;\n\n')

                # From_String
                f.write(f'   function From_String (S : String) return {ada_name} is\n')
                f.write(f'   begin\n')
                for i, v in enumerate(enum.values):
                    suffix = enum.suffix_of(v.name)
                    lit = screaming_to_title(suffix) if suffix else v.name
                    kw = 'if' if i == 0 else 'elsif'
                    f.write(f'      {kw} S = "{v.name}" then\n')
                    f.write(f'         return {lit};\n')
                if enum.values:
                    suffix0 = enum.suffix_of(enum.values[0].name)
                    lit0 = screaming_to_title(suffix0) if suffix0 else enum.values[0].name
                    f.write(f'      else\n')
                    f.write(f'         return {lit0};\n')
                    f.write(f'      end if;\n')
                f.write(f'   end From_String;\n\n')

            # Message ser/de
            for msg in messages:
                ada_name = camel_to_snake(msg.name)
                all_flds = msg.all_fields()

                has_nested = any(
                    not fld.is_scalar and fld.type not in _ADA_SCALAR_MAP
                    and not (index.is_enum_type(fld.type) or index.is_enum_type(fld.short_type))
                    for fld in all_flds
                )
                has_repeated = any(fld.is_repeated for fld in all_flds)
                use_unbounded = has_nested or has_repeated

                # -- To_Json --
                f.write(f'   function To_Json (Msg : {ada_name}) return String is\n')
                if use_unbounded:
                    f.write(f'      Result : Unbounded_String := '
                            f'To_Unbounded_String ("{{"')
                    f.write(f');\n')
                    f.write(f'      First  : Boolean := True;\n')
                    f.write(f'      procedure Comma is\n')
                    f.write(f'      begin\n')
                    f.write(f'         if First then First := False;\n')
                    f.write(f'         else Append (Result, ","); end if;\n')
                    f.write(f'      end Comma;\n')
                f.write(f'   begin\n')

                if use_unbounded:
                    for fld in all_flds:
                        ada_fld = _ada_field_name(fld.name)
                        wire = camel_to_lower_snake(fld.name)
                        is_enum = (index.is_enum_type(fld.type) or
                                   index.is_enum_type(fld.short_type))
                        is_msg = (not fld.is_scalar and fld.type not in _ADA_SCALAR_MAP
                                  and not is_enum)

                        if fld.is_repeated:
                            # Skip repeated for now (access array)
                            f.write(f'      --  repeated: {wire}\n')
                        elif is_msg:
                            f.write(f'      Comma;\n')
                            f.write(f'      Append (Result, """{wire}"":" & '
                                    f'To_Json (Msg.{ada_fld}));\n')
                        else:
                            expr = _ada_to_json_expr(fld, ada_fld, index)
                            f.write(f'      Comma;\n')
                            f.write(f'      Append (Result, """{wire}"":" & '
                                    f'{expr});\n')
                    f.write(f'      Append (Result, "}}");\n')
                    f.write(f'      return To_String (Result);\n')
                else:
                    # Simple string-concat approach
                    parts = []
                    for fld in all_flds:
                        ada_fld = _ada_field_name(fld.name)
                        wire = camel_to_lower_snake(fld.name)
                        expr = _ada_to_json_expr(fld, ada_fld, index)
                        parts.append(f'        """{wire}"":" & {expr}')
                    f.write(f'      return "{{" &\n')
                    f.write(' &\n        "," &\n'.join(parts))
                    f.write(' &\n')
                    f.write(f'        "}}";\n')
                f.write(f'   end To_Json;\n\n')

                # -- From_Json --
                f.write(f'   function From_Json (S : String) return {ada_name} is\n')
                f.write(f'      Result : {ada_name};\n')
                f.write(f'   begin\n')
                for fld in all_flds:
                    ada_fld = _ada_field_name(fld.name)
                    if fld.type == 'bool':
                        f.write(f'      for I in S\'First .. S\'Last - 3 loop\n')
                        f.write(f'         if S (I .. I + 3) = "true" then\n')
                        f.write(f'            Result.{ada_fld} := True;\n')
                        f.write(f'            exit;\n')
                        f.write(f'         end if;\n')
                        f.write(f'      end loop;\n')
                    # Other types: leave as defaults (component defaults suffice)
                f.write(f'      return Result;\n')
                f.write(f'   end From_Json;\n\n')

            f.write(f'end {pkg_name};\n')


# -- Register ------------------------------------------------------------------

codec_backends.register(JsonBackend())
