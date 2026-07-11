#!/usr/bin/env python3
"""xsd2proto -- convert a profiled subset of a UCI/A-GRA XSD drop to proto3.

Phase 1 of doc/plans/PYRAMID/uci_mms_conversion_plan.md.  Consumes a profile
manifest (pim/uci_profiles/*.json) naming a schema drop and the top-level
root messages to convert, resolves the transitive type closure, and emits:

  <out>/<drop>/pyramid/data_model/<proto_package>.proto   the converted tree
  <out>/<drop>/wire_names.json                            authoritative XSD
                                                          element names per
                                                          proto field (plan
                                                          D3: wire names are
                                                          data, not
                                                          derivation)
  <out>/<drop>/closure_report.json                        closure size and
                                                          fan-in, for
                                                          deliberate profile
                                                          pruning (plan D2)

Mapping rules follow the a-gra standard review §4.2 table (plan D5), with
deviations recorded here because they are converter policy, not schema
facts:

  * xs:dateTime / xs:date / xs:time / xs:duration map to ``string``, not
    google.protobuf well-known types -- the shipped OMS JSON codec and the
    hand-authored ``uci_seam_example`` contract already carry timestamps as
    strings, and the OMS wire form is a string; a well-known-type detour
    would buy nothing and cost every consumer a conversion.
  * Enum values are prefixed with the UPPER_SNAKE enum name (proto3 enum
    values share package scope; UCI literals collide across enums) and gain
    a ``*_UNSPECIFIED = 0`` sentinel.  The exact XSD literal for each value
    is preserved in wire_names.json.
  * simpleType restrictions without enumeration facets collapse to their
    base scalar; the facet (e.g. a UUID pattern) is retained as a comment.

The converter is deterministic: output depends only on the XSD bytes and
the profile manifest (document order drives emission order; headers embed
the source sha256, never a timestamp).  ``--check`` regenerates in memory
and fails if the checked-in output would change -- the plan D2 CI guard.

Strictness: unsupported XSD constructs (substitution groups, xs:any,
xs:list/xs:union, mixed content) are hard errors unless ``--lax`` is given,
in which case they are skipped and recorded in the closure report's
``skipped`` section.  The A-GRA dialect was measured conversion-friendly
(no substitution groups, regular MT/MDT discipline); anything outside that
should be looked at, not silently absorbed.

Stdlib only, matching the rest of pim/.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

PIM_DIR = Path(__file__).resolve().parent
XS = "{http://www.w3.org/2001/XMLSchema}"

# XSD builtin -> proto3 scalar.  A user-defined type of the same local name
# wins over a builtin (and is reported); the UCI/A-GRA dialect defines none.
XSD_BUILTINS: Dict[str, str] = {
    "string": "string", "normalizedString": "string", "token": "string",
    "Name": "string", "NCName": "string", "NMTOKEN": "string",
    "anyURI": "string", "ID": "string", "IDREF": "string",
    "language": "string", "QName": "string",
    "boolean": "bool",
    "decimal": "double", "double": "double",
    "float": "float",
    "byte": "int32", "short": "int32", "int": "int32",
    "long": "int64", "integer": "int64",
    "negativeInteger": "int64", "nonPositiveInteger": "int64",
    "unsignedByte": "uint32", "unsignedShort": "uint32",
    "unsignedInt": "uint32",
    "unsignedLong": "uint64", "nonNegativeInteger": "uint64",
    "positiveInteger": "uint64",
    # Converter policy deviation (see module docstring): temporal types are
    # strings on the OMS wire and stay strings in the contract.
    "dateTime": "string", "date": "string", "time": "string",
    "duration": "string", "gYear": "string", "gYearMonth": "string",
    "gMonthDay": "string", "gDay": "string", "gMonth": "string",
    "base64Binary": "bytes", "hexBinary": "bytes",
}


def snake_case(name: str) -> str:
    """PascalCase / acronym-run XSD name -> proto snake_case field name."""
    s = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    s = re.sub(r"[^0-9a-zA-Z]+", "_", s)
    return re.sub(r"_+", "_", s).strip("_").lower()


def upper_snake(name: str) -> str:
    return snake_case(name).upper()


def sanitize_type_name(name: str) -> str:
    """XSD type names are kept verbatim where valid proto identifiers."""
    out = re.sub(r"[^0-9a-zA-Z_]", "_", name)
    if out and out[0].isdigit():
        out = "_" + out
    return out


def local_name(qname: str) -> str:
    return qname.split(":")[-1]


class ConversionError(Exception):
    pass


@dataclass
class FieldSpec:
    proto_name: str
    proto_type: str        # scalar, message, or enum proto name
    wire_name: str         # exact XSD element/attribute name
    number: int
    label: str = ""        # "", "optional", "repeated"
    oneof: Optional[str] = None
    comment: List[str] = field(default_factory=list)
    is_attribute: bool = False
    # For repeated fields only: XSD minOccurs >= 1, i.e. an empty list is
    # schema-invalid.  proto3 'repeated' cannot express this, so it rides
    # the wire_names sidecar for the codec generators to enforce.
    required_repeated: bool = False


@dataclass
class MessageSpec:
    proto_name: str
    xsd_name: str          # source complexType name ("" if synthesized)
    fields: List[FieldSpec] = field(default_factory=list)
    oneofs: List[str] = field(default_factory=list)
    comment: List[str] = field(default_factory=list)
    synthesized: bool = False


@dataclass
class EnumSpec:
    proto_name: str
    xsd_name: str
    literals: List[str] = field(default_factory=list)
    comment: List[str] = field(default_factory=list)

    def value_name(self, literal: str) -> str:
        cleaned = re.sub(r"[^0-9a-zA-Z]+", "_", literal).strip("_").upper()
        if cleaned and cleaned[0].isdigit():
            cleaned = "N" + cleaned
        return f"{upper_snake(self.proto_name)}_{cleaned}"


def _doc_text(node: ET.Element) -> List[str]:
    """xs:annotation/xs:documentation -> normalized comment lines."""
    lines: List[str] = []
    for doc in node.findall(f"{XS}annotation/{XS}documentation"):
        text = " ".join((doc.text or "").split())
        while len(text) > 76:
            cut = text.rfind(" ", 0, 76)
            cut = cut if cut > 0 else 76
            lines.append(text[:cut])
            text = text[cut:].strip()
        if text:
            lines.append(text)
    return lines


class SchemaIndex:
    """All named XSD components across the drop's files, by local name."""

    def __init__(self, paths: List[Path], lax: bool):
        self.lax = lax
        self.sources: List[Tuple[str, str]] = []   # (basename, sha256)
        self.complex_types: Dict[str, ET.Element] = {}
        self.simple_types: Dict[str, ET.Element] = {}
        self.elements: Dict[str, ET.Element] = {}  # global elements
        self.doc_order: List[Tuple[str, str]] = [] # (kind, name)
        self.skipped: List[str] = []
        for path in paths:
            data = path.read_bytes()
            self.sources.append((path.name, hashlib.sha256(data).hexdigest()))
            try:
                root = ET.fromstring(data)
            except ET.ParseError as exc:
                raise ConversionError(f"{path}: XML parse error: {exc}")
            if root.tag != f"{XS}schema":
                raise ConversionError(f"{path}: root element is not xs:schema")
            for child in root:
                name = child.get("name", "")
                if child.tag == f"{XS}complexType" and name:
                    self._add(self.complex_types, "complexType", name, child)
                elif child.tag == f"{XS}simpleType" and name:
                    self._add(self.simple_types, "simpleType", name, child)
                elif child.tag == f"{XS}element" and name:
                    if child.get("substitutionGroup"):
                        self._unsupported(f"substitutionGroup on element {name}")
                    self.elements[name] = child
                elif child.tag in (f"{XS}import", f"{XS}include", f"{XS}annotation"):
                    continue  # all profile files are passed explicitly
                elif child.tag in (f"{XS}attributeGroup", f"{XS}group"):
                    self._unsupported(f"top-level {local_name(child.tag)} "
                                      f"'{name}'")

    def _add(self, table: Dict[str, ET.Element], kind: str, name: str,
             node: ET.Element) -> None:
        if name in table:
            raise ConversionError(f"duplicate {kind} '{name}' across schema "
                                  "files (local-name resolution assumes one "
                                  "logical namespace per drop)")
        if name in XSD_BUILTINS:
            self._unsupported(f"user {kind} '{name}' shadows an XSD builtin")
        table[name] = node
        self.doc_order.append((kind, name))

    def _unsupported(self, what: str) -> None:
        if self.lax:
            self.skipped.append(what)
        else:
            raise ConversionError(
                f"unsupported XSD construct: {what} (rerun with --lax to "
                "skip and record, but read the closure report afterwards)")


class Converter:
    def __init__(self, index: SchemaIndex, profile: dict):
        self.index = index
        self.profile = profile
        self.messages: Dict[str, MessageSpec] = {}
        self.enums: Dict[str, EnumSpec] = {}
        self.emit_order: List[Tuple[str, str]] = []  # (kind, proto name)
        self.roots: Dict[str, str] = {}              # element -> proto message
        self.fan_in: Dict[str, int] = {}
        self.scalar_aliases: Dict[str, Tuple[str, List[str]]] = {}

    # -- type resolution ---------------------------------------------------

    def resolve_type(self, qname: str, context: str) -> Tuple[str, str, List[str]]:
        """Resolve a type reference -> (kind, proto type, comment lines).

        kind is 'scalar', 'message', or 'enum'.
        """
        name = local_name(qname)
        if name in self.index.complex_types:
            return "message", self.convert_complex(name), []
        if name in self.index.simple_types:
            return self.resolve_simple(name)
        if name in XSD_BUILTINS:
            comment = []
            if name in ("dateTime", "date", "time", "duration"):
                comment = [f"xsd type: xs:{name} (carried as string on the OMS wire)"]
            return "scalar", XSD_BUILTINS[name], comment
        if name in ("anyType", "anySimpleType"):
            self.index._unsupported(f"xs:{name} at {context}")
            return "scalar", "string", [f"xsd type: xs:{name} (lax fallback)"]
        raise ConversionError(f"unresolved type reference '{qname}' at {context}")

    def resolve_simple(self, name: str) -> Tuple[str, str, List[str]]:
        node = self.index.simple_types[name]
        restriction = node.find(f"{XS}restriction")
        if restriction is None:
            self.index._unsupported(
                f"simpleType '{name}' without restriction (list/union)")
            return "scalar", "string", [f"xsd type: {name} (lax fallback)"]
        literals = [e.get("value", "")
                    for e in restriction.findall(f"{XS}enumeration")]
        if literals:
            return "enum", self.convert_enum(name, node, literals), []
        if name in self.scalar_aliases:
            proto, comment = self.scalar_aliases[name]
            return "scalar", proto, comment
        kind, proto, comment = self.resolve_type(
            restriction.get("base", ""), f"simpleType {name}")
        if kind != "scalar":
            raise ConversionError(
                f"simpleType '{name}' restricts non-scalar base")
        notes = [f"xsd type: {name}"]
        pattern = restriction.find(f"{XS}pattern")
        if pattern is not None:
            notes[0] += f" (pattern {pattern.get('value', '')})"
        self.scalar_aliases[name] = (proto, notes)
        return "scalar", proto, notes

    # -- enum conversion ---------------------------------------------------

    def convert_enum(self, name: str, node: ET.Element,
                     literals: List[str]) -> str:
        proto_name = sanitize_type_name(name)
        if proto_name not in self.enums:
            spec = EnumSpec(proto_name=proto_name, xsd_name=name,
                            literals=literals, comment=_doc_text(node))
            self.enums[proto_name] = spec
            self.emit_order.append(("enum", proto_name))
        return proto_name

    # -- message conversion ------------------------------------------------

    def convert_complex(self, name: str) -> str:
        proto_name = sanitize_type_name(name)
        if proto_name in self.messages:
            return proto_name
        node = self.index.complex_types[name]
        spec = MessageSpec(proto_name=proto_name, xsd_name=name,
                           comment=_doc_text(node))
        # Review §4.2: the per-type uci:version stamp (a namespaced attribute
        # on the declaration) is retained as a comment line.
        for key, value in node.attrib.items():
            if key.endswith("}version"):
                spec.comment.append(f"uci_version: {value}")
        # Register before descending so recursive/self-referential types
        # terminate; emit order records first-need order which is stable.
        self.messages[proto_name] = spec
        self.emit_order.append(("message", proto_name))
        self._fill_message(spec, node, name)
        return proto_name

    def _fill_message(self, spec: MessageSpec, node: ET.Element,
                      context: str) -> None:
        if node.get("mixed") == "true":
            self.index._unsupported(f"mixed content on {context}")
        number = 1
        body = node
        complex_content = node.find(f"{XS}complexContent")
        simple_content = node.find(f"{XS}simpleContent")
        if complex_content is not None:
            ext = complex_content.find(f"{XS}extension")
            if ext is None:
                self.index._unsupported(
                    f"complexContent restriction on {context}")
                return
            base_kind, base_type, _ = self.resolve_type(
                ext.get("base", ""), context)
            if base_kind != "message":
                raise ConversionError(
                    f"{context}: extension base is not a complexType")
            self._bump_fan_in(base_type)
            spec.fields.append(FieldSpec(
                proto_name="base", proto_type=base_type,
                wire_name="", number=number,
                comment=[f"composed xsd extension base {local_name(ext.get('base', ''))}"]))
            number += 1
            body = ext
        elif simple_content is not None:
            ext = simple_content.find(f"{XS}extension")
            if ext is None:
                self.index._unsupported(
                    f"simpleContent restriction on {context}")
                return
            kind, proto, comment = self.resolve_type(
                ext.get("base", ""), context)
            spec.fields.append(FieldSpec(
                proto_name="value", proto_type=proto, wire_name="",
                number=number, comment=comment
                + ["xsd simpleContent text value"]))
            number += 1
            body = ext

        number = self._convert_particles(spec, body, number, context)
        self._convert_attributes(spec, body, number, context)
        self._dedupe_field_names(spec, context)

    def _convert_particles(self, spec: MessageSpec, parent: ET.Element,
                           number: int, context: str,
                           optional_group: bool = False) -> int:
        for child in parent:
            if child.tag == f"{XS}sequence":
                if child.get("maxOccurs", "1") not in ("", "1"):
                    self.index._unsupported(f"repeated xs:sequence in {context}")
                    continue
                # An optional group (minOccurs="0") makes every child
                # optional on the wire even when the child itself defaults
                # to minOccurs="1" -- absence of the group is legal.
                number = self._convert_particles(
                    spec, child, number, context,
                    optional_group=optional_group
                    or child.get("minOccurs", "1") == "0")
            elif child.tag == f"{XS}element":
                number = self._convert_element(spec, child, number, context,
                                               oneof=None,
                                               optional_group=optional_group)
            elif child.tag == f"{XS}choice":
                number = self._convert_choice(spec, child, number, context)
            elif child.tag == f"{XS}any":
                self.index._unsupported(f"xs:any in {context}")
            elif child.tag in (f"{XS}annotation", f"{XS}attribute"):
                continue
            elif child.tag == f"{XS}group":
                self.index._unsupported(f"xs:group ref in {context}")
        return number

    def _convert_choice(self, spec: MessageSpec, choice: ET.Element,
                        number: int, context: str) -> int:
        repeated = choice.get("maxOccurs", "1") not in ("", "1")
        if repeated:
            # Repeated choice -> synthesized wrapper message holding the
            # oneof, referenced by a repeated field (proto3 forbids
            # `repeated oneof`).  Review §4.2 rule.
            wrapper_name = f"{spec.proto_name}_Choice"
            n = 2
            while wrapper_name in self.messages:
                wrapper_name = f"{spec.proto_name}_Choice_{n}"
                n += 1
            wrapper = MessageSpec(
                proto_name=wrapper_name, xsd_name="", synthesized=True,
                comment=[f"synthesized wrapper for a repeated xs:choice in {context}"])
            self.messages[wrapper_name] = wrapper
            self.emit_order.append(("message", wrapper_name))
            wnum = 1
            oneof = self._new_oneof(wrapper)
            for member in choice.findall(f"{XS}element"):
                wnum = self._convert_element(wrapper, member, wnum,
                                             f"{context} (choice)", oneof)
            self._dedupe_field_names(wrapper, context)
            self._bump_fan_in(wrapper_name)
            spec.fields.append(FieldSpec(
                proto_name="choice" if oneof == "choice" else oneof,
                proto_type=wrapper_name, wire_name="", number=number,
                label="repeated"))
            return number + 1
        oneof = self._new_oneof(spec)
        for member in choice.findall(f"{XS}element"):
            number = self._convert_element(spec, member, number,
                                           f"{context} (choice)", oneof)
        for member in choice:
            if member.tag not in (f"{XS}element", f"{XS}annotation"):
                self.index._unsupported(
                    f"non-element {local_name(member.tag)} inside xs:choice "
                    f"in {context}")
        return number

    def _new_oneof(self, spec: MessageSpec) -> str:
        name = "choice" if "choice" not in spec.oneofs else \
            f"choice_{len(spec.oneofs) + 1}"
        spec.oneofs.append(name)
        return name

    def _convert_element(self, spec: MessageSpec, element: ET.Element,
                         number: int, context: str,
                         oneof: Optional[str],
                         optional_group: bool = False) -> int:
        # Occurrence constraints always belong to the referencing particle;
        # a global element declaration cannot carry them, so they must be
        # read before any ref swap.
        min_occurs = element.get("minOccurs", "1")
        max_occurs = element.get("maxOccurs", "1")
        if element.get("ref"):
            ref = local_name(element.get("ref", ""))
            target = self.index.elements.get(ref)
            if target is None:
                raise ConversionError(
                    f"{context}: element ref '{ref}' is not a global element")
            element = target
        wire = element.get("name", "")
        type_ref = element.get("type", "")
        repeated = max_occurs not in ("", "1")
        comment = _doc_text(element)

        if type_ref:
            kind, proto_type, type_comment = self.resolve_type(
                type_ref, f"{context}.{wire}")
        else:
            inline_complex = element.find(f"{XS}complexType")
            inline_simple = element.find(f"{XS}simpleType")
            if inline_complex is not None:
                synth = f"{spec.proto_name}_{sanitize_type_name(wire)}"
                if synth not in self.messages:
                    inner = MessageSpec(
                        proto_name=synth, xsd_name="", synthesized=True,
                        comment=[f"synthesized from anonymous complexType of "
                                 f"element {wire} in {context}"])
                    self.messages[synth] = inner
                    self.emit_order.append(("message", synth))
                    self._fill_message(inner, inline_complex,
                                       f"{context}.{wire}")
                kind, proto_type, type_comment = "message", synth, []
            elif inline_simple is not None:
                self.index._unsupported(
                    f"anonymous simpleType on element {wire} in {context}")
                kind, proto_type, type_comment = "scalar", "string", \
                    ["lax fallback for anonymous simpleType"]
            else:
                self.index._unsupported(
                    f"untyped element {wire} in {context}")
                kind, proto_type, type_comment = "scalar", "string", \
                    ["lax fallback for untyped element"]

        if kind == "message":
            self._bump_fan_in(proto_type)

        label = ""
        if oneof is not None:
            if repeated:
                # proto3 forbids repeated members inside oneof: wrap.
                synth = f"{spec.proto_name}_{sanitize_type_name(wire)}_List"
                if synth not in self.messages:
                    inner = MessageSpec(
                        proto_name=synth, xsd_name="", synthesized=True,
                        comment=[f"synthesized list wrapper for repeated "
                                 f"choice member {wire} in {context}"])
                    inner.fields.append(FieldSpec(
                        proto_name="items", proto_type=proto_type,
                        wire_name=wire, number=1, label="repeated",
                        comment=type_comment))
                    self.messages[synth] = inner
                    self.emit_order.append(("message", synth))
                self._bump_fan_in(synth)
                proto_type, kind, type_comment = synth, "message", []
        elif repeated:
            label = "repeated"
        elif min_occurs == "0" or optional_group:
            label = "optional"

        spec.fields.append(FieldSpec(
            proto_name=snake_case(wire), proto_type=proto_type,
            wire_name=wire, number=number, label=label, oneof=oneof,
            comment=comment + type_comment,
            required_repeated=(label == "repeated"
                               and min_occurs != "0"
                               and not optional_group)))
        return number + 1

    def _convert_attributes(self, spec: MessageSpec, node: ET.Element,
                            number: int, context: str) -> None:
        for attr in node.findall(f"{XS}attribute"):
            wire = attr.get("name", "")
            if wire == "version":
                # Review §4.2: the per-type uci:version attribute is carried
                # as a comment, not a field.
                spec.comment.append(
                    f"uci version attribute: {attr.get('fixed') or attr.get('default') or 'per-instance'}")
                continue
            kind, proto_type, type_comment = self.resolve_type(
                attr.get("type", "xs:string"), f"{context}@{wire}")
            if kind == "message":
                raise ConversionError(
                    f"{context}: attribute '{wire}' has complex type")
            label = "" if attr.get("use") == "required" else "optional"
            spec.fields.append(FieldSpec(
                proto_name=snake_case(wire), proto_type=proto_type,
                wire_name=wire, number=number, label=label,
                comment=type_comment + ["xsd attribute"], is_attribute=True))
            number += 1

    def _dedupe_field_names(self, spec: MessageSpec, context: str) -> None:
        seen: Dict[str, int] = {}
        for fld in spec.fields:
            if fld.proto_name in seen:
                seen[fld.proto_name] += 1
                fld.comment.append(
                    "renamed: snake_case collision with an earlier field")
                fld.proto_name = f"{fld.proto_name}_{seen[fld.proto_name]}"
            else:
                seen[fld.proto_name] = 1

    def _bump_fan_in(self, proto_type: str) -> None:
        self.fan_in[proto_type] = self.fan_in.get(proto_type, 0) + 1

    # -- driving -----------------------------------------------------------

    def convert(self) -> None:
        missing = []
        for root in self.profile["roots"]:
            element = self.index.elements.get(root)
            if element is None:
                missing.append(root)
                continue
            type_ref = element.get("type", "")
            if not type_ref:
                inline = element.find(f"{XS}complexType")
                if inline is None:
                    raise ConversionError(
                        f"root element '{root}' has neither a type nor an "
                        "inline complexType")
                synth = sanitize_type_name(root)
                if synth not in self.messages:
                    spec = MessageSpec(proto_name=synth, xsd_name="",
                                       synthesized=True,
                                       comment=[f"synthesized from anonymous "
                                                f"complexType of root element {root}"])
                    self.messages[synth] = spec
                    self.emit_order.append(("message", synth))
                    self._fill_message(spec, inline, root)
                self.roots[root] = synth
                continue
            kind, proto_type, _ = self.resolve_type(type_ref, f"root {root}")
            if kind != "message":
                raise ConversionError(
                    f"root element '{root}' resolves to a non-message type")
            self.roots[root] = proto_type
        if missing:
            known = ", ".join(sorted(self.index.elements)[:20])
            raise ConversionError(
                f"profile root(s) not found as global elements: "
                f"{', '.join(missing)}; first known elements: {known}")


# -- emission ---------------------------------------------------------------


def _comment_lines(lines: List[str], indent: str) -> List[str]:
    return [f"{indent}// {line}" for line in lines]


def emit_proto(conv: Converter) -> str:
    profile = conv.profile
    out: List[str] = []
    out.append("// Generated by pim/xsd2proto.py -- DO NOT EDIT.")
    out.append(f"// profile: {profile['profile']}  drop: {profile['drop']}  "
               f"schema_version: {profile['schema_version']}")
    for name, digest in conv.index.sources:
        out.append(f"// source: {name} sha256:{digest}")
    out.append("// Wire element names live in wire_names.json beside this "
               "file (plan D3);")
    out.append("// mapping-rule deviations are documented in xsd2proto.py.")
    out.append("")
    out.append('syntax = "proto3";')
    out.append("")
    out.append(f"package {profile['proto_package']};")
    for kind, name in conv.emit_order:
        out.append("")
        if kind == "enum":
            spec = conv.enums[name]
            out.extend(_comment_lines(spec.comment, ""))
            out.append(f"enum {spec.proto_name} {{")
            out.append(f"  {upper_snake(spec.proto_name)}_UNSPECIFIED = 0;")
            for i, literal in enumerate(spec.literals, start=1):
                out.append(f"  {spec.value_name(literal)} = {i};")
            out.append("}")
        else:
            spec = conv.messages[name]
            out.extend(_comment_lines(spec.comment, ""))
            out.append(f"message {spec.proto_name} {{")
            open_oneof: Optional[str] = None
            for fld in spec.fields:
                indent = "  "
                if fld.oneof != open_oneof:
                    if open_oneof is not None:
                        out.append("  }")
                    if fld.oneof is not None:
                        out.append(f"  oneof {fld.oneof} {{")
                    open_oneof = fld.oneof
                if fld.oneof is not None:
                    indent = "    "
                out.extend(_comment_lines(fld.comment, indent))
                label = f"{fld.label} " if fld.label else ""
                out.append(f"{indent}{label}{fld.proto_type} "
                           f"{fld.proto_name} = {fld.number};")
            if open_oneof is not None:
                out.append("  }")
            out.append("}")
    out.append("")
    return "\n".join(out)


def emit_wire_names(conv: Converter) -> str:
    doc = {
        "profile": conv.profile["profile"],
        "drop": conv.profile["drop"],
        "schema_version": conv.profile["schema_version"],
        "package": conv.profile["proto_package"],
        "roots": dict(sorted(conv.roots.items())),
        "messages": {},
        "enums": {},
    }
    for name in sorted(conv.messages):
        spec = conv.messages[name]
        fields = {}
        for fld in spec.fields:
            if not fld.wire_name:
                continue  # base composition / synthesized carriers
            entry: dict = {"element": fld.wire_name}
            if fld.is_attribute:
                entry["attribute"] = True
            if fld.required_repeated:
                entry["required"] = True
            fields[fld.proto_name] = entry
        doc["messages"][name] = {"fields": fields}
        if spec.synthesized:
            doc["messages"][name]["synthesized"] = True
    for name in sorted(conv.enums):
        spec = conv.enums[name]
        doc["enums"][name] = {spec.value_name(l): l for l in spec.literals}
    return json.dumps(doc, indent=2, sort_keys=False) + "\n"


def emit_closure_report(conv: Converter) -> str:
    doc = {
        "profile": conv.profile["profile"],
        "drop": conv.profile["drop"],
        "counts": {
            "roots": len(conv.roots),
            "messages": len(conv.messages),
            "enums": len(conv.enums),
            "synthesized_messages": sum(
                1 for m in conv.messages.values() if m.synthesized),
        },
        "roots": dict(sorted(conv.roots.items())),
        "messages": sorted(conv.messages),
        "enums": sorted(conv.enums),
        "fan_in": dict(sorted(conv.fan_in.items(),
                              key=lambda kv: (-kv[1], kv[0]))),
        "skipped": sorted(conv.index.skipped),
    }
    return json.dumps(doc, indent=2, sort_keys=False) + "\n"


# -- CLI ----------------------------------------------------------------------


def locate_xsds(profile: dict, args: argparse.Namespace) -> List[Path]:
    if args.xsd:
        return [Path(p) for p in args.xsd]
    manifest_path = PIM_DIR / "schemas" / "schema_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    drop = profile["drop"]
    if drop not in manifest["drops"]:
        raise ConversionError(f"profile drop '{drop}' not in {manifest_path}")
    schemas_dir = Path(args.schemas_dir) if args.schemas_dir else \
        PIM_DIR / "schemas" / "dl"
    paths = []
    for spec in manifest["drops"][drop]["files"]:
        candidate = schemas_dir / drop / spec["name"]
        if not candidate.is_file():
            raise ConversionError(
                f"schema file absent: {candidate} -- run "
                f"pim/schemas/fetch_schemas.py {drop} (or pass --xsd)")
        paths.append(candidate)
    return paths


def outputs(conv: Converter) -> Dict[str, str]:
    return {
        f"pyramid/data_model/{conv.profile['proto_package']}.proto":
            emit_proto(conv),
        "wire_names.json": emit_wire_names(conv),
        "closure_report.json": emit_closure_report(conv),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert a profiled XSD subset to proto3.")
    parser.add_argument("profile", help="profile manifest JSON "
                        "(pim/uci_profiles/*.json)")
    parser.add_argument("--xsd", action="append",
                        help="explicit XSD path(s); overrides schema lookup")
    parser.add_argument("--schemas-dir",
                        help="directory holding <drop>/<file>.xsd "
                        "(default pim/schemas/dl)")
    parser.add_argument("--out", help="output root "
                        "(default pim/uci_generated); files land under "
                        "<out>/<drop>/")
    parser.add_argument("--check", action="store_true",
                        help="regenerate and fail if checked-in output "
                        "would change")
    parser.add_argument("--lax", action="store_true",
                        help="skip+record unsupported constructs instead of "
                        "failing")
    args = parser.parse_args()

    profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
    try:
        xsds = locate_xsds(profile, args)
        index = SchemaIndex(xsds, lax=args.lax)
        conv = Converter(index, profile)
        conv.convert()
        rendered = outputs(conv)
    except ConversionError as exc:
        print(f"xsd2proto: {exc}", file=sys.stderr)
        return 1

    out_root = Path(args.out) if args.out else PIM_DIR / "uci_generated"
    drop_dir = out_root / profile["drop"]

    if args.check:
        drift = []
        for rel, content in rendered.items():
            path = drop_dir / rel
            on_disk = path.read_text(encoding="utf-8") if path.is_file() else None
            if on_disk != content:
                drift.append(rel)
        if drift:
            print("xsd2proto --check: output drift in: " + ", ".join(drift),
                  file=sys.stderr)
            return 1
        print(f"xsd2proto --check: {drop_dir} is up to date")
        return 0

    for rel, content in rendered.items():
        path = drop_dir / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8", newline="\n")
        print(f"wrote {path}")
    report = json.loads(rendered["closure_report.json"])
    counts = report["counts"]
    print(f"closure: {counts['messages']} messages "
          f"({counts['synthesized_messages']} synthesized), "
          f"{counts['enums']} enums from {counts['roots']} roots")
    if report["skipped"]:
        print(f"skipped (lax): {len(report['skipped'])} constructs -- "
              "see closure_report.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
