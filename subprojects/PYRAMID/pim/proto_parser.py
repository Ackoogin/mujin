#!/usr/bin/env python3
"""
Full Proto IDL Parser

Parses .proto files into a complete model: packages, imports, options,
enums, messages (with nested types and oneof groups), services, and RPCs.

This is the single parser used by all codec backends and service generators.
No hardcoded knowledge of specific data models -- everything is derived
from the proto source.
"""

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional


# -- Data model ---------------------------------------------------------------

@dataclass
class ProtoEnumValue:
    """One enum constant: NAME = ordinal;"""
    name: str
    number: int


@dataclass
class ProtoEnum:
    """A proto enum definition."""
    name: str
    values: List[ProtoEnumValue] = field(default_factory=list)

    @property
    def prefix(self) -> str:
        """SCREAMING_SNAKE prefix derived from CamelCase name.

        StandardIdentity -> STANDARD_IDENTITY_
        """
        s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', self.name)
        s = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s)
        return s.upper() + '_'

    def suffix_of(self, value_name: str) -> Optional[str]:
        """Strip the enum prefix from a value name, or None if it doesn't match."""
        if value_name.startswith(self.prefix):
            return value_name[len(self.prefix):]
        return None


@dataclass
class ProtoField:
    """One field inside a message."""
    name: str
    type: str           # e.g. "double", "string", "Position", "pyramid.data_model.base.Identifier"
    number: int
    label: str = ''     # "optional", "repeated", or "" (implicit presence)
    oneof_group: Optional[str] = None

    @property
    def is_optional(self) -> bool:
        return self.label == 'optional'

    @property
    def is_repeated(self) -> bool:
        return self.label == 'repeated'

    @property
    def is_map(self) -> bool:
        return self.type.startswith('map<')

    @property
    def short_type(self) -> str:
        """Last segment of a fully qualified type: pyramid.data_model.base.Identifier -> Identifier."""
        return self.type.split('.')[-1]

    @property
    def is_scalar(self) -> bool:
        return self.type in _PROTO_SCALARS


@dataclass
class ProtoOneOf:
    """A oneof group inside a message."""
    name: str
    fields: List[ProtoField] = field(default_factory=list)


@dataclass
class ProtoMessage:
    """A proto message definition (can contain nested types)."""
    name: str
    fields: List[ProtoField] = field(default_factory=list)
    oneofs: List[ProtoOneOf] = field(default_factory=list)
    nested_enums: List[ProtoEnum] = field(default_factory=list)
    nested_messages: List['ProtoMessage'] = field(default_factory=list)

    def all_fields(self) -> List[ProtoField]:
        """All fields including those inside oneof groups."""
        result = list(self.fields)
        for oo in self.oneofs:
            result.extend(oo.fields)
        return result

    def field_by_name(self, name: str) -> Optional[ProtoField]:
        for f in self.all_fields():
            if f.name == name:
                return f
        return None


@dataclass
class ProtoRpc:
    """One rpc inside a service."""
    name: str
    request_type: str
    response_type: str
    server_streaming: bool = False
    client_streaming: bool = False


@dataclass
class ProtoService:
    """A proto service definition."""
    name: str
    rpcs: List[ProtoRpc] = field(default_factory=list)


@dataclass
class ProtoFile:
    """Complete parsed representation of a .proto file."""
    path: Path
    syntax: str = 'proto3'
    package: str = ''
    imports: List[str] = field(default_factory=list)
    options: Dict[str, str] = field(default_factory=dict)
    enums: List[ProtoEnum] = field(default_factory=list)
    messages: List[ProtoMessage] = field(default_factory=list)
    services: List[ProtoService] = field(default_factory=list)

    def find_message(self, name: str) -> Optional[ProtoMessage]:
        """Find a message by simple name (not fully qualified)."""
        for m in self.messages:
            if m.name == name:
                return m
        return None

    def find_enum(self, name: str) -> Optional[ProtoEnum]:
        """Find an enum by simple name."""
        for e in self.enums:
            if e.name == name:
                return e
        return None


# -- Proto scalar types -------------------------------------------------------

_PROTO_SCALARS = frozenset({
    'double', 'float',
    'int32', 'int64', 'uint32', 'uint64',
    'sint32', 'sint64',
    'fixed32', 'fixed64', 'sfixed32', 'sfixed64',
    'bool', 'string', 'bytes',
})


# -- Tokeniser / parser -------------------------------------------------------

def _strip_comments(text: str) -> str:
    """Remove // and /* */ comments."""
    text = re.sub(r'//[^\n]*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text


def _find_block(text: str, start: int) -> tuple:
    """Find matching { } block starting at `start` (which points at '{').

    Returns (body_text, end_pos) where end_pos is one past the '}'.
    """
    depth = 0
    i = start
    while i < len(text):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return text[start + 1:i], i + 1
        i += 1
    return text[start + 1:], len(text)


def _parse_enum_body(body: str) -> List[ProtoEnumValue]:
    """Parse the body of an enum { ... } block."""
    values = []
    for m in re.finditer(r'(\w+)\s*=\s*(-?\d+)\s*;', body):
        values.append(ProtoEnumValue(name=m.group(1), number=int(m.group(2))))
    return values


def _parse_message_body(body: str) -> ProtoMessage:
    """Parse the body of a message { ... } block into fields, oneofs, and nested types."""
    msg = ProtoMessage(name='')  # caller sets the name

    # First, extract nested enums, messages, and oneof blocks so they don't
    # interfere with field parsing.
    remaining = body

    # Nested enums
    for m in re.finditer(r'\benum\s+(\w+)\s*\{', remaining):
        enum_name = m.group(1)
        block_body, _ = _find_block(remaining, m.end() - 1)
        msg.nested_enums.append(ProtoEnum(name=enum_name, values=_parse_enum_body(block_body)))

    # Remove nested enum blocks from remaining text
    remaining = re.sub(r'\benum\s+\w+\s*\{[^}]*\}', '', remaining)

    # Nested messages (non-recursive for now -- handles one level of nesting)
    nested_msg_pattern = re.compile(r'\bmessage\s+(\w+)\s*\{')
    for m in nested_msg_pattern.finditer(remaining):
        nested_name = m.group(1)
        block_body, _ = _find_block(remaining, m.end() - 1)
        nested = _parse_message_body(block_body)
        nested.name = nested_name
        msg.nested_messages.append(nested)

    # Remove nested message blocks
    remaining = _remove_nested_blocks(remaining, 'message')

    # Oneof blocks
    for m in re.finditer(r'\boneof\s+(\w+)\s*\{', remaining):
        oneof_name = m.group(1)
        block_body, _ = _find_block(remaining, m.end() - 1)
        oo = ProtoOneOf(name=oneof_name)
        for fm in re.finditer(
                r'((?:repeated|optional)\s+)?([\w.]+)\s+(\w+)\s*=\s*(\d+)\s*;',
                block_body):
            label = (fm.group(1) or '').strip()
            oo.fields.append(ProtoField(
                name=fm.group(3),
                type=fm.group(2),
                number=int(fm.group(4)),
                label=label,
                oneof_group=oneof_name,
            ))
        msg.oneofs.append(oo)

    # Remove oneof blocks
    remaining = _remove_nested_blocks(remaining, 'oneof')

    # Regular fields (what's left)
    for fm in re.finditer(
            r'((?:repeated|optional)\s+)?([\w.]+)\s+(\w+)\s*=\s*(\d+)\s*;',
            remaining):
        label = (fm.group(1) or '').strip()
        msg.fields.append(ProtoField(
            name=fm.group(3),
            type=fm.group(2),
            number=int(fm.group(4)),
            label=label,
        ))

    # Sort fields by number
    msg.fields.sort(key=lambda f: f.number)
    for oo in msg.oneofs:
        oo.fields.sort(key=lambda f: f.number)

    return msg


def _remove_nested_blocks(text: str, keyword: str) -> str:
    """Remove all `keyword Name { ... }` blocks from text (handles nesting)."""
    result = []
    pattern = re.compile(rf'\b{keyword}\s+\w+\s*\{{')
    pos = 0
    for m in pattern.finditer(text):
        result.append(text[pos:m.start()])
        _, end = _find_block(text, m.end() - 1)
        pos = end
    result.append(text[pos:])
    return ''.join(result)


def parse_proto(path: Path) -> ProtoFile:
    """Parse a single .proto file into a ProtoFile model."""
    raw = path.read_text(encoding='utf-8')
    text = _strip_comments(raw)

    pf = ProtoFile(path=path)

    # Syntax
    m = re.search(r'\bsyntax\s*=\s*"([^"]+)"\s*;', text)
    if m:
        pf.syntax = m.group(1)

    # Package
    m = re.search(r'\bpackage\s+([\w.]+)\s*;', text)
    if m:
        pf.package = m.group(1)

    # Imports
    for m in re.finditer(r'\bimport\s+"([^"]+)"\s*;', text):
        pf.imports.append(m.group(1))

    # Options
    for m in re.finditer(r'\boption\s+(\w+)\s*=\s*"?([^";]+)"?\s*;', text):
        pf.options[m.group(1)] = m.group(2).strip()

    # Top-level enums (not inside a message)
    clean = _remove_nested_blocks(text, 'message')
    clean = _remove_nested_blocks(clean, 'service')
    for m in re.finditer(r'\benum\s+(\w+)\s*\{', clean):
        enum_name = m.group(1)
        block_body, _ = _find_block(clean, m.end() - 1)
        pf.enums.append(ProtoEnum(name=enum_name, values=_parse_enum_body(block_body)))

    # Top-level messages
    msg_text = _remove_nested_blocks(text, 'service')
    for m in re.finditer(r'\bmessage\s+(\w+)\s*\{', msg_text):
        msg_name = m.group(1)
        block_body, _ = _find_block(msg_text, m.end() - 1)
        msg = _parse_message_body(block_body)
        msg.name = msg_name
        pf.messages.append(msg)

    # Services
    for m in re.finditer(r'\bservice\s+(\w+)\s*\{', text):
        svc_name = m.group(1)
        block_body, _ = _find_block(text, m.end() - 1)
        svc = ProtoService(name=svc_name)
        for rm in re.finditer(
                r'\brpc\s+(\w+)\s*\(\s*(stream\s+)?([\w.]+)\s*\)\s*returns\s*\(\s*(stream\s+)?([\w.]+)\s*\)',
                block_body):
            svc.rpcs.append(ProtoRpc(
                name=rm.group(1),
                request_type=rm.group(3),
                response_type=rm.group(5),
                client_streaming=bool(rm.group(2)),
                server_streaming=bool(rm.group(4)),
            ))
        pf.services.append(svc)

    return pf


def parse_proto_tree(root: Path) -> List[ProtoFile]:
    """Parse all .proto files under a directory tree.

    Returns list sorted by package name for deterministic output.
    """
    files = sorted(root.rglob('*.proto'))
    result = [parse_proto(f) for f in files]
    result.sort(key=lambda pf: pf.package)
    return result


# -- Type resolution helpers --------------------------------------------------

class ProtoTypeIndex:
    """Index across multiple ProtoFile objects for cross-file type resolution."""

    def __init__(self, files: List[ProtoFile]):
        self._files = files
        self._msg_index: Dict[str, ProtoMessage] = {}
        self._enum_index: Dict[str, ProtoEnum] = {}
        self._pkg_index: Dict[str, ProtoFile] = {}
        for pf in files:
            self._pkg_index[pf.package] = pf
            for msg in pf.messages:
                fqn = f'{pf.package}.{msg.name}' if pf.package else msg.name
                self._msg_index[fqn] = msg
                self._msg_index[msg.name] = msg  # short name fallback
            for enum in pf.enums:
                fqn = f'{pf.package}.{enum.name}' if pf.package else enum.name
                self._enum_index[fqn] = enum
                self._enum_index[enum.name] = enum

    @property
    def files(self) -> List[ProtoFile]:
        return self._files

    def resolve_message(self, type_name: str) -> Optional[ProtoMessage]:
        """Resolve a type name to a message, trying FQN then short name."""
        return self._msg_index.get(type_name)

    def resolve_enum(self, type_name: str) -> Optional[ProtoEnum]:
        """Resolve a type name to an enum."""
        return self._enum_index.get(type_name)

    def is_enum_type(self, type_name: str) -> bool:
        return type_name in self._enum_index

    def is_message_type(self, type_name: str) -> bool:
        return type_name in self._msg_index

    def all_messages(self) -> List[ProtoMessage]:
        """All messages across all files (no duplicates)."""
        seen = set()
        result = []
        for pf in self._files:
            for msg in pf.messages:
                if msg.name not in seen:
                    seen.add(msg.name)
                    result.append(msg)
        return result

    def all_enums(self) -> List[ProtoEnum]:
        """All enums across all files (no duplicates)."""
        seen = set()
        result = []
        for pf in self._files:
            for enum in pf.enums:
                if enum.name not in seen:
                    seen.add(enum.name)
                    result.append(enum)
        return result


# -- Naming helpers (used by all backends) ------------------------------------

def camel_to_snake(name: str) -> str:
    """TacticalObject -> Tactical_Object (Ada style)."""
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    s = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s)
    return s


def camel_to_lower_snake(name: str) -> str:
    """ObjectOfInterest -> object_of_interest (wire-format style)."""
    return camel_to_snake(name).lower()


def snake_to_pascal(name: str) -> str:
    """entity_matches -> EntityMatches."""
    return ''.join(w.capitalize() for w in name.split('_'))


def lc_first(s: str) -> str:
    """CreateRequirement -> createRequirement."""
    return s[0].lower() + s[1:] if s else s


def screaming_to_title(suffix: str) -> str:
    """SEA_SURFACE -> Sea_Surface (Ada literal style)."""
    return '_'.join(w.capitalize() for w in suffix.split('_'))


def screaming_to_pascal(suffix: str) -> str:
    """SEA_SURFACE -> SeaSurface (C++ enum style)."""
    return ''.join(w.capitalize() for w in suffix.split('_'))


# -- CLI self-test ------------------------------------------------------------

if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print('Usage: python proto_parser.py <file.proto | proto_dir>')
        sys.exit(1)

    target = Path(sys.argv[1])
    if target.is_dir():
        files = parse_proto_tree(target)
    else:
        files = [parse_proto(target)]

    for pf in files:
        print(f'\n=== {pf.path.name} (package: {pf.package}) ===')
        if pf.imports:
            print(f'  Imports: {", ".join(pf.imports)}')
        for e in pf.enums:
            print(f'  Enum {e.name}: {len(e.values)} values')
        for m in pf.messages:
            n_fields = len(m.all_fields())
            n_oneof = len(m.oneofs)
            parts = [f'{n_fields} fields']
            if n_oneof:
                parts.append(f'{n_oneof} oneof')
            if m.nested_enums:
                parts.append(f'{len(m.nested_enums)} nested enums')
            if m.nested_messages:
                parts.append(f'{len(m.nested_messages)} nested msgs')
            print(f'  Message {m.name}: {", ".join(parts)}')
        for s in pf.services:
            print(f'  Service {s.name}: {len(s.rpcs)} rpcs')
            for r in s.rpcs:
                stream = ' (stream)' if r.server_streaming else ''
                print(f'    rpc {r.name}({r.request_type}) -> {r.response_type}{stream}')
