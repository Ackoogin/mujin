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
    documentation: List[str] = field(default_factory=list)


@dataclass
class ProtoEnum:
    """A proto enum definition."""
    name: str
    values: List[ProtoEnumValue] = field(default_factory=list)
    documentation: List[str] = field(default_factory=list)

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
    documentation: List[str] = field(default_factory=list)

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
    documentation: List[str] = field(default_factory=list)


@dataclass
class ProtoMessage:
    """A proto message definition (can contain nested types)."""
    name: str
    fields: List[ProtoField] = field(default_factory=list)
    oneofs: List[ProtoOneOf] = field(default_factory=list)
    nested_enums: List[ProtoEnum] = field(default_factory=list)
    nested_messages: List['ProtoMessage'] = field(default_factory=list)
    documentation: List[str] = field(default_factory=list)

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
    pattern: Optional[str] = None
    topic: Optional[str] = None
    qos: Dict[str, object] = field(default_factory=dict)
    documentation: List[str] = field(default_factory=list)


@dataclass
class ProtoService:
    """A proto service definition."""
    name: str
    rpcs: List[ProtoRpc] = field(default_factory=list)
    port_kind: Optional[str] = None
    documentation: List[str] = field(default_factory=list)


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


# Compiler-extension packages (custom option definitions). Parsed so method
# options can be captured, but never part of the generated SDK surface — every
# ProtoTypeIndex handed to a binding/IDL generator must exclude them.
BINDING_EXCLUDED_PACKAGES = frozenset({
    'pyramid.options',
})


def is_binding_proto(pf: 'ProtoFile') -> bool:
    """True for application contract protos that should produce bindings."""
    return pf.package not in BINDING_EXCLUDED_PACKAGES


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
    """Replace comments with whitespace while preserving source offsets."""
    def blank(match: re.Match) -> str:
        return ''.join('\n' if char == '\n' else ' ' for char in match.group())

    text = re.sub(r'//[^\n]*', blank, text)
    return re.sub(r'/\*.*?\*/', blank, text, flags=re.DOTALL)


def _documentation_before(raw: str, offset: int) -> List[str]:
    """Return contiguous source comments immediately before a declaration."""
    prefix = raw[:offset]
    match = re.search(
        r'(?:(?:[ \t]*//[^\n]*(?:\n|$))|(?:[ \t]*/\*.*?\*/[ \t]*(?:\n|$)))+[ \t\r\n]*$',
        prefix,
        flags=re.DOTALL,
    )
    if not match:
        return []

    lines: List[str] = []
    for comment in re.finditer(r'//([^\n]*)|/\*(.*?)\*/', match.group(), re.DOTALL):
        content = comment.group(1) if comment.group(1) is not None else comment.group(2)
        for line in content.splitlines() or ['']:
            line = line.strip()
            if line.startswith('*'):
                line = line[1:].lstrip()
            lines.append(line)
    while lines and not lines[0]:
        lines.pop(0)
    while lines and not lines[-1]:
        lines.pop()
    return lines


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


def _parse_enum_body(body: str, raw_body: str = '') -> List[ProtoEnumValue]:
    """Parse the body of an enum { ... } block."""
    values = []
    for m in re.finditer(r'(\w+)\s*=\s*(-?\d+)\s*;', body):
        values.append(ProtoEnumValue(
            name=m.group(1), number=int(m.group(2)),
            documentation=_documentation_before(raw_body, m.start()) if raw_body else [],
        ))
    return values


def _parse_message_body(body: str, raw_body: str = '') -> ProtoMessage:
    """Parse the body of a message { ... } block into fields, oneofs, and nested types."""
    msg = ProtoMessage(name='')  # caller sets the name

    # First, extract nested enums, messages, and oneof blocks so they don't
    # interfere with field parsing.
    remaining = body

    # Nested enums
    for m in re.finditer(r'\benum\s+(\w+)\s*\{', remaining):
        enum_name = m.group(1)
        block_body, _ = _find_block(remaining, m.end() - 1)
        raw_block, _ = _find_block(raw_body, m.end() - 1) if raw_body else ('', 0)
        msg.nested_enums.append(ProtoEnum(
            name=enum_name,
            values=_parse_enum_body(block_body, raw_block),
            documentation=_documentation_before(raw_body, m.start()) if raw_body else [],
        ))

    # Blank nested enum blocks while preserving positions for documentation.
    remaining = _remove_nested_blocks(remaining, 'enum')

    # Nested messages (non-recursive for now -- handles one level of nesting)
    nested_msg_pattern = re.compile(r'\bmessage\s+(\w+)\s*\{')
    for m in nested_msg_pattern.finditer(remaining):
        nested_name = m.group(1)
        block_body, _ = _find_block(remaining, m.end() - 1)
        raw_block, _ = _find_block(raw_body, m.end() - 1) if raw_body else ('', 0)
        nested = _parse_message_body(block_body, raw_block)
        nested.name = nested_name
        nested.documentation = (_documentation_before(raw_body, m.start())
                                if raw_body else [])
        msg.nested_messages.append(nested)

    # Remove nested message blocks
    remaining = _remove_nested_blocks(remaining, 'message')

    # Oneof blocks
    for m in re.finditer(r'\boneof\s+(\w+)\s*\{', remaining):
        oneof_name = m.group(1)
        block_body, _ = _find_block(remaining, m.end() - 1)
        raw_block, _ = _find_block(raw_body, m.end() - 1) if raw_body else ('', 0)
        oo = ProtoOneOf(
            name=oneof_name,
            documentation=_documentation_before(raw_body, m.start()) if raw_body else [],
        )
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
                documentation=(_documentation_before(raw_block, fm.start())
                               if raw_body else []),
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
            documentation=(_documentation_before(raw_body, fm.start())
                           if raw_body else []),
        ))

    # Sort fields by number
    msg.fields.sort(key=lambda f: f.number)
    for oo in msg.oneofs:
        oo.fields.sort(key=lambda f: f.number)

    return msg


def _remove_nested_blocks(text: str, keyword: str) -> str:
    """Blank all `keyword Name { ... }` blocks while preserving offsets."""
    result = []
    pattern = re.compile(rf'\b{keyword}\s+\w+\s*\{{')
    pos = 0
    for m in pattern.finditer(text):
        result.append(text[pos:m.start()])
        _, end = _find_block(text, m.end() - 1)
        result.append(''.join('\n' if char == '\n' else ' '
                              for char in text[m.start():end]))
        pos = end
    result.append(text[pos:])
    return ''.join(result)


def _short_type(type_name: str) -> str:
    return type_name.split('.')[-1] if type_name else ''


def _is_type(type_name: str, short_name: str) -> bool:
    return _short_type(type_name) == short_name


def _parse_interaction_option(body: str) -> tuple:
    """Parse a pyramid.options.Interaction method option body.

    The proto parser is intentionally lightweight, so this extracts the small
    option shape used by the generator rather than implementing full proto
    option grammar.
    """
    option_match = re.search(
        r'\boption\s+\(pyramid\.options\.pyramid_op\)\s*=\s*\{',
        body,
    )
    if not option_match:
        return None, None, {}

    option_body, _ = _find_block(body, option_match.end() - 1)
    pattern = None
    topic = None
    qos: Dict[str, object] = {}

    pattern_match = re.search(r'\bpattern\s*:\s*(\w+)', option_body)
    if pattern_match:
        pattern = pattern_match.group(1)

    topic_match = re.search(r'\btopic\s*:\s*"([^"]*)"', option_body)
    if topic_match:
        topic = topic_match.group(1)

    qos_match = re.search(r'\bqos\s*:\s*\{', option_body)
    if qos_match:
        qos_body, _ = _find_block(option_body, qos_match.end() - 1)
        for key in ('reliability', 'durability'):
            match = re.search(rf'\b{key}\s*:\s*(\w+)', qos_body)
            if match:
                qos[key] = match.group(1)
        depth_match = re.search(r'\bdepth\s*:\s*(\d+)', qos_body)
        if depth_match:
            qos['depth'] = int(depth_match.group(1))

    return pattern, topic, qos


def classify_port_service(service: ProtoService) -> Optional[str]:
    """Classify an MBSE port-grammar service shape.

    This is advisory Layer-1 metadata. Layer-2 method options remain
    authoritative wherever they are present.
    """
    rpcs = service.rpcs
    if len(rpcs) == 1:
        rpc = rpcs[0]
        if (rpc.name == 'Read'
                and not rpc.client_streaming
                and rpc.server_streaming
                and rpc.request_type == 'google.protobuf.Empty'):
            return 'information'
        return None

    if len(rpcs) != 4 or [rpc.name for rpc in rpcs] != [
            'Create', 'Read', 'Update', 'Cancel']:
        return None

    create, read, update, cancel = rpcs
    ack_responses = (
        _is_type(create.response_type, 'Ack')
        and _is_type(update.response_type, 'Ack')
        and _is_type(cancel.response_type, 'Ack')
    )
    if not ack_responses:
        return None
    if any(rpc.client_streaming for rpc in rpcs):
        return None
    if create.server_streaming or update.server_streaming or cancel.server_streaming:
        return None
    if not read.server_streaming:
        return None
    if not _is_type(read.request_type, 'Query'):
        return None
    if update.request_type != read.response_type:
        return None
    if not _is_type(cancel.request_type, 'Identifier'):
        return None
    return 'request'


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
        raw_block, _ = _find_block(raw, m.end() - 1)
        pf.enums.append(ProtoEnum(
            name=enum_name,
            values=_parse_enum_body(block_body, raw_block),
            documentation=_documentation_before(raw, m.start()),
        ))

    # Top-level messages
    msg_text = _remove_nested_blocks(text, 'service')
    for m in re.finditer(r'\bmessage\s+(\w+)\s*\{', msg_text):
        msg_name = m.group(1)
        block_body, _ = _find_block(msg_text, m.end() - 1)
        raw_block, _ = _find_block(raw, m.end() - 1)
        msg = _parse_message_body(block_body, raw_block)
        msg.name = msg_name
        msg.documentation = _documentation_before(raw, m.start())
        pf.messages.append(msg)

    # Services
    for m in re.finditer(r'\bservice\s+(\w+)\s*\{', text):
        svc_name = m.group(1)
        block_body, _ = _find_block(text, m.end() - 1)
        raw_block, _ = _find_block(raw, m.end() - 1)
        svc = ProtoService(
            name=svc_name,
            documentation=_documentation_before(raw, m.start()),
        )
        for rm in re.finditer(
                r'\brpc\s+(\w+)\s*\(\s*(stream\s+)?([\w.]+)\s*\)\s*returns\s*\(\s*(stream\s+)?([\w.]+)\s*\)',
                block_body):
            rpc_body = ''
            pos = rm.end()
            while pos < len(block_body) and block_body[pos].isspace():
                pos += 1
            if pos < len(block_body) and block_body[pos] == '{':
                rpc_body, _ = _find_block(block_body, pos)
            pattern, topic, qos = _parse_interaction_option(rpc_body)
            svc.rpcs.append(ProtoRpc(
                name=rm.group(1),
                request_type=rm.group(3),
                response_type=rm.group(5),
                client_streaming=bool(rm.group(2)),
                server_streaming=bool(rm.group(4)),
                pattern=pattern,
                topic=topic,
                qos=qos,
                documentation=_documentation_before(raw_block, rm.start()),
            ))
        svc.port_kind = classify_port_service(svc)
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
