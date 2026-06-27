#!/usr/bin/env python3
"""
Shared ROS2 intermediate representation (domain-model field model).

The native ROS2 IDL and the `domain_model <-> ROS2 message` marshalling are both
derived from this one model, so they are aligned by construction. It mirrors the
exact field shape that `cpp_codegen.CppTypesGenerator` emits for the
`pyramid::domain_model::*` structs (the canonical native representation every wire
codec round-trips), reusing cpp_codegen's own alias rules:

* scalar-wrapper messages collapse to their scalar (`Identifier` -> string,
  `Angle`/`Length` -> double, `Timestamp` -> double epoch). Such messages are not
  emitted as ROS2 types; fields use the underlying primitive.
* a `base` message field is inlined one level (with collision renaming), matching
  `_inline_base_fields`.
* a proto3 `optional` non-string field and every `oneof` member are
  `tl::optional<>` in domain_model -> a ROS2 `bool has_<name>` presence companion
  + the value field; an optional *string* stays a plain string (empty = absent),
  matching domain_model.
* `repeated` -> sequence; enum -> a wrapper message (`<E>` with `int32 value` +
  ordinal constants) so the value stays typed and the constants are discoverable.

Keeping the ROS2 shape equal to domain_model makes the converter near-mechanical
and avoids fragile alias-expansion / base-un-inlining.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from proto_parser import (  # noqa: E402
    ProtoField,
    ProtoMessage,
    ProtoTypeIndex,
    camel_to_lower_snake,
)
from cpp_codegen import (  # noqa: E402
    find_scalar_wrappers,
    _CPP_SCALAR_MAP,
    _cpp_ns_for_proto_type_package,
)

_PROTO_SCALARS = frozenset(_CPP_SCALAR_MAP.keys())

# proto scalar -> ROS2 primitive.
PROTO_TO_ROS: Dict[str, str] = {
    'double': 'float64', 'float': 'float32',
    'int32': 'int32', 'sint32': 'int32', 'sfixed32': 'int32',
    'uint32': 'uint32', 'fixed32': 'uint32',
    'int64': 'int64', 'sint64': 'int64', 'sfixed64': 'int64',
    'uint64': 'uint64', 'fixed64': 'uint64',
    'bool': 'bool', 'string': 'string',
}

# domain_model C++ scalar type -> ROS2 primitive (for collapsed aliases, whose
# underlying type cpp_codegen records as a C++ type rather than a proto scalar).
CPP_TO_ROS: Dict[str, str] = {
    'double': 'float64', 'float': 'float32',
    'int32_t': 'int32', 'int64_t': 'int64',
    'uint32_t': 'uint32', 'uint64_t': 'uint64',
    'bool': 'bool', 'std::string': 'string',
}

DATA_MODEL_NS = 'pyramid::domain_model'


def pascal(name: str) -> str:
    """Underscore/segmented name -> CamelCase ROS2 interface name."""
    parts = [p for p in name.replace('.', '_').split('_') if p]
    return ''.join(p[:1].upper() + p[1:] for p in parts)


def ros_header_stem(pascal_name: str) -> str:
    """ROS2 message name -> rosidl header stem (ObjectMatch -> object_match)."""
    return camel_to_lower_snake(pascal_name)


@dataclass
class DomainField:
    """One field of a domain_model struct, mapped to its ROS2 representation."""
    name: str                 # field name (after base-inline renaming)
    presence: str             # 'plain' | 'opt' | 'repeated'
    category: str             # 'scalar' | 'string' | 'enum' | 'message'
    ros_type: str             # element ROS2 token: float64 / string / <Pascal>
    cpp_scalar: str = ''      # scalar/string: domain C++ scalar (double, std::string)
    msg_short: str = ''       # enum/message: proto short name


class DomainIR:
    """Resolves proto types into the domain_model field model used for ROS2."""

    def __init__(self, index: ProtoTypeIndex):
        self.index = index
        self.aliases = find_scalar_wrappers(index)  # short -> C++ scalar type
        self._pkg_of: Dict[str, str] = {}
        for pf in index.files:
            for msg in pf.messages:
                self._pkg_of[msg.name] = pf.package
            for enum in pf.enums:
                self._pkg_of[enum.name] = pf.package

    # -- predicates -----------------------------------------------------------

    def is_alias(self, short: str) -> bool:
        return short in self.aliases

    def is_enum(self, short: str) -> bool:
        return self.index.resolve_enum(short) is not None

    def emitted_messages(self) -> List[ProtoMessage]:
        """All messages that become ROS2 .msg (i.e. excluding scalar aliases)."""
        return [m for m in self.index.all_messages() if m.name not in self.aliases]

    # -- naming ---------------------------------------------------------------

    def domain_fqn(self, short: str) -> str:
        pkg = self._pkg_of.get(short, '')
        ns = _cpp_ns_for_proto_type_package(pkg) if pkg else DATA_MODEL_NS
        return f'{ns}::{short}'

    def ros_msg_type(self, short: str) -> str:
        return f'pyramid_msgs::msg::{pascal(short)}'

    # -- field model ----------------------------------------------------------

    def _classify(self, field: ProtoField) -> DomainField:
        t = field.type
        short = t.split('.')[-1]
        if t in _PROTO_SCALARS:
            if t == 'string':
                cat, ros, cpp = 'string', 'string', 'std::string'
            elif t == 'bytes':
                raise NotImplementedError(
                    f'bytes field {field.name!r} not supported by the ROS2 codec')
            else:
                cat, ros, cpp = 'scalar', PROTO_TO_ROS[t], _CPP_SCALAR_MAP[t]
            return DomainField(field.name, 'plain', cat, ros, cpp_scalar=cpp)
        if short in self.aliases:
            cpp = self.aliases[short]
            if cpp == 'std::string':
                return DomainField(field.name, 'plain', 'string', 'string',
                                   cpp_scalar='std::string')
            return DomainField(field.name, 'plain', 'scalar', CPP_TO_ROS[cpp],
                               cpp_scalar=cpp)
        if self.is_enum(short):
            return DomainField(field.name, 'plain', 'enum', pascal(short),
                               msg_short=short)
        if self.index.resolve_message(short) is not None:
            return DomainField(field.name, 'plain', 'message', pascal(short),
                               msg_short=short)
        raise NotImplementedError(f'unresolved field type {t!r} ({field.name!r})')

    def _with_presence(self, base: DomainField, field: ProtoField,
                       oneof: bool) -> DomainField:
        if field.is_repeated:
            base.presence = 'repeated'
        elif oneof:
            base.presence = 'opt'          # domain: tl::optional for all oneof members
        elif field.is_optional and base.category != 'string':
            base.presence = 'opt'
        else:
            base.presence = 'plain'        # incl. optional string (plain in domain)
        return base

    def fields(self, msg: ProtoMessage) -> List[DomainField]:
        """Domain fields for a message: regular (+inlined base) then oneof members."""
        out: List[DomainField] = []
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                short = field.type.split('.')[-1]
                base_msg = (self.index.resolve_message(field.type)
                            or self.index.resolve_message(short))
                if base_msg is not None and base_msg.name not in self.aliases:
                    for bf in base_msg.fields:
                        name = bf.name
                        if name in own_names:
                            name = short.lower() + '_' + name
                        df = self._with_presence(self._classify(bf), bf, oneof=False)
                        df.name = name
                        out.append(df)
                    continue
            out.append(self._with_presence(self._classify(field), field, oneof=False))
        for oo in msg.oneofs:
            for field in oo.fields:
                out.append(self._with_presence(self._classify(field), field,
                                               oneof=True))
        return out
