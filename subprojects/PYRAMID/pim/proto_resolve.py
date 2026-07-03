#!/usr/bin/env python3
"""Language-neutral proto type resolution helpers.

Package-aware resolution of proto type names against a ProtoTypeIndex:
bare names are scoped to the current package first, then accepted only when
unique across the index.  Shared by the C++ and Ada code generators (and the
C-ABI / FlatBuffers layers built on them); nothing here is specific to any
target language.

Moved verbatim out of cpp_codegen.py (generator refactor plan, phase 1).
"""

from typing import List, Optional, Tuple

from proto_parser import (
    ProtoEnum,
    ProtoField,
    ProtoMessage,
    ProtoTypeIndex,
    _PROTO_SCALARS,
)

_DATA_MODEL_PROTO_ROOT = 'pyramid.data_model'


def _data_model_package_for_type(full_type: str) -> str:
    if not full_type.startswith(_DATA_MODEL_PROTO_ROOT + '.'):
        return ''
    if '.' not in full_type:
        return ''
    return full_type.rsplit('.', 1)[0]


def _qualified_package_for_type(full_type: str) -> str:
    """Package of any fully-qualified proto type (data-model or component).

    Unlike _data_model_package_for_type this does not restrict to the
    pyramid.data_model root, so service-local wrapper messages (canonically
    homed in their component package) also resolve.
    """
    if '.' not in full_type:
        return ''
    return full_type.rsplit('.', 1)[0]


def _message_matches(index: ProtoTypeIndex,
                     short_name: str) -> List[Tuple[str, ProtoMessage]]:
    result = []
    for pf in index.files:
        for msg in pf.messages:
            if msg.name == short_name:
                result.append((pf.package, msg))
    return result


def _enum_matches(index: ProtoTypeIndex,
                  short_name: str) -> List[Tuple[str, ProtoEnum]]:
    result = []
    for pf in index.files:
        for enum in pf.enums:
            if enum.name == short_name:
                result.append((pf.package, enum))
    return result


def _resolve_message(index: ProtoTypeIndex, type_name: str,
                     current_pkg: str = '') -> Tuple[Optional[ProtoMessage], str]:
    """Resolve a proto message with package-aware bare-name handling.

    Bare proto names are scoped to the current package first. If there is no
    current-package match, a bare name is accepted only when it is unique in the
    index. This keeps duplicate short names from resolving to an arbitrary
    package.
    """
    if not type_name or type_name in _PROTO_SCALARS:
        return None, ''
    if type_name.startswith('google.'):
        return None, ''
    if '.' in type_name:
        return index.resolve_message(type_name), type_name.rsplit('.', 1)[0]
    if current_pkg:
        fqn = f'{current_pkg}.{type_name}'
        msg = index.resolve_message(fqn)
        if msg is not None:
            return msg, current_pkg
    matches = _message_matches(index, type_name)
    if len(matches) == 1:
        return matches[0][1], matches[0][0]
    return None, ''


def _resolve_enum(index: ProtoTypeIndex, type_name: str,
                  current_pkg: str = '') -> Tuple[Optional[ProtoEnum], str]:
    if not type_name or type_name in _PROTO_SCALARS:
        return None, ''
    if type_name.startswith('google.'):
        return None, ''
    if '.' in type_name:
        return index.resolve_enum(type_name), type_name.rsplit('.', 1)[0]
    if current_pkg:
        fqn = f'{current_pkg}.{type_name}'
        enum = index.resolve_enum(fqn)
        if enum is not None:
            return enum, current_pkg
    matches = _enum_matches(index, type_name)
    if len(matches) == 1:
        return matches[0][1], matches[0][0]
    return None, ''


def _proto_type_fqn(index: ProtoTypeIndex, type_name: str,
                    current_pkg: str = '') -> str:
    if not type_name or type_name in _PROTO_SCALARS:
        return type_name
    if type_name.startswith('google.'):
        return type_name
    if '.' in type_name:
        return type_name
    msg, pkg = _resolve_message(index, type_name, current_pkg)
    if msg is not None and pkg:
        return f'{pkg}.{msg.name}'
    enum, pkg = _resolve_enum(index, type_name, current_pkg)
    if enum is not None and pkg:
        return f'{pkg}.{enum.name}'
    return ''


def _package_for_proto_type(index: ProtoTypeIndex, type_name: str,
                            current_pkg: str = '') -> str:
    if '.' in type_name and not type_name.startswith('google.'):
        return type_name.rsplit('.', 1)[0]
    fqn = _proto_type_fqn(index, type_name, current_pkg)
    if '.' in fqn and not fqn.startswith('google.'):
        return fqn.rsplit('.', 1)[0]
    return ''


def _is_proto_message_type(index: ProtoTypeIndex, type_name: str,
                           current_pkg: str = '') -> bool:
    fqn = _proto_type_fqn(index, type_name, current_pkg)
    return bool(fqn and index.is_message_type(fqn))


def _is_proto_enum_type(index: ProtoTypeIndex, type_name: str,
                        current_pkg: str = '') -> bool:
    fqn = _proto_type_fqn(index, type_name, current_pkg)
    return bool(fqn and index.is_enum_type(fqn))


def _field_with_type(field: ProtoField, field_type: str) -> ProtoField:
    return ProtoField(
        name=field.name,
        type=field_type or field.type,
        number=field.number,
        label=field.label,
        oneof_group=field.oneof_group,
    )
