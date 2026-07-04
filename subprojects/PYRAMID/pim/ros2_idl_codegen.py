#!/usr/bin/env python3
"""
Native ROS2 IDL generator.

Emits real ROS2 interface definitions (``.msg`` / ``.srv``) from the canonical
``.proto`` contracts, so PYRAMID-over-ROS2 is a first-class, introspectable ROS2
citizen rather than an opaque ``PclEnvelope`` byte blob. This is the foundation
of the "native IDL, not envelope" work tracked in
``doc/plans/PYRAMID/transport_plugins.md`` §1.

The interface shape mirrors the ``pyramid::domain_model::*`` structs (the native
representation every wire codec round-trips) via the shared
:mod:`ros2_ir` model, so the generated IDL and the ``domain_model <-> ROS2``
marshalling (:mod:`ros2_marshal_codegen`) are aligned by construction. See
``ros2_ir`` for the exact mapping rules (alias collapse, base inlining,
optional/oneof presence companions, enum wrappers).

Why not proto2ros: proto2ros bridges *protobuf C++ runtime objects* <-> ROS2 and
is messages-only (no ``.srv``/``.action``); PYRAMID keeps libprotobuf out of the
component/client and needs services + streaming. Useful as a mapping reference
only.
"""

from pathlib import Path
from typing import List, Optional, Tuple

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from proto_parser import ProtoEnum, ProtoFile, ProtoMessage, ProtoTypeIndex  # noqa: E402
from ros2_ir import CPP_TO_ROS, DomainIR, pascal  # noqa: E402

# Packages whose generated IDL we depend on but do not emit. The domain model
# collapses google.protobuf.Timestamp to an epoch double, so there is currently
# no external interface dependency.
IDL_EXTERNAL_DEPENDENCIES: Tuple[str, ...] = ()


def _is_service_package(pf: ProtoFile) -> bool:
    return '.services.' in f'.{pf.package}.'


def _is_empty_type(type_name: str) -> bool:
    return type_name == 'google.protobuf.Empty'


class Ros2IdlGenerator:
    """Generates ROS2 ``.msg`` / ``.srv`` definitions from a proto index."""

    def __init__(self, index: ProtoTypeIndex,
                 service_modules: Optional[List[ProtoFile]] = None):
        self.index = index
        self.ir = DomainIR(index)
        self.service_modules = service_modules

    # -- public API -----------------------------------------------------------

    def generate(self, output_dir: Path) -> List[Path]:
        msg_dir = output_dir / 'msg'
        srv_dir = output_dir / 'srv'
        msg_dir.mkdir(parents=True, exist_ok=True)
        srv_dir.mkdir(parents=True, exist_ok=True)

        written: List[Path] = []
        for enum in self.index.all_enums():
            written.append(self._write_enum_msg(msg_dir, enum))
        for msg in self.ir.emitted_messages():
            written.append(self._write_message_msg(msg_dir, msg))
        service_files = (self.service_modules if self.service_modules is not None
                         else self.index.files)
        for pf in service_files:
            if self.service_modules is None and not _is_service_package(pf):
                continue
            for svc in pf.services:
                for rpc in svc.rpcs:
                    written.extend(self._write_rpc(msg_dir, srv_dir, pf, svc, rpc))
        return sorted(set(written))

    # -- enums ----------------------------------------------------------------

    def _write_enum_msg(self, msg_dir: Path, enum: ProtoEnum) -> Path:
        path = msg_dir / f'{pascal(enum.name)}.msg'
        lines = [
            '# Auto-generated from .proto -- do not edit',
            f'# proto enum {enum.name}',
        ]
        for v in enum.values:
            lines.append(f'int32 {v.name}={v.number}')
        lines.append('int32 value')
        path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        return path

    # -- messages -------------------------------------------------------------

    def _message_body(self, msg: ProtoMessage) -> List[str]:
        lines: List[str] = []
        for df in self.ir.fields(msg):
            if df.presence == 'repeated':
                lines.append(f'{df.ros_type}[] {df.name}')
            elif df.presence == 'opt':
                lines.append(f'bool has_{df.name}')
                lines.append(f'{df.ros_type} {df.name}')
            else:
                lines.append(f'{df.ros_type} {df.name}')
        if not lines:
            lines.append('# (no fields)')
        return lines

    def _write_message_msg(self, msg_dir: Path, msg: ProtoMessage) -> Path:
        path = msg_dir / f'{pascal(msg.name)}.msg'
        header = [
            '# Auto-generated from .proto -- do not edit',
            f'# proto message {msg.name}',
        ]
        body = self._message_body(msg)
        path.write_text('\n'.join(header + body) + '\n', encoding='utf-8')
        return path

    # -- services -------------------------------------------------------------

    def _service_base_name(self, pf: ProtoFile, svc_name: str, rpc_name: str) -> str:
        parts = [p for p in pf.package.split('.') if p]
        comp = parts[2] if len(parts) > 2 else ''
        role = parts[4] if len(parts) > 4 else ''
        svc = svc_name
        for suffix in ('_Service', 'Service'):
            if svc.endswith(suffix):
                svc = svc[: -len(suffix)]
                break
        return pascal(comp) + pascal(role) + pascal(svc) + pascal(rpc_name)

    def _ref_token(self, type_name: str) -> Optional[str]:
        """ROS2 token for a service request/response type (None = empty)."""
        if _is_empty_type(type_name):
            return None
        short = type_name.split('.')[-1]
        if self.ir.is_alias(short):
            cpp = self.ir.aliases[short]
            return 'string' if cpp == 'std::string' else CPP_TO_ROS[cpp]
        if self.ir.is_enum(short) or self.index.resolve_message(short) is not None:
            return pascal(short)
        raise NotImplementedError(f'unresolved service type {type_name!r}')

    def _section(self, type_name: str, field: str) -> List[str]:
        token = self._ref_token(type_name)
        if token is None:
            return [f'# (empty {field})']
        return [f'{token} {field}']

    def _write_rpc(self, msg_dir: Path, srv_dir: Path, pf: ProtoFile,
                   svc, rpc) -> List[Path]:
        base = self._service_base_name(pf, svc.name, rpc.name)
        header = [
            '# Auto-generated from .proto -- do not edit',
            f'# {pf.package} {svc.name}.{rpc.name}',
        ]

        if rpc.server_streaming:
            srv_path = srv_dir / f'{base}.srv'
            srv_lines = list(header) + self._section(rpc.request_type, 'request') + [
                '---', 'bool accepted', 'int32 status', 'string correlation_id']
            srv_path.write_text('\n'.join(srv_lines) + '\n', encoding='utf-8')

            frame_path = msg_dir / f'{base}Frame.msg'
            frame_lines = list(header) + [
                'string correlation_id', 'bool end_of_stream', 'int32 status']
            frame_lines += self._section(rpc.response_type, 'data')
            frame_path.write_text('\n'.join(frame_lines) + '\n', encoding='utf-8')
            return [srv_path, frame_path]

        srv_path = srv_dir / f'{base}.srv'
        srv_lines = list(header) + self._section(rpc.request_type, 'request')
        srv_lines.append('---')
        srv_lines += self._section(rpc.response_type, 'response')
        srv_path.write_text('\n'.join(srv_lines) + '\n', encoding='utf-8')
        return [srv_path]


def generate_ros2_idl(index: ProtoTypeIndex, output_dir: Path,
                      service_modules: Optional[List[ProtoFile]] = None) -> List[Path]:
    """Module entry point used by the binding generator and tests."""
    return Ros2IdlGenerator(index, service_modules).generate(output_dir)


def _main(argv: List[str]) -> int:
    if len(argv) < 3:
        print('Usage: python ros2_idl_codegen.py <proto_dir> <out_dir>',
              file=sys.stderr)
        return 2
    from proto_parser import parse_proto_tree  # noqa: E402
    index = ProtoTypeIndex(parse_proto_tree(Path(argv[1])))
    written = generate_ros2_idl(index, Path(argv[2]))
    print(f'Wrote {len(written)} ROS2 IDL files to {argv[2]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(_main(sys.argv))
