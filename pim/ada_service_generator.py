#!/usr/bin/env python3
"""
Ada Service Stub Generator
Generates PCL-aligned EntityActions service stubs from a .proto IDL file.

Each rpc in a proto service block produces a Handle_<Op>_<Entity> procedure
matching the EntityActions CRUD contract:

  CreateXxx(Xxx)            returns (Identifier)   → Handle_Create_Xxx
  ReadXxx(XxxQuery)         returns (stream Xxx)   → Handle_Read_Xxx
  UpdateXxx(Xxx)            returns (Ack)           → Handle_Update_Xxx
  DeleteXxx(Identifier)     returns (Ack)           → Handle_Delete_Xxx

A Dispatch procedure is generated as the single integration point for any
transport (PCL, socket, shared memory, etc.) — it routes an incoming
operation+channel to the correct typed handler.

This replaces the former Pyramid.Middleware Send/Receive/Receive_Any pattern.

Usage:
    python ada_service_generator.py <file.proto> <output_dir>
    python ada_service_generator.py <proto_dir/>  <output_dir>
"""

import sys
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# -- EntityActions operation set -----------------------------------------------

# Maps rpc name prefix → op kind
OP_PREFIXES = ['Create', 'Read', 'Update', 'Delete']

# Ada parameter signature per operation (req type, optional rsp type)
# {entity} is substituted with the entity name (CamelCase Ada form)
OP_SIGNATURE: Dict[str, Dict] = {
    'Create': {'req': '{entity}',       'rsp': 'Identifier'},
    'Read':   {'req': '{entity}Query',  'rsp': '{entity}_Array'},
    'Update': {'req': '{entity}',       'rsp': 'Ack'},
    'Delete': {'req': 'Identifier',     'rsp': 'Ack'},
}

# Base-type short names from pyramid.data_model.base.* and pyramid.data_model.common.*
BASE_TYPE_MAP = {
    'pyramid.data_model.base.Identifier': 'Identifier',
    'pyramid.data_model.base.Query':      'Query',
    'pyramid.data_model.base.Ack':        'Ack',
    'pyramid.data_model.common.Query':    'Query',
    'pyramid.data_model.common.Ack':      'Ack',
    'pyramid.data_model.common.Capability': 'Capability',
}


# -- Proto parser --------------------------------------------------------------

def _strip_comments(text: str) -> str:
    """Remove // line comments and /* */ block comments."""
    text = re.sub(r'//[^\n]*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text


def _camel_to_snake(name: str) -> str:
    """TacticalObject → Tactical_Object  (Ada identifier style)."""
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    s = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s)
    return s


def _short_type(full_type: str) -> str:
    """pyramid.data_model.base.Identifier → Identifier."""
    if full_type in BASE_TYPE_MAP:
        return BASE_TYPE_MAP[full_type]
    # strip package prefix — keep the message name
    return full_type.split('.')[-1]


class ProtoRpc:
    """One rpc entry extracted from a proto service block."""

    def __init__(self, name: str, req: str, rsp: str, streaming: bool):
        self.name = name          # e.g. "CreateTacticalObject"
        self.req = req            # e.g. "TacticalObject"
        self.rsp = rsp            # e.g. "pyramid.data_model.base.Identifier"
        self.streaming = streaming  # True when returns (stream Xxx)

        # Detect op and entity from rpc name
        self.op: Optional[str] = None
        self.entity: Optional[str] = None  # CamelCase, e.g. "TacticalObject"
        for prefix in OP_PREFIXES:
            if name.startswith(prefix):
                self.op = prefix
                self.entity = name[len(prefix):]
                break

    @property
    def ada_handler(self) -> str:
        """Handle_Create_Tactical_Object"""
        return f'Handle_{self.op}_{_camel_to_snake(self.entity)}'

    @property
    def ada_channel(self) -> str:
        """Ch_Create_Tactical_Object"""
        return f'Ch_{self.op}_{_camel_to_snake(self.entity)}'

    @property
    def ada_req_type(self) -> str:
        sig = OP_SIGNATURE[self.op]['req']
        return sig.replace('{entity}', self.entity)

    @property
    def ada_rsp_type(self) -> Optional[str]:
        sig = OP_SIGNATURE[self.op].get('rsp')
        if sig is None:
            return None
        return sig.replace('{entity}', self.entity)


class ProtoService:
    """One service block from a .proto file."""

    def __init__(self, name: str, rpcs: List[ProtoRpc]):
        self.name = name   # e.g. "TacticalObjectService"
        self.rpcs = [r for r in rpcs if r.op is not None]  # only EntityActions rpcs


class ProtoFile:
    """Parsed .proto file."""

    def __init__(self, package: str, services: List[ProtoService]):
        self.package = package    # e.g. "pyramid.components.tactical_objects"
        self.services = services


def parse_proto(proto_path: Path) -> ProtoFile:
    text = _strip_comments(proto_path.read_text(encoding='utf-8'))

    # package
    pkg_match = re.search(r'\bpackage\s+([\w.]+)\s*;', text)
    package = pkg_match.group(1) if pkg_match else ''

    # services
    services: List[ProtoService] = []
    for svc_match in re.finditer(r'\bservice\s+(\w+)\s*\{([^}]*)\}', text, re.DOTALL):
        svc_name = svc_match.group(1)
        svc_body = svc_match.group(2)

        rpcs: List[ProtoRpc] = []
        for rpc_match in re.finditer(
                r'\brpc\s+(\w+)\s*\(\s*([\w.]+)\s*\)\s*returns\s*\(\s*(stream\s+)?([\w.]+)\s*\)',
                svc_body):
            rpc_name = rpc_match.group(1)
            req_type = rpc_match.group(2)
            streaming = bool(rpc_match.group(3))
            rsp_type = rpc_match.group(4)
            rpcs.append(ProtoRpc(rpc_name, req_type, rsp_type, streaming))

        services.append(ProtoService(svc_name, rpcs))

    return ProtoFile(package, services)


# -- Ada package name derivation -----------------------------------------------

def _pkg_name_from_proto(proto_file: ProtoFile) -> str:
    """
    pyramid.components.tactical_objects.services.provided →
        Pyramid.Services.Tactical_Objects.Provided
    pyramid.components.tactical_objects.services.consumed →
        Pyramid.Services.Tactical_Objects.Consumed
    pyramid.components.tactical_objects →
        Pyramid.Services.Tactical_Objects.Provided   (legacy fallback)
    """
    parts = proto_file.package.split('.')

    # Detect explicit provided/consumed leaf
    last = parts[-1].lower()
    explicit_suffix = None
    if last in ('provided', 'consumed'):
        explicit_suffix = parts[-1].capitalize()
        parts = parts[:-1]

    # Drop well-known structural prefixes/noise words
    skip = {'pyramid', 'components', 'data_model', 'base', 'services'}
    meaningful = [p for p in parts if p.lower() not in skip]

    ada_parts = ['Pyramid', 'Services']
    for p in meaningful:
        # Title-case each underscore-separated word: tactical_objects → Tactical_Objects
        titled = '_'.join(w.capitalize() for w in p.split('_'))
        ada_parts.append(titled)

    ada_parts.append(explicit_suffix if explicit_suffix else 'Provided')
    return '.'.join(ada_parts)


# -- Code generation -----------------------------------------------------------

class AdaServiceGenerator:

    def __init__(self, proto_input: str):
        """
        proto_input: path to a .proto file or a directory of .proto files.
        """
        self._proto_input = Path(proto_input)

    def generate(self, output_dir: str):
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        proto_files: List[Path] = []
        if self._proto_input.is_dir():
            proto_files = list(self._proto_input.rglob('*.proto'))
        elif self._proto_input.is_file():
            proto_files = [self._proto_input]
        else:
            print(f'ERROR: {self._proto_input} is not a file or directory', file=sys.stderr)
            sys.exit(1)

        for pf in proto_files:
            parsed = parse_proto(pf)
            # Collect all rpcs across all services in this file
            all_rpcs: List[Tuple[str, ProtoRpc]] = []
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    all_rpcs.append((svc.name, rpc))

            if not all_rpcs:
                continue

            pkg_name = _pkg_name_from_proto(parsed)
            ads_path = output_path / (pkg_name.lower().replace('.', '-') + '.ads')
            adb_path = output_path / (pkg_name.lower().replace('.', '-') + '.adb')

            self._write_spec(ads_path, pkg_name, parsed, all_rpcs)
            self._write_body(adb_path, pkg_name, parsed, all_rpcs)
            print(f'  Generated {pkg_name}')

    # -- Spec (.ads) -----------------------------------------------------------

    def _write_spec(self, path: Path, pkg_name: str, parsed: ProtoFile,
                    all_rpcs: List[Tuple[str, ProtoRpc]]):
        entity_types_for_arrays = sorted({
            rpc.entity for _, rpc in all_rpcs if rpc.op == 'Read'
        })

        with open(path, 'w') as f:
            f.write(f'--  Auto-generated EntityActions service specification\n')
            f.write(f'--  Generated from: {self._proto_input.name}'
                    f' by ada_service_generator.py\n')
            f.write(f'--  Package: {pkg_name}\n')
            f.write(f'--\n')
            f.write(f'--  Each Handle_<Op>_<Entity> procedure corresponds to one EntityActions\n')
            f.write(f'--  CRUD operation.  The Dispatch procedure is the single integration\n')
            f.write(f'--  point for any transport (PCL, socket, shared memory, etc.).\n')
            f.write(f'--  DO NOT add Pyramid.Middleware.Send/Receive calls here.\n')
            f.write(f'\n')
            f.write(f'with Pyramid.Model;  --  Identifier, Query, Ack\n')
            f.write(f'use  Pyramid.Model;\n')
            f.write(f'\n')
            f.write(f'package {pkg_name} is\n')
            f.write(f'\n')

            # Operation_Kind enumeration
            f.write(f'   type Operation_Kind is\n')
            f.write(f'     (Op_Create,\n')
            f.write(f'      Op_Read,\n')
            f.write(f'      Op_Update,\n')
            f.write(f'      Op_Delete);\n')
            f.write(f'\n')

            # Service_Channel enumeration — one per rpc
            f.write(f'   type Service_Channel is\n')
            channel_values = [f'      {rpc.ada_channel}' for _, rpc in all_rpcs]
            if channel_values:
                f.write('     (' + ',\n'.join(channel_values).lstrip() + ');\n')
            else:
                f.write('     (Ch_None);\n')
            f.write('\n')

            # Array types for streaming Read responses
            for et in entity_types_for_arrays:
                f.write(f'   type {et}_Array is array (Positive range <>) of {et};\n')
            if entity_types_for_arrays:
                f.write('\n')

            # Handler procedure declarations grouped by service
            f.write(f'   --  -- EntityActions handlers -------------------------------------\n')
            f.write(f'   --  Implement these procedures in the package body.\n')
            f.write(f'\n')

            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'   --  {svc_name}\n')
                    current_svc = svc_name

                req_t = rpc.ada_req_type
                rsp_t = rpc.ada_rsp_type

                if rsp_t:
                    f.write(f'   procedure {rpc.ada_handler}\n')
                    f.write(f'     (Request  : in  {req_t};\n')
                    f.write(f'      Response : out {rsp_t});\n')
                else:
                    f.write(f'   procedure {rpc.ada_handler}\n')
                    f.write(f'     (Request : in {req_t});\n')

            f.write('\n')

            # Dispatch procedure
            f.write(f'   --  -- Transport integration point ---------------------------------\n')
            f.write(f'   --  Route an incoming (channel, raw buffer) call to the correct\n')
            f.write(f'   --  typed handler.  The transport layer calls this; it never calls\n')
            f.write(f'   --  Handle_* procedures directly.\n')
            f.write(f'\n')
            f.write(f'   procedure Dispatch\n')
            f.write(f'     (Channel      : in  Service_Channel;\n')
            f.write(f'      Request_Buf  : in  System.Address;\n')
            f.write(f'      Request_Size : in  Natural;\n')
            f.write(f'      Response_Buf : out System.Address;\n')
            f.write(f'      Response_Size: out Natural);\n')
            f.write(f'\n')
            f.write(f'end {pkg_name};\n')

    # -- Body (.adb) -----------------------------------------------------------

    def _write_body(self, path: Path, pkg_name: str, parsed: ProtoFile,
                    all_rpcs: List[Tuple[str, ProtoRpc]]):
        with open(path, 'w') as f:
            f.write(f'--  Auto-generated EntityActions service body\n')
            f.write(f'--  Package body: {pkg_name}\n')
            f.write(f'--  TODO: replace null stubs with real implementations.\n')
            f.write(f'\n')
            f.write(f'with System;\n')
            f.write(f'\n')
            f.write(f'package body {pkg_name} is\n')
            f.write(f'\n')

            # Handler stubs
            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'   --  -- {svc_name} -------------------------------------\n')
                    current_svc = svc_name

                req_t = rpc.ada_req_type
                rsp_t = rpc.ada_rsp_type

                if rsp_t:
                    f.write(f'   procedure {rpc.ada_handler}\n')
                    f.write(f'     (Request  : in  {req_t};\n')
                    f.write(f'      Response : out {rsp_t})\n')
                else:
                    f.write(f'   procedure {rpc.ada_handler}\n')
                    f.write(f'     (Request : in {req_t})\n')
                f.write(f'   is\n')
                f.write(f'   begin\n')
                f.write(f'      null;  --  TODO: implement\n')
                f.write(f'   end {rpc.ada_handler};\n')
                f.write(f'\n')

            # Dispatch stub with case statement
            f.write(f'   procedure Dispatch\n')
            f.write(f'     (Channel      : in  Service_Channel;\n')
            f.write(f'      Request_Buf  : in  System.Address;\n')
            f.write(f'      Request_Size : in  Natural;\n')
            f.write(f'      Response_Buf : out System.Address;\n')
            f.write(f'      Response_Size: out Natural)\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            f.write(f'      Response_Buf  := System.Null_Address;\n')
            f.write(f'      Response_Size := 0;\n')
            f.write(f'      case Channel is\n')

            for _, rpc in all_rpcs:
                f.write(f'         when {rpc.ada_channel} =>\n')
                f.write(f'            null;  --  TODO: deserialise Request_Buf, call {rpc.ada_handler}\n')

            f.write(f'      end case;\n')
            f.write(f'   end Dispatch;\n')
            f.write(f'\n')
            f.write(f'end {pkg_name};\n')


def main():
    if len(sys.argv) < 3:
        print('Usage: python ada_service_generator.py <file.proto|proto_dir> <output_dir>')
        sys.exit(1)

    gen = AdaServiceGenerator(sys.argv[1])
    gen.generate(sys.argv[2])
    print('\n✓ Ada services generated')


if __name__ == '__main__':
    main()
