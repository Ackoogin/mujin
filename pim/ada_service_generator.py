#!/usr/bin/env python3
"""
Ada Service Stub Generator
Generates PCL-aligned EntityActions service stubs from a .proto IDL file.

Each rpc in a proto service block produces a Handle_<Op>_<Entity> procedure
matching the EntityActions CRUD contract:

  CreateXxx(Xxx)            returns (Identifier)   -> Handle_Create_Xxx
  ReadXxx(XxxQuery)         returns (stream Xxx)   -> Handle_Read_Xxx
  UpdateXxx(Xxx)            returns (Ack)           -> Handle_Update_Xxx
  DeleteXxx(Identifier)     returns (Ack)           -> Handle_Delete_Xxx

A Dispatch procedure is generated as the single integration point for any
transport (PCL, socket, shared memory, etc.) -- it routes an incoming
operation+channel to the correct typed handler.

Generated packages reference Tactical_Objects_Types for Ada type definitions.

Service wire-name constants and JSON builder functions are generated for
standard pyramid protocol interaction (GNATCOLL.JSON).

Usage:
    python ada_service_generator.py <file.proto> <output_dir>
    python ada_service_generator.py <proto_dir/>  <output_dir>
"""

import sys
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# -- EntityActions operation set -----------------------------------------------

OP_PREFIXES = ['Create', 'Read', 'Update', 'Delete']

# Base-type short names from pyramid.data_model.base.* and common.*
BASE_TYPE_MAP = {
    'pyramid.data_model.base.Identifier': 'Identifier',
    'pyramid.data_model.base.Query':      'Query',
    'pyramid.data_model.base.Ack':        'Ack',
    'pyramid.data_model.common.Query':    'Query',
    'pyramid.data_model.common.Ack':      'Ack',
    'pyramid.data_model.common.Capability': 'Identifier',
}

# Wire-name service prefix derived from proto service name
# e.g. Object_Of_Interest_Service -> "object_of_interest"
# RPC CreateRequirement -> "create_requirement"
# Full wire name: "object_of_interest.create_requirement"

# Standard topic names by convention
STANDARD_TOPICS = {
    'provided': {
        'entity_matches': 'standard.entity_matches',
        'evidence_requirements': 'standard.evidence_requirements',
    },
    'consumed': {
        'object_evidence': 'standard.object_evidence',
    },
}


# -- Proto parser --------------------------------------------------------------

def _strip_comments(text: str) -> str:
    """Remove // line comments and /* */ block comments."""
    text = re.sub(r'//[^\n]*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text


def _camel_to_snake(name: str) -> str:
    """TacticalObject -> Tactical_Object  (Ada identifier style)."""
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    s = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s)
    return s


def _camel_to_lower_snake(name: str) -> str:
    """ObjectOfInterest -> object_of_interest (wire-format style)."""
    return _camel_to_snake(name).lower()


def _short_type(full_type: str) -> str:
    """pyramid.data_model.base.Identifier -> Identifier."""
    if full_type in BASE_TYPE_MAP:
        return BASE_TYPE_MAP[full_type]
    return full_type.split('.')[-1]


def _proto_type_to_ada(full_type: str) -> str:
    """Map a proto type to its Ada equivalent in Tactical_Objects_Types."""
    short = _short_type(full_type)
    # Apply CamelCase -> Ada_Style conversion for compound names
    return _camel_to_snake(short)


def _service_wire_prefix(service_name: str) -> str:
    """Object_Of_Interest_Service -> object_of_interest."""
    # Remove _Service suffix if present
    name = service_name
    if name.endswith('_Service'):
        name = name[:-len('_Service')]
    return _camel_to_lower_snake(name)


class ProtoRpc:
    """One rpc entry extracted from a proto service block."""

    def __init__(self, name: str, req: str, rsp: str, streaming: bool):
        self.name = name
        self.req = req
        self.rsp = rsp
        self.streaming = streaming

        self.op: Optional[str] = None
        self.entity: Optional[str] = None
        for prefix in OP_PREFIXES:
            if name.startswith(prefix):
                self.op = prefix
                self.entity = name[len(prefix):]
                break

    @property
    def ada_handler(self) -> str:
        return f'Handle_{self.op}_{_camel_to_snake(self.entity)}'

    @property
    def ada_channel(self) -> str:
        return f'Ch_{self.op}_{_camel_to_snake(self.entity)}'

    @property
    def ada_req_type(self) -> str:
        return _proto_type_to_ada(self.req)

    @property
    def ada_rsp_type(self) -> str:
        base = _proto_type_to_ada(self.rsp)
        if self.streaming:
            return f'{base}_Array'
        return base

    @property
    def wire_name(self) -> str:
        """create_requirement (rpc part of wire name)."""
        return _camel_to_lower_snake(self.name)


class ProtoService:
    def __init__(self, name: str, rpcs: List[ProtoRpc]):
        self.name = name
        self.rpcs = [r for r in rpcs if r.op is not None]

    @property
    def wire_prefix(self) -> str:
        return _service_wire_prefix(self.name)


class ProtoFile:
    def __init__(self, package: str, services: List[ProtoService]):
        self.package = package
        self.services = services


def parse_proto(proto_path: Path) -> ProtoFile:
    text = _strip_comments(proto_path.read_text(encoding='utf-8'))

    pkg_match = re.search(r'\bpackage\s+([\w.]+)\s*;', text)
    package = pkg_match.group(1) if pkg_match else ''

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
    parts = proto_file.package.split('.')

    last = parts[-1].lower()
    explicit_suffix = None
    if last in ('provided', 'consumed'):
        explicit_suffix = parts[-1].capitalize()
        parts = parts[:-1]

    skip = {'pyramid', 'components', 'data_model', 'base', 'services'}
    meaningful = [p for p in parts if p.lower() not in skip]

    ada_parts = ['Pyramid', 'Services']
    for p in meaningful:
        titled = '_'.join(w.capitalize() for w in p.split('_'))
        ada_parts.append(titled)

    ada_parts.append(explicit_suffix if explicit_suffix else 'Provided')
    return '.'.join(ada_parts)


def _is_provided(proto_file: ProtoFile) -> bool:
    return 'provided' in proto_file.package.lower()


# -- Code generation -----------------------------------------------------------

class AdaServiceGenerator:

    def __init__(self, proto_input: str):
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
            _proto_type_to_ada(_short_type(rpc.rsp))
            for _, rpc in all_rpcs if rpc.streaming
        })

        is_provided = _is_provided(parsed)

        with open(path, 'w') as f:
            f.write(f'--  Auto-generated EntityActions service specification\n')
            f.write(f'--  Generated from: {self._proto_input.name}'
                    f' by ada_service_generator.py\n')
            f.write(f'--  Package: {pkg_name}\n')
            f.write(f'--\n')
            f.write(f'--  Each Handle_<Op>_<Entity> procedure corresponds to one EntityActions\n')
            f.write(f'--  CRUD operation.  The Dispatch procedure is the single integration\n')
            f.write(f'--  point for any transport (PCL, socket, shared memory, etc.).\n')
            f.write(f'\n')
            f.write(f'with Tactical_Objects_Types;  use Tactical_Objects_Types;\n')
            f.write(f'with System;\n')
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

            # Service_Channel enumeration
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

            # -- Service wire-name constants -----------------------------------
            f.write(f'   --  -- Service wire-name constants (generated from proto) --------\n')
            f.write(f'\n')

            for svc in parsed.services:
                prefix = svc.wire_prefix
                for rpc in svc.rpcs:
                    const_name = f'Svc_{_camel_to_snake(rpc.name)}'
                    wire_name = f'{prefix}.{rpc.wire_name}'
                    f.write(f'   {const_name} : constant String :=\n')
                    f.write(f'     "{wire_name}";\n')
            f.write('\n')

            # -- Topic name constants ------------------------------------------
            topic_set = STANDARD_TOPICS.get(
                'provided' if is_provided else 'consumed', {})
            if topic_set:
                f.write(f'   --  -- Standard topic name constants --------------------------\n')
                f.write(f'\n')
                for key, wire in topic_set.items():
                    const_name = f'Topic_{_camel_to_snake(key).title().replace(" ", "_")}'
                    # Title case each word
                    const_name = 'Topic_' + '_'.join(
                        w.capitalize() for w in key.split('_'))
                    f.write(f'   {const_name} : constant String :=\n')
                    f.write(f'     "{wire}";\n')
                f.write('\n')

            # Handler procedure declarations
            f.write(f'   --  -- EntityActions handlers ------------------------------------\n')
            f.write(f'   --  Implement these procedures in the package body.\n')
            f.write(f'\n')

            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'   --  {svc_name}\n')
                    current_svc = svc_name

                req_t = rpc.ada_req_type
                rsp_t = rpc.ada_rsp_type

                f.write(f'   procedure {rpc.ada_handler}\n')
                f.write(f'     (Request  : in  {req_t};\n')
                f.write(f'      Response : out {rsp_t});\n')

            f.write('\n')

            # -- JSON builder functions (provided only) ------------------------
            if is_provided:
                f.write(f'   --  -- JSON builder functions (GNATCOLL.JSON) -----------------\n')
                f.write(f'\n')
                f.write(f'   function Build_Standard_Requirement_Json\n')
                f.write(f'     (Policy      : String;\n')
                f.write(f'      Identity    : String;\n')
                f.write(f'      Dimension   : String := "";\n')
                f.write(f'      Min_Lat_Rad : Long_Float := 0.0;\n')
                f.write(f'      Max_Lat_Rad : Long_Float := 0.0;\n')
                f.write(f'      Min_Lon_Rad : Long_Float := 0.0;\n')
                f.write(f'      Max_Lon_Rad : Long_Float := 0.0) return String;\n')
                f.write(f'\n')
                f.write(f'   function Build_Standard_Evidence_Json\n')
                f.write(f'     (Identity    : String;\n')
                f.write(f'      Dimension   : String;\n')
                f.write(f'      Lat_Rad     : Long_Float;\n')
                f.write(f'      Lon_Rad     : Long_Float;\n')
                f.write(f'      Confidence  : Long_Float;\n')
                f.write(f'      Observed_At : Long_Float := 0.5) return String;\n')
                f.write(f'\n')

            # Dispatch procedure
            f.write(f'   --  -- Transport integration point ------------------------------\n')
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
        is_provided = _is_provided(parsed)

        with open(path, 'w') as f:
            f.write(f'--  Auto-generated EntityActions service body\n')
            f.write(f'--  Package body: {pkg_name}\n')
            f.write(f'\n')
            if is_provided:
                f.write(f'with GNATCOLL.JSON;  use GNATCOLL.JSON;\n')
            f.write(f'with System;\n')
            f.write(f'\n')
            f.write(f'package body {pkg_name} is\n')
            f.write(f'\n')

            # Handler stubs
            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'   --  -- {svc_name} ------------------------------------\n')
                    current_svc = svc_name

                req_t = rpc.ada_req_type
                rsp_t = rpc.ada_rsp_type

                f.write(f'   procedure {rpc.ada_handler}\n')
                f.write(f'     (Request  : in  {req_t};\n')
                f.write(f'      Response : out {rsp_t})\n')
                f.write(f'   is\n')
                f.write(f'      pragma Unreferenced (Request);\n')

                # Generate sensible default return values
                if rsp_t == 'Identifier':
                    f.write(f'   begin\n')
                    f.write(f'      Response := Null_Identifier;\n')
                elif rsp_t == 'Ack':
                    f.write(f'   begin\n')
                    f.write(f'      Response := Ack_Ok;\n')
                elif rsp_t.endswith('_Array'):
                    f.write(f'      Empty : {rsp_t} (1 .. 0);\n')
                    f.write(f'   begin\n')
                    f.write(f'      Response := Empty;\n')
                else:
                    f.write(f'      Default_Val : {rsp_t};\n')
                    f.write(f'   begin\n')
                    f.write(f'      Response := Default_Val;\n')

                f.write(f'   end {rpc.ada_handler};\n')
                f.write(f'\n')

            # -- JSON builder implementations (provided only) ------------------
            if is_provided:
                f.write(f'   --  -- JSON builder: Build_Standard_Requirement_Json ----------\n')
                f.write(f'\n')
                f.write(f'   function Build_Standard_Requirement_Json\n')
                f.write(f'     (Policy      : String;\n')
                f.write(f'      Identity    : String;\n')
                f.write(f'      Dimension   : String := "";\n')
                f.write(f'      Min_Lat_Rad : Long_Float := 0.0;\n')
                f.write(f'      Max_Lat_Rad : Long_Float := 0.0;\n')
                f.write(f'      Min_Lon_Rad : Long_Float := 0.0;\n')
                f.write(f'      Max_Lon_Rad : Long_Float := 0.0) return String\n')
                f.write(f'   is\n')
                f.write(f'      Obj : JSON_Value := Create_Object;\n')
                f.write(f'   begin\n')
                f.write(f'      Set_Field (Obj, "policy",   Policy);\n')
                f.write(f'      Set_Field (Obj, "identity", Identity);\n')
                f.write(f'      if Dimension /= "" then\n')
                f.write(f'         Set_Field (Obj, "dimension", Dimension);\n')
                f.write(f'      end if;\n')
                f.write(f'      Set_Field_Long_Float (Obj, "min_lat_rad", Min_Lat_Rad);\n')
                f.write(f'      Set_Field_Long_Float (Obj, "max_lat_rad", Max_Lat_Rad);\n')
                f.write(f'      Set_Field_Long_Float (Obj, "min_lon_rad", Min_Lon_Rad);\n')
                f.write(f'      Set_Field_Long_Float (Obj, "max_lon_rad", Max_Lon_Rad);\n')
                f.write(f'      return Write (Obj);\n')
                f.write(f'   end Build_Standard_Requirement_Json;\n')
                f.write(f'\n')

                f.write(f'   --  -- JSON builder: Build_Standard_Evidence_Json -------------\n')
                f.write(f'\n')
                f.write(f'   function Build_Standard_Evidence_Json\n')
                f.write(f'     (Identity    : String;\n')
                f.write(f'      Dimension   : String;\n')
                f.write(f'      Lat_Rad     : Long_Float;\n')
                f.write(f'      Lon_Rad     : Long_Float;\n')
                f.write(f'      Confidence  : Long_Float;\n')
                f.write(f'      Observed_At : Long_Float := 0.5) return String\n')
                f.write(f'   is\n')
                f.write(f'      Obj : JSON_Value := Create_Object;\n')
                f.write(f'   begin\n')
                f.write(f'      Set_Field (Obj, "identity",      Identity);\n')
                f.write(f'      Set_Field (Obj, "dimension",     Dimension);\n')
                f.write(f'      Set_Field_Long_Float (Obj, "latitude_rad",  Lat_Rad);\n')
                f.write(f'      Set_Field_Long_Float (Obj, "longitude_rad", Lon_Rad);\n')
                f.write(f'      Set_Field_Long_Float (Obj, "confidence",    Confidence);\n')
                f.write(f'      Set_Field_Long_Float (Obj, "observed_at",   Observed_At);\n')
                f.write(f'      return Write (Obj);\n')
                f.write(f'   end Build_Standard_Evidence_Json;\n')
                f.write(f'\n')

            # Dispatch stub with case statement
            f.write(f'   procedure Dispatch\n')
            f.write(f'     (Channel      : in  Service_Channel;\n')
            f.write(f'      Request_Buf  : in  System.Address;\n')
            f.write(f'      Request_Size : in  Natural;\n')
            f.write(f'      Response_Buf : out System.Address;\n')
            f.write(f'      Response_Size: out Natural)\n')
            f.write(f'   is\n')
            f.write(f'      pragma Unreferenced (Request_Buf, Request_Size);\n')
            f.write(f'   begin\n')
            f.write(f'      Response_Buf  := System.Null_Address;\n')
            f.write(f'      Response_Size := 0;\n')
            f.write(f'      case Channel is\n')

            for _, rpc in all_rpcs:
                f.write(f'         when {rpc.ada_channel} =>\n')
                f.write(f'            null;  --  TODO: deserialise, call {rpc.ada_handler}\n')

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
    print('\n\u2713 Ada services generated')


if __name__ == '__main__':
    main()
