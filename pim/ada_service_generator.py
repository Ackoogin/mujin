#!/usr/bin/env python3
"""
Ada Service Stub Generator
Generates PCL-aligned EntityActions service stubs from the SysML JSON IR.

Each entity property with flow direction (inout/out/in) produces a set of
Handle_<Op>_<Entity> procedures matching the EntityActions CRUD contract:

  inout → Handle_Create, Handle_Read, Handle_Update, Handle_Delete
  out   → Handle_Read only
  in    → Handle_Create, Handle_Update, Handle_Delete

A Dispatch procedure is generated as the single integration point for any
transport (PCL, socket, shared memory, etc.) — it routes an incoming
operation+channel to the correct typed handler.

This replaces the former Pyramid.Middleware Send/Receive/Receive_Any pattern.
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re


# ── EntityActions operation set per flow direction ───────────────────────────

FLOW_OPS = {
    'inout': ['Create', 'Read', 'Update', 'Delete'],
    'out':   ['Read'],
    'in':    ['Create', 'Update', 'Delete'],
    None:    ['Create', 'Read', 'Update', 'Delete'],  # default = inout
}

# Ada parameter signature per operation (Req type, optional Rsp type)
# Rsp is None when the operation has no response output parameter.
OP_SIGNATURE = {
    'Create': {'req': '{entity_type}',      'rsp': 'Identifier'},
    'Read':   {'req': 'Query',              'rsp': '{entity_type}_Array'},
    'Update': {'req': '{entity_type}',      'rsp': 'Ack'},
    'Delete': {'req': 'Identifier',         'rsp': 'Ack'},
}


class AdaServiceGenerator:

    def __init__(self, datamodel_file: str):
        with open(datamodel_file, 'r') as f:
            self.data = json.load(f)

    def generate(self, output_dir: str):
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        groups = self._group_services_by_component_and_type()

        for (component_ns, service_type), services in groups.items():
            all_service_props = []
            for service in services:
                entity_props = self._collect_all_entity_properties(service)
                if entity_props:
                    all_service_props.append({
                        'service_name': service['name'],
                        'properties': entity_props,
                    })

            if all_service_props:
                self._generate_grouped_service(
                    output_path, component_ns, service_type, all_service_props)

    # ── Grouping ──────────────────────────────────────────────────────────────

    def _group_services_by_component_and_type(self) -> Dict:
        grouped = {}

        for cls in self.data.get('classes', []):
            ns = cls.get('namespace', [])
            component_parts = []
            service_type = None

            for part in ns:
                part_lower = part.lower()
                if 'provided' in part_lower or 'offered' in part_lower:
                    service_type = 'Provided'
                    break
                elif 'consumed' in part_lower or 'required' in part_lower:
                    service_type = 'Consumed'
                    break
                elif 'service' in part_lower:
                    continue
                else:
                    component_parts.append(part)

            if service_type:
                key = (tuple(component_parts), service_type)
                grouped.setdefault(key, []).append(cls)

        return grouped

    def _collect_all_entity_properties(self, cls: Dict) -> List[Dict]:
        """Collect entity properties from this class and all ancestors."""
        props = []
        for parent_name in cls.get('generalizes', []):
            parent = self._find_class_by_name(parent_name)
            if parent:
                props.extend(self._collect_all_entity_properties(parent))

        for prop in cls.get('properties', []):
            type_name = prop.get('typeName')
            if type_name and self._is_entity_derived(type_name):
                props.append({
                    'name': prop['name'],
                    'type': type_name,
                    'flow_direction': prop.get('flow_direction'),  # None = inout
                })
        return props

    def _find_class_by_name(self, name: str) -> Optional[Dict]:
        for cls in self.data.get('classes', []):
            if cls['name'] == name:
                return cls
        return None

    def _is_entity_derived(self, type_name: str) -> bool:
        for dt in self.data.get('dataTypes', []):
            if dt['name'] == type_name:
                if 'Entity' in dt.get('generalizes', []):
                    return True
                for parent in dt.get('generalizes', []):
                    if self._is_entity_derived(parent):
                        return True
        return False

    # ── Code generation ───────────────────────────────────────────────────────

    def _generate_grouped_service(self, output_path: Path, component_ns: tuple,
                                   service_type: str, all_service_props: List[Dict]):
        """Generate one .ads/.adb pair for all provided/consumed services in a component."""
        pkg_parts = ['Pyramid', 'Services']

        for part in component_ns:
            clean = re.sub(r'^\d+\.\s*', '', part)
            clean = re.sub(r'[^\w\s]', '', clean)
            clean = re.sub(r'\s+', '_', clean)
            clean = re.sub(r'^\d+', '', clean)
            if clean:
                pkg_parts.append(clean)

        pkg_parts.append(service_type)
        pkg_name = '.'.join(pkg_parts)

        ads_path = output_path / (pkg_name.lower().replace('.', '-') + '.ads')
        adb_path = output_path / (pkg_name.lower().replace('.', '-') + '.adb')

        self._write_spec(ads_path, pkg_name, all_service_props)
        self._write_body(adb_path, pkg_name, all_service_props)
        print(f'  Generated {pkg_name}')

    def _all_ops(self, all_service_props: List[Dict]) -> List[Dict]:
        """Flatten to a list of (op, entity_type, prop_name) records."""
        ops = []
        for svc in all_service_props:
            for prop in svc['properties']:
                flow = prop['flow_direction']
                for op in FLOW_OPS.get(flow, FLOW_OPS[None]):
                    ops.append({
                        'op': op,
                        'entity_type': prop['type'],
                        'prop_name': prop['name'],
                        'service_name': svc['service_name'],
                    })
        return ops

    def _proc_name(self, op: str, prop_name: str) -> str:
        """Produce a stable Ada identifier: Handle_<Op>_<PropName>."""
        clean_prop = re.sub(r'\s+', '_', prop_name)
        clean_prop = re.sub(r'[^\w]', '_', clean_prop)
        return f'Handle_{op}_{clean_prop}'

    def _req_type(self, op: str, entity_type: str) -> str:
        sig = OP_SIGNATURE[op]['req']
        return sig.replace('{entity_type}', entity_type)

    def _rsp_type(self, op: str, entity_type: str) -> Optional[str]:
        sig = OP_SIGNATURE[op].get('rsp')
        if sig is None:
            return None
        return sig.replace('{entity_type}', entity_type)

    # ── Spec (.ads) ───────────────────────────────────────────────────────────

    def _write_spec(self, path: Path, pkg_name: str, all_service_props: List[Dict]):
        ops = self._all_ops(all_service_props)

        # Collect unique entity types for array type declarations
        entity_types = sorted({o['entity_type'] for o in ops if o['op'] == 'Read'})

        with open(path, 'w') as f:
            f.write(f'--  Auto-generated EntityActions service specification\n')
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

            # Operation kind enumeration
            f.write(f'   type Operation_Kind is\n')
            f.write(f'     (Op_Create,\n')
            f.write(f'      Op_Read,\n')
            f.write(f'      Op_Update,\n')
            f.write(f'      Op_Delete);\n')
            f.write(f'\n')

            # Service channel enumeration — one per (op, entity)
            f.write(f'   type Service_Channel is\n')
            channel_values = []
            for o in ops:
                channel_values.append(f'      Ch_{o["op"]}_{o["prop_name"]}')
            if channel_values:
                f.write(',\n'.join(channel_values))
                f.write(');\n')
            else:
                f.write('      Ch_None);\n')
            f.write('\n')

            # Array types for Read responses
            for et in entity_types:
                f.write(f'   type {et}_Array is array (Positive range <>) of {et};\n')
            if entity_types:
                f.write('\n')

            # Handler procedure declarations
            f.write(f'   --  ── EntityActions handlers ─────────────────────────────────────\n')
            f.write(f'   --  Implement these procedures in the package body.\n')
            f.write(f'\n')

            for svc in all_service_props:
                f.write(f'   --  {svc["service_name"]}\n')
                flow_ops = []
                for prop in svc['properties']:
                    flow = prop['flow_direction']
                    for op in FLOW_OPS.get(flow, FLOW_OPS[None]):
                        flow_ops.append((op, prop))

                for op, prop in flow_ops:
                    proc = self._proc_name(op, prop['name'])
                    req_t = self._req_type(op, prop['type'])
                    rsp_t = self._rsp_type(op, prop['type'])

                    if rsp_t:
                        f.write(f'   procedure {proc}\n')
                        f.write(f'     (Request  : in  {req_t};\n')
                        f.write(f'      Response : out {rsp_t});\n')
                    else:
                        f.write(f'   procedure {proc}\n')
                        f.write(f'     (Request : in {req_t});\n')
                f.write('\n')

            # Dispatch procedure
            f.write(f'   --  ── Transport integration point ─────────────────────────────────\n')
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

    # ── Body (.adb) ───────────────────────────────────────────────────────────

    def _write_body(self, path: Path, pkg_name: str, all_service_props: List[Dict]):
        ops = self._all_ops(all_service_props)

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
            for svc in all_service_props:
                f.write(f'   --  ── {svc["service_name"]} ─────────────────────────────────────\n')
                for prop in svc['properties']:
                    flow = prop['flow_direction']
                    for op in FLOW_OPS.get(flow, FLOW_OPS[None]):
                        proc = self._proc_name(op, prop['name'])
                        req_t = self._req_type(op, prop['type'])
                        rsp_t = self._rsp_type(op, prop['type'])

                        if rsp_t:
                            f.write(f'   procedure {proc}\n')
                            f.write(f'     (Request  : in  {req_t};\n')
                            f.write(f'      Response : out {rsp_t})\n')
                        else:
                            f.write(f'   procedure {proc}\n')
                            f.write(f'     (Request : in {req_t})\n')
                        f.write(f'   is\n')
                        f.write(f'   begin\n')
                        f.write(f'      null;  --  TODO: implement\n')
                        f.write(f'   end {proc};\n')
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

            for o in ops:
                proc = self._proc_name(o['op'], o['prop_name'])
                f.write(f'         when Ch_{o["op"]}_{o["prop_name"]} =>\n')
                f.write(f'            null;  --  TODO: deserialise Request_Buf, call {proc}\n')

            f.write(f'      end case;\n')
            f.write(f'   end Dispatch;\n')
            f.write(f'\n')
            f.write(f'end {pkg_name};\n')


def main():
    if len(sys.argv) < 3:
        print('Usage: python ada_service_generator.py <datamodel.json> <output_dir>')
        sys.exit(1)

    gen = AdaServiceGenerator(sys.argv[1])
    gen.generate(sys.argv[2])
    print('\n✓ Ada services generated')


if __name__ == '__main__':
    main()
