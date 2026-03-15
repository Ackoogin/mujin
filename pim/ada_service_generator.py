#!/usr/bin/env python3
"""
Ada Service Stub Generator
Generates event-driven service stubs for SysML services
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re


class AdaServiceGenerator:
    
    def __init__(self, datamodel_file: str):
        with open(datamodel_file, 'r') as f:
            self.data = json.load(f)
    
    def generate(self, output_dir: str):
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Group services by component and type (provided/consumed)
        services_grouped = self._group_services_by_component_and_type()
        
        for (component_ns, service_type), services in services_grouped.items():
            if not services:
                continue
            
            # Collect all entity properties from all services in this group
            all_service_props = []
            for service in services:
                entity_props = self._collect_all_entity_properties(service)
                if entity_props:
                    all_service_props.append({
                        'service_name': service['name'],
                        'properties': entity_props
                    })
            
            if all_service_props:
                self._generate_grouped_service(output_path, component_ns, service_type, all_service_props)
    
    def _group_services_by_component_and_type(self):
        """Group services by component and service type (provided/consumed)"""
        grouped = {}
        
        for cls in self.data.get('classes', []):
            ns = cls.get('namespace', [])
            
            # Determine component and service type from namespace
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
                if key not in grouped:
                    grouped[key] = []
                grouped[key].append(cls)
        
        return grouped
    
    def _collect_all_entity_properties(self, cls):
        all_props = []
        for parent_name in cls.get('generalizes', []):
            parent_class = self._find_class_by_name(parent_name)
            if parent_class:
                all_props.extend(self._collect_all_entity_properties(parent_class))
        
        for prop in cls.get('properties', []):
            type_name = prop.get('typeName')
            if type_name and self._is_entity_derived(type_name):
                all_props.append({
                    'name': prop['name'],
                    'type': type_name,
                    'flow_direction': prop.get('flow_direction', 'inout')
                })
        return all_props
    
    def _find_class_by_name(self, class_name: str):
        for cls in self.data.get('classes', []):
            if cls['name'] == class_name:
                return cls
        return None
    
    def _is_entity_derived(self, type_name: str):
        for dt in self.data.get('dataTypes', []):
            if dt['name'] == type_name:
                if 'Entity' in dt.get('generalizes', []):
                    return True
                for parent in dt.get('generalizes', []):
                    if self._is_entity_derived(parent):
                        return True
        return False
    
        if service_type:
            parts.append(service_type)
        
        # Clean service name
        clean_service = re.sub(r'[^\w]', '_', service_name)
        clean_service = re.sub(r'^\d+', '', clean_service)
        if clean_service:
            parts.append(clean_service)
        
        return '.'.join(parts)
    
    def _generate_grouped_service(self, output_path, component_ns, service_type, all_service_props):
        """Generate one package for all provided/consumed services in a component"""
        # Build package name: Pyramid.Services.Component.Provided/Consumed
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
        
        # Generate .ads
        ads_file = pkg_name.lower().replace('.', '-') + '.ads'
        with open(output_path / ads_file, 'w') as f:
            f.write(f"--  {service_type} Services\n")
            f.write("with System;\nwith Pyramid.Middleware;\n\n")
            f.write(f"package {pkg_name} is\n\n")
            
            # Channels for all services
            f.write("   --  Channels\n")
            channel_base = 100
            for svc_info in all_service_props:
                for prop in svc_info['properties']:
                    svc_name = svc_info['service_name']
                    prop_name = prop['name']
                    f.write(f"   Ch_{svc_name}_{prop_name}_Req : constant := {channel_base};\n")
                    if prop['flow_direction'] in ['inout', 'out']:
                        f.write(f"   Ch_{svc_name}_{prop_name}_Rsp : constant := {channel_base+1};\n")
                    channel_base += 10
            
            # Handlers for all services
            f.write("\n   --  Handlers\n")
            for svc_info in all_service_props:
                svc_name = svc_info['service_name']
                f.write(f"   --  {svc_name}\n")
                for prop in svc_info['properties']:
                    prop_name = prop['name']
                    prop_type = prop['type']
                    flow = prop['flow_direction']
                    f.write(f"   procedure On_{svc_name}_{prop_name}(Req : {prop_type}")
                    if flow in ['inout', 'out']:
                        f.write(f"; Rsp : out {prop_type}")
                    f.write(");\n")
                f.write("\n")
            
            f.write("   procedure Run_Loop;\n")
            f.write(f"end {pkg_name};\n")
        
        # Generate .adb
        adb_file = pkg_name.lower().replace('.', '-') + '.adb'
        with open(output_path / adb_file, 'w') as f:
            f.write(f"package body {pkg_name} is\n\n")
            
            # Handler implementations
            for svc_info in all_service_props:
                svc_name = svc_info['service_name']
                f.write(f"   --  {svc_name} handlers\n")
                for prop in svc_info['properties']:
                    prop_name = prop['name']
                    prop_type = prop['type']
                    flow = prop['flow_direction']
                    f.write(f"   procedure On_{svc_name}_{prop_name}(Req : {prop_type}")
                    if flow in ['inout', 'out']:
                        f.write(f"; Rsp : out {prop_type}")
                    f.write(") is\n   begin\n      null;\n   end;\n\n")
            
            # Event loop
            f.write("   procedure Run_Loop is\n   begin\n      loop\n         null;\n      end loop;\n   end;\n\n")
            f.write(f"end {pkg_name};\n")
        
        print(f"  Generated {pkg_name}")
        
        print(f"  Generated {pkg_name}")


def main():
    if len(sys.argv) < 3:
        print("Usage: python ada_service_generator.py <datamodel.json> <output_dir>")
        sys.exit(1)
    
    gen = AdaServiceGenerator(sys.argv[1])
    gen.generate(sys.argv[2])
    print("\n✓ Ada services generated")


if __name__ == '__main__':
    main()
