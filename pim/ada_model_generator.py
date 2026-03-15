#!/usr/bin/env python3
"""
Ada Data Model Generator
Generates Ada type definitions from parsed SysML data model
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re


class AdaDataModelGenerator:
    """Generate Ada type definitions from SysML data model"""
    
    # Type mapping from SysML to Ada
    TYPE_MAPPING = {
        'String': 'Unbounded_String',
        'Boolean': 'Boolean',
        'Integer': 'Integer',
        'Real': 'Float',
    }
    
    # Base type definitions
    BASE_TYPE_DEFINITIONS = {
        'Identifier': {
            'comment': 'Universally Unique Identifier (UUID)',
            'base_type': 'Unbounded_String'
        },
        'Timestamp': {
            'comment': 'Point in time',
            'base_type': 'Ada.Calendar.Time'
        },
        'Angle': {
            'comment': 'Angle in radians',
            'base_type': 'Float',
            'unit': 'radians'
        },
        'Length': {
            'comment': 'Length in meters',
            'base_type': 'Float',
            'unit': 'meters'
        },
        'Speed': {
            'comment': 'Speed in meters per second',
            'base_type': 'Float',
            'unit': 'meters_per_second'
        },
        'Percentage': {
            'comment': 'Percentage value (0-100)',
            'base_type': 'Float',
            'unit': 'percent'
        },
        'Quality': {
            'comment': 'Quality metric (0.0-1.0)',
            'base_type': 'Float',
            'unit': 'normalized'
        }
    }
    
    def __init__(self, datamodel_file: str):
        """Load data model"""
        with open(datamodel_file, 'r') as f:
            self.data = json.load(f)
        
        # Track all type names
        self.all_types = set()
    
    def generate(self, output_dir: str):
        """Generate Ada package structure"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Collect all type names
        self._collect_all_types()
        
        # Generate by namespace
        files_by_namespace = self._group_by_namespace()
        
        for namespace, types in files_by_namespace.items():
            if not types['dataTypes'] and not types['enumerations']:
                continue
            
            # Create package name from namespace
            package_name = self._namespace_to_package(namespace)
            
            # Generate .ads (specification)
            self._write_spec(output_path, package_name, namespace, types)
            
            # Generate .adb (body) if needed
            self._write_body(output_path, package_name, namespace, types)
    
    def _collect_all_types(self):
        """Collect all type names"""
        for dt in self.data.get('dataTypes', []):
            self.all_types.add(dt['name'])
        for enum in self.data.get('enumerations', []):
            self.all_types.add(enum['name'])
    
    def _group_by_namespace(self) -> Dict[tuple, Dict[str, List]]:
        """Group types by namespace"""
        by_namespace = {}
        
        for dt in self.data.get('dataTypes', []):
            ns = tuple(dt.get('namespace', []))
            if ns not in by_namespace:
                by_namespace[ns] = {'dataTypes': [], 'enumerations': []}
            by_namespace[ns]['dataTypes'].append(dt)
        
        for enum in self.data.get('enumerations', []):
            ns = tuple(enum.get('namespace', []))
            if ns not in by_namespace:
                by_namespace[ns] = {'dataTypes': [], 'enumerations': []}
            by_namespace[ns]['enumerations'].append(enum)
        
        return by_namespace
    
    def _namespace_to_package(self, namespace: tuple) -> str:
        """Convert namespace to Ada package name"""
        if not namespace:
            return 'Pyramid.Model'
        
        parts = ['Pyramid']
        for part in namespace:
            # Remove leading numbers (e.g., "1. Common" -> "Common")
            clean = re.sub(r'^\d+\.\s*', '', part)
            # Remove all other non-word characters
            clean = re.sub(r'[^\w\s]', '', clean)
            # Replace spaces with underscores
            clean = re.sub(r'\s+', '_', clean)
            # Remove any remaining leading digits
            clean = re.sub(r'^\d+', '', clean)
            if clean:
                parts.append(clean)
        
        return '.'.join(parts)
    
    def _write_spec(self, output_path: Path, package_name: str, namespace: tuple, types: Dict[str, List]):
        """Write Ada package specification (.ads)"""
        filename = package_name.lower().replace('.', '-') + '.ads'
        filepath = output_path / filename
        
        with open(filepath, 'w') as f:
            # File header
            f.write("--  Auto-generated from SysML model\n")
            f.write(f"--  Package: {package_name}\n")
            if namespace:
                f.write(f"--  Namespace: {' > '.join(namespace)}\n")
            f.write("\n")
            
            # Withs for dependencies
            withs = self._collect_withs(types, namespace)
            for with_clause in sorted(withs):
                f.write(f"{with_clause}\n")
            if withs:
                f.write("\n")
            
            # Package declaration
            f.write(f"package {package_name} is\n\n")
            
            # Sort types by dependency
            sorted_datatypes = self._sort_types_by_dependency(types['dataTypes'])
            
            # Write enumerations
            for enum in types['enumerations']:
                self._write_enum_spec(f, enum)
                f.write("\n")
            
            # Write type definitions
            for dt in sorted_datatypes:
                self._write_type_spec(f, dt, namespace)
                f.write("\n")
            
            f.write(f"end {package_name};\n")
        
        print(f"  Generated {filepath}")
    
    def _write_body(self, output_path: Path, package_name: str, namespace: tuple, types: Dict[str, List]):
        """Write Ada package body (.adb) - only if needed"""
        # For now, we don't need body files for simple types
        pass
    
    def _collect_withs(self, types: Dict[str, List], current_namespace: tuple) -> List[str]:
        """Collect 'with' clauses needed"""
        withs = set()
        
        # Always need these for common types
        needs_strings = False
        needs_vectors = False
        needs_calendar = False
        
        # Check what we need
        for dt in types['dataTypes']:
            # Check for base type usage
            dt_name = dt['name']
            if dt_name in self.BASE_TYPE_DEFINITIONS:
                base_def = self.BASE_TYPE_DEFINITIONS[dt_name]
                if 'Unbounded_String' in base_def['base_type']:
                    needs_strings = True
                if 'Calendar.Time' in base_def['base_type']:
                    needs_calendar = True
            
            # Check properties
            for prop in dt.get('properties', []):
                type_name = prop.get('typeName')
                if type_name == 'String' or type_name == 'Identifier':
                    needs_strings = True
                if type_name == 'Timestamp':
                    needs_calendar = True
                
                # Check for vectors (multiplicity *)
                mult = prop.get('multiplicity', {})
                if mult.get('upper') == '*':
                    needs_vectors = True
                
                # Check for cross-namespace types
                if type_name and type_name in self.all_types:
                    type_ns = self._find_type_namespace(type_name)
                    if type_ns and type_ns != current_namespace:
                        pkg = self._namespace_to_package(type_ns)
                        withs.add(f"with {pkg};")
        
        if needs_strings:
            withs.add("with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;")
        if needs_vectors:
            withs.add("with Ada.Containers.Vectors;")
        if needs_calendar:
            withs.add("with Ada.Calendar;")
        
        return sorted(withs)
    
    def _find_type_namespace(self, type_name: str) -> Optional[tuple]:
        """Find which namespace a type belongs to"""
        for dt in self.data.get('dataTypes', []):
            if dt['name'] == type_name:
                return tuple(dt.get('namespace', []))
        for enum in self.data.get('enumerations', []):
            if enum['name'] == type_name:
                return tuple(enum.get('namespace', []))
        return None
    
    def _write_enum_spec(self, f, enum: Dict[str, Any]):
        """Write enumeration type"""
        name = enum['name']
        
        # Write documentation
        doc = enum.get('documentation')
        if doc:
            f.write("   --  " + doc.replace('\n', '\n   --  ') + "\n")
        
        f.write(f"   type {name} is\n")
        f.write("     (")
        
        literals = enum.get('literals', [])
        for idx, literal in enumerate(literals):
            lit_name = literal['name']
            lit_doc = literal.get('documentation')
            
            # Write literal documentation before the literal (not inline)
            if lit_doc and idx == 0:
                # For first literal, include on same line as opening paren
                pass
            elif lit_doc:
                # For subsequent literals, add comment before
                f.write("\n      --  " + lit_doc.replace('\n', '\n      --  '))
            
            if idx > 0:
                f.write(",\n      ")
            
            f.write(lit_name)
        
        f.write(");\n")
        
        # Write literal documentation after the enum
        if any(lit.get('documentation') for lit in literals):
            f.write("   --  Values:\n")
            for literal in literals:
                lit_name = literal['name']
                lit_doc = literal.get('documentation')
                if lit_doc:
                    f.write(f"   --    {lit_name}: {lit_doc}\n")
    
    def _write_type_spec(self, f, datatype: Dict[str, Any], current_namespace: tuple):
        """Write type definition"""
        name = datatype['name']
        
        # Check if base type
        if name in self.BASE_TYPE_DEFINITIONS and not datatype.get('properties'):
            self._write_base_type(f, name)
            return
        
        # Write documentation
        doc = datatype.get('documentation')
        if doc:
            f.write("   --  " + doc.replace('\n', '\n   --  ') + "\n")
        
        properties = datatype.get('properties', [])
        generalizes = datatype.get('generalizes', [])
        
        if not properties and not generalizes:
            # Empty record
            f.write(f"   type {name} is null record;\n")
            return
        
        f.write(f"   type {name} is record\n")
        
        # Inheritance - include parent as component
        if generalizes:
            for parent in generalizes:
                parent_qual = self._qualify_type_name(parent, current_namespace)
                f.write(f"      Base_{parent} : {parent_qual};\n")
            if properties:
                f.write("\n")
        
        # Properties
        for prop in properties:
            prop_name = prop['name']
            prop_type = self._get_ada_type(prop, current_namespace)
            prop_doc = prop.get('documentation')
            
            if prop_doc:
                f.write(f"      --  {prop_doc}\n")
            
            f.write(f"      {prop_name} : {prop_type};\n")
        
        f.write(f"   end record;\n")
    
    def _write_base_type(self, f, name: str):
        """Write base type definition"""
        base_def = self.BASE_TYPE_DEFINITIONS[name]
        
        f.write(f"   --  {base_def['comment']}\n")
        if 'unit' in base_def:
            f.write(f"   --  Unit: {base_def['unit']}\n")
        
        f.write(f"   type {name} is new {base_def['base_type']};\n")
    
    def _get_ada_type(self, prop: Dict[str, Any], current_namespace: tuple) -> str:
        """Get Ada type for a property"""
        type_name = prop.get('typeName')
        mult = prop.get('multiplicity', {})
        
        # Map to Ada type
        if type_name in self.TYPE_MAPPING:
            ada_type = self.TYPE_MAPPING[type_name]
        elif type_name in self.BASE_TYPE_DEFINITIONS:
            ada_type = type_name
        elif type_name:
            ada_type = self._qualify_type_name(type_name, current_namespace)
        else:
            ada_type = 'Integer'  # Default
        
        # Handle multiplicity
        mult_lower = mult.get('lower', '1')
        mult_upper = mult.get('upper', '1')
        
        if mult_upper == '*':
            # Vector - needs instantiation
            # We'll use a generic pattern
            ada_type = f"{ada_type}_Vector"
        
        # Ada doesn't have built-in optional like C++, could use access types
        # For simplicity, we'll make fields not null for now
        
        return ada_type
    
    def _qualify_type_name(self, type_name: str, current_namespace: tuple) -> str:
        """Add package qualification if type is from different namespace"""
        if not type_name:
            return type_name
        
        # Primitives don't need qualification
        if type_name in self.TYPE_MAPPING:
            return self.TYPE_MAPPING[type_name]
        
        # Find type's namespace
        type_ns = self._find_type_namespace(type_name)
        
        # If same namespace, no qualification
        if not type_ns or type_ns == current_namespace:
            return type_name
        
        # Qualify with package name
        pkg = self._namespace_to_package(type_ns)
        return f"{pkg}.{type_name}"
    
    def _sort_types_by_dependency(self, datatypes: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Sort types by dependency order"""
        type_map = {dt['name']: dt for dt in datatypes}
        dependencies = {}
        
        for dt in datatypes:
            deps = set()
            
            # Add dependencies from inheritance
            for parent in dt.get('generalizes', []):
                if parent in type_map:
                    deps.add(parent)
            
            # Add dependencies from property types
            for prop in dt.get('properties', []):
                prop_type = prop.get('typeName')
                if prop_type and prop_type in type_map:
                    if prop_type not in self.BASE_TYPE_DEFINITIONS:
                        deps.add(prop_type)
            
            dependencies[dt['name']] = deps
        
        # Topological sort
        sorted_types = []
        in_degree = {name: len(dependencies.get(name, set())) for name in type_map.keys()}
        processed = set()
        
        queue = [name for name, degree in in_degree.items() if degree == 0]
        
        while queue:
            queue.sort()
            current = queue.pop(0)
            
            if current in processed:
                continue
            
            processed.add(current)
            sorted_types.append(type_map[current])
            
            for name, deps in dependencies.items():
                if current in deps and name not in processed:
                    in_degree[name] -= 1
                    if in_degree[name] == 0:
                        queue.append(name)
        
        if len(sorted_types) != len(datatypes):
            print(f"  Warning: Could not resolve all dependencies")
            return datatypes
        
        return sorted_types


def main():
    if len(sys.argv) < 3:
        print("Usage: python ada_model_generator.py <datamodel.json> <output_dir>")
        sys.exit(1)
    
    datamodel_file = sys.argv[1]
    output_dir = sys.argv[2]
    
    print(f"Generating Ada data model...")
    print(f"  Input: {datamodel_file}")
    print(f"  Output: {output_dir}")
    
    generator = AdaDataModelGenerator(datamodel_file)
    generator.generate(output_dir)
    
    print(f"\n✓ Ada data model generated in {output_dir}/")


if __name__ == '__main__':
    main()
