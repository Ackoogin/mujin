#!/usr/bin/env python3
"""
Ada Type Model Generator
Generates Ada package specifications with types from parsed SysML data model
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re


class AdaTypeGenerator:
    """Generate Ada type definitions from SysML data model"""
    
    # Type mapping from SysML to Ada
    TYPE_MAPPING = {
        'String': 'Unbounded_String',
        'Boolean': 'Boolean',
        'Integer': 'Integer',
        'Real': 'Float'
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
        self._collect_all_types()
    
    def _collect_all_types(self):
        """Collect all type names"""
        for dt in self.data.get('dataTypes', []):
            self.all_types.add(dt['name'])
        for enum in self.data.get('enumerations', []):
            self.all_types.add(enum['name'])
        for cls in self.data.get('classes', []):
            self.all_types.add(cls['name'])
    
    def generate(self, output_dir: str):
        """Generate Ada package structure"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Group by namespace
        files_by_namespace = self._group_by_namespace()
        
        for namespace, types in files_by_namespace.items():
            if not types['dataTypes'] and not types['enumerations']:
                continue
            
            # Create package name from namespace
            package_name = self._namespace_to_package(namespace)
            file_path = output_path / f"{package_name.lower().replace('.', '-')}.ads"
            
            with open(file_path, 'w') as f:
                self._write_package_spec(f, package_name, namespace, types)
            
            print(f"  Generated {file_path}")
    
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
            return 'Pyramid.Types'
        
        parts = ['Pyramid']
        for part in namespace:
            # Remove numbers and special chars
            clean = re.sub(r'^\d+\.\s*', '', part)
            clean = re.sub(r'[^\w\s]', '', clean)
            clean = re.sub(r'\s+', '_', clean)
            if clean:
                parts.append(clean)
        
        return '.'.join(parts)
    
    def _write_package_spec(self, f, package_name: str, namespace: tuple, types: Dict[str, List]):
        """Write Ada package specification"""
        # File header
        f.write("--  " + "=" * 76 + "\n")
        f.write(f"--  Package: {package_name}\n")
        f.write(f"--  Namespace: {' > '.join(namespace) if namespace else 'Root'}\n")
        f.write("--  Auto-generated from SysML model\n")
        f.write("--  " + "=" * 76 + "\n\n")
        
        # Required withs
        needs_unbounded = self._needs_unbounded_string(types)
        needs_calendar = self._needs_calendar(types)
        needs_vectors = self._needs_vectors(types)
        
        if needs_unbounded:
            f.write("with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;\n")
        if needs_calendar:
            f.write("with Ada.Calendar;\n")
        if needs_vectors:
            f.write("with Ada.Containers.Vectors;\n")
        
        # Check for dependencies on other packages
        deps = self._collect_dependencies(types, namespace)
        for dep in sorted(deps):
            f.write(f"with {dep};\n")
        
        if needs_unbounded or needs_calendar or needs_vectors or deps:
            f.write("\n")
        
        # Package declaration
        f.write(f"package {package_name} is\n\n")
        
        # Sort types by dependency
        sorted_datatypes = self._sort_types_by_dependency(types['dataTypes'])
        
        # Write enumerations first
        for enum in types['enumerations']:
            self._write_enum(f, enum)
            f.write("\n")
        
        # Write type definitions
        for dt in sorted_datatypes:
            self._write_type(f, dt, namespace)
            f.write("\n")
        
        # Package end
        f.write(f"end {package_name};\n")
    
    def _needs_unbounded_string(self, types: Dict[str, List]) -> bool:
        """Check if package needs Unbounded_String"""
        for dt in types['dataTypes']:
            if dt['name'] in self.BASE_TYPE_DEFINITIONS:
                if self.BASE_TYPE_DEFINITIONS[dt['name']]['base_type'] == 'Unbounded_String':
                    return True
            for prop in dt.get('properties', []):
                if prop.get('typeName') in ['String', 'Identifier']:
                    return True
        return False
    
    def _needs_calendar(self, types: Dict[str, List]) -> bool:
        """Check if package needs Ada.Calendar"""
        for dt in types['dataTypes']:
            if dt['name'] == 'Timestamp' or any(p.get('typeName') == 'Timestamp' 
                                                 for p in dt.get('properties', [])):
                return True
        return False
    
    def _needs_vectors(self, types: Dict[str, List]) -> bool:
        """Check if package needs Ada.Containers.Vectors"""
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                mult = prop.get('multiplicity', {})
                if mult.get('upper') == '*':
                    return True
        return False
    
    def _collect_dependencies(self, types: Dict[str, List], current_ns: tuple) -> set:
        """Collect package dependencies"""
        deps = set()
        
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                type_name = prop.get('typeName')
                if type_name and type_name not in self.TYPE_MAPPING:
                    type_ns = self._find_type_namespace(type_name)
                    if type_ns and type_ns != current_ns:
                        pkg = self._namespace_to_package(type_ns)
                        deps.add(pkg)
            
            # Check inheritance
            for parent in dt.get('generalizes', []):
                parent_ns = self._find_type_namespace(parent)
                if parent_ns and parent_ns != current_ns:
                    pkg = self._namespace_to_package(parent_ns)
                    deps.add(pkg)
        
        return deps
    
    def _find_type_namespace(self, type_name: str) -> Optional[tuple]:
        """Find namespace for a type"""
        for dt in self.data.get('dataTypes', []):
            if dt['name'] == type_name:
                return tuple(dt.get('namespace', []))
        for enum in self.data.get('enumerations', []):
            if enum['name'] == type_name:
                return tuple(enum.get('namespace', []))
        return None
    
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
        in_degree = {name: len(deps) for name, deps in dependencies.items()}
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
            return datatypes
        
        return sorted_types
    
    def _write_enum(self, f, enum: Dict[str, Any]):
        """Write an enumeration type"""
        name = enum['name']
        
        # Documentation
        doc = enum.get('documentation')
        if doc:
            f.write("   --  " + "\n   --  ".join(doc.split('\n')) + "\n")
        
        f.write(f"   type {name} is\n")
        f.write("     (")
        
        literals = enum.get('literals', [])
        for idx, literal in enumerate(literals):
            lit_name = literal['name']
            # Convert to Ada identifier
            ada_name = self._to_ada_identifier(lit_name)
            
            if idx > 0:
                f.write(",\n      ")
            f.write(ada_name)
        
        f.write(");\n")
    
    def _write_type(self, f, datatype: Dict[str, Any], current_ns: tuple):
        """Write a record type"""
        name = datatype['name']
        
        # Check if base type
        if name in self.BASE_TYPE_DEFINITIONS and not datatype.get('properties'):
            base_def = self.BASE_TYPE_DEFINITIONS[name]
            
            # Documentation
            f.write(f"   --  {base_def['comment']}\n")
            if 'unit' in base_def:
                f.write(f"   --  Unit: {base_def['unit']}\n")
            
            f.write(f"   type {name} is new {base_def['base_type']};\n")
            return
        
        # Documentation
        doc = datatype.get('documentation')
        if doc:
            f.write("   --  " + "\n   --  ".join(doc.split('\n')) + "\n")
        
        # Check for properties and inheritance
        properties = datatype.get('properties', [])
        generalizes = datatype.get('generalizes', [])
        
        if not properties and not generalizes:
            f.write(f"   type {name} is null record;\n")
            return
        
        f.write(f"   type {name} is record\n")
        
        # Handle inheritance - composition pattern
        if generalizes:
            for parent in generalizes:
                parent_qualified = self._qualify_type(parent, current_ns)
                f.write(f"      Base_{parent} : {parent_qualified};\n")
            if properties:
                f.write("\n")
        
        # Write properties
        for prop in properties:
            prop_name = self._to_ada_identifier(prop['name'])
            prop_type = self._get_ada_type(prop, current_ns)
            
            # Property documentation
            prop_doc = prop.get('documentation')
            if prop_doc:
                f.write("      --  " + "\n      --  ".join(prop_doc.split('\n')) + "\n")
            
            f.write(f"      {prop_name} : {prop_type};\n")
        
        f.write("   end record;\n")
        
        # Generate vector package if needed
        for prop in properties:
            mult = prop.get('multiplicity', {})
            if mult.get('upper') == '*':
                prop_type_name = prop.get('typeName')
                if prop_type_name:
                    vector_pkg = f"{name}_{self._to_ada_identifier(prop['name'])}_Vectors"
                    element_type = self._map_type(prop_type_name, current_ns)
                    f.write(f"\n   package {vector_pkg} is new Ada.Containers.Vectors\n")
                    f.write(f"     (Index_Type   => Positive,\n")
                    f.write(f"      Element_Type => {element_type});\n")
    
    def _qualify_type(self, type_name: str, current_ns: tuple) -> str:
        """Qualify type name if from different namespace"""
        if type_name in self.TYPE_MAPPING or type_name in self.BASE_TYPE_DEFINITIONS:
            return type_name
        
        type_ns = self._find_type_namespace(type_name)
        if type_ns and type_ns != current_ns:
            pkg = self._namespace_to_package(type_ns)
            return f"{pkg}.{type_name}"
        
        return type_name
    
    def _get_ada_type(self, prop: Dict[str, Any], current_ns: tuple) -> str:
        """Get Ada type for a property"""
        type_name = prop.get('typeName')
        mult = prop.get('multiplicity', {})
        
        # Map base type
        ada_type = self._map_type(type_name, current_ns)
        
        # Handle multiplicity
        mult_lower = mult.get('lower', '1')
        mult_upper = mult.get('upper', '1')
        
        if mult_upper == '*':
            # Use vector - will be defined as nested package
            parent_name = prop.get('parent_type', 'Type')
            prop_name = self._to_ada_identifier(prop['name'])
            ada_type = f"{parent_name}_{prop_name}_Vectors.Vector"
        elif mult_lower == '0' and mult_upper != '*':
            # Optional - use access type
            ada_type = f"access {ada_type}"
        
        return ada_type
    
    def _map_type(self, type_name: str, current_ns: tuple) -> str:
        """Map SysML type to Ada type"""
        if not type_name:
            return 'Integer'
        
        if type_name in self.TYPE_MAPPING:
            return self.TYPE_MAPPING[type_name]
        
        # Qualify if from different namespace
        return self._qualify_type(type_name, current_ns)
    
    def _to_ada_identifier(self, name: str) -> str:
        """Convert to valid Ada identifier"""
        if not name:
            return 'Unnamed'
        
        # Remove special chars
        name = re.sub(r'[^\w\s]', '', name)
        # Split on whitespace and underscores
        parts = re.split(r'[\s_]+', name)
        # Capitalize each part
        result = '_'.join(word.capitalize() for word in parts if word)
        
        # Ensure doesn't start with number
        if result and result[0].isdigit():
            result = 'T_' + result
        
        return result if result else 'Unnamed'


def main():
    if len(sys.argv) < 3:
        print("Usage: python ada_type_generator.py <datamodel.json> <output_dir>")
        sys.exit(1)
    
    datamodel_file = sys.argv[1]
    output_dir = sys.argv[2]
    
    print(f"Generating Ada type model...")
    print(f"  Input: {datamodel_file}")
    print(f"  Output: {output_dir}")
    
    generator = AdaTypeGenerator(datamodel_file)
    generator.generate(output_dir)
    
    print(f"\n[x] Ada type model generated in {output_dir}/")


if __name__ == '__main__':
    main()
