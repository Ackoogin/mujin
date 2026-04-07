#!/usr/bin/env python3
"""
Python Data Model Generator
Generates Python dataclasses from parsed SysML data model
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re


class PythonDataModelGenerator:
    """Generate Python dataclasses from SysML data model"""
    
    # Type mapping from SysML to Python
    TYPE_MAPPING = {
        'String': 'str',
        'Boolean': 'bool',
        'Integer': 'int',
        'Real': 'float',
        'Identifier': 'str',  # UUID as string
        'Timestamp': 'datetime',
        'Angle': 'float',
        'Length': 'float',
        'Speed': 'float',
        'Percentage': 'float',
        'Quality': 'float'
    }
    
    def __init__(self, datamodel_file: str):
        """Load data model"""
        with open(datamodel_file, 'r') as f:
            self.data = json.load(f)
        
        # Track generated types and imports
        self.generated_types = set()
        self.needs_datetime = False
        self.needs_uuid = False
        self.needs_list = False
        self.needs_optional = False
    
    def generate(self, output_dir: str):
        """Generate Python module structure"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Generate by namespace
        files_by_namespace = self._group_by_namespace()
        
        for namespace, types in files_by_namespace.items():
            if not types['dataTypes'] and not types['enumerations']:
                continue
            
            # Create module name from namespace
            module_name = self._namespace_to_module(namespace)
            file_path = output_path / f"{module_name}.py"
            
            with open(file_path, 'w') as f:
                self._write_module(f, namespace, types)
            
            print(f"  Generated {file_path}")
        
        # Generate __init__.py
        self._generate_init_file(output_path, files_by_namespace.keys())
    
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
    
    def _namespace_to_module(self, namespace: tuple) -> str:
        """Convert namespace to Python module name"""
        if not namespace:
            return 'model'
        
        # Clean and join namespace parts
        parts = []
        for part in namespace:
            # Remove numbers and special chars, convert to snake_case
            clean = re.sub(r'^\d+\.\s*', '', part)  # Remove leading numbers
            clean = re.sub(r'[^\w\s]', '', clean)   # Remove special chars
            clean = re.sub(r'\s+', '_', clean)       # Spaces to underscores
            clean = clean.lower()
            if clean:
                parts.append(clean)
        
        return '_'.join(parts) if parts else 'model'
    
    def _write_module(self, f, namespace: tuple, types: Dict[str, List]):
        """Write a Python module for a namespace"""
        # Reset per-file tracking
        self.needs_datetime = False
        self.needs_uuid = False
        self.needs_list = False
        self.needs_optional = False
        
        # Pre-scan to determine imports
        self._scan_for_imports(types)
        
        # Write module header
        f.write('"""\n')
        f.write(f'Data Model: {" > ".join(namespace) if namespace else "Root"}\n')
        f.write('Auto-generated from SysML model\n')
        f.write('"""\n\n')
        
        # Write imports
        f.write('from dataclasses import dataclass, field\n')
        f.write('from typing import List, Optional\n')
        f.write('from enum import Enum\n')
        if self.needs_datetime:
            f.write('from datetime import datetime\n')
        if self.needs_uuid:
            f.write('from uuid import UUID, uuid4\n')
        f.write('\n')
        
        # Write enums first
        for enum in types['enumerations']:
            self._write_enum(f, enum)
            f.write('\n')
        
        # Write dataclasses
        for dt in types['dataTypes']:
            self._write_dataclass(f, dt)
            f.write('\n')
    
    def _scan_for_imports(self, types: Dict[str, List]):
        """Scan types to determine needed imports"""
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                type_name = prop.get('typeName')
                if type_name == 'Timestamp':
                    self.needs_datetime = True
                if type_name == 'Identifier':
                    self.needs_uuid = True
                
                mult = prop.get('multiplicity', {})
                if mult.get('upper') == '*':
                    self.needs_list = True
                if mult.get('lower') == '0':
                    self.needs_optional = True
    
    def _write_enum(self, f, enum: Dict[str, Any]):
        """Write an Enum class"""
        name = enum['name']
        
        # Write documentation
        doc = enum.get('documentation')
        if doc:
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'# {line.strip()}\n')
        
        f.write(f'class {name}(Enum):')
        
        literals = enum.get('literals', [])
        if not literals:
            f.write('\n    pass\n')
            return
        
        f.write('\n')
        
        for literal in literals:
            lit_name = literal['name']
            lit_doc = literal.get('documentation')
            
            # Write literal documentation
            if lit_doc:
                for line in lit_doc.split('\n'):
                    if line.strip():
                        f.write(f'    # {line.strip()}\n')
            
            # Convert to UPPER_SNAKE_CASE
            enum_value = self._to_upper_snake_case(lit_name)
            f.write(f'    {enum_value} = "{lit_name}"\n')
    
    def _write_dataclass(self, f, datatype: Dict[str, Any]):
        """Write a dataclass"""
        name = datatype['name']
        
        # Write documentation
        doc = datatype.get('documentation')
        if doc:
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'# {line.strip()}\n')
        
        f.write('@dataclass\n')
        f.write(f'class {name}:\n')
        
        # Check for properties and inheritance
        properties = datatype.get('properties', [])
        generalizes = datatype.get('generalizes', [])
        
        if not properties and not generalizes:
            f.write('    pass\n')
            return
        
        # Write docstring
        if doc:
            f.write('    """\n')
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'    {line.strip()}\n')
            f.write('    """\n')
        
        # Handle inheritance - composition pattern (base as field)
        field_num = 0
        if generalizes:
            for parent in generalizes:
                f.write(f'    base_{parent.lower()}: {parent}\n')
                field_num += 1
        
        # Write properties
        for prop in properties:
            prop_name = self._to_snake_case(prop['name'])
            prop_type = self._get_python_type(prop)
            prop_doc = prop.get('documentation')
            
            # Write property documentation as comment
            if prop_doc:
                for line in prop_doc.split('\n'):
                    if line.strip():
                        f.write(f'    # {line.strip()}\n')
            
            # Write field with default factory for mutable types
            if 'List[' in prop_type:
                f.write(f'    {prop_name}: {prop_type} = field(default_factory=list)\n')
            else:
                f.write(f'    {prop_name}: {prop_type}\n')
    
    def _get_python_type(self, prop: Dict[str, Any]) -> str:
        """Get Python type annotation for a property"""
        type_name = prop.get('typeName')
        mult = prop.get('multiplicity', {})
        
        # Map base type
        if type_name in self.TYPE_MAPPING:
            py_type = self.TYPE_MAPPING[type_name]
        else:
            py_type = type_name if type_name else 'Any'
        
        # Handle multiplicity
        mult_lower = mult.get('lower', '1')
        mult_upper = mult.get('upper', '1')
        
        if mult_upper == '*':
            # List
            py_type = f'List[{py_type}]'
        
        if mult_lower == '0' and mult_upper != '*':
            # Optional
            py_type = f'Optional[{py_type}]'
        
        return py_type
    
    def _to_snake_case(self, name: str) -> str:
        """Convert to snake_case"""
        name = re.sub(r'[^\w\s]', '', name)
        name = re.sub(r'\s+', '_', name)
        name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', name)
        return name.lower()
    
    def _to_upper_snake_case(self, name: str) -> str:
        """Convert to UPPER_SNAKE_CASE"""
        return self._to_snake_case(name).upper()
    
    def _generate_init_file(self, output_path: Path, namespaces):
        """Generate __init__.py with exports"""
        init_file = output_path / '__init__.py'
        
        with open(init_file, 'w') as f:
            f.write('"""\n')
            f.write('Data Model Package\n')
            f.write('Auto-generated from SysML model\n')
            f.write('"""\n\n')
            
            # Import all modules
            modules = [self._namespace_to_module(ns) for ns in namespaces]
            modules = sorted(set(modules))
            
            for module in modules:
                f.write(f'from . import {module}\n')
            
            f.write('\n__all__ = [\n')
            for module in modules:
                f.write(f"    '{module}',\n")
            f.write(']\n')
        
        print(f"  Generated {init_file}")


def main():
    if len(sys.argv) < 3:
        print("Usage: python python_model_generator.py <datamodel.json> <output_dir>")
        sys.exit(1)
    
    datamodel_file = sys.argv[1]
    output_dir = sys.argv[2]
    
    print(f"Generating Python data model...")
    print(f"  Input: {datamodel_file}")
    print(f"  Output: {output_dir}")
    
    generator = PythonDataModelGenerator(datamodel_file)
    generator.generate(output_dir)
    
    print(f"\n✓ Python data model generated in {output_dir}/")


if __name__ == '__main__':
    main()
