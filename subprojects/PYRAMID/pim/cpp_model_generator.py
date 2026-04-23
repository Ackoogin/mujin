#!/usr/bin/env python3
"""
C++ Data Model Generator
Generates C++ header files with structs/classes from parsed SysML data model
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re


class CppDataModelGenerator:
    """Generate C++ data structures from SysML data model"""
    
    # Type mapping from SysML primitives to C++ (not including base types)
    TYPE_MAPPING = {
        'String': 'std::string',
        'Boolean': 'bool',
        'Integer': 'int32_t',
        'Real': 'double'
    }
    
    # Base type definitions - these are generated as structs
    BASE_TYPE_DEFINITIONS = {
        'Identifier': {
            'comment': 'Universally Unique Identifier (UUID)',
            'type': 'std::string',
            'constructor': None
        },
        'Timestamp': {
            'comment': 'Point in time (uses std::chrono)',
            'type': 'std::chrono::system_clock::time_point',
            'constructor': 'std::chrono::system_clock::now()'
        },
        'Angle': {
            'comment': 'Angle in radians',
            'type': 'double',
            'unit': 'radians'
        },
        'Length': {
            'comment': 'Length in meters',
            'type': 'double',
            'unit': 'meters'
        },
        'Speed': {
            'comment': 'Speed in meters per second',
            'type': 'double',
            'unit': 'meters_per_second'
        },
        'Percentage': {
            'comment': 'Percentage value (0-100)',
            'type': 'double',
            'unit': 'percent'
        },
        'Quality': {
            'comment': 'Quality metric (0.0-1.0)',
            'type': 'double',
            'unit': 'normalized'
        }
    }
    
    def __init__(self, datamodel_file: str):
        """Load data model"""
        with open(datamodel_file, 'r') as f:
            self.data = json.load(f)
        
        # Track includes needed
        self.needs_string = False
        self.needs_vector = False
        self.needs_optional = False
        self.needs_chrono = False
        self.needs_cstdint = False
        
        # Track all type names for forward declarations
        self.all_types = set()
    
    def generate(self, output_dir: str):
        """Generate C++ header structure"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Collect all type names first
        self._collect_all_types()
        
        # Generate by namespace
        files_by_namespace = self._group_by_namespace()
        
        for namespace, types in files_by_namespace.items():
            if not types['dataTypes'] and not types['enumerations']:
                continue
            
            # Create header file name from namespace
            header_name = self._namespace_to_header(namespace)
            file_path = output_path / f"{header_name}.h"
            
            with open(file_path, 'w', encoding='utf-8') as f:
                self._write_header(f, namespace, types, header_name)
            
            print(f"  Generated {file_path}")
        
        # Generate common types header
        self._generate_common_header(output_path)
    
    def _collect_all_types(self):
        """Collect all type names for forward declarations"""
        for dt in self.data.get('dataTypes', []):
            self.all_types.add(dt['name'])
        for enum in self.data.get('enumerations', []):
            self.all_types.add(enum['name'])
        for cls in self.data.get('classes', []):
            self.all_types.add(cls['name'])
    
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
    
    def _namespace_to_header(self, namespace: tuple) -> str:
        """Convert namespace to header file name"""
        if not namespace:
            return 'model'
        
        # Clean and join namespace parts
        parts = []
        for part in namespace:
            # Remove numbers and special chars
            clean = re.sub(r'^\d+\.\s*', '', part)  # Remove leading numbers
            clean = re.sub(r'[^\w\s]', '', clean)   # Remove special chars
            clean = re.sub(r'\s+', '_', clean)       # Spaces to underscores
            clean = clean.lower()
            if clean:
                parts.append(clean)
        
        return '_'.join(parts) if parts else 'model'
    
    def _namespace_to_cpp(self, namespace: tuple) -> List[str]:
        """Convert namespace tuple to C++ namespace components"""
        if not namespace:
            return ['pyramid']
        
        ns_parts = ['pyramid']
        for part in namespace:
            # Clean part
            clean = re.sub(r'^\d+\.\s*', '', part)
            clean = re.sub(r'[^\w]', '_', clean)
            clean = clean.lower()
            if clean and clean != 'pyramid':
                ns_parts.append(clean)
        
        return ns_parts
    
    def _write_header(self, f, namespace: tuple, types: Dict[str, List], header_name: str):
        """Write a C++ header file for a namespace"""
        # Reset per-file tracking
        self.needs_string = False
        self.needs_vector = False
        self.needs_optional = False
        self.needs_chrono = False
        self.needs_cstdint = False
        
        # Pre-scan to determine includes
        self._scan_for_includes(types)
        
        # Header guard
        guard_name = f"PYRAMID_{header_name.upper()}_H"
        f.write(f"#ifndef {guard_name}\n")
        f.write(f"#define {guard_name}\n\n")
        
        # File documentation
        f.write("/**\n")
        f.write(f" * Data Model: {' > '.join(namespace) if namespace else 'Root'}\n")
        f.write(" * Auto-generated from SysML model\n")
        f.write(" * C++14 compatible\n")
        f.write(" */\n\n")
        
        # System includes
        if self.needs_string:
            f.write("#include <string>\n")
        if self.needs_vector:
            f.write("#include <vector>\n")
        if self.needs_optional:
            f.write("#include <memory>\n")  # Use unique_ptr instead of optional for C++14
        if self.needs_chrono:
            f.write("#include <chrono>\n")
        if self.needs_cstdint:
            f.write("#include <cstdint>\n")
        
        if any([self.needs_string, self.needs_vector, self.needs_optional, 
                self.needs_chrono, self.needs_cstdint]):
            f.write("\n")
        
        # Cross-file includes for dependencies
        required_includes = self._collect_required_includes(types, namespace)
        if required_includes:
            f.write("// Required type dependencies\n")
            for inc in required_includes:
                f.write(f'#include "{inc}.h"\n')
            f.write("\n")
        
        # Namespace opening
        cpp_namespace = self._namespace_to_cpp(namespace)
        for ns in cpp_namespace:
            f.write(f"namespace {ns} {{\n")
        f.write("\n")
        
        # Forward declarations for types used in this file
        forward_decls = self._collect_forward_declarations(types)
        if forward_decls:
            f.write("// Forward declarations\n")
            for type_name in sorted(forward_decls):
                f.write(f"struct {type_name};\n")
            f.write("\n")
        
        # Write enumerations first (they have no dependencies)
        for enum in types['enumerations']:
            self._write_enum(f, enum)
            f.write("\n")
        
        # Sort dataTypes by dependency order
        sorted_datatypes = self._sort_types_by_dependency(types['dataTypes'])
        
        # Write structs/classes in dependency order
        for dt in sorted_datatypes:
            self._write_struct(f, dt, namespace)
            f.write("\n")
        
        # Namespace closing
        for ns in reversed(cpp_namespace):
            f.write(f"}} // namespace {ns}\n")
        
        f.write(f"\n#endif // {guard_name}\n")
    
    def _collect_required_includes(self, types: Dict[str, List], current_namespace: tuple) -> List[str]:
        """Collect header files that need to be included for cross-namespace dependencies"""
        required_includes = set()
        
        # Get current namespace's types
        current_type_names = set()
        for dt in types['dataTypes']:
            current_type_names.add(dt['name'])
        for enum in types['enumerations']:
            current_type_names.add(enum['name'])
        
        # Check all property types and parent types
        for dt in types['dataTypes']:
            # Check inheritance
            for parent in dt.get('generalizes', []):
                if parent not in current_type_names and parent not in self.TYPE_MAPPING:
                    # Find which namespace this parent is in
                    parent_ns = self._find_type_namespace(parent)
                    if parent_ns and parent_ns != current_namespace:
                        header_name = self._namespace_to_header(parent_ns)
                        required_includes.add(header_name)
            
            # Check property types
            for prop in dt.get('properties', []):
                prop_type = prop.get('typeName')
                if prop_type and prop_type not in current_type_names and prop_type not in self.TYPE_MAPPING:
                    # Find which namespace this type is in
                    type_ns = self._find_type_namespace(prop_type)
                    if type_ns and type_ns != current_namespace:
                        header_name = self._namespace_to_header(type_ns)
                        required_includes.add(header_name)
        
        return sorted(required_includes)
    
    def _find_type_namespace(self, type_name: str) -> Optional[tuple]:
        """Find which namespace a type belongs to"""
        # Search dataTypes
        for dt in self.data.get('dataTypes', []):
            if dt['name'] == type_name:
                return tuple(dt.get('namespace', []))
        
        # Search enumerations
        for enum in self.data.get('enumerations', []):
            if enum['name'] == type_name:
                return tuple(enum.get('namespace', []))
        
        # Search classes
        for cls in self.data.get('classes', []):
            if cls['name'] == type_name:
                return tuple(cls.get('namespace', []))
        
        return None
    
    def _scan_for_includes(self, types: Dict[str, List]):
        """Scan types to determine needed includes"""
        # First check if any base types are being defined (auto-generated)
        for dt in types['dataTypes']:
            dt_name = dt['name']
            if dt_name in self.BASE_TYPE_DEFINITIONS and not dt.get('properties'):
                # This is a base type being auto-generated, check its requirements
                base_def = self.BASE_TYPE_DEFINITIONS[dt_name]
                base_type = base_def['type']
                
                if 'std::string' in base_type:
                    self.needs_string = True
                if 'std::chrono' in base_type:
                    self.needs_chrono = True
                if 'int32_t' in base_type:
                    self.needs_cstdint = True
        
        # Then check properties
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                type_name = prop.get('typeName')
                
                # Check if using std library types directly (not base types)
                if type_name == 'String':
                    self.needs_string = True
                if type_name == 'Integer':
                    self.needs_cstdint = True
                
                # Check base types for their underlying needs
                if type_name == 'Identifier':
                    self.needs_string = True
                if type_name == 'Timestamp':
                    self.needs_chrono = True
                
                mult = prop.get('multiplicity', {})
                if mult.get('upper') == '*':
                    self.needs_vector = True
                if mult.get('lower') == '0' and mult.get('upper') != '*':
                    self.needs_optional = True
    
    def _sort_types_by_dependency(self, datatypes: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Sort types by dependency order using topological sort"""
        # Build dependency graph
        type_map = {dt['name']: dt for dt in datatypes}
        dependencies = {}
        
        for dt in datatypes:
            deps = set()
            
            # Add dependencies from inheritance (generalizes)
            for parent in dt.get('generalizes', []):
                if parent in type_map:
                    deps.add(parent)
            
            # Add dependencies from property types
            for prop in dt.get('properties', []):
                prop_type = prop.get('typeName')
                if prop_type and prop_type in type_map:
                    # Only add if not a base type (which will be auto-generated)
                    if prop_type not in self.BASE_TYPE_DEFINITIONS:
                        deps.add(prop_type)
            
            dependencies[dt['name']] = deps
        
        # Topological sort using Kahn's algorithm
        sorted_types = []
        in_degree = {name: len(dependencies.get(name, set())) for name in type_map.keys()}
        processed = set()
        
        # Find nodes with no dependencies (in-degree 0)
        queue = [name for name, degree in in_degree.items() if degree == 0]
        
        # Process queue
        while queue:
            # Sort for deterministic output
            queue.sort()
            current = queue.pop(0)
            
            if current in processed:
                continue
            
            processed.add(current)
            sorted_types.append(type_map[current])
            
            # For each type that depends on current, reduce its in-degree
            for name, deps in dependencies.items():
                if current in deps and name not in processed:
                    in_degree[name] -= 1
                    if in_degree[name] == 0:
                        queue.append(name)
        
        # Check for cycles (shouldn't happen with valid models)
        if len(sorted_types) != len(datatypes):
            # Could not resolve all dependencies
            remaining = [name for name in type_map.keys() if name not in processed]
            print(f"  Warning: Could not resolve all dependencies. Remaining: {', '.join(remaining[:5])}")
            return datatypes
        
        return sorted_types
    
    def _collect_forward_declarations(self, types: Dict[str, List]) -> set:
        """Collect types that need forward declarations (only for same-namespace types)"""
        forward_decls = set()
        
        # Get types defined in this file
        types_in_file = set()
        for dt in types['dataTypes']:
            types_in_file.add(dt['name'])
        for enum in types['enumerations']:
            types_in_file.add(enum['name'])
        
        # Check all property types - only forward declare if in same file
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                type_name = prop.get('typeName')
                # Only forward declare if:
                # 1. It's a user-defined type (not in TYPE_MAPPING)
                # 2. It's not a base type (not in BASE_TYPE_DEFINITIONS)
                # 3. It's defined in THIS file
                # 4. It appears before it's defined (we'll handle via ordering)
                if (type_name and 
                    type_name not in self.TYPE_MAPPING and 
                    type_name not in self.BASE_TYPE_DEFINITIONS and
                    type_name in types_in_file):
                    # We actually don't need forward decls if we sort by dependency
                    # But keep this for circular references within same file
                    pass
        
        # Return empty set since we now sort by dependency order
        return set()
    
    
    def _write_enum(self, f, enum: Dict[str, Any]):
        """Write an enum class"""
        name = enum['name']
        
        # Write documentation
        doc = enum.get('documentation')
        if doc:
            f.write("/**\n")
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f" * {line.strip()}\n")
            f.write(" */\n")
        
        f.write(f"enum class {name} {{\n")
        
        literals = enum.get('literals', [])
        if not literals:
            f.write("    // Empty enumeration\n")
        else:
            for idx, literal in enumerate(literals):
                lit_name = literal['name']
                lit_doc = literal.get('documentation')
                
                # Write literal documentation
                if lit_doc:
                    f.write("    /**\n")
                    for line in lit_doc.split('\n'):
                        if line.strip():
                            f.write(f"     * {line.strip()}\n")
                    f.write("     */\n")
                
                # Convert to PascalCase
                enum_value = self._to_pascal_case(lit_name)
                
                # Add comma except for last item
                comma = "," if idx < len(literals) - 1 else ""
                f.write(f"    {enum_value}{comma}\n")
        
        f.write("};\n")
        
        # Add string conversion functions if enum has literals
        if literals:
            f.write("\n")
            self._write_enum_conversions(f, name, literals)
    
    def _write_enum_conversions(self, f, enum_name: str, literals: List[Dict]):
        """Write to_string and from_string conversion functions for enum"""
        # to_string function
        f.write(f"inline const char* to_string({enum_name} value) {{\n")
        f.write("    switch (value) {\n")
        for literal in literals:
            lit_name = literal['name']
            enum_value = self._to_pascal_case(lit_name)
            f.write(f"        case {enum_name}::{enum_value}: return \"{lit_name}\";\n")
        f.write("        default: return \"Unknown\";\n")
        f.write("    }\n")
        f.write("}\n\n")
        
        # from_string function
        f.write(f"inline {enum_name} {enum_name}_from_string(const std::string& str) {{\n")
        for idx, literal in enumerate(literals):
            lit_name = literal['name']
            enum_value = self._to_pascal_case(lit_name)
            if_keyword = "if" if idx == 0 else "} else if"
            f.write(f"    {if_keyword} (str == \"{lit_name}\") {{\n")
            f.write(f"        return {enum_name}::{enum_value};\n")
        f.write("    }\n")
        # Return first value as default
        first_literal = self._to_pascal_case(literals[0]['name'])
        f.write(f"    return {enum_name}::{first_literal}; // Default\n")
        f.write("}\n")
    
    def _write_struct(self, f, datatype: Dict[str, Any], current_namespace: tuple = None):
        """Write a struct definition"""
        name = datatype['name']
        
        # Check if this is a base type that should be auto-filled
        if name in self.BASE_TYPE_DEFINITIONS and not datatype.get('properties'):
            base_def = self.BASE_TYPE_DEFINITIONS[name]
            
            # Write documentation
            f.write("/**\n")
            f.write(f" * {base_def['comment']}\n")
            if 'unit' in base_def:
                f.write(f" * Unit: {base_def['unit']}\n")
            f.write(" */\n")
            
            f.write(f"struct {name} {{\n")
            f.write(f"    {base_def['type']} value;\n")
            f.write("\n")
            
            # Add default constructor if specified
            if base_def.get('constructor'):
                f.write(f"    {name}() : value({base_def['constructor']}) {{}}\n")
            else:
                f.write(f"    {name}() : value() {{}}\n")
            
            # Add value constructor
            f.write(f"    explicit {name}({base_def['type']} v) : value(v) {{}}\n")
            
            # Add conversion operator
            f.write(f"    operator {base_def['type']}() const {{ return value; }}\n")
            
            f.write("};\n")
            return
        
        # Write documentation
        doc = datatype.get('documentation')
        if doc:
            f.write("/**\n")
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f" * {line.strip()}\n")
            f.write(" */\n")
        
        f.write(f"struct {name} {{\n")
        
        # Check for properties and inheritance
        properties = datatype.get('properties', [])
        generalizes = datatype.get('generalizes', [])
        
        if not properties and not generalizes:
            f.write("    // Empty struct\n")
            f.write("};\n")
            return
        
        # Handle inheritance - composition pattern (base as member)
        if generalizes:
            for parent in generalizes:
                # Qualify parent type if from different namespace
                if current_namespace:
                    parent_qualified = self._qualify_type_name(parent, current_namespace)
                else:
                    parent_qualified = parent
                f.write(f"    {parent_qualified} base_{self._to_snake_case(parent)};\n")
            if properties:
                f.write("\n")
        
        # Write properties
        has_unique_ptr = False
        for prop in properties:
            prop_name = self._to_snake_case(prop['name'])
            prop_type = self._get_cpp_type(prop, current_namespace)
            prop_doc = prop.get('documentation')
            
            # Track if we have unique_ptr members
            if 'std::unique_ptr<' in prop_type:
                has_unique_ptr = True
            
            # Write property documentation
            if prop_doc:
                f.write("    /**\n")
                for line in prop_doc.split('\n'):
                    if line.strip():
                        f.write(f"     * {line.strip()}\n")
                f.write("     */\n")
            
            f.write(f"    {prop_type} {prop_name};\n")
        
        # Add copy/move constructors if we have unique_ptr members
        if has_unique_ptr:
            f.write("\n")
            f.write("    // Constructors for copyability with unique_ptr members\n")
            f.write(f"    {name}() = default;\n")
            f.write(f"    {name}(const {name}& other);\n")
            f.write(f"    {name}({name}&&) = default;\n")
            f.write(f"    {name}& operator=(const {name}& other);\n")
            f.write(f"    {name}& operator=({name}&&) = default;\n")
        
        f.write("};\n")
        
        # Write copy constructor/assignment implementations after the struct
        if has_unique_ptr:
            f.write("\n")
            self._write_copy_implementation(f, name, properties, generalizes, current_namespace)
    
    def _extract_unique_ptr_type(self, cpp_type: str) -> str:
        """Extract the inner type from std::unique_ptr<Type>"""
        if not cpp_type.startswith('std::unique_ptr<'):
            return cpp_type
        
        # Remove 'std::unique_ptr<' prefix and trailing '>'
        # Handle nested templates by counting angle brackets
        start = len('std::unique_ptr<')
        depth = 1
        end = start
        
        for i in range(start, len(cpp_type)):
            if cpp_type[i] == '<':
                depth += 1
            elif cpp_type[i] == '>':
                depth -= 1
                if depth == 0:
                    end = i
                    break
        
        return cpp_type[start:end]
    
    def _write_copy_implementation(self, f, name: str, properties: List[Dict], generalizes: List[str], current_namespace: tuple):
        """Write copy constructor and assignment operator implementations"""
        # Copy constructor
        f.write(f"inline {name}::{name}(const {name}& other)\n")
        
        # Initialize base classes
        init_list = []
        for parent in generalizes:
            parent_member = f"base_{self._to_snake_case(parent)}"
            init_list.append(f"    {parent_member}(other.{parent_member})")
        
        # Initialize regular members (non-unique_ptr)
        for prop in properties:
            prop_name = self._to_snake_case(prop['name'])
            prop_type = self._get_cpp_type(prop, current_namespace)
            
            if 'std::unique_ptr<' not in prop_type:
                init_list.append(f"    {prop_name}(other.{prop_name})")
        
        if init_list:
            f.write("  : " + ",\n    ".join(init_list) + "\n")
        
        f.write("{\n")
        
        # Deep copy unique_ptr members
        for prop in properties:
            prop_name = self._to_snake_case(prop['name'])
            prop_type = self._get_cpp_type(prop, current_namespace)
            
            if 'std::unique_ptr<' in prop_type:
                inner_type = self._extract_unique_ptr_type(prop_type)
                f.write(f"    if (other.{prop_name}) {{\n")
                f.write(f"        {prop_name} = std::unique_ptr<{inner_type}>(new {inner_type}(*other.{prop_name}));\n")
                f.write("    }\n")
        
        f.write("}\n\n")
        
        # Copy assignment operator
        f.write(f"inline {name}& {name}::operator=(const {name}& other) {{\n")
        f.write("    if (this != &other) {\n")
        
        # Copy base classes
        for parent in generalizes:
            parent_member = f"base_{self._to_snake_case(parent)}"
            f.write(f"        {parent_member} = other.{parent_member};\n")
        
        # Copy regular members
        for prop in properties:
            prop_name = self._to_snake_case(prop['name'])
            prop_type = self._get_cpp_type(prop, current_namespace)
            
            if 'std::unique_ptr<' not in prop_type:
                f.write(f"        {prop_name} = other.{prop_name};\n")
        
        # Deep copy unique_ptr members
        for prop in properties:
            prop_name = self._to_snake_case(prop['name'])
            prop_type = self._get_cpp_type(prop, current_namespace)
            
            if 'std::unique_ptr<' in prop_type:
                inner_type = self._extract_unique_ptr_type(prop_type)
                f.write(f"        if (other.{prop_name}) {{\n")
                f.write(f"            {prop_name} = std::unique_ptr<{inner_type}>(new {inner_type}(*other.{prop_name}));\n")
                f.write("        } else {\n")
                f.write(f"            {prop_name}.reset();\n")
                f.write("        }\n")
        
        f.write("    }\n")
        f.write("    return *this;\n")
        f.write("}\n")
    
    def _qualify_type_name(self, type_name: str, current_namespace: tuple) -> str:
        """Add namespace qualification if type is from different namespace"""
        if not type_name:
            return type_name
        
        # Don't qualify primitives
        if type_name in self.TYPE_MAPPING:
            return type_name
        
        # Check if it's a base type - these need qualification if in different namespace
        if type_name in self.BASE_TYPE_DEFINITIONS:
            # Base types are always in a "base" namespace, need to find it
            type_ns = self._find_type_namespace(type_name)
            if type_ns and type_ns != current_namespace:
                cpp_ns = self._namespace_to_cpp(type_ns)
                return '::'.join(cpp_ns + [type_name])
            return type_name
        
        # Find which namespace this type is in
        type_ns = self._find_type_namespace(type_name)
        
        # If not found or same namespace, no qualification needed
        if not type_ns or type_ns == current_namespace:
            return type_name
        
        # Build fully qualified name
        cpp_ns = self._namespace_to_cpp(type_ns)
        return '::'.join(cpp_ns + [type_name])
    
    def _get_cpp_type(self, prop: Dict[str, Any], current_namespace: tuple = None) -> str:
        """Get C++ type for a property"""
        type_name = prop.get('typeName')
        mult = prop.get('multiplicity', {})
        
        # Determine the base C++ type
        if type_name in self.TYPE_MAPPING:
            # Use primitive mapping (std::string, bool, int32_t, double)
            cpp_type = self.TYPE_MAPPING[type_name]
        elif type_name:
            # Any other type (base types or user-defined) - needs qualification
            if current_namespace:
                cpp_type = self._qualify_type_name(type_name, current_namespace)
            else:
                cpp_type = type_name
        else:
            cpp_type = 'void*'
        
        # Handle multiplicity
        mult_lower = mult.get('lower', '1')
        mult_upper = mult.get('upper', '1')
        
        if mult_upper == '*':
            # Vector
            cpp_type = f'std::vector<{cpp_type}>'
        
        if mult_lower == '0' and mult_upper != '*':
            # Use unique_ptr for optional in C++14
            cpp_type = f'std::unique_ptr<{cpp_type}>'
        
        return cpp_type
    
    def _to_snake_case(self, name: str) -> str:
        """Convert to snake_case"""
        if not name:
            return 'unnamed'
        name = re.sub(r'[^\w\s]', '', name)
        name = re.sub(r'\s+', '_', name)
        name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', name)
        result = name.lower()
        return result if result else 'unnamed'
    
    def _to_pascal_case(self, name: str) -> str:
        """Convert to PascalCase"""
        if not name:
            return 'Unnamed'
        name = re.sub(r'[^\w\s]', '', name)
        parts = re.split(r'[\s_]+', name)
        result = ''.join(word.capitalize() for word in parts if word)
        return result if result else 'Unnamed'
    
    def _generate_common_header(self, output_path: Path):
        """Generate a common header with all includes"""
        common_file = output_path / "pyramid_types.h"
        
        # Get all header files
        files_by_namespace = self._group_by_namespace()
        headers = []
        for namespace in files_by_namespace.keys():
            header_name = self._namespace_to_header(namespace)
            headers.append(f"{header_name}.h")
        
        with open(common_file, 'w', encoding='utf-8') as f:
            f.write("#ifndef PYRAMID_TYPES_H\n")
            f.write("#define PYRAMID_TYPES_H\n\n")
            
            f.write("/**\n")
            f.write(" * Common include for all Pyramid data types\n")
            f.write(" * Auto-generated from SysML model\n")
            f.write(" * C++14 compatible\n")
            f.write(" */\n\n")
            
            for header in sorted(headers):
                f.write(f'#include "{header}"\n')
            
            f.write("\n#endif // PYRAMID_TYPES_H\n")
        
        print(f"  Generated {common_file}")


def main():
    if len(sys.argv) < 3:
        print("Usage: python cpp_model_generator.py <datamodel.json> <output_dir>")
        sys.exit(1)
    
    datamodel_file = sys.argv[1]
    output_dir = sys.argv[2]
    
    print(f"Generating C++14 data model...")
    print(f"  Input: {datamodel_file}")
    print(f"  Output: {output_dir}")
    
    generator = CppDataModelGenerator(datamodel_file)
    generator.generate(output_dir)
    
    print(f"\n✓ C++14 data model generated in {output_dir}/")
    print(f"\nInclude in your code:")
    print(f'  #include "pyramid_types.h"')


if __name__ == '__main__':
    main()
