#!/usr/bin/env python3
"""
Protobuf Generator
Transforms parsed SysML JSON model to Protocol Buffer (.proto) files
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Set, Optional


class ProtobufGenerator:
    """Generate Protocol Buffer definitions from parsed SysML model"""
    
    # Mapping of common SysML types to protobuf types
    TYPE_MAPPING = {
        'Identifier': 'Identifier',  # Use the message type
        'Timestamp': 'Timestamp',     # Use the message type (wraps google.protobuf.Timestamp)
        'Quality': 'Quality',         # Use the message type
        'Course': 'double',           # Simple double for course
        'Speed': 'Speed',             # Use the message type
        'Length': 'Length',           # Use the message type
        'Angle': 'Angle',             # Use the message type
        'Percentage': 'Percentage',   # Use the message type
        'String': 'string',
        'Boolean': 'bool',
        'Real': 'double',
        'Integer': 'int32',
        # Add more mappings as needed
    }
    
    # Base types that should be automatically filled with standard definitions
    BASE_TYPE_DEFINITIONS = {
        'Identifier': {
            'comment': 'Universally Unique Identifier (UUID)',
            'fields': [
                ('string', 'value', 1, 'UUID string representation (e.g., "550e8400-e29b-41d4-a716-446655440000")')
            ]
        },
        'Query': {
            'comment': 'EntityActions query scope',
            'fields': [
                ('repeated Identifier', 'id', 1, 'Filter by specific IDs; empty = all'),
                ('bool', 'one_shot', 2, 'true = single response; false = continuous stream'),
            ]
        },
        'Ack': {
            'comment': 'EntityActions acknowledgment',
            'fields': [
                ('bool', 'success', 1, 'Operation success/failure'),
            ]
        },
        'Timestamp': {
            'comment': 'Point in time (epoch milliseconds)',
            'fields': [
                ('int64', 'epoch_ms', 1, 'Milliseconds since Unix epoch'),
            ]
        },
        'Angle': {
            'comment': 'Angular measurement in radians (SI unit)',
            'fields': [
                ('double', 'radians', 1, 'Angle value in radians')
            ]
        },
        'Length': {
            'comment': 'Linear distance in meters (SI unit)',
            'fields': [
                ('double', 'meters', 1, 'Length value in meters')
            ]
        },
        'Speed': {
            'comment': 'Velocity in meters per second (SI unit)',
            'fields': [
                ('double', 'meters_per_second', 1, 'Speed value in m/s')
            ]
        },
        'Percentage': {
            'comment': 'Percentage value (0-100)',
            'fields': [
                ('double', 'value', 1, 'Percentage value (0.0 to 100.0)')
            ]
        },
        'Quality': {
            'comment': 'Quality metric (normalized 0-1)',
            'fields': [
                ('double', 'value', 1, 'Quality value (0.0 to 1.0)')
            ]
        }
    }
    
    def __init__(self, model: Dict[str, Any], output_dir: str = 'proto'):
        self.model = model
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Track all types for imports
        self.generated_types = set()
        self.field_counter = 1  # Protobuf field numbers start at 1
        
        # Build a complete type-to-package map for qualified names
        self.type_to_package = {}
        self._build_type_package_map()
    
    def _build_type_package_map(self):
        """Build a map of type names to their fully qualified package names"""
        for namespace_path, namespace_types in self._group_by_namespace().items():
            sanitized = self._sanitize_namespace(namespace_path)
            pkg = 'pyramid.' + '.'.join(sanitized) if sanitized else 'pyramid.model'
            
            for dt in namespace_types['dataTypes']:
                self.type_to_package[dt['name']] = pkg
            for enum in namespace_types['enumerations']:
                self.type_to_package[enum['name']] = pkg
            for cls in namespace_types['classes']:
                self.type_to_package[cls['name']] = pkg
        
    def generate(self):
        """Generate all .proto files from the model"""
        print(f"Generating protobuf files in {self.output_dir}/")
        
        # Group types by namespace
        types_by_namespace = self._group_by_namespace()
        
        # Generate a .proto file for each namespace
        for namespace_path, types in types_by_namespace.items():
            self._generate_proto_file(namespace_path, types)
        
        print(f"[x] Generated {len(types_by_namespace)} .proto files")
    
    def _group_by_namespace(self) -> Dict[tuple, Dict[str, List]]:
        """Group all types by their namespace, combining consumed/provided into components"""
        grouped = {}
        
        # Process data types
        for dt in self.model.get('dataTypes', []):
            ns = tuple(dt.get('namespace', []))
            ns = self._normalize_namespace(ns)
            if ns not in grouped:
                grouped[ns] = {'dataTypes': [], 'enumerations': [], 'classes': []}
            grouped[ns]['dataTypes'].append(dt)
        
        # Process enumerations
        for enum in self.model.get('enumerations', []):
            ns = tuple(enum.get('namespace', []))
            ns = self._normalize_namespace(ns)
            if ns not in grouped:
                grouped[ns] = {'dataTypes': [], 'enumerations': [], 'classes': []}
            grouped[ns]['enumerations'].append(enum)
        
        # Process classes - combine consumed/provided into parent component
        for cls in self.model.get('classes', []):
            ns = tuple(cls.get('namespace', []))
            ns = self._normalize_namespace(ns)
            if ns not in grouped:
                grouped[ns] = {'dataTypes': [], 'enumerations': [], 'classes': []}
            
            # Add service type annotation (consumed/provided)
            cls_copy = cls.copy()
            cls_copy['serviceType'] = self._get_service_type(cls.get('namespace', []))
            grouped[ns]['classes'].append(cls_copy)
        
        return grouped
    
    def _normalize_namespace(self, namespace: tuple) -> tuple:
        """Normalize namespace to group all provided services together and all consumed together
        
        Example:
            ('Components', 'Tactical Objects', 'Services', '1. Provided', 'Matching Objects') 
              -> ('Components', 'Tactical Objects', 'Services', 'Provided')
            ('Components', 'Tactical Objects', 'Services', '2. Consumed', 'Source Data')
              -> ('Components', 'Tactical Objects', 'Services', 'Consumed')
        """
        if not namespace:
            return namespace
        
        # Find if there's a provided/consumed level in the hierarchy
        normalized = list(namespace)
        service_type_idx = None
        
        for i, part in enumerate(normalized):
            part_lower = part.lower()
            if any(keyword in part_lower for keyword in ['provided', 'consumed', 'required', 'offered']):
                service_type_idx = i
                # Normalize to simple "Provided" or "Consumed"
                if 'provided' in part_lower or 'offered' in part_lower:
                    normalized[i] = 'Provided'
                elif 'consumed' in part_lower or 'required' in part_lower:
                    normalized[i] = 'Consumed'
                break
        
        # If we found a service type, remove everything after it (sub-packages)
        if service_type_idx is not None and service_type_idx < len(normalized) - 1:
            normalized = normalized[:service_type_idx + 1]
        
        return tuple(normalized)
    
    def _get_service_type(self, namespace: List[str]) -> str:
        """Determine if a service is consumed or provided based on namespace"""
        if not namespace:
            return 'unknown'
        
        last = namespace[-1].lower()
        
        if 'provided' in last or 'offered' in last:
            return 'provided'
        elif 'consumed' in last or 'required' in last:
            return 'consumed'
        
        return 'unknown'
    
    def _generate_proto_file(self, namespace_path: tuple, types: Dict[str, List]):
        """Generate a single .proto file for a namespace"""
        from io import StringIO
        
        # Create package name from namespace with pyramid prefix
        sanitized = self._sanitize_namespace(namespace_path)
        if sanitized:
            package_name = 'pyramid.' + '.'.join(sanitized)
        else:
            package_name = 'pyramid.model'
        
        # Keep package-style filenames, but place them under stable
        # `pyramid/data_model` or `pyramid/components` folders.
        filepath = self.output_dir / self._proto_relpath_for_package(package_name)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        # Use StringIO to buffer content before writing
        buffer = StringIO()
        content_written = False
        
        # Check which google imports are needed
        google_imports = self._needs_google_imports(types)
        
        # Generate enumerations first (they may be used by messages)
        enum_buffer = StringIO()
        for enum in types['enumerations']:
            self._write_enum(enum_buffer, enum)
            enum_buffer.write('\n')
            content_written = True
        
        # Generate messages from data types
        dt_buffer = StringIO()
        for dt in types['dataTypes']:
            written = self._write_message(dt_buffer, dt, package_name)
            if written:
                dt_buffer.write('\n')
                content_written = True
        
        # Generate messages from classes (services)
        cls_buffer = StringIO()
        for cls in types['classes']:
            written = self._write_message_from_class(cls_buffer, cls, package_name)
            if written:
                cls_buffer.write('\n')
                content_written = True
        
        # Only write file if there's actual content
        if not content_written:
            return  # Skip empty files
        
        with open(filepath, 'w') as f:
            # Write header
            f.write('syntax = "proto3";\n\n')
            f.write(f'package {package_name};\n\n')
            
            # Add imports
            imports = []
            
            # Add pyramid.base import if services use Identifier
            if google_imports['base_for_identifier']:
                # Check if Identifier is not in current package
                if 'Identifier' in self.type_to_package and self.type_to_package['Identifier'] != package_name:
                    imports.append(
                        f'import "{self._proto_relpath_for_package(self.type_to_package["Identifier"]).as_posix()}";'
                    )
            
            # Add imports for types from other packages
            type_imports = self._collect_imports(types, package_name)
            for import_path in type_imports:
                imports.append(f'import "{import_path}";')
            
            if imports:
                f.write('\n'.join(imports))
                f.write('\n\n')
            
            # Write accumulated content
            f.write(enum_buffer.getvalue())
            f.write(dt_buffer.getvalue())
            f.write(cls_buffer.getvalue())
        
        print(f"  Generated {filepath}")
    
    def _sanitize_namespace(self, namespace_path: tuple) -> List[str]:
        """Convert namespace path to valid protobuf package names"""
        sanitized = []
        for part in namespace_path:
            # Remove numbers and dots from start, convert to lowercase
            cleaned = part.strip()
            # Remove leading numbers and dots
            while cleaned and (cleaned[0].isdigit() or cleaned[0] in '. '):
                cleaned = cleaned[1:]
            # Replace spaces with underscores
            cleaned = cleaned.replace(' ', '_').replace('-', '_')
            # Convert to lowercase
            cleaned = cleaned.lower()
            if cleaned:
                sanitized.append(cleaned)
        return sanitized
    
    def _collect_imports(self, types: Dict[str, List], package_name: str) -> List[str]:
        """Collect imports needed for types referenced from other packages"""
        imports = set()
        
        # Build a map of type name to package
        type_to_package = {}
        for namespace_path, namespace_types in self._group_by_namespace().items():
            sanitized = self._sanitize_namespace(namespace_path)
            pkg = 'pyramid.' + '.'.join(sanitized) if sanitized else 'pyramid.model'
            
            for dt in namespace_types['dataTypes']:
                type_to_package[dt['name']] = pkg
            for enum in namespace_types['enumerations']:
                type_to_package[enum['name']] = pkg
            for cls in namespace_types['classes']:
                type_to_package[cls['name']] = pkg
        
        # Check data types for external references
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                type_name = prop.get('typeName')
                if type_name and type_name in type_to_package:
                    ref_package = type_to_package[type_name]
                    if ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())
            
            # Check parent types
            for parent in dt.get('generalizes', []):
                if parent in type_to_package:
                    ref_package = type_to_package[parent]
                    if ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())
        
        # Check classes for external references
        for cls in types['classes']:
            for prop in cls.get('properties', []):
                type_name = prop.get('typeName')
                if type_name and type_name in type_to_package:
                    ref_package = type_to_package[type_name]
                    if ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())
            
            # Check parent types - only import if they have non-entity properties
            for parent in cls.get('generalizes', []):
                if parent in type_to_package:
                    # Check if parent has non-entity properties before importing
                    if self._type_has_non_entity_properties(parent):
                        ref_package = type_to_package[parent]
                        if ref_package != package_name:
                            imports.add(self._proto_relpath_for_package(ref_package).as_posix())

        # Service generation introduces RPC signature dependencies that are not
        # present as ordinary message fields. In particular EntityActions
        # services synthesize Query/Ack/Identifier plus inherited entity
        # payload types, so force imports for those packages here.
        for cls in types['classes']:
            entity_properties = self._collect_all_entity_properties(cls)
            if not entity_properties:
                continue

            for prop in entity_properties:
                prop_type = prop.get('type')
                if prop_type and prop_type in type_to_package:
                    ref_package = type_to_package[prop_type]
                    if ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())

                flow_direction = prop.get('flow_direction', 'inout') or 'inout'
                support_types = {'Identifier'}
                if flow_direction in ('inout', 'out'):
                    support_types.add('Query')
                if flow_direction in ('inout', 'in'):
                    support_types.add('Ack')

                for support_type in support_types:
                    ref_package = type_to_package.get(support_type)
                    if ref_package and ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())

        return sorted(imports)

    def _proto_relpath_for_package(self, package_name: str) -> Path:
        """Map a proto package to the repo's foldered proto layout."""
        parts = package_name.split('.')
        if len(parts) >= 2 and parts[0] == 'pyramid' and parts[1] in ('data_model', 'components'):
            return Path('pyramid') / parts[1] / f'{package_name}.proto'
        return Path('pyramid') / f'{package_name}.proto'
    
    def _needs_google_imports(self, types: Dict[str, List]) -> Dict[str, bool]:
        """Check which google protobuf imports are needed"""
        needs = {'timestamp': False, 'base_for_identifier': False}

        # No google.protobuf.Empty -- EntityActions uses Ack instead

        for cls in types['classes']:
            for prop in cls.get('properties', []):
                type_name = prop.get('typeName')
                if type_name and self._is_entity_derived(type_name):
                    needs['base_for_identifier'] = True

        return needs
    
    def _write_enum(self, f, enum: Dict[str, Any]):
        """Write an enumeration definition"""
        name = enum['name']
        
        # Write documentation if available
        doc = enum.get('documentation')
        if doc:
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'// {line.strip()}\n')
        
        f.write(f'enum {name} {{\n')
        
        # Add unspecified value at 0 (protobuf best practice)
        enum_prefix = self._to_snake_case(name).upper()
        f.write(f'  {enum_prefix}_UNSPECIFIED = 0;\n')
        
        # Write enum values with documentation
        for idx, literal in enumerate(enum.get('literals', []), start=1):
            lit_name = literal['name']
            lit_doc = literal.get('documentation')
            
            # Write literal documentation if available
            if lit_doc:
                for line in lit_doc.split('\n'):
                    if line.strip():
                        f.write(f'  // {line.strip()}\n')
            
            f.write(f'  {enum_prefix}_{self._to_snake_case(lit_name).upper()} = {idx};\n')
        
        f.write('}\n')
    
    def _write_message(self, f, datatype: Dict[str, Any], current_package: str):
        """Write a message definition from a DataType"""
        name = datatype['name']
        
        # Check if this is a base type that should be auto-filled
        if name in self.BASE_TYPE_DEFINITIONS and not datatype.get('properties'):
            base_def = self.BASE_TYPE_DEFINITIONS[name]
            
            # Special handling for Timestamp - use google.protobuf.Timestamp
            if base_def.get('use_google_timestamp'):
                f.write(f'// {base_def["comment"]}\n')
                f.write(f'// Note: Use google.protobuf.Timestamp directly instead of this message\n')
                f.write(f'message {name} {{\n')
                f.write(f'  google.protobuf.Timestamp value = 1;\n')
                f.write('}\n')
                return True  # Message was written
            
            # Write base type with standard definition
            f.write(f'// {base_def["comment"]}\n')
            f.write(f'message {name} {{\n')
            for field_type, field_name, field_num, field_comment in base_def['fields']:
                if field_comment:
                    f.write(f'  {field_type} {field_name} = {field_num};  // {field_comment}\n')
                else:
                    f.write(f'  {field_type} {field_name} = {field_num};\n')
            f.write('}\n')
            return True  # Message was written
        
        # Check if message is empty (no properties and no parent)
        has_content = bool(datatype.get('properties') or datatype.get('generalizes'))
        if not has_content:
            return False  # Skip empty message
        
        # Standard message generation for non-base types
        # Write documentation if available
        doc = datatype.get('documentation')
        if doc:
            # Handle multi-line documentation
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'// {line.strip()}\n')
        
        f.write(f'message {name} {{\n')
        
        # Handle inheritance using composition
        generalizes = datatype.get('generalizes', [])
        field_num = 1
        
        if generalizes:
            # Add parent as first field (composition pattern)
            # But skip if parent is an empty abstract type (just a marker interface)
            for parent_name in generalizes:
                # Check if parent has any properties
                parent_has_properties = self._type_has_properties(parent_name)
                if parent_has_properties:
                    # Qualify parent type if from different package
                    qualified_parent = self._get_qualified_type(parent_name, current_package)
                    f.write(f'  {qualified_parent} base = {field_num};  // Inherited from {parent_name}\n')
                    field_num += 1
            if datatype.get('properties') and field_num > 1:
                f.write('\n')  # Blank line between base and own fields only if base was written
        
        # Write own fields
        for prop in datatype.get('properties', []):
            type_name = prop.get('typeName')
            field_name = self._to_snake_case(prop['name'])
            
            # Check if property type is abstract - generate oneof
            if type_name and self._is_abstract(type_name):
                concrete_children = self._find_concrete_children(type_name)
                if concrete_children:
                    # Generate oneof block
                    f.write(f'  oneof {field_name} {{\n')
                    for child in concrete_children:
                        child_qualified = self._get_qualified_type(child, current_package)
                        child_field = self._to_snake_case(child)
                        f.write(f'    {child_qualified} {child_field} = {field_num};\n')
                        field_num += 1
                    f.write(f'  }}\n')
                    continue
            
            # Regular field handling
            field_type = self._get_qualified_type(type_name, current_package)
            
            # Write property documentation if available
            prop_doc = prop.get('documentation')
            if prop_doc:
                for line in prop_doc.split('\n'):
                    if line.strip():
                        f.write(f'  // {line.strip()}\n')
            
            # Handle multiplicity
            mult_lower = prop.get('multiplicity', {}).get('lower', '1')
            mult_upper = prop.get('multiplicity', {}).get('upper', '1')
            
            if mult_upper == '*':
                # Repeated field
                f.write(f'  repeated {field_type} {field_name} = {field_num};\n')
            elif mult_lower == '0':
                # Optional field (implicit in proto3)
                f.write(f'  optional {field_type} {field_name} = {field_num};\n')
            else:
                # Required field (standard in proto3)
                f.write(f'  {field_type} {field_name} = {field_num};\n')
            
            field_num += 1
        
        f.write('}\n')
        return True  # Message was written
    
    def _collect_all_entity_properties(self, cls: Dict[str, Any]) -> List[Dict]:
        """Collect entity properties from class and all ancestors"""
        all_entity_props = []
        
        # First collect from parent classes (depth-first)
        for parent_name in cls.get('generalizes', []):
            parent_class = self._find_class_by_name(parent_name)
            if parent_class:
                # Recursively collect from parent
                parent_props = self._collect_all_entity_properties(parent_class)
                all_entity_props.extend(parent_props)
        
        # Then add this class's entity properties
        for prop in cls.get('properties', []):
            type_name = prop.get('typeName')
            if type_name and self._is_entity_derived(type_name):
                all_entity_props.append({
                    'name': self._to_snake_case(prop['name']),
                    'type': type_name,
                    'flow_direction': prop.get('flow_direction', 'inout'),
                    'multiplicity': prop.get('multiplicity', {})
                })
        
        return all_entity_props
    
    def _class_has_non_entity_content(self, class_name: str) -> bool:
        """Check if a class (recursively including parents) has any non-entity properties"""
        cls = self._find_class_by_name(class_name)
        if not cls:
            return False
        
        # Check this class's properties
        for prop in cls.get('properties', []):
            type_name = prop.get('typeName')
            if not (type_name and self._is_entity_derived(type_name)):
                # Found a non-entity property
                return True
        
        # Recursively check parents
        for parent_name in cls.get('generalizes', []):
            if self._class_has_non_entity_content(parent_name):
                return True
        
        return False
    
    def _find_class_by_name(self, class_name: str) -> Optional[Dict]:
        """Find a class definition by name"""
        # Search through all classes in the model
        for cls in self.model.get('classes', []):
            if cls['name'] == class_name:
                return cls
        return None
    
    def _write_message_from_class(self, f, cls: Dict[str, Any], current_package: str):
        """Write a message definition from a Class"""
        name = cls['name']
        service_type = cls.get('serviceType', 'unknown')
        
        # Check if message is empty (no properties and no parent)
        has_content = bool(cls.get('properties') or cls.get('generalizes'))
        if not has_content:
            return False  # Skip empty message
        
        # Track entity-derived properties for service generation
        # Collect only THIS class's entity properties (not inherited)
        entity_properties = []
        non_entity_properties = []
        
        for prop in cls.get('properties', []):
            type_name = prop.get('typeName')
            if type_name and self._is_entity_derived(type_name):
                entity_properties.append({
                    'name': self._to_snake_case(prop['name']),
                    'type': type_name,
                    'flow_direction': prop.get('flow_direction', 'inout'),
                    'multiplicity': prop.get('multiplicity', {})
                })
            else:
                non_entity_properties.append(prop)
        
        # Collect ALL entity properties including inherited for service generation
        all_entity_properties = self._collect_all_entity_properties(cls)
        
        # Check if any parent has non-entity content
        parent_has_non_entity = False
        for parent_name in cls.get('generalizes', []):
            if self._class_has_non_entity_content(parent_name):
                parent_has_non_entity = True
                break
        
        # Skip message if this class AND all parents only have entity properties
        skip_message = (entity_properties and 
                       not non_entity_properties and 
                       not parent_has_non_entity)
        
        # If skipping message, just generate service
        if skip_message:
            self._write_grpc_service(f, name, all_entity_properties, current_package)
            return True  # Service was written, counts as content
        
        # Write service type indicator
        if service_type in ['provided', 'consumed']:
            f.write(f'// Service Type: {service_type.upper()}\n')
        
        # Write documentation if available
        doc = cls.get('documentation')
        if doc:
            # Handle multi-line documentation
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'// {line.strip()}\n')
        
        f.write(f'message {name} {{\n')
        
        # Handle inheritance using composition
        generalizes = cls.get('generalizes', [])
        field_num = 1
        
        if generalizes:
            # Add parent as first field (composition pattern)
            # But skip if parent only has entity properties or is empty
            for parent_name in generalizes:
                # Check if parent has any non-entity properties
                parent_has_non_entity_properties = self._type_has_non_entity_properties(parent_name)
                if parent_has_non_entity_properties:
                    qualified_parent = self._get_qualified_type(parent_name, current_package)
                    f.write(f'  {qualified_parent} base = {field_num};  // Inherited from {parent_name}\n')
                    field_num += 1
            if cls.get('properties') and field_num > 1:
                f.write('\n')  # Blank line between base and own fields only if base was written
        
        # Write only non-entity fields (entity fields become service operations)
        for prop in non_entity_properties:
            type_name = prop.get('typeName')
            field_name = self._to_snake_case(prop['name'])
            
            # Check if property type is abstract - generate oneof
            if type_name and self._is_abstract(type_name):
                concrete_children = self._find_concrete_children(type_name)
                if concrete_children:
                    # Generate oneof block
                    f.write(f'  oneof {field_name} {{\n')
                    for child in concrete_children:
                        child_qualified = self._get_qualified_type(child, current_package)
                        child_field = self._to_snake_case(child)
                        f.write(f'    {child_qualified} {child_field} = {field_num};\n')
                        field_num += 1
                    f.write(f'  }}\n')
                    continue
            
            # Regular field handling
            field_type = self._get_qualified_type(type_name, current_package)
            
            # Write property documentation if available
            prop_doc = prop.get('documentation')
            if prop_doc:
                for line in prop_doc.split('\n'):
                    if line.strip():
                        f.write(f'  // {line.strip()}\n')
            
            # Handle multiplicity
            mult_lower = prop.get('multiplicity', {}).get('lower', '1')
            mult_upper = prop.get('multiplicity', {}).get('upper', '1')
            
            if mult_upper == '*':
                f.write(f'  repeated {field_type} {field_name} = {field_num};\n')
            elif mult_lower == '0':
                f.write(f'  optional {field_type} {field_name} = {field_num};\n')
            else:
                f.write(f'  {field_type} {field_name} = {field_num};\n')
            
            field_num += 1
        
        f.write('}\n')
        
        # Generate gRPC service with ALL entity properties (including inherited)
        if all_entity_properties:
            f.write('\n')
            self._write_grpc_service(f, name, all_entity_properties, current_package)
        
        # Handle operations from the class itself
        operations = cls.get('operations', [])
        if operations:
            f.write(f'\n// TODO: Service operations for {name}:\n')
            for op in operations:
                f.write(f'// - {op["name"]}()\n')
        
        return True  # Message was written
    
    def _is_entity_derived(self, type_name: str) -> bool:
        """Check if a type derives from Entity (by checking all parsed types)"""
        # Search through all data types to see if this type derives from Entity
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if dt['name'] == type_name:
                    generalizes = dt.get('generalizes', [])
                    if 'Entity' in generalizes:
                        return True
                    # Recursively check parent types
                    for parent in generalizes:
                        if self._is_entity_derived(parent):
                            return True
            # Also check classes
            for cls in namespace_types['classes']:
                if cls['name'] == type_name:
                    generalizes = cls.get('generalizes', [])
                    if 'Entity' in generalizes:
                        return True
                    for parent in generalizes:
                        if self._is_entity_derived(parent):
                            return True
        return False
    
    def _write_entity_service(self, f, service_name: str, entity_properties: List[Dict], current_package: str):
        """Generate EntityActions service definition for entity-derived properties.

        Produces Create/Read/Update/Delete RPCs with correct EntityActions signatures.
        This is a semantic contract consumed by codex generators -- not a gRPC endpoint.
        """
        f.write(f'// EntityActions Service for {service_name}\n')
        f.write(f'service {service_name}_Service {{\n')

        for prop in entity_properties:
            prop_name = prop['name']
            prop_type = prop['type']
            qualified_type = self._get_qualified_type(prop_type, current_package)
            flow_direction = prop.get('flow_direction', 'inout')

            qualified_identifier = self._get_qualified_type('Identifier', current_package)
            qualified_query = self._get_qualified_type('Query', current_package)
            qualified_ack = self._get_qualified_type('Ack', current_package)

            prop_camel = ''.join(word.capitalize() for word in prop_name.split('_'))

            # inout -> full CRUD
            # out   -> Read only
            # in    -> Create + Update + Delete (no Read)

            if flow_direction == 'inout' or flow_direction is None:
                f.write(f'  rpc Create{prop_camel} ({qualified_type}) returns ({qualified_identifier});\n')
                f.write(f'  rpc Read{prop_camel}   ({qualified_query}) returns (stream {qualified_type});\n')
                f.write(f'  rpc Update{prop_camel} ({qualified_type}) returns ({qualified_ack});\n')
                f.write(f'  rpc Delete{prop_camel} ({qualified_identifier}) returns ({qualified_ack});\n')
            elif flow_direction == 'out':
                f.write(f'  rpc Read{prop_camel} ({qualified_query}) returns (stream {qualified_type});\n')
            elif flow_direction == 'in':
                f.write(f'  rpc Create{prop_camel} ({qualified_type}) returns ({qualified_identifier});\n')
                f.write(f'  rpc Update{prop_camel} ({qualified_type}) returns ({qualified_ack});\n')
                f.write(f'  rpc Delete{prop_camel} ({qualified_identifier}) returns ({qualified_ack});\n')

            if entity_properties.index(prop) < len(entity_properties) - 1:
                f.write('\n')

        f.write('}\n')

    # Keep old name as alias so callers that weren't updated still work
    _write_grpc_service = _write_entity_service
    
    def _map_type(self, sysml_type: str) -> str:
        """Map SysML type to protobuf type"""
        if not sysml_type:
            return 'int32'  # Default type
        
        # Check mapping table
        if sysml_type in self.TYPE_MAPPING:
            return self.TYPE_MAPPING[sysml_type]
        
        # Assume it's a user-defined message type
        return sysml_type
    
    def _get_qualified_type(self, type_name: str, current_package: str) -> str:
        """Get fully qualified type name if type is from different package"""
        if not type_name:
            return 'int32'
        
        # Check if it's a mapped type
        if type_name in self.TYPE_MAPPING:
            mapped_type = self.TYPE_MAPPING[type_name]
            # If mapped to a custom type (not a proto primitive), need to qualify it
            if mapped_type in self.type_to_package:
                mapped_package = self.type_to_package[mapped_type]
                if mapped_package != current_package:
                    return f'{mapped_package}.{mapped_type}'
            return mapped_type
        
        # Check if type is from another package
        if type_name in self.type_to_package:
            type_package = self.type_to_package[type_name]
            if type_package != current_package:
                # Need to qualify with package name
                return f'{type_package}.{type_name}'
        
        # Same package or unknown - use unqualified name
        return type_name
    
    def _is_abstract(self, type_name: str) -> bool:
        """Check if a type is marked as abstract"""
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if dt['name'] == type_name:
                    return dt.get('isAbstract', False)
            for cls in namespace_types['classes']:
                if cls['name'] == type_name:
                    return cls.get('isAbstract', False)
        return False
    
    def _find_concrete_children(self, abstract_type: str) -> List[str]:
        """Find all concrete types that inherit from an abstract type"""
        children = []
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if abstract_type in dt.get('generalizes', []):
                    if not dt.get('isAbstract', False):
                        children.append(dt['name'])
            for cls in namespace_types['classes']:
                if abstract_type in cls.get('generalizes', []):
                    if not cls.get('isAbstract', False):
                        children.append(cls['name'])
        return children
    
    def _type_has_non_entity_properties(self, type_name: str) -> bool:
        """Check if a type has any non-entity properties defined"""
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if dt['name'] == type_name:
                    # DataTypes always have non-entity properties
                    return len(dt.get('properties', [])) > 0
            for cls in namespace_types['classes']:
                if cls['name'] == type_name:
                    # Check if has any non-entity properties
                    for prop in cls.get('properties', []):
                        type_name_prop = prop.get('typeName')
                        if not (type_name_prop and self._is_entity_derived(type_name_prop)):
                            return True  # Has at least one non-entity property
                    return False
        return False
    
    def _type_has_properties(self, type_name: str) -> bool:
        """Check if a type has any non-entity properties defined"""
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if dt['name'] == type_name:
                    return len(dt.get('properties', [])) > 0
            for cls in namespace_types['classes']:
                if cls['name'] == type_name:
                    # Check if class has any non-entity properties
                    for prop in cls.get('properties', []):
                        type_name_prop = prop.get('typeName')
                        if not (type_name_prop and self._is_entity_derived(type_name_prop)):
                            # Found a non-entity property
                            return True
                    # All properties are entity-derived, no message needed
                    return False
        return False
    
    def _to_snake_case(self, name: str) -> str:
        """Convert CamelCase or kebab-case to snake_case"""
        # Handle spaces first
        name = name.replace(' ', '_').replace('-', '_')
        
        # Insert underscores before uppercase letters
        result = []
        for i, char in enumerate(name):
            if char.isupper() and i > 0 and name[i-1].islower():
                result.append('_')
            result.append(char.lower())
        
        return ''.join(result)


def main():
    if len(sys.argv) < 2:
        print("Usage: python proto_generator.py <model.json> [output_dir]")
        print("\nGenerates Protocol Buffer .proto files from parsed SysML JSON model")
        sys.exit(1)
    
    model_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'proto'
    
    # Load the model
    with open(model_file, 'r') as f:
        model = json.load(f)
    
    # Generate protobuf files
    generator = ProtobufGenerator(model, output_dir)
    generator.generate()
    
    print(f"\n[x] Protobuf generation complete!")
    print(f"  Output directory: {output_dir}/")


if __name__ == '__main__':
    main()
