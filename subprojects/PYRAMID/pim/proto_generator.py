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
        'Timestamp': {
            'comment': 'Point in time (uses google.protobuf.Timestamp)',
            'use_google_timestamp': True  # Special flag to use google timestamp directly
        },
        'Time': {
            'comment': 'Time value in seconds (SI unit)',
            'fields': [
                ('double', 'seconds', 1, 'Time value in seconds')
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
        self.type_to_package = {}  # Backward-compatible name -> package map
        self.type_id_to_package = {}
        self.type_id_to_name = {}
        self.name_to_ids = {}
        self._build_type_package_map()
    
    def _build_type_package_map(self):
        """Map every message-producing type to its `pyramid.data_model.*` package.

        Interface blocks are realised as services, not messages, so they are not
        registered here (they are never referenced as proto field types).
        """
        for dt in self.model.get('dataTypes', []):
            self._register_type(dt, self._data_package(dt.get('namespace', [])))
        for enum in self.model.get('enumerations', []):
            self._register_type(enum, self._data_package(enum.get('namespace', [])))
        for cls in self.model.get('classes', []):
            if cls.get('stereotype') == 'interfaceBlock':
                continue
            self._register_type(cls, self._data_package(cls.get('namespace', [])))
        
    def _register_type(self, type_def: Dict[str, Any], pkg: str):
        """Register a type by UUID/XMI id first, with name lookup as a fallback."""
        name = type_def.get('name')
        type_id = type_def.get('id')
        if not name:
            return

        if type_id:
            self.type_id_to_package[type_id] = pkg
            self.type_id_to_name[type_id] = name
            self.name_to_ids.setdefault(name, set()).add(type_id)

        # Preserve old behavior for models that do not carry ids. If duplicate names exist,
        # UUID-based lookup remains authoritative via typeId/generalizesIds.
        self.type_to_package.setdefault(name, pkg)

    def _type_name(self, ref: Optional[str]) -> Optional[str]:
        """Return the display/type name for a UUID/XMI id or already-resolved name."""
        if not ref:
            return ref
        return self.type_id_to_name.get(ref, ref)

    def _prop_type_ref(self, prop: Dict[str, Any]) -> Optional[str]:
        """Prefer the original UUID/XMI type reference; fall back to resolved name."""
        return prop.get('typeId') or prop.get('type') or prop.get('typeName')

    def _generalization_refs(self, element: Dict[str, Any]) -> List[str]:
        """Prefer original generalization UUID/XMI refs; fall back to names."""
        return element.get('generalizesIds') or element.get('generalizes', [])

    def generate(self):
        """Generate `pyramid.data_model.*` and `pyramid.components.*.services.*` protos.

        The PYRAMID binding generator discovers data model files by the
        `pyramid.data_model` package prefix and service files by a `.services.`
        package segment, so output is split accordingly: all data into data_model
        packages, and services (only for concrete component blocks) into
        provided/consumed packages.
        """
        print(f"Generating protobuf files in {self.output_dir}/")
        from io import StringIO

        # Known data packages, used to resolve cross-file imports by scanning bodies.
        self._known_packages = set(self.type_id_to_package.values()) | set(self.type_to_package.values())

        # --- Group data (messages) by data_model package ---
        data_groups = {}
        for dt in self.model.get('dataTypes', []):
            data_groups.setdefault(self._data_package(dt.get('namespace', [])),
                                    {'dataTypes': [], 'enumerations': [], 'classes': []})['dataTypes'].append(dt)
        for enum in self.model.get('enumerations', []):
            data_groups.setdefault(self._data_package(enum.get('namespace', [])),
                                   {'dataTypes': [], 'enumerations': [], 'classes': []})['enumerations'].append(enum)
        for cls in self.model.get('classes', []):
            if cls.get('stereotype') == 'interfaceBlock':
                continue
            data_groups.setdefault(self._data_package(cls.get('namespace', [])),
                                   {'dataTypes': [], 'enumerations': [], 'classes': []})['classes'].append(cls)

        # --- Group services (port-driven) by component + direction ---
        # Only project components emit services; their ports may be inherited from
        # the generic component they generalize/refine.
        service_groups = {}
        for cls in self.model.get('classes', []):
            if not self._is_component_block(cls):
                continue
            for port in self._component_ports(cls):
                if port.get('kind') not in ('request', 'information'):
                    continue
                pkg = self._service_package(cls.get('namespace', []), port.get('direction'))
                service_groups.setdefault(pkg, []).append((cls, port))

        self._known_packages |= set(service_groups.keys())

        self._generated_packages = set(data_groups) | set(service_groups)
        # Project names (pyramid.components.<project>...) used to keep each
        # project's service payloads to common + that project's own variants.
        self._project_names = {p.split('.')[2] for p in service_groups
                               if len(p.split('.')) > 2}

        count = 0
        for pkg, types in sorted(data_groups.items()):
            if self._write_data_file(pkg, types):
                count += 1
        for pkg, entries in sorted(service_groups.items()):
            if self._write_service_file(pkg, entries):
                count += 1
        print(f"Generated {count} .proto files")
        return

    def _file_imports(self, body: str, current_package: str) -> List[str]:
        """Derive imports by scanning a body for fully-qualified cross-package references.

        A qualified reference is `pyramid.<lowercase pkg segments>.<TypeName>`; the
        package is the maximal lowercase dotted prefix before a Capitalised type.
        """
        import re
        imports = []
        if 'google.protobuf.Timestamp' in body:
            imports.append('import "google/protobuf/timestamp.proto";')
        if 'google.protobuf.Empty' in body:
            imports.append('import "google/protobuf/empty.proto";')
        pkg_imports = set()
        for m in re.finditer(r'(pyramid(?:\.[a-z_][a-z0-9_]*)+)\.[A-Z][A-Za-z0-9_]*', body):
            pkg = m.group(1)
            if pkg == current_package or pkg not in self._generated_packages:
                continue
            pkg_imports.add(f'import "{self._proto_relpath_for_package(pkg).as_posix()}";')
        return imports + sorted(pkg_imports)

    def _write_file(self, package_name: str, body: str) -> bool:
        """Write a .proto file with header + scanned imports, skipping empty bodies."""
        if not body.strip():
            return False
        filepath = self.output_dir / self._proto_relpath_for_package(package_name)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, 'w') as f:
            f.write('syntax = "proto3";\n\n')
            f.write(f'package {package_name};\n\n')
            imports = self._file_imports(body, package_name)
            if imports:
                f.write('\n'.join(imports))
                f.write('\n\n')
            f.write(body)
        print(f"  Generated {filepath}")
        return True

    def _write_data_file(self, package_name: str, types: Dict[str, List]) -> bool:
        """Write a data_model proto: enums + messages from dataTypes and classes."""
        from io import StringIO
        buf = StringIO()
        for enum in types['enumerations']:
            self._write_enum(buf, enum)
            buf.write('\n')
        for dt in types['dataTypes']:
            if self._write_message(buf, dt, package_name):
                buf.write('\n')
        for cls in types['classes']:
            if self._write_message_from_class(buf, cls, package_name):
                buf.write('\n')
        return self._write_file(package_name, buf.getvalue())

    def _write_service_file(self, package_name: str, entries: List) -> bool:
        """Write a services.{provided,consumed} proto from component port entries."""
        from io import StringIO
        buf = StringIO()
        seen = set()
        for cls, port in entries:
            iface = self._find_class_by_name(port.get('typeId') or port.get('typeName'))
            if self._write_one_port_service(buf, port, iface, package_name, seen):
                buf.write('\n')
        return self._write_file(package_name, buf.getvalue())

        print(f"? Generated {len(types_by_namespace)} .proto files")
    
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
    
    # Structural/category segments that must not become the component package
    # name. Everything maps to the agreed two-segment `pyramid.<project>.<component>`.
    CATEGORY_SEGMENTS = {
        'component', 'components', 'service', 'services', 'provided', 'consumed',
        'required', 'offered', 'structures', 'structural', 'structure',
        'requirements', 'behavior', 'behaviour', 'semantics', 'bridge', 'bridges',
    }

    def _clean_segment(self, part: str) -> str:
        """Strip enumeration prefixes/spaces and lowercase a single namespace segment."""
        cleaned = part.strip()
        while cleaned and (cleaned[0].isdigit() or cleaned[0] in '. '):
            cleaned = cleaned[1:]
        cleaned = cleaned.replace(' ', '_').replace('-', '_').lower()
        return cleaned

    def _project_component(self, namespace) -> tuple:
        """Resolve a namespace to (project, component) cleaned segments.

        project = first segment, unless it is 'Projects' -> the next segment.
        component = first following segment that is not a structural/category word.
        """
        cleaned = [c for c in (self._clean_segment(p) for p in (namespace or [])) if c]
        if not cleaned:
            return ('model', None)
        if cleaned[0] == 'projects' and len(cleaned) > 1:
            project, idx = cleaned[1], 2
        else:
            project, idx = cleaned[0], 1
        component = None
        for seg in cleaned[idx:]:
            if seg not in self.CATEGORY_SEGMENTS:
                component = seg
                break
        return (project, component)

    def _data_package(self, namespace) -> str:
        """Data-model package for a type: `pyramid.data_model.<project>[.<component>]`.

        The shared generic data model collapses to `pyramid.data_model.base` /
        `pyramid.data_model.common`.
        """
        cleaned = [c for c in (self._clean_segment(p) for p in (namespace or [])) if c]
        if 'data_model' in cleaned or 'data' in cleaned:
            if 'base' in cleaned:
                return 'pyramid.data_model.base'
            if 'common' in cleaned:
                return 'pyramid.data_model.common'
        project, component = self._project_component(namespace)
        parts = ['pyramid', 'data_model', project]
        if component and component != project:
            parts.append(component)
        return '.'.join(parts)

    def _service_package(self, namespace, direction: str) -> str:
        """Service package: `pyramid.components.<project>[.<component>].services.<dir>`."""
        project, component = self._project_component(namespace)
        parts = ['pyramid', 'components', project]
        if component and component != project:
            parts.append(component)
        parts.append('services')
        parts.append('provided' if direction == 'provided' else 'consumed')
        return '.'.join(parts)

    def _is_component_block(self, cls: Dict[str, Any]) -> bool:
        """True for a project-specific component that should expose services.

        Only components under a `Projects/<project>/Components` package get service
        packages. The generic layers (Generic PIM, Common PIM Components, PIM) are
        abstract definitions: they emit data_model only and are refined/generalized
        by the project components. Classified ports may be inherited from the
        generic component being generalized (see _component_ports).
        """
        if cls.get('stereotype') != 'block' or cls.get('isAbstract'):
            return False
        cleaned = [self._clean_segment(s) for s in cls.get('namespace', [])]
        if not cleaned or cleaned[0] != 'projects':
            return False
        if not any('component' in seg for seg in cleaned):
            return False
        return any(p.get('kind') in ('request', 'information')
                   for p in self._component_ports(cls))

    def _component_ports(self, cls: Dict[str, Any], visited=None) -> List[Dict[str, Any]]:
        """A component's ports including those inherited from generalized/refined generic components."""
        if visited is None:
            visited = set()
        cid = cls.get('id')
        if cid in visited:
            return []
        visited.add(cid)
        ports = list(cls.get('ports', []))
        for ancestor_ref in (cls.get('generalizesIds') or []) + (cls.get('refines') or []):
            anc = self._find_class_by_name(ancestor_ref)
            if anc and anc.get('stereotype') == 'block':
                ports += self._component_ports(anc, visited)
        # de-dup by port name; a derived component's own port wins
        seen, out = set(), []
        for p in ports:
            if p['name'] in seen:
                continue
            seen.add(p['name'])
            if not self._port_required(p):
                continue
            if self._port_disabled_by_structure_override(cls, p):
                continue
            out.append(p)
        return out

    def _port_required(self, port: Dict[str, Any]) -> bool:
        """Return whether a port should be emitted based on its lower multiplicity."""
        lower = port.get('multiplicity', {}).get('lower', '1')
        try:
            return int(str(lower).strip()) > 0
        except (TypeError, ValueError):
            return True

    def _port_disabled_by_structure_override(self, cls: Dict[str, Any],
                                             port: Dict[str, Any]) -> bool:
        """Return true when a project structure connects this port to a lower-0 override."""
        port_id = port.get('id')
        if not port_id:
            return False

        project, component = self._project_component(cls.get('namespace', []))
        if not project or not component:
            return False

        structure_ids = set()
        disabled_roles = set()
        for candidate in self.model.get('classes', []):
            cand_project, cand_component = self._project_component(
                candidate.get('namespace', []))
            if cand_project != project or cand_component is not None:
                continue
            cleaned = [self._clean_segment(s) for s in candidate.get('namespace', [])]
            if 'structure' not in cleaned:
                continue
            if candidate.get('id'):
                structure_ids.add(candidate['id'])
            for candidate_port in candidate.get('ports', []):
                if not self._port_required(candidate_port) and candidate_port.get('id'):
                    disabled_roles.add(candidate_port['id'])

        if not structure_ids or not disabled_roles:
            return False

        for connector in self.model.get('connectors', []):
            if connector.get('ownerId') not in structure_ids:
                continue
            roles = {end.get('role') for end in connector.get('ends', [])}
            if port_id in roles and roles.intersection(disabled_roles):
                return True
        return False

    def _normalize_namespace(self, namespace: tuple) -> tuple:
        """Legacy (project, component) grouping used only by type-iteration helpers."""
        if not namespace:
            return namespace
        project, component = self._project_component(namespace)
        return (project, component) if component else (project,)
    
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

        # Generate port-driven services for concrete component blocks. Abstract
        # template blocks (e.g. the generic component) are skipped: their ports
        # bind the universal base types, which are realised per concrete component.
        port_buffer = StringIO()
        for cls in types['classes']:
            if cls.get('stereotype') != 'block' or cls.get('isAbstract'):
                continue
            if not any(p.get('kind') in ('request', 'information') for p in cls.get('ports', [])):
                continue
            if self._write_port_services(port_buffer, cls, package_name):
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
            
            # Add google protobuf imports if needed
            if google_imports['timestamp']:
                imports.append('import "google/protobuf/timestamp.proto";')
            if google_imports['empty']:
                imports.append('import "google/protobuf/empty.proto";')
            
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
            f.write(port_buffer.getvalue())
        
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
                if dt.get('id'):
                    type_to_package[dt['id']] = pkg
                type_to_package.setdefault(dt['name'], pkg)
            for enum in namespace_types['enumerations']:
                if enum.get('id'):
                    type_to_package[enum['id']] = pkg
                type_to_package.setdefault(enum['name'], pkg)
            for cls in namespace_types['classes']:
                if cls.get('id'):
                    type_to_package[cls['id']] = pkg
                type_to_package.setdefault(cls['name'], pkg)
        
        # Check data types for external references
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                type_name = self._prop_type_ref(prop)
                if type_name and self._is_interface_block(type_name):
                    continue  # interface-typed fields are not emitted
                if type_name and type_name in type_to_package:
                    ref_package = type_to_package[type_name]
                    if ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())

            # Check parent types
            for parent in self._generalization_refs(dt):
                if parent in type_to_package:
                    ref_package = type_to_package[parent]
                    if ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())

        # Check classes for external references
        for cls in types['classes']:
            if cls.get('stereotype') == 'interfaceBlock':
                continue  # interface blocks emit no message
            for prop in cls.get('properties', []):
                type_name = self._prop_type_ref(prop)
                if type_name and self._is_interface_block(type_name):
                    continue  # interface-typed fields are not emitted
                if type_name and type_name in type_to_package:
                    ref_package = type_to_package[type_name]
                    if ref_package != package_name:
                        imports.add(self._proto_relpath_for_package(ref_package).as_posix())
            
            # Check parent types - only import if they have non-entity properties
            for parent in self._generalization_refs(cls):
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
            if cls.get('stereotype') == 'interfaceBlock':
                continue  # interface blocks emit no message/service
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

        # Port-driven services reference payload types (and their concrete
        # variants) plus Ack/Query/Identifier support types, none of which are
        # ordinary fields in this file.
        def _add_ref(ref):
            pkg = type_to_package.get(ref) or type_to_package.get(self._type_name(ref))
            if pkg and pkg != package_name:
                imports.add(self._proto_relpath_for_package(pkg).as_posix())

        for cls in types['classes']:
            if cls.get('stereotype') != 'block' or cls.get('isAbstract'):
                continue
            for port in cls.get('ports', []):
                if port.get('kind') not in ('request', 'information'):
                    continue
                iface = self._find_class_by_name(port.get('typeId') or port.get('typeName'))
                if not iface:
                    continue
                payloads = self._classify_iface_payloads(iface)
                # Import the payload base types' packages (referenced directly or
                # as a same-package oneof wrapper). Cross-package variants are not
                # expanded (see _payload_message), so their packages are not imported.
                for slot in ('request', 'requirement', 'information'):
                    pref = payloads.get(slot)
                    if pref:
                        _add_ref(pref)
                if port.get('kind') == 'request':
                    for support in ('Ack', 'Query', 'Identifier'):
                        _add_ref(support)

        return sorted(imports)
    
    def _proto_relpath_for_package(self, package_name: str) -> Path:
        """Map a proto package to the repo's foldered proto layout."""
        parts = package_name.split('.')
        if len(parts) >= 2 and parts[0] == 'pyramid' and parts[1] in ('data_model', 'components'):
            return Path('pyramid') / parts[1] / f'{package_name}.proto'
        return Path('pyramid') / f'{package_name}.proto'
    
    def _needs_google_imports(self, types: Dict[str, List]) -> Dict[str, bool]:
        """Check which google protobuf imports are needed"""
        needs = {'timestamp': False, 'empty': False, 'base_for_identifier': False}
        
        # Check if Timestamp base type is in this file (needs google import)
        for dt in types['dataTypes']:
            if dt['name'] == 'Timestamp':
                needs['timestamp'] = True
        
        # Check for Timestamp usage in properties
        for dt in types['dataTypes']:
            for prop in dt.get('properties', []):
                if self._type_name(self._prop_type_ref(prop)) == 'Timestamp':
                    needs['timestamp'] = True
        for cls in types['classes']:
            for prop in cls.get('properties', []):
                if self._type_name(self._prop_type_ref(prop)) == 'Timestamp':
                    needs['timestamp'] = True
                # Check if property is entity-derived (will generate service with Empty and Identifier)
                type_name = self._prop_type_ref(prop)
                if type_name and self._is_entity_derived(type_name):
                    needs['empty'] = True
                    needs['base_for_identifier'] = True

        # Port-driven services use google.protobuf.Empty (information reads and
        # absent request/requirement payloads).
        for cls in types['classes']:
            if cls.get('stereotype') != 'block' or cls.get('isAbstract'):
                continue
            if any(p.get('kind') in ('request', 'information') for p in cls.get('ports', [])):
                needs['empty'] = True

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
        
        f.write(f'enum {self._sanitize_ident(name)} {{\n')
        
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
                f.write(f'message {self._sanitize_ident(name)} {{\n')
                f.write(f'  google.protobuf.Timestamp value = 1;\n')
                f.write('}\n')
                return True  # Message was written
            
            # Write base type with standard definition
            f.write(f'// {base_def["comment"]}\n')
            f.write(f'message {self._sanitize_ident(name)} {{\n')
            for field_type, field_name, field_num, field_comment in base_def['fields']:
                if field_comment:
                    f.write(f'  {field_type} {field_name} = {field_num};  // {field_comment}\n')
                else:
                    f.write(f'  {field_type} {field_name} = {field_num};\n')
            f.write('}\n')
            return True  # Message was written
        
        # An empty dataType still needs to exist as a (valid, empty) message
        # because other messages reference it as a field type.
        has_content = bool(datatype.get('properties') or datatype.get('generalizes'))
        if not has_content:
            doc = datatype.get('documentation')
            if doc:
                for line in doc.split('\n'):
                    if line.strip():
                        f.write(f'// {line.strip()}\n')
            f.write(f'message {self._sanitize_ident(name)} {{}}\n')
            return True

        # Standard message generation for non-base types
        # Write documentation if available
        doc = datatype.get('documentation')
        if doc:
            # Handle multi-line documentation
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'// {line.strip()}\n')
        
        f.write(f'message {self._sanitize_ident(name)} {{\n')
        
        # Handle inheritance using composition
        generalizes = self._generalization_refs(datatype)
        field_num = 1
        
        if generalizes:
            # Add parent as first field (composition pattern)
            # But skip if parent is an empty abstract type (just a marker interface)
            base_count = 0
            seen_parents = set()
            for parent_name in generalizes:
                if parent_name in seen_parents:
                    continue  # skip duplicate generalizations
                seen_parents.add(parent_name)
                # Check if parent has any properties
                parent_has_properties = self._type_has_properties(parent_name)
                if parent_has_properties:
                    # Qualify parent type if from different package
                    qualified_parent = self._get_qualified_type(parent_name, current_package)
                    base_field = 'base' if base_count == 0 else f'base_{base_count + 1}'
                    base_count += 1
                    f.write(f'  {qualified_parent} {base_field} = {field_num};  // Inherited from {self._type_name(parent_name)}\n')
                    field_num += 1
            if datatype.get('properties') and field_num > 1:
                f.write('\n')  # Blank line between base and own fields only if base was written
        
        # Write own fields
        for prop in datatype.get('properties', []):
            type_name = self._emit_type_ref(self._prop_type_ref(prop))
            field_name = self._to_snake_case(prop['name'])

            # Skip interface-block fields with no data twin (pure service interfaces)
            if type_name is None and self._prop_type_ref(prop):
                continue

            # Check if property type is abstract - generate oneof (only when all
            # variants are in this data package, so a base/common package never
            # imports a component package and cycles).
            if type_name and self._is_abstract(type_name):
                concrete_children = self._find_concrete_children(type_name)
                if concrete_children and self._oneof_children_local(concrete_children, current_package):
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
        for parent_name in self._generalization_refs(cls):
            parent_class = self._find_class_by_name(parent_name)
            if parent_class:
                # Recursively collect from parent
                parent_props = self._collect_all_entity_properties(parent_class)
                all_entity_props.extend(parent_props)
        
        # Then add this class's entity properties
        for prop in cls.get('properties', []):
            type_name = self._prop_type_ref(prop)
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
            type_name = self._prop_type_ref(prop)
            if not (type_name and self._is_entity_derived(type_name)):
                # Found a non-entity property
                return True
        
        # Recursively check parents
        for parent_name in self._generalization_refs(cls):
            if self._class_has_non_entity_content(parent_name):
                return True
        
        return False
    
    def _find_class_by_name(self, class_name: str) -> Optional[Dict]:
        """Find a class definition by UUID/XMI id or, for legacy JSON, by name."""
        for cls in self.model.get('classes', []):
            if cls.get('id') == class_name or cls.get('name') == class_name:
                return cls
        return None
    
    def _write_message_from_class(self, f, cls: Dict[str, Any], current_package: str):
        """Write a message definition from a Class"""
        name = cls['name']
        service_type = cls.get('serviceType', 'unknown')

        # «InterfaceBlock» classes are port-type service interfaces, not data
        # messages. They are realised as gRPC services via component ports
        # (see _write_port_services), so do not emit a message for them.
        if cls.get('stereotype') == 'interfaceBlock':
            return False

        # Resolve each field's emit type (interface-block refs redirect to their
        # data twin; those with no twin are dropped, being pure service interfaces).
        fields = []
        for p in cls.get('properties', []):
            ref = self._prop_type_ref(p)
            if ref and self._is_interface_block(ref) and self._data_twin_ref(ref) is None:
                continue
            fields.append(p)

        # Skip empty message (no emittable fields and no content-bearing parent)
        has_parent = bool(self._generalization_refs(cls))
        if not fields and not has_parent:
            return False

        # Write documentation if available
        doc = cls.get('documentation')
        if doc:
            for line in doc.split('\n'):
                if line.strip():
                    f.write(f'// {line.strip()}\n')

        f.write(f'message {self._sanitize_ident(name)} {{\n')

        # Handle inheritance using composition
        generalizes = self._generalization_refs(cls)
        field_num = 1

        if generalizes:
            base_count = 0
            seen_parents = set()
            for parent_name in generalizes:
                if parent_name in seen_parents:
                    continue  # skip duplicate generalizations
                seen_parents.add(parent_name)
                if self._type_has_properties(parent_name):
                    qualified_parent = self._get_qualified_type(parent_name, current_package)
                    base_field = 'base' if base_count == 0 else f'base_{base_count + 1}'
                    base_count += 1
                    f.write(f'  {qualified_parent} {base_field} = {field_num};  // Inherited from {self._type_name(parent_name)}\n')
                    field_num += 1
            if fields and field_num > 1:
                f.write('\n')

        for prop in fields:
            type_name = self._emit_type_ref(self._prop_type_ref(prop))
            field_name = self._to_snake_case(prop['name'])

            # Abstract field -> oneof (only when all variants are in this package)
            if type_name and self._is_abstract(type_name):
                concrete_children = self._find_concrete_children(type_name)
                if concrete_children and self._oneof_children_local(concrete_children, current_package):
                    f.write(f'  oneof {field_name} {{\n')
                    for child in concrete_children:
                        child_qualified = self._get_qualified_type(child, current_package)
                        f.write(f'    {child_qualified} {self._to_snake_case(child)} = {field_num};\n')
                        field_num += 1
                    f.write(f'  }}\n')
                    continue

            field_type = self._get_qualified_type(type_name, current_package)

            prop_doc = prop.get('documentation')
            if prop_doc:
                for line in prop_doc.split('\n'):
                    if line.strip():
                        f.write(f'  // {line.strip()}\n')

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
        return True  # Message was written
    
    def _is_entity_derived(self, type_name: str) -> bool:
        """Check if a type derives from Entity (by checking all parsed types)"""
        # Search through all data types to see if this type derives from Entity
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if dt.get('id') == type_name or dt['name'] == self._type_name(type_name):
                    generalizes = self._generalization_refs(dt)
                    if any(self._type_name(g) == 'Entity' for g in generalizes):
                        return True
                    # Recursively check parent types
                    for parent in generalizes:
                        if self._is_entity_derived(parent):
                            return True
            # Also check classes
            for cls in namespace_types['classes']:
                if cls.get('id') == type_name or cls['name'] == self._type_name(type_name):
                    generalizes = self._generalization_refs(cls)
                    if any(self._type_name(g) == 'Entity' for g in generalizes):
                        return True
                    for parent in generalizes:
                        if self._is_entity_derived(parent):
                            return True
        return False
    
    def _write_grpc_service(self, f, service_name: str, entity_properties: List[Dict], current_package: str):
        """Generate gRPC service definition for entity-derived properties"""
        f.write(f'// gRPC Service for {service_name}\n')
        f.write(f'service {service_name}_Service {{\n')
        
        for prop in entity_properties:
            prop_name = prop['name']
            prop_type = prop['type']
            # Qualify the property type if from different package
            qualified_type = self._get_qualified_type(prop_type, current_package)
            flow_direction = prop.get('flow_direction', 'inout')
            
            # Get qualified Identifier type
            qualified_identifier = self._get_qualified_type('Identifier', current_package)
            qualified_query = self._get_qualified_type('Query', current_package)
            qualified_ack = self._get_qualified_type('Ack', current_package)            
            
            # CamelCase the property name for RPC method names
            prop_camel = ''.join(word.capitalize() for word in prop_name.split('_'))
            
            # Determine operations based on flow direction
            # inout = full CRUD
            # out = read only
            # in = write only (create, update, delete)
            
            # Generate operations based on flow direction and EntityActions semantics
            if flow_direction == 'inout' or flow_direction is None:
                # Full CRUD using EntityActions semantics
                f.write(f'  // Create {prop_name}\n')
                f.write(f'  rpc Create{prop_camel}({qualified_type}) returns ({qualified_identifier});\n')
                f.write(f'  \n')
                
                f.write(f'  // Read {prop_name}\n')
                f.write(f'  rpc Read{prop_camel}({qualified_query}) returns (stream {qualified_type});\n')
                f.write(f'  \n')
                
                f.write(f'  // Update {prop_name}\n')
                f.write(f'  rpc Update{prop_camel}({qualified_type}) returns ({qualified_ack});\n')
                f.write(f'  \n')
                
                f.write(f'  // Delete {prop_name}\n')
                f.write(f'  rpc Delete{prop_camel}({qualified_identifier}) returns ({qualified_ack});\n')
                
            elif flow_direction == 'out':
                # Read operations only
                f.write(f'  // Read {prop_name} (output only)\n')
                f.write(f'  rpc Read{prop_camel}({qualified_query}) returns (stream {qualified_type});\n')
                
            elif flow_direction == 'in':
                # Write operations only
                f.write(f'  // Create {prop_name} (input only)\n')
                f.write(f'  rpc Create{prop_camel}({qualified_type}) returns ({qualified_identifier});\n')
                f.write(f'  \n')
                
                f.write(f'  // Update {prop_name} (input only)\n')
                f.write(f'  rpc Update{prop_camel}({qualified_type}) returns ({qualified_ack});\n')
                f.write(f'  \n')
                
                f.write(f'  // Delete {prop_name} (input only)\n')
                f.write(f'  rpc Delete{prop_camel}({qualified_identifier}) returns ({qualified_ack});\n')
        
            
            # Add blank line between properties
            if entity_properties.index(prop) < len(entity_properties) - 1:
                f.write('\n')
        
        f.write('}\n')
    
    # ------------------------------------------------------------------
    # Port-driven service generation (RequestService / ProviderService model)
    # ------------------------------------------------------------------

    def _find_type_by_ref(self, ref: Optional[str]) -> Optional[Dict]:
        """Find a dataType or class by UUID/XMI id, falling back to name."""
        if not ref:
            return None
        name = self._type_name(ref)
        for namespace_types in self._group_by_namespace().values():
            for obj in namespace_types['dataTypes'] + namespace_types['classes']:
                if obj.get('id') == ref or obj.get('name') == name:
                    return obj
        return None

    def _is_interface_block(self, ref: Optional[str]) -> bool:
        """True if the referenced type is an «InterfaceBlock» (a service, not a message)."""
        obj = self._find_type_by_ref(ref)
        return bool(obj and obj.get('stereotype') == 'interfaceBlock')

    def _ref_package(self, ref: Optional[str]) -> Optional[str]:
        """The data_model package a type ref resolves to (by id, then name)."""
        if not ref:
            return None
        if ref in self.type_id_to_package:
            return self.type_id_to_package[ref]
        return self.type_to_package.get(self._type_name(ref))

    def _filter_variants_by_project(self, variants: List[str], current_package: str) -> List[str]:
        """Drop payload variants that belong to a *different* project than this service.

        Keeps common/generic variants (base, common, common_pim_components, generic_pim,
        pim, ...) and the current project's own variants; drops other projects' variants.
        """
        parts = current_package.split('.')
        current_project = parts[2] if len(parts) > 2 and parts[1] == 'components' else None
        if not current_project:
            return variants
        kept = []
        for v in variants:
            pkg = self._ref_package(v)
            vproj = None
            if pkg:
                pp = pkg.split('.')
                if len(pp) > 2 and pp[1] == 'data_model':
                    vproj = pp[2]
            if vproj and vproj in self._project_names and vproj != current_project:
                continue
            kept.append(v)
        return kept

    def _data_twin_ref(self, ref: Optional[str]) -> Optional[str]:
        """For an interface-block ref, return a same-named data (message) type ref, if any."""
        name = self._type_name(ref)
        if not name:
            return None
        for dt in self.model.get('dataTypes', []):
            if dt.get('name') == name:
                return dt.get('id') or name
        for cls in self.model.get('classes', []):
            if cls.get('name') == name and cls.get('stereotype') != 'interfaceBlock':
                return cls.get('id') or name
        return None

    def _emit_type_ref(self, ref: Optional[str]) -> Optional[str]:
        """Resolve a field's emit type: redirect interface-block refs to their data twin.

        Returns None when the field should be skipped (interface block with no
        data twin to fall back to).
        """
        if ref and self._is_interface_block(ref):
            return self._data_twin_ref(ref)
        return ref

    def _derives_from(self, ref: Optional[str], base_name: str, visited=None) -> bool:
        """Walk the generalization chain to test derivation from a named base type."""
        if visited is None:
            visited = set()
        name = self._type_name(ref)
        if not name or name in visited:
            return False
        visited.add(name)
        if name == base_name:
            return True
        obj = self._find_type_by_ref(ref)
        if not obj:
            return False
        for parent in obj.get('generalizes', []):
            if self._derives_from(parent, base_name, visited):
                return True
        return False

    def _concrete_descendants(self, ref: Optional[str], visited=None) -> List[str]:
        """All concrete (non-abstract) descendant type names of a type, transitively."""
        if visited is None:
            visited = set()
        name = self._type_name(ref)
        if not name or name in visited:
            return []
        visited.add(name)
        out = []
        for namespace_types in self._group_by_namespace().values():
            for obj in namespace_types['dataTypes'] + namespace_types['classes']:
                parent_names = [self._type_name(g) for g in obj.get('generalizes', [])]
                if name in parent_names:
                    child = obj['name']
                    if not obj.get('isAbstract', False) and child not in out:
                        out.append(child)
                    out.extend(self._concrete_descendants(child, visited))
        return out

    def _classify_iface_payloads(self, iface: Optional[Dict]) -> Dict[str, Optional[str]]:
        """Classify an interface block's bound properties into request/requirement/information.

        Support types (Ack/Query/Identifier) are ignored; anything that does not
        derive from Request/Requirement/Information falls into the information slot
        (provider-style payload).
        """
        res = {'request': None, 'requirement': None, 'information': None}
        if not iface:
            return res
        support = {'Ack', 'Query', 'Identifier'}
        for prop in iface.get('properties', []):
            ref = self._prop_type_ref(prop)
            if not ref:
                continue
            name = self._type_name(ref)
            if name in support:
                continue
            if self._derives_from(ref, 'Requirement') and not res['requirement']:
                res['requirement'] = ref
            elif self._derives_from(ref, 'Request') and not res['request']:
                res['request'] = ref
            elif self._derives_from(ref, 'Information') and not res['information']:
                res['information'] = ref
            elif not res['information']:
                res['information'] = ref
        return res

    def _port_label(self, name: str) -> str:
        """Strip a leading 'p' port-name convention (pCapability -> Capability)."""
        if len(name) > 1 and name[0] == 'p' and name[1].isupper():
            return name[1:]
        return name

    def _sanitize_ident(self, name: str) -> str:
        """Make a valid protobuf identifier from a (possibly spaced) name."""
        return ''.join(ch if (ch.isalnum() or ch == '_') else '_' for ch in name)

    def _payload_message(self, f, wrapper_name: str, payload_ref: Optional[str],
                         current_package: str) -> str:
        """Return the message type to use for a payload, writing a oneof wrapper if it has variants.

        If the payload type has multiple concrete variants (refined subclasses),
        emit a wrapper message with a `oneof` over them and return its name.
        Otherwise return the qualified payload type directly (or Empty if absent).
        """
        if not payload_ref:
            return 'google.protobuf.Empty'

        base_name = self._type_name(payload_ref)
        variants = []
        if not self._is_abstract(payload_ref):
            variants.append(base_name)
        for child in self._concrete_descendants(payload_ref):
            if child != base_name:
                variants.append(child)
        variants = list(dict.fromkeys(variants))  # de-dup, preserve order

        # Keep only common + this project's variants (drop other projects').
        variants = self._filter_variants_by_project(variants, current_package)

        if len(variants) <= 1:
            return self._get_qualified_type(payload_ref, current_package)

        # Service files reference data packages one-way, so expanding the oneof
        # across data packages is safe (no import cycle). A concrete component
        # binds its own bounded payload type, so this does not explode.
        f.write(f'message {wrapper_name} {{\n')
        f.write(f'  oneof payload {{\n')
        field_num = 1
        for variant in variants:
            q = self._get_qualified_type(variant, current_package)
            f.write(f'    {q} {self._to_snake_case(variant)} = {field_num};\n')
            field_num += 1
        f.write(f'  }}\n')
        f.write('}\n\n')
        return wrapper_name

    def _oneof_children_local(self, children: List[str], current_package: str) -> bool:
        """True if every oneof child resolves within the current (data) package.

        Used to gate abstract-field oneof expansion in data files so a base/common
        package never imports a component package (which would cycle).
        """
        return all('.' not in self._get_qualified_type(c, current_package) for c in children)

    def _write_one_port_service(self, f, port: Dict[str, Any], iface: Optional[Dict],
                                current_package: str, seen: set) -> bool:
        """Emit one port's service: request -> RequestService ops, information -> streaming read.

        The service is named after its port-type interface (e.g. `Matching_Objects_Service`).
        """
        kind = port.get('kind')
        if kind not in ('request', 'information') or iface is None:
            return False

        svc_name = f"{self._sanitize_ident(iface.get('name', 'Port'))}_Service"
        if svc_name in seen:
            n = 2
            while f'{svc_name}_{n}' in seen:
                n += 1
            svc_name = f'{svc_name}_{n}'
        seen.add(svc_name)

        payloads = self._classify_iface_payloads(iface)
        iface_name = iface.get('name', '?')
        direction = port.get('direction')

        if kind == 'request':
            ack = self._get_qualified_type('Ack', current_package)
            query = self._get_qualified_type('Query', current_package)
            identifier = self._get_qualified_type('Identifier', current_package)
            req_msg = self._payload_message(f, f'{svc_name}_Request',
                                            payloads.get('request'), current_package)
            reqmt_msg = self._payload_message(f, f'{svc_name}_Requirement',
                                              payloads.get('requirement'), current_package)
            f.write(f"// Request port '{port['name']}' ({direction}) refining {iface_name}\n")
            f.write(f'service {svc_name} {{\n')
            f.write(f'  rpc Create({req_msg}) returns ({ack});\n')
            f.write(f'  rpc Read({query}) returns (stream {reqmt_msg});\n')
            f.write(f'  rpc Update({reqmt_msg}) returns ({ack});\n')
            f.write(f'  rpc Cancel({identifier}) returns ({ack});\n')
            f.write('}\n')
            return True

        # information
        info_msg = self._payload_message(f, f'{svc_name}_Information',
                                         payloads.get('information'), current_package)
        f.write(f"// Information port '{port['name']}' ({direction}) refining {iface_name}\n")
        f.write(f'service {svc_name} {{\n')
        f.write(f'  rpc Read(google.protobuf.Empty) returns (stream {info_msg});\n')
        f.write('}\n')
        return True

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
        """Get fully qualified type name, resolving UUID/XMI ids before name fallback."""
        if not type_name:
            return 'int32'

        original_ref = type_name
        resolved_name = self._type_name(type_name)

        # Check if it's a mapped type
        if resolved_name in self.TYPE_MAPPING:
            mapped_type = self.TYPE_MAPPING[resolved_name]
            # If mapped to a custom type (not a proto primitive), need to qualify it
            if mapped_type in self.type_to_package:
                mapped_package = self.type_to_package[mapped_type]
                if mapped_package != current_package:
                    return f'{mapped_package}.{mapped_type}'
            return mapped_type

        safe_name = self._sanitize_ident(resolved_name)

        # UUID/XMI id lookup is authoritative
        if original_ref in self.type_id_to_package:
            type_package = self.type_id_to_package[original_ref]
            if type_package != current_package:
                return f'{type_package}.{safe_name}'
            return safe_name

        # Name lookup is retained for primitive hrefs/legacy JSON
        if resolved_name in self.type_to_package:
            type_package = self.type_to_package[resolved_name]
            if type_package != current_package:
                return f'{type_package}.{safe_name}'

        return safe_name

    def _is_abstract(self, type_name: str) -> bool:
        """Check if a type is marked as abstract"""
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if dt.get('id') == type_name or dt['name'] == self._type_name(type_name):
                    return dt.get('isAbstract', False)
            for cls in namespace_types['classes']:
                if cls.get('id') == type_name or cls['name'] == self._type_name(type_name):
                    return cls.get('isAbstract', False)
        return False
    
    def _find_concrete_children(self, abstract_type: str) -> List[str]:
        """Find all concrete types that inherit from an abstract type"""
        children = []
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if abstract_type in dt.get('generalizes', []) or abstract_type in dt.get('generalizesIds', []):
                    if not dt.get('isAbstract', False):
                        children.append(dt['name'])
            for cls in namespace_types['classes']:
                if abstract_type in cls.get('generalizes', []) or abstract_type in cls.get('generalizesIds', []):
                    if not cls.get('isAbstract', False):
                        children.append(cls['name'])
        return children
    
    def _type_has_non_entity_properties(self, type_name: str) -> bool:
        """Check if a type has any non-entity properties defined"""
        for namespace_types in self._group_by_namespace().values():
            for dt in namespace_types['dataTypes']:
                if dt.get('id') == type_name or dt['name'] == self._type_name(type_name):
                    # DataTypes always have non-entity properties
                    return len(dt.get('properties', [])) > 0
            for cls in namespace_types['classes']:
                if cls.get('id') == type_name or cls['name'] == self._type_name(type_name):
                    # Check if has any non-entity properties
                    for prop in cls.get('properties', []):
                        type_name_prop = self._prop_type_ref(prop)
                        if not (type_name_prop and self._is_entity_derived(type_name_prop)):
                            return True  # Has at least one non-entity property
                    return False
        return False
    
    def _type_has_properties(self, type_name: str, visited=None) -> bool:
        """True if the type emits a message worth referencing as a base field.

        DataTypes always emit a (possibly empty) message. Classes emit when they
        carry a non-interface field or inherit from a content-bearing parent.
        Interface blocks emit no message.
        """
        if visited is None:
            visited = set()
        name = self._type_name(type_name)
        if not name or name in visited:
            return False
        visited.add(name)
        obj = self._find_type_by_ref(type_name)
        if not obj or obj.get('stereotype') == 'interfaceBlock':
            return False
        # DataTypes always produce a message
        for dt in self.model.get('dataTypes', []):
            if dt.get('id') == obj.get('id'):
                return True
        for prop in obj.get('properties', []):
            tref = self._prop_type_ref(prop)
            if not (tref and self._is_interface_block(tref)):
                return True
        for parent in self._generalization_refs(obj):
            if self._type_has_properties(parent, visited):
                return True
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
    
    print(f"\n? Protobuf generation complete!")
    print(f"  Output directory: {output_dir}/")


if __name__ == '__main__':
    main()
