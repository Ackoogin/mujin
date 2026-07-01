#!/usr/bin/env python3
"""
SysML XMI Parser
Extracts logical data model from Cameo Systems Modeler XMI export
"""

import xml.etree.ElementTree as ET
import json
import re
from typing import Dict, List, Any, Optional
from pathlib import Path


class SysMLParser:
    """Parser for SysML data models exported as XMI"""
    
    def __init__(self, xmi_file: str):
        self.tree = ET.parse(xmi_file)
        self.root = self.tree.getroot()
        self.types = {}
        self.enumerations = {}
        self.search_root = None  # Will be set during parse
        
    def _get_package_path(self, element: ET.Element) -> List[str]:
        """Get the package hierarchy path for an element relative to search root"""
        path = []
        current = element
        
        # Walk up the tree to build the path
        while current is not None:
            # Stop if we've reached the search root
            if self.search_root is not None and current == self.search_root:
                break
                
            parent = self._get_parent(current)
            if parent is not None:
                # Check if parent is a Package
                xmi_type = self._get_xmi_attr(parent, 'type')
                if xmi_type == 'uml:Package' or parent.tag.endswith('Package'):
                    pkg_name = parent.get('name')
                    if pkg_name:
                        path.insert(0, pkg_name)
            current = parent
        
        return path
    
    def _get_parent(self, element: ET.Element) -> Optional[ET.Element]:
        """Get the parent element using tree traversal"""
        for parent in self.root.iter():
            for child in parent:
                if child == element:
                    return parent
        return None
    
    def _clean_documentation(self, html_text: str) -> str:
        """Clean HTML documentation text to plain text, preserving line breaks"""
        if not html_text:
            return None
        
        # Replace <br> tags with newlines before removing HTML
        text = html_text.replace('<br>', '\n').replace('<br/>', '\n').replace('<br />', '\n')
        
        # Replace <p> tags with newlines (paragraph breaks)
        text = re.sub(r'</p>\s*<p>', '\n', text)
        
        # Remove HTML tags
        text = re.sub('<[^<]+?>', '', text)
        
        # Decode HTML entities
        import html as html_module
        text = html_module.unescape(text)
        text = text.replace('&#160;', ' ')
        text = text.replace('&nbsp;', ' ')
        
        # Remove CSS/style content
        text = re.sub(r'p\s*{[^}]*}', '', text)
        text = re.sub(r'\w+\s*:\s*\w+\s*;', '', text)
        
        # Clean up excessive whitespace on each line, but preserve line breaks
        lines = []
        for line in text.split('\n'):
            # Strip leading/trailing whitespace from each line
            line = line.strip()
            # Collapse multiple spaces within a line
            line = re.sub(r'[ \t]+', ' ', line)
            lines.append(line)
        
        # Remove completely empty lines but keep lines with content
        lines = [line for line in lines if line]
        
        # Join with newlines to preserve line structure
        text = '\n'.join(lines)
        
        return text if text else None
        self.search_root = None  # Track the search root for relative paths
        
    def _get_package_path(self, element: ET.Element) -> List[str]:
        """Get the package hierarchy path for an element, relative to search root"""
        path = []
        current = element
        
        # Walk up the tree until we hit the search root or root
        while current is not None:
            parent = self._get_parent(current)
            if parent is None:
                break
            
            # Stop if we've reached the search root
            if self.search_root is not None and parent == self.search_root:
                break
                
            # Check if parent is a Package
            xmi_type = self._get_xmi_attr(parent, 'type')
            if xmi_type == 'uml:Package' or parent.tag.endswith('Package'):
                pkg_name = parent.get('name')
                if pkg_name:
                    path.insert(0, pkg_name)
            
            current = parent
        
        return path
    
    def _get_parent(self, element: ET.Element) -> Optional[ET.Element]:
        """Get the parent element of an element"""
        # Build parent map if needed
        if not hasattr(self, '_parent_map'):
            self._parent_map = {c: p for p in self.root.iter() for c in p}
        return self._parent_map.get(element)
        
    def _get_xmi_attr(self, element: ET.Element, attr_name: str) -> Optional[str]:
        """Get XMI attribute handling both with and without namespace prefix"""
        # Try with common XMI namespaces
        for ns in ['{http://www.omg.org/spec/XMI/20131001}', 
                   '{http://www.omg.org/XMI}', '']:
            val = element.get(f'{ns}{attr_name}')
            if val:
                return val
        return None
    
    def _find_package(self, package_name: str) -> Optional[ET.Element]:
        """Find a package by name in the model"""
        for elem in self.root.iter():
            # Check if it's a Package element
            xmi_type = self._get_xmi_attr(elem, 'type')
            if xmi_type == 'uml:Package' or elem.tag.endswith('Package'):
                name = elem.get('name')
                if name == package_name:
                    return elem
        return None
        
    def parse(self, package_filter: str = None) -> Dict[str, Any]:
        """Parse the XMI and extract data model
        
        Args:
            package_filter: Optional package name to restrict parsing to (e.g., 'PIM')
        """
        result = {
            'dataTypes': [],
            'enumerations': [],
            'classes': [],
            'connectors': [],
            'refinements': []
        }

        # Determine search root
        self.search_root = self.root
        if package_filter:
            self.search_root = self._find_package(package_filter)
            if self.search_root is None:
                print(f"Warning: Package '{package_filter}' not found, searching entire model")
                self.search_root = self.root
            else:
                print(f"[x] Found package '{package_filter}', restricting search")

        # Map class id -> stereotype ('block' | 'interfaceBlock'). Stereotype
        # applications (e.g. <sysml:Block base_Class=...>) live outside the
        # search root, so scan the whole document for them.
        self._build_stereotype_map()

        # Find all elements with xmi:type attribute
        # Search tree for specific SysML types only
        for elem in self.search_root.iter():
            xmi_type = self._get_xmi_attr(elem, 'type')
            
            # Process ValueTypes (all DataTypes are treated as ValueTypes)
            if xmi_type == 'uml:DataType':
                dt_info = self._parse_datatype(elem)
                if dt_info:
                    result['dataTypes'].append(dt_info)
                    
            # Process Enumerations
            elif xmi_type == 'uml:Enumeration':
                enum_info = self._parse_enumeration(elem)
                if enum_info:
                    result['enumerations'].append(enum_info)
                    
            # Process Classes (service interfaces)
            elif xmi_type == 'uml:Class':
                class_info = self._parse_class(elem)
                if class_info:
                    result['classes'].append(class_info)

            # Process Connectors (port-to-port wiring / bridges)
            elif xmi_type == 'uml:Connector':
                conn_info = self._parse_connector(elem)
                if conn_info:
                    result['connectors'].append(conn_info)

            # Process «Refine» / Abstraction relationships (client refines supplier).
            # The model expresses interface refinement (e.g. a specific requirement
            # interface -> Requirement -> RequestService) partly via these.
            elif xmi_type == 'uml:Abstraction':
                for refine in self._parse_refinement(elem):
                    result['refinements'].append(refine)

        return result

    def _parse_refinement(self, element: ET.Element) -> List[Dict[str, Any]]:
        """Parse a uml:Abstraction into client->supplier refinement edges.

        client/supplier may appear as space-separated attributes or as
        <client xmi:idref=.../> / <supplier xmi:idref=.../> child elements.
        """
        clients, suppliers = [], []
        cl = self._get_xmi_attr(element, 'client') or element.get('client')
        sup = self._get_xmi_attr(element, 'supplier') or element.get('supplier')
        if cl:
            clients += cl.split()
        if sup:
            suppliers += sup.split()
        for child in element:
            tag = child.tag.rsplit('}', 1)[-1]
            ref = child.get('{http://www.omg.org/spec/XMI/20131001}idref') or child.get('idref')
            if not ref:
                continue
            if tag == 'client':
                clients.append(ref)
            elif tag == 'supplier':
                suppliers.append(ref)
        return [{'client': c, 'supplier': s} for c in clients for s in suppliers if c and s]

    def _build_stereotype_map(self):
        """Build a map of class id -> stereotype name ('block' | 'interfaceBlock').

        SysML «Block»/«InterfaceBlock» stereotypes are applied via elements such
        as <sysml:Block base_Class='<class-id>'>. The tag-local name carries the
        stereotype; the base_Class attribute points at the stereotyped class.
        """
        self.stereotype_by_class = {}
        for elem in self.root.iter():
            tag = elem.tag.rsplit('}', 1)[-1]  # strip namespace
            if tag == 'InterfaceBlock':
                kind = 'interfaceBlock'
            elif tag == 'Block':
                kind = 'block'
            else:
                continue
            base_class = elem.get('base_Class')
            if base_class:
                self.stereotype_by_class[base_class] = kind
    
    
    def _parse_datatype(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML DataType element"""
        dt_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        is_abstract = element.get('isAbstract', 'false') == 'true'
        
        if not name:
            return None
        
        # Get package hierarchy
        namespace = self._get_package_path(element)
        
        # Parse properties/attributes
        properties = []
        for child in element:
            # Check if it's a property
            xmi_type = self._get_xmi_attr(child, 'type')
            if xmi_type == 'uml:Property':
                prop_info = self._parse_property(child)
                if prop_info:
                    properties.append(prop_info)
        
        # Parse generalizations (inheritance)
        generalizations = []
        for child in element:
            if child.tag.endswith('generalization'):
                general_id = child.get('general')
                if general_id and general_id not in generalizations:
                    generalizations.append(general_id)
        
        # Parse comments/documentation
        comment = None
        for child in element:
            if child.tag.endswith('ownedComment') or self._get_xmi_attr(child, 'type') == 'uml:Comment':
                body = child.get('body', '')
                # Strip HTML tags and clean up whitespace
                comment = self._clean_documentation(body)
                break
        
        return {
            'id': dt_id,
            'name': name,
            'namespace': namespace,
            'isAbstract': is_abstract,
            'properties': properties,
            'generalizes': generalizations,
            'documentation': comment
        }
    
    def _parse_property(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML Property (attribute)"""
        prop_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        type_ref = element.get('type')
        visibility = element.get('visibility', 'public')
        
        if not name:
            return None
        
        # Check for type reference via href (for primitive types)
        type_name = None
        if not type_ref:
            # Look for <type href=...> child element
            for child in element:
                if child.tag.endswith('type'):
                    href = child.get('href', '')
                    if 'Boolean' in href:
                        type_name = 'Boolean'
                    elif 'String' in href:
                        type_name = 'String'
                    elif 'Integer' in href:
                        type_name = 'Integer'
                    elif 'Real' in href:
                        type_name = 'Real'
                    break
        
        # Parse multiplicity - need to look for lowerValue and upperValue
        lower = '1'  # default
        upper = '1'  # default
        
        # Search all descendants for multiplicity elements
        for child in element.iter():
            if child.tag.endswith('lowerValue'):
                lower = child.get('value', '0')
            elif child.tag.endswith('upperValue'):
                upper_val = child.get('value', '1')
                upper = '*' if upper_val == '*' else upper_val
        
        # Parse flow direction from SysML FlowProperty
        # Search for FlowProperty that references this property
        flow_direction = None
        for elem in self.root.iter():
            if 'FlowProperty' in elem.tag:
                base_prop = elem.get('base_Property')
                if base_prop == prop_id:
                    flow_direction = elem.get('direction', 'inout')
                    break
        
        # Parse documentation for this property
        prop_doc = None
        for child in element:
            if child.tag.endswith('ownedComment') or self._get_xmi_attr(child, 'type') == 'uml:Comment':
                body = child.get('body', '')
                prop_doc = self._clean_documentation(body)
                break
        
        return {
            'id': prop_id,
            'name': name,
            'type': type_ref,
            'typeName': type_name,  # Set from href if found
            'visibility': visibility,
            'flow_direction': flow_direction,
            'documentation': prop_doc,
            'multiplicity': {
                'lower': lower,
                'upper': upper
            }
        }
    
    def _parse_enumeration(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML Enumeration"""
        enum_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        
        if not name:
            return None
        
        # Get package hierarchy
        namespace = self._get_package_path(element)
        
        # Parse literals
        literals = []
        for child in element:
            if child.tag.endswith('ownedLiteral'):
                lit_name = child.get('name')
                lit_id = self._get_xmi_attr(child, 'id')
                if lit_name:
                    # Parse documentation for this literal
                    lit_doc = None
                    for lit_child in child:
                        if lit_child.tag.endswith('ownedComment') or self._get_xmi_attr(lit_child, 'type') == 'uml:Comment':
                            body = lit_child.get('body', '')
                            lit_doc = self._clean_documentation(body)
                            break
                    
                    literals.append({
                        'id': lit_id,
                        'name': lit_name,
                        'documentation': lit_doc
                    })
        
        return {
            'id': enum_id,
            'name': name,
            'namespace': namespace,
            'literals': literals
        }
    
    def _parse_class(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML Class (service definition)"""
        class_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        is_abstract = element.get('isAbstract', 'false') == 'true'
        
        if not name:
            return None
        
        # Get package hierarchy
        namespace = self._get_package_path(element)
        
        # Parse properties/attributes and ports
        properties = []
        ports = []
        for child in element:
            xmi_type = self._get_xmi_attr(child, 'type')
            if xmi_type == 'uml:Property':
                prop_info = self._parse_property(child)
                if prop_info:
                    properties.append(prop_info)
            elif xmi_type == 'uml:Port':
                port_info = self._parse_port(child)
                if port_info:
                    ports.append(port_info)

        # Parse operations
        operations = []
        for child in element:
            xmi_type = self._get_xmi_attr(child, 'type')
            if xmi_type == 'uml:Operation':
                op_info = self._parse_operation(child)
                if op_info:
                    operations.append(op_info)
        
        # Parse generalizations (inheritance)
        generalizations = []
        for child in element:
            if child.tag.endswith('generalization'):
                general_id = child.get('general')
                if general_id and general_id not in generalizations:
                    generalizations.append(general_id)
        
        # Parse comments/documentation
        comment = None
        for child in element:
            if child.tag.endswith('ownedComment') or self._get_xmi_attr(child, 'type') == 'uml:Comment':
                body = child.get('body', '')
                comment = re.sub('<[^<]+?>', '', body).strip()
                break
        
        return {
            'id': class_id,
            'name': name,
            'namespace': namespace,
            'isAbstract': is_abstract,
            'stereotype': self.stereotype_by_class.get(class_id),
            'properties': properties,
            'ports': ports,
            'operations': operations,
            'generalizes': generalizations,
            'documentation': comment
        }

    def _parse_port(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML Port (ownedAttribute with xmi:type='uml:Port').

        A port is typed by an «InterfaceBlock» service interface. isConjugated
        flags a required/consumed port (vs a provided one). Multiplicity may be
        nested under xmi:Extension/modelExtension, so scan all descendants.
        """
        port_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        type_ref = element.get('type')

        if not name:
            return None

        is_conjugated = element.get('isConjugated', 'false') == 'true'
        aggregation = element.get('aggregation')

        # Parse multiplicity (same descendant scan as _parse_property)
        lower = '1'
        upper = '1'
        for child in element.iter():
            if child.tag.endswith('lowerValue'):
                lower = child.get('value', '0')
            elif child.tag.endswith('upperValue'):
                upper_val = child.get('value', '1')
                upper = '*' if upper_val == '*' else upper_val

        return {
            'id': port_id,
            'name': name,
            'typeId': type_ref,
            'typeName': None,  # resolved in resolve_references
            'isConjugated': is_conjugated,
            'aggregation': aggregation,
            'kind': None,       # resolved in resolve_references
            'direction': 'consumed' if is_conjugated else 'provided',
            'multiplicity': {
                'lower': lower,
                'upper': upper
            }
        }

    def _parse_connector(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML Connector (port-to-port wiring / bridge).

        Each connector has two ConnectorEnd children carrying partWithPort (the
        participating part) and role (the port id on that part). The owner is the
        nearest ancestor uml:Class.
        """
        conn_id = self._get_xmi_attr(element, 'id')

        ends = []
        for child in element:
            if self._get_xmi_attr(child, 'type') == 'uml:ConnectorEnd':
                ends.append({
                    'partWithPort': child.get('partWithPort'),
                    'role': child.get('role')
                })

        if not ends:
            return None

        # Find owning class by walking up the parent chain
        owner_id = None
        owner_name = None
        current = self._get_parent(element)
        while current is not None:
            if self._get_xmi_attr(current, 'type') == 'uml:Class':
                owner_id = self._get_xmi_attr(current, 'id')
                owner_name = current.get('name')
                break
            current = self._get_parent(current)

        return {
            'id': conn_id,
            'ownerId': owner_id,
            'ownerName': owner_name,
            'ends': ends
        }

    def _parse_operation(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML Operation (method/service operation)"""
        op_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        visibility = element.get('visibility', 'public')
        
        if not name:
            return None
        
        # Parse parameters
        parameters = []
        return_type = None
        
        for child in element:
            xmi_type = self._get_xmi_attr(child, 'type')
            if xmi_type == 'uml:Parameter':
                param_info = self._parse_parameter(child)
                if param_info:
                    if param_info.get('direction') == 'return':
                        return_type = param_info.get('type')
                    else:
                        parameters.append(param_info)
        
        return {
            'id': op_id,
            'name': name,
            'visibility': visibility,
            'parameters': parameters,
            'returnType': return_type
        }
    
    def _parse_parameter(self, element: ET.Element) -> Dict[str, Any]:
        """Parse a UML Parameter (operation parameter)"""
        param_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        type_ref = element.get('type')
        direction = element.get('direction', 'in')
        visibility = element.get('visibility', 'public')
        
        # Parse multiplicity
        lower = '1'
        upper = '1'
        
        for child in element.iter():
            if child.tag.endswith('lowerValue'):
                lower = child.get('value', '0')
            elif child.tag.endswith('upperValue'):
                upper_val = child.get('value', '1')
                upper = '*' if upper_val == '*' else upper_val
        
        return {
            'id': param_id,
            'name': name,
            'type': type_ref,
            'direction': direction,
            'visibility': visibility,
            'multiplicity': {
                'lower': lower,
                'upper': upper
            }
        }
    
    def _resolve_port_kind(self, type_id, class_by_id, type_lookup, visited, refine_map=None):
        """Classify a port by walking its type's generalization AND «Refine» chain.

        Returns 'request' if it reaches RequestService, 'information' if it
        reaches ProviderService, else None. Follows both id-based generalizations
        (client) and Refine edges (client -> supplier); must run before
        generalizations are rewritten to names.
        """
        if not type_id or type_id in visited:
            return None
        visited.add(type_id)

        name = type_lookup.get(type_id)
        if name == 'RequestService':
            return 'request'
        if name == 'ProviderService':
            return 'information'

        neighbours = list((refine_map or {}).get(type_id, []))
        cls = class_by_id.get(type_id)
        if cls:
            neighbours += cls.get('generalizes', [])
        for nxt in neighbours:
            kind = self._resolve_port_kind(nxt, class_by_id, type_lookup, visited, refine_map)
            if kind:
                return kind
        return None

    def resolve_references(self, model: Dict[str, Any]) -> Dict[str, Any]:
        """Resolve type references to actual type names"""
        # Build lookup tables
        type_lookup = {}
        
        for dt in model['dataTypes']:
            if dt['id']:
                type_lookup[dt['id']] = dt['name']
        
        for enum in model['enumerations']:
            if enum['id']:
                type_lookup[enum['id']] = enum['name']
                
        for cls in model['classes']:
            if cls['id']:
                type_lookup[cls['id']] = cls['name']

        # Resolve ports and connectors before generalization ids are rewritten
        # to names below (port-kind resolution walks id-based generalizations).
        class_by_id = {cls['id']: cls for cls in model['classes'] if cls.get('id')}
        port_by_id = {}

        # Build the «Refine» adjacency (client -> [suppliers]) and attach a
        # resolved `refines` list to each client class.
        refine_map = {}
        for ref in model.get('refinements', []):
            client, supplier = ref.get('client'), ref.get('supplier')
            if client and supplier:
                refine_map.setdefault(client, []).append(supplier)
        for cls in model['classes']:
            suppliers = refine_map.get(cls['id'], [])
            if suppliers:
                cls['refines'] = list(suppliers)
                cls['refinesNames'] = [type_lookup.get(s, s) for s in suppliers]

        for cls in model['classes']:
            for port in cls.get('ports', []):
                type_id = port.get('typeId')
                if type_id and type_id in type_lookup:
                    port['typeName'] = type_lookup[type_id]
                else:
                    port['typeName'] = type_id
                port['kind'] = self._resolve_port_kind(type_id, class_by_id, type_lookup, set(), refine_map)
                if port.get('id'):
                    port_by_id[port['id']] = port['name']

        for conn in model.get('connectors', []):
            if conn.get('ownerId') and not conn.get('ownerName'):
                conn['ownerName'] = type_lookup.get(conn['ownerId'])
            for end in conn.get('ends', []):
                # role references a port id; partWithPort references a part property
                end['roleName'] = port_by_id.get(end.get('role'))

        # Resolve property type references in DataTypes
        for dt in model['dataTypes']:
            for prop in dt['properties']:
                # If typeName already set from href, keep it; otherwise resolve from type_lookup
                if not prop.get('typeName'):
                    if prop['type'] and prop['type'] in type_lookup:
                        prop['typeName'] = type_lookup[prop['type']]
                    else:
                        prop['typeName'] = prop['type']
            
            # Resolve generalization references (retain original ids so the
            # generator can disambiguate same-named types by UUID).
            dt['generalizesIds'] = list(dt['generalizes'])
            resolved_generals = []
            for gen_id in dt['generalizes']:
                if gen_id in type_lookup:
                    resolved_generals.append(type_lookup[gen_id])
                else:
                    resolved_generals.append(gen_id)
            dt['generalizes'] = resolved_generals
        
        # Resolve type references in Classes
        for cls in model['classes']:
            # Resolve property types
            for prop in cls['properties']:
                # If typeName already set from href, keep it; otherwise resolve from type_lookup
                if not prop.get('typeName'):
                    if prop['type'] and prop['type'] in type_lookup:
                        prop['typeName'] = type_lookup[prop['type']]
                    else:
                        prop['typeName'] = prop['type']
            
            # Resolve operation parameter and return types
            for op in cls['operations']:
                if op['returnType'] and op['returnType'] in type_lookup:
                    op['returnTypeName'] = type_lookup[op['returnType']]
                else:
                    op['returnTypeName'] = op['returnType']
                    
                for param in op['parameters']:
                    if param['type'] and param['type'] in type_lookup:
                        param['typeName'] = type_lookup[param['type']]
                    else:
                        param['typeName'] = param['type']
            
            # Resolve generalization references (retain original ids so the
            # generator can disambiguate same-named types by UUID).
            cls['generalizesIds'] = list(cls['generalizes'])
            resolved_generals = []
            for gen_id in cls['generalizes']:
                if gen_id in type_lookup:
                    resolved_generals.append(type_lookup[gen_id])
                else:
                    resolved_generals.append(gen_id)
            cls['generalizes'] = resolved_generals
        
        return model


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python sysml_parser.py <xmi_file> [output_json] [--package PACKAGE_NAME]")
        print("\nOptions:")
        print("  --package PACKAGE_NAME    Restrict parsing to children of specified package (e.g., 'PIM')")
        sys.exit(1)
    
    xmi_file = sys.argv[1]
    
    # Parse arguments
    output_file = 'model.json'
    package_filter = None
    
    i = 2
    while i < len(sys.argv):
        if sys.argv[i] == '--package' and i + 1 < len(sys.argv):
            package_filter = sys.argv[i + 1]
            i += 2
        else:
            output_file = sys.argv[i]
            i += 1
    
    print(f"Parsing {xmi_file}...")
    if package_filter:
        print(f"Filtering to package: {package_filter}")
    
    parser = SysMLParser(xmi_file)
    model = parser.parse(package_filter=package_filter)
    model = parser.resolve_references(model)
    
    # Write to JSON
    with open(output_file, 'w') as f:
        json.dump(model, f, indent=2)
    
    print(f"[x] Parsed {len(model['dataTypes'])} data types")
    print(f"[x] Parsed {len(model['enumerations'])} enumerations")
    print(f"[x] Parsed {len(model['classes'])} classes/services")
    print(f"[x] Output written to {output_file}")


if __name__ == '__main__':
    main()
