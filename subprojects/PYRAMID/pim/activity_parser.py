#!/usr/bin/env python3
"""
Activity Behavior Parser for SysML/UML Activities
Extracts component behaviors, service interactions, and call sequences
"""

import xml.etree.ElementTree as ET
import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re


class ActivityParser:
    """Parse UML Activities to extract service interaction patterns"""
    
    def __init__(self, xmi_file: str):
        self.tree = ET.parse(xmi_file)
        self.root = self.tree.getroot()
        self.namespaces = self._extract_namespaces()
        
        # Build lookup tables
        self.elements_by_id = {}
        self.behaviors_by_id = {}
        self._build_lookup_tables()
        
        # Package filtering support
        self.search_root = None  # Track the search root for relative paths
        self._parent_map = None  # Lazy-initialized parent map
    
    def _extract_namespaces(self) -> Dict[str, str]:
        """Extract XML namespaces from root element"""
        namespaces = {}
        for key, value in self.root.attrib.items():
            if key.startswith('{http://www.w3.org/2000/xmlns/}'):
                prefix = key.split('}')[1] if '}' in key else ''
                namespaces[prefix] = value
        return namespaces
    
    def _get_xmi_attr(self, element: ET.Element, attr_name: str) -> Optional[str]:
        """Get XMI attribute handling namespace variations"""
        # Try common XMI namespace variations
        for ns in ['{http://www.omg.org/spec/XMI/20131001}',
                   '{http://www.omg.org/XMI}', '']:
            key = f'{ns}{attr_name}'
            if key in element.attrib:
                return element.attrib[key]
        return None
    
    def _get_parent(self, element: ET.Element) -> Optional[ET.Element]:
        """Get the parent element of an element"""
        # Build parent map if needed (lazy initialization)
        if self._parent_map is None:
            self._parent_map = {c: p for p in self.root.iter() for c in p}
        return self._parent_map.get(element)
    
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
    
    def _is_within_search_root(self, element: ET.Element) -> bool:
        """Check if element is within the search root scope"""
        if self.search_root is None:
            return True  # No filter, include everything
        
        # Walk up the tree to see if we hit search_root
        current = element
        while current is not None:
            if current == self.search_root:
                return True
            current = self._get_parent(current)
        
        return False
    
    def _build_lookup_tables(self):
        """Build lookup tables for quick element access"""
        for elem in self.root.iter():
            elem_id = self._get_xmi_attr(elem, 'id')
            if elem_id:
                self.elements_by_id[elem_id] = elem
                
                # Track behaviors separately
                xmi_type = self._get_xmi_attr(elem, 'type')
                if xmi_type == 'uml:Activity':
                    self.behaviors_by_id[elem_id] = elem
    
    def parse(self, package_filter: str = None) -> Dict[str, Any]:
        """Parse all activities and extract behavior patterns"""
        result = {
            'activities': [],
            'metadata': {
                'parser_version': '1.0',
                'total_activities': 0,
                'package_filter': package_filter
            }
        }
        
        # Set search root if package filter is specified
        if package_filter:
            self.search_root = self._find_package(package_filter)
            if self.search_root is None:
                print(f"Warning: Package '{package_filter}' not found")
                return result
            else:
                print(f"[x] Found package '{package_filter}', restricting search")
        
        # Find all Activity elements within scope
        for elem in self.root.iter():
            xmi_type = self._get_xmi_attr(elem, 'type')
            if xmi_type == 'uml:Activity':
                # Check if within search scope
                if not self._is_within_search_root(elem):
                    continue
                
                activity_info = self._parse_activity(elem)
                if activity_info:
                    result['activities'].append(activity_info)
        
        result['metadata']['total_activities'] = len(result['activities'])
        return result
    
    def _parse_activity(self, element: ET.Element) -> Optional[Dict[str, Any]]:
        """Parse a single UML Activity"""
        activity_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        
        if not name:
            return None
        
        # Get package hierarchy
        namespace = self._get_package_path(element)
        
        activity = {
            'id': activity_id,
            'name': name,
            'namespace': namespace,
            'type': 'Activity',
            'parameters': [],
            'nodes': [],
            'edges': [],
            'dataFlow': [],
            'controlFlow': []
        }
        
        # Parse owned parameters (inputs/outputs)
        for child in element:
            if child.tag.endswith('ownedParameter'):
                param = self._parse_parameter(child)
                if param:
                    activity['parameters'].append(param)
            
            # Parse nodes (actions, parameter nodes, etc.)
            elif child.tag.endswith('node'):
                node = self._parse_node(child)
                if node:
                    activity['nodes'].append(node)
            
            # Parse edges (control flow, object flow)
            elif child.tag.endswith('edge'):
                edge = self._parse_edge(child)
                if edge:
                    activity['edges'].append(edge)
                    if edge['type'] == 'ObjectFlow':
                        activity['dataFlow'].append(edge)
                    elif edge['type'] == 'ControlFlow':
                        activity['controlFlow'].append(edge)
        
        # Analyze interaction patterns
        activity['interactionPattern'] = self._analyze_interactions(activity)
        
        return activity
    
    def _parse_parameter(self, element: ET.Element) -> Optional[Dict[str, Any]]:
        """Parse an Activity parameter"""
        param_id = self._get_xmi_attr(element, 'id')
        name = element.get('name')
        param_type = element.get('type')
        direction = element.get('direction', 'in')
        
        return {
            'id': param_id,
            'name': name,
            'type': param_type,
            'direction': direction,
            'typeName': self._resolve_type_name(param_type)
        }
    
    def _parse_node(self, element: ET.Element) -> Optional[Dict[str, Any]]:
        """Parse an Activity node (action, parameter node, etc.)"""
        node_id = self._get_xmi_attr(element, 'id')
        xmi_type = self._get_xmi_attr(element, 'type')
        name = element.get('name', '')
        
        node = {
            'id': node_id,
            'type': xmi_type,
            'name': name
        }
        
        # Extract type-specific information
        if xmi_type == 'uml:CallBehaviorAction':
            node['behavior'] = element.get('behavior')
            node['behaviorName'] = self._resolve_behavior_name(element.get('behavior'))
            
            # Parse input/output pins
            node['inputs'] = []
            node['outputs'] = []
            
            for child in element:
                if child.tag.endswith('argument'):
                    pin = self._parse_pin(child)
                    if pin:
                        node['inputs'].append(pin)
                elif child.tag.endswith('result'):
                    pin = self._parse_pin(child)
                    if pin:
                        node['outputs'].append(pin)
        
        elif xmi_type == 'uml:ActivityParameterNode':
            node['parameter'] = element.get('parameter')
            node['nodeType'] = element.get('type')
        
        return node
    
    def _parse_pin(self, element: ET.Element) -> Optional[Dict[str, Any]]:
        """Parse an input/output pin"""
        pin_id = self._get_xmi_attr(element, 'id')
        xmi_type = self._get_xmi_attr(element, 'type')
        name = element.get('name', '')
        pin_type = element.get('type')
        
        return {
            'id': pin_id,
            'type': xmi_type,
            'name': name,
            'dataType': pin_type,
            'typeName': self._resolve_type_name(pin_type)
        }
    
    def _parse_edge(self, element: ET.Element) -> Optional[Dict[str, Any]]:
        """Parse a control or object flow edge"""
        edge_id = self._get_xmi_attr(element, 'id')
        xmi_type = self._get_xmi_attr(element, 'type')
        source = element.get('source')
        target = element.get('target')
        
        return {
            'id': edge_id,
            'type': xmi_type,
            'source': source,
            'target': target,
            'sourceName': self._resolve_element_name(source),
            'targetName': self._resolve_element_name(target)
        }
    
    def _analyze_interactions(self, activity: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze the activity to extract interaction patterns"""
        pattern = {
            'providedService': None,
            'consumedServices': [],
            'callSequence': [],
            'dataFlowMappings': []
        }
        
        # Identify provided service (input parameters)
        input_params = [p for p in activity['parameters'] if p['direction'] == 'in']
        output_params = [p for p in activity['parameters'] if p['direction'] in ['out', 'return']]
        
        if input_params or output_params:
            pattern['providedService'] = {
                'activityName': activity['name'],
                'inputs': input_params,
                'outputs': output_params
            }
        
        # Identify consumed services (CallBehaviorActions)
        call_actions = [n for n in activity['nodes'] if n['type'] == 'uml:CallBehaviorAction']
        
        for idx, action in enumerate(call_actions, 1):
            consumed = {
                'sequence': idx,
                'actionId': action['id'],
                'actionName': action.get('name', f"Action_{idx}"),
                'behavior': action.get('behaviorName', 'Unknown'),
                'inputs': action.get('inputs', []),
                'outputs': action.get('outputs', [])
            }
            pattern['consumedServices'].append(consumed)
        
        # Build call sequence from control flow
        if activity['controlFlow']:
            pattern['callSequence'] = self._build_call_sequence(
                activity['controlFlow'],
                call_actions
            )
        
        # Analyze data flow mappings
        if activity['dataFlow']:
            pattern['dataFlowMappings'] = self._analyze_data_flow(
                activity['dataFlow'],
                activity['nodes']
            )
        
        return pattern
    
    def _build_call_sequence(self, control_flows: List[Dict], actions: List[Dict]) -> List[Dict]:
        """Build the sequence of service calls from control flow"""
        sequence = []
        
        # Build adjacency map
        next_action = {}
        for flow in control_flows:
            next_action[flow['source']] = flow['target']
        
        # Find starting action (no incoming control flow)
        action_ids = {a['id'] for a in actions}
        targets = {f['target'] for f in control_flows}
        starts = action_ids - targets
        
        # Build sequence
        visited = set()
        for start in starts:
            current = start
            step = 1
            while current and current not in visited:
                visited.add(current)
                action = next((a for a in actions if a['id'] == current), None)
                if action:
                    sequence.append({
                        'step': step,
                        'actionId': current,
                        'actionName': action.get('name', ''),
                        'behavior': action.get('behaviorName', '')
                    })
                    step += 1
                current = next_action.get(current)
        
        return sequence
    
    def _analyze_data_flow(self, data_flows: List[Dict], nodes: List[Dict]) -> List[Dict]:
        """Analyze data flow between nodes"""
        mappings = []
        
        for flow in data_flows:
            source_node = next((n for n in nodes if n['id'] == flow['source']), None)
            target_node = next((n for n in nodes if n['id'] == flow['target']), None)
            
            if source_node and target_node:
                mappings.append({
                    'from': {
                        'nodeId': source_node['id'],
                        'nodeName': source_node.get('name', ''),
                        'nodeType': source_node.get('type', '')
                    },
                    'to': {
                        'nodeId': target_node['id'],
                        'nodeName': target_node.get('name', ''),
                        'nodeType': target_node.get('type', '')
                    },
                    'flowType': 'data'
                })
        
        return mappings
    
    def _resolve_type_name(self, type_id: Optional[str]) -> Optional[str]:
        """Resolve a type ID to its name"""
        if not type_id:
            return None
        
        elem = self.elements_by_id.get(type_id)
        if elem is not None:
            return elem.get('name')
        
        return None
    
    def _resolve_element_name(self, elem_id: Optional[str]) -> Optional[str]:
        """Resolve an element ID to its name"""
        if not elem_id:
            return None
        
        elem = self.elements_by_id.get(elem_id)
        if elem is not None:
            return elem.get('name', elem_id)
        
        return elem_id
    
    def _resolve_behavior_name(self, behavior_id: Optional[str]) -> Optional[str]:
        """Resolve a behavior ID to its name"""
        if not behavior_id:
            return None
        
        behavior = self.behaviors_by_id.get(behavior_id)
        if behavior is not None:
            return behavior.get('name')
        
        return None


def main():
    if len(sys.argv) < 3:
        print("Usage: python activity_parser.py <input.xmi> <output.json> [--package PACKAGE_NAME]")
        sys.exit(1)
    
    xmi_file = sys.argv[1]
    output_file = sys.argv[2]
    package_filter = None
    
    # Parse optional package filter
    i = 3
    while i < len(sys.argv):
        if sys.argv[i] == '--package' and i + 1 < len(sys.argv):
            package_filter = sys.argv[i + 1]
            i += 2
        else:
            output_file = sys.argv[i]
            i += 1
    
    print(f"Parsing activities from {xmi_file}...")
    if package_filter:
        print(f"Filtering to package: {package_filter}")
    
    parser = ActivityParser(xmi_file)
    result = parser.parse(package_filter=package_filter)
    
    print(f"[x] Parsed {len(result['activities'])} activities")
    
    # Write output
    with open(output_file, 'w') as f:
        json.dump(result, f, indent=2)
    
    print(f"[x] Output written to {output_file}")
    
    # Print summary
    for activity in result['activities']:
        namespace_str = ' > '.join(activity['namespace']) if activity['namespace'] else '(root)'
        print(f"\nActivity: {activity['name']}")
        print(f"  Namespace: {namespace_str}")
        print(f"  Parameters: {len(activity['parameters'])} (in/out)")
        print(f"  Nodes: {len(activity['nodes'])}")
        print(f"  Edges: {len(activity['edges'])}")
        
        pattern = activity.get('interactionPattern', {})
        consumed = pattern.get('consumedServices', [])
        if consumed:
            print(f"  Consumed Services: {len(consumed)}")
            for svc in consumed:
                print(f"    - {svc['behavior']}")


if __name__ == '__main__':
    main()
