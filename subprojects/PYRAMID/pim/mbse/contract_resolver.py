#!/usr/bin/env python3
"""
Service Contract Resolver
Matches Activities (behaviors) to Service definitions to create testable contracts
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple


class ContractResolver:
    """Resolve service contracts by matching activities to service interfaces"""
    
    def __init__(self, activities_file: str, datamodel_file: str):
        """Load activities and data model"""
        with open(activities_file, 'r') as f:
            self.activities_data = json.load(f)
        
        with open(datamodel_file, 'r') as f:
            self.datamodel_data = json.load(f)
        
        # Build indexes for fast lookup
        self._build_indexes()
    
    def _build_indexes(self):
        """Build lookup indexes for services and types"""
        self.services_by_namespace = {}
        self.services_by_property_type = {}
        self.types_by_name = {}
        
        # Index services (classes with flow properties)
        for cls in self.datamodel_data.get('classes', []):
            # Check if it's a service (has flow_direction properties)
            has_flow_props = any(
                prop.get('flow_direction') is not None 
                for prop in cls.get('properties', [])
            )
            
            if has_flow_props:
                # Index by namespace
                ns_key = tuple(cls.get('namespace', []))
                if ns_key not in self.services_by_namespace:
                    self.services_by_namespace[ns_key] = []
                self.services_by_namespace[ns_key].append(cls)
                
                # Index by property types
                for prop in cls.get('properties', []):
                    prop_type = prop.get('typeName')
                    if prop_type:
                        if prop_type not in self.services_by_property_type:
                            self.services_by_property_type[prop_type] = []
                        self.services_by_property_type[prop_type].append({
                            'service': cls,
                            'property': prop
                        })
        
        # Index all types
        for dt in self.datamodel_data.get('dataTypes', []):
            self.types_by_name[dt['name']] = dt
        for enum in self.datamodel_data.get('enumerations', []):
            self.types_by_name[enum['name']] = enum
        for cls in self.datamodel_data.get('classes', []):
            self.types_by_name[cls['name']] = cls
    
    def resolve_contracts(self) -> Dict[str, Any]:
        """Resolve all service contracts"""
        result = {
            'contracts': [],
            'unresolved_activities': [],
            'statistics': {
                'total_activities': len(self.activities_data.get('activities', [])),
                'resolved': 0,
                'unresolved': 0
            }
        }
        
        for activity in self.activities_data.get('activities', []):
            contract = self._resolve_activity_to_service(activity)
            
            if contract:
                result['contracts'].append(contract)
                result['statistics']['resolved'] += 1
            else:
                result['unresolved_activities'].append({
                    'activityName': activity['name'],
                    'namespace': activity.get('namespace', []),
                    'reason': 'No matching service found'
                })
                result['statistics']['unresolved'] += 1
        
        return result
    
    def _resolve_activity_to_service(self, activity: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Match an activity to a service definition"""
        activity_name = activity['name']
        activity_ns = activity.get('namespace', [])
        parameters = activity.get('parameters', [])
        
        # Extract input/output parameter types
        input_types = [p['typeName'] for p in parameters if p.get('direction') == 'in']
        output_types = [p['typeName'] for p in parameters if p.get('direction') in ['out', 'return']]
        
        # Strategy 1: Match by namespace proximity and parameter types
        matching_service = self._find_service_by_signature(
            input_types, 
            output_types, 
            activity_ns
        )
        
        if not matching_service:
            return None
        
        # Extract component information
        component_info = self._extract_component_info(activity_ns, matching_service)
        
        # Determine service type (provided/consumed)
        service_type = self._determine_service_type(matching_service['namespace'])
        
        # Extract consumed services from CallBehaviorActions
        consumed_services = self._extract_consumed_services(activity)
        
        # Build contract
        contract = {
            'component': component_info['component_name'],
            'componentNamespace': component_info['component_namespace'],
            'serviceType': service_type,
            'serviceInterface': matching_service['name'],
            'serviceNamespace': matching_service['namespace'],
            'implementedBy': {
                'activityId': activity['id'],
                'activityName': activity_name,
                'activityNamespace': activity_ns
            },
            'signature': {
                'inputs': [
                    {
                        'name': p['name'],
                        'type': p['typeName'],
                        'direction': p.get('direction', 'in')
                    }
                    for p in parameters if p.get('direction') == 'in'
                ],
                'outputs': [
                    {
                        'name': p['name'],
                        'type': p['typeName'],
                        'direction': p.get('direction', 'out')
                    }
                    for p in parameters if p.get('direction') in ['out', 'return']
                ]
            },
            'consumedServices': consumed_services,
            'interactionPattern': activity.get('interactionPattern', {}),
            'testable': True,
            'matchConfidence': 'high' if self._has_exact_match(matching_service, input_types, output_types) else 'medium'
        }
        
        return contract
    
    def _find_service_by_signature(
        self, 
        input_types: List[str], 
        output_types: List[str],
        activity_ns: List[str]
    ) -> Optional[Dict[str, Any]]:
        """Find service matching the parameter signature"""
        candidates = []
        
        # Find services that use these types
        all_param_types = set(input_types + output_types)
        
        for param_type in all_param_types:
            if param_type in self.services_by_property_type:
                for entry in self.services_by_property_type[param_type]:
                    service = entry['service']
                    prop = entry['property']
                    
                    # Check flow direction compatibility
                    flow_dir = prop.get('flow_direction')
                    
                    # Input types should match 'in' or 'inout' properties
                    if param_type in input_types and flow_dir in ['in', 'inout']:
                        candidates.append((service, 2))  # Higher score
                    # Output types should match 'out' or 'inout' properties
                    elif param_type in output_types and flow_dir in ['out', 'inout']:
                        candidates.append((service, 2))
                    # 'inout' matches both
                    elif flow_dir == 'inout':
                        candidates.append((service, 1))
        
        if not candidates:
            return None
        
        # Score candidates by namespace proximity
        scored_candidates = []
        for service, type_score in candidates:
            service_ns = service.get('namespace', [])
            proximity_score = self._calculate_namespace_proximity(activity_ns, service_ns)
            total_score = type_score + proximity_score
            scored_candidates.append((service, total_score))
        
        # Return highest scoring unique service
        if scored_candidates:
            scored_candidates.sort(key=lambda x: x[1], reverse=True)
            return scored_candidates[0][0]
        
        return None
    
    def _calculate_namespace_proximity(self, ns1: List[str], ns2: List[str]) -> int:
        """Calculate how close two namespaces are (higher = closer)"""
        # Count matching prefix elements
        score = 0
        for i in range(min(len(ns1), len(ns2))):
            if ns1[i] == ns2[i]:
                score += 2
            else:
                break
        
        # Bonus if one is contained in the other
        if len(ns1) < len(ns2) and ns2[:len(ns1)] == ns1:
            score += 1
        elif len(ns2) < len(ns1) and ns1[:len(ns2)] == ns2:
            score += 1
        
        return score
    
    def _has_exact_match(
        self, 
        service: Dict[str, Any], 
        input_types: List[str], 
        output_types: List[str]
    ) -> bool:
        """Check if service properties exactly match parameter types"""
        service_props = service.get('properties', [])
        
        # Check input types
        for input_type in input_types:
            found = any(
                prop['typeName'] == input_type and 
                prop.get('flow_direction') in ['in', 'inout']
                for prop in service_props
            )
            if not found:
                return False
        
        # Check output types
        for output_type in output_types:
            found = any(
                prop['typeName'] == output_type and 
                prop.get('flow_direction') in ['out', 'inout']
                for prop in service_props
            )
            if not found:
                return False
        
        return True
    
    def _extract_component_info(
        self, 
        activity_ns: List[str], 
        service: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Extract component information from namespace"""
        # Try to find component name in namespace hierarchy
        # Typically: ["2. Components", "7. Tactical Objects", ...]
        component_name = None
        component_ns = []
        
        # Look for "Components" in namespace
        if activity_ns:
            for i, part in enumerate(activity_ns):
                if 'component' in part.lower():
                    # Next element is likely the component name
                    if i + 1 < len(activity_ns):
                        component_name = activity_ns[i + 1]
                        component_ns = activity_ns[:i + 2]
                    break
        
        # Fallback to service namespace
        if not component_name:
            service_ns = service.get('namespace', [])
            for i, part in enumerate(service_ns):
                if 'component' in part.lower():
                    if i + 1 < len(service_ns):
                        component_name = service_ns[i + 1]
                        component_ns = service_ns[:i + 2]
                    break
        
        # Final fallback
        if not component_name and activity_ns:
            component_name = activity_ns[0] if activity_ns else 'Unknown'
            component_ns = activity_ns[:1]
        
        return {
            'component_name': component_name or 'Unknown',
            'component_namespace': component_ns
        }
    
    def _determine_service_type(self, service_ns: List[str]) -> str:
        """Determine if service is provided or consumed based on namespace"""
        ns_str = ' '.join(service_ns).lower()
        
        if 'provided' in ns_str or 'offered' in ns_str:
            return 'provided'
        elif 'consumed' in ns_str or 'required' in ns_str:
            return 'consumed'
        
        return 'unknown'
    
    def _extract_consumed_services(self, activity: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Extract consumed service calls from activity"""
        consumed = []
        
        pattern = activity.get('interactionPattern', {})
        consumed_services = pattern.get('consumedServices', [])
        
        for svc in consumed_services:
            # Try to resolve the behavior to a service
            action_name = svc.get('actionName', '')
            behavior_name = svc.get('behavior')
            
            # Extract types from inputs/outputs
            input_types = [inp.get('typeName') for inp in svc.get('inputs', []) if inp.get('typeName')]
            output_types = [out.get('typeName') for out in svc.get('outputs', []) if out.get('typeName')]
            
            consumed_entry = {
                'sequence': svc.get('sequence'),
                'actionName': action_name,
                'behaviorName': behavior_name,
                'inputTypes': input_types,
                'outputTypes': output_types,
                'resolved': False
            }
            
            # Try to resolve to a service interface
            if input_types or output_types:
                matching_service = self._find_service_by_signature(
                    input_types,
                    output_types,
                    []  # No namespace constraint for consumed services
                )
                
                if matching_service:
                    consumed_entry['resolved'] = True
                    consumed_entry['serviceInterface'] = matching_service['name']
                    consumed_entry['serviceNamespace'] = matching_service['namespace']
            
            consumed.append(consumed_entry)
        
        return consumed


def main():
    if len(sys.argv) < 4:
        print("Usage: python contract_resolver.py <activities.json> <datamodel.json> <output.json>")
        sys.exit(1)
    
    activities_file = sys.argv[1]
    datamodel_file = sys.argv[2]
    output_file = sys.argv[3]
    
    print(f"Resolving service contracts...")
    print(f"  Activities: {activities_file}")
    print(f"  Data Model: {datamodel_file}")
    
    resolver = ContractResolver(activities_file, datamodel_file)
    result = resolver.resolve_contracts()
    
    # Write output
    with open(output_file, 'w') as f:
        json.dump(result, f, indent=2)
    
    # Print summary
    stats = result['statistics']
    print(f"\n[x] Contract Resolution Complete")
    print(f"  Total Activities: {stats['total_activities']}")
    print(f"  Resolved: {stats['resolved']}")
    print(f"  Unresolved: {stats['unresolved']}")
    
    print(f"\n[x] Contracts written to {output_file}")
    
    # Print contract summary
    if result['contracts']:
        print(f"\nResolved Contracts:")
        for contract in result['contracts']:
            component = contract['component']
            service = contract['serviceInterface']
            service_type = contract['serviceType']
            activity = contract['implementedBy']['activityName']
            confidence = contract['matchConfidence']
            
            print(f"\n  [{confidence.upper()}] {component}")
            print(f"    {service_type.capitalize()} Service: {service}")
            print(f"    Implemented by: {activity}")
            
            consumed = contract.get('consumedServices', [])
            if consumed:
                print(f"    Consumes:")
                for svc in consumed:
                    if svc.get('resolved'):
                        print(f"      - {svc['serviceInterface']}")
                    else:
                        print(f"      - {svc['actionName']} (unresolved)")
    
    if result['unresolved_activities']:
        print(f"\nUnresolved Activities:")
        for unresolved in result['unresolved_activities']:
            print(f"  - {unresolved['activityName']}")
            print(f"    Reason: {unresolved['reason']}")


if __name__ == '__main__':
    main()
