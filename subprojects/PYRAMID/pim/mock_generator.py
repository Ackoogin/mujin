#!/usr/bin/env python3
"""
Component Mock Generator
Generates mock components that implement service contracts
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any
import re


class MockGenerator:
    """Generate mock components from service contracts"""
    
    def __init__(self, contracts_file: str, datamodel_file: str):
        """Load contracts and data model"""
        with open(contracts_file, 'r') as f:
            self.contracts = json.load(f)
        
        with open(datamodel_file, 'r') as f:
            self.datamodel = json.load(f)
        
        # Build type lookup
        self.types_by_name = {}
        for dt in self.datamodel.get('dataTypes', []):
            self.types_by_name[dt['name']] = dt
        for enum in self.datamodel.get('enumerations', []):
            self.types_by_name[enum['name']] = enum
    
    def generate(self, output_dir: str):
        """Generate mock components"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Group contracts by component
        by_component = self._group_by_component()
        
        for component_name, contracts in by_component.items():
            module_name = self._to_snake_case(component_name)
            file_path = output_path / f"mock_{module_name}.py"
            
            with open(file_path, 'w') as f:
                self._write_mock_component(f, component_name, contracts)
            
            print(f"  Generated {file_path}")
        
        # Generate __init__.py
        self._generate_init_file(output_path, by_component.keys())
    
    def _group_by_component(self) -> Dict[str, List[Dict]]:
        """Group contracts by component"""
        by_component = {}
        
        for contract in self.contracts.get('contracts', []):
            component = contract['component']
            if component not in by_component:
                by_component[component] = []
            by_component[component].append(contract)
        
        return by_component
    
    def _write_mock_component(self, f, component_name: str, contracts: List[Dict]):
        """Write mock component class"""
        class_name = f"Mock{self._to_pascal_case(component_name)}"
        
        # Write header
        f.write('"""\n')
        f.write(f'Mock Component: {component_name}\n')
        f.write('Auto-generated from service contracts\n')
        f.write('"""\n\n')
        
        # Write imports
        f.write('from typing import List, Optional, Dict, Any, Callable\n')
        f.write('from dataclasses import dataclass, field\n')
        f.write('from unittest.mock import Mock\n')
        f.write('import logging\n\n')
        
        f.write('logger = logging.getLogger(__name__)\n\n')
        
        # Write mock component class
        f.write(f'class {class_name}:\n')
        f.write(f'    """\n')
        f.write(f'    Mock implementation of {component_name} component\n')
        f.write(f'    \n')
        f.write(f'    Implements {len(contracts)} service contract(s):\n')
        for contract in contracts:
            f.write(f'    - {contract["serviceInterface"]} ({contract["serviceType"]})\n')
        f.write(f'    """\n\n')
        
        # Write __init__
        f.write('    def __init__(self):\n')
        f.write('        # Track method calls for verification\n')
        f.write('        self.call_history: List[Dict[str, Any]] = []\n')
        f.write('        \n')
        f.write('        # Mock consumed services\n')
        f.write('        self.consumed_services: Dict[str, Mock] = {}\n')
        
        # Initialize mocks for consumed services
        consumed_services = set()
        for contract in contracts:
            for consumed in contract.get('consumedServices', []):
                if consumed.get('resolved'):
                    service_name = consumed['serviceInterface']
                    consumed_services.add(service_name)
        
        for service_name in sorted(consumed_services):
            mock_var = self._to_snake_case(service_name)
            f.write(f'        self.consumed_services["{service_name}"] = Mock(name="{service_name}")\n')
        
        f.write('        \n')
        f.write('        # Configure default responses\n')
        f.write('        self._setup_default_responses()\n')
        f.write('        \n')
        f.write('        logger.info(f"Initialized {self.__class__.__name__}")\n\n')
        
        # Write setup method
        f.write('    def _setup_default_responses(self):\n')
        f.write('        """Configure default mock responses for consumed services"""\n')
        f.write('        # Override this method to customize mock behavior\n')
        f.write('        pass\n\n')
        
        # Write methods for each provided service
        for contract in contracts:
            if contract['serviceType'] == 'provided':
                self._write_service_methods(f, contract)
        
        # Write helper methods
        self._write_helper_methods(f)
    
    def _write_service_methods(self, f, contract: Dict[str, Any]):
        """Write methods implementing a service contract"""
        service_name = contract['serviceInterface']
        activity_name = contract['implementedBy']['activityName']
        signature = contract['signature']
        consumed = contract.get('consumedServices', [])
        
        f.write(f'    # Service: {service_name}\n')
        f.write(f'    # Implements: {activity_name}\n\n')
        
        # Generate method name from activity
        method_name = self._to_snake_case(activity_name)
        
        # Build parameter list
        params = ['self']
        for inp in signature.get('inputs', []):
            param_name = self._to_snake_case(inp['name'])
            param_type = inp['type']
            params.append(f'{param_name}: "{param_type}"')
        
        # Build return type
        outputs = signature.get('outputs', [])
        if len(outputs) == 0:
            return_type = 'None'
        elif len(outputs) == 1:
            return_type = f'"{outputs[0]["type"]}"'
        else:
            types = [f'"{out["type"]}"' for out in outputs]
            return_type = f'tuple[{", ".join(types)}]'
        
        # Write method signature
        f.write(f'    def {method_name}(')
        f.write(', '.join(params))
        f.write(f') -> {return_type}:\n')
        
        # Write docstring
        f.write(f'        """\n')
        f.write(f'        {activity_name}\n')
        f.write(f'        \n')
        if signature.get('inputs'):
            f.write(f'        Args:\n')
            for inp in signature['inputs']:
                f.write(f'            {self._to_snake_case(inp["name"])}: {inp["type"]}\n')
        f.write(f'        \n')
        if outputs:
            f.write(f'        Returns:\n')
            for out in outputs:
                f.write(f'            {out["type"]}\n')
        f.write(f'        """\n')
        
        # Record call
        f.write('        self.call_history.append({\n')
        f.write(f'            "method": "{method_name}",\n')
        f.write('            "args": {')
        arg_names = [self._to_snake_case(inp['name']) for inp in signature.get('inputs', [])]
        f.write(', '.join(f'"{name}": {name}' for name in arg_names))
        f.write('}\n')
        f.write('        })\n')
        f.write(f'        logger.debug(f"Called {method_name}")\n\n')
        
        # Call consumed services if any
        if consumed:
            f.write('        # Call consumed services\n')
            for svc in consumed:
                if svc.get('resolved'):
                    svc_name = svc['serviceInterface']
                    action_name = svc['actionName']
                    f.write(f'        # {action_name}\n')
                    f.write(f'        mock_svc = self.consumed_services.get("{svc_name}")\n')
                    f.write(f'        if mock_svc:\n')
                    
                    # Generate valid method name from action
                    svc_method = self._to_snake_case(action_name) if action_name else 'call'
                    input_args = ', '.join(arg_names[:1]) if arg_names else ''  # Pass first arg
                    f.write(f'            result_{svc["sequence"]} = mock_svc.{svc_method}({input_args})\n')
                    f.write(f'        else:\n')
                    f.write(f'            result_{svc["sequence"]} = None\n')
                    f.write('\n')
        
        # Return mock response
        if outputs:
            f.write('        # Return mock response\n')
            f.write('        # TODO: Customize response based on business logic\n')
            if len(outputs) == 1:
                out_type = outputs[0]['type']
                f.write(f'        return None  # Replace with mock {out_type}\n')
            else:
                f.write('        return (')
                f.write(', '.join('None' for _ in outputs))
                f.write(')  # Replace with mock objects\n')
        
        f.write('\n')
    
    def _write_helper_methods(self, f):
        """Write helper methods for testing"""
        f.write('    # Testing Helper Methods\n\n')
        
        f.write('    def get_call_count(self, method_name: str) -> int:\n')
        f.write('        """Get number of times a method was called"""\n')
        f.write('        return sum(1 for call in self.call_history if call["method"] == method_name)\n\n')
        
        f.write('    def get_calls(self, method_name: str) -> List[Dict[str, Any]]:\n')
        f.write('        """Get all calls to a specific method"""\n')
        f.write('        return [call for call in self.call_history if call["method"] == method_name]\n\n')
        
        f.write('    def reset_history(self):\n')
        f.write('        """Clear call history"""\n')
        f.write('        self.call_history.clear()\n')
        f.write('        logger.debug("Reset call history")\n\n')
        
        f.write('    def configure_consumed_service(self, service_name: str, mock_impl: Mock):\n')
        f.write('        """Configure behavior of a consumed service mock"""\n')
        f.write('        self.consumed_services[service_name] = mock_impl\n')
        f.write('        logger.debug(f"Configured consumed service: {service_name}")\n\n')
    
    def _to_snake_case(self, name: str) -> str:
        """Convert to snake_case"""
        if not name:
            return 'unnamed_action'
        name = re.sub(r'[^\w\s]', '', name)
        name = re.sub(r'\s+', '_', name)
        name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', name)
        result = name.lower()
        return result if result else 'unnamed_action'
    
    def _to_pascal_case(self, name: str) -> str:
        """Convert to PascalCase"""
        name = re.sub(r'[^\w\s]', '', name)
        parts = name.split()
        return ''.join(word.capitalize() for word in parts)
    
    def _generate_init_file(self, output_path: Path, components):
        """Generate __init__.py"""
        init_file = output_path / '__init__.py'
        
        with open(init_file, 'w') as f:
            f.write('"""\n')
            f.write('Component Mocks Package\n')
            f.write('Auto-generated from service contracts\n')
            f.write('"""\n\n')
            
            # Import all mocks
            for component in sorted(components):
                module_name = f"mock_{self._to_snake_case(component)}"
                class_name = f"Mock{self._to_pascal_case(component)}"
                f.write(f'from .{module_name} import {class_name}\n')
            
            f.write('\n__all__ = [\n')
            for component in sorted(components):
                class_name = f"Mock{self._to_pascal_case(component)}"
                f.write(f"    '{class_name}',\n")
            f.write(']\n')
        
        print(f"  Generated {init_file}")


def main():
    if len(sys.argv) < 4:
        print("Usage: python mock_generator.py <contracts.json> <datamodel.json> <output_dir>")
        sys.exit(1)
    
    contracts_file = sys.argv[1]
    datamodel_file = sys.argv[2]
    output_dir = sys.argv[3]
    
    print(f"Generating component mocks...")
    print(f"  Contracts: {contracts_file}")
    print(f"  Data Model: {datamodel_file}")
    print(f"  Output: {output_dir}")
    
    generator = MockGenerator(contracts_file, datamodel_file)
    generator.generate(output_dir)
    
    print(f"\n[x] Component mocks generated in {output_dir}/")


if __name__ == '__main__':
    main()
