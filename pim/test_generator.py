#!/usr/bin/env python3
"""
Contract Test Generator
Generates pytest tests that verify component mocks adhere to contracts
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any
import re


class TestGenerator:
    """Generate pytest tests from service contracts"""
    
    def __init__(self, contracts_file: str, datamodel_file: str):
        """Load contracts and data model"""
        with open(contracts_file, 'r') as f:
            self.contracts = json.load(f)
        
        with open(datamodel_file, 'r') as f:
            self.datamodel = json.load(f)
    
    def generate(self, output_dir: str):
        """Generate test files"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Group contracts by component
        by_component = self._group_by_component()
        
        for component_name, contracts in by_component.items():
            module_name = self._to_snake_case(component_name)
            file_path = output_path / f"test_{module_name}.py"
            
            with open(file_path, 'w') as f:
                self._write_test_file(f, component_name, contracts)
            
            print(f"  Generated {file_path}")
        
        # Generate conftest.py with fixtures
        self._generate_conftest(output_path, by_component.keys())
        
        # Generate pytest.ini
        self._generate_pytest_ini(output_path)
    
    def _group_by_component(self) -> Dict[str, List[Dict]]:
        """Group contracts by component"""
        by_component = {}
        
        for contract in self.contracts.get('contracts', []):
            component = contract['component']
            if component not in by_component:
                by_component[component] = []
            by_component[component].append(contract)
        
        return by_component
    
    def _write_test_file(self, f, component_name: str, contracts: List[Dict]):
        """Write test file for a component"""
        class_name = self._to_pascal_case(component_name)
        mock_class = f"Mock{class_name}"
        
        # Write header
        f.write('"""\n')
        f.write(f'Contract Tests: {component_name}\n')
        f.write('Auto-generated from service contracts\n')
        f.write('"""\n\n')
        
        # Write imports
        f.write('import pytest\n')
        f.write('from unittest.mock import Mock, MagicMock, call\n')
        f.write('from typing import Any\n\n')
        
        module_name = self._to_snake_case(component_name)
        f.write(f'from mocks.mock_{module_name} import {mock_class}\n\n')
        
        # Write fixture
        f.write('@pytest.fixture\n')
        f.write(f'def mock_{module_name}():\n')
        f.write(f'    """Fixture providing mock {component_name} instance"""\n')
        f.write(f'    return {mock_class}()\n\n')
        
        # Write tests for each contract
        for idx, contract in enumerate(contracts, 1):
            if contract['serviceType'] == 'provided':
                self._write_contract_tests(f, component_name, contract, idx)
    
    def _write_contract_tests(self, f, component_name: str, contract: Dict, contract_num: int):
        """Write tests for a single contract"""
        service_name = contract['serviceInterface']
        activity_name = contract['implementedBy']['activityName']
        method_name = self._to_snake_case(activity_name)
        fixture_name = self._to_snake_case(component_name)
        
        f.write(f'# Contract {contract_num}: {service_name}\n')
        f.write(f'# Activity: {activity_name}\n\n')
        
        # Test 1: Method exists
        f.write(f'def test_{method_name}_exists(mock_{fixture_name}):\n')
        f.write(f'    """Test that {method_name} method exists"""\n')
        f.write(f'    assert hasattr(mock_{fixture_name}, "{method_name}")\n')
        f.write(f'    assert callable(getattr(mock_{fixture_name}, "{method_name}"))\n\n')
        
        # Test 2: Signature matches contract
        inputs = contract['signature'].get('inputs', [])
        outputs = contract['signature'].get('outputs', [])
        
        if inputs:
            f.write(f'def test_{method_name}_accepts_inputs(mock_{fixture_name}):\n')
            f.write(f'    """Test that {method_name} accepts correct input types"""\n')
            f.write('    # Create mock inputs\n')
            for inp in inputs:
                param_name = self._to_snake_case(inp['name'])
                param_type = inp['type']
                f.write(f'    {param_name} = None  # TODO: Create mock {param_type}\n')
            
            f.write('\n    # Should not raise exception\n')
            param_names = [self._to_snake_case(inp['name']) for inp in inputs]
            f.write(f'    result = mock_{fixture_name}.{method_name}({", ".join(param_names)})\n\n')
        
        # Test 3: Returns correct output
        if outputs:
            f.write(f'def test_{method_name}_returns_output(mock_{fixture_name}):\n')
            f.write(f'    """Test that {method_name} returns correct output types"""\n')
            f.write('    # Create mock inputs\n')
            for inp in inputs:
                param_name = self._to_snake_case(inp['name'])
                f.write(f'    {param_name} = None  # TODO: Create mock {inp["type"]}\n')
            
            f.write('\n')
            param_names = [self._to_snake_case(inp['name']) for inp in inputs]
            f.write(f'    result = mock_{fixture_name}.{method_name}({", ".join(param_names)})\n')
            f.write('    \n')
            f.write('    # Verify return type\n')
            if len(outputs) == 1:
                f.write(f'    # Should return {outputs[0]["type"]}\n')
                f.write('    # assert isinstance(result, ExpectedType)\n')
            else:
                f.write(f'    # Should return tuple of ({", ".join(out["type"] for out in outputs)})\n')
                f.write('    # assert isinstance(result, tuple)\n')
                f.write(f'    # assert len(result) == {len(outputs)}\n')
            f.write('\n')
        
        # Test 4: Calls consumed services
        consumed = contract.get('consumedServices', [])
        resolved_consumed = [c for c in consumed if c.get('resolved')]
        
        if resolved_consumed:
            f.write(f'def test_{method_name}_calls_consumed_services(mock_{fixture_name}):\n')
            f.write(f'    """Test that {method_name} calls required consumed services"""\n')
            f.write('    # Setup mock consumed services\n')
            
            for svc in resolved_consumed:
                svc_name = svc['serviceInterface']
                svc_var = self._to_snake_case(svc_name)
                f.write(f'    mock_{svc_var} = Mock(name="{svc_name}")\n')
                f.write(f'    mock_{fixture_name}.configure_consumed_service("{svc_name}", mock_{svc_var})\n')
            
            f.write('\n    # Create mock inputs and call\n')
            for inp in inputs:
                param_name = self._to_snake_case(inp['name'])
                f.write(f'    {param_name} = None  # TODO: Create mock {inp["type"]}\n')
            
            param_names = [self._to_snake_case(inp['name']) for inp in inputs]
            f.write(f'    result = mock_{fixture_name}.{method_name}({", ".join(param_names)})\n')
            f.write('    \n')
            f.write('    # Verify consumed services were called\n')
            
            for svc in resolved_consumed:
                svc_name = svc['serviceInterface']
                svc_var = self._to_snake_case(svc_name)
                action_name = svc.get('actionName', 'call')
                method_name_svc = self._to_snake_case(action_name) if action_name else 'call'
                f.write(f'    # Should call {svc_name}\n')
                f.write(f'    mock_{svc_var}.{method_name_svc}.assert_called_once()\n')
            
            f.write('\n')
        
        # Test 5: Call sequence verification
        if len(resolved_consumed) > 1:
            f.write(f'def test_{method_name}_call_sequence(mock_{fixture_name}):\n')
            f.write(f'    """Test that {method_name} calls services in correct sequence"""\n')
            f.write('    # Setup mock consumed services\n')
            
            for svc in resolved_consumed:
                svc_name = svc['serviceInterface']
                svc_var = self._to_snake_case(svc_name)
                f.write(f'    mock_{svc_var} = Mock(name="{svc_name}")\n')
                f.write(f'    mock_{fixture_name}.configure_consumed_service("{svc_name}", mock_{svc_var})\n')
            
            f.write('\n    # Create mock inputs and call\n')
            for inp in inputs:
                param_name = self._to_snake_case(inp['name'])
                f.write(f'    {param_name} = None  # TODO: Create mock {inp["type"]}\n')
            
            param_names = [self._to_snake_case(inp['name']) for inp in inputs]
            f.write(f'    result = mock_{fixture_name}.{method_name}({", ".join(param_names)})\n')
            f.write('    \n')
            f.write('    # Verify call order\n')
            f.write('    # Expected sequence:\n')
            
            for svc in sorted(resolved_consumed, key=lambda x: x.get('sequence', 0)):
                f.write(f'    #   {svc["sequence"]}. {svc["serviceInterface"]}.{svc["actionName"]}\n')
            
            f.write('    # TODO: Implement sequence verification\n')
            f.write('\n')
        
        # Test 6: Records calls for testing
        f.write(f'def test_{method_name}_records_calls(mock_{fixture_name}):\n')
        f.write(f'    """Test that {method_name} records calls in history"""\n')
        f.write(f'    initial_count = mock_{fixture_name}.get_call_count("{method_name}")\n')
        f.write('    \n')
        
        for inp in inputs:
            param_name = self._to_snake_case(inp['name'])
            f.write(f'    {param_name} = None  # TODO: Create mock {inp["type"]}\n')
        
        param_names = [self._to_snake_case(inp['name']) for inp in inputs]
        f.write(f'    mock_{fixture_name}.{method_name}({", ".join(param_names)})\n')
        f.write('    \n')
        f.write(f'    assert mock_{fixture_name}.get_call_count("{method_name}") == initial_count + 1\n')
        f.write(f'    calls = mock_{fixture_name}.get_calls("{method_name}")\n')
        f.write('    assert len(calls) > 0\n')
        f.write('    assert calls[-1]["method"] == "' + method_name + '"\n\n')
    
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
    
    def _generate_conftest(self, output_path: Path, components):
        """Generate pytest configuration"""
        conftest_file = output_path / 'conftest.py'
        
        with open(conftest_file, 'w') as f:
            f.write('"""\n')
            f.write('Pytest configuration and shared fixtures\n')
            f.write('"""\n\n')
            f.write('import pytest\n')
            f.write('import sys\n')
            f.write('from pathlib import Path\n\n')
            
            f.write('# Add parent directory to path for imports\n')
            f.write('sys.path.insert(0, str(Path(__file__).parent.parent))\n\n')
            
            f.write('@pytest.fixture(autouse=True)\n')
            f.write('def reset_mocks(request):\n')
            f.write('    """Reset all mocks after each test"""\n')
            f.write('    yield\n')
            f.write('    # Cleanup code here if needed\n')
        
        print(f"  Generated {conftest_file}")
    
    def _generate_pytest_ini(self, output_path: Path):
        """Generate pytest.ini configuration"""
        ini_file = output_path / 'pytest.ini'
        
        with open(ini_file, 'w') as f:
            f.write('[pytest]\n')
            f.write('testpaths = .\n')
            f.write('python_files = test_*.py\n')
            f.write('python_classes = Test*\n')
            f.write('python_functions = test_*\n')
            f.write('addopts = -v --tb=short\n')
        
        print(f"  Generated {ini_file}")


def main():
    if len(sys.argv) < 4:
        print("Usage: python test_generator.py <contracts.json> <datamodel.json> <output_dir>")
        sys.exit(1)
    
    contracts_file = sys.argv[1]
    datamodel_file = sys.argv[2]
    output_dir = sys.argv[3]
    
    print(f"Generating contract tests...")
    print(f"  Contracts: {contracts_file}")
    print(f"  Data Model: {datamodel_file}")
    print(f"  Output: {output_dir}")
    
    generator = TestGenerator(contracts_file, datamodel_file)
    generator.generate(output_dir)
    
    print(f"\n✓ Contract tests generated in {output_dir}/")
    print(f"\nRun tests with: pytest {output_dir}")


if __name__ == '__main__':
    main()
