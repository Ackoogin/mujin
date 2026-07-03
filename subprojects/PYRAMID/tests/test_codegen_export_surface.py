#!/usr/bin/env python3
"""Guard the import surface of cpp_codegen / ada_codegen.

The generator refactor plan (doc/plans/PYRAMID/generator_refactor_plan.md)
turns both monoliths into re-export shims.  This test pins the names their
importers actually consume (grep-derived), so a missed re-export fails here
as a unit failure instead of at generation time inside a packaged SDK.

Keep the lists in sync with the importers: generate_bindings.py,
cabi_codegen.py, ada_cabi_codegen.py, ros2_ir.py,
backends/flatbuffers_backend.py.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

PIM_DIR = Path(__file__).resolve().parents[1] / 'pim'
sys.path.insert(0, str(PIM_DIR))

# name -> importers that rely on it
CPP_CODEGEN_SURFACE = [
    # generate_bindings.py
    'CppTypesGenerator',
    'CppDataModelCodecGenerator',
    'CppServiceGenerator',
    'parse_proto',
    # ros2_ir.py
    'find_scalar_wrappers',
    '_CPP_SCALAR_MAP',
    '_cpp_ns_for_proto_type_package',
    # cabi_codegen.py
    '_DATA_MODEL_PROTO_ROOT',
    '_DATA_MODEL_TYPES_HEADER',
    '_DATA_MODEL_TYPES_NS',
    '_cpp_ns_for_proto_package',
    '_field_with_type',
    '_is_proto_enum_type',
    '_is_proto_message_type',
    '_package_for_proto_type',
    '_proto_type_fqn',
    '_resolve_message',
    # backends/flatbuffers_backend.py
    '_resolve_enum',
]

ADA_CODEGEN_SURFACE = [
    # generate_bindings.py
    'AdaTypesGenerator',
    'AdaDataModelCodecGenerator',
    'AdaGenericServiceGenerator',
    'AdaServiceGenerator',
    'parse_proto',
    # ada_cabi_codegen.py
    '_ADA_SCALAR_MAP',
    '_ada_array_name_for_repeated',
    '_ada_field_name',
    '_ada_pkg_from_proto_pkg',
    '_ada_name',
    '_ada_pkg_segment',
    '_proto_pkg_of_type',
    '_ada_cabi_pkg_from_proto_pkg',
    '_ensure_parent_packages',
]


def _missing(module_name: str, names: list[str]) -> list[str]:
    mod = importlib.import_module(module_name)
    return [n for n in names if not hasattr(mod, n)]


def test_cpp_codegen_exports():
    assert _missing('cpp_codegen', CPP_CODEGEN_SURFACE) == []


def test_ada_codegen_exports():
    assert _missing('ada_codegen', ADA_CODEGEN_SURFACE) == []
