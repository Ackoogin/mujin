#!/usr/bin/env python3
"""Guard the public import surface of the split code generators.

The generator refactor (doc/plans/PYRAMID/generator_refactor_plan.md) split the
old cpp_codegen.py / ada_codegen.py monoliths into the pim/cpp/ and pim/ada/
packages; the compatibility shims were retired once no internal importer relied
on them (there are no external SDK consumers). The generator is still shipped to
downstream developers inside the packaged SDK, so this test pins the public
names each consumer imports at their new module homes -- a rename/removal fails
here as a unit failure instead of at generation time inside a packaged SDK.

Keep the maps in sync with the importers (generate_bindings.py, cabi_codegen.py,
ada_cabi_codegen.py, ros2_ir.py, backends/flatbuffers_backend.py).
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

PIM_DIR = Path(__file__).resolve().parents[1] / 'pim'
sys.path.insert(0, str(PIM_DIR))

# module -> public names that must remain importable from it
CPP_CODEGEN_SURFACE = {
    'cpp.types_gen': ['CppTypesGenerator', 'find_scalar_wrappers'],
    'cpp.json_codec_gen': ['CppDataModelCodecGenerator'],
    'cpp.service_gen': ['CppServiceGenerator'],
    'cpp.naming': [
        '_CPP_SCALAR_MAP',
        '_DATA_MODEL_TYPES_HEADER',
        '_DATA_MODEL_TYPES_NS',
        '_cpp_ns_for_proto_package',
        '_cpp_ns_for_proto_type_package',
    ],
    'proto_parser': ['parse_proto'],
    'proto_resolve': [
        '_DATA_MODEL_PROTO_ROOT',
        '_field_with_type',
        '_is_proto_enum_type',
        '_is_proto_message_type',
        '_package_for_proto_type',
        '_proto_type_fqn',
        '_resolve_message',
        '_resolve_enum',
    ],
}

ADA_CODEGEN_SURFACE = {
    'ada.types_gen': ['AdaTypesGenerator'],
    'ada.codec_gen': ['AdaDataModelCodecGenerator'],
    'ada.generic_service_gen': ['AdaGenericServiceGenerator'],
    'ada.service_gen': ['AdaServiceGenerator'],
    'proto_parser': ['parse_proto'],
    'ada.naming': [
        '_ADA_SCALAR_MAP',
        '_ada_array_name_for_repeated',
        '_ada_field_name',
        '_ada_pkg_from_proto_pkg',
        '_ada_name',
        '_ada_pkg_segment',
        '_proto_pkg_of_type',
        '_ada_cabi_pkg_from_proto_pkg',
        '_ensure_parent_packages',
    ],
}


def _missing(surface: dict) -> list[str]:
    gaps = []
    for module_name, names in surface.items():
        mod = importlib.import_module(module_name)
        gaps.extend(
            f'{module_name}.{n}' for n in names if not hasattr(mod, n))
    return gaps


def test_cpp_codegen_exports():
    assert _missing(CPP_CODEGEN_SURFACE) == []


def test_ada_codegen_exports():
    assert _missing(ADA_CODEGEN_SURFACE) == []
