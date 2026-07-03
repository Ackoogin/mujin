#!/usr/bin/env python3
"""Role-neutral generic-layout Ada service facade generator.

Split verbatim from ada_codegen.py (generator refactor plan, phase 4).
"""

from pathlib import Path
from typing import List, Tuple

from .naming import (
    _short_type,
    _generic_pkg_name_from_proto,
    _proto_pkg_of_type,
    _ada_pkg_from_proto_pkg,
    _ada_name,
)


class AdaGenericServiceGenerator:
    """Role-neutral Ada service facade for arbitrary (non-CRUD) proto services.

    The PYRAMID ``AdaServiceGenerator`` assumes CRUD-named RPCs
    (Create/Read/Update/Delete via ``OP_PREFIXES``) and silently drops any RPC
    that is not CRUD-shaped. Generic proto services use arbitrary RPC names, so
    this generator emits a minimal, role-neutral facade -- per-RPC wire-name
    constants plus one access-to-function handler per RPC, gathered into a
    ``Service_Handlers`` record. It mirrors the C++ generic ``ServiceHandler``
    and depends only on the generated types package, so it compiles standalone
    with no PCL, CRUD, or PYRAMID assumptions.

    Takes a parsed ``proto_parser.ProtoFile`` (services keep every RPC, unlike
    the CRUD-filtering ``ProtoService`` in this module).
    """

    def __init__(self, proto_file, enabled_backends=None):
        self._pf = proto_file

    def _ada_type(self, rpc_type: str, svc_pkg: str) -> Tuple[str, str]:
        """Return (Ada types package, Ada type name) for an RPC message type."""
        short = _short_type(rpc_type)
        pkg = _proto_pkg_of_type(rpc_type) or svc_pkg
        return _ada_pkg_from_proto_pkg(pkg), _ada_name(short)

    def generate(self, output_dir: str) -> None:
        pf = self._pf
        if not pf.services:
            return
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        pkg = _generic_pkg_name_from_proto(pf)
        path = out / (pkg.lower().replace('.', '-') + '.ads')

        type_pkgs: set = set()
        rows: List[Tuple[str, str, str, str]] = []  # base, wire, req, rsp
        for svc in pf.services:
            for rpc in svc.rpcs:
                req_pkg, req_ada = self._ada_type(rpc.request_type, pf.package)
                rsp_pkg, rsp_ada = self._ada_type(rpc.response_type, pf.package)
                type_pkgs.add(req_pkg)
                type_pkgs.add(rsp_pkg)
                base = f'{_ada_name(svc.name)}_{_ada_name(rpc.name)}'
                wire = f'{pf.package}.{svc.name}/{rpc.name}'
                rows.append((base, wire, req_ada, rsp_ada))

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated role-neutral service facade '
                    '(generic contract layout)\n')
            f.write(f'--  Package: {pkg}\n')
            f.write('--  Handlers are role-neutral: one access-to-function per '
                    'RPC (see C++ ServiceHandler).\n\n')
            for tp in sorted(type_pkgs):
                f.write(f'with {tp};  use {tp};\n')
            f.write(f'\npackage {pkg} is\n\n')
            f.write('   --  Service wire-name constants (package.Service/Rpc)\n')
            for base, wire, _req, _rsp in rows:
                f.write(f'   Svc_{base} : constant String := "{wire}";\n')
            f.write('\n   --  Role-neutral handlers: one access-to-function '
                    'per RPC.\n')
            for base, _wire, req_ada, rsp_ada in rows:
                f.write(f'   type Handle_{base} is\n')
                f.write(f'     access function (Request : {req_ada}) '
                        f'return {rsp_ada};\n')
            f.write('\n   type Service_Handlers is record\n')
            for base, _wire, _req, _rsp in rows:
                f.write(f'      On_{base} : Handle_{base} := null;\n')
            f.write('   end record;\n')
            f.write(f'\nend {pkg};\n')
        print(f'  Generated {pkg}')


