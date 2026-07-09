#!/usr/bin/env python3
"""Ada interaction facade generator -- Phase 4 (spec-only fallback).

doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md Phase 4's own risk
note pre-authorizes falling back to "spec-only" (declarations plus
pragma-stubbed bodies) if full Ada functional parity with the C++ Phase 2/3
runtime facade (subprojects/PYRAMID/pim/cpp/interaction_facade_gen.py) would
balloon the generator. That call is made here: the existing Ada service
binding generator (naming.py, service_spec_gen.py, service_body_gen.py) is a
flat procedural Register_Services/Dispatch surface with no tagged-type or
dynamic-dispatch machinery, and this build environment has no GNAT/gprbuild
to compile-verify anything emitted -- true for spec-only or full parity
alike. Reproducing the C++ facade's runtime dispatch (SubscriptionHandle,
TransitionWriter fan-out, a bounded snapshot store, oneof-variant routing)
in Ada from a standing start would be a large, unverifiable leap. Instead,
this module emits the *declared* interaction surface -- the same shape
client/provider code would program against -- with bodies that raise
Program_Error (the codebase's existing not-yet-implemented convention, see
service_body_gen.py), so a later phase can wire real dynamic dispatch behind
an unchanged spec.

Reuses `binding_contract.interaction_for_service` (Phase 0/1) for the
Request-shape classification, keeping the declared Ada surface consistent
with the same model the C++ facade is built from, rather than re-deriving a
parallel one.
"""

from pathlib import Path
from typing import Dict, List, Optional

from proto_parser import ProtoFile, ProtoRpc, ProtoService, ProtoTypeIndex, parse_proto
from binding_contract import Interaction, interaction_for_service
from .naming import (
    _ada_req_type,
    _find_proto_root,
    _proto_type_to_ada,
    _service_ada_prefix,
    _short_type,
)


def _build_type_index(proto_path: Path) -> Optional[ProtoTypeIndex]:
    """Best-effort ProtoTypeIndex of *proto_path* plus its direct imports.

    Mirrors the loading done inline by naming.py's _collect_type_pkgs (same
    proto tree, same import-resolution shortcuts) rather than introducing a
    second convention for it.
    """
    try:
        svc_pf = parse_proto(proto_path)
        loaded = [svc_pf]
        proto_root = _find_proto_root(proto_path)
        if proto_root is not None:
            for imp in svc_pf.imports:
                if imp.startswith('google/'):
                    continue
                imp_path = proto_root / imp
                if imp_path.exists():
                    try:
                        loaded.append(parse_proto(imp_path))
                    except Exception:
                        pass
        return ProtoTypeIndex(loaded)
    except Exception:
        return None


def _request_shape_interactions(pf: Path, parsed: ProtoFile) -> List[Interaction]:
    """Request-shape `Interaction`s declared by *parsed* (Phase 0/1 model)."""
    index = _build_type_index(pf)
    if index is None:
        return []
    out: List[Interaction] = []
    for svc in parsed.services:
        try:
            ia = interaction_for_service(index, parsed, svc)
        except Exception:
            ia = None
        if ia is not None and ia.port_kind == 'request':
            out.append(ia)
    return out


class InteractionFacadeSpecMixin:
    """Appends the Phase 4 spec-only interaction facade to a service's
    generated .ads/.adb.

    Scoped to Request-shape services (Create/Update/Cancel command legs plus
    a Read/requirement leg) -- the same scope the plan's Phase 2/3 C++
    client/provider facades cover. Information-shape ports already have a
    working pub/sub Subscribe_*/Publish_* surface from the pre-existing
    generator; adding an RPC-streaming alternative for them is not part of
    this phase's scope.
    """

    def _write_interaction_facade_spec(self, f, pf: Path, parsed: ProtoFile) -> None:
        interactions = _request_shape_interactions(pf, parsed)
        if not interactions:
            return

        rpcs_by_service: Dict[str, Dict[str, ProtoRpc]] = {
            svc.name: {rpc.name: rpc for rpc in svc.rpcs} for svc in parsed.services
        }

        f.write('   --  -- Interaction facade (D1: interchangeable RPC/pub-sub binding) --\n')
        f.write('   --  Phase 4 spec-only fallback -- see\n')
        f.write('   --  doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md Phase 4.\n')
        f.write('   --  Declares the same client/provider-facing surface as the C++\n')
        f.write('   --  runtime facade (Submit/Transitions/Configure_Interaction_Binding);\n')
        f.write('   --  bodies raise Program_Error pending dynamic-dispatch runtime support.\n')
        f.write('\n')

        for ia in interactions:
            prefix = _service_ada_prefix(ia.service_name)
            rpcs = rpcs_by_service.get(ia.service_name, {})

            f.write(f'   procedure {prefix}_Configure_Interaction_Binding\n')
            f.write(f'     (Config_Json : String);\n')
            f.write(f'   --  Runtime selection of RPC vs pub/sub realization per leg (D1).\n')
            f.write(f'   --  Config_Json keys: "request_leg", "requirement_leg" -> "rpc" | "pubsub".\n')
            f.write('\n')

            request_leg = next((leg for leg in ia.legs if leg.name == 'request'), None)
            if request_leg is not None:
                for ep in request_leg.side_a:
                    rpc = rpcs.get(ep.rpc_name)
                    if rpc is None:
                        continue
                    req_t = _ada_req_type(rpc)
                    f.write(f'   procedure {prefix}_Submit_{ep.rpc_name}\n')
                    f.write(f'     (Request         : {req_t};\n')
                    f.write(f'      Result_Accepted : out Boolean;\n')
                    f.write(f'      Result_Status    : out Interfaces.C.int;\n')
                    f.write(f'      Result_Has_Ack   : out Boolean;\n')
                    f.write(f'      Result_Ack       : out Ack);\n')
                    f.write(f'   --  D3: Result_Has_Ack is only ever True under RPC realization --\n')
                    f.write(f'   --  no ack is synthesized under pub/sub, in either direction.\n')
                    f.write('\n')

            requirement_leg = next((leg for leg in ia.legs if leg.name == 'requirement'), None)
            if requirement_leg is not None and requirement_leg.side_a:
                read_rpc = rpcs.get(requirement_leg.side_a[0].rpc_name)
                if read_rpc is not None:
                    item_t = _proto_type_to_ada(_short_type(read_rpc.response_type))
                    f.write(f'   type {prefix}_Transition_Callback is\n')
                    f.write(f'     access procedure (Item : {item_t});\n')
                    f.write('\n')
                    f.write(f'   procedure {prefix}_Transitions\n')
                    f.write(f'     (Filter   : {_ada_req_type(read_rpc)};\n')
                    f.write(f'      Callback : {prefix}_Transition_Callback);\n')
                    f.write(f'   --  D4: pub/sub realization only observes future transitions;\n')
                    f.write(f'   --  a late-joining Query for an id already at rest re-triggers via\n')
                    f.write(f'   --  the provider''s snapshot store (D6), not a topic replay.\n')
                    f.write('\n')

    def _write_interaction_facade_body(self, f, pf: Path, parsed: ProtoFile) -> None:
        interactions = _request_shape_interactions(pf, parsed)
        if not interactions:
            return

        rpcs_by_service: Dict[str, Dict[str, ProtoRpc]] = {
            svc.name: {rpc.name: rpc for rpc in svc.rpcs} for svc in parsed.services
        }

        f.write('   --  -- Interaction facade (Phase 4 spec-only fallback) ---------------\n')
        f.write('\n')

        for ia in interactions:
            prefix = _service_ada_prefix(ia.service_name)
            rpcs = rpcs_by_service.get(ia.service_name, {})

            f.write(f'   procedure {prefix}_Configure_Interaction_Binding\n')
            f.write(f'     (Config_Json : String)\n')
            f.write(f'   is\n')
            f.write(f'      pragma Unreferenced (Config_Json);\n')
            f.write(f'   begin\n')
            f.write(f'      raise Program_Error with\n')
            f.write(f'        "{prefix}_Configure_Interaction_Binding: Phase 4 spec-only ' +
                     'fallback -- runtime dynamic dispatch not yet implemented (see ' +
                     'doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md Phase 4)";\n')
            f.write(f'   end {prefix}_Configure_Interaction_Binding;\n')
            f.write('\n')

            request_leg = next((leg for leg in ia.legs if leg.name == 'request'), None)
            if request_leg is not None:
                for ep in request_leg.side_a:
                    rpc = rpcs.get(ep.rpc_name)
                    if rpc is None:
                        continue
                    req_t = _ada_req_type(rpc)
                    f.write(f'   procedure {prefix}_Submit_{ep.rpc_name}\n')
                    f.write(f'     (Request         : {req_t};\n')
                    f.write(f'      Result_Accepted : out Boolean;\n')
                    f.write(f'      Result_Status    : out Interfaces.C.int;\n')
                    f.write(f'      Result_Has_Ack   : out Boolean;\n')
                    f.write(f'      Result_Ack       : out Ack)\n')
                    f.write(f'   is\n')
                    f.write(f'      pragma Unreferenced (Request);\n')
                    f.write(f'   begin\n')
                    f.write(f'      raise Program_Error with\n')
                    f.write(f'        "{prefix}_Submit_{ep.rpc_name}: Phase 4 spec-only ' +
                             'fallback -- runtime dynamic dispatch not yet implemented (see ' +
                             'doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md Phase 4)";\n')
                    f.write(f'   end {prefix}_Submit_{ep.rpc_name};\n')
                    f.write('\n')

            requirement_leg = next((leg for leg in ia.legs if leg.name == 'requirement'), None)
            if requirement_leg is not None and requirement_leg.side_a:
                read_rpc = rpcs.get(requirement_leg.side_a[0].rpc_name)
                if read_rpc is not None:
                    f.write(f'   procedure {prefix}_Transitions\n')
                    f.write(f'     (Filter   : {_ada_req_type(read_rpc)};\n')
                    f.write(f'      Callback : {prefix}_Transition_Callback)\n')
                    f.write(f'   is\n')
                    f.write(f'      pragma Unreferenced (Filter, Callback);\n')
                    f.write(f'   begin\n')
                    f.write(f'      raise Program_Error with\n')
                    f.write(f'        "{prefix}_Transitions: Phase 4 spec-only fallback -- ' +
                             'runtime dynamic dispatch not yet implemented (see ' +
                             'doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md Phase 4)";\n')
                    f.write(f'   end {prefix}_Transitions;\n')
                    f.write('\n')
