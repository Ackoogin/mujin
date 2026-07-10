#!/usr/bin/env python3
"""Ada interaction facade generator -- runtime dispatch (F1).

Emits a real client (`RequestPortClient`-equivalent: `Submit_*`/
`Transitions`) and a real provider (`RequestPortProvider`-equivalent:
`Interaction_Handlers`/`Provider_Bind`/`Send_Transition`) for every
Request-shape service, mirroring the C++ Phase 2/3 runtime facade
(`subprojects/PYRAMID/pim/cpp/interaction_facade_gen.py`) as closely as
Ada's binding model allows:

- RPC realization dispatches through the existing per-RPC `Invoke_*`
  procedures on the client, and through dedicated `Add_Service`/
  `Add_Stream_Service` PCL ports on the provider (a live PCL stream for
  the requirement leg, not a poll -- see `Pcl_Bindings.Invoke_Stream`/
  `Add_Stream_Service`/`Stream_Send`).
- Pub/sub realization dispatches through `Pcl_Bindings.Add_Publisher`/
  `Add_Subscriber`/`Port_Publish` directly (there is no pre-existing
  generated `Publish_*`/`Subscribe_*` wrapper layer for Request-shape
  contract-derived topics in Ada, unlike the base "standard topics"
  system -- this module builds its own, self-contained, exactly as the
  C++ facade builds its own runtime machinery independent of
  `ProvidedService`/`ConsumedService`).
- D1 (configurable realization), D2 (pub/sub fails closed for
  non-projectable commands), D3 (no synthetic ack under pub/sub), and a
  bounded D4 snapshot store (late-joining RPC Read streams get replayed
  the current snapshot at open time) are implemented. The narrower D4
  case -- an *already-open* RPC stream missing a state change because a
  late pub/sub-realized command doesn't independently trigger a re-send
  -- is not (documented gap, not a silently-wrong behaviour): C++'s
  `republishSnapshotFor()` on the inbound-command path has no Ada
  equivalent here.

All state is package-body-private and singleton per generated package,
matching the existing flat/procedural Ada binding style (no tagged types,
no per-instance closures): there is at most one live interaction binding
per generated service per process, configured via `Client_Bind`/
`Provider_Bind` called once from the owning component's `on_configure()`.

Reuses `binding_contract.interaction_for_service`/
`command_projectability_for_service` (the same shared model the C++
facade is built from) rather than re-deriving a parallel one.
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from proto_parser import ProtoFile, ProtoRpc, ProtoService, ProtoTypeIndex, parse_proto
from binding_contract import (
    CommandProjectability,
    Interaction,
    command_projectability_for_service,
    interaction_for_service,
)
from .naming import (
    _ada_field_name,
    _ada_req_type,
    _find_proto_root,
    _proto_type_to_ada,
    _rpc_ada_svc_const,
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


def _fqn(index: ProtoTypeIndex, type_name: str, current_package: str) -> str:
    """Minimal local re-implementation of binding_contract._fully_qualified_type
    (kept local rather than importing a private helper across modules; same
    approach pim/cpp/interaction_facade_gen.py takes)."""
    if not type_name or type_name.startswith('google.'):
        return ''
    if '.' in type_name:
        return type_name
    fqn = f'{current_package}.{type_name}' if current_package else type_name
    if index.resolve_message(fqn) is not None:
        return fqn
    if index.resolve_message(type_name) is not None:
        return type_name
    return ''


def _requirement_correlation_field(
    index: ProtoTypeIndex, pf: ProtoFile, read_rpc: ProtoRpc,
) -> Optional[str]:
    """Ada field name of the Read stream frame's single-oneof-single-variant
    wrapping a message with an `Id` field (D4's client/provider-side
    correlation-id filter) -- Ada port of the C++ facade's
    `_requirement_correlation_field`. Only handles the single-oneof shape
    every Requirement wrapper in the tree uses today (e.g. A-GRA's
    `MAAction_Service_Requirement.ma_action_status`); returns `None` (a
    documented gap, not a silently-wrong filter) otherwise, in which case
    generated code treats every frame as matching every filter.
    """
    frame_fqn = _fqn(index, read_rpc.response_type, pf.package)
    frame_msg = index.resolve_message(frame_fqn) if frame_fqn else None
    if frame_msg is None:
        return None
    if len(frame_msg.oneofs) != 1 or len(frame_msg.oneofs[0].fields) != 1:
        return None
    variant_field = frame_msg.oneofs[0].fields[0]
    variant_fqn = _fqn(index, variant_field.type, pf.package)
    variant_msg = index.resolve_message(variant_fqn) if variant_fqn else None
    if variant_msg is None:
        return None
    if not any(fld.name == 'id' for fld in variant_msg.fields):
        return None
    return _ada_field_name(variant_field.name)


def _ordered_command_projectability(
    index: ProtoTypeIndex, pf: ProtoFile, svc: ProtoService,
) -> Tuple[List[CommandProjectability], Optional[CommandProjectability]]:
    """`(matches_variant entries, the single is_wrapper entry or None)`,
    matching C++'s dispatch order for inbound pub/sub commands: variant
    checks first, the wrapper-is-the-payload case as the final fallback
    (`_write_dispatch_pubsub_command`'s ordering)."""
    cps = command_projectability_for_service(index, pf, svc)
    variants = [cp for cp in cps if cp.projectable and cp.reason == 'matches_variant']
    wrapper = next((cp for cp in cps if cp.projectable and cp.reason == 'is_wrapper'), None)
    return variants, wrapper


class InteractionFacadeSpecMixin:
    """Appends the runtime interaction facade to a service's generated
    .ads/.adb: a client surface (`Submit_*`/`Transitions`) on the consumed
    side, a provider surface (`Interaction_Handlers`/`Provider_Bind`/
    `Send_Transition`) on the provided side.

    Scoped to Request-shape services (Create/Update/Cancel command legs plus
    a Read/requirement leg) -- the same scope the C++ facade covers.
    Information-shape ports already have a working pub/sub
    Subscribe_*/Publish_* surface from the pre-existing generator; adding an
    RPC-streaming alternative for them is not part of this scope.
    """

    def _write_interaction_facade_spec(
        self, f, pf: Path, parsed: ProtoFile, is_provided: bool,
    ) -> None:
        interactions = _request_shape_interactions(pf, parsed)
        if not interactions:
            return
        index = _build_type_index(pf)
        if index is None:
            return

        rpcs_by_service: Dict[str, Dict[str, ProtoRpc]] = {
            svc.name: {rpc.name: rpc for rpc in svc.rpcs} for svc in parsed.services
        }
        svcs_by_name: Dict[str, ProtoService] = {svc.name: svc for svc in parsed.services}

        f.write('   --  -- Interaction facade (D1: interchangeable RPC/pub-sub binding) --\n')
        f.write('   --  Runtime dispatch -- see subprojects/PYRAMID/pim/ada/\n')
        f.write('   --  interaction_facade_gen.py and doc/plans/PYRAMID/\n')
        f.write('   --  rpc_pubsub_interchangeability_plan.md.\n')
        f.write('\n')

        for ia in interactions:
            prefix = _service_ada_prefix(ia.service_name)
            rpcs = rpcs_by_service.get(ia.service_name, {})
            svc = svcs_by_name.get(ia.service_name)
            request_leg = next((leg for leg in ia.legs if leg.name == 'request'), None)
            requirement_leg = next((leg for leg in ia.legs if leg.name == 'requirement'), None)
            read_rpc = None
            if requirement_leg is not None and requirement_leg.side_a:
                read_rpc = rpcs.get(requirement_leg.side_a[0].rpc_name)
            frame_t = _proto_type_to_ada(_short_type(read_rpc.response_type)) if read_rpc else None

            f.write(f'   type {prefix}_Interaction_Binding is\n')
            f.write(f'     ({prefix}_Binding_Rpc, {prefix}_Binding_Pubsub);\n')
            f.write('\n')

            if frame_t is not None:
                f.write(f'   type {prefix}_Transition_Callback is\n')
                f.write(f'     access procedure (Item : {frame_t});\n')
                f.write('\n')

            if not is_provided:
                f.write(f'   procedure {prefix}_Client_Bind\n')
                f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
                f.write(f'      Executor  : Pcl_Bindings.Pcl_Executor_Access);\n')
                f.write(f'   --  Bind the pub/sub-realization ports (request-leg publisher,\n')
                f.write(f'   --  requirement-leg subscriber) and remember Executor for the\n')
                f.write(f'   --  RPC-realization Invoke_*/Invoke_Stream calls. Call once from\n')
                f.write(f'   --  on_configure(); harmless if a binding later selects "rpc" for\n')
                f.write(f'   --  both legs -- the bound pub/sub ports then simply sit idle.\n')
                f.write('\n')

                f.write(f'   procedure {prefix}_Configure_Interaction_Binding\n')
                f.write(f'     (Config_Json : String);\n')
                f.write(f'   --  Runtime selection of RPC vs pub/sub realization per leg (D1).\n')
                f.write(f'   --  Config_Json keys: "binding" (both legs), "request_leg",\n')
                f.write(f'   --  "requirement_leg" -> "rpc" | "pubsub". Unset legs fall back to\n')
                f.write(f'   --  "binding"; unset "binding" defaults to "rpc".\n')
                f.write('\n')

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

                if read_rpc is not None:
                    f.write(f'   procedure {prefix}_Transitions\n')
                    f.write(f'     (Filter   : {_ada_req_type(read_rpc)};\n')
                    f.write(f'      Callback : {prefix}_Transition_Callback);\n')
                    f.write(f'   --  D4: a late-joining RPC Read stream replays the provider\'s\n')
                    f.write(f'   --  current snapshot for matching ids at open time; a late-joining\n')
                    f.write(f'   --  pub/sub subscription only observes future publications.\n')
                    f.write('\n')

            else:
                if request_leg is not None and svc is not None:
                    variants, wrapper = _ordered_command_projectability(index, parsed, svc)
                    for cp in variants + ([wrapper] if wrapper else []):
                        rpc = rpcs.get(cp.rpc_name)
                        if rpc is None:
                            continue
                        req_t = _ada_req_type(rpc)
                        f.write(f'   type {prefix}_On_{cp.rpc_name}_Access is\n')
                        f.write(f'     access function (Request : {req_t}) return Ack;\n')
                        f.write('\n')

                    f.write(f'   type {prefix}_Interaction_Handlers is record\n')
                    for cp in variants + ([wrapper] if wrapper else []):
                        if cp.rpc_name not in rpcs:
                            continue
                        f.write(f'      On_{cp.rpc_name} : {prefix}_On_{cp.rpc_name}_Access := null;\n')
                    f.write(f'   end record;\n')
                    f.write('\n')

                    f.write(f'   procedure {prefix}_Configure_Interaction_Binding\n')
                    f.write(f'     (Config_Json : String);\n')
                    f.write(f'   --  Selects the requirement leg\'s emission realization (D1); the\n')
                    f.write(f'   --  request leg is always dual-bound and listening on both RPC and\n')
                    f.write(f'   --  pub/sub -- no per-leg choice on the provider side (matches the\n')
                    f.write(f'   --  C++ facade\'s RequestPortProvider::configureInteractionBinding()).\n')
                    f.write(f'   --  Config_Json keys: "requirement_leg" or "binding" -> "rpc" | "pubsub".\n')
                    f.write('\n')

                    f.write(f'   procedure {prefix}_Provider_Bind\n')
                    f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
                    f.write(f'      Executor  : Pcl_Bindings.Pcl_Executor_Access;\n')
                    f.write(f'      Handlers  : {prefix}_Interaction_Handlers);\n')
                    f.write(f'   --  Binds unary Create/Update/Cancel RPC ports, a streaming Read\n')
                    f.write(f'   --  RPC port, and the pub/sub request/requirement probe ports.\n')
                    f.write(f'   --  Call once from on_configure().\n')
                    f.write('\n')

                    if frame_t is not None:
                        f.write(f'   procedure {prefix}_Send_Transition (Transition : {frame_t});\n')
                        f.write(f'   --  D6: the provider\'s single way to emit a transition. Fans out\n')
                        f.write(f'   --  to every open Read stream matching the transition\'s\n')
                        f.write(f'   --  correlation id (RPC realization) or publishes on the\n')
                        f.write(f'   --  requirement topic (pub/sub realization), per\n')
                        f.write(f'   --  {prefix}_Configure_Interaction_Binding. Either way, records a\n')
                        f.write(f'   --  bounded per-id snapshot new RPC Read streams replay at open\n')
                        f.write(f'   --  time (D4).\n')
                        f.write('\n')

    def _write_interaction_facade_body(
        self, f, pf: Path, parsed: ProtoFile, is_provided: bool,
    ) -> None:
        interactions = _request_shape_interactions(pf, parsed)
        if not interactions:
            return
        index = _build_type_index(pf)
        if index is None:
            return

        rpcs_by_service: Dict[str, Dict[str, ProtoRpc]] = {
            svc.name: {rpc.name: rpc for rpc in svc.rpcs} for svc in parsed.services
        }
        svcs_by_name: Dict[str, ProtoService] = {svc.name: svc for svc in parsed.services}

        f.write('   --  -- Interaction facade -- runtime dispatch (F1) --------------------\n')
        f.write('\n')

        for ia in interactions:
            prefix = _service_ada_prefix(ia.service_name)
            rpcs = rpcs_by_service.get(ia.service_name, {})
            svc = svcs_by_name.get(ia.service_name)
            request_leg = next((leg for leg in ia.legs if leg.name == 'request'), None)
            requirement_leg = next((leg for leg in ia.legs if leg.name == 'requirement'), None)
            read_rpc = None
            if requirement_leg is not None and requirement_leg.side_a:
                read_rpc = rpcs.get(requirement_leg.side_a[0].rpc_name)
            frame_t = _proto_type_to_ada(_short_type(read_rpc.response_type)) if read_rpc else None
            frame_schema = _short_type(read_rpc.response_type) if read_rpc else None
            corr_field = (
                _requirement_correlation_field(index, parsed, read_rpc)
                if read_rpc is not None else None
            )
            request_wire = request_leg.side_b[0].endpoint_name if request_leg else None
            requirement_wire = requirement_leg.side_b[0].endpoint_name if requirement_leg else None

            if not is_provided:
                self._write_client_facade_body(
                    f, prefix, ia, index, parsed, svc, rpcs, read_rpc, frame_t,
                    frame_schema, corr_field, request_wire, requirement_wire,
                )
            else:
                self._write_provider_facade_body(
                    f, prefix, ia, index, parsed, svc, rpcs, read_rpc, frame_t,
                    frame_schema, corr_field, request_wire, requirement_wire,
                )

    # -- Client (consumed-role) body -----------------------------------

    def _write_local_endpoint_route(self, f, svc_const, kind) -> None:
        # F1's current scope: local-executor dispatch only -- the same
        # single-process proof C++'s Phase 2 facade tests cover (not the
        # cross-process real-transport proof agra_seam_interchange_test.cpp
        # adds in a later phase). Remote routing would need a Config_Json
        # parameter threaded through Bind, a mechanical follow-up.
        f.write(f'      declare\n')
        f.write(f'         Endpoint_C : Interfaces.C.Strings.chars_ptr :=\n')
        f.write(f'           Interfaces.C.Strings.New_String ({svc_const});\n')
        f.write(f'         Route : aliased Pcl_Bindings.Pcl_Endpoint_Route :=\n')
        f.write(f'           (Endpoint_Name => Endpoint_C,\n')
        f.write(f'            Endpoint_Kind => Pcl_Bindings.{kind},\n')
        f.write(f'            Route_Mode    => Pcl_Bindings.PCL_ROUTE_LOCAL,\n')
        f.write(f'            Peer_Ids      => System.Null_Address,\n')
        f.write(f'            Peer_Count    => 0);\n')
        f.write(f'      begin\n')
        f.write(f'         Route_Status := Pcl_Bindings.Set_Endpoint_Route (Executor, Route\'Access);\n')
        f.write(f'         Interfaces.C.Strings.Free (Endpoint_C);\n')
        f.write(f'      end;\n')

    def _write_client_facade_body(
        self, f, prefix, ia, index, parsed, svc, rpcs, read_rpc, frame_t, frame_schema,
        corr_field, request_wire, requirement_wire,
    ) -> None:
        request_leg = next((leg for leg in ia.legs if leg.name == 'request'), None)
        cp_by_rpc: Dict[str, CommandProjectability] = {}
        if svc is not None:
            variants, wrapper = _ordered_command_projectability(index, parsed, svc)
            for cp in variants + ([wrapper] if wrapper else []):
                cp_by_rpc[cp.rpc_name] = cp

        f.write(f'   {prefix}_Request_Binding     : {prefix}_Interaction_Binding :=\n')
        f.write(f'     {prefix}_Binding_Rpc;\n')
        f.write(f'   {prefix}_Requirement_Binding : {prefix}_Interaction_Binding :=\n')
        f.write(f'     {prefix}_Binding_Rpc;\n')
        f.write(f'   {prefix}_Client_Executor   : Pcl_Bindings.Pcl_Executor_Access := null;\n')
        f.write(f'   {prefix}_Request_Publisher : Pcl_Bindings.Pcl_Port_Access := null;\n')
        f.write('\n')

        f.write(f'   {prefix}_Rpc_Reply_Ready    : Boolean := False;\n')
        f.write(f'   {prefix}_Rpc_Reply_Status   : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_STATE;\n')
        f.write(f'   {prefix}_Rpc_Reply_Has_Ack  : Boolean := False;\n')
        f.write(f'   {prefix}_Rpc_Reply_Ack      : Ack;\n')
        f.write('\n')

        # Forward declarations (+ pragma Convention) for the C-convention
        # callbacks below -- Ada requires the convention pragma to precede
        # the body, via a separate spec (see Service_MA_Action_Create's
        # existing forward-decl pattern earlier in this file).
        f.write(f'   procedure {prefix}_On_Rpc_Reply\n')
        f.write(f'     (Resp : access constant Pcl_Bindings.Pcl_Msg; User_Data : System.Address);\n')
        f.write(f'   pragma Convention (C, {prefix}_On_Rpc_Reply);\n')
        f.write('\n')
        if frame_t is not None:
            f.write(f'   procedure {prefix}_On_Requirement_Message\n')
            f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Msg       : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      User_Data : System.Address);\n')
            f.write(f'   pragma Convention (C, {prefix}_On_Requirement_Message);\n')
            f.write('\n')
            f.write(f'   procedure {prefix}_On_Stream_Msg\n')
            f.write(f'     (Msg           : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      End_Of_Stream : Pcl_Bindings.C_Bool;\n')
            f.write(f'      Status        : Pcl_Bindings.Pcl_Status;\n')
            f.write(f'      User_Data     : System.Address);\n')
            f.write(f'   pragma Convention (C, {prefix}_On_Stream_Msg);\n')
            f.write('\n')

        f.write(f'   function {prefix}_Parse_Binding (Value : String) return {prefix}_Interaction_Binding is\n')
        f.write(f'     (if Value = "pubsub" then {prefix}_Binding_Pubsub else {prefix}_Binding_Rpc);\n')
        f.write('\n')

        f.write(f'   function {prefix}_Id_Matches\n')
        f.write(f'     (Ids   : Pyramid.Data_Model.Common.Types.Identifier_Array_Acc;\n')
        f.write(f'      Value : Unbounded_String) return Boolean\n')
        f.write(f'   is\n')
        f.write(f'   begin\n')
        f.write(f'      if Ids = null or else Ids\'Length = 0 then\n')
        f.write(f'         return True;\n')
        f.write(f'      end if;\n')
        f.write(f'      for I in Ids\'Range loop\n')
        f.write(f'         if Ids (I) = Value then\n')
        f.write(f'            return True;\n')
        f.write(f'         end if;\n')
        f.write(f'      end loop;\n')
        f.write(f'      return False;\n')
        f.write(f'   end {prefix}_Id_Matches;\n')
        f.write('\n')

        f.write(f'   procedure {prefix}_On_Rpc_Reply\n')
        f.write(f'     (Resp : access constant Pcl_Bindings.Pcl_Msg; User_Data : System.Address)\n')
        f.write(f'   is\n')
        f.write(f'      pragma Unreferenced (User_Data);\n')
        f.write(f'   begin\n')
        f.write(f'      {prefix}_Rpc_Reply_Ready := True;\n')
        f.write(f'      if Resp = null or else Resp.Size = 0 then\n')
        f.write(f'         {prefix}_Rpc_Reply_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;\n')
        f.write(f'         {prefix}_Rpc_Reply_Has_Ack := False;\n')
        f.write(f'         return;\n')
        f.write(f'      end if;\n')
        f.write(f'      {prefix}_Rpc_Reply_Status := Pcl_Bindings.PCL_OK;\n')
        f.write(f'      {prefix}_Rpc_Reply_Has_Ack :=\n')
        f.write(f'        Try_Registry_Decode (Resp, "Ack", {prefix}_Rpc_Reply_Ack\'Address);\n')
        f.write(f'   end {prefix}_On_Rpc_Reply;\n')
        f.write('\n')

        if frame_t is not None:
            f.write(f'   {prefix}_Max_Transition_Subscriptions : constant := 8;\n')
            f.write(f'   type {prefix}_Transition_Slot is record\n')
            f.write(f'      Active   : Boolean := False;\n')
            f.write(f'      One_Shot : Boolean := False;\n')
            f.write(f'      Ids      : Pyramid.Data_Model.Common.Types.Identifier_Array_Acc := null;\n')
            f.write(f'      Callback : {prefix}_Transition_Callback := null;\n')
            f.write(f'   end record;\n')
            f.write(f'   {prefix}_Transition_Slots :\n')
            f.write(f'     array (1 .. {prefix}_Max_Transition_Subscriptions) of {prefix}_Transition_Slot;\n')
            f.write('\n')

            f.write(f'   procedure {prefix}_On_Requirement_Message\n')
            f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Msg       : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      User_Data : System.Address)\n')
            f.write(f'   is\n')
            f.write(f'      pragma Unreferenced (Self, User_Data);\n')
            f.write(f'      Frame : {frame_t};\n')
            f.write(f'   begin\n')
            f.write(f'      if not Try_Registry_Decode (Msg, "{frame_schema}", Frame\'Address) then\n')
            f.write(f'         return;\n')
            f.write(f'      end if;\n')
            if corr_field is not None:
                f.write(f'      declare\n')
                f.write(f'         Has_Id   : constant Boolean := Frame.Has_{corr_field};\n')
                f.write(f'         Frame_Id : Unbounded_String := Null_Unbounded_String;\n')
                f.write(f'      begin\n')
                f.write(f'         if Has_Id then\n')
                f.write(f'            Frame_Id := Frame.{corr_field}.Id;\n')
                f.write(f'         end if;\n')
                f.write(f'         for I in {prefix}_Transition_Slots\'Range loop\n')
                f.write(f'            if {prefix}_Transition_Slots (I).Active\n')
                f.write(f'              and then (not Has_Id or else {prefix}_Id_Matches\n')
                f.write(f'                          ({prefix}_Transition_Slots (I).Ids, Frame_Id))\n')
                f.write(f'            then\n')
                f.write(f'               if {prefix}_Transition_Slots (I).Callback /= null then\n')
                f.write(f'                  {prefix}_Transition_Slots (I).Callback (Frame);\n')
                f.write(f'               end if;\n')
                f.write(f'               if {prefix}_Transition_Slots (I).One_Shot then\n')
                f.write(f'                  {prefix}_Transition_Slots (I).Active := False;\n')
                f.write(f'               end if;\n')
                f.write(f'            end if;\n')
                f.write(f'         end loop;\n')
                f.write(f'      end;\n')
            else:
                f.write(f'      --  D4: no resolvable correlation field on this frame type --\n')
                f.write(f'      --  documented gap (see _requirement_correlation_field); every\n')
                f.write(f'      --  active subscription is treated as accept-all.\n')
                f.write(f'      for I in {prefix}_Transition_Slots\'Range loop\n')
                f.write(f'         if {prefix}_Transition_Slots (I).Active then\n')
                f.write(f'            if {prefix}_Transition_Slots (I).Callback /= null then\n')
                f.write(f'               {prefix}_Transition_Slots (I).Callback (Frame);\n')
                f.write(f'            end if;\n')
                f.write(f'            if {prefix}_Transition_Slots (I).One_Shot then\n')
                f.write(f'               {prefix}_Transition_Slots (I).Active := False;\n')
                f.write(f'            end if;\n')
                f.write(f'         end if;\n')
                f.write(f'      end loop;\n')
            f.write(f'   end {prefix}_On_Requirement_Message;\n')
            f.write('\n')

            f.write(f'   {prefix}_Max_Rpc_Streams : constant := 8;\n')
            f.write(f'   type {prefix}_Rpc_Stream_Slot is record\n')
            f.write(f'      Active   : Boolean := False;\n')
            f.write(f'      Callback : {prefix}_Transition_Callback := null;\n')
            f.write(f'   end record;\n')
            f.write(f'   {prefix}_Rpc_Stream_Slots :\n')
            f.write(f'     array (1 .. {prefix}_Max_Rpc_Streams) of {prefix}_Rpc_Stream_Slot;\n')
            f.write('\n')

            f.write(f'   function {prefix}_Slot_Address (Index : Positive) return System.Address is\n')
            f.write(f'     (System.Storage_Elements.To_Address\n')
            f.write(f'        (System.Storage_Elements.Integer_Address (Index)));\n')
            f.write('\n')
            f.write(f'   function {prefix}_Address_To_Slot (Addr : System.Address) return Positive is\n')
            f.write(f'     (Positive (System.Storage_Elements.To_Integer (Addr)));\n')
            f.write('\n')

            f.write(f'   procedure {prefix}_On_Stream_Msg\n')
            f.write(f'     (Msg           : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      End_Of_Stream : Pcl_Bindings.C_Bool;\n')
            f.write(f'      Status        : Pcl_Bindings.Pcl_Status;\n')
            f.write(f'      User_Data     : System.Address)\n')
            f.write(f'   is\n')
            f.write(f'      pragma Unreferenced (Status);\n')
            f.write(f'      Index : constant Positive := {prefix}_Address_To_Slot (User_Data);\n')
            f.write(f'   begin\n')
            f.write(f'      if Msg /= null and then Msg.Size > 0\n')
            f.write(f'        and then {prefix}_Rpc_Stream_Slots (Index).Callback /= null\n')
            f.write(f'      then\n')
            f.write(f'         declare\n')
            f.write(f'            Frame : {frame_t};\n')
            f.write(f'         begin\n')
            f.write(f'            if Try_Registry_Decode (Msg, "{frame_schema}", Frame\'Address) then\n')
            f.write(f'               {prefix}_Rpc_Stream_Slots (Index).Callback (Frame);\n')
            f.write(f'            end if;\n')
            f.write(f'         end;\n')
            f.write(f'      end if;\n')
            f.write(f'      if Boolean (End_Of_Stream) then\n')
            f.write(f'         {prefix}_Rpc_Stream_Slots (Index).Active := False;\n')
            f.write(f'      end if;\n')
            f.write(f'   end {prefix}_On_Stream_Msg;\n')
            f.write('\n')

        f.write(f'   procedure {prefix}_Client_Bind\n')
        f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
        f.write(f'      Executor  : Pcl_Bindings.Pcl_Executor_Access)\n')
        f.write(f'   is\n')
        f.write(f'      Port : Pcl_Bindings.Pcl_Port_Access;\n')
        f.write(f'      Route_Status : Pcl_Bindings.Pcl_Status;\n')
        f.write(f'      pragma Unreferenced (Port, Route_Status);\n')
        f.write(f'   begin\n')
        f.write(f'      {prefix}_Client_Executor := Executor;\n')
        if request_leg is not None:
            for ep in request_leg.side_a:
                rpc = rpcs.get(ep.rpc_name)
                if rpc is None:
                    continue
                svc_const = _rpc_ada_svc_const(ia.service_name, rpc, set())
                self._write_local_endpoint_route(f, svc_const, 'PCL_ENDPOINT_CONSUMED')
        if read_rpc is not None and frame_t is not None:
            read_svc_const = _rpc_ada_svc_const(ia.service_name, read_rpc, set())
            self._write_local_endpoint_route(f, read_svc_const, 'PCL_ENDPOINT_STREAM_CONSUMED')
        if request_wire is not None:
            f.write(f'      declare\n')
            f.write(f'         Topic_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String ("{request_wire}");\n')
            f.write(f'         Type_C  : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String (Json_Content_Type);\n')
            f.write(f'      begin\n')
            f.write(f'         {prefix}_Request_Publisher :=\n')
            f.write(f'           Pcl_Bindings.Add_Publisher (Container, Topic_C, Type_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Topic_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         Route_Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           ({prefix}_Request_Publisher, Pcl_Bindings.PCL_ROUTE_LOCAL,\n')
            f.write(f'            System.Null_Address, 0);\n')
            f.write(f'      end;\n')
        if requirement_wire is not None and frame_t is not None:
            f.write(f'      declare\n')
            f.write(f'         Topic_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String ("{requirement_wire}");\n')
            f.write(f'         Type_C  : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String (Json_Content_Type);\n')
            f.write(f'      begin\n')
            f.write(f'         Port := Pcl_Bindings.Add_Subscriber\n')
            f.write(f'           (Container, Topic_C, Type_C,\n')
            f.write(f'            {prefix}_On_Requirement_Message\'Access, System.Null_Address);\n')
            f.write(f'         Interfaces.C.Strings.Free (Topic_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         Route_Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           (Port, Pcl_Bindings.PCL_ROUTE_LOCAL, System.Null_Address, 0);\n')
            f.write(f'      end;\n')
        f.write(f'   end {prefix}_Client_Bind;\n')
        f.write('\n')

        f.write(f'   procedure {prefix}_Configure_Interaction_Binding (Config_Json : String) is\n')
        f.write(f'      Binding_Value     : constant String := Config_Value (Config_Json, "binding");\n')
        f.write(f'      Request_Value     : constant String := Config_Value (Config_Json, "request_leg");\n')
        f.write(f'      Requirement_Value : constant String :=\n')
        f.write(f'        Config_Value (Config_Json, "requirement_leg");\n')
        f.write(f'   begin\n')
        f.write(f'      {prefix}_Request_Binding := {prefix}_Parse_Binding\n')
        f.write(f'        (if Request_Value\'Length /= 0 then Request_Value else Binding_Value);\n')
        f.write(f'      {prefix}_Requirement_Binding := {prefix}_Parse_Binding\n')
        f.write(f'        (if Requirement_Value\'Length /= 0 then Requirement_Value else Binding_Value);\n')
        f.write(f'   end {prefix}_Configure_Interaction_Binding;\n')
        f.write('\n')

        if request_leg is not None:
            for ep in request_leg.side_a:
                rpc = rpcs.get(ep.rpc_name)
                if rpc is None:
                    continue
                cp = cp_by_rpc.get(ep.rpc_name)
                self._write_client_submit_body(f, prefix, ia, rpc, ep, cp)

        if read_rpc is not None and frame_t is not None:
            self._write_client_transitions_body(
                f, prefix, ia, read_rpc, frame_t, frame_schema,
            )

    def _write_client_submit_body(self, f, prefix, ia, rpc, ep, cp) -> None:
        req_t = _ada_req_type(rpc)
        req_schema = _short_type(rpc.request_type)
        svc_const = _rpc_ada_svc_const(ia.service_name, rpc, set())
        projectable = cp is not None and cp.projectable

        f.write(f'   procedure {prefix}_Submit_{ep.rpc_name}\n')
        f.write(f'     (Request         : {req_t};\n')
        f.write(f'      Result_Accepted : out Boolean;\n')
        f.write(f'      Result_Status    : out Interfaces.C.int;\n')
        f.write(f'      Result_Has_Ack   : out Boolean;\n')
        f.write(f'      Result_Ack       : out Ack)\n')
        f.write(f'   is\n')
        f.write(f'   begin\n')
        f.write(f'      Result_Ack := (Success => False, Identifier => Null_Unbounded_String);\n')
        f.write(f'      if {prefix}_Request_Binding = {prefix}_Binding_Rpc then\n')
        f.write(f'         {prefix}_Rpc_Reply_Ready := False;\n')
        f.write(f'         declare\n')
        f.write(f'            Payload : Unbounded_String;\n')
        f.write(f'            Encoded : constant Boolean :=\n')
        f.write(f'              Try_Registry_Encode\n')
        f.write(f'                (Json_Content_Type, "{req_schema}", Request\'Address, Payload);\n')
        f.write(f'         begin\n')
        f.write(f'            if Encoded then\n')
        f.write(f'               declare\n')
        f.write(f'                  Payload_Str : constant String := To_String (Payload);\n')
        f.write(f'                  Buf : Interfaces.C.Strings.chars_ptr :=\n')
        f.write(f'                    Interfaces.C.Strings.New_String (Payload_Str);\n')
        f.write(f'                  Svc_C : Interfaces.C.Strings.chars_ptr :=\n')
        f.write(f'                    Interfaces.C.Strings.New_String ({svc_const});\n')
        f.write(f'                  Msg : aliased Pcl_Bindings.Pcl_Msg :=\n')
        f.write(f'                    (Data      => To_Address (Buf),\n')
        f.write(f'                     Size      => Interfaces.C.unsigned (Payload_Str\'Length),\n')
        f.write(f'                     Type_Name => Interfaces.C.Strings.New_String (Json_Content_Type));\n')
        f.write(f'                  Status : Pcl_Bindings.Pcl_Status;\n')
        f.write(f'                  pragma Unreferenced (Status);\n')
        f.write(f'               begin\n')
        f.write(f'                  Status := Pcl_Bindings.Invoke_Async\n')
        f.write(f'                    ({prefix}_Client_Executor, Svc_C, Msg\'Access,\n')
        f.write(f'                     {prefix}_On_Rpc_Reply\'Access, System.Null_Address);\n')
        f.write(f'                  Interfaces.C.Strings.Free (Buf);\n')
        f.write(f'                  Interfaces.C.Strings.Free (Svc_C);\n')
        f.write(f'                  Interfaces.C.Strings.Free (Msg.Type_Name);\n')
        f.write(f'               end;\n')
        f.write(f'            else\n')
        f.write(f'               {prefix}_Rpc_Reply_Ready := True;\n')
        f.write(f'               {prefix}_Rpc_Reply_Status := Pcl_Bindings.PCL_ERR_INVALID;\n')
        f.write(f'               {prefix}_Rpc_Reply_Has_Ack := False;\n')
        f.write(f'            end if;\n')
        f.write(f'         end;\n')
        f.write(f'         Result_Accepted := {prefix}_Rpc_Reply_Ready\n')
        f.write(f'           and then {prefix}_Rpc_Reply_Status = Pcl_Bindings.PCL_OK;\n')
        f.write(f'         Result_Status := Interfaces.C.int ({prefix}_Rpc_Reply_Status);\n')
        f.write(f'         Result_Has_Ack :=\n')
        f.write(f'           {prefix}_Rpc_Reply_Ready and then {prefix}_Rpc_Reply_Has_Ack;\n')
        f.write(f'         if Result_Has_Ack then\n')
        f.write(f'            Result_Ack := {prefix}_Rpc_Reply_Ack;\n')
        f.write(f'         end if;\n')
        if not projectable:
            f.write(f'      else\n')
            f.write(f'         --  D2: not projectable to pub/sub -- fail closed, no synthetic ack.\n')
            f.write(f'         Result_Accepted := False;\n')
            f.write(f'         Result_Status := Interfaces.C.int (Pcl_Bindings.PCL_ERR_STATE);\n')
            f.write(f'         Result_Has_Ack := False;\n')
            f.write(f'      end if;\n')
            f.write(f'   end {prefix}_Submit_{ep.rpc_name};\n')
            f.write('\n')
            return
        f.write(f'      else\n')
        f.write(f'         Result_Has_Ack := False;\n')
        f.write(f'         declare\n')
        if cp.reason == 'is_wrapper':
            f.write(f'            Payload : Unbounded_String;\n')
            f.write(f'            Encoded : constant Boolean :=\n')
            f.write(f'              Try_Registry_Encode\n')
            f.write(f'                (Json_Content_Type, "{req_schema}", Request\'Address, Payload);\n')
        else:
            wrapper_t = _ada_req_type_of_wrapper(cp)
            wrapper_schema = _short_type(cp.wrapper_type)
            variant_field = _ada_field_name(cp.variant_field)
            f.write(f'            Wrapper : {wrapper_t} := (others => <>);\n')
            f.write(f'            Payload : Unbounded_String;\n')
            f.write(f'            Encoded : Boolean;\n')
        f.write(f'            Status  : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_STATE;\n')
        f.write(f'         begin\n')
        if cp.reason != 'is_wrapper':
            f.write(f'            Wrapper.Has_{variant_field} := True;\n')
            f.write(f'            Wrapper.{variant_field} := Request;\n')
            f.write(f'            Encoded :=\n')
            f.write(f'              Try_Registry_Encode\n')
            f.write(f'                (Json_Content_Type, "{wrapper_schema}", Wrapper\'Address, Payload);\n')
        f.write(f'            if Encoded and then {prefix}_Request_Publisher /= null then\n')
        f.write(f'               declare\n')
        f.write(f'                  Payload_Str : constant String := To_String (Payload);\n')
        f.write(f'                  Buf : Interfaces.C.Strings.chars_ptr :=\n')
        f.write(f'                    Interfaces.C.Strings.New_String (Payload_Str);\n')
        f.write(f'                  Msg : aliased Pcl_Bindings.Pcl_Msg :=\n')
        f.write(f'                    (Data      => To_Address (Buf),\n')
        f.write(f'                     Size      => Interfaces.C.unsigned (Payload_Str\'Length),\n')
        f.write(f'                     Type_Name => Interfaces.C.Strings.New_String (Json_Content_Type));\n')
        f.write(f'               begin\n')
        f.write(f'                  Status := Pcl_Bindings.Port_Publish ({prefix}_Request_Publisher, Msg\'Access);\n')
        f.write(f'                  Interfaces.C.Strings.Free (Buf);\n')
        f.write(f'                  Interfaces.C.Strings.Free (Msg.Type_Name);\n')
        f.write(f'               end;\n')
        f.write(f'            end if;\n')
        f.write(f'            Result_Accepted := Status = Pcl_Bindings.PCL_OK;\n')
        f.write(f'            Result_Status := Interfaces.C.int (Status);\n')
        f.write(f'         end;\n')
        f.write(f'      end if;\n')
        f.write(f'   end {prefix}_Submit_{ep.rpc_name};\n')
        f.write('\n')

    def _write_client_transitions_body(self, f, prefix, ia, read_rpc, frame_t, frame_schema) -> None:
        req_t = _ada_req_type(read_rpc)
        read_svc_const = _rpc_ada_svc_const(ia.service_name, read_rpc, set())

        f.write(f'   procedure {prefix}_Transitions\n')
        f.write(f'     (Filter   : {req_t};\n')
        f.write(f'      Callback : {prefix}_Transition_Callback)\n')
        f.write(f'   is\n')
        f.write(f'   begin\n')
        f.write(f'      if {prefix}_Requirement_Binding = {prefix}_Binding_Pubsub then\n')
        f.write(f'         for I in {prefix}_Transition_Slots\'Range loop\n')
        f.write(f'            if not {prefix}_Transition_Slots (I).Active then\n')
        f.write(f'               {prefix}_Transition_Slots (I) :=\n')
        f.write(f'                 (Active   => True,\n')
        f.write(f'                  One_Shot => Filter.Has_One_Shot and then Filter.One_Shot,\n')
        f.write(f'                  Ids      => Filter.Id,\n')
        f.write(f'                  Callback => Callback);\n')
        f.write(f'               return;\n')
        f.write(f'            end if;\n')
        f.write(f'         end loop;\n')
        f.write(f'         --  Table full (bounded, {prefix}_Max_Transition_Subscriptions): dropped.\n')
        f.write(f'      else\n')
        f.write(f'         declare\n')
        f.write(f'            Payload : Unbounded_String;\n')
        f.write(f'            Encoded : constant Boolean :=\n')
        f.write(f'              Try_Registry_Encode (Json_Content_Type, "Query", Filter\'Address, Payload);\n')
        f.write(f'         begin\n')
        f.write(f'            if not Encoded then\n')
        f.write(f'               return;\n')
        f.write(f'            end if;\n')
        f.write(f'            for I in {prefix}_Rpc_Stream_Slots\'Range loop\n')
        f.write(f'               if not {prefix}_Rpc_Stream_Slots (I).Active then\n')
        f.write(f'                  {prefix}_Rpc_Stream_Slots (I) := (Active => True, Callback => Callback);\n')
        f.write(f'                  declare\n')
        f.write(f'                     Payload_Str : constant String := To_String (Payload);\n')
        f.write(f'                     Buf : Interfaces.C.Strings.chars_ptr :=\n')
        f.write(f'                       Interfaces.C.Strings.New_String (Payload_Str);\n')
        f.write(f'                     Svc_C : Interfaces.C.Strings.chars_ptr :=\n')
        f.write(f'                       Interfaces.C.Strings.New_String ({read_svc_const});\n')
        f.write(f'                     Msg : aliased Pcl_Bindings.Pcl_Msg :=\n')
        f.write(f'                       (Data      => To_Address (Buf),\n')
        f.write(f'                        Size      => Interfaces.C.unsigned (Payload_Str\'Length),\n')
        f.write(f'                        Type_Name => Interfaces.C.Strings.New_String (Json_Content_Type));\n')
        f.write(f'                     Stream_Ctx : aliased Pcl_Bindings.Pcl_Stream_Context_Access := null;\n')
        f.write(f'                     Status : Pcl_Bindings.Pcl_Status;\n')
        f.write(f'                     pragma Unreferenced (Status);\n')
        f.write(f'                  begin\n')
        f.write(f'                     Status := Pcl_Bindings.Invoke_Stream\n')
        f.write(f'                       ({prefix}_Client_Executor, Svc_C, Msg\'Access,\n')
        f.write(f'                        {prefix}_On_Stream_Msg\'Access, {prefix}_Slot_Address (I),\n')
        f.write(f'                        Stream_Ctx\'Access);\n')
        f.write(f'                     Interfaces.C.Strings.Free (Buf);\n')
        f.write(f'                     Interfaces.C.Strings.Free (Svc_C);\n')
        f.write(f'                     Interfaces.C.Strings.Free (Msg.Type_Name);\n')
        f.write(f'                  end;\n')
        f.write(f'                  return;\n')
        f.write(f'               end if;\n')
        f.write(f'            end loop;\n')
        f.write(f'         end;\n')
        f.write(f'         --  Table full (bounded, {prefix}_Max_Rpc_Streams): dropped.\n')
        f.write(f'      end if;\n')
        f.write(f'   end {prefix}_Transitions;\n')
        f.write('\n')

    # -- Provider (provided-role) body -----------------------------------

    def _write_provider_facade_body(
        self, f, prefix, ia, index, parsed, svc, rpcs, read_rpc, frame_t,
        frame_schema, corr_field, request_wire, requirement_wire,
    ) -> None:
        if svc is None:
            return
        request_leg = next((leg for leg in ia.legs if leg.name == 'request'), None)
        if request_leg is None:
            return
        variants, wrapper = _ordered_command_projectability(index, parsed, svc)
        commands = variants + ([wrapper] if wrapper else [])
        if not commands:
            return
        wrapper_t = _ada_req_type_of_wrapper(wrapper) if wrapper else None

        f.write(f'   {prefix}_Requirement_Binding : {prefix}_Interaction_Binding :=\n')
        f.write(f'     {prefix}_Binding_Rpc;\n')
        f.write(f'   {prefix}_Provider_Handlers  : {prefix}_Interaction_Handlers;\n')
        f.write(f'   {prefix}_Requirement_Publisher : Pcl_Bindings.Pcl_Port_Access := null;\n')
        f.write('\n')

        # Forward declarations (+ pragma Convention) for the C-convention
        # callbacks below -- see the client body's identical note.
        for cp in commands:
            f.write(f'   function {prefix}_Handle_{cp.rpc_name}\n')
            f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Request   : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      Response  : access Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;\n')
            f.write(f'      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;\n')
            f.write(f'   pragma Convention (C, {prefix}_Handle_{cp.rpc_name});\n')
            f.write('\n')
        if frame_t is not None:
            f.write(f'   function {prefix}_Handle_Read_Stream\n')
            f.write(f'     (Self       : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Request    : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      Stream_Ctx : Pcl_Bindings.Pcl_Stream_Context_Access;\n')
            f.write(f'      User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;\n')
            f.write(f'   pragma Convention (C, {prefix}_Handle_Read_Stream);\n')
            f.write('\n')
        if wrapper_t is not None:
            f.write(f'   procedure {prefix}_On_Request_Message\n')
            f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Msg       : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      User_Data : System.Address);\n')
            f.write(f'   pragma Convention (C, {prefix}_On_Request_Message);\n')
            f.write('\n')

        f.write(f'   function {prefix}_Parse_Binding (Value : String) return {prefix}_Interaction_Binding is\n')
        f.write(f'     (if Value = "pubsub" then {prefix}_Binding_Pubsub else {prefix}_Binding_Rpc);\n')
        f.write('\n')

        if frame_t is not None:
            f.write(f'   {prefix}_Max_Snapshots : constant := 32;\n')
            f.write(f'   type {prefix}_Snapshot_Slot is record\n')
            f.write(f'      Used  : Boolean := False;\n')
            f.write(f'      Id    : Unbounded_String := Null_Unbounded_String;\n')
            f.write(f'      Frame : {frame_t};\n')
            f.write(f'   end record;\n')
            f.write(f'   {prefix}_Snapshots :\n')
            f.write(f'     array (1 .. {prefix}_Max_Snapshots) of {prefix}_Snapshot_Slot;\n')
            f.write(f'   {prefix}_Next_Snapshot_Slot : Positive := 1;\n')
            f.write('\n')

            f.write(f'   procedure {prefix}_Record_Snapshot (Id : Unbounded_String; Frame : {frame_t}) is\n')
            f.write(f'   begin\n')
            f.write(f'      if Id = Null_Unbounded_String then\n')
            f.write(f'         return;\n')
            f.write(f'      end if;\n')
            f.write(f'      for I in {prefix}_Snapshots\'Range loop\n')
            f.write(f'         if {prefix}_Snapshots (I).Used and then {prefix}_Snapshots (I).Id = Id then\n')
            f.write(f'            {prefix}_Snapshots (I).Frame := Frame;\n')
            f.write(f'            return;\n')
            f.write(f'         end if;\n')
            f.write(f'      end loop;\n')
            f.write(f'      {prefix}_Snapshots ({prefix}_Next_Snapshot_Slot) := (Used => True, Id => Id, Frame => Frame);\n')
            f.write(f'      if {prefix}_Next_Snapshot_Slot = {prefix}_Max_Snapshots then\n')
            f.write(f'         {prefix}_Next_Snapshot_Slot := 1;\n')
            f.write(f'      else\n')
            f.write(f'         {prefix}_Next_Snapshot_Slot := {prefix}_Next_Snapshot_Slot + 1;\n')
            f.write(f'      end if;\n')
            f.write(f'   end {prefix}_Record_Snapshot;\n')
            f.write('\n')

            f.write(f'   {prefix}_Max_Open_Streams : constant := 8;\n')
            f.write(f'   type {prefix}_Open_Stream_Slot is record\n')
            f.write(f'      Active   : Boolean := False;\n')
            f.write(f'      One_Shot : Boolean := False;\n')
            f.write(f'      Ids      : Pyramid.Data_Model.Common.Types.Identifier_Array_Acc := null;\n')
            f.write(f'      Ctx      : Pcl_Bindings.Pcl_Stream_Context_Access := null;\n')
            f.write(f'   end record;\n')
            f.write(f'   {prefix}_Open_Streams :\n')
            f.write(f'     array (1 .. {prefix}_Max_Open_Streams) of {prefix}_Open_Stream_Slot;\n')
            f.write('\n')

            f.write(f'   function {prefix}_Id_Matches\n')
            f.write(f'     (Ids   : Pyramid.Data_Model.Common.Types.Identifier_Array_Acc;\n')
            f.write(f'      Value : Unbounded_String) return Boolean\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            f.write(f'      if Ids = null or else Ids\'Length = 0 then\n')
            f.write(f'         return True;\n')
            f.write(f'      end if;\n')
            f.write(f'      for I in Ids\'Range loop\n')
            f.write(f'         if Ids (I) = Value then\n')
            f.write(f'            return True;\n')
            f.write(f'         end if;\n')
            f.write(f'      end loop;\n')
            f.write(f'      return False;\n')
            f.write(f'   end {prefix}_Id_Matches;\n')
            f.write('\n')

            f.write(f'   procedure {prefix}_Stream_Send_Frame (Index : Positive; Transition : {frame_t}) is\n')
            f.write(f'      Payload : Unbounded_String;\n')
            f.write(f'      Encoded : constant Boolean :=\n')
            f.write(f'        Try_Registry_Encode\n')
            f.write(f'          (Json_Content_Type, "{frame_schema}", Transition\'Address, Payload);\n')
            f.write(f'   begin\n')
            f.write(f'      if not Encoded then\n')
            f.write(f'         return;\n')
            f.write(f'      end if;\n')
            f.write(f'      declare\n')
            f.write(f'         Payload_Str : constant String := To_String (Payload);\n')
            f.write(f'         Buf : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String (Payload_Str);\n')
            f.write(f'         Msg : aliased Pcl_Bindings.Pcl_Msg :=\n')
            f.write(f'           (Data      => To_Address (Buf),\n')
            f.write(f'            Size      => Interfaces.C.unsigned (Payload_Str\'Length),\n')
            f.write(f'            Type_Name => Interfaces.C.Strings.New_String (Json_Content_Type));\n')
            f.write(f'         Status : Pcl_Bindings.Pcl_Status;\n')
            f.write(f'         pragma Unreferenced (Status);\n')
            f.write(f'      begin\n')
            f.write(f'         Status := Pcl_Bindings.Stream_Send ({prefix}_Open_Streams (Index).Ctx, Msg\'Access);\n')
            f.write(f'         Interfaces.C.Strings.Free (Buf);\n')
            f.write(f'         Interfaces.C.Strings.Free (Msg.Type_Name);\n')
            f.write(f'      end;\n')
            f.write(f'   end {prefix}_Stream_Send_Frame;\n')
            f.write('\n')

            f.write(f'   procedure {prefix}_Send_Transition (Transition : {frame_t}) is\n')
            f.write(f'      Has_Id   : constant Boolean :=')
            if corr_field is not None:
                f.write(f' Transition.Has_{corr_field};\n')
            else:
                f.write(f' False;\n')
            f.write(f'      Frame_Id : Unbounded_String := Null_Unbounded_String;\n')
            f.write(f'   begin\n')
            if corr_field is not None:
                f.write(f'      if Has_Id then\n')
                f.write(f'         Frame_Id := Transition.{corr_field}.Id;\n')
                f.write(f'      end if;\n')
            f.write(f'      {prefix}_Record_Snapshot (Frame_Id, Transition);\n')
            f.write(f'      if {prefix}_Requirement_Binding = {prefix}_Binding_Rpc then\n')
            f.write(f'         for I in {prefix}_Open_Streams\'Range loop\n')
            f.write(f'            if {prefix}_Open_Streams (I).Active\n')
            f.write(f'              and then (not Has_Id or else {prefix}_Id_Matches\n')
            f.write(f'                          ({prefix}_Open_Streams (I).Ids, Frame_Id))\n')
            f.write(f'            then\n')
            f.write(f'               {prefix}_Stream_Send_Frame (I, Transition);\n')
            f.write(f'               if {prefix}_Open_Streams (I).One_Shot then\n')
            f.write(f'                  declare\n')
            f.write(f'                     St : Pcl_Bindings.Pcl_Status;\n')
            f.write(f'                     pragma Unreferenced (St);\n')
            f.write(f'                  begin\n')
            f.write(f'                     St := Pcl_Bindings.Stream_End ({prefix}_Open_Streams (I).Ctx);\n')
            f.write(f'                  end;\n')
            f.write(f'                  {prefix}_Open_Streams (I).Active := False;\n')
            f.write(f'               end if;\n')
            f.write(f'            end if;\n')
            f.write(f'         end loop;\n')
            f.write(f'      else\n')
            f.write(f'         if {prefix}_Requirement_Publisher /= null then\n')
            f.write(f'            declare\n')
            f.write(f'               Payload : Unbounded_String;\n')
            f.write(f'               Encoded : constant Boolean :=\n')
            f.write(f'                 Try_Registry_Encode\n')
            f.write(f'                   (Json_Content_Type, "{frame_schema}", Transition\'Address, Payload);\n')
            f.write(f'            begin\n')
            f.write(f'               if Encoded then\n')
            f.write(f'                  declare\n')
            f.write(f'                     Payload_Str : constant String := To_String (Payload);\n')
            f.write(f'                     Buf : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'                       Interfaces.C.Strings.New_String (Payload_Str);\n')
            f.write(f'                     Msg : aliased Pcl_Bindings.Pcl_Msg :=\n')
            f.write(f'                       (Data      => To_Address (Buf),\n')
            f.write(f'                        Size      => Interfaces.C.unsigned (Payload_Str\'Length),\n')
            f.write(f'                        Type_Name => Interfaces.C.Strings.New_String (Json_Content_Type));\n')
            f.write(f'                     Status : Pcl_Bindings.Pcl_Status;\n')
            f.write(f'                     pragma Unreferenced (Status);\n')
            f.write(f'                  begin\n')
            f.write(f'                     Status := Pcl_Bindings.Port_Publish ({prefix}_Requirement_Publisher, Msg\'Access);\n')
            f.write(f'                     Interfaces.C.Strings.Free (Buf);\n')
            f.write(f'                     Interfaces.C.Strings.Free (Msg.Type_Name);\n')
            f.write(f'                  end;\n')
            f.write(f'               end if;\n')
            f.write(f'            end;\n')
            f.write(f'         end if;\n')
            f.write(f'      end if;\n')
            f.write(f'   end {prefix}_Send_Transition;\n')
            f.write('\n')

            f.write(f'   function {prefix}_Handle_Read_Stream\n')
            f.write(f'     (Self       : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Request    : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      Stream_Ctx : Pcl_Bindings.Pcl_Stream_Context_Access;\n')
            f.write(f'      User_Data  : System.Address) return Pcl_Bindings.Pcl_Status\n')
            f.write(f'   is\n')
            f.write(f'      pragma Unreferenced (Self, User_Data);\n')
            f.write(f'      Filter : Query;\n')
            f.write(f'   begin\n')
            f.write(f'      if not Try_Registry_Decode (Request, "Query", Filter\'Address) then\n')
            f.write(f'         return Pcl_Bindings.PCL_ERR_INVALID;\n')
            f.write(f'      end if;\n')
            f.write(f'      for I in {prefix}_Open_Streams\'Range loop\n')
            f.write(f'         if not {prefix}_Open_Streams (I).Active then\n')
            f.write(f'            {prefix}_Open_Streams (I) :=\n')
            f.write(f'              (Active   => True,\n')
            f.write(f'               One_Shot => Filter.Has_One_Shot and then Filter.One_Shot,\n')
            f.write(f'               Ids      => Filter.Id,\n')
            f.write(f'               Ctx      => Stream_Ctx);\n')
            f.write(f'            --  D4: late-join replay -- send every stored snapshot matching\n')
            f.write(f'            --  this Read\'s filter immediately, so a reader arriving after\n')
            f.write(f'            --  the transition already happened still observes it.\n')
            f.write(f'            for J in {prefix}_Snapshots\'Range loop\n')
            f.write(f'               if {prefix}_Snapshots (J).Used\n')
            f.write(f'                 and then {prefix}_Id_Matches (Filter.Id, {prefix}_Snapshots (J).Id)\n')
            f.write(f'               then\n')
            f.write(f'                  {prefix}_Stream_Send_Frame (I, {prefix}_Snapshots (J).Frame);\n')
            f.write(f'               end if;\n')
            f.write(f'            end loop;\n')
            f.write(f'            return Pcl_Bindings.PCL_STREAMING;\n')
            f.write(f'         end if;\n')
            f.write(f'      end loop;\n')
            f.write(f'      return Pcl_Bindings.PCL_ERR_NOMEM;\n')
            f.write(f'   end {prefix}_Handle_Read_Stream;\n')
            f.write('\n')

        for cp in commands:
            self._write_provider_unary_handler(f, prefix, ia, rpcs, cp)

        if wrapper_t is not None:
            wrapper_schema = _short_type(wrapper.wrapper_type)
            self._write_provider_dispatch_pubsub(
                f, prefix, wrapper_t, wrapper_schema, commands, request_wire,
            )

        f.write(f'   procedure {prefix}_Configure_Interaction_Binding (Config_Json : String) is\n')
        f.write(f'      Binding_Value     : constant String := Config_Value (Config_Json, "binding");\n')
        f.write(f'      Requirement_Value : constant String :=\n')
        f.write(f'        Config_Value (Config_Json, "requirement_leg");\n')
        f.write(f'   begin\n')
        f.write(f'      {prefix}_Requirement_Binding := {prefix}_Parse_Binding\n')
        f.write(f'        (if Requirement_Value\'Length /= 0 then Requirement_Value else Binding_Value);\n')
        f.write(f'   end {prefix}_Configure_Interaction_Binding;\n')
        f.write('\n')

        f.write(f'   procedure {prefix}_Provider_Bind\n')
        f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
        f.write(f'      Executor  : Pcl_Bindings.Pcl_Executor_Access;\n')
        f.write(f'      Handlers  : {prefix}_Interaction_Handlers)\n')
        f.write(f'   is\n')
        f.write(f'      pragma Unreferenced (Executor);\n')
        f.write(f'      Port : Pcl_Bindings.Pcl_Port_Access;\n')
        f.write(f'      Route_Status : Pcl_Bindings.Pcl_Status;\n')
        f.write(f'      pragma Unreferenced (Port, Route_Status);\n')
        f.write(f'   begin\n')
        f.write(f'      {prefix}_Provider_Handlers := Handlers;\n')
        for cp in commands:
            svc_const = _rpc_ada_svc_const(ia.service_name, rpcs[cp.rpc_name], set())
            f.write(f'      declare\n')
            f.write(f'         Svc_C  : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String ({svc_const});\n')
            f.write(f'         Type_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String (Json_Content_Type);\n')
            f.write(f'      begin\n')
            f.write(f'         Port := Pcl_Bindings.Add_Service\n')
            f.write(f'           (Container, Svc_C, Type_C,\n')
            f.write(f'            {prefix}_Handle_{cp.rpc_name}\'Access, System.Null_Address);\n')
            f.write(f'         Interfaces.C.Strings.Free (Svc_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         Route_Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           (Port, Pcl_Bindings.PCL_ROUTE_LOCAL, System.Null_Address, 0);\n')
            f.write(f'      end;\n')
        if frame_t is not None:
            read_svc_const = _rpc_ada_svc_const(ia.service_name, read_rpc, set())
            f.write(f'      declare\n')
            f.write(f'         Svc_C  : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String ({read_svc_const});\n')
            f.write(f'         Type_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String (Json_Content_Type);\n')
            f.write(f'      begin\n')
            f.write(f'         Port := Pcl_Bindings.Add_Stream_Service\n')
            f.write(f'           (Container, Svc_C, Type_C,\n')
            f.write(f'            {prefix}_Handle_Read_Stream\'Access, System.Null_Address);\n')
            f.write(f'         Interfaces.C.Strings.Free (Svc_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         Route_Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           (Port, Pcl_Bindings.PCL_ROUTE_LOCAL, System.Null_Address, 0);\n')
            f.write(f'      end;\n')
        if request_wire is not None and wrapper_t is not None:
            f.write(f'      declare\n')
            f.write(f'         Topic_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String ("{request_wire}");\n')
            f.write(f'         Type_C  : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String (Json_Content_Type);\n')
            f.write(f'      begin\n')
            f.write(f'         Port := Pcl_Bindings.Add_Subscriber\n')
            f.write(f'           (Container, Topic_C, Type_C,\n')
            f.write(f'            {prefix}_On_Request_Message\'Access, System.Null_Address);\n')
            f.write(f'         Interfaces.C.Strings.Free (Topic_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         Route_Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           (Port, Pcl_Bindings.PCL_ROUTE_LOCAL, System.Null_Address, 0);\n')
            f.write(f'      end;\n')
        if requirement_wire is not None and frame_t is not None:
            f.write(f'      declare\n')
            f.write(f'         Topic_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String ("{requirement_wire}");\n')
            f.write(f'         Type_C  : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'           Interfaces.C.Strings.New_String (Json_Content_Type);\n')
            f.write(f'      begin\n')
            f.write(f'         {prefix}_Requirement_Publisher :=\n')
            f.write(f'           Pcl_Bindings.Add_Publisher (Container, Topic_C, Type_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Topic_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         Route_Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           ({prefix}_Requirement_Publisher, Pcl_Bindings.PCL_ROUTE_LOCAL,\n')
            f.write(f'            System.Null_Address, 0);\n')
            f.write(f'      end;\n')
        f.write(f'   end {prefix}_Provider_Bind;\n')
        f.write('\n')

    def _write_provider_unary_handler(self, f, prefix, ia, rpcs, cp) -> None:
        rpc = rpcs[cp.rpc_name]
        req_t = _ada_req_type(rpc)
        req_schema = _short_type(rpc.request_type)

        f.write(f'   function {prefix}_Handle_{cp.rpc_name}\n')
        f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
        f.write(f'      Request   : access constant Pcl_Bindings.Pcl_Msg;\n')
        f.write(f'      Response  : access Pcl_Bindings.Pcl_Msg;\n')
        f.write(f'      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;\n')
        f.write(f'      User_Data : System.Address) return Pcl_Bindings.Pcl_Status\n')
        f.write(f'   is\n')
        f.write(f'      pragma Unreferenced (Self, Ctx, User_Data);\n')
        f.write(f'      Req_Value : {req_t};\n')
        f.write(f'      Result    : Ack;\n')
        f.write(f'      Payload   : Unbounded_String;\n')
        f.write(f'   begin\n')
        f.write(f'      if not Try_Registry_Decode (Request, "{req_schema}", Req_Value\'Address) then\n')
        f.write(f'         Response.Data := System.Null_Address;\n')
        f.write(f'         Response.Size := 0;\n')
        f.write(f'         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;\n')
        f.write(f'         return Pcl_Bindings.PCL_ERR_INVALID;\n')
        f.write(f'      end if;\n')
        f.write(f'      if {prefix}_Provider_Handlers.On_{cp.rpc_name} = null then\n')
        f.write(f'         Result := (Success => False, Identifier => Null_Unbounded_String);\n')
        f.write(f'      else\n')
        f.write(f'         Result := {prefix}_Provider_Handlers.On_{cp.rpc_name} (Req_Value);\n')
        f.write(f'      end if;\n')
        f.write(f'      if not Try_Registry_Encode (Json_Content_Type, "Ack", Result\'Address, Payload) then\n')
        f.write(f'         Response.Data := System.Null_Address;\n')
        f.write(f'         Response.Size := 0;\n')
        f.write(f'         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;\n')
        f.write(f'         return Pcl_Bindings.PCL_ERR_INVALID;\n')
        f.write(f'      end if;\n')
        f.write(f'      declare\n')
        f.write(f'         Payload_Str : constant String := To_String (Payload);\n')
        f.write(f'         Buf : Interfaces.C.Strings.chars_ptr :=\n')
        f.write(f'           Interfaces.C.Strings.New_String (Payload_Str);\n')
        f.write(f'      begin\n')
        f.write(f'         Response.Data := To_Address (Buf);\n')
        f.write(f'         Response.Size := Interfaces.C.unsigned (Payload_Str\'Length);\n')
        f.write(f'         Response.Type_Name := Interfaces.C.Strings.New_String (Json_Content_Type);\n')
        f.write(f'      end;\n')
        f.write(f'      return Pcl_Bindings.PCL_OK;\n')
        f.write(f'   exception\n')
        f.write(f'      when others =>\n')
        f.write(f'         Response.Data := System.Null_Address;\n')
        f.write(f'         Response.Size := 0;\n')
        f.write(f'         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;\n')
        f.write(f'         return Pcl_Bindings.PCL_ERR_INVALID;\n')
        f.write(f'   end {prefix}_Handle_{cp.rpc_name};\n')
        f.write('\n')

    def _write_provider_dispatch_pubsub(
        self, f, prefix, wrapper_t, wrapper_schema, commands, request_wire,
    ) -> None:
        f.write(f'   procedure {prefix}_On_Request_Message\n')
        f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
        f.write(f'      Msg       : access constant Pcl_Bindings.Pcl_Msg;\n')
        f.write(f'      User_Data : System.Address)\n')
        f.write(f'   is\n')
        f.write(f'      pragma Unreferenced (Self, User_Data);\n')
        f.write(f'      Wrapper : {wrapper_t};\n')
        f.write(f'      Ignored : Ack;\n')
        f.write(f'      pragma Unreferenced (Ignored);\n')
        f.write(f'   begin\n')
        f.write(f'      if not Try_Registry_Decode (Msg, "{wrapper_schema}", Wrapper\'Address) then\n')
        f.write(f'         return;\n')
        f.write(f'      end if;\n')
        variant_cps = [cp for cp in commands if cp.reason == 'matches_variant']
        wrapper_cp = next((cp for cp in commands if cp.reason != 'matches_variant'), None)
        if not variant_cps:
            # No matches_variant commands at all -- the wrapper (is_wrapper)
            # is the only command this service can carry over pub/sub;
            # dispatch it unconditionally.
            if wrapper_cp is not None:
                f.write(f'      if {prefix}_Provider_Handlers.On_{wrapper_cp.rpc_name} /= null then\n')
                f.write(f'         Ignored := {prefix}_Provider_Handlers.On_{wrapper_cp.rpc_name} (Wrapper);\n')
                f.write(f'      end if;\n')
        else:
            for i, cp in enumerate(variant_cps):
                variant_field = _ada_field_name(cp.variant_field)
                kw = 'if' if i == 0 else 'elsif'
                f.write(f'      {kw} Wrapper.Has_{variant_field} then\n')
                f.write(f'         if {prefix}_Provider_Handlers.On_{cp.rpc_name} /= null then\n')
                f.write(f'            Ignored := {prefix}_Provider_Handlers.On_{cp.rpc_name} (Wrapper.{variant_field});\n')
                f.write(f'         end if;\n')
            if wrapper_cp is not None:
                f.write(f'      else\n')
                f.write(f'         if {prefix}_Provider_Handlers.On_{wrapper_cp.rpc_name} /= null then\n')
                f.write(f'            Ignored := {prefix}_Provider_Handlers.On_{wrapper_cp.rpc_name} (Wrapper);\n')
                f.write(f'         end if;\n')
            f.write(f'      end if;\n')
        f.write(f'   end {prefix}_On_Request_Message;\n')
        f.write('\n')


def _ada_req_type_of_wrapper(cp: CommandProjectability) -> str:
    return _proto_type_to_ada(_short_type(cp.wrapper_type))
