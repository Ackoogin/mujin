#!/usr/bin/env python3
"""Ada service binding specification (.ads) emitter.

Split verbatim from ada_codegen.py (generator refactor plan, phase 4).
"""

from pathlib import Path
from typing import List, Tuple

from proto_parser import (
    ProtoFile,
    ProtoRpc,
    camel_to_snake as _camel_to_snake,
)
from .naming import (
    _short_type,
    _proto_type_to_ada,
    _service_wire_prefix,
    _duplicate_rpc_names,
    _rpc_ada_handler,
    _rpc_ada_channel,
    _rpc_ada_invoke_name,
    _rpc_ada_decode_response_name,
    _rpc_ada_svc_const,
    _rpc_ada_handler_field,
    _crud_rpcs,
    _rpc_wire_name,
    _ada_req_type,
    _ada_rsp_type,
    _is_provided,
    _applicable_topics,
)


class SpecEmitterMixin:
    """Service constants, topic constants, handler surface and PCL
    binding declarations of the generated .ads."""

    # -- Spec (.ads) -----------------------------------------------------------

    def _write_spec(self, path: Path, pkg_name: str, parsed: ProtoFile,
                    all_rpcs: List[Tuple[str, ProtoRpc]],
                    type_pkgs: List[str]):
        entity_types_for_arrays = sorted({
            _proto_type_to_ada(_short_type(rpc.response_type))
            for _, rpc in all_rpcs if rpc.server_streaming
        })
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)

        is_provided = _is_provided(parsed)
        has_grpc = False
        has_ros2 = 'ros2' in self._enabled_backends
        sub_topics, pub_topics, _ = _applicable_topics(
            parsed, is_provided, self._proto_input, self._topics)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'--  Auto-generated service binding specification\n')
            f.write(f'--  Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'--  Package: {pkg_name}\n')
            f.write(f'--\n')
            f.write(f'--  Architecture: component logic > service binding (this) > PCL\n')
            f.write(f'--\n')
            f.write(f'--  This package provides:\n')
            f.write(f'--    1. Wire-name constants and topic constants\n')
            f.write(f'--    2. EntityActions callback access types and handler set record\n')
            f.write(f'--    3. PCL binding procedures (Register_Services, Subscribe_*, Invoke_*, Publish_*)\n')
            f.write(f'--    4. Msg_To_String utility for PCL message payloads\n')
            f.write(f'--\n')
            f.write(f'--  RPC request/response payloads use the proto-native data model.\n')
            f.write(f'--  Standard topic payloads also use canonical proto-derived types.\n')
            f.write(f'\n')
            for tp in type_pkgs:
                f.write(f'with {tp};  use {tp};\n')
            f.write(f'with Pcl_Bindings;\n')
            f.write(f'with Interfaces.C;\n')
            f.write(f'with System;\n')
            f.write(f'\n')
            f.write(f'package {pkg_name} is\n')
            f.write(f'\n')

            # Operation_Kind enumeration
            f.write(f'   type Operation_Kind is\n')
            f.write(f'     (Op_Create,\n')
            f.write(f'      Op_Read,\n')
            f.write(f'      Op_Update,\n')
            f.write(f'      Op_Delete);\n')
            f.write(f'\n')

            # Service_Channel enumeration
            f.write(f'   type Service_Channel is\n')
            channel_values = [
                f'      {_rpc_ada_channel(svc_name, rpc, duplicate_rpc_names)}'
                for svc_name, rpc in all_rpcs
            ]
            if channel_values:
                f.write('     (' + ',\n'.join(channel_values).lstrip() + ');\n')
            else:
                f.write('     (Ch_None);\n')
            f.write('\n')

            # Array types for streaming Read responses
            for et in entity_types_for_arrays:
                f.write(f'   type {et}_Array is array (Positive range <>) of {et};\n')
            if entity_types_for_arrays:
                f.write('\n')

            # -- Service wire-name constants -----------------------------------
            f.write(f'   --  -- Service wire-name constants (generated from proto) --------\n')
            f.write(f'\n')

            for svc in parsed.services:
                prefix = _service_wire_prefix(svc.name)
                for rpc in _crud_rpcs(svc):
                    const_name = _rpc_ada_svc_const(
                        svc.name, rpc, duplicate_rpc_names)
                    wire_name = f'{prefix}.{_rpc_wire_name(rpc)}'
                    f.write(f'   {const_name} : constant String :=\n')
                    f.write(f'     "{wire_name}";\n')
            f.write('\n')

            # -- Topic name constants ------------------------------------------
            all_topics = dict(sub_topics)
            all_topics.update(pub_topics)
            if all_topics:
                f.write(f'   --  -- Standard topic name constants --------------------------\n')
                f.write(f'\n')
                for key, wire in all_topics.items():
                    const_name = 'Topic_' + '_'.join(
                        w.capitalize() for w in key.split('_'))
                    f.write(f'   {const_name} : constant String :=\n')
                    f.write(f'     "{wire}";\n')
                f.write('\n')

            if has_ros2:
                f.write(f'   --  -- ROS2 endpoint constants --------------------------------\n')
                f.write(f'\n')
                f.write(f'   Ros2_Transport_Content_Type : constant String := "application/ros2";\n')
                f.write(f'\n')
                for svc in parsed.services:
                    svc_name = svc.name
                    if svc_name.endswith('_Service'):
                        svc_name = svc_name[:-len('_Service')]
                    svc_const_prefix = _camel_to_snake(svc_name)
                    for rpc in _crud_rpcs(svc):
                        rpc_name = _camel_to_snake(rpc.name)
                        base_name = f'{svc_const_prefix}_{rpc_name}'
                        if rpc.server_streaming:
                            base = f'/pyramid/stream/{_service_wire_prefix(svc.name)}/{_rpc_wire_name(rpc)}'
                            f.write(f'   {base_name}_Open_Service : constant String :=\n')
                            f.write(f'     "{base}/open";\n')
                            f.write(f'   {base_name}_Frame_Topic : constant String :=\n')
                            f.write(f'     "{base}/frames";\n')
                            f.write(f'   {base_name}_Cancel_Topic : constant String :=\n')
                            f.write(f'     "{base}/cancel";\n')
                        else:
                            endpoint = f'/pyramid/service/{_service_wire_prefix(svc.name)}/{_rpc_wire_name(rpc)}'
                            f.write(f'   {base_name}_Service : constant String :=\n')
                            f.write(f'     "{endpoint}";\n')
                        f.write(f'\n')

            # -- Msg_To_String utility -----------------------------------------
            f.write(f'   --  -- PCL message utility ------------------------------------\n')
            f.write(f'\n')
            f.write(f'   function Msg_To_String\n')
            f.write(f'     (Data : System.Address;\n')
            f.write(f'      Size : Interfaces.C.unsigned) return String;\n')
            f.write(f'\n')
            f.write(f'   Json_Content_Type : constant String := "application/json";\n')
            f.write(f'   Flatbuffers_Content_Type : constant String := "application/flatbuffers";\n')
            if has_grpc:
                f.write(f'   Grpc_Content_Type : constant String := "application/grpc";\n')
            f.write(f'\n')
            f.write(f'   function Supports_Content_Type (Content_Type : String) return Boolean;\n')
            f.write(f'\n')
            if has_grpc:
                f.write(f'   procedure Configure_Grpc_Library (Path : String);\n')
                f.write(f'   procedure Configure_Grpc_Channel (Channel : String);\n')
                f.write(f'\n')

            # Handler callback declarations
            f.write(f'   --  -- EntityActions handler callbacks ----------------------------\n')
            f.write(f'   --  Supply these callbacks from your component at registration time.\n')
            f.write(f'\n')

            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'   --  {svc_name}\n')
                    current_svc = svc_name

                req_t = _ada_req_type(rpc)
                rsp_t = _ada_rsp_type(rpc)
                handler_name = _rpc_ada_handler(
                    svc_name, rpc, duplicate_rpc_names)

                if rpc.server_streaming:
                    f.write(f'   type {handler_name}_Access is access function\n')
                    f.write(f'     (Request : {req_t}) return {rsp_t};\n')
                else:
                    f.write(f'   type {handler_name}_Access is access procedure\n')
                    f.write(f'     (Request  : in  {req_t};\n')
                    f.write(f'      Response : out {rsp_t});\n')

            f.write(f'\n')
            f.write(f'   type Service_Handlers is record\n')
            for svc_name, rpc in all_rpcs:
                field_name = _rpc_ada_handler_field(
                    svc_name, rpc, duplicate_rpc_names)
                handler_name = _rpc_ada_handler(
                    svc_name, rpc, duplicate_rpc_names)
                f.write(f'      {field_name} : {handler_name}_Access := null;\n')
            f.write(f'   end record;\n')

            f.write('\n')

            # -- PCL binding procedures ----------------------------------------
            f.write(f'   --  -- PCL binding procedures ------------------------------------\n')
            f.write(f'   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.\n')
            f.write(f'   --  Serialisation is handled internally (codec baked at generation time).\n')
            f.write(f'\n')

            # Subscribe helpers for topics (provided side: Ada client subscribes)
            for key in sub_topics:
                ada_name = 'Subscribe_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                f.write(f'   procedure {ada_name}\n')
                f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
                f.write(f'      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;\n')
                f.write(f'      User_Data : System.Address := System.Null_Address;\n')
                f.write(f'      Content_Type : String := "application/json");\n')
                f.write(f'\n')

            # Topic decode helpers so component callbacks do not branch on
            # wire content type.
            for key in all_topics:
                decode_name = 'Decode_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                spec = self._topics.spec(key)
                f.write(f'   function {decode_name}\n')
                f.write(f'     (Msg : access constant Pcl_Bindings.Pcl_Msg)\n')
                f.write(f'      return {spec.ada_payload_type};\n')
                f.write(f'\n')

            f.write(f'   procedure Register_Services\n')
            f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Handlers  : access constant Service_Handlers := null;\n')
            f.write(f'      Content_Type : String := "application/json");\n')
            f.write(f'\n')

            if is_provided:
                for svc in parsed.services:
                    for rpc in _crud_rpcs(svc):
                        decode_name = _rpc_ada_decode_response_name(
                            svc.name, rpc, duplicate_rpc_names)
                        invoke_name = _rpc_ada_invoke_name(
                            svc.name, rpc, duplicate_rpc_names)
                        f.write(f'   function {decode_name}\n')
                        f.write(f'     (Msg : access constant Pcl_Bindings.Pcl_Msg)\n')
                        f.write(f'      return {_ada_rsp_type(rpc)};\n')
                        f.write(f'\n')
                        f.write(f'   --  Invoke via executor transport (transport-agnostic).\n')
                        f.write(f'   procedure {invoke_name}\n')
                        f.write(f'     (Executor  : Pcl_Bindings.Pcl_Executor_Access;\n')
                        f.write(f'      Request   : {_ada_req_type(rpc)};\n')
                        f.write(f'      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;\n')
                        f.write(f'      User_Data : System.Address := System.Null_Address;\n')
                        f.write(f'      Content_Type : String := "application/json");\n')
                        f.write(f'\n')

            # Publish helpers for consumed topics (Ada client publishes to server)
            for key in pub_topics:
                ada_name = 'Publish_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                spec = self._topics.spec(key)
                f.write(f'   procedure {ada_name}\n')
                f.write(f'     (Exec    : Pcl_Bindings.Pcl_Executor_Access;\n')
                f.write(f'      Payload : {spec.ada_payload_type};\n')
                f.write(f'      Content_Type : String := "application/json");\n')
                f.write(f'\n')
                f.write(f'   procedure {ada_name}\n')
                f.write(f'     (Exec    : Pcl_Bindings.Pcl_Executor_Access;\n')
                f.write(f'      Payload : String;\n')
                f.write(f'      Content_Type : String := "application/json");\n')
                f.write(f'\n')

            # Dispatch procedure
            f.write(f'   --  -- Transport integration point ------------------------------\n')
            f.write(f'\n')
            f.write(f'   procedure Dispatch\n')
            f.write(f'     (Handlers      : access constant Service_Handlers := null;\n')
            f.write(f'      Channel       : in  Service_Channel;\n')
            f.write(f'      Request_Buf   : in  System.Address;\n')
            f.write(f'      Request_Size  : in  Natural;\n')
            f.write(f'      Content_Type  : in  String := "application/json";\n')
            f.write(f'      Response_Buf  : out System.Address;\n')
            f.write(f'      Response_Size : out Natural);\n')
            f.write(f'\n')
            f.write(f'end {pkg_name};\n')

