#!/usr/bin/env python3
"""Ada service binding body (.adb) emitter.

Split verbatim from ada_codegen.py (generator refactor plan, phase 4).
"""

from pathlib import Path
from typing import List, Tuple

from proto_parser import ProtoFile, ProtoRpc
from .naming import (
    _short_type,
    _proto_type_to_ada,
    _duplicate_rpc_names,
    _rpc_ada_handler,
    _rpc_ada_channel,
    _rpc_ada_invoke_name,
    _rpc_ada_decode_response_name,
    _rpc_ada_svc_const,
    _rpc_ada_handler_field,
    _rpc_ada_callback_name,
    _crud_rpcs,
    _ada_req_type,
    _ada_rsp_type,
    _is_provided,
    _applicable_topics,
    _grpc_transport_pkg_from_proto,
    _ada_cabi_pkg_from_proto_pkg,
    _ada_cabi_type_name,
    _binding_proto_files,
    _collect_array_schema_bindings,
    _flatbuffers_func_suffix_for_type,
    _flatbuffers_func_suffix_for_stream,
    _ada_name,
)
from .types_gen import AdaTypesGenerator


def _collect_cabi_message_bindings(
        proto_path: Path,
        allowed_type_pkgs: List[str]) -> List[Tuple[str, str, str, str]]:
    """Return (schema_id, native_type, cabi_type, cabi_pkg) for real messages.

    Scalar-wrapper messages are intentionally omitted because they do not have
    generated ``pyramid_<Type>_c`` C structs.
    """
    dm_files = _binding_proto_files(proto_path)
    if not dm_files:
        return []

    aliases = AdaTypesGenerator(dm_files)._find_scalar_wrappers()
    result: List[Tuple[str, str, str, str]] = []
    for pf in dm_files:
        types_pkg = AdaTypesGenerator._ada_pkg_for_file(pf)
        if types_pkg not in allowed_type_pkgs:
            continue
        cabi_pkg = _ada_cabi_pkg_from_proto_pkg(pf.package)
        for msg in pf.messages:
            if msg.name in aliases:
                continue
            schema = msg.name
            native = _ada_name(msg.name)
            cabi = _ada_cabi_type_name(msg.name, pf.package)
            result.append((schema, native, cabi, cabi_pkg))
    return result


def _collect_alias_schema_bindings(
        proto_path: Path,
        allowed_type_pkgs: List[str]) -> List[Tuple[str, str, str, str]]:
    """Return (schema_id, native_alias_type, kind, cabi_pkg) for scalar aliases.

    Scalar-wrapper messages (e.g. ``Identifier`` -> string, unit wrappers ->
    numeric) have no ``pyramid_<Type>_c`` struct; they cross the C-ABI codec
    boundary as ``pyramid_str_t`` (string) or a bare scalar pointer (numeric).
    ``kind`` is ``"string"`` or ``"numeric"``; ``cabi_pkg`` is the module's Cabi
    package (used for ``Pyramid_Str_T`` / ``Dup_Str`` / ``To_Ada_String``).
    """
    dm_files = _binding_proto_files(proto_path)
    if not dm_files:
        return []

    aliases = AdaTypesGenerator(dm_files)._find_scalar_wrappers()
    result: List[Tuple[str, str, str, str]] = []
    for pf in dm_files:
        types_pkg = AdaTypesGenerator._ada_pkg_for_file(pf)
        if types_pkg not in allowed_type_pkgs:
            continue
        cabi_pkg = _ada_cabi_pkg_from_proto_pkg(pf.package)
        for msg in pf.messages:
            if msg.name not in aliases:
                continue
            kind = 'string' if aliases[msg.name] == 'Unbounded_String' else 'numeric'
            result.append((msg.name, _ada_name(msg.name), kind, cabi_pkg))
    return result


class BodyEmitterMixin:
    """Dispatch/handler, codec and PCL binding implementation of the
    generated .adb.  NOTE: _write_body is one large entangled
    emitter method; splitting it further would mean threading
    the output stream and dozens of locals, so it stays whole
    (recorded as follow-up in the refactor plan)."""

    # -- Body (.adb) -----------------------------------------------------------

    def _write_body(self, path: Path, pkg_name: str, parsed: ProtoFile,
                    all_rpcs: List[Tuple[str, ProtoRpc]],
                    type_pkgs: List[str],
                    codec_pkgs: List[str], pf: Path = None):
        is_provided = _is_provided(parsed)
        has_grpc = False
        sub_topics, pub_topics, _ = _applicable_topics(
            parsed, is_provided, self._proto_input, self._topics)
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)
        cabi_bindings = _collect_cabi_message_bindings(
            self._proto_input, type_pkgs)
        alias_bindings = _collect_alias_schema_bindings(
            self._proto_input, type_pkgs)
        array_bindings = _collect_array_schema_bindings(
            cabi_bindings, all_rpcs, sub_topics, pub_topics, self._topics)
        # String aliases need their module Cabi package (Pyramid_Str_T/Dup_Str).
        cabi_pkgs = sorted(
            {pkg for _, _, _, pkg in cabi_bindings}
            | {pkg for _, _, kind, pkg in alias_bindings if kind == 'string'}
            | {pkg for _, _, _, _, pkg in array_bindings})

        # Reference a data-model native type by its declaring Types package.
        # Bare names are unsafe here: they may be hidden by a directly-visible
        # ancestor package of the same simple name (e.g. message
        # ``Sensor_Products`` inside ``...Sensor_Products.Provided``) or be
        # ambiguous because several use-visible packages reuse the name (e.g.
        # ``Generic_Component``). The Types package is derived from the binding's
        # Cabi package, which differs only in its final segment.
        def native_mark(native: str, cabi_pkg: str) -> str:
            return f'{cabi_pkg.rsplit(".", 1)[0]}.Types.{native}'

        # Pointer-conversion package and C free routine for a message binding,
        # keyed off the globally-unique qualified C-ABI record name so that
        # duplicate short names across packages do not collide.
        def msg_ptr_pkg(cabi: str) -> str:
            return cabi.removesuffix('_C') + '_Pointers'

        def msg_free(cabi: str) -> str:
            return 'Free_' + cabi.removesuffix('_C')

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'--  Auto-generated service binding body\n')
            f.write(f'--  Package body: {pkg_name}\n')
            f.write(f'\n')
            f.write(f'with Ada.Strings.Fixed;\n')
            f.write(f'with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;\n')
            f.write(f'with Ada.Unchecked_Conversion;\n')
            f.write(f'with Interfaces.C.Strings;\n')
            f.write(f'with Pcl_Bindings;\n')
            f.write(f'with Pcl_Plugins;\n')
            f.write(f'with System;\n')
            f.write(f'with System.Address_To_Access_Conversions;\n')
            f.write(f'with System.Storage_Elements;\n')
            for tp in type_pkgs:
                f.write(f'with {tp};  use {tp};\n')
            for cp in cabi_pkgs:
                f.write(f'with {cp};  use {cp};\n')
            if has_grpc:
                f.write(f'with {_grpc_transport_pkg_from_proto(parsed)};\n')
            f.write(f'\n')
            f.write(f'package body {pkg_name} is\n')
            f.write(f'   use type System.Address;\n')
            f.write(f'   use type Interfaces.C.unsigned;\n')
            f.write(f'   use type Interfaces.C.Strings.chars_ptr;\n')
            f.write(f'   use type Pcl_Bindings.Pcl_Resp_Cb_Access;\n')
            f.write(f'   use type Pcl_Bindings.Pcl_Status;\n')
            f.write(f'   use type Pcl_Plugins.Pcl_Codec_Const_Access;\n')
            f.write(f'   use type Pcl_Plugins.Pcl_Codec_Decode_Access;\n')
            f.write(f'   use type Pcl_Plugins.Pcl_Codec_Encode_Access;\n')
            f.write(f'   use type Pcl_Plugins.Pcl_Codec_Free_Msg_Access;\n')
            f.write(f'\n')

            # -- Internal helpers ----------------------------------------------
            f.write(f'   function To_Address is new\n')
            f.write(f'     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);\n')
            f.write(f'\n')
            f.write(f'   --  A codec plugin Decode call may allocate this string on a different\n')
            f.write(f'   --  C runtime than this Ada binary (e.g. an MSVC-built plugin DLL vs.\n')
            f.write(f'   --  a GNAT/MinGW executable). Free it through pcl_free, the PCL portable\n')
            f.write(f'   --  allocator, instead of Interfaces.C.Strings.Free (which resolves to\n')
            f.write(f'   --  this binary linked C runtime and would corrupt a foreign heap).\n')
            f.write(f'   procedure Pcl_Free (Ptr : System.Address);\n')
            f.write(f'   pragma Import (C, Pcl_Free, "pcl_free");\n')
            f.write(f'\n')
            f.write(f'   function Pcl_Codec_Registry_Get_At\n')
            f.write(f'     (Registry     : System.Address;\n')
            f.write(f'      Content_Type : Interfaces.C.Strings.chars_ptr;\n')
            f.write(f'      Index        : Interfaces.C.unsigned)\n')
            f.write(f'      return Pcl_Plugins.Pcl_Codec_Const_Access;\n')
            f.write(f'   pragma Import (C, Pcl_Codec_Registry_Get_At,\n')
            f.write(f'                  "pcl_codec_registry_get_at");\n')
            f.write(f'\n')
            if array_bindings:
                # C_Free only ever releases Slice.Ptr, an array a codec plugin
                # allocated in its own Decode call -- possibly a different C
                # runtime than this Ada binary (e.g. an MSVC-built plugin DLL
                # vs. a GNAT/MinGW executable) -- so it must go through
                # pcl_free, PCL's portable allocator, not plain free().
                f.write(f'   procedure C_Free (Ptr : System.Address);\n')
                f.write(f'   pragma Import (C, C_Free, "pcl_free");\n')
                f.write(f'\n')
            f.write(f'   type Service_Handlers_Access is access constant Service_Handlers;\n')
            f.write(f'\n')
            f.write(f'   function To_Handlers is new\n')
            f.write(f'     Ada.Unchecked_Conversion (System.Address, Service_Handlers_Access);\n')
            f.write(f'\n')

            f.write(f'   function Find_Char\n')
            f.write(f'     (Source : String;\n')
            f.write(f'      Ch     : Character;\n')
            f.write(f'      From   : Natural) return Natural\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            f.write(f'      if Source\'Length = 0 or else From > Source\'Last then\n')
            f.write(f'         return 0;\n')
            f.write(f'      end if;\n')
            f.write(f'      for I in From .. Source\'Last loop\n')
            f.write(f'         if Source (I) = Ch then\n')
            f.write(f'            return I;\n')
            f.write(f'         end if;\n')
            f.write(f'      end loop;\n')
            f.write(f'      return 0;\n')
            f.write(f'   end Find_Char;\n')
            f.write(f'\n')

            f.write(f'   function Config_Value\n')
            f.write(f'     (Config_Json : String;\n')
            f.write(f'      Key         : String) return String\n')
            f.write(f'   is\n')
            f.write(f'      Needle : constant String := \'"\' & Key & \'"\' ;\n')
            f.write(f'      Key_Pos : constant Natural :=\n')
            f.write(f'        Ada.Strings.Fixed.Index (Config_Json, Needle);\n')
            f.write(f'      Colon_Pos : Natural;\n')
            f.write(f'      First_Quote : Natural;\n')
            f.write(f'      Second_Quote : Natural;\n')
            f.write(f'   begin\n')
            f.write(f'      if Key_Pos = 0 then\n')
            f.write(f'         return "";\n')
            f.write(f'      end if;\n')
            f.write(f'      Colon_Pos := Find_Char\n')
            f.write(f'        (Config_Json, \':\', Key_Pos + Needle\'Length);\n')
            f.write(f'      if Colon_Pos = 0 then\n')
            f.write(f'         return "";\n')
            f.write(f'      end if;\n')
            f.write(f'      First_Quote := Find_Char (Config_Json, \'"\' , Colon_Pos + 1);\n')
            f.write(f'      if First_Quote = 0 then\n')
            f.write(f'         return "";\n')
            f.write(f'      end if;\n')
            f.write(f'      Second_Quote := Find_Char (Config_Json, \'"\' , First_Quote + 1);\n')
            f.write(f'      if Second_Quote = 0 then\n')
            f.write(f'         return "";\n')
            f.write(f'      end if;\n')
            f.write(f'      if Second_Quote = First_Quote + 1 then\n')
            f.write(f'         return "";\n')
            f.write(f'      end if;\n')
            f.write(f'      return Config_Json (First_Quote + 1 .. Second_Quote - 1);\n')
            f.write(f'   end Config_Value;\n')
            f.write(f'\n')

            f.write(f'   function Config_Route (Config_Json : String) return String is\n')
            f.write(f'      Transport : constant String := Config_Value (Config_Json, "transport");\n')
            f.write(f'   begin\n')
            f.write(f'      if Transport\'Length /= 0 then\n')
            f.write(f'         return Transport;\n')
            f.write(f'      end if;\n')
            f.write(f'      return Config_Value (Config_Json, "route");\n')
            f.write(f'   end Config_Route;\n')
            f.write(f'\n')

            f.write(f'   function Config_Peer (Config_Json : String) return String is\n')
            f.write(f'      Peer : constant String := Config_Value (Config_Json, "peer");\n')
            f.write(f'   begin\n')
            f.write(f'      if Peer\'Length /= 0 then\n')
            f.write(f'         return Peer;\n')
            f.write(f'      end if;\n')
            f.write(f'      return Config_Value (Config_Json, "peer_id");\n')
            f.write(f'   end Config_Peer;\n')
            f.write(f'\n')

            f.write(f'   procedure Apply_Port_Config\n')
            f.write(f'     (Port        : Pcl_Bindings.Pcl_Port_Access;\n')
            f.write(f'      Config_Json : String)\n')
            f.write(f'   is\n')
            f.write(f'      Route : constant String := Config_Route (Config_Json);\n')
            f.write(f'      Peer  : constant String := Config_Peer (Config_Json);\n')
            f.write(f'      Peer_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;\n')
            f.write(f'      Peer_Array : aliased Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.Null_Ptr;\n')
            f.write(f'      Route_Mode : Interfaces.C.unsigned;\n')
            f.write(f'      Status : Pcl_Bindings.Pcl_Status;\n')
            f.write(f'   begin\n')
            f.write(f'      if Config_Json\'Length = 0 then\n')
            f.write(f'         return;\n')
            f.write(f'      end if;\n')
            f.write(f'      if Route\'Length = 0 or else Route = "local" then\n')
            f.write(f'         Route_Mode := Pcl_Bindings.PCL_ROUTE_LOCAL;\n')
            f.write(f'      elsif Route = "remote" then\n')
            f.write(f'         Route_Mode := Pcl_Bindings.PCL_ROUTE_REMOTE;\n')
            f.write(f'      else\n')
            f.write(f'         raise Program_Error with "unsupported transport route: " & Route;\n')
            f.write(f'      end if;\n')
            f.write(f'      if Peer\'Length /= 0 then\n')
            f.write(f'         Peer_C := Interfaces.C.Strings.New_String (Peer);\n')
            f.write(f'         Peer_Array := Peer_C;\n')
            f.write(f'         Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           (Port, Route_Mode, Peer_Array\'Address, 1);\n')
            f.write(f'         Interfaces.C.Strings.Free (Peer_C);\n')
            f.write(f'      else\n')
            f.write(f'         Status := Pcl_Bindings.Port_Set_Route\n')
            f.write(f'           (Port, Route_Mode, System.Null_Address, 0);\n')
            f.write(f'      end if;\n')
            f.write(f'      if Status /= Pcl_Bindings.PCL_OK then\n')
            f.write(f'         raise Program_Error with "PCL port route configuration failed";\n')
            f.write(f'      end if;\n')
            f.write(f'   end Apply_Port_Config;\n')
            f.write(f'\n')

            f.write(f'   procedure Apply_Endpoint_Config\n')
            f.write(f'     (Executor      : Pcl_Bindings.Pcl_Executor_Access;\n')
            f.write(f'      Endpoint_Name : String;\n')
            f.write(f'      Kind          : Pcl_Bindings.Pcl_Endpoint_Kind;\n')
            f.write(f'      Config_Json   : String)\n')
            f.write(f'   is\n')
            f.write(f'      Route_Name : constant String := Config_Route (Config_Json);\n')
            f.write(f'      Peer       : constant String := Config_Peer (Config_Json);\n')
            f.write(f'      Endpoint_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.New_String (Endpoint_Name);\n')
            f.write(f'      Peer_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;\n')
            f.write(f'      Peer_Array : aliased Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.Null_Ptr;\n')
            f.write(f'      Route : aliased Pcl_Bindings.Pcl_Endpoint_Route;\n')
            f.write(f'      Status : Pcl_Bindings.Pcl_Status;\n')
            f.write(f'   begin\n')
            f.write(f'      if Config_Json\'Length = 0 then\n')
            f.write(f'         Interfaces.C.Strings.Free (Endpoint_C);\n')
            f.write(f'         return;\n')
            f.write(f'      end if;\n')
            f.write(f'      Route.Endpoint_Name := Endpoint_C;\n')
            f.write(f'      Route.Endpoint_Kind := Kind;\n')
            f.write(f'      if Route_Name\'Length = 0 or else Route_Name = "local" then\n')
            f.write(f'         Route.Route_Mode := Pcl_Bindings.PCL_ROUTE_LOCAL;\n')
            f.write(f'      elsif Route_Name = "remote" then\n')
            f.write(f'         Route.Route_Mode := Pcl_Bindings.PCL_ROUTE_REMOTE;\n')
            f.write(f'      else\n')
            f.write(f'         Interfaces.C.Strings.Free (Endpoint_C);\n')
            f.write(f'         raise Program_Error with "unsupported transport route: " & Route_Name;\n')
            f.write(f'      end if;\n')
            f.write(f'      if Peer\'Length /= 0 then\n')
            f.write(f'         Peer_C := Interfaces.C.Strings.New_String (Peer);\n')
            f.write(f'         Peer_Array := Peer_C;\n')
            f.write(f'         Route.Peer_Ids := Peer_Array\'Address;\n')
            f.write(f'         Route.Peer_Count := 1;\n')
            f.write(f'      else\n')
            f.write(f'         Route.Peer_Ids := System.Null_Address;\n')
            f.write(f'         Route.Peer_Count := 0;\n')
            f.write(f'      end if;\n')
            f.write(f'      Status := Pcl_Bindings.Set_Endpoint_Route (Executor, Route\'Access);\n')
            f.write(f'      if Peer_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'         Interfaces.C.Strings.Free (Peer_C);\n')
            f.write(f'      end if;\n')
            f.write(f'      Interfaces.C.Strings.Free (Endpoint_C);\n')
            f.write(f'      if Status /= Pcl_Bindings.PCL_OK then\n')
            f.write(f'         raise Program_Error with "PCL endpoint route configuration failed";\n')
            f.write(f'      end if;\n')
            f.write(f'   end Apply_Endpoint_Config;\n')
            f.write(f'\n')
            for schema, native, cabi, _pkg in cabi_bindings:
                del schema
                ptr_pkg = msg_ptr_pkg(cabi)
                f.write(f'   package {ptr_pkg} is new\n')
                f.write(f'     System.Address_To_Access_Conversions'
                        f' ({native_mark(native, _pkg)});\n')
                f.write(f'\n')
            for schema, native, _kind, _pkg in alias_bindings:
                del schema
                # Only string aliases deref via a pointer package; numeric
                # aliases pass the scalar address straight through. (And the
                # native alias name, e.g. Length, can clash with use-visible
                # functions, so avoid instantiating one unless needed.)
                if _kind != 'string':
                    continue
                ptr_pkg = f'{native}_Pointers'
                f.write(f'   package {ptr_pkg} is new\n')
                f.write(f'     System.Address_To_Access_Conversions'
                        f' ({native_mark(native, _pkg)});\n')
                f.write(f'\n')

            f.write(f'   function Handler_Address\n')
            f.write(f'     (Handlers : access constant Service_Handlers) return System.Address is\n')
            f.write(f'   begin\n')
            f.write(f'      if Handlers = null then\n')
            f.write(f'         return System.Null_Address;\n')
            f.write(f'      end if;\n')
            f.write(f'\n')
            f.write(f"      return Handlers.all'Address;\n")
            f.write(f'   end Handler_Address;\n')
            f.write(f'\n')

            # -- Msg_To_String -------------------------------------------------
            f.write(f'   function Msg_To_String\n')
            f.write(f'     (Data : System.Address;\n')
            f.write(f'      Size : Interfaces.C.unsigned) return String\n')
            f.write(f'   is\n')
            f.write(f'      use System.Storage_Elements;\n')
            f.write(f'      type Char_Array is array (1 .. Natural (Size)) of Character;\n')
            f.write(f'      pragma Pack (Char_Array);\n')
            f.write(f'      Chars : Char_Array;\n')
            f.write(f"      for Chars'Address use Data;\n")
            f.write(f'      pragma Import (Ada, Chars);\n')
            f.write(f'   begin\n')
            f.write(f'      return String (Chars);\n')
            f.write(f'   end Msg_To_String;\n')
            f.write(f'\n')

            f.write(f'   function Registry_Has_Codec (Content_Type : String) return Boolean is\n')
            f.write(f'      Content_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.New_String (Content_Type);\n')
            f.write(f'      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;\n')
            f.write(f'   begin\n')
            f.write(f'      if Content_Type = "" then\n')
            f.write(f'         Interfaces.C.Strings.Free (Content_C);\n')
            f.write(f'         return False;\n')
            f.write(f'      end if;\n')
            f.write(f'      Codec := Pcl_Plugins.Pcl_Codec_Registry_Get\n')
            f.write(f'        (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C);\n')
            f.write(f'      Interfaces.C.Strings.Free (Content_C);\n')
            f.write(f'      return Codec /= null;\n')
            f.write(f'   exception\n')
            f.write(f'      when others =>\n')
            f.write(f'         if Content_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'            Interfaces.C.Strings.Free (Content_C);\n')
            f.write(f'         end if;\n')
            f.write(f'         return False;\n')
            f.write(f'   end Registry_Has_Codec;\n')
            f.write(f'\n')

            # Fail-closed gate: every encode/decode requires a registered codec
            # plugin for the content type. With no plugin the facade raises
            # instead of falling back to a built-in codec (matches C++).
            f.write(f'   procedure Require_Codec (Content_Type : String) is\n')
            f.write(f'      Effective : constant String :=\n')
            f.write(f'        (if Content_Type = "" then Json_Content_Type else Content_Type);\n')
            f.write(f'   begin\n')
            if has_grpc:
                f.write(f'      if Effective = Grpc_Content_Type then\n')
                f.write(f'         return;  --  gRPC target carries its own transport+codec\n')
                f.write(f'      end if;\n')
            f.write(f'      if not Registry_Has_Codec (Effective) then\n')
            f.write(f'         raise Program_Error with\n')
            f.write(f'           "fail-closed: no codec plugin registered for content type "\n')
            f.write(f'           & Effective;\n')
            f.write(f'      end if;\n')
            f.write(f'   end Require_Codec;\n')
            f.write(f'\n')

            f.write(f'   function Try_Cabi_Registry_Encode\n')
            f.write(f'     (Codec     : Pcl_Plugins.Pcl_Codec_Const_Access;\n')
            f.write(f'      Schema_C  : Interfaces.C.Strings.chars_ptr;\n')
            f.write(f'      Schema_Id : String;\n')
            f.write(f'      Value     : System.Address;\n')
            f.write(f'      Msg       : access Pcl_Bindings.Pcl_Msg)\n')
            f.write(f'      return Pcl_Bindings.Pcl_Status\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            f.write(f'      if Codec = null or else Codec.all.Encode = null then\n')
            f.write(f'         return Pcl_Bindings.PCL_ERR_INVALID;\n')
            f.write(f'      end if;\n')
            for schema, native, cabi, _pkg in cabi_bindings:
                ptr_pkg = msg_ptr_pkg(cabi)
                f.write(f'      if Schema_Id = "{schema}" then\n')
                f.write(f'         declare\n')
                f.write(f'            Native : constant {ptr_pkg}.Object_Pointer :=\n')
                f.write(f'              {ptr_pkg}.To_Pointer (Value);\n')
                f.write(f'            C_Value : aliased {cabi} := (others => <>);\n')
                f.write(f'            Status : Pcl_Bindings.Pcl_Status :=\n')
                f.write(f'              Pcl_Bindings.PCL_ERR_INVALID;\n')
                f.write(f'         begin\n')
                f.write(f'            if Value = System.Null_Address then\n')
                f.write(f'               return Pcl_Bindings.PCL_ERR_INVALID;\n')
                f.write(f'            end if;\n')
                f.write(f'            To_C (Native.all, C_Value);\n')
                f.write(f'            Status := Codec.all.Encode.all\n')
                f.write(f"              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);\n")
                f.write(f"            {msg_free(cabi)} (C_Value'Access);\n")
                f.write(f'            return Status;\n')
                f.write(f'         end;\n')
                f.write(f'      end if;\n')
            # Scalar aliases: string -> pyramid_str_t, numeric -> scalar pointer.
            for schema, native, kind, alias_pkg in alias_bindings:
                ptr_pkg = f'{native}_Pointers'
                f.write(f'      if Schema_Id = "{schema}" then\n')
                f.write(f'         if Value = System.Null_Address then\n')
                f.write(f'            return Pcl_Bindings.PCL_ERR_INVALID;\n')
                f.write(f'         end if;\n')
                if kind == 'string':
                    f.write(f'         declare\n')
                    f.write(f'            Native : constant {ptr_pkg}.Object_Pointer :=\n')
                    f.write(f'              {ptr_pkg}.To_Pointer (Value);\n')
                    f.write(f'            S : constant String := To_String (Native.all);\n')
                    f.write(f'            C_Value : aliased {alias_pkg}.Pyramid_Str_T;\n')
                    f.write(f'            Status : Pcl_Bindings.Pcl_Status;\n')
                    f.write(f'         begin\n')
                    f.write(f'            C_Value.Ptr := Interfaces.C.Strings.New_String (S);\n')
                    f.write(f"            C_Value.Len := Interfaces.C.unsigned (S'Length);\n")
                    f.write(f'            Status := Codec.all.Encode.all\n')
                    f.write(f"              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);\n")
                    f.write(f'            Interfaces.C.Strings.Free (C_Value.Ptr);\n')
                    f.write(f'            return Status;\n')
                    f.write(f'         end;\n')
                else:
                    f.write(f'         return Codec.all.Encode.all\n')
                    f.write(f'           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);\n')
                f.write(f'      end if;\n')
            f.write(f'      return Pcl_Bindings.PCL_ERR_NOT_FOUND;\n')
            f.write(f'   end Try_Cabi_Registry_Encode;\n')
            f.write(f'\n')

            f.write(f'   function Try_Cabi_Registry_Decode\n')
            f.write(f'     (Codec     : Pcl_Plugins.Pcl_Codec_Const_Access;\n')
            f.write(f'      Schema_C  : Interfaces.C.Strings.chars_ptr;\n')
            f.write(f'      Schema_Id : String;\n')
            f.write(f'      Msg       : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      Value     : System.Address)\n')
            f.write(f'      return Pcl_Bindings.Pcl_Status\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            f.write(f'      if Codec = null or else Codec.all.Decode = null then\n')
            f.write(f'         return Pcl_Bindings.PCL_ERR_INVALID;\n')
            f.write(f'      end if;\n')
            for schema, native, cabi, _pkg in cabi_bindings:
                ptr_pkg = msg_ptr_pkg(cabi)
                f.write(f'      if Schema_Id = "{schema}" then\n')
                f.write(f'         declare\n')
                f.write(f'            Native : constant {ptr_pkg}.Object_Pointer :=\n')
                f.write(f'              {ptr_pkg}.To_Pointer (Value);\n')
                f.write(f'            C_Value : aliased {cabi} := (others => <>);\n')
                f.write(f'            Status : Pcl_Bindings.Pcl_Status :=\n')
                f.write(f'              Pcl_Bindings.PCL_ERR_INVALID;\n')
                f.write(f'         begin\n')
                f.write(f'            if Value = System.Null_Address then\n')
                f.write(f'               return Pcl_Bindings.PCL_ERR_INVALID;\n')
                f.write(f'            end if;\n')
                f.write(f'            Status := Codec.all.Decode.all\n')
                f.write(f"              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);\n")
                f.write(f'            if Status = Pcl_Bindings.PCL_OK then\n')
                f.write(f'               From_C (C_Value, Native.all);\n')
                f.write(f'            end if;\n')
                f.write(f"            {msg_free(cabi)} (C_Value'Access);\n")
                f.write(f'            return Status;\n')
                f.write(f'         end;\n')
                f.write(f'      end if;\n')
            # Scalar aliases: string <- pyramid_str_t, numeric <- scalar pointer.
            for schema, native, kind, alias_pkg in alias_bindings:
                ptr_pkg = f'{native}_Pointers'
                f.write(f'      if Schema_Id = "{schema}" then\n')
                f.write(f'         if Value = System.Null_Address then\n')
                f.write(f'            return Pcl_Bindings.PCL_ERR_INVALID;\n')
                f.write(f'         end if;\n')
                if kind == 'string':
                    f.write(f'         declare\n')
                    f.write(f'            Native : constant {ptr_pkg}.Object_Pointer :=\n')
                    f.write(f'              {ptr_pkg}.To_Pointer (Value);\n')
                    f.write(f'            C_Value : aliased {alias_pkg}.Pyramid_Str_T;\n')
                    f.write(f'            Status : Pcl_Bindings.Pcl_Status;\n')
                    f.write(f'         begin\n')
                    f.write(f'            Status := Codec.all.Decode.all\n')
                    f.write(f"              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);\n")
                    f.write(f'            if Status = Pcl_Bindings.PCL_OK then\n')
                    f.write(f'               if C_Value.Ptr = Interfaces.C.Strings.Null_Ptr then\n')
                    f.write(f'                  Native.all := Null_Unbounded_String;\n')
                    f.write(f'               else\n')
                    f.write(f'                  Native.all := To_Unbounded_String\n')
                    f.write(f'                    (Interfaces.C.Strings.Value\n')
                    f.write(f'                       (C_Value.Ptr,\n')
                    f.write(f'                        Interfaces.C.size_t (C_Value.Len)));\n')
                    f.write(f'               end if;\n')
                    f.write(f'            end if;\n')
                    f.write(f'            if C_Value.Ptr /= Interfaces.C.Strings.Null_Ptr then\n')
                    f.write(f'               Pcl_Free (To_Address (C_Value.Ptr));\n')
                    f.write(f'            end if;\n')
                    f.write(f'            return Status;\n')
                    f.write(f'         end;\n')
                else:
                    f.write(f'         return Codec.all.Decode.all\n')
                    f.write(f'           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);\n')
                f.write(f'      end if;\n')
            f.write(f'      return Pcl_Bindings.PCL_ERR_NOT_FOUND;\n')
            f.write(f'   end Try_Cabi_Registry_Decode;\n')
            f.write(f'\n')

            f.write(f'   function Try_Registry_Encode\n')
            f.write(f'     (Content_Type : String;\n')
            f.write(f'      Schema_Id    : String;\n')
            f.write(f'      Value        : System.Address;\n')
            f.write(f'      Wire         : out Unbounded_String) return Boolean\n')
            f.write(f'   is\n')
            f.write(f'      Effective : constant String :=\n')
            f.write(f'        (if Content_Type = "" then Json_Content_Type else Content_Type);\n')
            f.write(f'      Content_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.New_String (Effective);\n')
            f.write(f'      Schema_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.New_String (Schema_Id);\n')
            f.write(f'      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;\n')
            f.write(f'      Msg   : aliased Pcl_Bindings.Pcl_Msg :=\n')
            f.write(f'        (Data      => System.Null_Address,\n')
            f.write(f'         Size      => 0,\n')
            f.write(f'         Type_Name => Interfaces.C.Strings.Null_Ptr);\n')
            f.write(f'      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;\n')
            f.write(f'      Index : Interfaces.C.unsigned := 0;\n')
            f.write(f'   begin\n')
            f.write(f'      Wire := Null_Unbounded_String;\n')
            f.write(f'      if Effective = "" then\n')
            f.write(f'         Interfaces.C.Strings.Free (Content_C);\n')
            f.write(f'         Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'         return False;\n')
            f.write(f'      end if;\n')
            # Cross-language codec plugins consume the frozen C representation.
            # If Try_Cabi cannot marshal this schema, callers fail closed rather
            # than falling back to generated Ada codec sources.
            f.write(f'      loop\n')
            f.write(f'         Codec := Pcl_Codec_Registry_Get_At\n')
            f.write(f'           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);\n')
            f.write(f'         exit when Codec = null;\n')
            f.write(f'         if Codec.all.Encode /= null then\n')
            f.write(f'            Msg :=\n')
            f.write(f'              (Data      => System.Null_Address,\n')
            f.write(f'               Size      => 0,\n')
            f.write(f'               Type_Name => Interfaces.C.Strings.Null_Ptr);\n')
            f.write(f'            Status := Try_Cabi_Registry_Encode\n')
            f.write(f"              (Codec, Schema_C, Schema_Id, Value, Msg'Access);\n")
            f.write(f'            if Status = Pcl_Bindings.PCL_OK then\n')
            f.write(f'               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then\n')
            f.write(f'                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));\n')
            f.write(f'               end if;\n')
            f.write(f'               if Codec.all.Free_Msg /= null then\n')
            f.write(f"                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);\n")
            f.write(f'               end if;\n')
            f.write(f'               Interfaces.C.Strings.Free (Content_C);\n')
            f.write(f'               Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'               return True;\n')
            f.write(f'            end if;\n')
            f.write(f'            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then\n')
            f.write(f"               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);\n")
            f.write(f'            end if;\n')
            f.write(f'         end if;\n')
            f.write(f"         exit when Index = Interfaces.C.unsigned'Last;\n")
            f.write(f'         Index := Index + 1;\n')
            f.write(f'      end loop;\n')
            f.write(f'      Interfaces.C.Strings.Free (Content_C);\n')
            f.write(f'      Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'      return False;\n')
            f.write(f'   exception\n')
            f.write(f'      when others =>\n')
            f.write(f'         if Content_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'            Interfaces.C.Strings.Free (Content_C);\n')
            f.write(f'         end if;\n')
            f.write(f'         if Schema_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'            Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'         end if;\n')
            f.write(f'         return False;\n')
            f.write(f'   end Try_Registry_Encode;\n')
            f.write(f'\n')

            f.write(f'   function Try_Registry_Decode\n')
            f.write(f'     (Msg       : access constant Pcl_Bindings.Pcl_Msg;\n')
            f.write(f'      Schema_Id : String;\n')
            f.write(f'      Value     : System.Address) return Boolean\n')
            f.write(f'   is\n')
            f.write(f'      Schema_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.New_String (Schema_Id);\n')
            f.write(f'      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;\n')
            f.write(f'      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;\n')
            f.write(f'      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;\n')
            f.write(f'      Index : Interfaces.C.unsigned := 0;\n')
            f.write(f'   begin\n')
            f.write(f'      if Msg = null then\n')
            f.write(f'         Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'         return False;\n')
            f.write(f'      end if;\n')
            f.write(f'      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);\n')
            f.write(f'      end if;\n')
            # Only C-ABI-marshalled types may cross to the plugin. Schemas
            # without marshalling fail closed at the caller.
            f.write(f'      loop\n')
            f.write(f'         Codec := Pcl_Codec_Registry_Get_At\n')
            f.write(f'           (Pcl_Plugins.Pcl_Codec_Registry_Default,\n')
            f.write(f'            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),\n')
            f.write(f'            Index);\n')
            f.write(f'         exit when Codec = null;\n')
            f.write(f'         if Codec.all.Decode /= null then\n')
            f.write(f'            Status := Try_Cabi_Registry_Decode\n')
            f.write(f'              (Codec, Schema_C, Schema_Id, Msg, Value);\n')
            f.write(f'            if Status = Pcl_Bindings.PCL_OK then\n')
            f.write(f'               Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'               if Type_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'                  Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'               end if;\n')
            f.write(f'               return True;\n')
            f.write(f'            end if;\n')
            f.write(f'         end if;\n')
            f.write(f"         exit when Index = Interfaces.C.unsigned'Last;\n")
            f.write(f'         Index := Index + 1;\n')
            f.write(f'      end loop;\n')
            f.write(f'      Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'      if Type_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'      end if;\n')
            f.write(f'      return False;\n')
            f.write(f'   exception\n')
            f.write(f'      when others =>\n')
            f.write(f'         if Schema_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'            Interfaces.C.Strings.Free (Schema_C);\n')
            f.write(f'         end if;\n')
            f.write(f'         if Type_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'            Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         end if;\n')
            f.write(f'         return False;\n')
            f.write(f'   end Try_Registry_Decode;\n')
            f.write(f'\n')

            f.write(f'   function Try_Registry_Decode_Raw\n')
            f.write(f'     (Content_Type : String;\n')
            f.write(f'      Data         : System.Address;\n')
            f.write(f'      Size         : Natural;\n')
            f.write(f'      Schema_Id    : String;\n')
            f.write(f'      Value        : System.Address) return Boolean\n')
            f.write(f'   is\n')
            f.write(f'      Effective : constant String :=\n')
            f.write(f'        (if Content_Type = "" then Json_Content_Type else Content_Type);\n')
            f.write(f'      Type_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.New_String (Effective);\n')
            f.write(f'      Msg : aliased Pcl_Bindings.Pcl_Msg :=\n')
            f.write(f'        (Data      => Data,\n')
            f.write(f'         Size      => Interfaces.C.unsigned (Size),\n')
            f.write(f'         Type_Name => Type_C);\n')
            f.write(f'      Ok : Boolean := False;\n')
            f.write(f'   begin\n')
            f.write(f'      Ok := Try_Registry_Decode (Msg\'Access, Schema_Id, Value);\n')
            f.write(f'      Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'      return Ok;\n')
            f.write(f'   exception\n')
            f.write(f'      when others =>\n')
            f.write(f'         if Type_C /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'            Interfaces.C.Strings.Free (Type_C);\n')
            f.write(f'         end if;\n')
            f.write(f'         return False;\n')
            f.write(f'   end Try_Registry_Decode_Raw;\n')
            f.write(f'\n')

            for schema, array_type, native, cabi, cabi_pkg in array_bindings:
                c_array_type = f'{array_type}_C_Array'
                f.write(f'   function Try_Registry_Encode_{array_type}\n')
                f.write(f'     (Content_Type : String;\n')
                f.write(f'      Payload      : {array_type};\n')
                f.write(f'      Wire         : out Unbounded_String) return Boolean\n')
                f.write(f'   is\n')
                f.write(f'      Effective : constant String :=\n')
                f.write(f'        (if Content_Type = "" then Json_Content_Type else Content_Type);\n')
                f.write(f'      Content_C : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'        Interfaces.C.Strings.New_String (Effective);\n')
                f.write(f'      Schema_C : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'        Interfaces.C.Strings.New_String ("{schema}");\n')
                f.write(f'      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;\n')
                f.write(f'      Msg   : aliased Pcl_Bindings.Pcl_Msg :=\n')
                f.write(f'        (Data      => System.Null_Address,\n')
                f.write(f'         Size      => 0,\n')
                f.write(f'         Type_Name => Interfaces.C.Strings.Null_Ptr);\n')
                f.write(f'      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;\n')
                f.write(f'      Index : Interfaces.C.unsigned := 0;\n')
                f.write(f'   begin\n')
                f.write(f'      Wire := Null_Unbounded_String;\n')
                f.write(f'      loop\n')
                f.write(f'         Codec := Pcl_Codec_Registry_Get_At\n')
                f.write(f'           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);\n')
                f.write(f'         exit when Codec = null;\n')
                f.write(f'         if Codec.all.Encode /= null then\n')
                f.write(f'            Msg :=\n')
                f.write(f'              (Data      => System.Null_Address,\n')
                f.write(f'               Size      => 0,\n')
                f.write(f'               Type_Name => Interfaces.C.Strings.Null_Ptr);\n')
                f.write(f'            declare\n')
                f.write(f'               Slice : aliased {cabi_pkg}.Pyramid_Slice_T :=\n')
                f.write(f'                 (Ptr => System.Null_Address, Len => 0);\n')
                f.write(f'            begin\n')
                f.write(f"               if Payload'Length = 0 then\n")
                f.write(f'                  Status := Codec.all.Encode.all\n')
                f.write(f"                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);\n")
                f.write(f'               else\n')
                f.write(f'                  declare\n')
                f.write(f"                     Count : constant Natural := Payload'Length;\n")
                f.write(f'                     type {c_array_type} is array (Positive range 1 .. Count)\n')
                f.write(f'                       of aliased {cabi};\n')
                f.write(f'                     pragma Convention (C, {c_array_type});\n')
                f.write(f'                     Values : {c_array_type} := (others => (others => <>));\n')
                f.write(f'                  begin\n')
                f.write(f'                     for Offset in 0 .. Count - 1 loop\n')
                f.write(f"                        To_C (Payload (Payload'First + Offset),\n")
                f.write(f'                              Values (Values\'First + Offset));\n')
                f.write(f'                     end loop;\n')
                f.write(f"                     Slice.Ptr := Values (Values'First)'Address;\n")
                f.write(f'                     Slice.Len := Interfaces.C.unsigned (Count);\n')
                f.write(f'                     Status := Codec.all.Encode.all\n')
                f.write(f"                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);\n")
                f.write(f"                     for I in Values'Range loop\n")
                f.write(f"                        {msg_free(cabi)} (Values (I)'Access);\n")
                f.write(f'                     end loop;\n')
                f.write(f'                  end;\n')
                f.write(f'               end if;\n')
                f.write(f'            end;\n')
                f.write(f'            if Status = Pcl_Bindings.PCL_OK then\n')
                f.write(f'               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then\n')
                f.write(f'                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));\n')
                f.write(f'               end if;\n')
                f.write(f'               if Codec.all.Free_Msg /= null then\n')
                f.write(f"                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);\n")
                f.write(f'               end if;\n')
                f.write(f'               Interfaces.C.Strings.Free (Content_C);\n')
                f.write(f'               Interfaces.C.Strings.Free (Schema_C);\n')
                f.write(f'               return True;\n')
                f.write(f'            end if;\n')
                f.write(f'            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then\n')
                f.write(f"               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);\n")
                f.write(f'            end if;\n')
                f.write(f'         end if;\n')
                f.write(f"         exit when Index = Interfaces.C.unsigned'Last;\n")
                f.write(f'         Index := Index + 1;\n')
                f.write(f'      end loop;\n')
                f.write(f'      Interfaces.C.Strings.Free (Content_C);\n')
                f.write(f'      Interfaces.C.Strings.Free (Schema_C);\n')
                f.write(f'      return False;\n')
                f.write(f'   exception\n')
                f.write(f'      when others =>\n')
                f.write(f'         if Content_C /= Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'            Interfaces.C.Strings.Free (Content_C);\n')
                f.write(f'         end if;\n')
                f.write(f'         if Schema_C /= Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'            Interfaces.C.Strings.Free (Schema_C);\n')
                f.write(f'         end if;\n')
                f.write(f'         return False;\n')
                f.write(f'   end Try_Registry_Encode_{array_type};\n')
                f.write(f'\n')

                f.write(f'   function Registry_Decode_{array_type}\n')
                f.write(f'     (Msg : access constant Pcl_Bindings.Pcl_Msg)\n')
                f.write(f'      return {array_type}\n')
                f.write(f'   is\n')
                f.write(f'      Empty : {array_type} (1 .. 0);\n')
                f.write(f'      Schema_C : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'        Interfaces.C.Strings.New_String ("{schema}");\n')
                f.write(f'      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;\n')
                f.write(f'      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;\n')
                f.write(f'      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;\n')
                f.write(f'      Index : Interfaces.C.unsigned := 0;\n')
                f.write(f'      Decoded : Boolean := False;\n')
                f.write(f'      Slice : aliased {cabi_pkg}.Pyramid_Slice_T :=\n')
                f.write(f'        (Ptr => System.Null_Address, Len => 0);\n')
                f.write(f'   begin\n')
                f.write(f'      if Msg = null then\n')
                f.write(f'         raise Program_Error with "codec registry decode failed for schema {schema}";\n')
                f.write(f'      end if;\n')
                f.write(f'      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);\n')
                f.write(f'      end if;\n')
                f.write(f'      loop\n')
                f.write(f'         Codec := Pcl_Codec_Registry_Get_At\n')
                f.write(f'           (Pcl_Plugins.Pcl_Codec_Registry_Default,\n')
                f.write(f'            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),\n')
                f.write(f'            Index);\n')
                f.write(f'         exit when Codec = null;\n')
                f.write(f'         if Codec.all.Decode /= null then\n')
                f.write(f'            Slice := (Ptr => System.Null_Address, Len => 0);\n')
                f.write(f'            Status := Codec.all.Decode.all\n')
                f.write(f"              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);\n")
                f.write(f'            if Status = Pcl_Bindings.PCL_OK then\n')
                f.write(f'               Decoded := True;\n')
                f.write(f'               exit;\n')
                f.write(f'            end if;\n')
                f.write(f'            if Slice.Ptr /= System.Null_Address then\n')
                f.write(f'               C_Free (Slice.Ptr);\n')
                f.write(f'               Slice.Ptr := System.Null_Address;\n')
                f.write(f'            end if;\n')
                f.write(f'         end if;\n')
                f.write(f"         exit when Index = Interfaces.C.unsigned'Last;\n")
                f.write(f'         Index := Index + 1;\n')
                f.write(f'      end loop;\n')
                f.write(f'      if not Decoded then\n')
                f.write(f'         raise Program_Error with "codec registry decode failed for schema {schema}";\n')
                f.write(f'      end if;\n')
                f.write(f'      Interfaces.C.Strings.Free (Schema_C);\n')
                f.write(f'      Schema_C := Interfaces.C.Strings.Null_Ptr;\n')
                f.write(f'      if Type_C /= Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'         Interfaces.C.Strings.Free (Type_C);\n')
                f.write(f'         Type_C := Interfaces.C.Strings.Null_Ptr;\n')
                f.write(f'      end if;\n')
                f.write(f'      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then\n')
                f.write(f'         if Slice.Ptr /= System.Null_Address then\n')
                f.write(f'            C_Free (Slice.Ptr);\n')
                f.write(f'            Slice.Ptr := System.Null_Address;\n')
                f.write(f'         end if;\n')
                f.write(f'         return Empty;\n')
                f.write(f'      end if;\n')
                f.write(f'      declare\n')
                f.write(f'         Count : constant Natural := Natural (Slice.Len);\n')
                f.write(f'         type {c_array_type} is array (Positive range 1 .. Count)\n')
                f.write(f'           of aliased {cabi};\n')
                f.write(f'         pragma Convention (C, {c_array_type});\n')
                f.write(f'         Values : {c_array_type};\n')
                f.write(f"         for Values'Address use Slice.Ptr;\n")
                f.write(f'         pragma Import (Ada, Values);\n')
                f.write(f'         Result : {array_type} (1 .. Count);\n')
                f.write(f'      begin\n')
                f.write(f'         for I in 1 .. Count loop\n')
                f.write(f'            From_C (Values (I), Result (I));\n')
                f.write(f"            {msg_free(cabi)} (Values (I)'Access);\n")
                f.write(f'         end loop;\n')
                f.write(f'         C_Free (Slice.Ptr);\n')
                f.write(f'         Slice.Ptr := System.Null_Address;\n')
                f.write(f'         return Result;\n')
                f.write(f'      end;\n')
                f.write(f'   exception\n')
                f.write(f'      when others =>\n')
                f.write(f'         if Schema_C /= Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'            Interfaces.C.Strings.Free (Schema_C);\n')
                f.write(f'         end if;\n')
                f.write(f'         if Type_C /= Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'            Interfaces.C.Strings.Free (Type_C);\n')
                f.write(f'         end if;\n')
                f.write(f'         if Slice.Ptr /= System.Null_Address then\n')
                f.write(f'            C_Free (Slice.Ptr);\n')
                f.write(f'         end if;\n')
                f.write(f'         raise;\n')
                f.write(f'   end Registry_Decode_{array_type};\n')
                f.write(f'\n')

            if has_grpc:
                f.write(f'   package Grpc_Transport renames {_grpc_transport_pkg_from_proto(parsed)};\n')
                f.write(f'   Grpc_Channel : Unbounded_String := Null_Unbounded_String;\n')
            f.write(f'\n')

            f.write(f'   function Supports_Content_Type (Content_Type : String) return Boolean is\n')
            f.write(f'   begin\n')
            f.write(f'      return Registry_Has_Codec\n')
            f.write(f'        ((if Content_Type = "" then Json_Content_Type else Content_Type))')
            if has_grpc:
                f.write(f'\n')
                f.write(f'        or else Content_Type = Grpc_Content_Type;\n')
            else:
                f.write(f';\n')
            f.write(f'   end Supports_Content_Type;\n')
            f.write(f'\n')
            if has_grpc:
                f.write(f'   procedure Configure_Grpc_Library (Path : String) is\n')
                f.write(f'   begin\n')
                f.write(f'      Grpc_Transport.Configure_Library (Path);\n')
                f.write(f'   end Configure_Grpc_Library;\n')
                f.write(f'\n')
                f.write(f'   procedure Configure_Grpc_Channel (Channel : String) is\n')
                f.write(f'   begin\n')
                f.write(f'      Grpc_Channel := To_Unbounded_String (Channel);\n')
                f.write(f'   end Configure_Grpc_Channel;\n')
                f.write(f'\n')
            f.write(f'   function Message_Content_Type\n')
            f.write(f'     (Msg : access constant Pcl_Bindings.Pcl_Msg) return String is\n')
            f.write(f'   begin\n')
            f.write(f'      if Msg = null or else Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then\n')
            f.write(f'         return Json_Content_Type;\n')
            f.write(f'      end if;\n')
            f.write(f'      return Interfaces.C.Strings.Value (Msg.Type_Name);\n')
            f.write(f'   end Message_Content_Type;\n')
            f.write(f'\n')
            if has_grpc:
                f.write(f'   procedure Emit_Invoke_Response\n')
                f.write(f'     (Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;\n')
                f.write(f'      User_Data : System.Address;\n')
                f.write(f'      Payload   : String) is\n')
                f.write(f'      Payload_Bytes : aliased constant String := Payload;\n')
                f.write(f'      Type_Name : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'        Interfaces.C.Strings.New_String (Json_Content_Type);\n')
                f.write(f'      Msg : aliased Pcl_Bindings.Pcl_Msg;\n')
                f.write(f'   begin\n')
                f.write(f"      Msg.Data :=\n")
                f.write(f"        (if Payload_Bytes'Length = 0\n")
                f.write(f"         then System.Null_Address\n")
                f.write(f"         else Payload_Bytes (Payload_Bytes'First)'Address);\n")
                f.write(f"      Msg.Size := Interfaces.C.unsigned (Payload_Bytes'Length);\n")
                f.write(f'      Msg.Type_Name := Type_Name;\n')
                f.write(f'      if Callback /= null then\n')
                f.write(f"         Callback (Msg'Access, User_Data);\n")
                f.write(f'      end if;\n')
                f.write(f'      Interfaces.C.Strings.Free (Type_Name);\n')
                f.write(f'   end Emit_Invoke_Response;\n')
                f.write(f'\n')

            def write_payload_decode_function(func_name: str, payload_type: str,
                                              short_type: str, is_array: bool,
                                              fb_suffix: str,
                                              schema_id: str) -> None:
                f.write(f'   function {func_name}\n')
                f.write(f'     (Msg : access constant Pcl_Bindings.Pcl_Msg)\n')
                f.write(f'      return {payload_type}\n')
                f.write(f'   is\n')
                f.write(f'      Empty : {payload_type} (1 .. 0);\n') if is_array else None
                f.write(f'      Payload : constant String :=\n')
                f.write(f'        (if Msg = null or else Msg.Data = System.Null_Address\n')
                f.write(f'         then ""\n')
                f.write(f'         else Msg_To_String (Msg.Data, Msg.Size));\n')
                f.write(f'      Content_Type : constant String := Message_Content_Type (Msg);\n')
                f.write(f'   begin\n')
                f.write(f'      if Payload = "" then\n')
                if is_array:
                    f.write(f'         return Empty;\n')
                elif short_type == 'Identifier':
                    f.write(f'         return Null_Unbounded_String;\n')
                else:
                    f.write(f'         declare\n')
                    f.write(f'            Result : {payload_type};\n')
                    f.write(f'         begin\n')
                    f.write(f'            return Result;\n')
                    f.write(f'         end;\n')
                f.write(f'      end if;\n')
                f.write(f'      Require_Codec (Content_Type);  --  fail closed if no plugin\n')
                f.write(f'\n')
                if is_array:
                    f.write(f'      return Registry_Decode_{payload_type} (Msg);\n')
                else:
                    f.write(f'      declare\n')
                    f.write(f'         Result : {payload_type};\n')
                    f.write(f'      begin\n')
                    f.write(f'         if Try_Registry_Decode (Msg, "{schema_id}", Result\'Address) then\n')
                    f.write(f'            return Result;\n')
                    f.write(f'         end if;\n')
                    f.write(f'      end;\n')
                    f.write(f'\n')
                    f.write(f'      raise Program_Error with\n')
                    f.write(f'        "codec registry decode failed for schema {schema_id}";\n')
                f.write(f'   end {func_name};\n')
                f.write(f'\n')

            all_topics = dict(sub_topics)
            all_topics.update(pub_topics)
            for key in all_topics:
                spec = self._topics.spec(key)
                func_name = 'Decode_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                write_payload_decode_function(
                    func_name,
                    spec.ada_payload_type,
                    spec.short_type,
                    spec.is_array,
                    spec.flatbuffers_suffix,
                    spec.short_type)

            if is_provided:
                for svc_name, rpc in all_rpcs:
                    rsp_short = _short_type(rpc.response_type)
                    rsp_fb_suffix = (
                        _flatbuffers_func_suffix_for_stream(rpc.response_type)
                        if rpc.server_streaming else
                        _flatbuffers_func_suffix_for_type(rpc.response_type)
                    )
                    write_payload_decode_function(
                        _rpc_ada_decode_response_name(
                            svc_name, rpc, duplicate_rpc_names),
                        _ada_rsp_type(rpc),
                        rsp_short,
                        rpc.server_streaming,
                        rsp_fb_suffix,
                        rsp_short)

            # Default handler implementations
            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'   --  -- {svc_name} ------------------------------------\n')
                    current_svc = svc_name

                req_t = _ada_req_type(rpc)
                rsp_t = _ada_rsp_type(rpc)
                handler_name = _rpc_ada_handler(
                    svc_name, rpc, duplicate_rpc_names)

                if rpc.server_streaming:
                    f.write(f'   function Default_{handler_name}\n')
                    f.write(f'     (Request : {req_t}) return {rsp_t}\n')
                    f.write(f'   is\n')
                    f.write(f'      pragma Unreferenced (Request);\n')
                    f.write(f'      Empty : {rsp_t} (1 .. 0);\n')
                    f.write(f'   begin\n')
                    f.write(f'      return Empty;\n')
                else:
                    f.write(f'   procedure Default_{handler_name}\n')
                    f.write(f'     (Request  : in  {req_t};\n')
                    f.write(f'      Response : out {rsp_t})\n')
                    f.write(f'   is\n')
                    f.write(f'      pragma Unreferenced (Request);\n')

                    # Generate sensible default return values
                    if rsp_t == 'Identifier':
                        f.write(f'   begin\n')
                        f.write(f'      Response := Null_Unbounded_String;\n')
                    elif rsp_t == 'Ack':
                        f.write(f'   begin\n')
                        # Ack carries more than Success (e.g. an Identifier),
                        # so default any remaining components.
                        f.write(f'      Response := (Success => True, others => <>);\n')
                    else:
                        f.write(f'      Default_Val : {rsp_t};\n')
                        f.write(f'   begin\n')
                        f.write(f'      Response := Default_Val;\n')

                f.write(f'   end Default_{handler_name};\n')
                f.write(f'\n')

            for svc_name, rpc in all_rpcs:
                callback_name = _rpc_ada_callback_name(
                    svc_name, rpc, duplicate_rpc_names)
                f.write(f'   function {callback_name}\n')
                f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
                f.write(f'      Request   : access constant Pcl_Bindings.Pcl_Msg;\n')
                f.write(f'      Response  : access Pcl_Bindings.Pcl_Msg;\n')
                f.write(f'      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;\n')
                f.write(f'      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;\n')
                f.write(f'   pragma Convention (C, {callback_name});\n')
                f.write(f'\n')

            f.write(f'   procedure Register_Services\n')
            f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
            f.write(f'      Handlers  : access constant Service_Handlers := null;\n')
            f.write(f'      Content_Type : String := "application/json";\n')
            f.write(f'      Transport_Config : String := "")\n')
            f.write(f'   is\n')
            f.write(f'      Handler_Ptr : constant System.Address := Handler_Address (Handlers);\n')
            f.write(f'   begin\n')
            for svc_name, rpc in all_rpcs:
                callback_name = _rpc_ada_callback_name(
                    svc_name, rpc, duplicate_rpc_names)
                service_const = _rpc_ada_svc_const(
                    svc_name, rpc, duplicate_rpc_names)
                f.write(f'      declare\n')
                f.write(f'         Service_Name : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'           Interfaces.C.Strings.New_String ({service_const});\n')
                f.write(f'         Type_Name : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'           Interfaces.C.Strings.New_String (Content_Type);\n')
                f.write(f'         Port : Pcl_Bindings.Pcl_Port_Access;\n')
                f.write(f'         pragma Unreferenced (Port);\n')
                f.write(f'      begin\n')
                f.write(f'         Port := Pcl_Bindings.Add_Service\n')
                f.write(f'           (Container    => Container,\n')
                f.write(f'            Service_Name => Service_Name,\n')
                f.write(f'            Type_Name    => Type_Name,\n')
                f.write(f'            Handler      => {callback_name}\'Access,\n')
                f.write(f'            User_Data    => Handler_Ptr);\n')
                f.write(f'         if Port /= null then\n')
                f.write(f'            Apply_Port_Config (Port, Transport_Config);\n')
                f.write(f'         end if;\n')
                f.write(f'         Interfaces.C.Strings.Free (Service_Name);\n')
                f.write(f'         Interfaces.C.Strings.Free (Type_Name);\n')
                f.write(f'      end;\n')
            f.write(f'   end Register_Services;\n')
            f.write(f'\n')

            f.write(f'   procedure Configure_Consumed_Transport\n')
            f.write(f'     (Executor    : Pcl_Bindings.Pcl_Executor_Access;\n')
            f.write(f'      Config_Json : String)\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            for svc_name, rpc in all_rpcs:
                service_const = _rpc_ada_svc_const(
                    svc_name, rpc, duplicate_rpc_names)
                # Streaming rpcs (server_streaming) route under the distinct
                # PCL_ENDPOINT_STREAM_CONSUMED kind, symmetric with
                # PCL_ENDPOINT_STREAM_PROVIDED on the provider side --
                # pcl_executor_invoke_stream() only consults a route
                # installed under this kind, not the unary PCL_ENDPOINT_CONSUMED
                # (see PCL.078 / REQ_PCL_470-472).
                consumed_kind = (
                    'Pcl_Bindings.PCL_ENDPOINT_STREAM_CONSUMED' if rpc.server_streaming
                    else 'Pcl_Bindings.PCL_ENDPOINT_CONSUMED')
                f.write(f'      Apply_Endpoint_Config\n')
                f.write(f'        (Executor, {service_const},\n')
                f.write(f'         {consumed_kind}, Config_Json);\n')
            f.write(f'   end Configure_Consumed_Transport;\n')
            f.write(f'\n')

            f.write(f'   procedure Configure_Publisher_Transport\n')
            f.write(f'     (Executor    : Pcl_Bindings.Pcl_Executor_Access;\n')
            f.write(f'      Config_Json : String)\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            for key in pub_topics:
                const_name = 'Topic_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                f.write(f'      Apply_Endpoint_Config\n')
                f.write(f'        (Executor, {const_name},\n')
                f.write(f'         Pcl_Bindings.PCL_ENDPOINT_PUBLISHER, Config_Json);\n')
            f.write(f'   end Configure_Publisher_Transport;\n')
            f.write(f'\n')

            for svc_name, rpc in all_rpcs:
                callback_name = _rpc_ada_callback_name(
                    svc_name, rpc, duplicate_rpc_names)
                channel_name = _rpc_ada_channel(
                    svc_name, rpc, duplicate_rpc_names)
                f.write(f'   function {callback_name}\n')
                f.write(f'     (Self      : Pcl_Bindings.Pcl_Container_Access;\n')
                f.write(f'      Request   : access constant Pcl_Bindings.Pcl_Msg;\n')
                f.write(f'      Response  : access Pcl_Bindings.Pcl_Msg;\n')
                f.write(f'      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;\n')
                f.write(f'      User_Data : System.Address) return Pcl_Bindings.Pcl_Status\n')
                f.write(f'   is\n')
                f.write(f'      pragma Unreferenced (Self, Ctx);\n')
                f.write(f'      Handlers_Ptr : constant Service_Handlers_Access :=\n')
                f.write(f'        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));\n')
                f.write(f'      Req_Type  : constant String :=\n')
                f.write(f'        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr\n')
                f.write(f'         then "application/json"\n')
                f.write(f'         else Interfaces.C.Strings.Value (Request.Type_Name));\n')
                f.write(f'      Resp_Buf  : System.Address := System.Null_Address;\n')
                f.write(f'      Resp_Size : Natural := 0;\n')
                f.write(f'   begin\n')
                f.write(f'      Dispatch\n')
                f.write(f'        (Handlers      => Handlers_Ptr,\n')
                f.write(f'         Channel       => {channel_name},\n')
                f.write(f'         Request_Buf   => Request.Data,\n')
                f.write(f'         Request_Size  => Natural (Request.Size),\n')
                f.write(f'         Content_Type  => Req_Type,\n')
                f.write(f'         Response_Buf  => Resp_Buf,\n')
                f.write(f'         Response_Size => Resp_Size);\n')
                f.write(f'      Response.Data := Resp_Buf;\n')
                f.write(f'      Response.Size := Interfaces.C.unsigned (Resp_Size);\n')
                f.write(f'      Response.Type_Name :=\n')
                f.write(f'        Interfaces.C.Strings.New_String (Req_Type);\n')
                f.write(f'      return Pcl_Bindings.PCL_OK;\n')
                f.write(f'   exception\n')
                f.write(f'      when others =>\n')
                f.write(f'         Response.Data := System.Null_Address;\n')
                f.write(f'         Response.Size := 0;\n')
                f.write(f'         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;\n')
                f.write(f'         return Pcl_Bindings.PCL_ERR_INVALID;\n')
                f.write(f'   end {callback_name};\n')
                f.write(f'\n')

            # -- PCL binding implementations -----------------------------------
            f.write(f'   --  -- PCL binding implementations -------------------------------\n')
            f.write(f'\n')

            # Subscribe helpers (provided side)
            for key, wire in sub_topics.items():
                ada_name = 'Subscribe_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                const_name = 'Topic_' + '_'.join(
                    w.capitalize() for w in key.split('_'))

                f.write(f'   procedure {ada_name}\n')
                f.write(f'     (Container : Pcl_Bindings.Pcl_Container_Access;\n')
                f.write(f'      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;\n')
                f.write(f'      User_Data : System.Address := System.Null_Address;\n')
                f.write(f'      Content_Type : String := "application/json";\n')
                f.write(f'      Transport_Config : String := "")\n')
                f.write(f'   is\n')
                f.write(f'      Topic  : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'        Interfaces.C.Strings.New_String ({const_name});\n')
                f.write(f'      Type_N : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'        Interfaces.C.Strings.New_String (Content_Type);\n')
                f.write(f'      Port   : Pcl_Bindings.Pcl_Port_Access;\n')
                f.write(f'      pragma Unreferenced (Port);\n')
                f.write(f'   begin\n')
                f.write(f'      Port := Pcl_Bindings.Add_Subscriber\n')
                f.write(f'        (Container => Container,\n')
                f.write(f'         Topic     => Topic,\n')
                f.write(f'         Type_Name => Type_N,\n')
                f.write(f'         Callback  => Callback,\n')
                f.write(f'         User_Data => User_Data);\n')
                f.write(f'      if Port /= null then\n')
                f.write(f'         Apply_Port_Config (Port, Transport_Config);\n')
                f.write(f'      end if;\n')
                f.write(f'      Interfaces.C.Strings.Free (Topic);\n')
                f.write(f'      Interfaces.C.Strings.Free (Type_N);\n')
                f.write(f'   end {ada_name};\n')
                f.write(f'\n')

            # Invoke helpers (provided services) -- typed, serialize internally
            if is_provided:
                for svc in parsed.services:
                    for rpc in _crud_rpcs(svc):
                        invoke_name = _rpc_ada_invoke_name(
                            svc.name, rpc, duplicate_rpc_names)
                        service_const = _rpc_ada_svc_const(
                            svc.name, rpc, duplicate_rpc_names)
                        req_fb_suffix = _flatbuffers_func_suffix_for_type(rpc.request_type)
                        f.write(f'   procedure {invoke_name}\n')
                        f.write(f'     (Executor  : Pcl_Bindings.Pcl_Executor_Access;\n')
                        f.write(f'      Request   : {_ada_req_type(rpc)};\n')
                        f.write(f'      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;\n')
                        f.write(f'      User_Data : System.Address := System.Null_Address;\n')
                        f.write(f'      Content_Type : String := "application/json")\n')
                        f.write(f'   is\n')
                        f.write(f'      use type Pcl_Bindings.Pcl_Status;\n')
                        f.write(f'      function Build_Payload return String is\n')
                        f.write(f'         Registry_Payload : Unbounded_String := Null_Unbounded_String;\n')
                        f.write(f'      begin\n')
                        f.write(f'         Require_Codec (Content_Type);  --  fail closed if no plugin\n')
                        if has_grpc:
                            f.write(f'         if Content_Type = Grpc_Content_Type then\n')
                            f.write(f'            return "";\n')
                            f.write(f'         end if;\n')
                        f.write(f'         if Try_Registry_Encode\n')
                        f.write(f'           (Content_Type, "{_short_type(rpc.request_type)}", Request\'Address,\n')
                        f.write(f'            Registry_Payload)\n')
                        f.write(f'         then\n')
                        f.write(f'            return To_String (Registry_Payload);\n')
                        f.write(f'         end if;\n')
                        f.write(f'         raise Program_Error with\n')
                        f.write(f'           "codec registry encode failed for schema {_short_type(rpc.request_type)}";\n')
                        f.write(f'      end Build_Payload;\n')
                        f.write(f'      Payload : constant String := Build_Payload;\n')
                        f.write(f'      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;\n')
                        f.write(f'      Payload_Bytes : aliased constant String := Payload;\n')
                        f.write(f'      Svc_C  : Interfaces.C.Strings.chars_ptr :=\n')
                        f.write(f'        Interfaces.C.Strings.New_String ({service_const});\n')
                        f.write(f'      Msg    : aliased Pcl_Bindings.Pcl_Msg;\n')
                        f.write(f'      Status : Pcl_Bindings.Pcl_Status;\n')
                        f.write(f'      pragma Unreferenced (Status);\n')
                        f.write(f'   begin\n')
                        if has_grpc:
                            f.write(f'      if Content_Type = Grpc_Content_Type then\n')
                            f.write(f'         if Ada.Strings.Unbounded.Length (Grpc_Channel) = 0 then\n')
                            f.write(f'            raise Program_Error with "gRPC channel not configured";\n')
                            f.write(f'         end if;\n')
                            f.write(f'         declare\n')
                            rsp_schema = _short_type(rpc.response_type)
                            if rpc.server_streaming:
                                elem_type = _proto_type_to_ada(rpc.response_type)
                                f.write(f'            Rsp : constant Grpc_Transport.{_ada_rsp_type(rpc)} :=\n')
                                f.write(f'              Grpc_Transport.{invoke_name}\n')
                                f.write(f'                (To_String (Grpc_Channel), Request);\n')
                                f.write(f'            Acc : Unbounded_String := To_Unbounded_String ("[");\n')
                                f.write(f'         begin\n')
                                f.write(f"            for I in Rsp'Range loop\n")
                                f.write(f"               if I > Rsp'First then\n")
                                f.write(f'                  Append (Acc, ",");\n')
                                f.write(f'               end if;\n')
                                if elem_type == 'Identifier':
                                    f.write(f'               Append (Acc, """" & To_String (Rsp (I)) & """");\n')
                                else:
                                    # Codec-source-free: render each gRPC stream item
                                    # to JSON through the codec registry, not a
                                    # compiled-in To_Json.
                                    f.write(f'               declare\n')
                                    f.write(f'                  Item_Json : Unbounded_String := Null_Unbounded_String;\n')
                                    f.write(f'               begin\n')
                                    f.write(f'                  if not Try_Registry_Encode\n')
                                    f.write(f'                       (Json_Content_Type, "{rsp_schema}", Rsp (I)\'Address, Item_Json)\n')
                                    f.write(f'                  then\n')
                                    f.write(f'                     raise Program_Error with\n')
                                    f.write(f'                       "codec registry encode failed for schema {rsp_schema}";\n')
                                    f.write(f'                  end if;\n')
                                    f.write(f'                  Append (Acc, To_String (Item_Json));\n')
                                    f.write(f'               end;\n')
                                f.write(f'            end loop;\n')
                                f.write(f'            Append (Acc, "]");\n')
                                f.write(f'            Emit_Invoke_Response\n')
                                f.write(f'              (Callback, User_Data, To_String (Acc));\n')
                            elif _ada_rsp_type(rpc) == 'Identifier':
                                f.write(f'            Rsp : constant {_ada_rsp_type(rpc)} :=\n')
                                f.write(f'              Grpc_Transport.{invoke_name}\n')
                                f.write(f'                (To_String (Grpc_Channel), Request);\n')
                                f.write(f'            Response_Payload : constant String :=\n')
                                f.write(f'              """" & To_String (Rsp) & """";\n')
                                f.write(f'         begin\n')
                                f.write(f'            Emit_Invoke_Response\n')
                                f.write(f'              (Callback, User_Data, Response_Payload);\n')
                            else:
                                # Codec-source-free: render the gRPC response to JSON
                                # through the codec registry, not a compiled-in To_Json.
                                f.write(f'            Rsp : constant {_ada_rsp_type(rpc)} :=\n')
                                f.write(f'              Grpc_Transport.{invoke_name}\n')
                                f.write(f'                (To_String (Grpc_Channel), Request);\n')
                                f.write(f'            Rsp_Json : Unbounded_String := Null_Unbounded_String;\n')
                                f.write(f'         begin\n')
                                f.write(f'            if not Try_Registry_Encode\n')
                                f.write(f'                 (Json_Content_Type, "{rsp_schema}", Rsp\'Address, Rsp_Json)\n')
                                f.write(f'            then\n')
                                f.write(f'               raise Program_Error with\n')
                                f.write(f'                 "codec registry encode failed for schema {rsp_schema}";\n')
                                f.write(f'            end if;\n')
                                f.write(f'            Emit_Invoke_Response\n')
                                f.write(f'              (Callback, User_Data, To_String (Rsp_Json));\n')
                            f.write(f'         end;\n')
                            f.write(f'         return;\n')
                            f.write(f'      end if;\n')
                            f.write(f'\n')
                        f.write(f'      if Content_Type = "" or else Content_Type = "application/json" then\n')
                        f.write(f'         Req_C := Interfaces.C.Strings.New_String (Payload);\n')
                        f.write(f"         Msg.Data := To_Address (Req_C);\n")
                        f.write(f'      else\n')
                        f.write(f"         Msg.Data :=\n")
                        f.write(f"           (if Payload_Bytes'Length = 0\n")
                        f.write(f"            then System.Null_Address\n")
                        f.write(f"            else Payload_Bytes (Payload_Bytes'First)'Address);\n")
                        f.write(f'      end if;\n')
                        f.write(f"      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);\n")
                        f.write(f'      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);\n')
                        f.write(f'      Status := Pcl_Bindings.Invoke_Async\n')
                        f.write(f"        (Executor, Svc_C, Msg'Access, Callback, User_Data);\n")
                        f.write(f'      if Req_C /= Interfaces.C.Strings.Null_Ptr then\n')
                        f.write(f'         Interfaces.C.Strings.Free (Req_C);\n')
                        f.write(f'      end if;\n')
                        f.write(f'      Interfaces.C.Strings.Free (Svc_C);\n')
                        f.write(f'      Interfaces.C.Strings.Free (Msg.Type_Name);\n')
                        f.write(f'   end {invoke_name};\n')
                        f.write(f'\n')

            # Publish helpers (consumed topics -- Ada client publishes to server)
            for key, wire in pub_topics.items():
                ada_name = 'Publish_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                const_name = 'Topic_' + '_'.join(
                    w.capitalize() for w in key.split('_'))
                spec = self._topics.spec(key)
                f.write(f'   procedure {ada_name}\n')
                f.write(f'     (Exec    : Pcl_Bindings.Pcl_Executor_Access;\n')
                f.write(f'      Payload : {spec.ada_payload_type};\n')
                f.write(f'      Content_Type : String := "application/json")\n')
                f.write(f'   is\n')
                if spec.is_array:
                    f.write(f'      Wire_Payload : Unbounded_String := Null_Unbounded_String;\n')
                    f.write(f'   begin\n')
                    f.write(f'      Require_Codec (Content_Type);  --  fail closed if no plugin\n')
                    f.write(f'      if not Try_Registry_Encode_{spec.ada_payload_type}\n')
                    f.write(f'        (Content_Type, Payload, Wire_Payload)\n')
                    f.write(f'      then\n')
                    f.write(f'         raise Program_Error with\n')
                    f.write(f'           "codec registry encode failed for schema {spec.short_type}Array";\n')
                    f.write(f'      end if;\n')
                    f.write(f'      {ada_name} (Exec, To_String (Wire_Payload), Content_Type);\n')
                    f.write(f'   end {ada_name};\n')
                    f.write(f'\n')
                else:
                    f.write(f'      function Build_Wire_Payload return String is\n')
                    f.write(f'         Registry_Payload : Unbounded_String := Null_Unbounded_String;\n')
                    f.write(f'      begin\n')
                    f.write(f'         Require_Codec (Content_Type);  --  fail closed if no plugin\n')
                    f.write(f'         if Try_Registry_Encode\n')
                    f.write(f'           (Content_Type, "{spec.short_type}", Payload\'Address,\n')
                    f.write(f'            Registry_Payload)\n')
                    f.write(f'         then\n')
                    f.write(f'            return To_String (Registry_Payload);\n')
                    f.write(f'         end if;\n')
                    f.write(f'         raise Program_Error with\n')
                    f.write(f'           "codec registry encode failed for schema {spec.short_type}";\n')
                    f.write(f'      end Build_Wire_Payload;\n')
                    f.write(f'      Wire_Payload : constant String := Build_Wire_Payload;\n')
                    f.write(f'   begin\n')
                    f.write(f'      {ada_name} (Exec, Wire_Payload, Content_Type);\n')
                    f.write(f'   end {ada_name};\n')
                    f.write(f'\n')

                f.write(f'   procedure {ada_name}\n')
                f.write(f'     (Exec    : Pcl_Bindings.Pcl_Executor_Access;\n')
                f.write(f'      Payload : String;\n')
                f.write(f'      Content_Type : String := "application/json")\n')
                f.write(f'   is\n')
                f.write(f'      use type Pcl_Bindings.Pcl_Status;\n')
                f.write(f'      Payload_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;\n')
                f.write(f'      Payload_Bytes : aliased constant String := Payload;\n')
                f.write(f'      Topic_C   : Interfaces.C.Strings.chars_ptr :=\n')
                f.write(f'        Interfaces.C.Strings.New_String ({const_name});\n')
                f.write(f'      Msg       : aliased Pcl_Bindings.Pcl_Msg;\n')
                f.write(f'      Status    : Pcl_Bindings.Pcl_Status;\n')
                f.write(f'      pragma Unreferenced (Status);\n')
                f.write(f'   begin\n')
                f.write(f'      if Content_Type = "" or else Content_Type = "application/json" then\n')
                f.write(f'         Payload_C := Interfaces.C.Strings.New_String (Payload);\n')
                f.write(f"         Msg.Data := To_Address (Payload_C);\n")
                f.write(f'      else\n')
                f.write(f"         Msg.Data :=\n")
                f.write(f"           (if Payload_Bytes'Length = 0\n")
                f.write(f"            then System.Null_Address\n")
                f.write(f"            else Payload_Bytes (Payload_Bytes'First)'Address);\n")
                f.write(f'      end if;\n')
                f.write(f"      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);\n")
                f.write(f'      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);\n')
                f.write(f"      Status := Pcl_Bindings.Publish (Exec, Topic_C, Msg'Access);\n")
                f.write(f'      if Payload_C /= Interfaces.C.Strings.Null_Ptr then\n')
                f.write(f'         Interfaces.C.Strings.Free (Payload_C);\n')
                f.write(f'      end if;\n')
                f.write(f'      Interfaces.C.Strings.Free (Topic_C);\n')
                f.write(f'      Interfaces.C.Strings.Free (Msg.Type_Name);\n')
                f.write(f'   end {ada_name};\n')
                f.write(f'\n')

            # -- Helper: copy string into heap buffer (caller frees) --------
            f.write(f'   procedure Copy_To_Buf\n')
            f.write(f'     (S    : in  String;\n')
            f.write(f'      Buf  : out System.Address;\n')
            f.write(f'      Size : out Natural)\n')
            f.write(f'   is\n')
            f.write(f'      C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write(f'        Interfaces.C.Strings.New_String (S);\n')
            f.write(f'   begin\n')
            f.write(f'      Buf  := To_Address (C);\n')
            f.write(f"      Size := S'Length;\n")
            f.write(f'   end Copy_To_Buf;\n')
            f.write(f'\n')

            # Dispatch -- deserialise request, call handler, serialise response,
            # allocate response buffer via New_String (caller frees with C free).
            f.write(f'   procedure Dispatch\n')
            f.write(f'     (Handlers      : access constant Service_Handlers := null;\n')
            f.write(f'      Channel       : in  Service_Channel;\n')
            f.write(f'      Request_Buf   : in  System.Address;\n')
            f.write(f'      Request_Size  : in  Natural;\n')
            f.write(f'      Content_Type  : in  String := "application/json";\n')
            f.write(f'      Response_Buf  : out System.Address;\n')
            f.write(f'      Response_Size : out Natural)\n')
            f.write(f'   is\n')
            f.write(f'      Request_Payload : constant String :=\n')
            f.write(f'        Msg_To_String (Request_Buf, Interfaces.C.unsigned (Request_Size));\n')
            f.write(f'   begin\n')
            f.write(f'      Response_Buf  := System.Null_Address;\n')
            f.write(f'      Response_Size := 0;\n')
            f.write(f'      case Channel is\n')

            for svc_name, rpc in all_rpcs:
                req_t = _ada_req_type(rpc)
                rsp_t = _ada_rsp_type(rpc)
                handler_fn = _rpc_ada_handler(
                    svc_name, rpc, duplicate_rpc_names)
                handler_field = _rpc_ada_handler_field(
                    svc_name, rpc, duplicate_rpc_names)
                channel_name = _rpc_ada_channel(
                    svc_name, rpc, duplicate_rpc_names)
                req_fb_suffix = _flatbuffers_func_suffix_for_type(rpc.request_type)
                rsp_fb_suffix = (
                    _flatbuffers_func_suffix_for_stream(rpc.response_type)
                    if rpc.server_streaming else
                    _flatbuffers_func_suffix_for_type(rpc.response_type)
                )
                f.write(f'         when {channel_name} =>\n')
                f.write(f'            declare\n')
                f.write(f'               function Decode_Request return {req_t} is\n')
                f.write(f'                  Result : {req_t};\n')
                f.write(f'               begin\n')
                f.write(f'                  Require_Codec (Content_Type);  --  fail closed if no plugin\n')
                f.write(f'                  if Try_Registry_Decode_Raw\n')
                f.write(f'                    (Content_Type, Request_Buf, Request_Size,\n')
                f.write(f'                     "{_short_type(rpc.request_type)}", Result\'Address)\n')
                f.write(f'                  then\n')
                f.write(f'                     return Result;\n')
                f.write(f'                  end if;\n')
                f.write(f'\n')
                f.write(f'                  raise Program_Error with\n')
                f.write(f'                    "codec registry decode failed for schema {_short_type(rpc.request_type)}";\n')
                f.write(f'               end Decode_Request;\n')
                f.write(f'               Req : constant {req_t} := Decode_Request;\n')

                if rpc.server_streaming:
                    # Streaming: handler is a function returning unconstrained array
                    f.write(f'               Rsp : constant {rsp_t} :=\n')
                    f.write(f'                 (if Handlers /= null and then Handlers.{handler_field} /= null\n')
                    f.write(f'                  then Handlers.{handler_field}.all (Req)\n')
                    f.write(f'                  else Default_{handler_fn} (Req));\n')
                    f.write(f'            begin\n')

                    f.write(f'               declare\n')
                    f.write(f'                  Wire_Response : Unbounded_String := Null_Unbounded_String;\n')
                    f.write(f'               begin\n')
                    f.write(f'                  Require_Codec (Content_Type);  --  fail closed if no plugin\n')
                    f.write(f'                  if not Try_Registry_Encode_{rsp_t}\n')
                    f.write(f'                    (Content_Type, Rsp, Wire_Response)\n')
                    f.write(f'                  then\n')
                    f.write(f'                     raise Program_Error with\n')
                    f.write(f'                       "codec registry encode failed for schema {_short_type(rpc.response_type)}Array";\n')
                    f.write(f'                  end if;\n')
                    f.write(f'                  Copy_To_Buf\n')
                    f.write(f'                    (To_String (Wire_Response),\n')
                    f.write(f'                    Response_Buf, Response_Size);\n')
                    f.write(f'               end;\n')
                else:
                    # Non-streaming: handler is a procedure with out parameter
                    f.write(f'               Rsp : {rsp_t};\n')
                    f.write(f'               Wire_Response : Unbounded_String := Null_Unbounded_String;\n')
                    f.write(f'            begin\n')
                    f.write(f'               if Handlers /= null and then Handlers.{handler_field} /= null then\n')
                    f.write(f'                  Handlers.{handler_field}.all (Req, Rsp);\n')
                    f.write(f'               else\n')
                    f.write(f'                  Default_{handler_fn} (Req, Rsp);\n')
                    f.write(f'               end if;\n')

                    # Serialise response and copy to buffer
                    f.write(f'               Require_Codec (Content_Type);  --  fail closed if no plugin\n')
                    f.write(f'               if not Try_Registry_Encode\n')
                    f.write(f'                 (Content_Type, "{_short_type(rpc.response_type)}", Rsp\'Address,\n')
                    f.write(f'                  Wire_Response)\n')
                    f.write(f'               then\n')
                    f.write(f'                  raise Program_Error with\n')
                    f.write(f'                    "codec registry encode failed for schema {_short_type(rpc.response_type)}";\n')
                    f.write(f'               end if;\n')
                    f.write(f'               Copy_To_Buf\n')
                    f.write(f'                 (To_String (Wire_Response),\n')
                    f.write(f'                  Response_Buf, Response_Size);\n')

                f.write(f'            end;\n')

            f.write(f'      end case;\n')
            f.write(f'   end Dispatch;\n')
            f.write(f'\n')
            if pf is not None:
                self._write_interaction_facade_body(f, pf, parsed)
            f.write(f'end {pkg_name};\n')


