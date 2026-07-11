#!/usr/bin/env python3
"""Minimal Ada OMS-JSON package emitter for the UCI seam contract.

The C++ plugin is the PCL wire implementation.  This package exposes the
same content type and root wrappers to Ada clients without introducing a
GNATCOLL runtime dependency into generated bindings.
"""
from pathlib import Path
from typing import List
from proto_parser import ProtoTypeIndex


class AdaOmsJsonCodecGenerator:
    """Seam-contract-only template emitter.

    Unlike the C++ generator (generalized in UCI MMS plan Phase 2), this
    package body is a hand-shaped template for the exact
    ``pim/uci_seam_example`` contract: it references the seam's message
    shapes and its two components packages by name.  It therefore guards on
    that shape and SKIPs (with a printed note) for any other UCI package --
    emitting it against an xsd2proto-generated tree would produce
    non-compiling Ada referencing types that do not exist.  True Ada
    generalization is tracked as remaining Phase-2 work in
    doc/plans/PYRAMID/uci_mms_conversion_plan.md."""

    _SEAM_COMPONENT_PACKAGES = (
        'pyramid.components.uci.mission_autonomy.services.provided',
        'pyramid.components.uci.c2_station.services.consumed',
    )

    def __init__(self, index: ProtoTypeIndex):
        self.pf = next((p for p in index.files if p.package == 'pyramid.data_model.uci'), None)
        if self.pf is not None:
            packages = {p.package for p in index.files}
            is_seam = (
                self.pf.find_message('ActionCommand') is not None
                and self.pf.find_message('ActionCommandStatus') is not None
                and all(pkg in packages
                        for pkg in self._SEAM_COMPONENT_PACKAGES))
            if not is_seam:
                print('  oms_json (Ada): package pyramid.data_model.uci is '
                      'not the seam contract shape -- skipping (Ada '
                      'generalization is pending Phase-2 work)')
                self.pf = None

    def generate(self, out: Path) -> List[Path]:
        out.mkdir(parents=True, exist_ok=True)
        if self.pf is None:
            return []
        spec = out / 'pyramid-data_model-uci-oms_json_codec.ads'
        body = out / 'pyramid-data_model-uci-oms_json_codec.adb'
        spec.write_text('''with Pyramid.Data_Model.Uci.Types;
with Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types;
with Pyramid.Components.Uci.C2_Station.Services.Consumed.Types;
package Pyramid.Data_Model.Uci.Oms_Json_Codec is
   Content_Type : constant String := "application/oms-json";
   function To_Oms_Json (Msg : Pyramid.Data_Model.Uci.Types.Action_Command)
      return String;
   function To_Oms_Json (Msg : Pyramid.Data_Model.Uci.Types.Action_Command_Status)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Request)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Requirement)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Request)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Requirement)
      return String;
end Pyramid.Data_Model.Uci.Oms_Json_Codec;
'''.replace('\\"', '""'), encoding='utf-8')
        # Keep this emitter dependency-free: generated Ada bindings do not
        # require GNATCOLL.JSON merely to use the same OMS wire shape.
        body.write_text('''with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types;
with Pyramid.Components.Uci.C2_Station.Services.Consumed.Types;
package body Pyramid.Data_Model.Uci.Oms_Json_Codec is
   package U renames Pyramid.Data_Model.Uci.Types;
   use type U.Government_Identifier_Array_Acc;
   use type U.Command_Array_Acc;
   function Q (S : Unbounded_String) return String is
   begin
      return "\\\"" & To_String (S) & "\\\"";
   end Q;
   function Security (S : U.Security_Information) return String is
   begin
      if S.Owner_Producer = null then
         raise Constraint_Error with "OwnerProducer is required";
      end if;
      return "{\\\"Classification\\\":" & Q (S.Classification) &
        ",\\\"OwnerProducer\\\":[{\\\"GovernmentIdentifier\\\":" &
        Q (S.Owner_Producer (S.Owner_Producer'First).Government_Identifier) & "}]}";
   end Security;
   function Header (H : U.Message_Header) return String is
      Prefix : Unbounded_String := Null_Unbounded_String;
      Service : Unbounded_String := Null_Unbounded_String;
   begin
      if Length (H.Val_Mission_Id.Val_Uuid.Uuid) /= 0 then
         Prefix := To_Unbounded_String ("\\\"MissionID\\\":{\\\"UUID\\\":" &
           Q (H.Val_Mission_Id.Val_Uuid.Uuid) & "},");
      end if;
      if Length (H.Val_Service_Id.Val_Uuid.Uuid) /= 0 then
         Service := To_Unbounded_String (",\\\"ServiceID\\\":{\\\"UUID\\\":" &
           Q (H.Val_Service_Id.Val_Uuid.Uuid) & "}");
      end if;
      return "{" & To_String (Prefix) & "\\\"SystemID\\\":{\\\"UUID\\\":" &
        Q (H.Val_System_Id.Val_Uuid.Uuid) & "}" & To_String (Service) & ",\\\"Timestamp\\\":" &
        Q (H.Timestamp) & ",\\\"SchemaVersion\\\":" &
        Q (H.Schema_Version) & ",\\\"Mode\\\":" & Q (H.Mode) & "}";
   end Header;
   function To_Oms_Json (Msg : U.Action_Command) return String is
      Result : Unbounded_String := To_Unbounded_String ("{\\\"ActionCommand\\\":{\\\"SecurityInformation\\\":" &
        Security (Msg.Val_Security_Information) & ",\\\"MessageHeader\\\":" & Header (Msg.Val_Message_Header) & ",\\\"MessageData\\\":{\\\"Command\\\":[");
   begin
      if Msg.Message_Data.Command = null then raise Constraint_Error with "Command is required"; end if;
      for I in Msg.Message_Data.Command'Range loop
         declare C : constant U.Capability := Msg.Message_Data.Command (I).Val_Capability; begin
            if I /= Msg.Message_Data.Command'First then Append (Result, ","); end if;
            Append (Result, "{\\\"Capability\\\":{\\\"CommandID\\\":{\\\"UUID\\\":" & Q (C.Val_Command_Id.Val_Uuid.Uuid) & "},\\\"CommandState\\\":" & Q (C.Command_State) & ",\\\"CapabilityID\\\":{\\\"UUID\\\":" & Q (C.Val_Capability_Id.Val_Uuid.Uuid) & "},\\\"Ranking\\\":{\\\"Rank\\\":{\\\"Priority\\\":" & Integer'Image (C.Val_Ranking.Val_Rank.Priority) & "}},\\\"ActionID\\\":{\\\"UUID\\\":" & Q (C.Val_Action_Id.Val_Uuid.Uuid) & "}}}");
         end;
      end loop;
      return To_String (Result) & "]}}}";
   end To_Oms_Json;
   function To_Oms_Json (Msg : U.Action_Command_Status) return String is
   begin
      return "{\\\"ActionCommandStatus\\\":{\\\"SecurityInformation\\\":" & Security (Msg.Val_Security_Information) &
        ",\\\"MessageHeader\\\":" & Header (Msg.Val_Message_Header) & ",\\\"MessageData\\\":{\\\"CommandID\\\":{\\\"UUID\\\":" &
        Q (Msg.Message_Data.Val_Command_Id.Val_Uuid.Uuid) & "},\\\"CommandProcessingState\\\":" &
        Q (Msg.Message_Data.Command_Processing_State) & "}}}";
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Request)
      return String is
   begin
      if Msg.Has_Action_Command = Msg.Has_Update then
         raise Constraint_Error with "exactly one UCI request variant is required";
      elsif Msg.Has_Action_Command then
         return To_Oms_Json (Msg.Action_Command);
      end if;
      return To_Oms_Json (Msg.Update);
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Requirement)
      return String is
   begin
      if not Msg.Has_Action_Command_Status then
         raise Constraint_Error with "ActionCommandStatus is required";
      end if;
      return To_Oms_Json (Msg.Action_Command_Status);
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Request)
      return String is
   begin
      if Msg.Has_Action_Command = Msg.Has_Update then
         raise Constraint_Error with "exactly one UCI request variant is required";
      elsif Msg.Has_Action_Command then
         return To_Oms_Json (Msg.Action_Command);
      end if;
      return To_Oms_Json (Msg.Update);
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Requirement)
      return String is
   begin
      if not Msg.Has_Action_Command_Status then
         raise Constraint_Error with "ActionCommandStatus is required";
      end if;
      return To_Oms_Json (Msg.Action_Command_Status);
   end To_Oms_Json;
end Pyramid.Data_Model.Uci.Oms_Json_Codec;
'''.replace(chr(92) + '"', '""'), encoding='utf-8')
        return [spec, body]
