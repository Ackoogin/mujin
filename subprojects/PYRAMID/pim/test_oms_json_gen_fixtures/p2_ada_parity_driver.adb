--  Cross-language parity driver for the generated P2 (A-GRA 5.0a) Ada
--  OMS-JSON encoder (pim/test_oms_json_gen.py, AdaCompileParityTest).
--  Builds the same MA_MissionPlanCommandStatusMT value as the C++ smoke
--  driver's self-test and prints To_Oms_Json; the Python side compares both
--  against p2_parity_golden.json.  The UUIDs are in the A-GRA hexBinary wire
--  form (32 hex digits, no hyphens), which is the drop difference this
--  profile's parity golden exists to pin.
with Ada.Text_IO;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Agra.Types;  use Pyramid.Data_Model.Agra.Types;
with Pyramid.Data_Model.Agra.Oms_Json_Codec;

procedure P2_Ada_Parity_Driver is
   M  : Ma_Mission_Plan_Command_Status_Mt;
   Op : Owner_Producer_Choice_Type;
begin
   M.Security_Information.Classification := Enum_U;
   Op.Has_Government_Identifier := True;
   Op.Government_Identifier := Enum_Usa;
   M.Security_Information.Owner_Producer :=
     new Owner_Producer_Choice_Type_Array'(1 => Op);
   M.Message_Header.System_Id.Uuid :=
     To_Unbounded_String ("123E4567E89B42D3A456426614174000");
   M.Message_Header.Timestamp :=
     To_Unbounded_String ("2026-01-01T00:00:00Z");
   M.Message_Header.Schema_Version := To_Unbounded_String ("005.0a.ASK");
   M.Message_Header.Mode := Enum_Simulation;
   M.Message_Data.Command_Id.Uuid :=
     To_Unbounded_String ("123E4567E89B42D3A456426614174001");
   M.Message_Data.Planning_Status.Command_Processing_State := Enum_Received;
   M.Message_Data.Planning_Status.Command_Status := Enum_Queued;
   Ada.Text_IO.Put_Line
     (Pyramid.Data_Model.Agra.Oms_Json_Codec.To_Oms_Json (M));
end P2_Ada_Parity_Driver;
