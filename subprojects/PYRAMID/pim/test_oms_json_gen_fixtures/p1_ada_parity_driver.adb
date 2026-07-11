--  Cross-language parity driver for the generated P1 Ada OMS-JSON encoder
--  (pim/test_oms_json_gen.py, AdaCompileParityTest).  Builds the same
--  ActionCommandStatusMT value as the C++ smoke driver's self-test and
--  prints To_Oms_Json; the Python side compares the two wire documents.
with Ada.Text_IO;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Uci.Types;  use Pyramid.Data_Model.Uci.Types;
with Pyramid.Data_Model.Uci.Oms_Json_Codec;

procedure P1_Ada_Parity_Driver is
   M  : Action_Command_Status_Mt;
   Op : Owner_Producer_Choice_Type;
begin
   M.Security_Information.Classification := Enum_U;
   Op.Has_Government_Identifier := True;
   Op.Government_Identifier := Enum_Usa;
   M.Security_Information.Owner_Producer :=
     new Owner_Producer_Choice_Type_Array'(1 => Op);
   M.Message_Header.System_Id.Uuid :=
     To_Unbounded_String ("123e4567-e89b-42d3-a456-426614174000");
   M.Message_Header.Timestamp :=
     To_Unbounded_String ("2026-01-01T00:00:00Z");
   M.Message_Header.Schema_Version := To_Unbounded_String ("002.5.0");
   M.Message_Header.Mode := Enum_Simulation;
   M.Message_Data.Base.Command_Id.Uuid :=
     To_Unbounded_String ("123e4567-e89b-42d3-a456-426614174001");
   M.Message_Data.Base.Command_Processing_State := Enum_Received;
   Ada.Text_IO.Put_Line
     (Pyramid.Data_Model.Uci.Oms_Json_Codec.To_Oms_Json (M));
end P1_Ada_Parity_Driver;
