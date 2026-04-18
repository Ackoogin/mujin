--  Auto-generated service FlatBuffers codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Autonomy.Types_Codec;
with Pyramid.Data_Model.Common.Types_Codec;

package body Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec is
   use type Interfaces.C.size_t;
   use type Interfaces.C.Strings.chars_ptr;
   use type System.Address;

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

   function Copy_From_Buffer
     (Data : System.Address; Size : Interfaces.C.size_t) return String
   is
      type Char_Array is array (1 .. Natural (Size)) of Character;
      pragma Pack (Char_Array);
   begin
      if Data = System.Null_Address or else Size = 0 then
         return "";
      end if;

      declare
         Chars : Char_Array;
         for Chars'Address use Data;
         pragma Import (Ada, Chars);
      begin
         return String (Chars);
      end;
   end Copy_From_Buffer;

   procedure Free_Buffer (Data : System.Address)
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_free_buffer";

   function Imported_To_Binary_Fact_Update
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_FactUpdate_to_flatbuffer_json";

   function Imported_From_Binary_Fact_Update
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_FactUpdate_from_flatbuffer_json";

   function Imported_To_Binary_State_Update
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_StateUpdate_to_flatbuffer_json";

   function Imported_From_Binary_State_Update
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_StateUpdate_from_flatbuffer_json";

   function Imported_To_Binary_Mission_Intent
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_MissionIntent_to_flatbuffer_json";

   function Imported_From_Binary_Mission_Intent
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_MissionIntent_from_flatbuffer_json";

   function Imported_To_Binary_Agent_State
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_AgentState_to_flatbuffer_json";

   function Imported_From_Binary_Agent_State
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_AgentState_from_flatbuffer_json";

   function Imported_To_Binary_Policy_Envelope
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PolicyEnvelope_to_flatbuffer_json";

   function Imported_From_Binary_Policy_Envelope
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PolicyEnvelope_from_flatbuffer_json";

   function Imported_To_Binary_Session
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Session_to_flatbuffer_json";

   function Imported_From_Binary_Session
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Session_from_flatbuffer_json";

   function Imported_To_Binary_Capabilities
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Capabilities_to_flatbuffer_json";

   function Imported_From_Binary_Capabilities
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Capabilities_from_flatbuffer_json";

   function Imported_To_Binary_String_Key_Value
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_StringKeyValue_to_flatbuffer_json";

   function Imported_From_Binary_String_Key_Value
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_StringKeyValue_from_flatbuffer_json";

   function Imported_To_Binary_Command
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Command_to_flatbuffer_json";

   function Imported_From_Binary_Command
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Command_from_flatbuffer_json";

   function Imported_To_Binary_Goal_Dispatch
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_GoalDispatch_to_flatbuffer_json";

   function Imported_From_Binary_Goal_Dispatch
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_GoalDispatch_from_flatbuffer_json";

   function Imported_To_Binary_Decision_Record
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_DecisionRecord_to_flatbuffer_json";

   function Imported_From_Binary_Decision_Record
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_DecisionRecord_from_flatbuffer_json";

   function Imported_To_Binary_Command_Result
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_CommandResult_to_flatbuffer_json";

   function Imported_From_Binary_Command_Result
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_CommandResult_from_flatbuffer_json";

   function Imported_To_Binary_Dispatch_Result
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_DispatchResult_to_flatbuffer_json";

   function Imported_From_Binary_Dispatch_Result
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_DispatchResult_from_flatbuffer_json";

   function Imported_To_Binary_Session_Snapshot
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionSnapshot_to_flatbuffer_json";

   function Imported_From_Binary_Session_Snapshot
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionSnapshot_from_flatbuffer_json";

   function Imported_To_Binary_Session_Step_Request
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionStepRequest_to_flatbuffer_json";

   function Imported_From_Binary_Session_Step_Request
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionStepRequest_from_flatbuffer_json";

   function Imported_To_Binary_Session_Stop_Request
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionStopRequest_to_flatbuffer_json";

   function Imported_From_Binary_Session_Stop_Request
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionStopRequest_from_flatbuffer_json";

   function Imported_To_Binary_Ack
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Ack_to_flatbuffer_json";

   function Imported_From_Binary_Ack
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Ack_from_flatbuffer_json";

   function Imported_To_Binary_Query
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Query_to_flatbuffer_json";

   function Imported_From_Binary_Query
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Query_from_flatbuffer_json";

   function Imported_To_Binary_Identifier
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Identifier_to_flatbuffer_json";

   function Imported_From_Binary_Identifier
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Identifier_from_flatbuffer_json";

   function Imported_To_Binary_Capabilities_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_CapabilitiesArray_to_flatbuffer_json";

   function Imported_From_Binary_Capabilities_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_CapabilitiesArray_from_flatbuffer_json";

   function Imported_To_Binary_Session_Snapshot_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionSnapshotArray_to_flatbuffer_json";

   function Imported_From_Binary_Session_Snapshot_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_SessionSnapshotArray_from_flatbuffer_json";

   function Imported_To_Binary_Command_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_CommandArray_to_flatbuffer_json";

   function Imported_From_Binary_Command_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_CommandArray_from_flatbuffer_json";

   function Imported_To_Binary_Goal_Dispatch_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_GoalDispatchArray_to_flatbuffer_json";

   function Imported_From_Binary_Goal_Dispatch_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_GoalDispatchArray_from_flatbuffer_json";

   function Imported_To_Binary_Decision_Record_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_DecisionRecordArray_to_flatbuffer_json";

   function Imported_From_Binary_Decision_Record_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_DecisionRecordArray_from_flatbuffer_json";

   function To_Binary_Fact_Update (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Fact_Update (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Fact_Update";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Fact_Update;

   function From_Binary_Fact_Update (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Fact_Update
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Fact_Update";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Fact_Update;

   function To_Binary_State_Update (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_State_Update (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for State_Update";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_State_Update;

   function From_Binary_State_Update (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_State_Update
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for State_Update";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_State_Update;

   function To_Binary_Mission_Intent (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Mission_Intent (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Mission_Intent";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Mission_Intent;

   function From_Binary_Mission_Intent (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Mission_Intent
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Mission_Intent";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Mission_Intent;

   function To_Binary_Agent_State (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Agent_State (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Agent_State";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Agent_State;

   function From_Binary_Agent_State (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Agent_State
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Agent_State";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Agent_State;

   function To_Binary_Policy_Envelope (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Policy_Envelope (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Policy_Envelope";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Policy_Envelope;

   function From_Binary_Policy_Envelope (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Policy_Envelope
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Policy_Envelope";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Policy_Envelope;

   function To_Binary_Session (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Session (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Session";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Session;

   function From_Binary_Session (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Session
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Session";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Session;

   function To_Binary_Capabilities (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Capabilities (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Capabilities";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Capabilities;

   function From_Binary_Capabilities (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Capabilities
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Capabilities";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Capabilities;

   function To_Binary_String_Key_Value (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_String_Key_Value (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for String_Key_Value";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_String_Key_Value;

   function From_Binary_String_Key_Value (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_String_Key_Value
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for String_Key_Value";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_String_Key_Value;

   function To_Binary_Command (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Command (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Command";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Command;

   function From_Binary_Command (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Command
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Command";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Command;

   function To_Binary_Goal_Dispatch (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Goal_Dispatch (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Goal_Dispatch";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Goal_Dispatch;

   function From_Binary_Goal_Dispatch (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Goal_Dispatch
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Goal_Dispatch";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Goal_Dispatch;

   function To_Binary_Decision_Record (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Decision_Record (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Decision_Record";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Decision_Record;

   function From_Binary_Decision_Record (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Decision_Record
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Decision_Record";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Decision_Record;

   function To_Binary_Command_Result (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Command_Result (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Command_Result";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Command_Result;

   function From_Binary_Command_Result (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Command_Result
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Command_Result";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Command_Result;

   function To_Binary_Dispatch_Result (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Dispatch_Result (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Dispatch_Result";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Dispatch_Result;

   function From_Binary_Dispatch_Result (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Dispatch_Result
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Dispatch_Result";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Dispatch_Result;

   function To_Binary_Session_Snapshot (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Session_Snapshot (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Session_Snapshot";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Session_Snapshot;

   function From_Binary_Session_Snapshot (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Session_Snapshot
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Session_Snapshot";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Session_Snapshot;

   function To_Binary_Session_Step_Request (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Session_Step_Request (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Session_Step_Request";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Session_Step_Request;

   function From_Binary_Session_Step_Request (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Session_Step_Request
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Session_Step_Request";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Session_Step_Request;

   function To_Binary_Session_Stop_Request (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Session_Stop_Request (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Session_Stop_Request";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Session_Stop_Request;

   function From_Binary_Session_Stop_Request (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Session_Stop_Request
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Session_Stop_Request";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Session_Stop_Request;

   function To_Binary_Ack (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Ack (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Ack";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Ack;

   function From_Binary_Ack (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Ack
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Ack";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Ack;

   function To_Binary_Query (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Query (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Query";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Query;

   function From_Binary_Query (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Query
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Query";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Query;

   function To_Binary_Identifier (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Identifier (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Identifier";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Identifier;

   function From_Binary_Identifier (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Identifier
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Identifier";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Identifier;

   function To_Binary_Capabilities_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Capabilities_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Capabilities_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Capabilities_Array;

   function From_Binary_Capabilities_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Capabilities_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Capabilities_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Capabilities_Array;

   function To_Binary_Session_Snapshot_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Session_Snapshot_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Session_Snapshot_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Session_Snapshot_Array;

   function From_Binary_Session_Snapshot_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Session_Snapshot_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Session_Snapshot_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Session_Snapshot_Array;

   function To_Binary_Command_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Command_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Command_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Command_Array;

   function From_Binary_Command_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Command_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Command_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Command_Array;

   function To_Binary_Goal_Dispatch_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Goal_Dispatch_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Goal_Dispatch_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Goal_Dispatch_Array;

   function From_Binary_Goal_Dispatch_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Goal_Dispatch_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Goal_Dispatch_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Goal_Dispatch_Array;

   function To_Binary_Decision_Record_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Decision_Record_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Decision_Record_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Decision_Record_Array;

   function From_Binary_Decision_Record_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Decision_Record_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Decision_Record_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Decision_Record_Array;

   function To_Binary_Fact_Update (Msg : Pyramid.Data_Model.Autonomy.Types.Fact_Update) return String is
   begin
      return To_Binary_Fact_Update (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Fact_Update;

   function From_Binary_Fact_Update
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Fact_Update) return Pyramid.Data_Model.Autonomy.Types.Fact_Update
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Fact_Update (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Fact_Update;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Fact_Update;

   function To_Binary_State_Update (Msg : Pyramid.Data_Model.Autonomy.Types.State_Update) return String is
   begin
      return To_Binary_State_Update (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_State_Update;

   function From_Binary_State_Update
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.State_Update) return Pyramid.Data_Model.Autonomy.Types.State_Update
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_State_Update (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.State_Update;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_State_Update;

   function To_Binary_Mission_Intent (Msg : Pyramid.Data_Model.Autonomy.Types.Mission_Intent) return String is
   begin
      return To_Binary_Mission_Intent (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Mission_Intent;

   function From_Binary_Mission_Intent
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Mission_Intent) return Pyramid.Data_Model.Autonomy.Types.Mission_Intent
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Mission_Intent (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Mission_Intent;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Mission_Intent;

   function To_Binary_Agent_State (Msg : Pyramid.Data_Model.Autonomy.Types.Agent_State) return String is
   begin
      return To_Binary_Agent_State (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Agent_State;

   function From_Binary_Agent_State
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Agent_State) return Pyramid.Data_Model.Autonomy.Types.Agent_State
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Agent_State (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Agent_State;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Agent_State;

   function To_Binary_Policy_Envelope (Msg : Pyramid.Data_Model.Autonomy.Types.Policy_Envelope) return String is
   begin
      return To_Binary_Policy_Envelope (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Policy_Envelope;

   function From_Binary_Policy_Envelope
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Policy_Envelope) return Pyramid.Data_Model.Autonomy.Types.Policy_Envelope
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Policy_Envelope (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Policy_Envelope;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Policy_Envelope;

   function To_Binary_Session (Msg : Pyramid.Data_Model.Autonomy.Types.Session) return String is
   begin
      return To_Binary_Session (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Session;

   function From_Binary_Session
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session) return Pyramid.Data_Model.Autonomy.Types.Session
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Session (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Session;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Session;

   function To_Binary_Capabilities (Msg : Pyramid.Data_Model.Autonomy.Types.Capabilities) return String is
   begin
      return To_Binary_Capabilities (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Capabilities;

   function From_Binary_Capabilities
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Capabilities) return Pyramid.Data_Model.Autonomy.Types.Capabilities
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Capabilities (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Capabilities;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Capabilities;

   function To_Binary_String_Key_Value (Msg : Pyramid.Data_Model.Autonomy.Types.String_Key_Value) return String is
   begin
      return To_Binary_String_Key_Value (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_String_Key_Value;

   function From_Binary_String_Key_Value
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.String_Key_Value) return Pyramid.Data_Model.Autonomy.Types.String_Key_Value
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_String_Key_Value (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.String_Key_Value;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_String_Key_Value;

   function To_Binary_Command (Msg : Pyramid.Data_Model.Autonomy.Types.Command) return String is
   begin
      return To_Binary_Command (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Command;

   function From_Binary_Command
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Command) return Pyramid.Data_Model.Autonomy.Types.Command
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Command (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Command;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Command;

   function To_Binary_Goal_Dispatch (Msg : Pyramid.Data_Model.Autonomy.Types.Goal_Dispatch) return String is
   begin
      return To_Binary_Goal_Dispatch (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Goal_Dispatch;

   function From_Binary_Goal_Dispatch
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Goal_Dispatch) return Pyramid.Data_Model.Autonomy.Types.Goal_Dispatch
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Goal_Dispatch (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Goal_Dispatch;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Goal_Dispatch;

   function To_Binary_Decision_Record (Msg : Pyramid.Data_Model.Autonomy.Types.Decision_Record) return String is
   begin
      return To_Binary_Decision_Record (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Decision_Record;

   function From_Binary_Decision_Record
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Decision_Record) return Pyramid.Data_Model.Autonomy.Types.Decision_Record
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Decision_Record (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Decision_Record;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Decision_Record;

   function To_Binary_Command_Result (Msg : Pyramid.Data_Model.Autonomy.Types.Command_Result) return String is
   begin
      return To_Binary_Command_Result (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Command_Result;

   function From_Binary_Command_Result
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Command_Result) return Pyramid.Data_Model.Autonomy.Types.Command_Result
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Command_Result (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Command_Result;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Command_Result;

   function To_Binary_Dispatch_Result (Msg : Pyramid.Data_Model.Autonomy.Types.Dispatch_Result) return String is
   begin
      return To_Binary_Dispatch_Result (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Dispatch_Result;

   function From_Binary_Dispatch_Result
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Dispatch_Result) return Pyramid.Data_Model.Autonomy.Types.Dispatch_Result
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Dispatch_Result (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Dispatch_Result;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Dispatch_Result;

   function To_Binary_Session_Snapshot (Msg : Pyramid.Data_Model.Autonomy.Types.Session_Snapshot) return String is
   begin
      return To_Binary_Session_Snapshot (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Session_Snapshot;

   function From_Binary_Session_Snapshot
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session_Snapshot) return Pyramid.Data_Model.Autonomy.Types.Session_Snapshot
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Session_Snapshot (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Session_Snapshot;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Session_Snapshot;

   function To_Binary_Session_Step_Request (Msg : Pyramid.Data_Model.Autonomy.Types.Session_Step_Request) return String is
   begin
      return To_Binary_Session_Step_Request (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Session_Step_Request;

   function From_Binary_Session_Step_Request
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session_Step_Request) return Pyramid.Data_Model.Autonomy.Types.Session_Step_Request
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Session_Step_Request (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Session_Step_Request;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Session_Step_Request;

   function To_Binary_Session_Stop_Request (Msg : Pyramid.Data_Model.Autonomy.Types.Session_Stop_Request) return String is
   begin
      return To_Binary_Session_Stop_Request (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Session_Stop_Request;

   function From_Binary_Session_Stop_Request
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session_Stop_Request) return Pyramid.Data_Model.Autonomy.Types.Session_Stop_Request
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Session_Stop_Request (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Session_Stop_Request;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Session_Stop_Request;

   function To_Binary_Ack (Msg : Pyramid.Data_Model.Common.Types.Ack) return String is
   begin
      return To_Binary_Ack (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Ack;

   function From_Binary_Ack
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Ack) return Pyramid.Data_Model.Common.Types.Ack
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Ack (Payload);
      Result : Pyramid.Data_Model.Common.Types.Ack;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Ack;

   function To_Binary_Query (Msg : Pyramid.Data_Model.Common.Types.Query) return String is
   begin
      return To_Binary_Query (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Query;

   function From_Binary_Query
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Query) return Pyramid.Data_Model.Common.Types.Query
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Query (Payload);
      Result : Pyramid.Data_Model.Common.Types.Query;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Query;

   function To_Binary_Identifier (Msg : Pyramid.Data_Model.Base.Types.Identifier) return String is
   begin
      return To_Binary_Identifier (String'(Write (Create (UTF8_String'(Ada.Strings.Unbounded.To_String (Msg))))));
   end To_Binary_Identifier;

   function From_Binary_Identifier
     (Payload : String; Tag : access Pyramid.Data_Model.Base.Types.Identifier) return Pyramid.Data_Model.Base.Types.Identifier
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Identifier (Payload);
      Result : Pyramid.Data_Model.Base.Types.Identifier;
   begin
      declare
         Value : constant JSON_Value := Read (Json);
         Text  : constant String := String'(UTF8_String'(Get (Value)));
      begin
         Result := To_Unbounded_String (Text);
      end;
      return Result;
   end From_Binary_Identifier;

end Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec;
