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

   function Imported_To_Binary_Requirement_Reference
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_RequirementReference_to_flatbuffer_json";

   function Imported_From_Binary_Requirement_Reference
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_RequirementReference_from_flatbuffer_json";

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

   function Imported_To_Binary_Planning_Policy
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningPolicy_to_flatbuffer_json";

   function Imported_From_Binary_Planning_Policy
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningPolicy_from_flatbuffer_json";

   function Imported_To_Binary_Planning_Goal
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningGoal_to_flatbuffer_json";

   function Imported_From_Binary_Planning_Goal
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningGoal_from_flatbuffer_json";

   function Imported_To_Binary_World_Fact_Update
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_WorldFactUpdate_to_flatbuffer_json";

   function Imported_From_Binary_World_Fact_Update
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_WorldFactUpdate_from_flatbuffer_json";

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

   function Imported_To_Binary_Planned_Component_Interaction
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlannedComponentInteraction_to_flatbuffer_json";

   function Imported_From_Binary_Planned_Component_Interaction
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlannedComponentInteraction_from_flatbuffer_json";

   function Imported_To_Binary_Plan_Step
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanStep_to_flatbuffer_json";

   function Imported_From_Binary_Plan_Step
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanStep_from_flatbuffer_json";

   function Imported_To_Binary_Plan
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Plan_to_flatbuffer_json";

   function Imported_From_Binary_Plan
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Plan_from_flatbuffer_json";

   function Imported_To_Binary_Requirement_Placement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_RequirementPlacement_to_flatbuffer_json";

   function Imported_From_Binary_Requirement_Placement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_RequirementPlacement_from_flatbuffer_json";

   function Imported_To_Binary_Achievement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Achievement_to_flatbuffer_json";

   function Imported_From_Binary_Achievement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Achievement_from_flatbuffer_json";

   function Imported_To_Binary_Entity
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Entity_to_flatbuffer_json";

   function Imported_From_Binary_Entity
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_Entity_from_flatbuffer_json";

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

   function Imported_To_Binary_Planning_Execution_Requirement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningExecutionRequirement_to_flatbuffer_json";

   function Imported_From_Binary_Planning_Execution_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningExecutionRequirement_from_flatbuffer_json";

   function Imported_To_Binary_Execution_Run
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_ExecutionRun_to_flatbuffer_json";

   function Imported_From_Binary_Execution_Run
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_ExecutionRun_from_flatbuffer_json";

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

   function Imported_To_Binary_Planning_Execution_Requirement_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningExecutionRequirementArray_to_flatbuffer_json";

   function Imported_From_Binary_Planning_Execution_Requirement_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanningExecutionRequirementArray_from_flatbuffer_json";

   function Imported_To_Binary_Plan_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanArray_to_flatbuffer_json";

   function Imported_From_Binary_Plan_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_PlanArray_from_flatbuffer_json";

   function Imported_To_Binary_Execution_Run_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_ExecutionRunArray_to_flatbuffer_json";

   function Imported_From_Binary_Execution_Run_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_ExecutionRunArray_from_flatbuffer_json";

   function Imported_To_Binary_Requirement_Placement_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_RequirementPlacementArray_to_flatbuffer_json";

   function Imported_From_Binary_Requirement_Placement_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_autonomy_backend_RequirementPlacementArray_from_flatbuffer_json";

   function To_Binary_Requirement_Reference (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Requirement_Reference (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Requirement_Reference";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Requirement_Reference;

   function From_Binary_Requirement_Reference (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Requirement_Reference
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Requirement_Reference";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Requirement_Reference;

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

   function To_Binary_Planning_Policy (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Planning_Policy (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Planning_Policy";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Planning_Policy;

   function From_Binary_Planning_Policy (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Planning_Policy
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Planning_Policy";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Planning_Policy;

   function To_Binary_Planning_Goal (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Planning_Goal (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Planning_Goal";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Planning_Goal;

   function From_Binary_Planning_Goal (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Planning_Goal
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Planning_Goal";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Planning_Goal;

   function To_Binary_World_Fact_Update (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_World_Fact_Update (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for World_Fact_Update";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_World_Fact_Update;

   function From_Binary_World_Fact_Update (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_World_Fact_Update
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for World_Fact_Update";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_World_Fact_Update;

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

   function To_Binary_Planned_Component_Interaction (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Planned_Component_Interaction (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Planned_Component_Interaction";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Planned_Component_Interaction;

   function From_Binary_Planned_Component_Interaction (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Planned_Component_Interaction
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Planned_Component_Interaction";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Planned_Component_Interaction;

   function To_Binary_Plan_Step (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Plan_Step (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Plan_Step";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Plan_Step;

   function From_Binary_Plan_Step (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Plan_Step
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Plan_Step";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Plan_Step;

   function To_Binary_Plan (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Plan (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Plan";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Plan;

   function From_Binary_Plan (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Plan
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Plan";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Plan;

   function To_Binary_Requirement_Placement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Requirement_Placement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Requirement_Placement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Requirement_Placement;

   function From_Binary_Requirement_Placement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Requirement_Placement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Requirement_Placement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Requirement_Placement;

   function To_Binary_Achievement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Achievement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Achievement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Achievement;

   function From_Binary_Achievement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Achievement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Achievement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Achievement;

   function To_Binary_Entity (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Entity (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Entity";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Entity;

   function From_Binary_Entity (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Entity
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Entity";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Entity;

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

   function To_Binary_Planning_Execution_Requirement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Planning_Execution_Requirement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Planning_Execution_Requirement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Planning_Execution_Requirement;

   function From_Binary_Planning_Execution_Requirement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Planning_Execution_Requirement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Planning_Execution_Requirement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Planning_Execution_Requirement;

   function To_Binary_Execution_Run (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Execution_Run (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Execution_Run";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Execution_Run;

   function From_Binary_Execution_Run (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Execution_Run
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Execution_Run";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Execution_Run;

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

   function To_Binary_Planning_Execution_Requirement_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Planning_Execution_Requirement_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Planning_Execution_Requirement_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Planning_Execution_Requirement_Array;

   function From_Binary_Planning_Execution_Requirement_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Planning_Execution_Requirement_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Planning_Execution_Requirement_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Planning_Execution_Requirement_Array;

   function To_Binary_Plan_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Plan_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Plan_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Plan_Array;

   function From_Binary_Plan_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Plan_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Plan_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Plan_Array;

   function To_Binary_Execution_Run_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Execution_Run_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Execution_Run_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Execution_Run_Array;

   function From_Binary_Execution_Run_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Execution_Run_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Execution_Run_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Execution_Run_Array;

   function To_Binary_Requirement_Placement_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Requirement_Placement_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Requirement_Placement_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Requirement_Placement_Array;

   function From_Binary_Requirement_Placement_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Requirement_Placement_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Requirement_Placement_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Requirement_Placement_Array;

   function To_Binary_Requirement_Reference (Msg : Pyramid.Data_Model.Autonomy.Types.Requirement_Reference) return String is
   begin
      return To_Binary_Requirement_Reference (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Requirement_Reference;

   function From_Binary_Requirement_Reference
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Requirement_Reference) return Pyramid.Data_Model.Autonomy.Types.Requirement_Reference
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Requirement_Reference (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Requirement_Reference;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Requirement_Reference;

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

   function To_Binary_Planning_Policy (Msg : Pyramid.Data_Model.Autonomy.Types.Planning_Policy) return String is
   begin
      return To_Binary_Planning_Policy (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Planning_Policy;

   function From_Binary_Planning_Policy
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planning_Policy) return Pyramid.Data_Model.Autonomy.Types.Planning_Policy
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Planning_Policy (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Planning_Policy;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Planning_Policy;

   function To_Binary_Planning_Goal (Msg : Pyramid.Data_Model.Autonomy.Types.Planning_Goal) return String is
   begin
      return To_Binary_Planning_Goal (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Planning_Goal;

   function From_Binary_Planning_Goal
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planning_Goal) return Pyramid.Data_Model.Autonomy.Types.Planning_Goal
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Planning_Goal (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Planning_Goal;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Planning_Goal;

   function To_Binary_World_Fact_Update (Msg : Pyramid.Data_Model.Autonomy.Types.World_Fact_Update) return String is
   begin
      return To_Binary_World_Fact_Update (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_World_Fact_Update;

   function From_Binary_World_Fact_Update
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.World_Fact_Update) return Pyramid.Data_Model.Autonomy.Types.World_Fact_Update
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_World_Fact_Update (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.World_Fact_Update;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_World_Fact_Update;

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

   function To_Binary_Planned_Component_Interaction (Msg : Pyramid.Data_Model.Autonomy.Types.Planned_Component_Interaction) return String is
   begin
      return To_Binary_Planned_Component_Interaction (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Planned_Component_Interaction;

   function From_Binary_Planned_Component_Interaction
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planned_Component_Interaction) return Pyramid.Data_Model.Autonomy.Types.Planned_Component_Interaction
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Planned_Component_Interaction (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Planned_Component_Interaction;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Planned_Component_Interaction;

   function To_Binary_Plan_Step (Msg : Pyramid.Data_Model.Autonomy.Types.Plan_Step) return String is
   begin
      return To_Binary_Plan_Step (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Plan_Step;

   function From_Binary_Plan_Step
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Plan_Step) return Pyramid.Data_Model.Autonomy.Types.Plan_Step
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Plan_Step (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Plan_Step;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Plan_Step;

   function To_Binary_Plan (Msg : Pyramid.Data_Model.Autonomy.Types.Plan) return String is
   begin
      return To_Binary_Plan (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Plan;

   function From_Binary_Plan
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Plan) return Pyramid.Data_Model.Autonomy.Types.Plan
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Plan (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Plan;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Plan;

   function To_Binary_Requirement_Placement (Msg : Pyramid.Data_Model.Autonomy.Types.Requirement_Placement) return String is
   begin
      return To_Binary_Requirement_Placement (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Requirement_Placement;

   function From_Binary_Requirement_Placement
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Requirement_Placement) return Pyramid.Data_Model.Autonomy.Types.Requirement_Placement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Requirement_Placement (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Requirement_Placement;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Requirement_Placement;

   function To_Binary_Achievement (Msg : Pyramid.Data_Model.Common.Types.Achievement) return String is
   begin
      return To_Binary_Achievement (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Achievement;

   function From_Binary_Achievement
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Achievement) return Pyramid.Data_Model.Common.Types.Achievement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Achievement (Payload);
      Result : Pyramid.Data_Model.Common.Types.Achievement;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Achievement;

   function To_Binary_Entity (Msg : Pyramid.Data_Model.Common.Types.Entity) return String is
   begin
      return To_Binary_Entity (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Entity;

   function From_Binary_Entity
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Entity) return Pyramid.Data_Model.Common.Types.Entity
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Entity (Payload);
      Result : Pyramid.Data_Model.Common.Types.Entity;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Entity;

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

   function To_Binary_Planning_Execution_Requirement (Msg : Pyramid.Data_Model.Autonomy.Types.Planning_Execution_Requirement) return String is
   begin
      return To_Binary_Planning_Execution_Requirement (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Planning_Execution_Requirement;

   function From_Binary_Planning_Execution_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planning_Execution_Requirement) return Pyramid.Data_Model.Autonomy.Types.Planning_Execution_Requirement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Planning_Execution_Requirement (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Planning_Execution_Requirement;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Planning_Execution_Requirement;

   function To_Binary_Execution_Run (Msg : Pyramid.Data_Model.Autonomy.Types.Execution_Run) return String is
   begin
      return To_Binary_Execution_Run (Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Msg));
   end To_Binary_Execution_Run;

   function From_Binary_Execution_Run
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Execution_Run) return Pyramid.Data_Model.Autonomy.Types.Execution_Run
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Execution_Run (Payload);
      Result : Pyramid.Data_Model.Autonomy.Types.Execution_Run;
   begin
      Result := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Execution_Run;

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
