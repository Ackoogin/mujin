--  Auto-generated service binding body
--  Package body: Pyramid.Services.Autonomy_Backend.Provided

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Plugins;
with System;
with System.Address_To_Access_Conversions;
with System.Storage_Elements;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Data_Model.Autonomy.Cabi;  use Pyramid.Data_Model.Autonomy.Cabi;
with Pyramid.Data_Model.Base.Cabi;  use Pyramid.Data_Model.Base.Cabi;
with Pyramid.Data_Model.Common.Cabi;  use Pyramid.Data_Model.Common.Cabi;

package body Pyramid.Services.Autonomy_Backend.Provided is
   use type System.Address;
   use type Interfaces.C.unsigned;
   use type Interfaces.C.Strings.chars_ptr;
   use type Pcl_Bindings.Pcl_Resp_Cb_Access;
   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Plugins.Pcl_Codec_Const_Access;
   use type Pcl_Plugins.Pcl_Codec_Decode_Access;
   use type Pcl_Plugins.Pcl_Codec_Encode_Access;
   use type Pcl_Plugins.Pcl_Codec_Free_Msg_Access;

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

   function Pcl_Codec_Registry_Get_At
     (Registry     : System.Address;
      Content_Type : Interfaces.C.Strings.chars_ptr;
      Index        : Interfaces.C.unsigned)
      return Pcl_Plugins.Pcl_Codec_Const_Access;
   pragma Import (C, Pcl_Codec_Registry_Get_At,
                  "pcl_codec_registry_get_at");

   procedure C_Free (Ptr : System.Address);
   pragma Import (C, C_Free, "free");

   type Service_Handlers_Access is access constant Service_Handlers;

   function To_Handlers is new
     Ada.Unchecked_Conversion (System.Address, Service_Handlers_Access);

   package Requirement_Reference_Pointers is new
     System.Address_To_Access_Conversions (Requirement_Reference);

   package Agent_State_Pointers is new
     System.Address_To_Access_Conversions (Agent_State);

   package Planning_Policy_Pointers is new
     System.Address_To_Access_Conversions (Planning_Policy);

   package Planning_Goal_Pointers is new
     System.Address_To_Access_Conversions (Planning_Goal);

   package Execution_Policy_Pointers is new
     System.Address_To_Access_Conversions (Execution_Policy);

   package Planning_Requirement_Pointers is new
     System.Address_To_Access_Conversions (Planning_Requirement);

   package Execution_Requirement_Pointers is new
     System.Address_To_Access_Conversions (Execution_Requirement);

   package World_Fact_Update_Pointers is new
     System.Address_To_Access_Conversions (World_Fact_Update);

   package State_Update_Pointers is new
     System.Address_To_Access_Conversions (State_Update);

   package Capabilities_Pointers is new
     System.Address_To_Access_Conversions (Capabilities);

   package Planned_Component_Interaction_Pointers is new
     System.Address_To_Access_Conversions (Planned_Component_Interaction);

   package Plan_Step_Pointers is new
     System.Address_To_Access_Conversions (Plan_Step);

   package Plan_Pointers is new
     System.Address_To_Access_Conversions (Plan);

   package Requirement_Placement_Pointers is new
     System.Address_To_Access_Conversions (Requirement_Placement);

   package Execution_Run_Pointers is new
     System.Address_To_Access_Conversions (Execution_Run);

   package Geodetic_Position_Pointers is new
     System.Address_To_Access_Conversions (Geodetic_Position);

   package Poly_Area_Pointers is new
     System.Address_To_Access_Conversions (Poly_Area);

   package Achievement_Pointers is new
     System.Address_To_Access_Conversions (Achievement);

   package Requirement_Pointers is new
     System.Address_To_Access_Conversions (Requirement);

   package Capability_Pointers is new
     System.Address_To_Access_Conversions (Capability);

   package Entity_Pointers is new
     System.Address_To_Access_Conversions (Entity);

   package Circle_Area_Pointers is new
     System.Address_To_Access_Conversions (Circle_Area);

   package Point_Pointers is new
     System.Address_To_Access_Conversions (Point);

   package Contraint_Pointers is new
     System.Address_To_Access_Conversions (Contraint);

   package Ack_Pointers is new
     System.Address_To_Access_Conversions (Ack);

   package Query_Pointers is new
     System.Address_To_Access_Conversions (Query);

   package Identifier_Pointers is new
     System.Address_To_Access_Conversions (Identifier);

   function Handler_Address
     (Handlers : access constant Service_Handlers) return System.Address is
   begin
      if Handlers = null then
         return System.Null_Address;
      end if;

      return Handlers.all'Address;
   end Handler_Address;

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String
   is
      use System.Storage_Elements;
      type Char_Array is array (1 .. Natural (Size)) of Character;
      pragma Pack (Char_Array);
      Chars : Char_Array;
      for Chars'Address use Data;
      pragma Import (Ada, Chars);
   begin
      return String (Chars);
   end Msg_To_String;

   function Registry_Has_Codec (Content_Type : String) return Boolean is
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Content_Type);
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
   begin
      if Content_Type = "" then
         Interfaces.C.Strings.Free (Content_C);
         return False;
      end if;
      Codec := Pcl_Plugins.Pcl_Codec_Registry_Get
        (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C);
      Interfaces.C.Strings.Free (Content_C);
      return Codec /= null;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         return False;
   end Registry_Has_Codec;

   procedure Require_Codec (Content_Type : String) is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
   begin
      if not Registry_Has_Codec (Effective) then
         raise Program_Error with
           "fail-closed: no codec plugin registered for content type "
           & Effective;
      end if;
   end Require_Codec;

   function Try_Cabi_Registry_Encode
     (Codec     : Pcl_Plugins.Pcl_Codec_Const_Access;
      Schema_C  : Interfaces.C.Strings.chars_ptr;
      Schema_Id : String;
      Value     : System.Address;
      Msg       : access Pcl_Bindings.Pcl_Msg)
      return Pcl_Bindings.Pcl_Status
   is
   begin
      if Codec = null or else Codec.all.Encode = null then
         return Pcl_Bindings.PCL_ERR_INVALID;
      end if;
      if Schema_Id = "RequirementReference" then
         declare
            Native : constant Requirement_Reference_Pointers.Object_Pointer :=
              Requirement_Reference_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_Reference_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Requirement_Reference (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "AgentState" then
         declare
            Native : constant Agent_State_Pointers.Object_Pointer :=
              Agent_State_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Agent_State_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Agent_State (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanningPolicy" then
         declare
            Native : constant Planning_Policy_Pointers.Object_Pointer :=
              Planning_Policy_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planning_Policy_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Planning_Policy (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanningGoal" then
         declare
            Native : constant Planning_Goal_Pointers.Object_Pointer :=
              Planning_Goal_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planning_Goal_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Planning_Goal (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ExecutionPolicy" then
         declare
            Native : constant Execution_Policy_Pointers.Object_Pointer :=
              Execution_Policy_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Execution_Policy_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Execution_Policy (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanningRequirement" then
         declare
            Native : constant Planning_Requirement_Pointers.Object_Pointer :=
              Planning_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planning_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Planning_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ExecutionRequirement" then
         declare
            Native : constant Execution_Requirement_Pointers.Object_Pointer :=
              Execution_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Execution_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Execution_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "WorldFactUpdate" then
         declare
            Native : constant World_Fact_Update_Pointers.Object_Pointer :=
              World_Fact_Update_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_World_Fact_Update_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_World_Fact_Update (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "StateUpdate" then
         declare
            Native : constant State_Update_Pointers.Object_Pointer :=
              State_Update_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_State_Update_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_State_Update (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Capabilities" then
         declare
            Native : constant Capabilities_Pointers.Object_Pointer :=
              Capabilities_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Capabilities_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Capabilities (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlannedComponentInteraction" then
         declare
            Native : constant Planned_Component_Interaction_Pointers.Object_Pointer :=
              Planned_Component_Interaction_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planned_Component_Interaction_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Planned_Component_Interaction (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanStep" then
         declare
            Native : constant Plan_Step_Pointers.Object_Pointer :=
              Plan_Step_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Plan_Step_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Plan_Step (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Plan" then
         declare
            Native : constant Plan_Pointers.Object_Pointer :=
              Plan_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Plan_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Plan (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "RequirementPlacement" then
         declare
            Native : constant Requirement_Placement_Pointers.Object_Pointer :=
              Requirement_Placement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_Placement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Requirement_Placement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ExecutionRun" then
         declare
            Native : constant Execution_Run_Pointers.Object_Pointer :=
              Execution_Run_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Execution_Run_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Execution_Run (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "GeodeticPosition" then
         declare
            Native : constant Geodetic_Position_Pointers.Object_Pointer :=
              Geodetic_Position_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Geodetic_Position_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Geodetic_Position (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PolyArea" then
         declare
            Native : constant Poly_Area_Pointers.Object_Pointer :=
              Poly_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Poly_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Poly_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Achievement" then
         declare
            Native : constant Achievement_Pointers.Object_Pointer :=
              Achievement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Achievement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Achievement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Requirement" then
         declare
            Native : constant Requirement_Pointers.Object_Pointer :=
              Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Capability" then
         declare
            Native : constant Capability_Pointers.Object_Pointer :=
              Capability_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Capability_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Capability (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Entity" then
         declare
            Native : constant Entity_Pointers.Object_Pointer :=
              Entity_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Entity_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Entity (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "CircleArea" then
         declare
            Native : constant Circle_Area_Pointers.Object_Pointer :=
              Circle_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Circle_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Circle_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Point" then
         declare
            Native : constant Point_Pointers.Object_Pointer :=
              Point_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Point_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Point (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Contraint" then
         declare
            Native : constant Contraint_Pointers.Object_Pointer :=
              Contraint_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Contraint_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Contraint (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Ack" then
         declare
            Native : constant Ack_Pointers.Object_Pointer :=
              Ack_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Ack_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Ack (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Query" then
         declare
            Native : constant Query_Pointers.Object_Pointer :=
              Query_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Query_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Query (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Angle" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Length" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Timestamp" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Identifier" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         declare
            Native : constant Identifier_Pointers.Object_Pointer :=
              Identifier_Pointers.To_Pointer (Value);
            S : constant String := To_String (Native.all);
            C_Value : aliased Pyramid.Data_Model.Base.Cabi.Pyramid_Str_T;
            Status : Pcl_Bindings.Pcl_Status;
         begin
            C_Value.Ptr := Interfaces.C.Strings.New_String (S);
            C_Value.Len := Interfaces.C.unsigned (S'Length);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Interfaces.C.Strings.Free (C_Value.Ptr);
            return Status;
         end;
      end if;
      if Schema_Id = "Speed" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Percentage" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      return Pcl_Bindings.PCL_ERR_NOT_FOUND;
   end Try_Cabi_Registry_Encode;

   function Try_Cabi_Registry_Decode
     (Codec     : Pcl_Plugins.Pcl_Codec_Const_Access;
      Schema_C  : Interfaces.C.Strings.chars_ptr;
      Schema_Id : String;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      Value     : System.Address)
      return Pcl_Bindings.Pcl_Status
   is
   begin
      if Codec = null or else Codec.all.Decode = null then
         return Pcl_Bindings.PCL_ERR_INVALID;
      end if;
      if Schema_Id = "RequirementReference" then
         declare
            Native : constant Requirement_Reference_Pointers.Object_Pointer :=
              Requirement_Reference_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_Reference_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Requirement_Reference (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "AgentState" then
         declare
            Native : constant Agent_State_Pointers.Object_Pointer :=
              Agent_State_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Agent_State_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Agent_State (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanningPolicy" then
         declare
            Native : constant Planning_Policy_Pointers.Object_Pointer :=
              Planning_Policy_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planning_Policy_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Planning_Policy (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanningGoal" then
         declare
            Native : constant Planning_Goal_Pointers.Object_Pointer :=
              Planning_Goal_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planning_Goal_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Planning_Goal (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ExecutionPolicy" then
         declare
            Native : constant Execution_Policy_Pointers.Object_Pointer :=
              Execution_Policy_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Execution_Policy_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Execution_Policy (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanningRequirement" then
         declare
            Native : constant Planning_Requirement_Pointers.Object_Pointer :=
              Planning_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planning_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Planning_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ExecutionRequirement" then
         declare
            Native : constant Execution_Requirement_Pointers.Object_Pointer :=
              Execution_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Execution_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Execution_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "WorldFactUpdate" then
         declare
            Native : constant World_Fact_Update_Pointers.Object_Pointer :=
              World_Fact_Update_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_World_Fact_Update_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_World_Fact_Update (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "StateUpdate" then
         declare
            Native : constant State_Update_Pointers.Object_Pointer :=
              State_Update_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_State_Update_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_State_Update (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Capabilities" then
         declare
            Native : constant Capabilities_Pointers.Object_Pointer :=
              Capabilities_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Capabilities_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Capabilities (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlannedComponentInteraction" then
         declare
            Native : constant Planned_Component_Interaction_Pointers.Object_Pointer :=
              Planned_Component_Interaction_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Planned_Component_Interaction_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Planned_Component_Interaction (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PlanStep" then
         declare
            Native : constant Plan_Step_Pointers.Object_Pointer :=
              Plan_Step_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Plan_Step_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Plan_Step (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Plan" then
         declare
            Native : constant Plan_Pointers.Object_Pointer :=
              Plan_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Plan_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Plan (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "RequirementPlacement" then
         declare
            Native : constant Requirement_Placement_Pointers.Object_Pointer :=
              Requirement_Placement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_Placement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Requirement_Placement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ExecutionRun" then
         declare
            Native : constant Execution_Run_Pointers.Object_Pointer :=
              Execution_Run_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Execution_Run_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Execution_Run (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "GeodeticPosition" then
         declare
            Native : constant Geodetic_Position_Pointers.Object_Pointer :=
              Geodetic_Position_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Geodetic_Position_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Geodetic_Position (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PolyArea" then
         declare
            Native : constant Poly_Area_Pointers.Object_Pointer :=
              Poly_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Poly_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Poly_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Achievement" then
         declare
            Native : constant Achievement_Pointers.Object_Pointer :=
              Achievement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Achievement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Achievement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Requirement" then
         declare
            Native : constant Requirement_Pointers.Object_Pointer :=
              Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Capability" then
         declare
            Native : constant Capability_Pointers.Object_Pointer :=
              Capability_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Capability_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Capability (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Entity" then
         declare
            Native : constant Entity_Pointers.Object_Pointer :=
              Entity_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Entity_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Entity (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "CircleArea" then
         declare
            Native : constant Circle_Area_Pointers.Object_Pointer :=
              Circle_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Circle_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Circle_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Point" then
         declare
            Native : constant Point_Pointers.Object_Pointer :=
              Point_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Point_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Point (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Contraint" then
         declare
            Native : constant Contraint_Pointers.Object_Pointer :=
              Contraint_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Contraint_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Contraint (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Ack" then
         declare
            Native : constant Ack_Pointers.Object_Pointer :=
              Ack_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Ack_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Ack (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Query" then
         declare
            Native : constant Query_Pointers.Object_Pointer :=
              Query_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Query_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Query (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Angle" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Length" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Timestamp" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Identifier" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         declare
            Native : constant Identifier_Pointers.Object_Pointer :=
              Identifier_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid.Data_Model.Base.Cabi.Pyramid_Str_T;
            Status : Pcl_Bindings.Pcl_Status;
         begin
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               if C_Value.Ptr = Interfaces.C.Strings.Null_Ptr then
                  Native.all := Null_Unbounded_String;
               else
                  Native.all := To_Unbounded_String
                    (Interfaces.C.Strings.Value
                       (C_Value.Ptr,
                        Interfaces.C.size_t (C_Value.Len)));
               end if;
            end if;
            if C_Value.Ptr /= Interfaces.C.Strings.Null_Ptr then
               Interfaces.C.Strings.Free (C_Value.Ptr);
            end if;
            return Status;
         end;
      end if;
      if Schema_Id = "Speed" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Percentage" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      return Pcl_Bindings.PCL_ERR_NOT_FOUND;
   end Try_Cabi_Registry_Decode;

   function Try_Registry_Encode
     (Content_Type : String;
      Schema_Id    : String;
      Value        : System.Address;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Schema_Id);
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      if Effective = "" then
         Interfaces.C.Strings.Free (Content_C);
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            Status := Try_Cabi_Registry_Encode
              (Codec, Schema_C, Schema_Id, Value, Msg'Access);
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode;

   function Try_Registry_Decode
     (Msg       : access constant Pcl_Bindings.Pcl_Msg;
      Schema_Id : String;
      Value     : System.Address) return Boolean
   is
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Schema_Id);
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      if Msg = null then
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Status := Try_Cabi_Registry_Decode
              (Codec, Schema_C, Schema_Id, Msg, Value);
            if Status = Pcl_Bindings.PCL_OK then
               Interfaces.C.Strings.Free (Schema_C);
               if Type_C /= Interfaces.C.Strings.Null_Ptr then
                  Interfaces.C.Strings.Free (Type_C);
               end if;
               return True;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Schema_C);
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
      end if;
      return False;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         return False;
   end Try_Registry_Decode;

   function Try_Registry_Decode_Raw
     (Content_Type : String;
      Data         : System.Address;
      Size         : Natural;
      Schema_Id    : String;
      Value        : System.Address) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Type_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Msg : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => Data,
         Size      => Interfaces.C.unsigned (Size),
         Type_Name => Type_C);
      Ok : Boolean := False;
   begin
      Ok := Try_Registry_Decode (Msg'Access, Schema_Id, Value);
      Interfaces.C.Strings.Free (Type_C);
      return Ok;
   exception
      when others =>
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         return False;
   end Try_Registry_Decode_Raw;

   function Try_Registry_Encode_Capabilities_Array
     (Content_Type : String;
      Payload      : Capabilities_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("CapabilitiesArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Capabilities_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Capabilities_C;
                     pragma Convention (C, Capabilities_Array_C_Array);
                     Values : Capabilities_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Capabilities (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Capabilities_Array;

   function Registry_Decode_Capabilities_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Capabilities_Array
   is
      Empty : Capabilities_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("CapabilitiesArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema CapabilitiesArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema CapabilitiesArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Capabilities_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Capabilities_C;
         pragma Convention (C, Capabilities_Array_C_Array);
         Values : Capabilities_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Capabilities_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Capabilities (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Capabilities_Array;

   function Try_Registry_Encode_Planning_Requirement_Array
     (Content_Type : String;
      Payload      : Planning_Requirement_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("PlanningRequirementArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Planning_Requirement_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Planning_Requirement_C;
                     pragma Convention (C, Planning_Requirement_Array_C_Array);
                     Values : Planning_Requirement_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Planning_Requirement (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Planning_Requirement_Array;

   function Registry_Decode_Planning_Requirement_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Planning_Requirement_Array
   is
      Empty : Planning_Requirement_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("PlanningRequirementArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema PlanningRequirementArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema PlanningRequirementArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Planning_Requirement_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Planning_Requirement_C;
         pragma Convention (C, Planning_Requirement_Array_C_Array);
         Values : Planning_Requirement_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Planning_Requirement_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Planning_Requirement (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Planning_Requirement_Array;

   function Try_Registry_Encode_Execution_Requirement_Array
     (Content_Type : String;
      Payload      : Execution_Requirement_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ExecutionRequirementArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Execution_Requirement_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Execution_Requirement_C;
                     pragma Convention (C, Execution_Requirement_Array_C_Array);
                     Values : Execution_Requirement_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Execution_Requirement (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Execution_Requirement_Array;

   function Registry_Decode_Execution_Requirement_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Requirement_Array
   is
      Empty : Execution_Requirement_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ExecutionRequirementArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema ExecutionRequirementArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema ExecutionRequirementArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Execution_Requirement_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Execution_Requirement_C;
         pragma Convention (C, Execution_Requirement_Array_C_Array);
         Values : Execution_Requirement_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Execution_Requirement_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Execution_Requirement (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Execution_Requirement_Array;

   function Try_Registry_Encode_Plan_Array
     (Content_Type : String;
      Payload      : Plan_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("PlanArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Plan_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Plan_C;
                     pragma Convention (C, Plan_Array_C_Array);
                     Values : Plan_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Plan (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Plan_Array;

   function Registry_Decode_Plan_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Plan_Array
   is
      Empty : Plan_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("PlanArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema PlanArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema PlanArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Plan_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Plan_C;
         pragma Convention (C, Plan_Array_C_Array);
         Values : Plan_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Plan_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Plan (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Plan_Array;

   function Try_Registry_Encode_Execution_Run_Array
     (Content_Type : String;
      Payload      : Execution_Run_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ExecutionRunArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Execution_Run_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Execution_Run_C;
                     pragma Convention (C, Execution_Run_Array_C_Array);
                     Values : Execution_Run_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Execution_Run (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Execution_Run_Array;

   function Registry_Decode_Execution_Run_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Run_Array
   is
      Empty : Execution_Run_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ExecutionRunArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema ExecutionRunArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema ExecutionRunArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Execution_Run_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Execution_Run_C;
         pragma Convention (C, Execution_Run_Array_C_Array);
         Values : Execution_Run_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Execution_Run_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Execution_Run (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Execution_Run_Array;

   function Try_Registry_Encode_Requirement_Placement_Array
     (Content_Type : String;
      Payload      : Requirement_Placement_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("RequirementPlacementArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Requirement_Placement_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Requirement_Placement_C;
                     pragma Convention (C, Requirement_Placement_Array_C_Array);
                     Values : Requirement_Placement_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Requirement_Placement (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Requirement_Placement_Array;

   function Registry_Decode_Requirement_Placement_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Requirement_Placement_Array
   is
      Empty : Requirement_Placement_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("RequirementPlacementArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Autonomy.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema RequirementPlacementArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema RequirementPlacementArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Requirement_Placement_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Requirement_Placement_C;
         pragma Convention (C, Requirement_Placement_Array_C_Array);
         Values : Requirement_Placement_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Requirement_Placement_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Requirement_Placement (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Requirement_Placement_Array;


   function Supports_Content_Type (Content_Type : String) return Boolean is
   begin
      return Registry_Has_Codec
        ((if Content_Type = "" then Json_Content_Type else Content_Type));
   end Supports_Content_Type;

   function Message_Content_Type
     (Msg : access constant Pcl_Bindings.Pcl_Msg) return String is
   begin
      if Msg = null or else Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         return Json_Content_Type;
      end if;
      return Interfaces.C.Strings.Value (Msg.Type_Name);
   end Message_Content_Type;

   function Decode_Capabilities_Read_Capabilities_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Capabilities_Array
   is
      Empty : Capabilities_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      return Registry_Decode_Capabilities_Array (Msg);
   end Decode_Capabilities_Read_Capabilities_Response;

   function Decode_Planning_Requirement_Create_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Null_Unbounded_String;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Identifier;
      begin
         if Try_Registry_Decode (Msg, "Identifier", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Identifier";
   end Decode_Planning_Requirement_Create_Planning_Requirement_Response;

   function Decode_Planning_Requirement_Read_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Planning_Requirement_Array
   is
      Empty : Planning_Requirement_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      return Registry_Decode_Planning_Requirement_Array (Msg);
   end Decode_Planning_Requirement_Read_Planning_Requirement_Response;

   function Decode_Planning_Requirement_Update_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_Planning_Requirement_Update_Planning_Requirement_Response;

   function Decode_Planning_Requirement_Delete_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_Planning_Requirement_Delete_Planning_Requirement_Response;

   function Decode_Execution_Requirement_Create_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Null_Unbounded_String;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Identifier;
      begin
         if Try_Registry_Decode (Msg, "Identifier", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Identifier";
   end Decode_Execution_Requirement_Create_Execution_Requirement_Response;

   function Decode_Execution_Requirement_Read_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Requirement_Array
   is
      Empty : Execution_Requirement_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      return Registry_Decode_Execution_Requirement_Array (Msg);
   end Decode_Execution_Requirement_Read_Execution_Requirement_Response;

   function Decode_Execution_Requirement_Update_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_Execution_Requirement_Update_Execution_Requirement_Response;

   function Decode_Execution_Requirement_Delete_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_Execution_Requirement_Delete_Execution_Requirement_Response;

   function Decode_State_Create_State_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Null_Unbounded_String;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Identifier;
      begin
         if Try_Registry_Decode (Msg, "Identifier", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Identifier";
   end Decode_State_Create_State_Response;

   function Decode_State_Update_State_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_State_Update_State_Response;

   function Decode_State_Delete_State_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_State_Delete_State_Response;

   function Decode_Plan_Create_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Null_Unbounded_String;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Identifier;
      begin
         if Try_Registry_Decode (Msg, "Identifier", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Identifier";
   end Decode_Plan_Create_Plan_Response;

   function Decode_Plan_Read_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Plan_Array
   is
      Empty : Plan_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      return Registry_Decode_Plan_Array (Msg);
   end Decode_Plan_Read_Plan_Response;

   function Decode_Plan_Update_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_Plan_Update_Plan_Response;

   function Decode_Plan_Delete_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Ack;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema Ack";
   end Decode_Plan_Delete_Plan_Response;

   function Decode_Execution_Run_Read_Run_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Run_Array
   is
      Empty : Execution_Run_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      return Registry_Decode_Execution_Run_Array (Msg);
   end Decode_Execution_Run_Read_Run_Response;

   function Decode_Requirement_Placement_Read_Placement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Requirement_Placement_Array
   is
      Empty : Requirement_Placement_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      return Registry_Decode_Requirement_Placement_Array (Msg);
   end Decode_Requirement_Placement_Read_Placement_Response;

   --  -- Capabilities_Service ------------------------------------
   function Default_Handle_Capabilities_Read_Capabilities
     (Request : Query) return Capabilities_Array
   is
      pragma Unreferenced (Request);
      Empty : Capabilities_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Capabilities_Read_Capabilities;

   --  -- Planning_Requirement_Service ------------------------------------
   procedure Default_Handle_Planning_Requirement_Create_Planning_Requirement
     (Request  : in  Planning_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Planning_Requirement_Create_Planning_Requirement;

   function Default_Handle_Planning_Requirement_Read_Planning_Requirement
     (Request : Query) return Planning_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Planning_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Planning_Requirement_Read_Planning_Requirement;

   procedure Default_Handle_Planning_Requirement_Update_Planning_Requirement
     (Request  : in  Planning_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Planning_Requirement_Update_Planning_Requirement;

   procedure Default_Handle_Planning_Requirement_Delete_Planning_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Planning_Requirement_Delete_Planning_Requirement;

   --  -- Execution_Requirement_Service ------------------------------------
   procedure Default_Handle_Execution_Requirement_Create_Execution_Requirement
     (Request  : in  Execution_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Execution_Requirement_Create_Execution_Requirement;

   function Default_Handle_Execution_Requirement_Read_Execution_Requirement
     (Request : Query) return Execution_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Execution_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Execution_Requirement_Read_Execution_Requirement;

   procedure Default_Handle_Execution_Requirement_Update_Execution_Requirement
     (Request  : in  Execution_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Execution_Requirement_Update_Execution_Requirement;

   procedure Default_Handle_Execution_Requirement_Delete_Execution_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Execution_Requirement_Delete_Execution_Requirement;

   --  -- State_Service ------------------------------------
   procedure Default_Handle_State_Create_State
     (Request  : in  State_Update;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_State_Create_State;

   procedure Default_Handle_State_Update_State
     (Request  : in  State_Update;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_State_Update_State;

   procedure Default_Handle_State_Delete_State
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_State_Delete_State;

   --  -- Plan_Service ------------------------------------
   procedure Default_Handle_Plan_Create_Plan
     (Request  : in  Plan;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Plan_Create_Plan;

   function Default_Handle_Plan_Read_Plan
     (Request : Query) return Plan_Array
   is
      pragma Unreferenced (Request);
      Empty : Plan_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Plan_Read_Plan;

   procedure Default_Handle_Plan_Update_Plan
     (Request  : in  Plan;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Plan_Update_Plan;

   procedure Default_Handle_Plan_Delete_Plan
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Plan_Delete_Plan;

   --  -- Execution_Run_Service ------------------------------------
   function Default_Handle_Execution_Run_Read_Run
     (Request : Query) return Execution_Run_Array
   is
      pragma Unreferenced (Request);
      Empty : Execution_Run_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Execution_Run_Read_Run;

   --  -- Requirement_Placement_Service ------------------------------------
   function Default_Handle_Requirement_Placement_Read_Placement
     (Request : Query) return Requirement_Placement_Array
   is
      pragma Unreferenced (Request);
      Empty : Requirement_Placement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Requirement_Placement_Read_Placement;

   function Service_Capabilities_Read_Capabilities
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Capabilities_Read_Capabilities);

   function Service_Planning_Requirement_Create_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Planning_Requirement_Create_Planning_Requirement);

   function Service_Planning_Requirement_Read_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Planning_Requirement_Read_Planning_Requirement);

   function Service_Planning_Requirement_Update_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Planning_Requirement_Update_Planning_Requirement);

   function Service_Planning_Requirement_Delete_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Planning_Requirement_Delete_Planning_Requirement);

   function Service_Execution_Requirement_Create_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Execution_Requirement_Create_Execution_Requirement);

   function Service_Execution_Requirement_Read_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Execution_Requirement_Read_Execution_Requirement);

   function Service_Execution_Requirement_Update_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Execution_Requirement_Update_Execution_Requirement);

   function Service_Execution_Requirement_Delete_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Execution_Requirement_Delete_Execution_Requirement);

   function Service_State_Create_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_State_Create_State);

   function Service_State_Update_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_State_Update_State);

   function Service_State_Delete_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_State_Delete_State);

   function Service_Plan_Create_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Plan_Create_Plan);

   function Service_Plan_Read_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Plan_Read_Plan);

   function Service_Plan_Update_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Plan_Update_Plan);

   function Service_Plan_Delete_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Plan_Delete_Plan);

   function Service_Execution_Run_Read_Run
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Execution_Run_Read_Run);

   function Service_Requirement_Placement_Read_Placement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Requirement_Placement_Read_Placement);

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json")
   is
      Handler_Ptr : constant System.Address := Handler_Address (Handlers);
   begin
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Capabilities_Read_Capabilities);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Capabilities_Read_Capabilities'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Create_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Planning_Requirement_Create_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Read_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Planning_Requirement_Read_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Update_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Planning_Requirement_Update_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Delete_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Planning_Requirement_Delete_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Create_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Execution_Requirement_Create_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Read_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Execution_Requirement_Read_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Update_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Execution_Requirement_Update_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Delete_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Execution_Requirement_Delete_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_State_Create_State);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_State_Create_State'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_State_Update_State);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_State_Update_State'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_State_Delete_State);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_State_Delete_State'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Plan_Create_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Plan_Create_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Plan_Read_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Plan_Read_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Plan_Update_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Plan_Update_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Plan_Delete_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Plan_Delete_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Execution_Run_Read_Run);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Execution_Run_Read_Run'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Requirement_Placement_Read_Placement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Requirement_Placement_Read_Placement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
   end Register_Services;

   function Service_Capabilities_Read_Capabilities
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Capabilities_Read_Capabilities,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Capabilities_Read_Capabilities;

   function Service_Planning_Requirement_Create_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Planning_Requirement_Create_Planning_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Planning_Requirement_Create_Planning_Requirement;

   function Service_Planning_Requirement_Read_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Planning_Requirement_Read_Planning_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Planning_Requirement_Read_Planning_Requirement;

   function Service_Planning_Requirement_Update_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Planning_Requirement_Update_Planning_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Planning_Requirement_Update_Planning_Requirement;

   function Service_Planning_Requirement_Delete_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Planning_Requirement_Delete_Planning_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Planning_Requirement_Delete_Planning_Requirement;

   function Service_Execution_Requirement_Create_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Execution_Requirement_Create_Execution_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Execution_Requirement_Create_Execution_Requirement;

   function Service_Execution_Requirement_Read_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Execution_Requirement_Read_Execution_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Execution_Requirement_Read_Execution_Requirement;

   function Service_Execution_Requirement_Update_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Execution_Requirement_Update_Execution_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Execution_Requirement_Update_Execution_Requirement;

   function Service_Execution_Requirement_Delete_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Execution_Requirement_Delete_Execution_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Execution_Requirement_Delete_Execution_Requirement;

   function Service_State_Create_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_State_Create_State,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_State_Create_State;

   function Service_State_Update_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_State_Update_State,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_State_Update_State;

   function Service_State_Delete_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_State_Delete_State,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_State_Delete_State;

   function Service_Plan_Create_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Plan_Create_Plan,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Plan_Create_Plan;

   function Service_Plan_Read_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Plan_Read_Plan,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Plan_Read_Plan;

   function Service_Plan_Update_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Plan_Update_Plan,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Plan_Update_Plan;

   function Service_Plan_Delete_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Plan_Delete_Plan,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Plan_Delete_Plan;

   function Service_Execution_Run_Read_Run
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Execution_Run_Read_Run,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Execution_Run_Read_Run;

   function Service_Requirement_Placement_Read_Placement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Requirement_Placement_Read_Placement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Requirement_Placement_Read_Placement;

   --  -- PCL binding implementations -------------------------------

   procedure Invoke_Capabilities_Read_Capabilities
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Query";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Capabilities_Read_Capabilities);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Capabilities_Read_Capabilities;

   procedure Invoke_Planning_Requirement_Create_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Planning_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "PlanningRequirement", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema PlanningRequirement";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Create_Planning_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Planning_Requirement_Create_Planning_Requirement;

   procedure Invoke_Planning_Requirement_Read_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Query";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Read_Planning_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Planning_Requirement_Read_Planning_Requirement;

   procedure Invoke_Planning_Requirement_Update_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Planning_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "PlanningRequirement", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema PlanningRequirement";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Update_Planning_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Planning_Requirement_Update_Planning_Requirement;

   procedure Invoke_Planning_Requirement_Delete_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Identifier", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Identifier";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Planning_Requirement_Delete_Planning_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Planning_Requirement_Delete_Planning_Requirement;

   procedure Invoke_Execution_Requirement_Create_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Execution_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "ExecutionRequirement", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema ExecutionRequirement";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Create_Execution_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Execution_Requirement_Create_Execution_Requirement;

   procedure Invoke_Execution_Requirement_Read_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Query";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Read_Execution_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Execution_Requirement_Read_Execution_Requirement;

   procedure Invoke_Execution_Requirement_Update_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Execution_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "ExecutionRequirement", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema ExecutionRequirement";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Update_Execution_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Execution_Requirement_Update_Execution_Requirement;

   procedure Invoke_Execution_Requirement_Delete_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Identifier", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Identifier";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Execution_Requirement_Delete_Execution_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Execution_Requirement_Delete_Execution_Requirement;

   procedure Invoke_State_Create_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "StateUpdate", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema StateUpdate";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_State_Create_State);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_State_Create_State;

   procedure Invoke_State_Update_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "StateUpdate", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema StateUpdate";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_State_Update_State);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_State_Update_State;

   procedure Invoke_State_Delete_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Identifier", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Identifier";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_State_Delete_State);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_State_Delete_State;

   procedure Invoke_Plan_Create_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Plan;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Plan", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Plan";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Plan_Create_Plan);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Plan_Create_Plan;

   procedure Invoke_Plan_Read_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Query";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Plan_Read_Plan);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Plan_Read_Plan;

   procedure Invoke_Plan_Update_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Plan;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Plan", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Plan";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Plan_Update_Plan);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Plan_Update_Plan;

   procedure Invoke_Plan_Delete_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Identifier", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Identifier";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Plan_Delete_Plan);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Plan_Delete_Plan;

   procedure Invoke_Execution_Run_Read_Run
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Query";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Execution_Run_Read_Run);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Execution_Run_Read_Run;

   procedure Invoke_Requirement_Placement_Read_Placement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema Query";
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Requirement_Placement_Read_Placement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Requirement_Placement_Read_Placement;

   procedure Copy_To_Buf
     (S    : in  String;
      Buf  : out System.Address;
      Size : out Natural)
   is
      C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (S);
   begin
      Buf  := To_Address (C);
      Size := S'Length;
   end Copy_To_Buf;

   procedure Dispatch
     (Handlers      : access constant Service_Handlers := null;
      Channel       : in  Service_Channel;
      Request_Buf   : in  System.Address;
      Request_Size  : in  Natural;
      Content_Type  : in  String := "application/json";
      Response_Buf  : out System.Address;
      Response_Size : out Natural)
   is
      Request_Payload : constant String :=
        Msg_To_String (Request_Buf, Interfaces.C.unsigned (Request_Size));
   begin
      Response_Buf  := System.Null_Address;
      Response_Size := 0;
      case Channel is
         when Ch_Capabilities_Read_Capabilities =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Capabilities_Array :=
                 (if Handlers /= null and then Handlers.On_Capabilities_Read_Capabilities /= null
                  then Handlers.On_Capabilities_Read_Capabilities.all (Req)
                  else Default_Handle_Capabilities_Read_Capabilities (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Capabilities_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema CapabilitiesArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Planning_Requirement_Create_Planning_Requirement =>
            declare
               function Decode_Request return Planning_Requirement is
                  Result : Planning_Requirement;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "PlanningRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema PlanningRequirement";
               end Decode_Request;
               Req : constant Planning_Requirement := Decode_Request;
               Rsp : Identifier;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Planning_Requirement_Create_Planning_Requirement /= null then
                  Handlers.On_Planning_Requirement_Create_Planning_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Planning_Requirement_Create_Planning_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Identifier", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Identifier";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Planning_Requirement_Read_Planning_Requirement =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Planning_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Planning_Requirement_Read_Planning_Requirement /= null
                  then Handlers.On_Planning_Requirement_Read_Planning_Requirement.all (Req)
                  else Default_Handle_Planning_Requirement_Read_Planning_Requirement (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Planning_Requirement_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema PlanningRequirementArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Planning_Requirement_Update_Planning_Requirement =>
            declare
               function Decode_Request return Planning_Requirement is
                  Result : Planning_Requirement;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "PlanningRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema PlanningRequirement";
               end Decode_Request;
               Req : constant Planning_Requirement := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Planning_Requirement_Update_Planning_Requirement /= null then
                  Handlers.On_Planning_Requirement_Update_Planning_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Planning_Requirement_Update_Planning_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Planning_Requirement_Delete_Planning_Requirement =>
            declare
               function Decode_Request return Identifier is
                  Result : Identifier;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Identifier", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Identifier";
               end Decode_Request;
               Req : constant Identifier := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Planning_Requirement_Delete_Planning_Requirement /= null then
                  Handlers.On_Planning_Requirement_Delete_Planning_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Planning_Requirement_Delete_Planning_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Execution_Requirement_Create_Execution_Requirement =>
            declare
               function Decode_Request return Execution_Requirement is
                  Result : Execution_Requirement;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "ExecutionRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema ExecutionRequirement";
               end Decode_Request;
               Req : constant Execution_Requirement := Decode_Request;
               Rsp : Identifier;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Execution_Requirement_Create_Execution_Requirement /= null then
                  Handlers.On_Execution_Requirement_Create_Execution_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Execution_Requirement_Create_Execution_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Identifier", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Identifier";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Execution_Requirement_Read_Execution_Requirement =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Execution_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Execution_Requirement_Read_Execution_Requirement /= null
                  then Handlers.On_Execution_Requirement_Read_Execution_Requirement.all (Req)
                  else Default_Handle_Execution_Requirement_Read_Execution_Requirement (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Execution_Requirement_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema ExecutionRequirementArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Execution_Requirement_Update_Execution_Requirement =>
            declare
               function Decode_Request return Execution_Requirement is
                  Result : Execution_Requirement;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "ExecutionRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema ExecutionRequirement";
               end Decode_Request;
               Req : constant Execution_Requirement := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Execution_Requirement_Update_Execution_Requirement /= null then
                  Handlers.On_Execution_Requirement_Update_Execution_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Execution_Requirement_Update_Execution_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Execution_Requirement_Delete_Execution_Requirement =>
            declare
               function Decode_Request return Identifier is
                  Result : Identifier;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Identifier", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Identifier";
               end Decode_Request;
               Req : constant Identifier := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Execution_Requirement_Delete_Execution_Requirement /= null then
                  Handlers.On_Execution_Requirement_Delete_Execution_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Execution_Requirement_Delete_Execution_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_State_Create_State =>
            declare
               function Decode_Request return State_Update is
                  Result : State_Update;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "StateUpdate", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema StateUpdate";
               end Decode_Request;
               Req : constant State_Update := Decode_Request;
               Rsp : Identifier;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_State_Create_State /= null then
                  Handlers.On_State_Create_State.all (Req, Rsp);
               else
                  Default_Handle_State_Create_State (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Identifier", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Identifier";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_State_Update_State =>
            declare
               function Decode_Request return State_Update is
                  Result : State_Update;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "StateUpdate", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema StateUpdate";
               end Decode_Request;
               Req : constant State_Update := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_State_Update_State /= null then
                  Handlers.On_State_Update_State.all (Req, Rsp);
               else
                  Default_Handle_State_Update_State (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_State_Delete_State =>
            declare
               function Decode_Request return Identifier is
                  Result : Identifier;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Identifier", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Identifier";
               end Decode_Request;
               Req : constant Identifier := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_State_Delete_State /= null then
                  Handlers.On_State_Delete_State.all (Req, Rsp);
               else
                  Default_Handle_State_Delete_State (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Plan_Create_Plan =>
            declare
               function Decode_Request return Plan is
                  Result : Plan;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Plan", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Plan";
               end Decode_Request;
               Req : constant Plan := Decode_Request;
               Rsp : Identifier;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Plan_Create_Plan /= null then
                  Handlers.On_Plan_Create_Plan.all (Req, Rsp);
               else
                  Default_Handle_Plan_Create_Plan (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Identifier", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Identifier";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Plan_Read_Plan =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Plan_Array :=
                 (if Handlers /= null and then Handlers.On_Plan_Read_Plan /= null
                  then Handlers.On_Plan_Read_Plan.all (Req)
                  else Default_Handle_Plan_Read_Plan (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Plan_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema PlanArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Plan_Update_Plan =>
            declare
               function Decode_Request return Plan is
                  Result : Plan;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Plan", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Plan";
               end Decode_Request;
               Req : constant Plan := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Plan_Update_Plan /= null then
                  Handlers.On_Plan_Update_Plan.all (Req, Rsp);
               else
                  Default_Handle_Plan_Update_Plan (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Plan_Delete_Plan =>
            declare
               function Decode_Request return Identifier is
                  Result : Identifier;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Identifier", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Identifier";
               end Decode_Request;
               Req : constant Identifier := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Plan_Delete_Plan /= null then
                  Handlers.On_Plan_Delete_Plan.all (Req, Rsp);
               else
                  Default_Handle_Plan_Delete_Plan (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Execution_Run_Read_Run =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Execution_Run_Array :=
                 (if Handlers /= null and then Handlers.On_Execution_Run_Read_Run /= null
                  then Handlers.On_Execution_Run_Read_Run.all (Req)
                  else Default_Handle_Execution_Run_Read_Run (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Execution_Run_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema ExecutionRunArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Requirement_Placement_Read_Placement =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Requirement_Placement_Array :=
                 (if Handlers /= null and then Handlers.On_Requirement_Placement_Read_Placement /= null
                  then Handlers.On_Requirement_Placement_Read_Placement.all (Req)
                  else Default_Handle_Requirement_Placement_Read_Placement (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Requirement_Placement_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema RequirementPlacementArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Autonomy_Backend.Provided;
