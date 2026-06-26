--  Auto-generated Ada C-ABI mirror specification
--  Generated from: pyramid.data_model.autonomy.proto by generate_bindings.py (ada cabi)
--  Package: Pyramid.Data_Model.Autonomy.Cabi

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Base.Cabi;  use Pyramid.Data_Model.Base.Cabi;
with Pyramid.Data_Model.Common.Cabi;  use Pyramid.Data_Model.Common.Cabi;

package Pyramid.Data_Model.Autonomy.Cabi is

   pragma Elaborate_Body;

   type Pyramid_Str_T is record
      Ptr : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.Null_Ptr;
      Len : Interfaces.C.unsigned := 0;
   end record;
   pragma Convention (C, Pyramid_Str_T);

   type Pyramid_Slice_T is record
      Ptr : System.Address := System.Null_Address;
      Len : Interfaces.C.unsigned := 0;
   end record;
   pragma Convention (C, Pyramid_Slice_T);

   type Pyramid_Requirement_Reference_C is record
      Requirement_Id : Pyramid_Str_T;
      Component_Name : Pyramid_Str_T;
      Service_Name : Pyramid_Str_T;
      Type_Name : Pyramid_Str_T;
   end record;
   pragma Convention (C, Pyramid_Requirement_Reference_C);

   type Pyramid_Agent_State_C is record
      Agent_Id : Pyramid_Str_T;
      Agent_Type : Pyramid_Str_T;
      Available : Interfaces.C.unsigned_char;
   end record;
   pragma Convention (C, Pyramid_Agent_State_C);

   type Pyramid_Planning_Policy_C is record
      Max_Replans : Interfaces.C.unsigned;
      Enable_Replanning : Interfaces.C.unsigned_char;
   end record;
   pragma Convention (C, Pyramid_Planning_Policy_C);

   type Pyramid_Planning_Goal_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Name : Pyramid_Str_T;
      Has_Requirement : Interfaces.C.unsigned_char := 0;
      Requirement : Pyramid_Requirement_Reference_C;
      Has_Expression : Interfaces.C.unsigned_char := 0;
      Expression : Pyramid_Str_T;
   end record;
   pragma Convention (C, Pyramid_Planning_Goal_C);

   type Pyramid_Execution_Policy_C is record
      Max_Replans : Interfaces.C.unsigned;
      Enable_Replanning : Interfaces.C.unsigned_char;
      Max_Concurrent_Placements : Interfaces.C.unsigned;
   end record;
   pragma Convention (C, Pyramid_Execution_Policy_C);

   type Pyramid_Planning_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Upstream_Requirement : Pyramid_Slice_T;
      Goal : Pyramid_Slice_T;
      Policy : Pyramid_Planning_Policy_C;
      Available_Agents : Pyramid_Slice_T;
   end record;
   pragma Convention (C, Pyramid_Planning_Requirement_C);

   type Pyramid_Execution_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Upstream_Requirement : Pyramid_Slice_T;
      Plan_Id : Pyramid_Str_T;
      Policy : Pyramid_Execution_Policy_C;
      Available_Agents : Pyramid_Slice_T;
      Planning_Requirement_Id : Pyramid_Str_T;
   end record;
   pragma Convention (C, Pyramid_Execution_Requirement_C);

   type Pyramid_World_Fact_Update_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Entity_Source : Pyramid_Str_T;
      Key : Pyramid_Str_T;
      Value : Interfaces.C.unsigned_char;
      Source : Pyramid_Str_T;
      Authority : Interfaces.C.int;
   end record;
   pragma Convention (C, Pyramid_World_Fact_Update_C);

   type Pyramid_State_Update_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Fact_Update : Pyramid_Slice_T;
   end record;
   pragma Convention (C, Pyramid_State_Update_C);

   type Pyramid_Capabilities_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Backend_Id : Pyramid_Str_T;
      Supports_Planning_Requirements : Interfaces.C.unsigned_char;
      Supports_Execution_Requirements : Interfaces.C.unsigned_char;
      Supports_Approved_Plan_Execution : Interfaces.C.unsigned_char;
      Supports_Replanning : Interfaces.C.unsigned_char;
      Supports_Typed_Component_Requirement_Placement : Interfaces.C.unsigned_char;
      Supports_State_Update_Ingress : Interfaces.C.unsigned_char;
   end record;
   pragma Convention (C, Pyramid_Capabilities_C);

   type Pyramid_Planned_Component_Interaction_C is record
      Target_Component : Pyramid_Str_T;
      Target_Service : Pyramid_Str_T;
      Target_Type : Pyramid_Str_T;
      Operation : Interfaces.C.int;
   end record;
   pragma Convention (C, Pyramid_Planned_Component_Interaction_C);

   type Pyramid_Plan_Step_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Sequence_Number : Interfaces.C.unsigned;
      Action_Name : Pyramid_Str_T;
      Signature : Pyramid_Str_T;
      Interaction : Pyramid_Slice_T;
   end record;
   pragma Convention (C, Pyramid_Plan_Step_C);

   type Pyramid_Plan_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Planning_Requirement_Id : Pyramid_Str_T;
      Backend_Id : Pyramid_Str_T;
      World_Version : Interfaces.C.unsigned_long;
      Replan_Count : Interfaces.C.unsigned;
      Plan_Success : Interfaces.C.unsigned_char;
      Solve_Time_Ms : Interfaces.C.double;
      Step : Pyramid_Slice_T;
      Compiled_Bt_Xml : Pyramid_Str_T;
      Has_Predicted_Quality : Interfaces.C.unsigned_char := 0;
      Predicted_Quality : Interfaces.C.double;
   end record;
   pragma Convention (C, Pyramid_Plan_C);

   type Pyramid_Requirement_Placement_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Execution_Requirement_Id : Pyramid_Str_T;
      Planning_Requirement_Id : Pyramid_Str_T;
      Plan_Id : Pyramid_Str_T;
      Plan_Step_Id : Pyramid_Str_T;
      Target_Component : Pyramid_Str_T;
      Target_Service : Pyramid_Str_T;
      Target_Type : Pyramid_Str_T;
      Operation : Interfaces.C.int;
      Target_Requirement_Id : Pyramid_Str_T;
      Related_Entity_Id : Pyramid_Slice_T;
      Progress : Interfaces.C.int;
   end record;
   pragma Convention (C, Pyramid_Requirement_Placement_C);

   type Pyramid_Execution_Run_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Execution_Requirement_Id : Pyramid_Str_T;
      Planning_Requirement_Id : Pyramid_Str_T;
      Plan_Id : Pyramid_Str_T;
      State : Interfaces.C.int;
      Achievement : Pyramid_Achievement_C;
      Replan_Count : Interfaces.C.unsigned;
      Outstanding_Placement : Pyramid_Slice_T;
   end record;
   pragma Convention (C, Pyramid_Execution_Run_C);

   procedure To_C
     (In_Value  : Requirement_Reference;
      Out_Value : out Pyramid_Requirement_Reference_C);
   procedure From_C
     (In_Value  : Pyramid_Requirement_Reference_C;
      Out_Value : out Requirement_Reference);
   procedure Free_Requirement_Reference (Value : access Pyramid_Requirement_Reference_C);
   pragma Import (C, Free_Requirement_Reference, "pyramid_RequirementReference_c_free");

   procedure To_C
     (In_Value  : Agent_State;
      Out_Value : out Pyramid_Agent_State_C);
   procedure From_C
     (In_Value  : Pyramid_Agent_State_C;
      Out_Value : out Agent_State);
   procedure Free_Agent_State (Value : access Pyramid_Agent_State_C);
   pragma Import (C, Free_Agent_State, "pyramid_AgentState_c_free");

   procedure To_C
     (In_Value  : Planning_Policy;
      Out_Value : out Pyramid_Planning_Policy_C);
   procedure From_C
     (In_Value  : Pyramid_Planning_Policy_C;
      Out_Value : out Planning_Policy);
   procedure Free_Planning_Policy (Value : access Pyramid_Planning_Policy_C);
   pragma Import (C, Free_Planning_Policy, "pyramid_PlanningPolicy_c_free");

   procedure To_C
     (In_Value  : Planning_Goal;
      Out_Value : out Pyramid_Planning_Goal_C);
   procedure From_C
     (In_Value  : Pyramid_Planning_Goal_C;
      Out_Value : out Planning_Goal);
   procedure Free_Planning_Goal (Value : access Pyramid_Planning_Goal_C);
   pragma Import (C, Free_Planning_Goal, "pyramid_PlanningGoal_c_free");

   procedure To_C
     (In_Value  : Execution_Policy;
      Out_Value : out Pyramid_Execution_Policy_C);
   procedure From_C
     (In_Value  : Pyramid_Execution_Policy_C;
      Out_Value : out Execution_Policy);
   procedure Free_Execution_Policy (Value : access Pyramid_Execution_Policy_C);
   pragma Import (C, Free_Execution_Policy, "pyramid_ExecutionPolicy_c_free");

   procedure To_C
     (In_Value  : Planning_Requirement;
      Out_Value : out Pyramid_Planning_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Planning_Requirement_C;
      Out_Value : out Planning_Requirement);
   procedure Free_Planning_Requirement (Value : access Pyramid_Planning_Requirement_C);
   pragma Import (C, Free_Planning_Requirement, "pyramid_PlanningRequirement_c_free");

   procedure To_C
     (In_Value  : Execution_Requirement;
      Out_Value : out Pyramid_Execution_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Execution_Requirement_C;
      Out_Value : out Execution_Requirement);
   procedure Free_Execution_Requirement (Value : access Pyramid_Execution_Requirement_C);
   pragma Import (C, Free_Execution_Requirement, "pyramid_ExecutionRequirement_c_free");

   procedure To_C
     (In_Value  : World_Fact_Update;
      Out_Value : out Pyramid_World_Fact_Update_C);
   procedure From_C
     (In_Value  : Pyramid_World_Fact_Update_C;
      Out_Value : out World_Fact_Update);
   procedure Free_World_Fact_Update (Value : access Pyramid_World_Fact_Update_C);
   pragma Import (C, Free_World_Fact_Update, "pyramid_WorldFactUpdate_c_free");

   procedure To_C
     (In_Value  : State_Update;
      Out_Value : out Pyramid_State_Update_C);
   procedure From_C
     (In_Value  : Pyramid_State_Update_C;
      Out_Value : out State_Update);
   procedure Free_State_Update (Value : access Pyramid_State_Update_C);
   pragma Import (C, Free_State_Update, "pyramid_StateUpdate_c_free");

   procedure To_C
     (In_Value  : Capabilities;
      Out_Value : out Pyramid_Capabilities_C);
   procedure From_C
     (In_Value  : Pyramid_Capabilities_C;
      Out_Value : out Capabilities);
   procedure Free_Capabilities (Value : access Pyramid_Capabilities_C);
   pragma Import (C, Free_Capabilities, "pyramid_Capabilities_c_free");

   procedure To_C
     (In_Value  : Planned_Component_Interaction;
      Out_Value : out Pyramid_Planned_Component_Interaction_C);
   procedure From_C
     (In_Value  : Pyramid_Planned_Component_Interaction_C;
      Out_Value : out Planned_Component_Interaction);
   procedure Free_Planned_Component_Interaction (Value : access Pyramid_Planned_Component_Interaction_C);
   pragma Import (C, Free_Planned_Component_Interaction, "pyramid_PlannedComponentInteraction_c_free");

   procedure To_C
     (In_Value  : Plan_Step;
      Out_Value : out Pyramid_Plan_Step_C);
   procedure From_C
     (In_Value  : Pyramid_Plan_Step_C;
      Out_Value : out Plan_Step);
   procedure Free_Plan_Step (Value : access Pyramid_Plan_Step_C);
   pragma Import (C, Free_Plan_Step, "pyramid_PlanStep_c_free");

   procedure To_C
     (In_Value  : Plan;
      Out_Value : out Pyramid_Plan_C);
   procedure From_C
     (In_Value  : Pyramid_Plan_C;
      Out_Value : out Plan);
   procedure Free_Plan (Value : access Pyramid_Plan_C);
   pragma Import (C, Free_Plan, "pyramid_Plan_c_free");

   procedure To_C
     (In_Value  : Requirement_Placement;
      Out_Value : out Pyramid_Requirement_Placement_C);
   procedure From_C
     (In_Value  : Pyramid_Requirement_Placement_C;
      Out_Value : out Requirement_Placement);
   procedure Free_Requirement_Placement (Value : access Pyramid_Requirement_Placement_C);
   pragma Import (C, Free_Requirement_Placement, "pyramid_RequirementPlacement_c_free");

   procedure To_C
     (In_Value  : Execution_Run;
      Out_Value : out Pyramid_Execution_Run_C);
   procedure From_C
     (In_Value  : Pyramid_Execution_Run_C;
      Out_Value : out Execution_Run);
   procedure Free_Execution_Run (Value : access Pyramid_Execution_Run_C);
   pragma Import (C, Free_Execution_Run, "pyramid_ExecutionRun_c_free");

end Pyramid.Data_Model.Autonomy.Cabi;
