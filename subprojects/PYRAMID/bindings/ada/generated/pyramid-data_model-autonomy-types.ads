--  Auto-generated types specification
--  Generated from: autonomy.proto by generate_bindings.py (types)
--  Package: Pyramid.Data_Model.Autonomy.Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;

package Pyramid.Data_Model.Autonomy.Types is


   type Fact_Authority_Level is
     (Level_Unspecified,
      Level_Believed,
      Level_Confirmed);

   type Planning_Execution_Mode is
     (Mode_Unspecified,
      Mode_PlanAndExecute,
      Mode_PlanOnly,
      Mode_ExecuteApprovedPlan);

   type Planning_Execution_State is
     (State_Unspecified,
      State_Accepted,
      State_Planning,
      State_Executing,
      State_WaitingForComponents,
      State_Achieved,
      State_Failed,
      State_Cancelled);

   type Requirement_Placement_Operation is
     (Operation_Unspecified,
      Operation_CreateRequirement,
      Operation_ReadRequirement,
      Operation_UpdateRequirement,
      Operation_DeleteRequirement,
      Operation_ReadProduct,
      Operation_ReadCapability);

   type Related_Entity_Id_Array is array (Positive range <>) of Unbounded_String;
   type Related_Entity_Id_Array_Acc is access all Related_Entity_Id_Array;

   subtype Base_Achievement is Pyramid.Data_Model.Common.Types.Achievement;
   subtype Base_Progress is Pyramid.Data_Model.Common.Types.Progress;

   type Requirement_Reference is record
      Requirement_Id : Unbounded_String := Null_Unbounded_String;
      Component_Name : Unbounded_String := Null_Unbounded_String;
      Service_Name : Unbounded_String := Null_Unbounded_String;
      Type_Name : Unbounded_String := Null_Unbounded_String;
   end record;

   type Upstream_Requirement_Array is array (Positive range <>) of Requirement_Reference;
   type Upstream_Requirement_Array_Acc is access all Upstream_Requirement_Array;

   type Agent_State is record
      Agent_Id : Unbounded_String := Null_Unbounded_String;
      Agent_Type : Unbounded_String := Null_Unbounded_String;
      Available : Boolean := False;
   end record;

   type Available_Agents_Array is array (Positive range <>) of Agent_State;
   type Available_Agents_Array_Acc is access all Available_Agents_Array;

   type Planning_Policy is record
      Max_Replans : Natural := 0;
      Enable_Replanning : Boolean := False;
      Max_Concurrent_Placements : Natural := 0;
   end record;

   type Planning_Goal is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Name : Unbounded_String := Null_Unbounded_String;
      --  oneof goal
      Has_Requirement : Boolean := False;
      Requirement : Requirement_Reference;
      Has_Expression : Boolean := False;
      Expression : Unbounded_String := Null_Unbounded_String;
   end record;

   type Goal_Array is array (Positive range <>) of Planning_Goal;
   type Goal_Array_Acc is access all Goal_Array;

   type Planning_Execution_Requirement is record
      Base : Entity;
      Status : Achievement;
      Upstream_Requirement : Upstream_Requirement_Array_Acc := null;
      Goal : Goal_Array_Acc := null;
      Policy : Planning_Policy;
      Available_Agents : Available_Agents_Array_Acc := null;
      Mode : Planning_Execution_Mode := Mode_Unspecified;
      Approved_Plan_Id : Unbounded_String := Null_Unbounded_String;
   end record;

   type World_Fact_Update is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Entity_Source : Unbounded_String := Null_Unbounded_String;
      Key : Unbounded_String := Null_Unbounded_String;
      Value : Boolean := False;
      Source : Unbounded_String := Null_Unbounded_String;
      Authority : Fact_Authority_Level := Level_Unspecified;
   end record;

   type Fact_Update_Array is array (Positive range <>) of World_Fact_Update;
   type Fact_Update_Array_Acc is access all Fact_Update_Array;

   type State_Update is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Fact_Update : Fact_Update_Array_Acc := null;
   end record;

   type Capabilities is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Backend_Id : Unbounded_String := Null_Unbounded_String;
      Supports_Plan_Only : Boolean := False;
      Supports_Plan_And_Execute : Boolean := False;
      Supports_Execute_Approved_Plan : Boolean := False;
      Supports_Replanning : Boolean := False;
      Supports_Typed_Component_Requirement_Placement : Boolean := False;
      Supports_State_Update_Ingress : Boolean := False;
   end record;

   type Planned_Component_Interaction is record
      Target_Component : Unbounded_String := Null_Unbounded_String;
      Target_Service : Unbounded_String := Null_Unbounded_String;
      Target_Type : Unbounded_String := Null_Unbounded_String;
      Operation : Requirement_Placement_Operation := Operation_Unspecified;
   end record;

   type Interaction_Array is array (Positive range <>) of Planned_Component_Interaction;
   type Interaction_Array_Acc is access all Interaction_Array;

   type Plan_Step is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Sequence_Number : Natural := 0;
      Action_Name : Unbounded_String := Null_Unbounded_String;
      Signature : Unbounded_String := Null_Unbounded_String;
      Interaction : Interaction_Array_Acc := null;
   end record;

   type Step_Array is array (Positive range <>) of Plan_Step;
   type Step_Array_Acc is access all Step_Array;

   type Plan is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Planning_Execution_Requirement_Id : Unbounded_String := Null_Unbounded_String;
      Backend_Id : Unbounded_String := Null_Unbounded_String;
      World_Version : Long_Integer := 0;
      Replan_Count : Natural := 0;
      Plan_Success : Boolean := False;
      Solve_Time_Ms : Long_Float := 0.0;
      Step : Step_Array_Acc := null;
      Compiled_Bt_Xml : Unbounded_String := Null_Unbounded_String;
      Predicted_Quality : Long_Float := 0.0;
   end record;

   type Requirement_Placement is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Planning_Execution_Requirement_Id : Unbounded_String := Null_Unbounded_String;
      Plan_Id : Unbounded_String := Null_Unbounded_String;
      Plan_Step_Id : Unbounded_String := Null_Unbounded_String;
      Target_Component : Unbounded_String := Null_Unbounded_String;
      Target_Service : Unbounded_String := Null_Unbounded_String;
      Target_Type : Unbounded_String := Null_Unbounded_String;
      Operation : Requirement_Placement_Operation := Operation_Unspecified;
      Target_Requirement_Id : Unbounded_String := Null_Unbounded_String;
      Related_Entity_Id : Related_Entity_Id_Array_Acc := null;
      Progress : Base_Progress := Progress_Unspecified;
   end record;

   type Outstanding_Placement_Array is array (Positive range <>) of Requirement_Placement;
   type Outstanding_Placement_Array_Acc is access all Outstanding_Placement_Array;

   type Execution_Run is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Planning_Execution_Requirement_Id : Unbounded_String := Null_Unbounded_String;
      Plan_Id : Unbounded_String := Null_Unbounded_String;
      State : Planning_Execution_State := State_Unspecified;
      Achievement : Base_Achievement;
      Replan_Count : Natural := 0;
      Outstanding_Placement : Outstanding_Placement_Array_Acc := null;
   end record;

end Pyramid.Data_Model.Autonomy.Types;
