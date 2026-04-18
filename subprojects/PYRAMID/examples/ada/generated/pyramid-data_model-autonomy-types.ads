--  Auto-generated types specification
--  Generated from: autonomy.proto by generate_bindings.py (types)
--  Package: Pyramid.Data_Model.Autonomy.Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;

package Pyramid.Data_Model.Autonomy.Types is


   type Autonomy_Backend_State is
     (State_Unspecified,
      State_Idle,
      State_Ready,
      State_WaitingForResults,
      State_Complete,
      State_Failed,
      State_Stopped);

   type Command_Status is
     (Status_Unspecified,
      Status_Pending,
      Status_Running,
      Status_Succeeded,
      Status_FailedTransient,
      Status_FailedPermanent,
      Status_Cancelled);

   type Stop_Mode is
     (Mode_Unspecified,
      Mode_Drain,
      Mode_Immediate);

   type Fact_Authority_Level is
     (Level_Unspecified,
      Level_Believed,
      Level_Confirmed);

   type Goal_Fluents_Array is array (Positive range <>) of Unbounded_String;
   type Goal_Fluents_Array_Acc is access all Goal_Fluents_Array;
   type Goals_Array is array (Positive range <>) of Unbounded_String;
   type Goals_Array_Acc is access all Goals_Array;
   type Planned_Action_Signatures_Array is array (Positive range <>) of Unbounded_String;
   type Planned_Action_Signatures_Array_Acc is access all Planned_Action_Signatures_Array;

   type Fact_Update is record
      Key : Unbounded_String := Null_Unbounded_String;
      Value : Boolean := False;
      Source : Unbounded_String := Null_Unbounded_String;
      Authority : Fact_Authority_Level := Level_Unspecified;
   end record;

   type Fact_Updates_Array is array (Positive range <>) of Fact_Update;
   type Fact_Updates_Array_Acc is access all Fact_Updates_Array;

   type Observed_Updates_Array is array (Positive range <>) of Fact_Update;
   type Observed_Updates_Array_Acc is access all Observed_Updates_Array;

   type State_Update is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Fact_Updates : Fact_Updates_Array_Acc := null;
   end record;

   type Mission_Intent is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Goal_Fluents : Goal_Fluents_Array_Acc := null;
   end record;

   type Agent_State is record
      Agent_Id : Unbounded_String := Null_Unbounded_String;
      Agent_Type : Unbounded_String := Null_Unbounded_String;
      Available : Boolean := False;
   end record;

   type Available_Agents_Array is array (Positive range <>) of Agent_State;
   type Available_Agents_Array_Acc is access all Available_Agents_Array;

   type Agent_States_Array is array (Positive range <>) of Agent_State;
   type Agent_States_Array_Acc is access all Agent_States_Array;

   type Policy_Envelope is record
      Max_Replans : Natural := 0;
      Enable_Goal_Dispatch : Boolean := False;
   end record;

   type Session is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Intent : Mission_Intent;
      Policy : Policy_Envelope;
      Available_Agents : Available_Agents_Array_Acc := null;
   end record;

   type Capabilities is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Backend_Id : Unbounded_String := Null_Unbounded_String;
      Supports_Batch_Planning : Boolean := False;
      Supports_External_Command_Dispatch : Boolean := False;
      Supports_Replanning : Boolean := False;
   end record;

   type String_Key_Value is record
      Key : Unbounded_String := Null_Unbounded_String;
      Value : Unbounded_String := Null_Unbounded_String;
   end record;

   type Request_Fields_Array is array (Positive range <>) of String_Key_Value;
   type Request_Fields_Array_Acc is access all Request_Fields_Array;

   type Command is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Command_Id : Unbounded_String := Null_Unbounded_String;
      Action_Name : Unbounded_String := Null_Unbounded_String;
      Signature : Unbounded_String := Null_Unbounded_String;
      Service_Name : Unbounded_String := Null_Unbounded_String;
      Operation : Unbounded_String := Null_Unbounded_String;
      Request_Fields : Request_Fields_Array_Acc := null;
   end record;

   type Outstanding_Commands_Array is array (Positive range <>) of Command;
   type Outstanding_Commands_Array_Acc is access all Outstanding_Commands_Array;

   type Goal_Dispatch is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Dispatch_Id : Unbounded_String := Null_Unbounded_String;
      Agent_Id : Unbounded_String := Null_Unbounded_String;
      Goals : Goals_Array_Acc := null;
   end record;

   type Outstanding_Goal_Dispatches_Array is array (Positive range <>) of Goal_Dispatch;
   type Outstanding_Goal_Dispatches_Array_Acc is access all Outstanding_Goal_Dispatches_Array;

   type Decision_Record is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Session_Id : Unbounded_String := Null_Unbounded_String;
      Backend_Id : Unbounded_String := Null_Unbounded_String;
      World_Version : Long_Integer := 0;
      Replan_Count : Natural := 0;
      Plan_Success : Boolean := False;
      Solve_Time_Ms : Long_Float := 0.0;
      Planned_Action_Signatures : Planned_Action_Signatures_Array_Acc := null;
      Compiled_Bt_Xml : Unbounded_String := Null_Unbounded_String;
   end record;

   type Decision_History_Array is array (Positive range <>) of Decision_Record;
   type Decision_History_Array_Acc is access all Decision_History_Array;

   type Command_Result is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Entity_Source : Unbounded_String := Null_Unbounded_String;
      Command_Id : Unbounded_String := Null_Unbounded_String;
      Status : Command_Status := Status_Unspecified;
      Observed_Updates : Observed_Updates_Array_Acc := null;
      Source : Unbounded_String := Null_Unbounded_String;
   end record;

   type Dispatch_Result is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Entity_Source : Unbounded_String := Null_Unbounded_String;
      Dispatch_Id : Unbounded_String := Null_Unbounded_String;
      Status : Command_Status := Status_Unspecified;
      Observed_Updates : Observed_Updates_Array_Acc := null;
      Source : Unbounded_String := Null_Unbounded_String;
   end record;

   type Session_Snapshot is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Session_Id : Unbounded_String := Null_Unbounded_String;
      State : Autonomy_Backend_State := State_Unspecified;
      World_Version : Long_Integer := 0;
      Replan_Count : Natural := 0;
      Agent_States : Agent_States_Array_Acc := null;
      Outstanding_Commands : Outstanding_Commands_Array_Acc := null;
      Outstanding_Goal_Dispatches : Outstanding_Goal_Dispatches_Array_Acc := null;
      Decision_History : Decision_History_Array_Acc := null;
   end record;

   type Session_Step_Request is record
      Session_Id : Unbounded_String := Null_Unbounded_String;
   end record;

   type Session_Stop_Request is record
      Session_Id : Unbounded_String := Null_Unbounded_String;
      Mode : Stop_Mode := Mode_Unspecified;
   end record;

end Pyramid.Data_Model.Autonomy.Types;
