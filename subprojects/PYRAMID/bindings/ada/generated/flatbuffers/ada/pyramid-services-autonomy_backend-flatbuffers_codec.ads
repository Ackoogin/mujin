--  Auto-generated service FlatBuffers codec
--  Backend: flatbuffers
--  Generated from proto service closure for pyramid.components.autonomy_backend.services

with Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;

package Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec is
   Content_Type : constant String := "application/flatbuffers";

   function To_Binary_Requirement_Reference (Json : String) return String;
   function To_Binary_Requirement_Reference (Msg : Pyramid.Data_Model.Autonomy.Types.Requirement_Reference) return String;
   function From_Binary_Requirement_Reference (Payload : String) return String;

   function From_Binary_Requirement_Reference
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Requirement_Reference) return Pyramid.Data_Model.Autonomy.Types.Requirement_Reference;

   function To_Binary_Agent_State (Json : String) return String;
   function To_Binary_Agent_State (Msg : Pyramid.Data_Model.Autonomy.Types.Agent_State) return String;
   function From_Binary_Agent_State (Payload : String) return String;

   function From_Binary_Agent_State
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Agent_State) return Pyramid.Data_Model.Autonomy.Types.Agent_State;

   function To_Binary_Planning_Policy (Json : String) return String;
   function To_Binary_Planning_Policy (Msg : Pyramid.Data_Model.Autonomy.Types.Planning_Policy) return String;
   function From_Binary_Planning_Policy (Payload : String) return String;

   function From_Binary_Planning_Policy
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planning_Policy) return Pyramid.Data_Model.Autonomy.Types.Planning_Policy;

   function To_Binary_Planning_Goal (Json : String) return String;
   function To_Binary_Planning_Goal (Msg : Pyramid.Data_Model.Autonomy.Types.Planning_Goal) return String;
   function From_Binary_Planning_Goal (Payload : String) return String;

   function From_Binary_Planning_Goal
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planning_Goal) return Pyramid.Data_Model.Autonomy.Types.Planning_Goal;

   function To_Binary_Execution_Policy (Json : String) return String;
   function To_Binary_Execution_Policy (Msg : Pyramid.Data_Model.Autonomy.Types.Execution_Policy) return String;
   function From_Binary_Execution_Policy (Payload : String) return String;

   function From_Binary_Execution_Policy
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Execution_Policy) return Pyramid.Data_Model.Autonomy.Types.Execution_Policy;

   function To_Binary_World_Fact_Update (Json : String) return String;
   function To_Binary_World_Fact_Update (Msg : Pyramid.Data_Model.Autonomy.Types.World_Fact_Update) return String;
   function From_Binary_World_Fact_Update (Payload : String) return String;

   function From_Binary_World_Fact_Update
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.World_Fact_Update) return Pyramid.Data_Model.Autonomy.Types.World_Fact_Update;

   function To_Binary_State_Update (Json : String) return String;
   function To_Binary_State_Update (Msg : Pyramid.Data_Model.Autonomy.Types.State_Update) return String;
   function From_Binary_State_Update (Payload : String) return String;

   function From_Binary_State_Update
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.State_Update) return Pyramid.Data_Model.Autonomy.Types.State_Update;

   function To_Binary_Capabilities (Json : String) return String;
   function To_Binary_Capabilities (Msg : Pyramid.Data_Model.Autonomy.Types.Capabilities) return String;
   function From_Binary_Capabilities (Payload : String) return String;

   function From_Binary_Capabilities
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Capabilities) return Pyramid.Data_Model.Autonomy.Types.Capabilities;

   function To_Binary_Planned_Component_Interaction (Json : String) return String;
   function To_Binary_Planned_Component_Interaction (Msg : Pyramid.Data_Model.Autonomy.Types.Planned_Component_Interaction) return String;
   function From_Binary_Planned_Component_Interaction (Payload : String) return String;

   function From_Binary_Planned_Component_Interaction
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planned_Component_Interaction) return Pyramid.Data_Model.Autonomy.Types.Planned_Component_Interaction;

   function To_Binary_Plan_Step (Json : String) return String;
   function To_Binary_Plan_Step (Msg : Pyramid.Data_Model.Autonomy.Types.Plan_Step) return String;
   function From_Binary_Plan_Step (Payload : String) return String;

   function From_Binary_Plan_Step
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Plan_Step) return Pyramid.Data_Model.Autonomy.Types.Plan_Step;

   function To_Binary_Plan (Json : String) return String;
   function To_Binary_Plan (Msg : Pyramid.Data_Model.Autonomy.Types.Plan) return String;
   function From_Binary_Plan (Payload : String) return String;

   function From_Binary_Plan
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Plan) return Pyramid.Data_Model.Autonomy.Types.Plan;

   function To_Binary_Requirement_Placement (Json : String) return String;
   function To_Binary_Requirement_Placement (Msg : Pyramid.Data_Model.Autonomy.Types.Requirement_Placement) return String;
   function From_Binary_Requirement_Placement (Payload : String) return String;

   function From_Binary_Requirement_Placement
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Requirement_Placement) return Pyramid.Data_Model.Autonomy.Types.Requirement_Placement;

   function To_Binary_Achievement (Json : String) return String;
   function To_Binary_Achievement (Msg : Pyramid.Data_Model.Common.Types.Achievement) return String;
   function From_Binary_Achievement (Payload : String) return String;

   function From_Binary_Achievement
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Achievement) return Pyramid.Data_Model.Common.Types.Achievement;

   function To_Binary_Entity (Json : String) return String;
   function To_Binary_Entity (Msg : Pyramid.Data_Model.Common.Types.Entity) return String;
   function From_Binary_Entity (Payload : String) return String;

   function From_Binary_Entity
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Entity) return Pyramid.Data_Model.Common.Types.Entity;

   function To_Binary_Ack (Json : String) return String;
   function To_Binary_Ack (Msg : Pyramid.Data_Model.Common.Types.Ack) return String;
   function From_Binary_Ack (Payload : String) return String;

   function From_Binary_Ack
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Ack) return Pyramid.Data_Model.Common.Types.Ack;

   function To_Binary_Query (Json : String) return String;
   function To_Binary_Query (Msg : Pyramid.Data_Model.Common.Types.Query) return String;
   function From_Binary_Query (Payload : String) return String;

   function From_Binary_Query
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Query) return Pyramid.Data_Model.Common.Types.Query;

   function To_Binary_Planning_Requirement (Json : String) return String;
   function To_Binary_Planning_Requirement (Msg : Pyramid.Data_Model.Autonomy.Types.Planning_Requirement) return String;
   function From_Binary_Planning_Requirement (Payload : String) return String;

   function From_Binary_Planning_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Planning_Requirement) return Pyramid.Data_Model.Autonomy.Types.Planning_Requirement;

   function To_Binary_Execution_Requirement (Json : String) return String;
   function To_Binary_Execution_Requirement (Msg : Pyramid.Data_Model.Autonomy.Types.Execution_Requirement) return String;
   function From_Binary_Execution_Requirement (Payload : String) return String;

   function From_Binary_Execution_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Execution_Requirement) return Pyramid.Data_Model.Autonomy.Types.Execution_Requirement;

   function To_Binary_Execution_Run (Json : String) return String;
   function To_Binary_Execution_Run (Msg : Pyramid.Data_Model.Autonomy.Types.Execution_Run) return String;
   function From_Binary_Execution_Run (Payload : String) return String;

   function From_Binary_Execution_Run
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Execution_Run) return Pyramid.Data_Model.Autonomy.Types.Execution_Run;

   function To_Binary_Identifier (Json : String) return String;
   function To_Binary_Identifier (Msg : Pyramid.Data_Model.Base.Types.Identifier) return String;
   function From_Binary_Identifier (Payload : String) return String;

   function From_Binary_Identifier
     (Payload : String; Tag : access Pyramid.Data_Model.Base.Types.Identifier) return Pyramid.Data_Model.Base.Types.Identifier;

   function To_Binary_Capabilities_Array (Json : String) return String;
   function From_Binary_Capabilities_Array (Payload : String) return String;

   function To_Binary_Planning_Requirement_Array (Json : String) return String;
   function From_Binary_Planning_Requirement_Array (Payload : String) return String;

   function To_Binary_Execution_Requirement_Array (Json : String) return String;
   function From_Binary_Execution_Requirement_Array (Payload : String) return String;

   function To_Binary_Plan_Array (Json : String) return String;
   function From_Binary_Plan_Array (Payload : String) return String;

   function To_Binary_Execution_Run_Array (Json : String) return String;
   function From_Binary_Execution_Run_Array (Payload : String) return String;

   function To_Binary_Requirement_Placement_Array (Json : String) return String;
   function From_Binary_Requirement_Placement_Array (Payload : String) return String;

end Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec;
