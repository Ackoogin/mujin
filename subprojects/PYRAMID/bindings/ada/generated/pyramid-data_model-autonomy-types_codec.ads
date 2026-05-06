--  Auto-generated data model JSON codec specification
--  Generated from: pyramid.data_model.autonomy.proto by generate_bindings.py (codec)
--  Package: Pyramid.Data_Model.Autonomy.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;

package Pyramid.Data_Model.Autonomy.Types_Codec is

   function To_String (V : Fact_Authority_Level) return String;
   function Fact_Authority_Level_From_String (S : String) return Fact_Authority_Level;
   function To_String (V : Execution_State) return String;
   function Execution_State_From_String (S : String) return Execution_State;
   function To_String (V : Requirement_Placement_Operation) return String;
   function Requirement_Placement_Operation_From_String (S : String) return Requirement_Placement_Operation;

   function To_Json (Msg : Requirement_Reference) return String;
   function From_Json (S : String; Tag : access Requirement_Reference) return Requirement_Reference;
   function To_Json (Msg : Agent_State) return String;
   function From_Json (S : String; Tag : access Agent_State) return Agent_State;
   function To_Json (Msg : Planning_Policy) return String;
   function From_Json (S : String; Tag : access Planning_Policy) return Planning_Policy;
   function To_Json (Msg : Planning_Goal) return String;
   function From_Json (S : String; Tag : access Planning_Goal) return Planning_Goal;
   function To_Json (Msg : Execution_Policy) return String;
   function From_Json (S : String; Tag : access Execution_Policy) return Execution_Policy;
   function To_Json (Msg : Planning_Requirement) return String;
   function From_Json (S : String; Tag : access Planning_Requirement) return Planning_Requirement;
   function To_Json (Msg : Execution_Requirement) return String;
   function From_Json (S : String; Tag : access Execution_Requirement) return Execution_Requirement;
   function To_Json (Msg : World_Fact_Update) return String;
   function From_Json (S : String; Tag : access World_Fact_Update) return World_Fact_Update;
   function To_Json (Msg : State_Update) return String;
   function From_Json (S : String; Tag : access State_Update) return State_Update;
   function To_Json (Msg : Capabilities) return String;
   function From_Json (S : String; Tag : access Capabilities) return Capabilities;
   function To_Json (Msg : Planned_Component_Interaction) return String;
   function From_Json (S : String; Tag : access Planned_Component_Interaction) return Planned_Component_Interaction;
   function To_Json (Msg : Plan_Step) return String;
   function From_Json (S : String; Tag : access Plan_Step) return Plan_Step;
   function To_Json (Msg : Plan) return String;
   function From_Json (S : String; Tag : access Plan) return Plan;
   function To_Json (Msg : Requirement_Placement) return String;
   function From_Json (S : String; Tag : access Requirement_Placement) return Requirement_Placement;
   function To_Json (Msg : Execution_Run) return String;
   function From_Json (S : String; Tag : access Execution_Run) return Execution_Run;

end Pyramid.Data_Model.Autonomy.Types_Codec;
