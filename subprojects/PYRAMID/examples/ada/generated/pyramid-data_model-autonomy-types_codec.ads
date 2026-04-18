--  Auto-generated data model JSON codec specification
--  Generated from: autonomy.proto by generate_bindings.py (codec)
--  Package: Pyramid.Data_Model.Autonomy.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;

package Pyramid.Data_Model.Autonomy.Types_Codec is

   function To_String (V : Autonomy_Backend_State) return String;
   function Autonomy_Backend_State_From_String (S : String) return Autonomy_Backend_State;
   function To_String (V : Command_Status) return String;
   function Command_Status_From_String (S : String) return Command_Status;
   function To_String (V : Stop_Mode) return String;
   function Stop_Mode_From_String (S : String) return Stop_Mode;
   function To_String (V : Fact_Authority_Level) return String;
   function Fact_Authority_Level_From_String (S : String) return Fact_Authority_Level;

   function To_Json (Msg : Fact_Update) return String;
   function From_Json (S : String; Tag : access Fact_Update) return Fact_Update;
   function To_Json (Msg : State_Update) return String;
   function From_Json (S : String; Tag : access State_Update) return State_Update;
   function To_Json (Msg : Mission_Intent) return String;
   function From_Json (S : String; Tag : access Mission_Intent) return Mission_Intent;
   function To_Json (Msg : Agent_State) return String;
   function From_Json (S : String; Tag : access Agent_State) return Agent_State;
   function To_Json (Msg : Policy_Envelope) return String;
   function From_Json (S : String; Tag : access Policy_Envelope) return Policy_Envelope;
   function To_Json (Msg : Session) return String;
   function From_Json (S : String; Tag : access Session) return Session;
   function To_Json (Msg : Capabilities) return String;
   function From_Json (S : String; Tag : access Capabilities) return Capabilities;
   function To_Json (Msg : String_Key_Value) return String;
   function From_Json (S : String; Tag : access String_Key_Value) return String_Key_Value;
   function To_Json (Msg : Command) return String;
   function From_Json (S : String; Tag : access Command) return Command;
   function To_Json (Msg : Goal_Dispatch) return String;
   function From_Json (S : String; Tag : access Goal_Dispatch) return Goal_Dispatch;
   function To_Json (Msg : Decision_Record) return String;
   function From_Json (S : String; Tag : access Decision_Record) return Decision_Record;
   function To_Json (Msg : Command_Result) return String;
   function From_Json (S : String; Tag : access Command_Result) return Command_Result;
   function To_Json (Msg : Dispatch_Result) return String;
   function From_Json (S : String; Tag : access Dispatch_Result) return Dispatch_Result;
   function To_Json (Msg : Session_Snapshot) return String;
   function From_Json (S : String; Tag : access Session_Snapshot) return Session_Snapshot;
   function To_Json (Msg : Session_Step_Request) return String;
   function From_Json (S : String; Tag : access Session_Step_Request) return Session_Step_Request;
   function To_Json (Msg : Session_Stop_Request) return String;
   function From_Json (S : String; Tag : access Session_Stop_Request) return Session_Stop_Request;

end Pyramid.Data_Model.Autonomy.Types_Codec;
