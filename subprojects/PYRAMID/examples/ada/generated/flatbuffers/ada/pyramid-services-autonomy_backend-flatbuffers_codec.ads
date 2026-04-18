--  Auto-generated service FlatBuffers codec
--  Backend: flatbuffers
--  Generated from proto service closure for pyramid.components.autonomy_backend.services

with Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;

package Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec is
   Content_Type : constant String := "application/flatbuffers";

   function To_Binary_Fact_Update (Json : String) return String;
   function To_Binary_Fact_Update (Msg : Pyramid.Data_Model.Autonomy.Types.Fact_Update) return String;
   function From_Binary_Fact_Update (Payload : String) return String;

   function From_Binary_Fact_Update
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Fact_Update) return Pyramid.Data_Model.Autonomy.Types.Fact_Update;

   function To_Binary_State_Update (Json : String) return String;
   function To_Binary_State_Update (Msg : Pyramid.Data_Model.Autonomy.Types.State_Update) return String;
   function From_Binary_State_Update (Payload : String) return String;

   function From_Binary_State_Update
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.State_Update) return Pyramid.Data_Model.Autonomy.Types.State_Update;

   function To_Binary_Mission_Intent (Json : String) return String;
   function To_Binary_Mission_Intent (Msg : Pyramid.Data_Model.Autonomy.Types.Mission_Intent) return String;
   function From_Binary_Mission_Intent (Payload : String) return String;

   function From_Binary_Mission_Intent
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Mission_Intent) return Pyramid.Data_Model.Autonomy.Types.Mission_Intent;

   function To_Binary_Agent_State (Json : String) return String;
   function To_Binary_Agent_State (Msg : Pyramid.Data_Model.Autonomy.Types.Agent_State) return String;
   function From_Binary_Agent_State (Payload : String) return String;

   function From_Binary_Agent_State
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Agent_State) return Pyramid.Data_Model.Autonomy.Types.Agent_State;

   function To_Binary_Policy_Envelope (Json : String) return String;
   function To_Binary_Policy_Envelope (Msg : Pyramid.Data_Model.Autonomy.Types.Policy_Envelope) return String;
   function From_Binary_Policy_Envelope (Payload : String) return String;

   function From_Binary_Policy_Envelope
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Policy_Envelope) return Pyramid.Data_Model.Autonomy.Types.Policy_Envelope;

   function To_Binary_Session (Json : String) return String;
   function To_Binary_Session (Msg : Pyramid.Data_Model.Autonomy.Types.Session) return String;
   function From_Binary_Session (Payload : String) return String;

   function From_Binary_Session
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session) return Pyramid.Data_Model.Autonomy.Types.Session;

   function To_Binary_Capabilities (Json : String) return String;
   function To_Binary_Capabilities (Msg : Pyramid.Data_Model.Autonomy.Types.Capabilities) return String;
   function From_Binary_Capabilities (Payload : String) return String;

   function From_Binary_Capabilities
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Capabilities) return Pyramid.Data_Model.Autonomy.Types.Capabilities;

   function To_Binary_String_Key_Value (Json : String) return String;
   function To_Binary_String_Key_Value (Msg : Pyramid.Data_Model.Autonomy.Types.String_Key_Value) return String;
   function From_Binary_String_Key_Value (Payload : String) return String;

   function From_Binary_String_Key_Value
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.String_Key_Value) return Pyramid.Data_Model.Autonomy.Types.String_Key_Value;

   function To_Binary_Command (Json : String) return String;
   function To_Binary_Command (Msg : Pyramid.Data_Model.Autonomy.Types.Command) return String;
   function From_Binary_Command (Payload : String) return String;

   function From_Binary_Command
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Command) return Pyramid.Data_Model.Autonomy.Types.Command;

   function To_Binary_Goal_Dispatch (Json : String) return String;
   function To_Binary_Goal_Dispatch (Msg : Pyramid.Data_Model.Autonomy.Types.Goal_Dispatch) return String;
   function From_Binary_Goal_Dispatch (Payload : String) return String;

   function From_Binary_Goal_Dispatch
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Goal_Dispatch) return Pyramid.Data_Model.Autonomy.Types.Goal_Dispatch;

   function To_Binary_Decision_Record (Json : String) return String;
   function To_Binary_Decision_Record (Msg : Pyramid.Data_Model.Autonomy.Types.Decision_Record) return String;
   function From_Binary_Decision_Record (Payload : String) return String;

   function From_Binary_Decision_Record
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Decision_Record) return Pyramid.Data_Model.Autonomy.Types.Decision_Record;

   function To_Binary_Command_Result (Json : String) return String;
   function To_Binary_Command_Result (Msg : Pyramid.Data_Model.Autonomy.Types.Command_Result) return String;
   function From_Binary_Command_Result (Payload : String) return String;

   function From_Binary_Command_Result
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Command_Result) return Pyramid.Data_Model.Autonomy.Types.Command_Result;

   function To_Binary_Dispatch_Result (Json : String) return String;
   function To_Binary_Dispatch_Result (Msg : Pyramid.Data_Model.Autonomy.Types.Dispatch_Result) return String;
   function From_Binary_Dispatch_Result (Payload : String) return String;

   function From_Binary_Dispatch_Result
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Dispatch_Result) return Pyramid.Data_Model.Autonomy.Types.Dispatch_Result;

   function To_Binary_Session_Snapshot (Json : String) return String;
   function To_Binary_Session_Snapshot (Msg : Pyramid.Data_Model.Autonomy.Types.Session_Snapshot) return String;
   function From_Binary_Session_Snapshot (Payload : String) return String;

   function From_Binary_Session_Snapshot
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session_Snapshot) return Pyramid.Data_Model.Autonomy.Types.Session_Snapshot;

   function To_Binary_Session_Step_Request (Json : String) return String;
   function To_Binary_Session_Step_Request (Msg : Pyramid.Data_Model.Autonomy.Types.Session_Step_Request) return String;
   function From_Binary_Session_Step_Request (Payload : String) return String;

   function From_Binary_Session_Step_Request
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session_Step_Request) return Pyramid.Data_Model.Autonomy.Types.Session_Step_Request;

   function To_Binary_Session_Stop_Request (Json : String) return String;
   function To_Binary_Session_Stop_Request (Msg : Pyramid.Data_Model.Autonomy.Types.Session_Stop_Request) return String;
   function From_Binary_Session_Stop_Request (Payload : String) return String;

   function From_Binary_Session_Stop_Request
     (Payload : String; Tag : access Pyramid.Data_Model.Autonomy.Types.Session_Stop_Request) return Pyramid.Data_Model.Autonomy.Types.Session_Stop_Request;

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

   function To_Binary_Identifier (Json : String) return String;
   function To_Binary_Identifier (Msg : Pyramid.Data_Model.Base.Types.Identifier) return String;
   function From_Binary_Identifier (Payload : String) return String;

   function From_Binary_Identifier
     (Payload : String; Tag : access Pyramid.Data_Model.Base.Types.Identifier) return Pyramid.Data_Model.Base.Types.Identifier;

   function To_Binary_Capabilities_Array (Json : String) return String;
   function From_Binary_Capabilities_Array (Payload : String) return String;

   function To_Binary_Session_Snapshot_Array (Json : String) return String;
   function From_Binary_Session_Snapshot_Array (Payload : String) return String;

   function To_Binary_Command_Array (Json : String) return String;
   function From_Binary_Command_Array (Payload : String) return String;

   function To_Binary_Goal_Dispatch_Array (Json : String) return String;
   function From_Binary_Goal_Dispatch_Array (Payload : String) return String;

   function To_Binary_Decision_Record_Array (Json : String) return String;
   function From_Binary_Decision_Record_Array (Payload : String) return String;

end Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec;
