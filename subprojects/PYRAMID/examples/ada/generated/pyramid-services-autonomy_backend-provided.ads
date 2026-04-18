--  Auto-generated service binding specification
--  Generated from: provided.proto by generate_bindings.py
--  Package: Pyramid.Services.Autonomy_Backend.Provided
--
--  Architecture: component logic > service binding (this) > PCL
--
--  This package provides:
--    1. Wire-name constants and topic constants
--    2. EntityActions callback access types and handler set record
--    3. PCL binding procedures (Register_Services, Subscribe_*, Invoke_*, Publish_*)
--    4. Msg_To_String utility for PCL message payloads
--
--  RPC request/response payloads use the proto-native data model.
--  Standard topic payloads also use canonical proto-derived types.

with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;
with Pcl_Bindings;
with Interfaces.C;
with System;

package Pyramid.Services.Autonomy_Backend.Provided is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   type Service_Channel is
     (Ch_Read_Capabilities,
      Ch_Create_Session,
      Ch_Read_Session,
      Ch_Update_Session,
      Ch_Delete_Session,
      Ch_Create_State,
      Ch_Update_State,
      Ch_Delete_State,
      Ch_Create_Intent,
      Ch_Update_Intent,
      Ch_Delete_Intent,
      Ch_Read_Command,
      Ch_Read_Goal_Dispatch,
      Ch_Read_Decision_Record,
      Ch_Create_Command_Result,
      Ch_Update_Command_Result,
      Ch_Delete_Command_Result,
      Ch_Create_Dispatch_Result,
      Ch_Update_Dispatch_Result,
      Ch_Delete_Dispatch_Result);

   type Capabilities_Array is array (Positive range <>) of Capabilities;
   type Command_Array is array (Positive range <>) of Command;
   type Decision_Record_Array is array (Positive range <>) of Decision_Record;
   type Goal_Dispatch_Array is array (Positive range <>) of Goal_Dispatch;
   type Session_Snapshot_Array is array (Positive range <>) of Session_Snapshot;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Read_Capabilities : constant String :=
     "capabilities.read_capabilities";
   Svc_Create_Session : constant String :=
     "session.create_session";
   Svc_Read_Session : constant String :=
     "session.read_session";
   Svc_Update_Session : constant String :=
     "session.update_session";
   Svc_Delete_Session : constant String :=
     "session.delete_session";
   Svc_Create_State : constant String :=
     "state.create_state";
   Svc_Update_State : constant String :=
     "state.update_state";
   Svc_Delete_State : constant String :=
     "state.delete_state";
   Svc_Create_Intent : constant String :=
     "intent.create_intent";
   Svc_Update_Intent : constant String :=
     "intent.update_intent";
   Svc_Delete_Intent : constant String :=
     "intent.delete_intent";
   Svc_Read_Command : constant String :=
     "command.read_command";
   Svc_Read_Goal_Dispatch : constant String :=
     "goal_dispatch.read_goal_dispatch";
   Svc_Read_Decision_Record : constant String :=
     "decision_record.read_decision_record";
   Svc_Create_Command_Result : constant String :=
     "command_result.create_command_result";
   Svc_Update_Command_Result : constant String :=
     "command_result.update_command_result";
   Svc_Delete_Command_Result : constant String :=
     "command_result.delete_command_result";
   Svc_Create_Dispatch_Result : constant String :=
     "dispatch_result.create_dispatch_result";
   Svc_Update_Dispatch_Result : constant String :=
     "dispatch_result.update_dispatch_result";
   Svc_Delete_Dispatch_Result : constant String :=
     "dispatch_result.delete_dispatch_result";

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   --  -- EntityActions handler callbacks ----------------------------
   --  Supply these callbacks from your component at registration time.

   --  Capabilities_Service
   type Handle_Read_Capabilities_Access is access function
     (Request : Query) return Capabilities_Array;
   --  Session_Service
   type Handle_Create_Session_Access is access procedure
     (Request  : in  Session;
      Response : out Identifier);
   type Handle_Read_Session_Access is access function
     (Request : Query) return Session_Snapshot_Array;
   type Handle_Update_Session_Access is access procedure
     (Request  : in  Session_Step_Request;
      Response : out Ack);
   type Handle_Delete_Session_Access is access procedure
     (Request  : in  Session_Stop_Request;
      Response : out Ack);
   --  State_Service
   type Handle_Create_State_Access is access procedure
     (Request  : in  State_Update;
      Response : out Identifier);
   type Handle_Update_State_Access is access procedure
     (Request  : in  State_Update;
      Response : out Ack);
   type Handle_Delete_State_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Intent_Service
   type Handle_Create_Intent_Access is access procedure
     (Request  : in  Mission_Intent;
      Response : out Identifier);
   type Handle_Update_Intent_Access is access procedure
     (Request  : in  Mission_Intent;
      Response : out Ack);
   type Handle_Delete_Intent_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Command_Service
   type Handle_Read_Command_Access is access function
     (Request : Query) return Command_Array;
   --  GoalDispatch_Service
   type Handle_Read_Goal_Dispatch_Access is access function
     (Request : Query) return Goal_Dispatch_Array;
   --  DecisionRecord_Service
   type Handle_Read_Decision_Record_Access is access function
     (Request : Query) return Decision_Record_Array;
   --  CommandResult_Service
   type Handle_Create_Command_Result_Access is access procedure
     (Request  : in  Command_Result;
      Response : out Identifier);
   type Handle_Update_Command_Result_Access is access procedure
     (Request  : in  Command_Result;
      Response : out Ack);
   type Handle_Delete_Command_Result_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  DispatchResult_Service
   type Handle_Create_Dispatch_Result_Access is access procedure
     (Request  : in  Dispatch_Result;
      Response : out Identifier);
   type Handle_Update_Dispatch_Result_Access is access procedure
     (Request  : in  Dispatch_Result;
      Response : out Ack);
   type Handle_Delete_Dispatch_Result_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);

   type Service_Handlers is record
      On_Read_Capabilities : Handle_Read_Capabilities_Access := null;
      On_Create_Session : Handle_Create_Session_Access := null;
      On_Read_Session : Handle_Read_Session_Access := null;
      On_Update_Session : Handle_Update_Session_Access := null;
      On_Delete_Session : Handle_Delete_Session_Access := null;
      On_Create_State : Handle_Create_State_Access := null;
      On_Update_State : Handle_Update_State_Access := null;
      On_Delete_State : Handle_Delete_State_Access := null;
      On_Create_Intent : Handle_Create_Intent_Access := null;
      On_Update_Intent : Handle_Update_Intent_Access := null;
      On_Delete_Intent : Handle_Delete_Intent_Access := null;
      On_Read_Command : Handle_Read_Command_Access := null;
      On_Read_Goal_Dispatch : Handle_Read_Goal_Dispatch_Access := null;
      On_Read_Decision_Record : Handle_Read_Decision_Record_Access := null;
      On_Create_Command_Result : Handle_Create_Command_Result_Access := null;
      On_Update_Command_Result : Handle_Update_Command_Result_Access := null;
      On_Delete_Command_Result : Handle_Delete_Command_Result_Access := null;
      On_Create_Dispatch_Result : Handle_Create_Dispatch_Result_Access := null;
      On_Update_Dispatch_Result : Handle_Update_Dispatch_Result_Access := null;
      On_Delete_Dispatch_Result : Handle_Delete_Dispatch_Result_Access := null;
   end record;

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Capabilities
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Create_Session
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Session;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Session
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Update_Session
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Session_Step_Request;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Delete_Session
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Session_Stop_Request;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Create_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Update_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Delete_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Create_Intent
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Mission_Intent;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Update_Intent
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Mission_Intent;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Delete_Intent
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Command
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Goal_Dispatch
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Decision_Record
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Create_Command_Result
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Command_Result;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Update_Command_Result
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Command_Result;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Delete_Command_Result
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Create_Dispatch_Result
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Dispatch_Result;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Update_Dispatch_Result
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Dispatch_Result;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Delete_Dispatch_Result
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   --  -- Transport integration point ------------------------------

   procedure Dispatch
     (Handlers      : access constant Service_Handlers := null;
      Channel       : in  Service_Channel;
      Request_Buf   : in  System.Address;
      Request_Size  : in  Natural;
      Content_Type  : in  String := "application/json";
      Response_Buf  : out System.Address;
      Response_Size : out Natural);

end Pyramid.Services.Autonomy_Backend.Provided;
