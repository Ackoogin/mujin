--  Auto-generated service binding specification
--  Generated from: pyramid.components.autonomy_backend.services.provided.proto by generate_bindings.py
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
     (Ch_Capabilities_Read_Capabilities,
      Ch_Planning_Requirement_Create_Planning_Requirement,
      Ch_Planning_Requirement_Read_Planning_Requirement,
      Ch_Planning_Requirement_Update_Planning_Requirement,
      Ch_Planning_Requirement_Delete_Planning_Requirement,
      Ch_Execution_Requirement_Create_Execution_Requirement,
      Ch_Execution_Requirement_Read_Execution_Requirement,
      Ch_Execution_Requirement_Update_Execution_Requirement,
      Ch_Execution_Requirement_Delete_Execution_Requirement,
      Ch_State_Create_State,
      Ch_State_Update_State,
      Ch_State_Delete_State,
      Ch_Plan_Create_Plan,
      Ch_Plan_Read_Plan,
      Ch_Plan_Update_Plan,
      Ch_Plan_Delete_Plan,
      Ch_Execution_Run_Read_Run,
      Ch_Requirement_Placement_Read_Placement);

   type Capabilities_Array is array (Positive range <>) of Capabilities;
   type Execution_Requirement_Array is array (Positive range <>) of Execution_Requirement;
   type Execution_Run_Array is array (Positive range <>) of Execution_Run;
   type Plan_Array is array (Positive range <>) of Plan;
   type Planning_Requirement_Array is array (Positive range <>) of Planning_Requirement;
   type Requirement_Placement_Array is array (Positive range <>) of Requirement_Placement;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Capabilities_Read_Capabilities : constant String :=
     "capabilities.read_capabilities";
   Svc_Planning_Requirement_Create_Planning_Requirement : constant String :=
     "planning_requirement.create_planning_requirement";
   Svc_Planning_Requirement_Read_Planning_Requirement : constant String :=
     "planning_requirement.read_planning_requirement";
   Svc_Planning_Requirement_Update_Planning_Requirement : constant String :=
     "planning_requirement.update_planning_requirement";
   Svc_Planning_Requirement_Delete_Planning_Requirement : constant String :=
     "planning_requirement.delete_planning_requirement";
   Svc_Execution_Requirement_Create_Execution_Requirement : constant String :=
     "execution_requirement.create_execution_requirement";
   Svc_Execution_Requirement_Read_Execution_Requirement : constant String :=
     "execution_requirement.read_execution_requirement";
   Svc_Execution_Requirement_Update_Execution_Requirement : constant String :=
     "execution_requirement.update_execution_requirement";
   Svc_Execution_Requirement_Delete_Execution_Requirement : constant String :=
     "execution_requirement.delete_execution_requirement";
   Svc_State_Create_State : constant String :=
     "state.create_state";
   Svc_State_Update_State : constant String :=
     "state.update_state";
   Svc_State_Delete_State : constant String :=
     "state.delete_state";
   Svc_Plan_Create_Plan : constant String :=
     "plan.create_plan";
   Svc_Plan_Read_Plan : constant String :=
     "plan.read_plan";
   Svc_Plan_Update_Plan : constant String :=
     "plan.update_plan";
   Svc_Plan_Delete_Plan : constant String :=
     "plan.delete_plan";
   Svc_Execution_Run_Read_Run : constant String :=
     "execution_run.read_run";
   Svc_Requirement_Placement_Read_Placement : constant String :=
     "requirement_placement.read_placement";

   --  -- ROS2 endpoint constants --------------------------------

   Ros2_Transport_Content_Type : constant String := "application/ros2";

   Capabilities_Read_Capabilities_Open_Service : constant String :=
     "/pyramid/stream/capabilities/read_capabilities/open";
   Capabilities_Read_Capabilities_Frame_Topic : constant String :=
     "/pyramid/stream/capabilities/read_capabilities/frames";
   Capabilities_Read_Capabilities_Cancel_Topic : constant String :=
     "/pyramid/stream/capabilities/read_capabilities/cancel";

   Planning_Requirement_Create_Planning_Requirement_Service : constant String :=
     "/pyramid/service/planning_requirement/create_planning_requirement";

   Planning_Requirement_Read_Planning_Requirement_Open_Service : constant String :=
     "/pyramid/stream/planning_requirement/read_planning_requirement/open";
   Planning_Requirement_Read_Planning_Requirement_Frame_Topic : constant String :=
     "/pyramid/stream/planning_requirement/read_planning_requirement/frames";
   Planning_Requirement_Read_Planning_Requirement_Cancel_Topic : constant String :=
     "/pyramid/stream/planning_requirement/read_planning_requirement/cancel";

   Planning_Requirement_Update_Planning_Requirement_Service : constant String :=
     "/pyramid/service/planning_requirement/update_planning_requirement";

   Planning_Requirement_Delete_Planning_Requirement_Service : constant String :=
     "/pyramid/service/planning_requirement/delete_planning_requirement";

   Execution_Requirement_Create_Execution_Requirement_Service : constant String :=
     "/pyramid/service/execution_requirement/create_execution_requirement";

   Execution_Requirement_Read_Execution_Requirement_Open_Service : constant String :=
     "/pyramid/stream/execution_requirement/read_execution_requirement/open";
   Execution_Requirement_Read_Execution_Requirement_Frame_Topic : constant String :=
     "/pyramid/stream/execution_requirement/read_execution_requirement/frames";
   Execution_Requirement_Read_Execution_Requirement_Cancel_Topic : constant String :=
     "/pyramid/stream/execution_requirement/read_execution_requirement/cancel";

   Execution_Requirement_Update_Execution_Requirement_Service : constant String :=
     "/pyramid/service/execution_requirement/update_execution_requirement";

   Execution_Requirement_Delete_Execution_Requirement_Service : constant String :=
     "/pyramid/service/execution_requirement/delete_execution_requirement";

   State_Create_State_Service : constant String :=
     "/pyramid/service/state/create_state";

   State_Update_State_Service : constant String :=
     "/pyramid/service/state/update_state";

   State_Delete_State_Service : constant String :=
     "/pyramid/service/state/delete_state";

   Plan_Create_Plan_Service : constant String :=
     "/pyramid/service/plan/create_plan";

   Plan_Read_Plan_Open_Service : constant String :=
     "/pyramid/stream/plan/read_plan/open";
   Plan_Read_Plan_Frame_Topic : constant String :=
     "/pyramid/stream/plan/read_plan/frames";
   Plan_Read_Plan_Cancel_Topic : constant String :=
     "/pyramid/stream/plan/read_plan/cancel";

   Plan_Update_Plan_Service : constant String :=
     "/pyramid/service/plan/update_plan";

   Plan_Delete_Plan_Service : constant String :=
     "/pyramid/service/plan/delete_plan";

   Execution_Run_Read_Run_Open_Service : constant String :=
     "/pyramid/stream/execution_run/read_run/open";
   Execution_Run_Read_Run_Frame_Topic : constant String :=
     "/pyramid/stream/execution_run/read_run/frames";
   Execution_Run_Read_Run_Cancel_Topic : constant String :=
     "/pyramid/stream/execution_run/read_run/cancel";

   Requirement_Placement_Read_Placement_Open_Service : constant String :=
     "/pyramid/stream/requirement_placement/read_placement/open";
   Requirement_Placement_Read_Placement_Frame_Topic : constant String :=
     "/pyramid/stream/requirement_placement/read_placement/frames";
   Requirement_Placement_Read_Placement_Cancel_Topic : constant String :=
     "/pyramid/stream/requirement_placement/read_placement/cancel";

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   Json_Content_Type : constant String := "application/json";
   Flatbuffers_Content_Type : constant String := "application/flatbuffers";
   Grpc_Content_Type : constant String := "application/grpc";

   function Supports_Content_Type (Content_Type : String) return Boolean;

   procedure Configure_Grpc_Library (Path : String);
   procedure Configure_Grpc_Channel (Channel : String);

   --  -- EntityActions handler callbacks ----------------------------
   --  Supply these callbacks from your component at registration time.

   --  Capabilities_Service
   type Handle_Capabilities_Read_Capabilities_Access is access function
     (Request : Query) return Capabilities_Array;
   --  Planning_Requirement_Service
   type Handle_Planning_Requirement_Create_Planning_Requirement_Access is access procedure
     (Request  : in  Planning_Requirement;
      Response : out Identifier);
   type Handle_Planning_Requirement_Read_Planning_Requirement_Access is access function
     (Request : Query) return Planning_Requirement_Array;
   type Handle_Planning_Requirement_Update_Planning_Requirement_Access is access procedure
     (Request  : in  Planning_Requirement;
      Response : out Ack);
   type Handle_Planning_Requirement_Delete_Planning_Requirement_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Execution_Requirement_Service
   type Handle_Execution_Requirement_Create_Execution_Requirement_Access is access procedure
     (Request  : in  Execution_Requirement;
      Response : out Identifier);
   type Handle_Execution_Requirement_Read_Execution_Requirement_Access is access function
     (Request : Query) return Execution_Requirement_Array;
   type Handle_Execution_Requirement_Update_Execution_Requirement_Access is access procedure
     (Request  : in  Execution_Requirement;
      Response : out Ack);
   type Handle_Execution_Requirement_Delete_Execution_Requirement_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  State_Service
   type Handle_State_Create_State_Access is access procedure
     (Request  : in  State_Update;
      Response : out Identifier);
   type Handle_State_Update_State_Access is access procedure
     (Request  : in  State_Update;
      Response : out Ack);
   type Handle_State_Delete_State_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Plan_Service
   type Handle_Plan_Create_Plan_Access is access procedure
     (Request  : in  Plan;
      Response : out Identifier);
   type Handle_Plan_Read_Plan_Access is access function
     (Request : Query) return Plan_Array;
   type Handle_Plan_Update_Plan_Access is access procedure
     (Request  : in  Plan;
      Response : out Ack);
   type Handle_Plan_Delete_Plan_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Execution_Run_Service
   type Handle_Execution_Run_Read_Run_Access is access function
     (Request : Query) return Execution_Run_Array;
   --  Requirement_Placement_Service
   type Handle_Requirement_Placement_Read_Placement_Access is access function
     (Request : Query) return Requirement_Placement_Array;

   type Service_Handlers is record
      On_Capabilities_Read_Capabilities : Handle_Capabilities_Read_Capabilities_Access := null;
      On_Planning_Requirement_Create_Planning_Requirement : Handle_Planning_Requirement_Create_Planning_Requirement_Access := null;
      On_Planning_Requirement_Read_Planning_Requirement : Handle_Planning_Requirement_Read_Planning_Requirement_Access := null;
      On_Planning_Requirement_Update_Planning_Requirement : Handle_Planning_Requirement_Update_Planning_Requirement_Access := null;
      On_Planning_Requirement_Delete_Planning_Requirement : Handle_Planning_Requirement_Delete_Planning_Requirement_Access := null;
      On_Execution_Requirement_Create_Execution_Requirement : Handle_Execution_Requirement_Create_Execution_Requirement_Access := null;
      On_Execution_Requirement_Read_Execution_Requirement : Handle_Execution_Requirement_Read_Execution_Requirement_Access := null;
      On_Execution_Requirement_Update_Execution_Requirement : Handle_Execution_Requirement_Update_Execution_Requirement_Access := null;
      On_Execution_Requirement_Delete_Execution_Requirement : Handle_Execution_Requirement_Delete_Execution_Requirement_Access := null;
      On_State_Create_State : Handle_State_Create_State_Access := null;
      On_State_Update_State : Handle_State_Update_State_Access := null;
      On_State_Delete_State : Handle_State_Delete_State_Access := null;
      On_Plan_Create_Plan : Handle_Plan_Create_Plan_Access := null;
      On_Plan_Read_Plan : Handle_Plan_Read_Plan_Access := null;
      On_Plan_Update_Plan : Handle_Plan_Update_Plan_Access := null;
      On_Plan_Delete_Plan : Handle_Plan_Delete_Plan_Access := null;
      On_Execution_Run_Read_Run : Handle_Execution_Run_Read_Run_Access := null;
      On_Requirement_Placement_Read_Placement : Handle_Requirement_Placement_Read_Placement_Access := null;
   end record;

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json");

   function Decode_Capabilities_Read_Capabilities_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Capabilities_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Capabilities_Read_Capabilities
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Planning_Requirement_Create_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Planning_Requirement_Create_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Planning_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Planning_Requirement_Read_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Planning_Requirement_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Planning_Requirement_Read_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Planning_Requirement_Update_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Planning_Requirement_Update_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Planning_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Planning_Requirement_Delete_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Planning_Requirement_Delete_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Execution_Requirement_Create_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Execution_Requirement_Create_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Execution_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Execution_Requirement_Read_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Requirement_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Execution_Requirement_Read_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Execution_Requirement_Update_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Execution_Requirement_Update_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Execution_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Execution_Requirement_Delete_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Execution_Requirement_Delete_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_State_Create_State_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_State_Create_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_State_Update_State_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_State_Update_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_State_Delete_State_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_State_Delete_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Plan_Create_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Plan_Create_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Plan;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Plan_Read_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Plan_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Plan_Read_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Plan_Update_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Plan_Update_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Plan;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Plan_Delete_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Plan_Delete_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Execution_Run_Read_Run_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Run_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Execution_Run_Read_Run
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Requirement_Placement_Read_Placement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Requirement_Placement_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Requirement_Placement_Read_Placement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
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
