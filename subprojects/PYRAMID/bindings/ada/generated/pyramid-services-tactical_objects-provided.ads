--  Auto-generated service binding specification
--  Generated from: provided.proto by generate_bindings.py
--  Package: Pyramid.Services.Tactical_Objects.Provided
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
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with Pcl_Bindings;
with Interfaces.C;
with System;

package Pyramid.Services.Tactical_Objects.Provided is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   type Service_Channel is
     (Ch_Read_Match,
      Ch_Create_Requirement,
      Ch_Read_Requirement,
      Ch_Update_Requirement,
      Ch_Delete_Requirement,
      Ch_Read_Detail);

   type Object_Detail_Array is array (Positive range <>) of Object_Detail;
   type Object_Interest_Requirement_Array is array (Positive range <>) of Object_Interest_Requirement;
   type Object_Match_Array is array (Positive range <>) of Object_Match;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Read_Match : constant String :=
     "matching_objects.read_match";
   Svc_Create_Requirement : constant String :=
     "object_of_interest.create_requirement";
   Svc_Read_Requirement : constant String :=
     "object_of_interest.read_requirement";
   Svc_Update_Requirement : constant String :=
     "object_of_interest.update_requirement";
   Svc_Delete_Requirement : constant String :=
     "object_of_interest.delete_requirement";
   Svc_Read_Detail : constant String :=
     "specific_object_detail.read_detail";

   --  -- Standard topic name constants --------------------------

   Topic_Entity_Matches : constant String :=
     "standard.entity_matches";
   Topic_Evidence_Requirements : constant String :=
     "standard.evidence_requirements";

   --  -- ROS2 endpoint constants --------------------------------

   Ros2_Transport_Content_Type : constant String := "application/ros2";

   Matching_Objects_Read_Match_Open_Service : constant String :=
     "/pyramid/stream/matching_objects/read_match/open";
   Matching_Objects_Read_Match_Frame_Topic : constant String :=
     "/pyramid/stream/matching_objects/read_match/frames";
   Matching_Objects_Read_Match_Cancel_Topic : constant String :=
     "/pyramid/stream/matching_objects/read_match/cancel";

   Object_Of_Interest_Create_Requirement_Service : constant String :=
     "/pyramid/service/object_of_interest/create_requirement";

   Object_Of_Interest_Read_Requirement_Open_Service : constant String :=
     "/pyramid/stream/object_of_interest/read_requirement/open";
   Object_Of_Interest_Read_Requirement_Frame_Topic : constant String :=
     "/pyramid/stream/object_of_interest/read_requirement/frames";
   Object_Of_Interest_Read_Requirement_Cancel_Topic : constant String :=
     "/pyramid/stream/object_of_interest/read_requirement/cancel";

   Object_Of_Interest_Update_Requirement_Service : constant String :=
     "/pyramid/service/object_of_interest/update_requirement";

   Object_Of_Interest_Delete_Requirement_Service : constant String :=
     "/pyramid/service/object_of_interest/delete_requirement";

   Specific_Object_Detail_Read_Detail_Open_Service : constant String :=
     "/pyramid/stream/specific_object_detail/read_detail/open";
   Specific_Object_Detail_Read_Detail_Frame_Topic : constant String :=
     "/pyramid/stream/specific_object_detail/read_detail/frames";
   Specific_Object_Detail_Read_Detail_Cancel_Topic : constant String :=
     "/pyramid/stream/specific_object_detail/read_detail/cancel";

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

   --  Matching_Objects_Service
   type Handle_Read_Match_Access is access function
     (Request : Query) return Object_Match_Array;
   --  Object_Of_Interest_Service
   type Handle_Create_Requirement_Access is access procedure
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier);
   type Handle_Read_Requirement_Access is access function
     (Request : Query) return Object_Interest_Requirement_Array;
   type Handle_Update_Requirement_Access is access procedure
     (Request  : in  Object_Interest_Requirement;
      Response : out Ack);
   type Handle_Delete_Requirement_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Specific_Object_Detail_Service
   type Handle_Read_Detail_Access is access function
     (Request : Query) return Object_Detail_Array;

   type Service_Handlers is record
      On_Read_Match : Handle_Read_Match_Access := null;
      On_Create_Requirement : Handle_Create_Requirement_Access := null;
      On_Read_Requirement : Handle_Read_Requirement_Access := null;
      On_Update_Requirement : Handle_Update_Requirement_Access := null;
      On_Delete_Requirement : Handle_Delete_Requirement_Access := null;
      On_Read_Detail : Handle_Read_Detail_Access := null;
   end record;

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Subscribe_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   procedure Subscribe_Evidence_Requirements
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Entity_Matches
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Match_Array;

   function Decode_Evidence_Requirements
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Evidence_Requirement;

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json");

   function Decode_Read_Match_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Match_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Match
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Create_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Create_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Object_Interest_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Read_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Interest_Requirement_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Update_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Update_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Object_Interest_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Delete_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Delete_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Read_Detail_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Detail_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Read_Detail
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

end Pyramid.Services.Tactical_Objects.Provided;
