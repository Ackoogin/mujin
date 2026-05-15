--  Auto-generated service binding specification
--  Generated from: pyramid.components.sensor_data_interpretation.services.provided.proto by generate_bindings.py
--  Package: Pyramid.Services.Sensor_Data_Interpretation.Provided
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
with Pyramid.Data_Model.Sensors.Types;  use Pyramid.Data_Model.Sensors.Types;
with Pcl_Bindings;
with Interfaces.C;
with System;

package Pyramid.Services.Sensor_Data_Interpretation.Provided is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   type Service_Channel is
     (Ch_Interpretation_Requirement_Read_Capability,
      Ch_Interpretation_Requirement_Create_Requirement,
      Ch_Interpretation_Requirement_Read_Requirement,
      Ch_Interpretation_Requirement_Update_Requirement,
      Ch_Interpretation_Requirement_Delete_Requirement);

   type Capability_Array is array (Positive range <>) of Capability;
   type Interpretation_Requirement_Array is array (Positive range <>) of Interpretation_Requirement;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Interpretation_Requirement_Read_Capability : constant String :=
     "interpretation_requirement.read_capability";
   Svc_Interpretation_Requirement_Create_Requirement : constant String :=
     "interpretation_requirement.create_requirement";
   Svc_Interpretation_Requirement_Read_Requirement : constant String :=
     "interpretation_requirement.read_requirement";
   Svc_Interpretation_Requirement_Update_Requirement : constant String :=
     "interpretation_requirement.update_requirement";
   Svc_Interpretation_Requirement_Delete_Requirement : constant String :=
     "interpretation_requirement.delete_requirement";

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   Json_Content_Type : constant String := "application/json";
   Flatbuffers_Content_Type : constant String := "application/flatbuffers";

   function Supports_Content_Type (Content_Type : String) return Boolean;

   --  -- EntityActions handler callbacks ----------------------------
   --  Supply these callbacks from your component at registration time.

   --  Interpretation_Requirement_Service
   type Handle_Interpretation_Requirement_Read_Capability_Access is access function
     (Request : Query) return Capability_Array;
   type Handle_Interpretation_Requirement_Create_Requirement_Access is access procedure
     (Request  : in  Interpretation_Requirement;
      Response : out Identifier);
   type Handle_Interpretation_Requirement_Read_Requirement_Access is access function
     (Request : Query) return Interpretation_Requirement_Array;
   type Handle_Interpretation_Requirement_Update_Requirement_Access is access procedure
     (Request  : in  Interpretation_Requirement;
      Response : out Ack);
   type Handle_Interpretation_Requirement_Delete_Requirement_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);

   type Service_Handlers is record
      On_Interpretation_Requirement_Read_Capability : Handle_Interpretation_Requirement_Read_Capability_Access := null;
      On_Interpretation_Requirement_Create_Requirement : Handle_Interpretation_Requirement_Create_Requirement_Access := null;
      On_Interpretation_Requirement_Read_Requirement : Handle_Interpretation_Requirement_Read_Requirement_Access := null;
      On_Interpretation_Requirement_Update_Requirement : Handle_Interpretation_Requirement_Update_Requirement_Access := null;
      On_Interpretation_Requirement_Delete_Requirement : Handle_Interpretation_Requirement_Delete_Requirement_Access := null;
   end record;

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json");

   function Decode_Interpretation_Requirement_Read_Capability_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Capability_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Interpretation_Requirement_Read_Capability
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Interpretation_Requirement_Create_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Interpretation_Requirement_Create_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Interpretation_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Interpretation_Requirement_Read_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Interpretation_Requirement_Array;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Interpretation_Requirement_Read_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Interpretation_Requirement_Update_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Interpretation_Requirement_Update_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Interpretation_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json");

   function Decode_Interpretation_Requirement_Delete_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack;

   --  Invoke via executor transport (transport-agnostic).
   procedure Invoke_Interpretation_Requirement_Delete_Requirement
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

end Pyramid.Services.Sensor_Data_Interpretation.Provided;
