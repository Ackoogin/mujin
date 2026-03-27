--  Auto-generated service binding specification
--  Generated from: pyramid by ada_service_generator.py
--  Package: Pyramid.Services.Tactical_Objects.Provided
--
--  Architecture: component logic > service binding (this) > PCL
--
--  This package provides:
--    1. Wire-name constants and topic constants
--    2. EntityActions handler stubs (Handle_*)
--    3. PCL binding procedures (Subscribe_*, Invoke_*, Publish_*)
--    4. Msg_To_String utility for PCL message payloads
--
--  JSON serialisation/deserialisation is provided by the companion
--  Pyramid.Services.Tactical_Objects.Json_Codec package.

with Pyramid_Data_Model_Base_Types;  use Pyramid_Data_Model_Base_Types;
with Pyramid_Data_Model_Common_Types;  use Pyramid_Data_Model_Common_Types;
with Pyramid_Data_Model_Tactical_Types;  use Pyramid_Data_Model_Tactical_Types;
with Pyramid.Services.Tactical_Objects.Json_Codec;
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

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   --  -- EntityActions handlers ------------------------------------
   --  Implement these procedures in the package body.

   --  Matching_Objects_Service
   function Handle_Read_Match
     (Request : Query) return Object_Match_Array;
   --  Object_Of_Interest_Service
   procedure Handle_Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier);
   function Handle_Read_Requirement
     (Request : Query) return Object_Interest_Requirement_Array;
   procedure Handle_Update_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Ack);
   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack);
   --  Specific_Object_Detail_Service
   function Handle_Read_Detail
     (Request : Query) return Object_Detail_Array;

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Subscribe_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address);

   procedure Subscribe_Evidence_Requirements
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address);

   procedure Invoke_Read_Match
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address);

   procedure Invoke_Create_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : Pyramid.Services.Tactical_Objects.Json_Codec.Create_Requirement_Request;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address);

   procedure Invoke_Read_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address);

   procedure Invoke_Update_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : Pyramid.Services.Tactical_Objects.Json_Codec.Create_Requirement_Request;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address);

   procedure Invoke_Delete_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address);

   procedure Invoke_Read_Detail
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address);

   --  -- Transport integration point ------------------------------

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural);

end Pyramid.Services.Tactical_Objects.Provided;
