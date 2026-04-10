--  Auto-generated service binding specification
--  Generated from: consumed.proto by generate_bindings.py
--  Package: Pyramid.Services.Tactical_Objects.Consumed
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
--  Bridge-topic adapter codecs, when generated, live in the companion
--  Pyramid.Services.Tactical_Objects.Json_Codec package.

with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with Pyramid.Services.Tactical_Objects.Wire_Types;
with Pcl_Bindings;
with Interfaces.C;
with System;

package Pyramid.Services.Tactical_Objects.Consumed is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   type Service_Channel is
     (Ch_Read_Detail,
      Ch_Create_Requirement,
      Ch_Read_Requirement,
      Ch_Update_Requirement,
      Ch_Delete_Requirement,
      Ch_Read_Capability);

   type Capability_Array is array (Positive range <>) of Capability;
   type Object_Detail_Array is array (Positive range <>) of Object_Detail;
   type Object_Evidence_Requirement_Array is array (Positive range <>) of Object_Evidence_Requirement;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Read_Detail : constant String :=
     "object_evidence.read_detail";
   Svc_Create_Requirement : constant String :=
     "object_solution_evidence.create_requirement";
   Svc_Read_Requirement : constant String :=
     "object_solution_evidence.read_requirement";
   Svc_Update_Requirement : constant String :=
     "object_solution_evidence.update_requirement";
   Svc_Delete_Requirement : constant String :=
     "object_solution_evidence.delete_requirement";
   Svc_Read_Capability : constant String :=
     "object_source_capability.read_capability";

   --  -- Standard topic name constants --------------------------

   Topic_Object_Evidence : constant String :=
     "standard.object_evidence";

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   --  -- EntityActions handler callbacks ----------------------------
   --  Supply these callbacks from your component at registration time.

   --  Object_Evidence_Service
   type Handle_Read_Detail_Access is access function
     (Request : Query) return Object_Detail_Array;
   --  Object_Solution_Evidence_Service
   type Handle_Create_Requirement_Access is access procedure
     (Request  : in  Object_Evidence_Requirement;
      Response : out Identifier);
   type Handle_Read_Requirement_Access is access function
     (Request : Query) return Object_Evidence_Requirement_Array;
   type Handle_Update_Requirement_Access is access procedure
     (Request  : in  Object_Evidence_Requirement;
      Response : out Ack);
   type Handle_Delete_Requirement_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Object_Source_Capability_Service
   type Handle_Read_Capability_Access is access function
     (Request : Query) return Capability_Array;

   type Service_Handlers is record
      On_Read_Detail : Handle_Read_Detail_Access := null;
      On_Create_Requirement : Handle_Create_Requirement_Access := null;
      On_Read_Requirement : Handle_Read_Requirement_Access := null;
      On_Update_Requirement : Handle_Update_Requirement_Access := null;
      On_Delete_Requirement : Handle_Delete_Requirement_Access := null;
      On_Read_Capability : Handle_Read_Capability_Access := null;
   end record;

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json");

   procedure Publish_Object_Evidence
     (Exec    : Pcl_Bindings.Pcl_Executor_Access;
      Payload : Pyramid.Services.Tactical_Objects.Wire_Types.Object_Evidence;
      Content_Type : String := "application/json");

   procedure Publish_Object_Evidence
     (Exec    : Pcl_Bindings.Pcl_Executor_Access;
      Payload : String;
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

end Pyramid.Services.Tactical_Objects.Consumed;
