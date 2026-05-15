--  Auto-generated service binding specification
--  Generated from: pyramid.components.sensor_data_interpretation.services.consumed.proto by generate_bindings.py
--  Package: Pyramid.Services.Sensor_Data_Interpretation.Consumed
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

package Pyramid.Services.Sensor_Data_Interpretation.Consumed is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   type Service_Channel is
     (Ch_Data_Provision_Dependency_Create_Requirement,
      Ch_Data_Provision_Dependency_Read_Requirement,
      Ch_Data_Provision_Dependency_Update_Requirement,
      Ch_Data_Provision_Dependency_Delete_Requirement,
      Ch_Data_Processing_Dependency_Create_Requirement,
      Ch_Data_Processing_Dependency_Read_Requirement,
      Ch_Data_Processing_Dependency_Update_Requirement,
      Ch_Data_Processing_Dependency_Delete_Requirement);

   type Object_Aquisition_Requirement_Array is array (Positive range <>) of Object_Aquisition_Requirement;
   type Object_Evidence_Provision_Requirement_Array is array (Positive range <>) of Object_Evidence_Provision_Requirement;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Data_Provision_Dependency_Create_Requirement : constant String :=
     "data_provision_dependency.create_requirement";
   Svc_Data_Provision_Dependency_Read_Requirement : constant String :=
     "data_provision_dependency.read_requirement";
   Svc_Data_Provision_Dependency_Update_Requirement : constant String :=
     "data_provision_dependency.update_requirement";
   Svc_Data_Provision_Dependency_Delete_Requirement : constant String :=
     "data_provision_dependency.delete_requirement";
   Svc_Data_Processing_Dependency_Create_Requirement : constant String :=
     "data_processing_dependency.create_requirement";
   Svc_Data_Processing_Dependency_Read_Requirement : constant String :=
     "data_processing_dependency.read_requirement";
   Svc_Data_Processing_Dependency_Update_Requirement : constant String :=
     "data_processing_dependency.update_requirement";
   Svc_Data_Processing_Dependency_Delete_Requirement : constant String :=
     "data_processing_dependency.delete_requirement";

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   Json_Content_Type : constant String := "application/json";
   Flatbuffers_Content_Type : constant String := "application/flatbuffers";

   function Supports_Content_Type (Content_Type : String) return Boolean;

   --  -- EntityActions handler callbacks ----------------------------
   --  Supply these callbacks from your component at registration time.

   --  Data_Provision_Dependency_Service
   type Handle_Data_Provision_Dependency_Create_Requirement_Access is access procedure
     (Request  : in  Object_Evidence_Provision_Requirement;
      Response : out Identifier);
   type Handle_Data_Provision_Dependency_Read_Requirement_Access is access function
     (Request : Query) return Object_Evidence_Provision_Requirement_Array;
   type Handle_Data_Provision_Dependency_Update_Requirement_Access is access procedure
     (Request  : in  Object_Evidence_Provision_Requirement;
      Response : out Ack);
   type Handle_Data_Provision_Dependency_Delete_Requirement_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);
   --  Data_Processing_Dependency_Service
   type Handle_Data_Processing_Dependency_Create_Requirement_Access is access procedure
     (Request  : in  Object_Aquisition_Requirement;
      Response : out Identifier);
   type Handle_Data_Processing_Dependency_Read_Requirement_Access is access function
     (Request : Query) return Object_Aquisition_Requirement_Array;
   type Handle_Data_Processing_Dependency_Update_Requirement_Access is access procedure
     (Request  : in  Object_Aquisition_Requirement;
      Response : out Ack);
   type Handle_Data_Processing_Dependency_Delete_Requirement_Access is access procedure
     (Request  : in  Identifier;
      Response : out Ack);

   type Service_Handlers is record
      On_Data_Provision_Dependency_Create_Requirement : Handle_Data_Provision_Dependency_Create_Requirement_Access := null;
      On_Data_Provision_Dependency_Read_Requirement : Handle_Data_Provision_Dependency_Read_Requirement_Access := null;
      On_Data_Provision_Dependency_Update_Requirement : Handle_Data_Provision_Dependency_Update_Requirement_Access := null;
      On_Data_Provision_Dependency_Delete_Requirement : Handle_Data_Provision_Dependency_Delete_Requirement_Access := null;
      On_Data_Processing_Dependency_Create_Requirement : Handle_Data_Processing_Dependency_Create_Requirement_Access := null;
      On_Data_Processing_Dependency_Read_Requirement : Handle_Data_Processing_Dependency_Read_Requirement_Access := null;
      On_Data_Processing_Dependency_Update_Requirement : Handle_Data_Processing_Dependency_Update_Requirement_Access := null;
      On_Data_Processing_Dependency_Delete_Requirement : Handle_Data_Processing_Dependency_Delete_Requirement_Access := null;
   end record;

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
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

end Pyramid.Services.Sensor_Data_Interpretation.Consumed;
