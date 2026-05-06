--  Auto-generated gRPC transport spec -- do not edit
--  Backend: grpc | Package: Pyramid.Components.Sensor_data_interpretation.Services.Consumed.GRPC_Transport
--
--  Component-facing calls are typed; the JSON/C ABI shim is private.

with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Sensors.Types;

package Pyramid.Components.Sensor_data_interpretation.Services.Consumed.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   procedure Configure_Library (Path : String);

   type Object_Evidence_Provision_Requirement_Array is array (Positive range <>) of Pyramid.Data_Model.Sensors.Types.Object_Evidence_Provision_Requirement;
   type Object_Aquisition_Requirement_Array is array (Positive range <>) of Pyramid.Data_Model.Sensors.Types.Object_Aquisition_Requirement;

   --  Data_Provision_Dependency_Service

   function Invoke_Create_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Sensors.Types.Object_Evidence_Provision_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_Read_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Evidence_Provision_Requirement_Array;

   function Invoke_Update_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Sensors.Types.Object_Evidence_Provision_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_Delete_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

   --  Data_Processing_Dependency_Service

   function Invoke_Create_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Sensors.Types.Object_Aquisition_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_Read_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Aquisition_Requirement_Array;

   function Invoke_Update_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Sensors.Types.Object_Aquisition_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_Delete_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

end Pyramid.Components.Sensor_data_interpretation.Services.Consumed.GRPC_Transport;
