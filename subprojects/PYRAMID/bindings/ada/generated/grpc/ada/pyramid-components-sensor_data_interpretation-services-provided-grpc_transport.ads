--  Auto-generated gRPC transport spec -- do not edit
--  Backend: grpc | Package: Pyramid.Components.Sensor_data_interpretation.Services.Provided.GRPC_Transport
--
--  Component-facing calls are typed; the JSON/C ABI shim is private.

with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Sensors.Types;

package Pyramid.Components.Sensor_data_interpretation.Services.Provided.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   procedure Configure_Library (Path : String);

   type Capability_Array is array (Positive range <>) of Pyramid.Data_Model.Common.Types.Capability;
   type Interpretation_Requirement_Array is array (Positive range <>) of Pyramid.Data_Model.Sensors.Types.Interpretation_Requirement;

   --  Interpretation_Requirement_Service

   function Invoke_interpretation_requirement_Read_Capability
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Capability_Array;

   function Invoke_interpretation_requirement_Create_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Sensors.Types.Interpretation_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_interpretation_requirement_Read_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Interpretation_Requirement_Array;

   function Invoke_interpretation_requirement_Update_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Sensors.Types.Interpretation_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_interpretation_requirement_Delete_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

end Pyramid.Components.Sensor_data_interpretation.Services.Provided.GRPC_Transport;
