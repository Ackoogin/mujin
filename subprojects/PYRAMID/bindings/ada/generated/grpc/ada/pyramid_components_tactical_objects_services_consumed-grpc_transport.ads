--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport
--
--  Component-facing calls are typed; the JSON/C ABI shim is private.

with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;

package Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   procedure Configure_Library (Path : String);

   type Object_Detail_Array is array (Positive range <>) of Pyramid.Data_Model.Tactical.Types.Object_Detail;
   type Object_Evidence_Requirement_Array is array (Positive range <>) of Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement;
   type Capability_Array is array (Positive range <>) of Pyramid.Data_Model.Common.Types.Capability;

   --  Object_Evidence_Service

   function Invoke_Read_Detail
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Detail_Array;

   --  Object_Solution_Evidence_Service

   function Invoke_Create_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_Read_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Evidence_Requirement_Array;

   function Invoke_Update_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_Delete_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

   --  Object_Source_Capability_Service

   function Invoke_Read_Capability
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Capability_Array;

end Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport;
