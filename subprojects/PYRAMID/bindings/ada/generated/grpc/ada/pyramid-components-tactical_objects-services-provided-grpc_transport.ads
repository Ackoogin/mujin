--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport
--
--  Component-facing calls are typed; the JSON/C ABI shim is private.

with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;

package Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   procedure Configure_Library (Path : String);

   type Object_Match_Array is array (Positive range <>) of Pyramid.Data_Model.Tactical.Types.Object_Match;
   type Object_Interest_Requirement_Array is array (Positive range <>) of Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement;
   type Object_Detail_Array is array (Positive range <>) of Pyramid.Data_Model.Tactical.Types.Object_Detail;

   --  Matching_Objects_Service

   function Invoke_Read_Match
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Match_Array;

   --  Object_Of_Interest_Service

   function Invoke_Create_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_Read_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Interest_Requirement_Array;

   function Invoke_Update_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_Delete_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

   --  Specific_Object_Detail_Service

   function Invoke_Read_Detail
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Detail_Array;

end Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport;
