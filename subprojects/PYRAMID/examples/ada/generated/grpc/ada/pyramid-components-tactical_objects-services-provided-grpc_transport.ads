--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport
--
--  gRPC transport for Ada requires C++ interop via grpc_core.
--  This package defines the Ada-side interface; actual gRPC
--  communication is delegated to C++ via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport is

   --  Matching_Objects_Service

   procedure Invoke_Read_Match
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_matching_objects_service_read_match";

   --  Object_Of_Interest_Service

   procedure Invoke_Create_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_of_interest_service_create_requirement";

   procedure Invoke_Read_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_of_interest_service_read_requirement";

   procedure Invoke_Update_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_of_interest_service_update_requirement";

   procedure Invoke_Delete_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_of_interest_service_delete_requirement";

   --  Specific_Object_Detail_Service

   procedure Invoke_Read_Detail
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_specific_object_detail_service_read_detail";

   --  Server lifecycle

   procedure Start_Server (Address : Interfaces.C.Strings.chars_ptr)
     with Import, Convention => C,
          External_Name => "grpc_server_start";

   procedure Stop_Server
     with Import, Convention => C,
          External_Name => "grpc_server_stop";

end Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport;
