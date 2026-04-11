--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport
--
--  gRPC transport for Ada requires C++ interop via grpc_core.
--  This package defines the Ada-side interface; actual gRPC
--  communication is delegated to C++ via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport is

   --  Object_Evidence_Service

   procedure Invoke_Read_Detail
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_evidence_service_read_detail";

   --  Object_Solution_Evidence_Service

   procedure Invoke_Create_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_solution_evidence_service_create_requirement";

   procedure Invoke_Read_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_solution_evidence_service_read_requirement";

   procedure Invoke_Update_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_solution_evidence_service_update_requirement";

   procedure Invoke_Delete_Requirement
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_solution_evidence_service_delete_requirement";

   --  Object_Source_Capability_Service

   procedure Invoke_Read_Capability
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_object_source_capability_service_read_capability";

   --  Server lifecycle

   procedure Start_Server (Address : Interfaces.C.Strings.chars_ptr)
     with Import, Convention => C,
          External_Name => "grpc_server_start";

   procedure Stop_Server
     with Import, Convention => C,
          External_Name => "grpc_server_stop";

end Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport;
