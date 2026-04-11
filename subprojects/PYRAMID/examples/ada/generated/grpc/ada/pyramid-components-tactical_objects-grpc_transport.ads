--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.GRPC_Transport
--
--  gRPC transport for Ada requires C++ interop via grpc_core.
--  This package defines the Ada-side interface; actual gRPC
--  communication is delegated to C++ via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Tactical_objects.GRPC_Transport is

   --  TacticalObjectService

   procedure Invoke_Create_Tactical_Object
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_tactical_object_service_create_tactical_object";

   procedure Invoke_Read_Tactical_Object
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_tactical_object_service_read_tactical_object";

   procedure Invoke_Update_Tactical_Object
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_tactical_object_service_update_tactical_object";

   procedure Invoke_Delete_Tactical_Object
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_tactical_object_service_delete_tactical_object";

   --  ZoneService

   procedure Invoke_Create_Zone
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_zone_service_create_zone";

   procedure Invoke_Read_Zone
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_zone_service_read_zone";

   procedure Invoke_Update_Zone
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_zone_service_update_zone";

   procedure Invoke_Delete_Zone
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_zone_service_delete_zone";

   --  ObservationIngressService

   procedure Invoke_Create_Observation
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address)
     with Import, Convention => C,
          External_Name => "grpc_observation_ingress_service_create_observation";

   --  Server lifecycle

   procedure Start_Server (Address : Interfaces.C.Strings.chars_ptr)
     with Import, Convention => C,
          External_Name => "grpc_server_start";

   procedure Stop_Server
     with Import, Convention => C,
          External_Name => "grpc_server_stop";

end Pyramid.Components.Tactical_objects.GRPC_Transport;
