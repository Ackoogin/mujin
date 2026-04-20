--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.GRPC_Transport
--
--  gRPC transport for Ada uses the generated canonical C ABI shim.
--  Requests and responses are JSON strings; the shim performs
--  protobuf encoding/decoding and gRPC client calls in C++.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Tactical_objects.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   --  TacticalObjectService

   function Invoke_Create_Tactical_Object_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_tactical_object_service_create_tactical_object_json";

   function Invoke_Read_Tactical_Object_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_tactical_object_service_read_tactical_object_json";

   function Invoke_Update_Tactical_Object_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_tactical_object_service_update_tactical_object_json";

   function Invoke_Delete_Tactical_Object_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_tactical_object_service_delete_tactical_object_json";

   --  ZoneService

   function Invoke_Create_Zone_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_zone_service_create_zone_json";

   function Invoke_Read_Zone_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_zone_service_read_zone_json";

   function Invoke_Update_Zone_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_zone_service_update_zone_json";

   function Invoke_Delete_Zone_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_zone_service_delete_zone_json";

   --  ObservationIngressService

   function Invoke_Create_Observation_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_tactical_objects_observation_ingress_service_create_observation_json";

   procedure Free_String (Value : Interfaces.C.Strings.chars_ptr)
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_grpc_free_string";

end Pyramid.Components.Tactical_objects.GRPC_Transport;
