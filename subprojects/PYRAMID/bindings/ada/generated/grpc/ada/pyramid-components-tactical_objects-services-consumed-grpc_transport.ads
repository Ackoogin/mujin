--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport
--
--  gRPC transport for Ada uses the generated canonical C ABI shim.
--  Requests and responses are JSON strings; the shim performs
--  protobuf encoding/decoding and gRPC client calls in C++.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   --  Object_Evidence_Service

   function Invoke_Read_Detail_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_consumed_object_evidence_service_read_detail_json";

   --  Object_Solution_Evidence_Service

   function Invoke_Create_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_consumed_object_solution_evidence_service_create_requirement_json";

   function Invoke_Read_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_consumed_object_solution_evidence_service_read_requirement_json";

   function Invoke_Update_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_consumed_object_solution_evidence_service_update_requirement_json";

   function Invoke_Delete_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_consumed_object_solution_evidence_service_delete_requirement_json";

   --  Object_Source_Capability_Service

   function Invoke_Read_Capability_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_consumed_object_source_capability_service_read_capability_json";

   procedure Free_String (Value : Interfaces.C.Strings.chars_ptr)
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_grpc_free_string";

end Pyramid.Components.Tactical_objects.Services.Consumed.GRPC_Transport;
