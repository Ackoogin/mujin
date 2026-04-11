--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport
--
--  gRPC transport for Ada uses the generated canonical C ABI shim.
--  Requests and responses are JSON strings; the shim performs
--  protobuf encoding/decoding and gRPC client calls in C++.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   --  Matching_Objects_Service

   function Invoke_Read_Match_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_matching_objects_service_read_match_json";

   --  Object_Of_Interest_Service

   function Invoke_Create_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_object_of_interest_service_create_requirement_json";

   function Invoke_Read_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_object_of_interest_service_read_requirement_json";

   function Invoke_Update_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_object_of_interest_service_update_requirement_json";

   function Invoke_Delete_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_object_of_interest_service_delete_requirement_json";

   --  Specific_Object_Detail_Service

   function Invoke_Read_Detail_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_specific_object_detail_service_read_detail_json";

   procedure Free_String (Value : Interfaces.C.Strings.chars_ptr)
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_grpc_free_string";

end Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport;
