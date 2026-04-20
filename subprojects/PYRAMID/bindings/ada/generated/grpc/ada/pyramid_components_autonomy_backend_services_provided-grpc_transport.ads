--  Auto-generated gRPC transport spec — do not edit
--  Backend: grpc | Package: Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport
--
--  gRPC transport for Ada uses the generated canonical C ABI shim.
--  Requests and responses are JSON strings; the shim performs
--  protobuf encoding/decoding and gRPC client calls in C++.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   --  Capabilities_Service

   function Invoke_Read_Capabilities_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_capabilities_service_read_capabilities_json";

   --  Planning_Execution_Service

   function Invoke_Create_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_planning_execution_service_create_requirement_json";

   function Invoke_Read_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_planning_execution_service_read_requirement_json";

   function Invoke_Update_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_planning_execution_service_update_requirement_json";

   function Invoke_Delete_Requirement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_planning_execution_service_delete_requirement_json";

   --  State_Service

   function Invoke_Create_State_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_state_service_create_state_json";

   function Invoke_Update_State_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_state_service_update_state_json";

   function Invoke_Delete_State_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_state_service_delete_state_json";

   --  Plan_Service

   function Invoke_Read_Plan_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_plan_service_read_plan_json";

   --  Execution_Run_Service

   function Invoke_Read_Run_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_execution_run_service_read_run_json";

   --  Requirement_Placement_Service

   function Invoke_Read_Placement_Json
     (Channel      : Interfaces.C.Strings.chars_ptr;
      Request_Json : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "grpc_provided_requirement_placement_service_read_placement_json";

   procedure Free_String (Value : Interfaces.C.Strings.chars_ptr)
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_grpc_free_string";

end Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport;
