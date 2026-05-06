--  Auto-generated gRPC transport spec -- do not edit
--  Backend: grpc | Package: Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport
--
--  Component-facing calls are typed; the JSON/C ABI shim is private.

with Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;

package Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport is

   Content_Type : constant String := "application/grpc";

   procedure Configure_Library (Path : String);

   type Capabilities_Array is array (Positive range <>) of Pyramid.Data_Model.Autonomy.Types.Capabilities;
   type Planning_Requirement_Array is array (Positive range <>) of Pyramid.Data_Model.Autonomy.Types.Planning_Requirement;
   type Execution_Requirement_Array is array (Positive range <>) of Pyramid.Data_Model.Autonomy.Types.Execution_Requirement;
   type Plan_Array is array (Positive range <>) of Pyramid.Data_Model.Autonomy.Types.Plan;
   type Execution_Run_Array is array (Positive range <>) of Pyramid.Data_Model.Autonomy.Types.Execution_Run;
   type Requirement_Placement_Array is array (Positive range <>) of Pyramid.Data_Model.Autonomy.Types.Requirement_Placement;

   --  Capabilities_Service

   function Invoke_capabilities_Read_Capabilities
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Capabilities_Array;

   --  Planning_Requirement_Service

   function Invoke_planning_requirement_Create_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Planning_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_planning_requirement_Read_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Planning_Requirement_Array;

   function Invoke_planning_requirement_Update_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Planning_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_planning_requirement_Delete_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

   --  Execution_Requirement_Service

   function Invoke_execution_requirement_Create_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Execution_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_execution_requirement_Read_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Execution_Requirement_Array;

   function Invoke_execution_requirement_Update_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Execution_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_execution_requirement_Delete_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

   --  State_Service

   function Invoke_state_Create_State
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.State_Update)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_state_Update_State
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.State_Update)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_state_Delete_State
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

   --  Plan_Service

   function Invoke_plan_Create_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Plan)
      return Pyramid.Data_Model.Base.Types.Identifier;

   function Invoke_plan_Read_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Plan_Array;

   function Invoke_plan_Update_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Plan)
      return Pyramid.Data_Model.Common.Types.Ack;

   function Invoke_plan_Delete_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack;

   --  Execution_Run_Service

   function Invoke_execution_run_Read_Run
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Execution_Run_Array;

   --  Requirement_Placement_Service

   function Invoke_requirement_placement_Read_Placement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Requirement_Placement_Array;

end Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport;
