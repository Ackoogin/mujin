--  Auto-generated Protobuf codec spec -- do not edit
--  Backend: protobuf | Package: Pyramid.Data_model.Autonomy.Protobuf_Codec
--
--  Thin Ada binding to protobuf C++ codec via C interop.
--  Protobuf has no native Ada support; serialisation is
--  delegated to the C++ implementation via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Data_model.Autonomy.Protobuf_Codec is

   Content_Type : constant String := "application/protobuf";

   --  RequirementReference: protobuf SerializeToString / ParseFromArray
   function To_Binary_Requirement_Reference (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RequirementReference_to_protobuf";

   function From_Binary_Requirement_Reference
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RequirementReference_from_protobuf";

   --  AgentState: protobuf SerializeToString / ParseFromArray
   function To_Binary_Agent_State (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "AgentState_to_protobuf";

   function From_Binary_Agent_State
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "AgentState_from_protobuf";

   --  PlanningPolicy: protobuf SerializeToString / ParseFromArray
   function To_Binary_Planning_Policy (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "PlanningPolicy_to_protobuf";

   function From_Binary_Planning_Policy
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "PlanningPolicy_from_protobuf";

   --  PlanningGoal: protobuf SerializeToString / ParseFromArray
   function To_Binary_Planning_Goal (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "PlanningGoal_to_protobuf";

   function From_Binary_Planning_Goal
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "PlanningGoal_from_protobuf";

   --  ExecutionPolicy: protobuf SerializeToString / ParseFromArray
   function To_Binary_Execution_Policy (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ExecutionPolicy_to_protobuf";

   function From_Binary_Execution_Policy
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ExecutionPolicy_from_protobuf";

   --  PlanningRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Planning_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "PlanningRequirement_to_protobuf";

   function From_Binary_Planning_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "PlanningRequirement_from_protobuf";

   --  ExecutionRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Execution_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ExecutionRequirement_to_protobuf";

   function From_Binary_Execution_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ExecutionRequirement_from_protobuf";

   --  WorldFactUpdate: protobuf SerializeToString / ParseFromArray
   function To_Binary_World_Fact_Update (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "WorldFactUpdate_to_protobuf";

   function From_Binary_World_Fact_Update
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "WorldFactUpdate_from_protobuf";

   --  StateUpdate: protobuf SerializeToString / ParseFromArray
   function To_Binary_State_Update (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "StateUpdate_to_protobuf";

   function From_Binary_State_Update
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "StateUpdate_from_protobuf";

   --  Capabilities: protobuf SerializeToString / ParseFromArray
   function To_Binary_Capabilities (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Capabilities_to_protobuf";

   function From_Binary_Capabilities
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Capabilities_from_protobuf";

   --  PlannedComponentInteraction: protobuf SerializeToString / ParseFromArray
   function To_Binary_Planned_Component_Interaction (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "PlannedComponentInteraction_to_protobuf";

   function From_Binary_Planned_Component_Interaction
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "PlannedComponentInteraction_from_protobuf";

   --  PlanStep: protobuf SerializeToString / ParseFromArray
   function To_Binary_Plan_Step (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "PlanStep_to_protobuf";

   function From_Binary_Plan_Step
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "PlanStep_from_protobuf";

   --  Plan: protobuf SerializeToString / ParseFromArray
   function To_Binary_Plan (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Plan_to_protobuf";

   function From_Binary_Plan
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Plan_from_protobuf";

   --  RequirementPlacement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Requirement_Placement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RequirementPlacement_to_protobuf";

   function From_Binary_Requirement_Placement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RequirementPlacement_from_protobuf";

   --  ExecutionRun: protobuf SerializeToString / ParseFromArray
   function To_Binary_Execution_Run (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ExecutionRun_to_protobuf";

   function From_Binary_Execution_Run
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ExecutionRun_from_protobuf";

end Pyramid.Data_model.Autonomy.Protobuf_Codec;
