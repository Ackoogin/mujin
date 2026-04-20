--  tobj_evidence_provider.ads
--
--  Reusable component: subscribes to standard.evidence_requirements,
--  publishes standard observations to standard.object_evidence via
--  the generated service bindings.
--
--  Architecture: component logic (this) > service binding > PCL

with Pcl_Bindings;
with Ada.Strings.Unbounded;
with Pyramid.Services.Tactical_Objects.Provided;
with System;

package Tobj_Evidence_Provider is

   Content_Type : Ada.Strings.Unbounded.Unbounded_String :=
     Ada.Strings.Unbounded.To_Unbounded_String
       (Pyramid.Services.Tactical_Objects.Provided.Json_Content_Type);

   --  Component state (written by callbacks on the executor thread)
   Evidence_Req_Received : Boolean := False;
   Observation_Sent      : Boolean := False;

   --  Set before activating — the executor handle for publishing
   Exec_Handle : Pcl_Bindings.Pcl_Executor_Access := null;

   --  PCL lifecycle callback — subscribes to standard.evidence_requirements
   function On_Configure
     (Container : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, On_Configure);

   --  Subscriber callback for standard.evidence_requirements
   procedure On_Evidence_Requirement
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Evidence_Requirement);

   --  Reset state between runs
   procedure Reset;

end Tobj_Evidence_Provider;
