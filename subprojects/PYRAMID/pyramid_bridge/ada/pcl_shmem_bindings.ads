--  pcl_shmem_bindings.ads
--
--  Ada thin-binding for the PCL shared-memory cross-process bus transport.
--  Mirrors pcl/pcl_transport_shared_memory.h.
--
--  Usage:
--    Ctx := Pcl_Shmem_Bindings.Create
--             (New_String ("my_bus"), New_String ("my_participant"), Exec);
--    Pcl_Bindings.Set_Transport
--             (Exec, Pcl_Shmem_Bindings.Get_Transport (Ctx));
--    -- configure, activate and add Gateway_Container (Ctx) to the executor

with Pcl_Bindings;
with Interfaces.C.Strings;

package Pcl_Shmem_Bindings is

   type Pcl_Shared_Memory_Transport is limited private;
   type Pcl_Shared_Memory_Transport_Access is access all Pcl_Shared_Memory_Transport;
   pragma Convention (C, Pcl_Shared_Memory_Transport_Access);

   --  Join (or create) a named cross-process shared-memory bus.
   --  Bus_Name must be the same string in every participating process.
   --  Participant_Id must be unique within the bus.
   function Create
     (Bus_Name       : Interfaces.C.Strings.chars_ptr;
      Participant_Id : Interfaces.C.Strings.chars_ptr;
      Executor       : Pcl_Bindings.Pcl_Executor_Access)
      return Pcl_Shared_Memory_Transport_Access;
   pragma Import (C, Create, "pcl_shared_memory_transport_create");

   function Get_Transport
     (Ctx : Pcl_Shared_Memory_Transport_Access)
      return Pcl_Bindings.Pcl_Transport_Const_Access;
   pragma Import (C, Get_Transport, "pcl_shared_memory_transport_get_transport");

   --  Internal gateway container that dispatches inbound service requests.
   --  Must be configured, activated, and added to the executor.
   function Gateway_Container
     (Ctx : Pcl_Shared_Memory_Transport_Access)
      return Pcl_Bindings.Pcl_Container_Access;
   pragma Import (C, Gateway_Container,
                  "pcl_shared_memory_transport_gateway_container");

   procedure Destroy (Ctx : Pcl_Shared_Memory_Transport_Access);
   pragma Import (C, Destroy, "pcl_shared_memory_transport_destroy");

private
   type Pcl_Shared_Memory_Transport is null record;
   pragma Convention (C, Pcl_Shared_Memory_Transport);

end Pcl_Shmem_Bindings;
