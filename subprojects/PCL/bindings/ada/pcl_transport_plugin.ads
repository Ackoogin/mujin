--  Ada helper for loading the socket transport as a runtime .so plugin.
--
--  A plugin-only Ada client links no transport library: it loads the socket
--  transport from a .so at run time, exactly like the codec. The plugin is
--  configured through the same opaque config_json that codec plugins receive
--  (uniform pass-through); for the socket transport that JSON carries role,
--  host, port and the executor pointer the plugin reinstalls.

with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with System;

package Pcl_Transport_Plugin is

   --  Resolve the transport plugin path: explicit Path when non-empty,
   --  otherwise the PCL_TRANSPORT_PLUGIN environment variable. Returns the
   --  empty string when neither is set.
   function Resolve_Path (Path : String) return String;

   --  Load the socket transport plugin and install it on Exec as a client
   --  connection to Host:Port. Returns True on success, filling Handle and
   --  Vtable for later teardown via Destroy.
   function Load_Socket_Client
     (Plugin_Path : String;
      Host        : Interfaces.C.Strings.chars_ptr;
      Port        : Interfaces.C.unsigned_short;
      Exec        : Pcl_Bindings.Pcl_Executor_Access;
      Handle      : out System.Address;
      Vtable      : out Pcl_Bindings.Pcl_Transport_Const_Access)
      return Boolean;

   --  Destroy the loaded transport (frees the socket transport object) and
   --  unload the plugin. NULL-safe.
   procedure Destroy
     (Handle : System.Address;
      Vtable : Pcl_Bindings.Pcl_Transport_Const_Access);

end Pcl_Transport_Plugin;
