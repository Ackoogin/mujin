with Ada.Environment_Variables;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Unchecked_Conversion;
with Interfaces;
with Pcl_Plugins;
with System.Storage_Elements;

package body Pcl_Transport_Plugin is

   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Bindings.Pcl_Transport_Const_Access;
   use type System.Address;

   type Transport_Destroy_Access is access procedure
     (T : Pcl_Bindings.Pcl_Transport_Const_Access);
   pragma Convention (C, Transport_Destroy_Access);

   function To_Destroy is new Ada.Unchecked_Conversion
     (System.Address, Transport_Destroy_Access);

   function Trim (S : String) return String is
   begin
      return Ada.Strings.Fixed.Trim (S, Ada.Strings.Both);
   end Trim;

   function Resolve_Path (Path : String) return String is
   begin
      if Path'Length > 0 then
         return Path;
      end if;
      if Ada.Environment_Variables.Exists ("PCL_TRANSPORT_PLUGIN") then
         return Ada.Environment_Variables.Value ("PCL_TRANSPORT_PLUGIN");
      end if;
      return "";
   end Resolve_Path;

   function Config_JSON
     (Host : Interfaces.C.Strings.chars_ptr;
      Port : Interfaces.C.unsigned_short;
      Exec : Pcl_Bindings.Pcl_Executor_Access) return String
   is
      Exec_Int : constant Interfaces.Integer_64 :=
        Interfaces.Integer_64
          (System.Storage_Elements.To_Integer (Exec.all'Address));
   begin
      return "{""role"":""client"",""host"":""" &
             Interfaces.C.Strings.Value (Host) & """,""port"":" &
             Trim (Interfaces.C.unsigned_short'Image (Port)) &
             ",""executor"":" &
             Trim (Interfaces.Integer_64'Image (Exec_Int)) & "}";
   end Config_JSON;

   function Load_Socket_Client
     (Plugin_Path : String;
      Host        : Interfaces.C.Strings.chars_ptr;
      Port        : Interfaces.C.unsigned_short;
      Exec        : Pcl_Bindings.Pcl_Executor_Access;
      Handle      : out System.Address;
      Vtable      : out Pcl_Bindings.Pcl_Transport_Const_Access)
      return Boolean
   is
      Path_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Plugin_Path);
      Config_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Config_JSON (Host, Port, Exec));
      H        : aliased System.Address := System.Null_Address;
      V        : aliased Pcl_Bindings.Pcl_Transport_Const_Access := null;
      Status   : Pcl_Bindings.Pcl_Status;
   begin
      Handle := System.Null_Address;
      Vtable := null;
      Status := Pcl_Plugins.Pcl_Plugin_Load_Transport
        (Path_C, Config_C, H'Access, V'Access);
      Interfaces.C.Strings.Free (Path_C);
      Interfaces.C.Strings.Free (Config_C);
      if Status /= Pcl_Bindings.PCL_OK or else V = null then
         return False;
      end if;
      Handle := H;
      Vtable := V;
      return True;
   end Load_Socket_Client;

   procedure Destroy
     (Handle : System.Address;
      Vtable : Pcl_Bindings.Pcl_Transport_Const_Access)
   is
      Sym_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String
          ("pcl_socket_transport_plugin_destroy");
      Destroy_Sym : System.Address;
   begin
      if Handle /= System.Null_Address and then Vtable /= null then
         Destroy_Sym := Pcl_Plugins.Pcl_Plugin_Symbol (Handle, Sym_C);
         if Destroy_Sym /= System.Null_Address then
            To_Destroy (Destroy_Sym).all (Vtable);
         end if;
      end if;
      Interfaces.C.Strings.Free (Sym_C);
      if Handle /= System.Null_Address then
         Pcl_Plugins.Pcl_Plugin_Unload (Handle);
      end if;
   end Destroy;

end Pcl_Transport_Plugin;
