--  Thin Ada binding for the PCL runtime plugin and codec registry ABI.
--
--  This package mirrors the C declarations in:
--
--   * ``pcl/pcl_codec.h``
--   * ``pcl/pcl_codec_registry.h``
--   * ``pcl/pcl_plugin_loader.h``

with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with System;

package Pcl_Plugins is
  Pcl_Codec_Abi_Version     : constant Interfaces.C.unsigned := 2;
  Pcl_Transport_Abi_Version : constant Interfaces.C.unsigned := 1;

  type Pcl_Codec_Encode_Access is access function
    (Codec_Ctx : System.Address;
     Schema_Id : Interfaces.C.Strings.chars_ptr;
     Value     : System.Address;
     Out_Msg   : access Pcl_Bindings.Pcl_Msg) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Pcl_Codec_Encode_Access);

  type Pcl_Codec_Decode_Access is access function
    (Codec_Ctx : System.Address;
     Schema_Id : Interfaces.C.Strings.chars_ptr;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     Out_Value : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Pcl_Codec_Decode_Access);

  type Pcl_Codec_Free_Msg_Access is access procedure
    (Codec_Ctx : System.Address;
     Msg       : access Pcl_Bindings.Pcl_Msg);
  pragma Convention(C, Pcl_Codec_Free_Msg_Access);

  type Pcl_Codec is record
    Abi_Version  : Interfaces.C.unsigned;
    Content_Type : Interfaces.C.Strings.chars_ptr;
    Encode       : Pcl_Codec_Encode_Access;
    Decode       : Pcl_Codec_Decode_Access;
    Free_Msg     : Pcl_Codec_Free_Msg_Access;
    Codec_Ctx    : System.Address;
  end record;
  pragma Convention(C, Pcl_Codec);

  type Pcl_Codec_Const_Access is access constant Pcl_Codec;
  pragma Convention(C, Pcl_Codec_Const_Access);

  function Pcl_Codec_Registry_Create return System.Address;
  pragma Import(C, Pcl_Codec_Registry_Create,
                "pcl_codec_registry_create");

  procedure Pcl_Codec_Registry_Destroy(Registry : System.Address);
  pragma Import(C, Pcl_Codec_Registry_Destroy,
                "pcl_codec_registry_destroy");

  function Pcl_Codec_Registry_Get
    (Registry     : System.Address;
     Content_Type : Interfaces.C.Strings.chars_ptr)
      return Pcl_Codec_Const_Access;
  pragma Import(C, Pcl_Codec_Registry_Get,
                "pcl_codec_registry_get");

  function Pcl_Codec_Registry_Count
    (Registry : System.Address) return Interfaces.C.unsigned;
  pragma Import(C, Pcl_Codec_Registry_Count,
                "pcl_codec_registry_count");

  function Pcl_Codec_Registry_Default return System.Address;
  pragma Import(C, Pcl_Codec_Registry_Default,
                "pcl_codec_registry_default");

  function Pcl_Plugin_Load_Codec
    (Path        : Interfaces.C.Strings.chars_ptr;
     Config_Json : Interfaces.C.Strings.chars_ptr;
     Registry    : System.Address;
     Out_Handle  : access System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Import(C, Pcl_Plugin_Load_Codec,
                "pcl_plugin_load_codec");

  --  Load a transport plugin and return its borrowed transport vtable.
  --  Config_Json is opaque, plugin-specific configuration (uniform with the
  --  codec plugin loader); for the socket transport plugin it carries role,
  --  host, port and the executor pointer.
  function Pcl_Plugin_Load_Transport
    (Path        : Interfaces.C.Strings.chars_ptr;
     Config_Json : Interfaces.C.Strings.chars_ptr;
     Out_Handle  : access System.Address;
     Out_Vtable  : access Pcl_Bindings.Pcl_Transport_Const_Access)
      return Pcl_Bindings.Pcl_Status;
  pragma Import(C, Pcl_Plugin_Load_Transport,
                "pcl_plugin_load_transport");

  --  Load codec plugins listed in a path-list environment variable (split on
  --  ':' POSIX / ';' Windows). Missing vars and individual missing/invalid
  --  paths are skipped. Returns PCL_OK when the variable is unset/empty.
  function Pcl_Codec_Registry_Load_Plugins_From_Env
    (Registry : System.Address;
     Env_Var  : Interfaces.C.Strings.chars_ptr) return Pcl_Bindings.Pcl_Status;
  pragma Import(C, Pcl_Codec_Registry_Load_Plugins_From_Env,
                "pcl_codec_registry_load_plugins_from_env");

  --  Open an arbitrary shared library (no PCL entry point required): a
  --  portable wrapper over dlopen / LoadLibrary for libraries that export
  --  plain C symbols (e.g. the generated gRPC C shim). Returns Null_Address
  --  on failure. Resolve symbols with Pcl_Plugin_Symbol; release with
  --  Pcl_Plugin_Unload.
  function Pcl_Plugin_Open
    (Path : Interfaces.C.Strings.chars_ptr) return System.Address;
  pragma Import(C, Pcl_Plugin_Open, "pcl_plugin_open");

  procedure Pcl_Plugin_Unload(Handle : System.Address);
  pragma Import(C, Pcl_Plugin_Unload, "pcl_plugin_unload");

  function Pcl_Plugin_Unload_Transport
    (Handle : System.Address;
     Vtable : Pcl_Bindings.Pcl_Transport_Const_Access)
      return Pcl_Bindings.Pcl_Status;
  pragma Import(C, Pcl_Plugin_Unload_Transport,
                "pcl_plugin_unload_transport");

  function Pcl_Plugin_Symbol
    (Handle : System.Address;
     Name   : Interfaces.C.Strings.chars_ptr) return System.Address;
  pragma Import(C, Pcl_Plugin_Symbol, "pcl_plugin_symbol");
end Pcl_Plugins;
