with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Interfaces.C.Strings;

package body Pcl_Process_Runtime is
  use type Interfaces.C.Strings.chars_ptr;
  use type Pcl_Bindings.Pcl_Status;
  use type System.Address;

  type C_Endpoint is record
    Name : Interfaces.C.Strings.chars_ptr;
    Kind : Pcl_Bindings.Pcl_Endpoint_Kind;
  end record;
  pragma Convention(C, C_Endpoint);

  type C_Endpoint_Array is array (Positive range <>) of aliased C_Endpoint;
  type C_Endpoint_Array_Access is access all C_Endpoint_Array;

  type C_Port is record
    Name                  : Interfaces.C.Strings.chars_ptr;
    Rpc_Endpoints         : System.Address;
    Rpc_Endpoint_Count    : Interfaces.C.size_t;
    Pubsub_Endpoints      : System.Address;
    Pubsub_Endpoint_Count : Interfaces.C.size_t;
  end record;
  pragma Convention(C, C_Port);

  type C_Port_Array is array (Positive range <>) of aliased C_Port;
  type C_Endpoint_Owners is
    array (Positive range <>) of C_Endpoint_Array_Access;

  procedure Free is new Ada.Unchecked_Deallocation
    (C_Endpoint_Array, C_Endpoint_Array_Access);

  function C_Create
    (Duration_Seconds : Interfaces.C.unsigned;
     Out_Runtime      : access System.Address)
      return Pcl_Bindings.Pcl_Status;
  pragma Import(C, C_Create, "pcl_process_runtime_create");

  function C_Executor
    (Self : System.Address) return Pcl_Bindings.Pcl_Executor_Access;
  pragma Import(C, C_Executor, "pcl_process_runtime_executor");

  function C_Load_Codec
    (Self        : System.Address;
     Plugin_Path : Interfaces.C.Strings.chars_ptr)
      return Pcl_Bindings.Pcl_Status;
  pragma Import(C, C_Load_Codec, "pcl_process_runtime_load_codec");

  function C_Load_Ports_File
    (Self        : System.Address;
     Config_Path : Interfaces.C.Strings.chars_ptr;
     Ports       : System.Address;
     Port_Count  : Interfaces.C.size_t)
      return Pcl_Bindings.Pcl_Status;
  pragma Import(C, C_Load_Ports_File, "pcl_process_runtime_load_ports_file");

  function C_Run
    (Self      : System.Address;
     Component : Pcl_Bindings.Pcl_Container_Access)
      return Pcl_Bindings.Pcl_Status;
  pragma Import(C, C_Run, "pcl_process_runtime_run");

  procedure C_Request_Shutdown(Self : System.Address);
  pragma Import
    (C, C_Request_Shutdown, "pcl_process_runtime_request_shutdown");

  function C_Error
    (Self : System.Address) return Interfaces.C.Strings.chars_ptr;
  pragma Import(C, C_Error, "pcl_process_runtime_error");

  procedure C_Destroy(Self : System.Address);
  pragma Import(C, C_Destroy, "pcl_process_runtime_destroy");

  procedure Require_Ok
    (Self   : Runtime;
     Status : Pcl_Bindings.Pcl_Status) is
  begin
    if Status /= Pcl_Bindings.PCL_OK then
      raise Program_Error with Error(Self);
    end if;
  end Require_Ok;

  function Make_Endpoint
    (Name : String;
     Kind : Pcl_Bindings.Pcl_Endpoint_Kind) return Deployment_Endpoint is
  begin
    return (Name => To_Unbounded_String(Name), Kind => Kind);
  end Make_Endpoint;

  function Make_Port
    (Name             : String;
     Rpc_Endpoints    : Endpoint_Array;
     Pubsub_Endpoints : Endpoint_Array) return Deployment_Port is
  begin
    return
      (Name             => To_Unbounded_String(Name),
       Rpc_Endpoints    => new Endpoint_Array'(Rpc_Endpoints),
       Pubsub_Endpoints => new Endpoint_Array'(Pubsub_Endpoints));
  end Make_Port;

  procedure Create
    (Self             : in out Runtime;
     Duration_Seconds : Interfaces.C.unsigned := 0) is
    Handle : aliased System.Address := System.Null_Address;
    Status : Pcl_Bindings.Pcl_Status;
  begin
    if Self.Handle /= System.Null_Address then
      raise Program_Error with "process runtime is already created";
    end if;
    Status := C_Create(Duration_Seconds, Handle'Access);
    if Status /= Pcl_Bindings.PCL_OK then
      raise Program_Error with "cannot create process runtime";
    end if;
    Self.Handle := Handle;
  end Create;

  function Executor
    (Self : Runtime) return Pcl_Bindings.Pcl_Executor_Access is
  begin
    return C_Executor(Self.Handle);
  end Executor;

  procedure Load_Codec(Self : in out Runtime; Plugin_Path : String) is
    Path   : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String(Plugin_Path);
    Status : Pcl_Bindings.Pcl_Status;
  begin
    Status := C_Load_Codec(Self.Handle, Path);
    Interfaces.C.Strings.Free(Path);
    Require_Ok(Self, Status);
  end Load_Codec;

  procedure Load_Ports_File
    (Self        : in out Runtime;
     Config_Path : String;
     Ports       : Port_Array) is
    Path       : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String(Config_Path);
    C_Ports    : aliased C_Port_Array(Ports'Range);
    Rpc_Owners : C_Endpoint_Owners(Ports'Range) := (others => null);
    Pub_Owners : C_Endpoint_Owners(Ports'Range) := (others => null);
    Status     : Pcl_Bindings.Pcl_Status;
  begin
    for Index in Ports'Range loop
      C_Ports(Index).Name :=
        Interfaces.C.Strings.New_String(To_String(Ports(Index).Name));
      if Ports(Index).Rpc_Endpoints /= null
        and then Ports(Index).Rpc_Endpoints'Length > 0
      then
        Rpc_Owners(Index) :=
          new C_Endpoint_Array(Ports(Index).Rpc_Endpoints'Range);
        for Endpoint_Index in Ports(Index).Rpc_Endpoints'Range loop
          Rpc_Owners(Index)(Endpoint_Index) :=
            (Name => Interfaces.C.Strings.New_String
               (To_String(Ports(Index).Rpc_Endpoints(Endpoint_Index).Name)),
             Kind => Ports(Index).Rpc_Endpoints(Endpoint_Index).Kind);
        end loop;
        C_Ports(Index).Rpc_Endpoints :=
          Rpc_Owners(Index)(Rpc_Owners(Index)'First)'Address;
        C_Ports(Index).Rpc_Endpoint_Count :=
          Interfaces.C.size_t(Rpc_Owners(Index)'Length);
      else
        C_Ports(Index).Rpc_Endpoints := System.Null_Address;
        C_Ports(Index).Rpc_Endpoint_Count := 0;
      end if;
      if Ports(Index).Pubsub_Endpoints /= null
        and then Ports(Index).Pubsub_Endpoints'Length > 0
      then
        Pub_Owners(Index) :=
          new C_Endpoint_Array(Ports(Index).Pubsub_Endpoints'Range);
        for Endpoint_Index in Ports(Index).Pubsub_Endpoints'Range loop
          Pub_Owners(Index)(Endpoint_Index) :=
            (Name => Interfaces.C.Strings.New_String
               (To_String(Ports(Index).Pubsub_Endpoints(Endpoint_Index).Name)),
             Kind => Ports(Index).Pubsub_Endpoints(Endpoint_Index).Kind);
        end loop;
        C_Ports(Index).Pubsub_Endpoints :=
          Pub_Owners(Index)(Pub_Owners(Index)'First)'Address;
        C_Ports(Index).Pubsub_Endpoint_Count :=
          Interfaces.C.size_t(Pub_Owners(Index)'Length);
      else
        C_Ports(Index).Pubsub_Endpoints := System.Null_Address;
        C_Ports(Index).Pubsub_Endpoint_Count := 0;
      end if;
    end loop;

    Status := C_Load_Ports_File
      (Self.Handle, Path, C_Ports(C_Ports'First)'Address,
       Interfaces.C.size_t(C_Ports'Length));

    Interfaces.C.Strings.Free(Path);
    for Index in Ports'Range loop
      Interfaces.C.Strings.Free(C_Ports(Index).Name);
      if Rpc_Owners(Index) /= null then
        for Endpoint_Index in Rpc_Owners(Index)'Range loop
          Interfaces.C.Strings.Free(Rpc_Owners(Index)(Endpoint_Index).Name);
        end loop;
        Free(Rpc_Owners(Index));
      end if;
      if Pub_Owners(Index) /= null then
        for Endpoint_Index in Pub_Owners(Index)'Range loop
          Interfaces.C.Strings.Free(Pub_Owners(Index)(Endpoint_Index).Name);
        end loop;
        Free(Pub_Owners(Index));
      end if;
    end loop;
    Require_Ok(Self, Status);
  end Load_Ports_File;

  function Run
    (Self      : in out Runtime;
     Component : Pcl_Bindings.Pcl_Container_Access)
      return Pcl_Bindings.Pcl_Status is
  begin
    return C_Run(Self.Handle, Component);
  end Run;

  procedure Request_Shutdown(Self : in out Runtime) is
  begin
    C_Request_Shutdown(Self.Handle);
  end Request_Shutdown;

  function Error(Self : Runtime) return String is
    Message : constant Interfaces.C.Strings.chars_ptr := C_Error(Self.Handle);
  begin
    if Message = Interfaces.C.Strings.Null_Ptr then
      return "";
    end if;
    return Interfaces.C.Strings.Value(Message);
  end Error;

  procedure Destroy(Self : in out Runtime) is
  begin
    C_Destroy(Self.Handle);
    Self.Handle := System.Null_Address;
  end Destroy;
end Pcl_Process_Runtime;
