--  Process-level PCL lifecycle, codec, and deployment routing runtime.

with Ada.Strings.Unbounded;
with Interfaces.C;
with Pcl_Bindings;
with System;

package Pcl_Process_Runtime is
  type Deployment_Endpoint is record
    Name : Ada.Strings.Unbounded.Unbounded_String;
    Kind : Pcl_Bindings.Pcl_Endpoint_Kind;
  end record;

  type Endpoint_Array is array (Positive range <>) of Deployment_Endpoint;
  type Endpoint_Array_Access is access all Endpoint_Array;
  No_Endpoints : constant Endpoint_Array (1 .. 0) := (others => <>);

  type Deployment_Port is record
    Name             : Ada.Strings.Unbounded.Unbounded_String;
    Rpc_Endpoints    : Endpoint_Array_Access;
    Pubsub_Endpoints : Endpoint_Array_Access;
  end record;
  type Port_Array is array (Positive range <>) of Deployment_Port;

  function Make_Endpoint
    (Name : String;
     Kind : Pcl_Bindings.Pcl_Endpoint_Kind) return Deployment_Endpoint;

  function Make_Port
    (Name             : String;
     Rpc_Endpoints    : Endpoint_Array;
     Pubsub_Endpoints : Endpoint_Array) return Deployment_Port;

  type Runtime is limited private;

  procedure Create
    (Self             : in out Runtime;
     Duration_Seconds : Interfaces.C.unsigned := 0);
  function Executor
    (Self : Runtime) return Pcl_Bindings.Pcl_Executor_Access;
  procedure Load_Codec(Self : in out Runtime; Plugin_Path : String);
  procedure Load_Ports_File
    (Self        : in out Runtime;
     Config_Path : String;
     Ports       : Port_Array);
  function Run
    (Self      : in out Runtime;
     Component : Pcl_Bindings.Pcl_Container_Access)
      return Pcl_Bindings.Pcl_Status;
  procedure Request_Shutdown(Self : in out Runtime);
  function Error(Self : Runtime) return String;
  procedure Destroy(Self : in out Runtime);

private
  type Runtime is limited record
    Handle : System.Address := System.Null_Address;
  end record;
end Pcl_Process_Runtime;
