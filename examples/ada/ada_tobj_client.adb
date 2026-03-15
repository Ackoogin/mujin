-- ada_tobj_client.adb
-- Standalone Ada executable that connects to a TacticalObjectsComponent server
-- via TCP socket transport, subscribes to entity_updates, receives and decodes
-- entity update batch frames.
--
-- Uses the EntityActions-aligned service interface (tactical_objects_service)
-- generated from the proto/ada codex.  No bare JSON strings or PCL service
-- name literals appear in this file.
--
-- Usage: ada_tobj_client --host 127.0.0.1 --port 19123

with Ada.Command_Line;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Streaming_Codec;
with Tactical_Objects_Types;   use Tactical_Objects_Types;
with Tactical_Objects_Service;
with System;

procedure Ada_Tobj_Client is
  use type Interfaces.C.unsigned;
  use type Interfaces.C.unsigned_short;
  use type Interfaces.C.double;
  use type Pcl_Bindings.Pcl_Status;
  use type Pcl_Bindings.Pcl_Executor_Access;
  use type Pcl_Bindings.Pcl_Container_Access;
  use type Pcl_Bindings.Pcl_Socket_Transport_Access;
  use type Pcl_Bindings.Pcl_Port_Access;
  use type Streaming_Codec.Byte;
  use type System.Address;

  function To_Address is new
    Ada.Unchecked_Conversion(Interfaces.C.Strings.chars_ptr, System.Address);

  -- ── Configuration ──────────────────────────────────────────────────────────

  Host_Str : Interfaces.C.Strings.chars_ptr :=
    Interfaces.C.Strings.New_String("127.0.0.1");
  Port_Val : Interfaces.C.unsigned_short := 19000;

  procedure Parse_Args is
    use Ada.Command_Line;
    I : Natural := 1;
  begin
    while I <= Argument_Count loop
      if Argument(I) = "--host" and then I + 1 <= Argument_Count then
        Interfaces.C.Strings.Free(Host_Str);
        Host_Str := Interfaces.C.Strings.New_String(Argument(I + 1));
        I := I + 2;
      elsif Argument(I) = "--port" and then I + 1 <= Argument_Count then
        Port_Val := Interfaces.C.unsigned_short'Value(Argument(I + 1));
        I := I + 2;
      else
        I := I + 1;
      end if;
    end loop;
  end Parse_Args;

  -- ── Logging ────────────────────────────────────────────────────────────────

  procedure Log(Msg : String) is
  begin
    Ada.Text_IO.Put_Line(Ada.Text_IO.Standard_Error,
                         "[ada_tobj_client] " & Msg);
    Ada.Text_IO.Flush(Ada.Text_IO.Standard_Error);
  end Log;

  -- ── Client state ───────────────────────────────────────────────────────────

  Frames_Received : Natural := 0;

  -- ── Subscriber callback ────────────────────────────────────────────────────
  --
  --  Receives raw binary batch frames on the entity_updates topic.
  --  Decodes with Streaming_Codec then maps each frame to a typed
  --  TacticalObject via Tactical_Objects_Service.Frame_To_Tactical_Object.

  procedure Entity_Updates_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Entity_Updates_Cb);

  procedure Entity_Updates_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address)
  is
    pragma Unreferenced(Container, User_Data);
    Result : Streaming_Codec.Decode_Result;
  begin
    if Msg.Data = System.Null_Address or else Msg.Size = 0 then
      return;
    end if;

    Result := Streaming_Codec.Decode_Batch(Msg.Data, Msg.Size);
    Frames_Received := Frames_Received + Result.Count;

    if Result.Count > 0 then
      Log("Received batch with" & Natural'Image(Result.Count) & " entities");

      --  Map first frame to a typed TacticalObject and log it
      declare
        Obj : constant Tactical_Object :=
          Tactical_Objects_Service.Frame_To_Tactical_Object(
            Result.Frames(0));
      begin
        Log("  entity: " & Tactical_Objects_Service.Tactical_Object_Image(Obj));
      end;
    end if;
  end Entity_Updates_Cb;

  -- ── on_configure: subscribe to entity_updates ─────────────────────────────

  function On_Configure_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, On_Configure_Cb);

  function On_Configure_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status
  is
    Topic  : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("entity_updates");
    Type_N : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("application/octet-stream");
    Port   : Pcl_Bindings.Pcl_Port_Access;
    pragma Unreferenced(Port);
  begin
    Port := Pcl_Bindings.Add_Subscriber(
      Container => Container,
      Topic     => Topic,
      Type_Name => Type_N,
      Callback  => Entity_Updates_Cb'Unrestricted_Access,
      User_Data => User_Data);
    Interfaces.C.Strings.Free(Topic);
    Interfaces.C.Strings.Free(Type_N);
    return Pcl_Bindings.PCL_OK;
  end On_Configure_Cb;

  -- ── Main variables ─────────────────────────────────────────────────────────

  Exec      : Pcl_Bindings.Pcl_Executor_Access;
  Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
  Container : Pcl_Bindings.Pcl_Container_Access;
  Name_C    : Interfaces.C.Strings.chars_ptr;
  Cbs       : aliased Pcl_Bindings.Pcl_Callbacks;
  Status    : Pcl_Bindings.Pcl_Status;

begin
  Parse_Args;
  Log("Connecting to " & Interfaces.C.Strings.Value(Host_Str) &
      ":" & Interfaces.C.unsigned_short'Image(Port_Val));

  -- ── Create executor ────────────────────────────────────────────────────────

  Exec := Pcl_Bindings.Create_Executor;
  if Exec = null then
    Log("FAIL: could not create executor");
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- ── Connect via socket client transport ────────────────────────────────────

  Transport := Pcl_Bindings.Create_Socket_Client(Host_Str, Port_Val, Exec);
  Interfaces.C.Strings.Free(Host_Str);

  if Transport = null then
    Log("FAIL: could not connect to server");
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- ── Set transport on executor ──────────────────────────────────────────────

  Status := Pcl_Bindings.Set_Transport(
    Exec, Pcl_Bindings.Get_Socket_Transport(Transport));
  if Status /= Pcl_Bindings.PCL_OK then
    Log("FAIL: could not set transport (status=" &
        Pcl_Bindings.Pcl_Status'Image(Status) & ")");
    Pcl_Bindings.Destroy_Socket_Transport(Transport);
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- ── Create subscriber container ────────────────────────────────────────────

  Cbs := (On_Configure  => On_Configure_Cb'Unrestricted_Access,
          On_Activate   => null,
          On_Deactivate => null,
          On_Cleanup    => null,
          On_Shutdown   => null,
          On_Tick       => null);
  Name_C := Interfaces.C.Strings.New_String("ada_tobj_client");
  Container := Pcl_Bindings.Create_Container(Name_C, Cbs'Access,
                                              System.Null_Address);
  Interfaces.C.Strings.Free(Name_C);

  if Container = null then
    Log("FAIL: could not create container");
    Pcl_Bindings.Destroy_Socket_Transport(Transport);
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  Status := Pcl_Bindings.Configure(Container);
  Status := Pcl_Bindings.Activate(Container);
  Status := Pcl_Bindings.Add_Container(Exec, Container);

  -- ── Invoke EntityActions Read via subscribe_interest (async) ──────────────
  --
  --  Build a typed TacticalObjectQuery (from the proto IDL) then serialise
  --  it to JSON.  The service name is taken from the service stub constant —
  --  no bare strings in call sites.
  --
  --  The call is fully async: invoke_remote_async enqueues the SVC_REQ to
  --  the send_thread and returns immediately.  The response callback fires on
  --  the PCL executor thread during the spin loop below.

  declare
    Svc_Response_Ready : Boolean                   := False;
    Svc_Response_Size  : Interfaces.C.unsigned     := 0;

    procedure Svc_Response_Cb
      (Resp      : access constant Pcl_Bindings.Pcl_Msg;
       User_Data : System.Address);
    pragma Convention(C, Svc_Response_Cb);

    procedure Svc_Response_Cb
      (Resp      : access constant Pcl_Bindings.Pcl_Msg;
       User_Data : System.Address)
    is
      pragma Unreferenced(User_Data);
    begin
      if Resp /= null then
        Svc_Response_Size := Resp.Size;
      end if;
      Svc_Response_Ready := True;
    end Svc_Response_Cb;

    Query : Tactical_Object_Query;
  begin
    Query.By_Type        := (Has   => True,
                             Value => Platform);
    Query.By_Affiliation := (Has   => True,
                             Value => Hostile);
    Query.By_Region      := (Has   => True,
                             Value => (Min_Lat => 50.0, Max_Lat => 52.0,
                                       Min_Lon => -1.0, Max_Lon =>  1.0));

    declare
      Req_Str : constant String :=
        Tactical_Objects_Service.Build_Read_Request_Json(Query);
      Req_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String(Req_Str);
      Svc_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String(
          Tactical_Objects_Service.Read_Service_Name);
      Req     : aliased Pcl_Bindings.Pcl_Msg;
    begin
      Log("Read request: " & Req_Str);

      Req.Data      := To_Address(Req_C);
      Req.Size      := Interfaces.C.unsigned(Req_Str'Length);
      Req.Type_Name := Interfaces.C.Strings.New_String("application/json");

      Status := Pcl_Bindings.Invoke_Remote_Async(
        Transport, Svc_C, Req'Access,
        Svc_Response_Cb'Unrestricted_Access, System.Null_Address);

      if Status /= Pcl_Bindings.PCL_OK then
        Log(Tactical_Objects_Service.Read_Service_Name &
            " async submit FAILED, status=" &
            Pcl_Bindings.Pcl_Status'Image(Status));
      end if;

      Interfaces.C.Strings.Free(Req_C);
      Interfaces.C.Strings.Free(Svc_C);
      Interfaces.C.Strings.Free(Req.Type_Name);
    end;

    --  Spin until the service response arrives, then continue to wait for
    --  entity_updates.  Both are drained through the same spin loop.
    Log("Spinning to receive service response and entity updates...");
    for Iteration in 1 .. 200 loop
      Status := Pcl_Bindings.Spin_Once(Exec, 0);
      if Svc_Response_Ready and then not (Status = Pcl_Bindings.PCL_OK) then
        null;
      end if;
      exit when Svc_Response_Ready and then Frames_Received > 0;
      delay 0.01;  -- 10 ms
    end loop;

    if Svc_Response_Ready then
      Log(Tactical_Objects_Service.Read_Service_Name &
          " OK, response size=" &
          Interfaces.C.unsigned'Image(Svc_Response_Size));
    else
      Log(Tactical_Objects_Service.Read_Service_Name & " TIMEOUT: no response");
    end if;
  end;

  -- ── Report pass/fail ──────────────────────────────────────────────────────

  if Frames_Received > 0 then
    Log("PASS: received" & Natural'Image(Frames_Received) & " entity frames");
    Ada.Command_Line.Set_Exit_Status(0);
  else
    Log("FAIL: no entity frames received");
    Ada.Command_Line.Set_Exit_Status(1);
  end if;

  -- ── Cleanup ───────────────────────────────────────────────────────────────

  Pcl_Bindings.Destroy_Socket_Transport(Transport);
  Pcl_Bindings.Destroy_Container(Container);
  Pcl_Bindings.Destroy_Executor(Exec);
end Ada_Tobj_Client;
