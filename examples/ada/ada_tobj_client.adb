-- ada_tobj_client.adb
-- Standalone Ada executable that connects to a TacticalObjectsComponent server
-- via TCP socket transport, subscribes to entity_updates, invokes
-- subscribe_interest, receives and decodes entity update batch frames.
--
-- Usage: ada_tobj_client --host 127.0.0.1 --port 19123

with Ada.Command_Line;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Streaming_Codec;
with System;

procedure Ada_Tobj_Client is
  use type Interfaces.C.unsigned;
  use type Interfaces.C.unsigned_short;
  use type Pcl_Bindings.Pcl_Status;
  use type Pcl_Bindings.Pcl_Executor_Access;
  use type Pcl_Bindings.Pcl_Container_Access;
  use type Pcl_Bindings.Pcl_Socket_Transport_Access;
  use type Pcl_Bindings.Pcl_Port_Access;
  use type Streaming_Codec.Byte;
  use type System.Address;

  function To_Address is new Ada.Unchecked_Conversion(Interfaces.C.Strings.chars_ptr, System.Address);

  -- ── Configuration ──────────────────────────────────────────────────────

  Host_Str : String := "127.0.0.1";
  Port_Val : Interfaces.C.unsigned_short := 19000;

  procedure Parse_Args is
    use Ada.Command_Line;
    I : Natural := 1;
  begin
    while I <= Argument_Count loop
      if Argument(I) = "--host" and then I + 1 <= Argument_Count then
        -- Ada strings are immutable length; use a helper
        declare
          H : constant String := Argument(I + 1);
        begin
          Host_Str(Host_Str'First .. Host_Str'First + H'Length - 1) :=
            H(H'First .. H'Last);
          -- This is simplified; real impl would use Unbounded_String
          null;
        end;
        I := I + 2;
      elsif Argument(I) = "--port" and then I + 1 <= Argument_Count then
        Port_Val := Interfaces.C.unsigned_short'Value(Argument(I + 1));
        I := I + 2;
      else
        I := I + 1;
      end if;
    end loop;
  end Parse_Args;

  -- ── Logging ────────────────────────────────────────────────────────────

  procedure Log(Msg : String) is
  begin
    Ada.Text_IO.Put_Line(Ada.Text_IO.Standard_Error, "[ada_tobj_client] " & Msg);
    Ada.Text_IO.Flush(Ada.Text_IO.Standard_Error);
  end Log;

  -- ── Client state ───────────────────────────────────────────────────────

  Frames_Received : Natural := 0;

  -- ── Callbacks ──────────────────────────────────────────────────────────

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
    pragma Unreferenced(Container);
    pragma Unreferenced(User_Data);
    Result : Streaming_Codec.Decode_Result;
  begin
    if Msg.Data = System.Null_Address or else Msg.Size = 0 then
      return;
    end if;
    Result := Streaming_Codec.Decode_Batch(Msg.Data, Msg.Size);
    Frames_Received := Frames_Received + Result.Count;
    if Result.Count > 0 then
      Log("Received batch with" & Natural'Image(Result.Count) & " entities");
      -- Print first entity details
      declare
        F : constant Streaming_Codec.Entity_Update_Frame :=
          Result.Frames(0);
      begin
        Log("  entity msg_type=" & Streaming_Codec.Byte'Image(F.Message_Type));
        if F.Has_Position then
          Log("  position: lat=" &
              Interfaces.C.double'Image(F.Pos.Lat) & " lon=" &
              Interfaces.C.double'Image(F.Pos.Lon));
        end if;
        if F.Has_Affiliation then
          Log("  affiliation ordinal=" &
              Streaming_Codec.Byte'Image(F.Affiliation_Val));
        end if;
      end;
    end if;
  end Entity_Updates_Cb;

  function On_Configure_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, On_Configure_Cb);

  function On_Configure_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status
  is
    Topic : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("entity_updates");
    Type_N : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("application/octet-stream");
    Port : Pcl_Bindings.Pcl_Port_Access;
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

  -- ── Main ───────────────────────────────────────────────────────────────

  Exec      : Pcl_Bindings.Pcl_Executor_Access;
  Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
  Container : Pcl_Bindings.Pcl_Container_Access;
  Host_C    : Interfaces.C.Strings.chars_ptr;
  Name_C    : Interfaces.C.Strings.chars_ptr;
  Cbs       : aliased Pcl_Bindings.Pcl_Callbacks;
  Status    : Pcl_Bindings.Pcl_Status;

begin
  Parse_Args;
  Log("Connecting to " & Host_Str & ":" &
      Interfaces.C.unsigned_short'Image(Port_Val));

  -- Create executor
  Exec := Pcl_Bindings.Create_Executor;
  if Exec = null then
    Log("FAIL: could not create executor");
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- Connect via socket transport
  Host_C := Interfaces.C.Strings.New_String(Host_Str);
  Transport := Pcl_Bindings.Create_Socket_Client(Host_C, Port_Val, Exec);
  Interfaces.C.Strings.Free(Host_C);

  if Transport = null then
    Log("FAIL: could not connect to server");
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- Set transport on executor
  Status := Pcl_Bindings.Set_Transport(
    Exec, Pcl_Bindings.Get_Socket_Transport(Transport));
  if Status /= Pcl_Bindings.PCL_OK then
    Log("FAIL: could not set transport");
    Pcl_Bindings.Destroy_Socket_Transport(Transport);
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- Create subscriber container
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

  -- Invoke remote subscribe_interest
  declare
    Req_Str : constant String :=
      "{""object_type"":""Platform"",""affiliation"":""Hostile""," &
      """area"":{""min_lat"":50,""max_lat"":52,""min_lon"":-1,""max_lon"":1}," &
      """expires_at"":9999}";
    Req_C : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String(Req_Str);
    Svc_C : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("subscribe_interest");
    Req : aliased Pcl_Bindings.Pcl_Msg;
    Resp_Buf : aliased String (1 .. 512) := (others => ' ');
    Resp : aliased Pcl_Bindings.Pcl_Msg;
  begin
    Req.Data := To_Address(Req_C);
    Req.Size := Interfaces.C.unsigned(Req_Str'Length);
    Req.Type_Name := Interfaces.C.Strings.New_String("application/json");

    Resp.Data := Resp_Buf(1)'Address;
    Resp.Size := Interfaces.C.unsigned(Resp_Buf'Length);
    Resp.Type_Name := Interfaces.C.Strings.Null_Ptr;

    Status := Pcl_Bindings.Invoke_Remote(
      Transport, Svc_C, Req'Access, Resp'Access);

    if Status = Pcl_Bindings.PCL_OK then
      Log("subscribe_interest OK, response size=" &
          Interfaces.C.unsigned'Image(Resp.Size));
    else
      Log("subscribe_interest FAILED, status=" &
          Pcl_Bindings.Pcl_Status'Image(Status));
    end if;

    Interfaces.C.Strings.Free(Req_C);
    Interfaces.C.Strings.Free(Svc_C);
    Interfaces.C.Strings.Free(Req.Type_Name);
  end;

  -- Spin to receive entity updates (up to 200 iterations or until received)
  Log("Spinning to receive entity updates...");
  for Iteration in 1 .. 200 loop
    Status := Pcl_Bindings.Spin_Once(Exec, 0);
    exit when Frames_Received > 0;
    delay 0.01;  -- 10ms
  end loop;

  -- Report
  if Frames_Received > 0 then
    Log("PASS: received" & Natural'Image(Frames_Received) & " entity frames");
    Ada.Command_Line.Set_Exit_Status(0);
  else
    Log("FAIL: no entity frames received");
    Ada.Command_Line.Set_Exit_Status(1);
  end if;

  -- Cleanup
  Pcl_Bindings.Destroy_Socket_Transport(Transport);
  Pcl_Bindings.Destroy_Container(Container);
  Pcl_Bindings.Destroy_Executor(Exec);
end Ada_Tobj_Client;
