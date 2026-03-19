-- ada_active_find_e2e.adb
-- Multi-process Ada E2E test for the ActiveFind flow.
--
-- Two PCL containers run in a single Ada process, connected to a
-- TacticalObjectsComponent server via TCP socket transport:
--
--   1. Ada Client — subscribes to entity_updates, invokes subscribe_interest
--      with query_mode=active_find and battle_dimension=SeaSurface.
--   2. Evidence Provider — subscribes to evidence_requirements, and on
--      receipt publishes an observation batch to observation_ingress that
--      satisfies the requirement (SeaSurface Hostile Platform).
--
-- The server (tobj_socket_server --no-entity) runs the
-- TacticalObjectsComponent which processes the interest, derives an
-- evidence requirement, correlates the incoming observation to create
-- a tracked entity, and streams entity updates back to the client.
--
-- Usage: ada_active_find_e2e --host 127.0.0.1 --port 19123

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
with System.Storage_Elements;

procedure Ada_Active_Find_E2E is
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

  -- -- Configuration ----------------------------------------------------------

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

  -- -- Logging ----------------------------------------------------------------

  procedure Log(Msg : String) is
  begin
    Ada.Text_IO.Put_Line(Ada.Text_IO.Standard_Error,
                         "[ada_active_find] " & Msg);
    Ada.Text_IO.Flush(Ada.Text_IO.Standard_Error);
  end Log;

  -- -- Shared state -----------------------------------------------------------

  Frames_Received       : Natural := 0;
  Found_Hostile_Platform: Boolean := False;
  Evidence_Req_Received : Boolean := False;
  Observation_Sent      : Boolean := False;

  -- Store executor handle for evidence provider to publish through
  Exec_Handle : Pcl_Bindings.Pcl_Executor_Access := null;

  -- ═══════════════════════════════════════════════════════════════════════════
  -- Container 1: Ada Client — subscribes to entity_updates
  -- ═══════════════════════════════════════════════════════════════════════════

  procedure Client_Entity_Updates_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Client_Entity_Updates_Cb);

  procedure Client_Entity_Updates_Cb
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
      for I in 0 .. Result.Count - 1 loop
        declare
          Obj : constant Tactical_Object :=
            Tactical_Objects_Service.Frame_To_Tactical_Object(
              Result.Frames(I));
        begin
          Log("  entity: " &
              Tactical_Objects_Service.Tactical_Object_Image(Obj));
          if Obj.Obj_Type = Platform and then Obj.Affil = Hostile then
            Found_Hostile_Platform := True;
          end if;
        end;
      end loop;
    end if;
  end Client_Entity_Updates_Cb;

  function Client_On_Configure
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Client_On_Configure);

  function Client_On_Configure
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
      Callback  => Client_Entity_Updates_Cb'Unrestricted_Access,
      User_Data => User_Data);
    Interfaces.C.Strings.Free(Topic);
    Interfaces.C.Strings.Free(Type_N);
    return Pcl_Bindings.PCL_OK;
  end Client_On_Configure;

  -- ═══════════════════════════════════════════════════════════════════════════
  -- Container 2: Evidence Provider — subscribes to evidence_requirements
  -- ═══════════════════════════════════════════════════════════════════════════

  procedure Provider_Evidence_Req_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Provider_Evidence_Req_Cb);

  procedure Provider_Evidence_Req_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address)
  is
    pragma Unreferenced(Container, User_Data);
  begin
    if Msg.Data = System.Null_Address or else Msg.Size = 0 then
      return;
    end if;

    Evidence_Req_Received := True;
    Log("Evidence requirement received");

    -- Publish an observation batch to observation_ingress that satisfies
    -- the requirement: SeaSurface Hostile Platform at (51.0, 0.0).
    -- The SIDC "SHSP------*****" encodes:
    --   S = Warfighting, H = Hostile, S = SeaSurface, P = Present
    if Exec_Handle /= null and then not Observation_Sent then
      declare
        -- Build a minimal observation batch JSON
        Obs_Json : constant String :=
          "{""observations"":[{" &
          """observation_id"":""00000000-0000-4000-8000-000000000001""," &
          """observed_at"":0.5," &
          """object_hint_type"":""Platform""," &
          """position"":{""lat"":51.0,""lon"":0.0,""alt"":0.0}," &
          """velocity"":{""north"":0.0,""east"":0.0,""down"":0.0}," &
          """affiliation_hint"":""Hostile""," &
          """confidence"":0.9," &
          """uncertainty_radius_m"":0.0," &
          """source_sidc"":""SHSP------*****""" &
          "}]}";
        Obs_C   : Interfaces.C.Strings.chars_ptr :=
          Interfaces.C.Strings.New_String(Obs_Json);
        Topic_C : Interfaces.C.Strings.chars_ptr :=
          Interfaces.C.Strings.New_String("observation_ingress");
        Pub_Msg : aliased Pcl_Bindings.Pcl_Msg;
        Status  : Pcl_Bindings.Pcl_Status;
      begin
        Pub_Msg.Data      := To_Address(Obs_C);
        Pub_Msg.Size      := Interfaces.C.unsigned(Obs_Json'Length);
        Pub_Msg.Type_Name := Interfaces.C.Strings.New_String("application/json");

        Log("Publishing observation to observation_ingress (" &
            Natural'Image(Obs_Json'Length) & " bytes)");

        Status := Pcl_Bindings.Publish(
          Exec_Handle, Topic_C, Pub_Msg'Access);

        if Status = Pcl_Bindings.PCL_OK then
          Observation_Sent := True;
          Log("Observation published successfully");
        else
          Log("Observation publish FAILED, status=" &
              Pcl_Bindings.Pcl_Status'Image(Status));
        end if;

        Interfaces.C.Strings.Free(Obs_C);
        Interfaces.C.Strings.Free(Topic_C);
        Interfaces.C.Strings.Free(Pub_Msg.Type_Name);
      end;
    end if;
  end Provider_Evidence_Req_Cb;

  function Provider_On_Configure
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Provider_On_Configure);

  function Provider_On_Configure
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status
  is
    Topic  : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("evidence_requirements");
    Type_N : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("application/json");
    Port   : Pcl_Bindings.Pcl_Port_Access;
    pragma Unreferenced(Port);
  begin
    Port := Pcl_Bindings.Add_Subscriber(
      Container => Container,
      Topic     => Topic,
      Type_Name => Type_N,
      Callback  => Provider_Evidence_Req_Cb'Unrestricted_Access,
      User_Data => User_Data);
    Interfaces.C.Strings.Free(Topic);
    Interfaces.C.Strings.Free(Type_N);
    return Pcl_Bindings.PCL_OK;
  end Provider_On_Configure;

  -- -- Main variables ---------------------------------------------------------

  Exec       : Pcl_Bindings.Pcl_Executor_Access;
  Transport  : Pcl_Bindings.Pcl_Socket_Transport_Access;
  Client_C   : Pcl_Bindings.Pcl_Container_Access;
  Provider_C : Pcl_Bindings.Pcl_Container_Access;
  Name_C     : Interfaces.C.Strings.chars_ptr;
  Client_Cbs : aliased Pcl_Bindings.Pcl_Callbacks;
  Prov_Cbs   : aliased Pcl_Bindings.Pcl_Callbacks;
  Status     : Pcl_Bindings.Pcl_Status;

begin
  Parse_Args;
  Log("Connecting to " & Interfaces.C.Strings.Value(Host_Str) &
      ":" & Interfaces.C.unsigned_short'Image(Port_Val));

  -- -- Create executor --------------------------------------------------------

  Exec := Pcl_Bindings.Create_Executor;
  if Exec = null then
    Log("FAIL: could not create executor");
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;
  Exec_Handle := Exec;

  -- -- Connect via socket client transport ------------------------------------

  Transport := Pcl_Bindings.Create_Socket_Client(Host_Str, Port_Val, Exec);
  Interfaces.C.Strings.Free(Host_Str);

  if Transport = null then
    Log("FAIL: could not connect to server");
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  Status := Pcl_Bindings.Set_Transport(
    Exec, Pcl_Bindings.Get_Socket_Transport(Transport));
  if Status /= Pcl_Bindings.PCL_OK then
    Log("FAIL: could not set transport");
    Pcl_Bindings.Destroy_Socket_Transport(Transport);
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- -- Create Ada client container --------------------------------------------

  Client_Cbs := (On_Configure  => Client_On_Configure'Unrestricted_Access,
                 On_Activate   => null,
                 On_Deactivate => null,
                 On_Cleanup    => null,
                 On_Shutdown   => null,
                 On_Tick       => null);
  Name_C := Interfaces.C.Strings.New_String("ada_client");
  Client_C := Pcl_Bindings.Create_Container(Name_C, Client_Cbs'Access,
                                             System.Null_Address);
  Interfaces.C.Strings.Free(Name_C);

  if Client_C = null then
    Log("FAIL: could not create client container");
    Pcl_Bindings.Destroy_Socket_Transport(Transport);
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  Status := Pcl_Bindings.Configure(Client_C);
  Status := Pcl_Bindings.Activate(Client_C);
  Status := Pcl_Bindings.Add_Container(Exec, Client_C);

  -- -- Create evidence provider container -------------------------------------

  Prov_Cbs := (On_Configure  => Provider_On_Configure'Unrestricted_Access,
               On_Activate   => null,
               On_Deactivate => null,
               On_Cleanup    => null,
               On_Shutdown   => null,
               On_Tick       => null);
  Name_C := Interfaces.C.Strings.New_String("ada_evidence_provider");
  Provider_C := Pcl_Bindings.Create_Container(Name_C, Prov_Cbs'Access,
                                               System.Null_Address);
  Interfaces.C.Strings.Free(Name_C);

  if Provider_C = null then
    Log("FAIL: could not create provider container");
    Pcl_Bindings.Destroy_Container(Client_C);
    Pcl_Bindings.Destroy_Socket_Transport(Transport);
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  Status := Pcl_Bindings.Configure(Provider_C);
  Status := Pcl_Bindings.Activate(Provider_C);
  Status := Pcl_Bindings.Add_Container(Exec, Provider_C);

  -- -- Invoke subscribe_interest with active_find -----------------------------

  declare
    Svc_Response_Ready : Boolean               := False;
    Svc_Response_Body  : Interfaces.C.unsigned  := 0;
    Solution_Id_Found  : Boolean                := False;
    Ev_Reqs_Found      : Boolean                := False;

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
        Svc_Response_Body := Resp.Size;
        -- Parse for solution_id and evidence_requirements presence
        -- (simple substring check since we don't have a JSON parser in Ada)
        if Resp.Data /= System.Null_Address and then Resp.Size > 0 then
          declare
            use System.Storage_Elements;
            type Char_Array is array (1 .. Natural(Resp.Size)) of Character;
            pragma Pack(Char_Array);
            Chars : Char_Array;
            for Chars'Address use Resp.Data;
            pragma Import(Ada, Chars);
            Body_Str : constant String := String(Chars);
          begin
            Log("Service response: " & Body_Str);
            -- Check for solution_id
            for I in Body_Str'First .. Body_Str'Last - 10 loop
              if Body_Str(I .. I + 10) = "solution_id" then
                Solution_Id_Found := True;
                exit;
              end if;
            end loop;
            -- Check for evidence_requirements (21 chars)
            for I in Body_Str'First .. Body_Str'Last - 20 loop
              if Body_Str(I .. I + 20) = "evidence_requirements" then
                Ev_Reqs_Found := True;
                exit;
              end if;
            end loop;
          end;
        end if;
      end if;
      Svc_Response_Ready := True;
    end Svc_Response_Cb;

    Query : Tactical_Object_Query;
  begin
    Query.Mode              := (Has => True, Value => Active_Find);
    Query.By_Type           := (Has => True, Value => Platform);
    Query.By_Affiliation    := (Has => True, Value => Hostile);
    Query.By_Battle_Dimension := (Has => True, Value => Sea_Surface);
    Query.By_Region         := (Has => True,
                                Value => (Min_Lat => 50.0, Max_Lat => 52.0,
                                          Min_Lon => -1.0, Max_Lon =>  1.0));

    declare
      Req_Str : constant String :=
        Tactical_Objects_Service.Build_Active_Find_Request_Json(Query);
      Req_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String(Req_Str);
      Svc_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String(
          Tactical_Objects_Service.Read_Service_Name);
      Req     : aliased Pcl_Bindings.Pcl_Msg;
    begin
      Log("ActiveFind request: " & Req_Str);

      Req.Data      := To_Address(Req_C);
      Req.Size      := Interfaces.C.unsigned(Req_Str'Length);
      Req.Type_Name := Interfaces.C.Strings.New_String("application/json");

      Status := Pcl_Bindings.Invoke_Remote_Async(
        Transport, Svc_C, Req'Access,
        Svc_Response_Cb'Unrestricted_Access, System.Null_Address);

      if Status /= Pcl_Bindings.PCL_OK then
        Log("subscribe_interest async submit FAILED, status=" &
            Pcl_Bindings.Pcl_Status'Image(Status));
      end if;

      Interfaces.C.Strings.Free(Req_C);
      Interfaces.C.Strings.Free(Svc_C);
      Interfaces.C.Strings.Free(Req.Type_Name);
    end;

    -- Spin until the full flow completes:
    --   1. Service response with solution_id + evidence_requirements
    --   2. Evidence provider receives requirement and publishes observation
    --   3. Server correlates and streams entity_updates
    --   4. Client receives entity update frames
    Log("Spinning to drive ActiveFind flow...");
    for Iteration in 1 .. 400 loop
      Status := Pcl_Bindings.Spin_Once(Exec, 0);
      exit when Svc_Response_Ready and then Frames_Received > 0;
      delay 0.01;  -- 10 ms
    end loop;

    -- Report findings
    if Svc_Response_Ready then
      Log("subscribe_interest OK, response size=" &
          Interfaces.C.unsigned'Image(Svc_Response_Body));
      if Solution_Id_Found then
        Log("  solution_id present in response");
      else
        Log("  WARNING: solution_id NOT found in response");
      end if;
      if Ev_Reqs_Found then
        Log("  evidence_requirements present in response");
      else
        Log("  WARNING: evidence_requirements NOT found in response");
      end if;
    else
      Log("subscribe_interest TIMEOUT: no response");
    end if;
  end;

  -- -- Report pass/fail -------------------------------------------------------

  Log("--- Results ---");
  Log("  Evidence requirement received: " &
      Boolean'Image(Evidence_Req_Received));
  Log("  Observation sent: " &
      Boolean'Image(Observation_Sent));
  Log("  Entity frames received: " &
      Natural'Image(Frames_Received));
  Log("  Found Hostile Platform: " &
      Boolean'Image(Found_Hostile_Platform));

  if Evidence_Req_Received and then
     Observation_Sent and then
     Frames_Received > 0 and then
     Found_Hostile_Platform
  then
    Log("PASS: ActiveFind flow completed — evidence provider drove " &
        "entity creation, client received Hostile Platform update");
    Ada.Command_Line.Set_Exit_Status(0);
  else
    Log("FAIL: ActiveFind flow incomplete");
    Ada.Command_Line.Set_Exit_Status(1);
  end if;

  -- -- Cleanup ----------------------------------------------------------------

  Pcl_Bindings.Destroy_Socket_Transport(Transport);
  Pcl_Bindings.Destroy_Container(Provider_C);
  Pcl_Bindings.Destroy_Container(Client_C);
  Pcl_Bindings.Destroy_Executor(Exec);
end Ada_Active_Find_E2E;
