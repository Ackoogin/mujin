-- ada_active_find_e2e.adb
-- Multi-process Ada E2E test for the ActiveFind flow via the StandardBridge.
--
-- Two PCL containers run in a single Ada process, connected to a server via
-- TCP socket transport.  The server runs TacticalObjectsComponent + StandardBridge.
--
-- STANDARD interface (bridge topics/services):
--
--   1. Ada Client — calls "object_of_interest.create_requirement" (standard JSON),
--      subscribes to "standard.entity_matches" (standard JSON).
--   2. Evidence Provider — subscribes to "standard.evidence_requirements" (JSON),
--      and on receipt publishes a standard observation to "standard.object_evidence".
--
-- The bridge translates:
--   - create_requirement  → subscribe_interest (internal)
--   - standard.object_evidence → processObservationBatch (internal)
--   - entity_updates (binary) → standard.entity_matches (JSON)
--   - evidence_requirements (JSON) → standard.evidence_requirements (JSON)
--
-- Usage: ada_active_find_e2e --host 127.0.0.1 --port 19123

with Ada.Command_Line;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
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
  use type System.Address;

  function To_Address is new
    Ada.Unchecked_Conversion(Interfaces.C.Strings.chars_ptr, System.Address);

  Pi         : constant Interfaces.C.double := 3.14159265358979323846;
  Deg_To_Rad : constant Interfaces.C.double := Pi / 180.0;

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

  Matches_Received       : Natural := 0;
  Found_Hostile_Entity   : Boolean := False;
  Evidence_Req_Received  : Boolean := False;
  Observation_Sent       : Boolean := False;

  -- Store executor handle for evidence provider to publish through
  Exec_Handle : Pcl_Bindings.Pcl_Executor_Access := null;

  -- ═══════════════════════════════════════════════════════════════════════════
  -- Container 1: Ada Client — subscribes to standard.entity_matches
  -- ═══════════════════════════════════════════════════════════════════════════

  procedure Client_Entity_Matches_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Client_Entity_Matches_Cb);

  procedure Client_Entity_Matches_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address)
  is
    pragma Unreferenced(Container, User_Data);
  begin
    if Msg.Data = System.Null_Address or else Msg.Size = 0 then
      return;
    end if;

    declare
      use System.Storage_Elements;
      type Char_Array is array (1 .. Natural(Msg.Size)) of Character;
      pragma Pack(Char_Array);
      Chars : Char_Array;
      for Chars'Address use Msg.Data;
      pragma Import(Ada, Chars);
      Body_Str : constant String := String(Chars);
    begin
      Log("standard.entity_matches: " & Body_Str);
      --  Count opening braces as a proxy for entity count in the JSON array
      for C of Body_Str loop
        if C = '{' then
          Matches_Received := Matches_Received + 1;
        end if;
      end loop;

      --  Check for HOSTILE identity in the JSON
      for I in Body_Str'First .. Body_Str'Last - 24 loop
        if Body_Str(I .. I + 24) = "STANDARD_IDENTITY_HOSTILE" then
          Found_Hostile_Entity := True;
          exit;
        end if;
      end loop;
    end;
  end Client_Entity_Matches_Cb;

  function Client_On_Configure
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Client_On_Configure);

  function Client_On_Configure
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status
  is
    Topic  : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String(
        Tactical_Objects_Service.Standard_Entity_Matches_Topic);
    Type_N : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("application/json");
    Port   : Pcl_Bindings.Pcl_Port_Access;
    pragma Unreferenced(Port);
  begin
    Port := Pcl_Bindings.Add_Subscriber(
      Container => Container,
      Topic     => Topic,
      Type_Name => Type_N,
      Callback  => Client_Entity_Matches_Cb'Unrestricted_Access,
      User_Data => User_Data);
    Interfaces.C.Strings.Free(Topic);
    Interfaces.C.Strings.Free(Type_N);
    return Pcl_Bindings.PCL_OK;
  end Client_On_Configure;

  -- ═══════════════════════════════════════════════════════════════════════════
  -- Container 2: Evidence Provider — subscribes to standard.evidence_requirements
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

    if Msg.Size > 0 then
      declare
        use System.Storage_Elements;
        type Char_Array is array (1 .. Natural(Msg.Size)) of Character;
        pragma Pack(Char_Array);
        Chars : Char_Array;
        for Chars'Address use Msg.Data;
        pragma Import(Ada, Chars);
      begin
        Log("Standard evidence requirement: " & String(Chars));
      end;
    else
      Log("Standard evidence requirement received");
    end if;

    -- Publish a standard observation to standard.object_evidence.
    -- The bridge converts this to a processObservationBatch() call internally.
    -- Position: 51.0°N 0.0°E → radians; identity: HOSTILE; dim: SEA_SURFACE.
    if Exec_Handle /= null and then not Observation_Sent then
      declare
        Obs_Json : constant String :=
          Tactical_Objects_Service.Build_Standard_Evidence_Json(
            Identity    => "STANDARD_IDENTITY_HOSTILE",
            Dimension   => "BATTLE_DIMENSION_SEA_SURFACE",
            Lat_Rad     => 51.0 * Deg_To_Rad,
            Lon_Rad     => 0.0 * Deg_To_Rad,
            Confidence  => 0.9,
            Observed_At => 0.5);
        Obs_C   : Interfaces.C.Strings.chars_ptr :=
          Interfaces.C.Strings.New_String(Obs_Json);
        Topic_C : Interfaces.C.Strings.chars_ptr :=
          Interfaces.C.Strings.New_String(
            Tactical_Objects_Service.Standard_Object_Evidence_Topic);
        Pub_Msg : aliased Pcl_Bindings.Pcl_Msg;
        Status  : Pcl_Bindings.Pcl_Status;
      begin
        Pub_Msg.Data      := To_Address(Obs_C);
        Pub_Msg.Size      := Interfaces.C.unsigned(Obs_Json'Length);
        Pub_Msg.Type_Name := Interfaces.C.Strings.New_String("application/json");

        Log("Publishing standard observation to " &
            Tactical_Objects_Service.Standard_Object_Evidence_Topic &
            " (" & Natural'Image(Obs_Json'Length) & " bytes)");

        Status := Pcl_Bindings.Publish(
          Exec_Handle, Topic_C, Pub_Msg'Access);

        if Status = Pcl_Bindings.PCL_OK then
          Observation_Sent := True;
          Log("Standard observation published successfully");
        else
          Log("Standard observation publish FAILED, status=" &
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
      Interfaces.C.Strings.New_String(
        Tactical_Objects_Service.Standard_Evidence_Reqs_Topic);
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

  -- -- Call object_of_interest.create_requirement (standard bridge) -----------
  --
  --  Standard ActiveFind: policy=DATA_POLICY_OBTAIN, identity=HOSTILE,
  --  dimension=SEA_SURFACE, bounding box [50°,52°] × [-1°,1°] in radians.

  declare
    Svc_Response_Ready : Boolean               := False;
    Svc_Response_Body  : Interfaces.C.unsigned := 0;
    Interest_Id_Found  : Boolean               := False;

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
            Log("create_requirement response: " & Body_Str);
            -- Check for interest_id in response
            for I in Body_Str'First .. Body_Str'Last - 10 loop
              if Body_Str(I .. I + 10) = "interest_id" then
                Interest_Id_Found := True;
                exit;
              end if;
            end loop;
          end;
        end if;
      end if;
      Svc_Response_Ready := True;
    end Svc_Response_Cb;

  begin
    declare
      Req_Str : constant String :=
        Tactical_Objects_Service.Build_Standard_Requirement_Json(
          Policy      => "DATA_POLICY_OBTAIN",
          Identity    => "STANDARD_IDENTITY_HOSTILE",
          Dimension   => "BATTLE_DIMENSION_SEA_SURFACE",
          Min_Lat_Rad => 50.0 * Deg_To_Rad,
          Max_Lat_Rad => 52.0 * Deg_To_Rad,
          Min_Lon_Rad => (-1.0) * Deg_To_Rad,
          Max_Lon_Rad => 1.0 * Deg_To_Rad);
      Req_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String(Req_Str);
      Svc_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String(
          Tactical_Objects_Service.Standard_Create_Requirement_Service);
      Req     : aliased Pcl_Bindings.Pcl_Msg;
    begin
      Log("Standard ActiveFind request: " & Req_Str);

      Req.Data      := To_Address(Req_C);
      Req.Size      := Interfaces.C.unsigned(Req_Str'Length);
      Req.Type_Name := Interfaces.C.Strings.New_String("application/json");

      Status := Pcl_Bindings.Invoke_Remote_Async(
        Transport, Svc_C, Req'Access,
        Svc_Response_Cb'Unrestricted_Access, System.Null_Address);

      if Status /= Pcl_Bindings.PCL_OK then
        Log("create_requirement async submit FAILED, status=" &
            Pcl_Bindings.Pcl_Status'Image(Status));
      end if;

      Interfaces.C.Strings.Free(Req_C);
      Interfaces.C.Strings.Free(Svc_C);
      Interfaces.C.Strings.Free(Req.Type_Name);
    end;

    -- Spin until the full standard flow completes:
    --   1. create_requirement response with interest_id
    --   2. Bridge forwards evidence_requirements → standard.evidence_requirements
    --   3. Evidence provider receives and publishes to standard.object_evidence
    --   4. Bridge converts to processObservationBatch → correlates entity
    --   5. Server streams entity_updates (binary) → bridge publishes standard.entity_matches
    --   6. Client receives standard entity matches
    Log("Spinning to drive standard ActiveFind flow...");
    for Iteration in 1 .. 400 loop
      Status := Pcl_Bindings.Spin_Once(Exec, 0);
      exit when Svc_Response_Ready and then Matches_Received > 0;
      delay 0.01;  -- 10 ms
    end loop;

    -- Report findings
    if Svc_Response_Ready then
      Log("create_requirement OK, response size=" &
          Interfaces.C.unsigned'Image(Svc_Response_Body));
      if Interest_Id_Found then
        Log("  interest_id present in response");
      else
        Log("  WARNING: interest_id NOT found in response");
      end if;
    else
      Log("create_requirement TIMEOUT: no response");
    end if;
  end;

  -- -- Report pass/fail -------------------------------------------------------

  Log("--- Results ---");
  Log("  Evidence requirement received: " &
      Boolean'Image(Evidence_Req_Received));
  Log("  Standard observation sent:     " &
      Boolean'Image(Observation_Sent));
  Log("  Standard entity matches received: " &
      Natural'Image(Matches_Received));
  Log("  Found HOSTILE entity:          " &
      Boolean'Image(Found_Hostile_Entity));

  if Evidence_Req_Received and then
     Observation_Sent and then
     Matches_Received > 0 and then
     Found_Hostile_Entity
  then
    Log("PASS: Standard ActiveFind flow completed — " &
        "evidence provider drove entity creation via bridge, " &
        "client received HOSTILE entity via standard.entity_matches");
    Ada.Command_Line.Set_Exit_Status(0);
  else
    Log("FAIL: Standard ActiveFind flow incomplete");
    Ada.Command_Line.Set_Exit_Status(1);
  end if;

  -- -- Cleanup ----------------------------------------------------------------

  Pcl_Bindings.Destroy_Socket_Transport(Transport);
  Pcl_Bindings.Destroy_Container(Provider_C);
  Pcl_Bindings.Destroy_Container(Client_C);
  Pcl_Bindings.Destroy_Executor(Exec);
end Ada_Active_Find_E2E;
