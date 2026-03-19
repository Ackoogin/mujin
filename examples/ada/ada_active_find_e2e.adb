-- ada_active_find_e2e.adb
-- Multi-process Ada E2E test for the standard ActiveFind flow via StandardBridge.
--
-- Two PCL containers in a single Ada process connect to the server (which runs
-- TacticalObjectsComponent + StandardBridge) over TCP socket transport.
--
-- STANDARD interface:
--   1. Ada Client — calls "object_of_interest.create_requirement" (standard JSON),
--      subscribes to "standard.entity_matches" (standard JSON array).
--   2. Evidence Provider — subscribes to "standard.evidence_requirements",
--      and on receipt publishes a standard observation to "standard.object_evidence".
--
-- JSON is built and parsed with GNATCOLL.JSON (no raw string manipulation).
--
-- Usage: ada_active_find_e2e --host 127.0.0.1 --port 19123

with Ada.Command_Line;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces.C.Strings;
with GNATCOLL.JSON;
with Pcl_Bindings;
with Tactical_Objects_Types;   use Tactical_Objects_Types;
with Tactical_Objects_Service;
with System;
with System.Storage_Elements;

procedure Ada_Active_Find_E2E is
  use GNATCOLL.JSON;
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

  Pi         : constant Long_Float := 3.14159265358979323846;
  Deg_To_Rad : constant Long_Float := Pi / 180.0;

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

  --  Helper: extract Ada String from a pcl_msg_t data pointer + size
  function Msg_To_String
    (Data : System.Address; Size : Interfaces.C.unsigned) return String
  is
    use System.Storage_Elements;
    type Char_Array is array (1 .. Natural(Size)) of Character;
    pragma Pack(Char_Array);
    Chars : Char_Array;
    for Chars'Address use Data;
    pragma Import(Ada, Chars);
  begin
    return String(Chars);
  end Msg_To_String;

  -- -- Shared state -----------------------------------------------------------

  Matches_Received      : Natural := 0;
  Found_Hostile_Entity  : Boolean := False;
  Evidence_Req_Received : Boolean := False;
  Observation_Sent      : Boolean := False;

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
      Body_Str     : constant String :=
        Msg_To_String(Msg.Data, Msg.Size);
      Parse_Result : GNATCOLL.JSON.Read_Result;
      Val          : GNATCOLL.JSON.JSON_Value;
      Arr          : GNATCOLL.JSON.JSON_Array;
    begin
      Log("standard.entity_matches: " & Body_Str);
      Parse_Result := GNATCOLL.JSON.Read(Body_Str);
      if not Parse_Result.Success then
        Log("WARNING: could not parse entity_matches JSON");
        return;
      end if;
      Val := Parse_Result.Value;
      if Val.Kind /= GNATCOLL.JSON.JSON_Array_Type then
        return;
      end if;
      Arr := GNATCOLL.JSON.Get(Val);
      for I in 1 .. GNATCOLL.JSON.Length(Arr) loop
        declare
          Ent      : constant GNATCOLL.JSON.JSON_Value :=
            GNATCOLL.JSON.Get(Arr, I);
          Identity : constant String :=
            (if GNATCOLL.JSON.Has_Field(Ent, "identity")
             then GNATCOLL.JSON.Get(Ent, "identity")
             else "");
          Obj_Id   : constant String :=
            (if GNATCOLL.JSON.Has_Field(Ent, "object_id")
             then GNATCOLL.JSON.Get(Ent, "object_id")
             else "");
        begin
          Log("  entity[" & Natural'Image(I) & "] id=" & Obj_Id &
              " identity=" & Identity);
          Matches_Received := Matches_Received + 1;
          if Identity = "STANDARD_IDENTITY_HOSTILE" then
            Found_Hostile_Entity := True;
          end if;
        end;
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
    Log("Standard evidence requirement: " &
        Msg_To_String(Msg.Data, Msg.Size));

    -- Publish a standard observation to standard.object_evidence.
    -- Position: 51.0°N 0.0°E in radians; HOSTILE; SEA_SURFACE dimension.
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
        Pub_St  : Pcl_Bindings.Pcl_Status;
      begin
        Pub_Msg.Data      := To_Address(Obs_C);
        Pub_Msg.Size      := Interfaces.C.unsigned(Obs_Json'Length);
        Pub_Msg.Type_Name := Interfaces.C.Strings.New_String("application/json");

        Log("Publishing standard observation to " &
            Tactical_Objects_Service.Standard_Object_Evidence_Topic);

        Pub_St := Pcl_Bindings.Publish(
          Exec_Handle, Topic_C, Pub_Msg'Access);

        if Pub_St = Pcl_Bindings.PCL_OK then
          Observation_Sent := True;
          Log("Standard observation published successfully");
        else
          Log("Standard observation publish FAILED, status=" &
              Pcl_Bindings.Pcl_Status'Image(Pub_St));
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

  -- -- Call object_of_interest.create_requirement (ActiveFind via bridge) ----
  --
  --  Policy DATA_POLICY_OBTAIN → bridge maps to active_find.
  --  Dimension filter SEA_SURFACE included: the evidence observation will
  --  carry this dimension hint for the correlation engine.
  --  Bounding box [50°,52°] × [-1°,1°] in radians.

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
      if Resp /= null and then
         Resp.Data /= System.Null_Address and then
         Resp.Size > 0
      then
        Svc_Response_Body := Resp.Size;
        declare
          Body_Str     : constant String :=
            Msg_To_String(Resp.Data, Resp.Size);
          Parse_Result : GNATCOLL.JSON.Read_Result;
        begin
          Log("create_requirement response: " & Body_Str);
          Parse_Result := GNATCOLL.JSON.Read(Body_Str);
          if Parse_Result.Success and then
             GNATCOLL.JSON.Has_Field(Parse_Result.Value, "interest_id")
          then
            Interest_Id_Found := True;
          end if;
        end;
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

    -- Spin until the full standard flow completes.
    Log("Spinning to drive standard ActiveFind flow...");
    for Iteration in 1 .. 400 loop
      Status := Pcl_Bindings.Spin_Once(Exec, 0);
      exit when Svc_Response_Ready and then Matches_Received > 0;
      delay 0.01;  -- 10 ms
    end loop;

    -- Report intermediate findings
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
  Log("  Standard entity matches:       " &
      Natural'Image(Matches_Received));
  Log("  Found HOSTILE entity:          " &
      Boolean'Image(Found_Hostile_Entity));

  if Evidence_Req_Received and then
     Observation_Sent and then
     Matches_Received > 0 and then
     Found_Hostile_Entity
  then
    Log("PASS: Standard ActiveFind flow — evidence provider drove entity " &
        "creation via bridge, client received HOSTILE via standard.entity_matches");
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
