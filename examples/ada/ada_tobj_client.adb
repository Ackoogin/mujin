-- ada_tobj_client.adb
-- Standalone Ada executable that connects to a TacticalObjectsComponent server
-- via TCP socket transport.
--
-- Uses the STANDARD bridge interface:
--   - Calls  "object_of_interest.create_requirement"  (standard JSON)
--   - Subscribes to "standard.entity_matches"         (standard JSON array)
--
-- JSON is built with GNATCOLL.JSON (no string concatenation).
--
-- Usage: ada_tobj_client --host 127.0.0.1 --port 19123

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

procedure Ada_Tobj_Client is
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
                         "[ada_tobj_client] " & Msg);
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

  -- -- Client state -----------------------------------------------------------

  Matches_Received : Natural := 0;

  -- -- Subscriber callback: standard.entity_matches (JSON array) -------------
  --
  --  Parses the JSON array with GNATCOLL.JSON, counts entity objects,
  --  and logs each one.

  procedure Entity_Matches_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Entity_Matches_Cb);

  procedure Entity_Matches_Cb
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
      if Val.Kind = GNATCOLL.JSON.JSON_Array_Type then
        Arr := GNATCOLL.JSON.Get(Val);
        for I in 1 .. GNATCOLL.JSON.Length(Arr) loop
          declare
            Ent      : constant GNATCOLL.JSON.JSON_Value :=
              GNATCOLL.JSON.Get(Arr, I);
            Identity : constant String :=
              (if GNATCOLL.JSON.Has_Field(Ent, "identity")
               then GNATCOLL.JSON.Get(Ent, "identity")
               else "?");
            Obj_Id   : constant String :=
              (if GNATCOLL.JSON.Has_Field(Ent, "object_id")
               then GNATCOLL.JSON.Get(Ent, "object_id")
               else "?");
          begin
            Log("  entity[" & Natural'Image(I) & "] id=" & Obj_Id &
                " identity=" & Identity);
            Matches_Received := Matches_Received + 1;
          end;
        end loop;
      end if;
    end;
  end Entity_Matches_Cb;

  -- -- on_configure: subscribe to standard.entity_matches --------------------

  function On_Configure_Cb
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, On_Configure_Cb);

  function On_Configure_Cb
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
      Callback  => Entity_Matches_Cb'Unrestricted_Access,
      User_Data => User_Data);
    Interfaces.C.Strings.Free(Topic);
    Interfaces.C.Strings.Free(Type_N);
    return Pcl_Bindings.PCL_OK;
  end On_Configure_Cb;

  -- -- Main variables ---------------------------------------------------------

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

  -- -- Create executor --------------------------------------------------------

  Exec := Pcl_Bindings.Create_Executor;
  if Exec = null then
    Log("FAIL: could not create executor");
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- -- Connect via socket client transport ------------------------------------

  Transport := Pcl_Bindings.Create_Socket_Client(Host_Str, Port_Val, Exec);
  Interfaces.C.Strings.Free(Host_Str);

  if Transport = null then
    Log("FAIL: could not connect to server");
    Pcl_Bindings.Destroy_Executor(Exec);
    Ada.Command_Line.Set_Exit_Status(1);
    return;
  end if;

  -- -- Set transport on executor ----------------------------------------------

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

  -- -- Create subscriber container --------------------------------------------

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

  -- -- Call object_of_interest.create_requirement (standard bridge) ----------
  --
  --  Query policy: DATA_POLICY_QUERY (read-current, no active tasking).
  --  Identity filter: HOSTILE only.  No dimension filter (entity has no
  --  mil-class battle-dimension set, so omit to avoid false exclusion).
  --  Bounding box: lat [50°, 52°] lon [-1°, 1°] in radians.

  declare
    Pi            : constant Long_Float := 3.14159265358979323846;
    Deg_To_Rad    : constant Long_Float := Pi / 180.0;

    Svc_Response_Ready   : Boolean               := False;
    Svc_Response_Size    : Interfaces.C.unsigned := 0;
    Interest_Id_Received : Boolean               := False;

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
        Svc_Response_Size := Resp.Size;
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
            Interest_Id_Received := True;
          end if;
        end;
      end if;
      Svc_Response_Ready := True;
    end Svc_Response_Cb;

  begin
    declare
      --  No dimension filter: this is a passive read-current query.
      --  The test entity has affiliation=Hostile but no milclass battle-dim.
      Req_Str : constant String :=
        Tactical_Objects_Service.Build_Standard_Requirement_Json(
          Policy      => "DATA_POLICY_QUERY",
          Identity    => "STANDARD_IDENTITY_HOSTILE",
          Dimension   => "",           --  omit: no dimension filter
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
      Log("create_requirement request: " & Req_Str);

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

    --  Spin until the service response arrives then wait for entity matches.
    Log("Spinning to receive service response and entity matches...");
    for Iteration in 1 .. 200 loop
      Status := Pcl_Bindings.Spin_Once(Exec, 0);
      exit when Svc_Response_Ready and then Matches_Received > 0;
      delay 0.01;  -- 10 ms
    end loop;

    if Svc_Response_Ready then
      Log("create_requirement OK, response size=" &
          Interfaces.C.unsigned'Image(Svc_Response_Size));
      if Interest_Id_Received then
        Log("  interest_id present");
      else
        Log("  WARNING: interest_id NOT found in response");
      end if;
    else
      Log("create_requirement TIMEOUT: no response");
    end if;
  end;

  -- -- Report pass/fail ------------------------------------------------------

  if Matches_Received > 0 then
    Log("PASS: received" & Natural'Image(Matches_Received) &
        " standard entity match(es) via bridge");
    Ada.Command_Line.Set_Exit_Status(0);
  else
    Log("FAIL: no standard entity matches received");
    Ada.Command_Line.Set_Exit_Status(1);
  end if;

  -- -- Cleanup ---------------------------------------------------------------

  Pcl_Bindings.Destroy_Socket_Transport(Transport);
  Pcl_Bindings.Destroy_Container(Container);
  Pcl_Bindings.Destroy_Executor(Exec);
end Ada_Tobj_Client;
