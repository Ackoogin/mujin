--  tactical_objects_service.adb
--
--  Body for EntityActions service interface -- Tactical Objects component.
--  See tactical_objects_service.ads for rationale.

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with System;

package body Tactical_Objects_Service is

  -- =========================================================================
  --  Standard provided service handler stubs
  -- =========================================================================

  procedure Handle_Read_Match
    (Request  : in  Query;
     Response : out Object_Match_Array)
  is
    pragma Unreferenced(Request);
    Empty : Object_Match_Array(1 .. 0);
  begin
    Response := Empty;  --  TODO: implement
  end Handle_Read_Match;

  procedure Handle_Create_Requirement
    (Request  : in  Object_Interest_Requirement;
     Response : out Identifier)
  is
    pragma Unreferenced(Request);
  begin
    Response := Null_Identifier;  --  TODO: implement
  end Handle_Create_Requirement;

  procedure Handle_Read_Requirement
    (Request  : in  Query;
     Response : out Object_Interest_Requirement)
  is
    pragma Unreferenced(Request);
    Empty : Object_Interest_Requirement;
  begin
    Response := Empty;  --  TODO: implement
  end Handle_Read_Requirement;

  procedure Handle_Update_Requirement
    (Request  : in  Object_Interest_Requirement;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Update_Requirement;

  procedure Handle_Delete_Requirement
    (Request  : in  Identifier;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Delete_Requirement;

  procedure Handle_Read_Detail
    (Request  : in  Query;
     Response : out Object_Detail_Array)
  is
    pragma Unreferenced(Request);
    Empty : Object_Detail_Array(1 .. 0);
  begin
    Response := Empty;  --  TODO: implement
  end Handle_Read_Detail;

  -- =========================================================================
  --  Standard consumed service handler stubs
  -- =========================================================================

  procedure Handle_Read_Evidence_Detail
    (Request  : in  Query;
     Response : out Object_Detail_Array)
  is
    pragma Unreferenced(Request);
    Empty : Object_Detail_Array(1 .. 0);
  begin
    Response := Empty;  --  TODO: implement
  end Handle_Read_Evidence_Detail;

  procedure Handle_Create_Evidence_Requirement
    (Request  : in  Object_Evidence_Requirement;
     Response : out Identifier)
  is
    pragma Unreferenced(Request);
  begin
    Response := Null_Identifier;  --  TODO: implement
  end Handle_Create_Evidence_Requirement;

  procedure Handle_Read_Evidence_Requirement
    (Request  : in  Query;
     Response : out Object_Evidence_Requirement)
  is
    pragma Unreferenced(Request);
    Empty : Object_Evidence_Requirement;
  begin
    Response := Empty;  --  TODO: implement
  end Handle_Read_Evidence_Requirement;

  procedure Handle_Update_Evidence_Requirement
    (Request  : in  Object_Evidence_Requirement;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Update_Evidence_Requirement;

  procedure Handle_Delete_Evidence_Requirement
    (Request  : in  Identifier;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Delete_Evidence_Requirement;

  -- =========================================================================
  --  Legacy EntityActions handler stubs
  -- =========================================================================

  procedure Handle_Create_Tactical_Object
    (Request  : in  Tactical_Object;
     Response : out Identifier)
  is
    pragma Unreferenced(Request);
  begin
    Response := Null_Identifier;  --  TODO: implement
  end Handle_Create_Tactical_Object;

  procedure Handle_Read_Tactical_Object
    (Request  : in  Tactical_Object_Query;
     Response : out Tactical_Object_Array)
  is
    pragma Unreferenced(Request);
    Empty : Tactical_Object_Array(1 .. 0);
  begin
    Response := Empty;  --  TODO: implement
  end Handle_Read_Tactical_Object;

  procedure Handle_Update_Tactical_Object
    (Request  : in  Tactical_Object;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Update_Tactical_Object;

  procedure Handle_Delete_Tactical_Object
    (Request  : in  Identifier;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Delete_Tactical_Object;

  procedure Handle_Create_Zone
    (Request  : in  Zone_Entity;
     Response : out Identifier)
  is
    pragma Unreferenced(Request);
  begin
    Response := Null_Identifier;  --  TODO: implement
  end Handle_Create_Zone;

  procedure Handle_Read_Zone
    (Request  : in  Zone_Query;
     Response : out Zone_Array)
  is
    pragma Unreferenced(Request);
    Empty : Zone_Array(1 .. 0);
  begin
    Response := Empty;  --  TODO: implement
  end Handle_Read_Zone;

  procedure Handle_Update_Zone
    (Request  : in  Zone_Entity;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Update_Zone;

  procedure Handle_Delete_Zone
    (Request  : in  Identifier;
     Response : out Ack)
  is
    pragma Unreferenced(Request);
  begin
    Response := Ack_Ok;  --  TODO: implement
  end Handle_Delete_Zone;

  procedure Handle_Create_Observation
    (Request  : in  Observation;
     Response : out Identifier)
  is
    pragma Unreferenced(Request);
  begin
    Response := Null_Identifier;  --  TODO: implement
  end Handle_Create_Observation;

  procedure Dispatch
    (Channel      : in  Service_Channel;
     Request_Buf  : in  System.Address;
     Request_Size : in  Natural;
     Response_Buf : out System.Address;
     Response_Size: out Natural)
  is
    pragma Unreferenced(Request_Buf, Request_Size);
  begin
    Response_Buf  := System.Null_Address;
    Response_Size := 0;
    case Channel is
      when Ch_Create_Tactical_Object => null;  --  TODO: deserialise, call Handle_Create_Tactical_Object
      when Ch_Read_Tactical_Object   => null;  --  TODO: deserialise, call Handle_Read_Tactical_Object
      when Ch_Update_Tactical_Object => null;  --  TODO: deserialise, call Handle_Update_Tactical_Object
      when Ch_Delete_Tactical_Object => null;  --  TODO: deserialise, call Handle_Delete_Tactical_Object
      when Ch_Create_Zone            => null;  --  TODO: deserialise, call Handle_Create_Zone
      when Ch_Read_Zone              => null;  --  TODO: deserialise, call Handle_Read_Zone
      when Ch_Update_Zone            => null;  --  TODO: deserialise, call Handle_Update_Zone
      when Ch_Delete_Zone            => null;  --  TODO: deserialise, call Handle_Delete_Zone
      when Ch_Create_Observation     => null;  --  TODO: deserialise, call Handle_Create_Observation
    end case;
  end Dispatch;

  -- -- Internal helpers ------------------------------------------------------

  function Double_Image (V : Interfaces.C.double) return String is
  begin
    return Interfaces.C.double'Image(V);
  end Double_Image;

  function Bool_To_Json (V : Boolean) return String is
  begin
    return (if V then "true" else "false");
  end Bool_To_Json;
  pragma Unreferenced(Bool_To_Json);

  -- -- Build_Read_Request_Json -----------------------------------------------
  --
  --  Serialise TacticalObjectQuery -> JSON accepted by subscribe_interest.
  --
  --  C++ subscribe_interest expects (subset used by the test):
  --    { "object_type": "Platform",
  --      "affiliation": "Hostile",
  --      "area": { "min_lat": 50, "max_lat": 52,
  --                "min_lon": -1, "max_lon":  1 },
  --      "expires_at": 9999 }

  function Build_Read_Request_Json
    (Query : Tactical_Object_Query) return String
  is
    S : Unbounded_String := To_Unbounded_String("{");
    First : Boolean := True;

    procedure Comma is
    begin
      if not First then
        Append(S, ",");
      end if;
      First := False;
    end Comma;

    function Affiliation_Json_Name (A : Affiliation) return String is
    begin
      case A is
        when Friendly       => return "Friendly";
        when Hostile        => return "Hostile";
        when Neutral        => return "Neutral";
        when Unknown        => return "Unknown";
        when Assumed_Friend => return "AssumedFriend";
        when Suspect        => return "Suspect";
        when Joker          => return "Joker";
        when Faker          => return "Faker";
        when Pending        => return "Pending";
        when others         => return "Unknown";
      end case;
    end Affiliation_Json_Name;

    function Object_Type_Json_Name (T : Object_Type) return String is
    begin
      case T is
        when Platform     => return "Platform";
        when Person       => return "Person";
        when Equipment    => return "Equipment";
        when Unit         => return "Unit";
        when Formation    => return "Formation";
        when Installation => return "Installation";
        when Feature      => return "Feature";
        when Route        => return "Route";
        when Point        => return "Point";
        when Area         => return "Area";
        when Zone         => return "Zone";
        when others       => return "Platform";
      end case;
    end Object_Type_Json_Name;

    function Battle_Dim_Json_Name (D : Battle_Dimension) return String is
    begin
      case D is
        when Ground             => return "Ground";
        when Air                => return "Air";
        when Sea_Surface        => return "SeaSurface";
        when Subsurface         => return "Subsurface";
        when Dimension_Unknown  => return "Unknown";
        when others             => return "Ground";
      end case;
    end Battle_Dim_Json_Name;

  begin
    if Query.Mode.Has and then Query.Mode.Value = Active_Find then
      Comma;
      Append(S, """query_mode"":""active_find""");
    end if;
    if Query.By_Type.Has then
      Comma;
      Append(S, """object_type"":""" &
                Object_Type_Json_Name(Query.By_Type.Value) & """");
    end if;

    if Query.By_Affiliation.Has then
      Comma;
      Append(S, """affiliation"":""" &
                Affiliation_Json_Name(Query.By_Affiliation.Value) & """");
    end if;

    if Query.By_Battle_Dimension.Has then
      Comma;
      Append(S, """battle_dimension"":""" &
                Battle_Dim_Json_Name(Query.By_Battle_Dimension.Value) & """");
    end if;

    if Query.By_Region.Has then
      declare
        BB : constant Bounding_Box := Query.By_Region.Value;
      begin
        Comma;
        Append(S, """area"":{" &
                  """min_lat"":" & Double_Image(BB.Min_Lat) & "," &
                  """max_lat"":" & Double_Image(BB.Max_Lat) & "," &
                  """min_lon"":" & Double_Image(BB.Min_Lon) & "," &
                  """max_lon"":" & Double_Image(BB.Max_Lon) & "}");
        --  Add expires_at when a region is specified (far-future default)
        Comma;
        Append(S, """expires_at"":9999");
      end;
    end if;

    if Query.Max_Age_Seconds.Has then
      Comma;
      Append(S, """max_age_seconds"":" &
                Double_Image(Query.Max_Age_Seconds.Value));
    end if;

    if Query.By_Source_System.Has then
      Comma;
      Append(S, """source_system"":""" &
                To_String(Query.By_Source_System.Value) & """");
    end if;

    Append(S, "}");
    return To_String(S);
  end Build_Read_Request_Json;

  -- -- Build_Standard_Requirement_Json ----------------------------------------
  --
  --  Build standard bridge request JSON for object_of_interest.create_requirement.
  --  Uses STANDARD_IDENTITY_* and BATTLE_DIMENSION_* enum names.
  --  Positions are in radians.

  function Build_Standard_Requirement_Json
    (Policy    : String;
     Identity  : String;
     Dimension : String;
     Min_Lat_Rad : Interfaces.C.double;
     Max_Lat_Rad : Interfaces.C.double;
     Min_Lon_Rad : Interfaces.C.double;
     Max_Lon_Rad : Interfaces.C.double) return String
  is
    S : Unbounded_String := To_Unbounded_String("{");
  begin
    Append(S, """policy"":""" & Policy & """,");
    Append(S, """identity"":""" & Identity & """,");
    Append(S, """dimension"":""" & Dimension & """,");
    Append(S, """min_lat_rad"":" & Double_Image(Min_Lat_Rad) & ",");
    Append(S, """max_lat_rad"":" & Double_Image(Max_Lat_Rad) & ",");
    Append(S, """min_lon_rad"":" & Double_Image(Min_Lon_Rad) & ",");
    Append(S, """max_lon_rad"":" & Double_Image(Max_Lon_Rad));
    Append(S, "}");
    return To_String(S);
  end Build_Standard_Requirement_Json;

  -- -- Build_Standard_Evidence_Json -------------------------------------------
  --
  --  Build standard bridge evidence JSON for standard.object_evidence.

  function Build_Standard_Evidence_Json
    (Identity    : String;
     Dimension   : String;
     Lat_Rad     : Interfaces.C.double;
     Lon_Rad     : Interfaces.C.double;
     Confidence  : Interfaces.C.double;
     Observed_At : Interfaces.C.double := 0.5) return String
  is
    S : Unbounded_String := To_Unbounded_String("{");
  begin
    Append(S, """identity"":""" & Identity & """,");
    Append(S, """dimension"":""" & Dimension & """,");
    Append(S, """latitude_rad"":" & Double_Image(Lat_Rad) & ",");
    Append(S, """longitude_rad"":" & Double_Image(Lon_Rad) & ",");
    Append(S, """confidence"":" & Double_Image(Confidence) & ",");
    Append(S, """observed_at"":" & Double_Image(Observed_At));
    Append(S, "}");
    return To_String(S);
  end Build_Standard_Evidence_Json;

  -- -- Build_Active_Find_Request_Json -----------------------------------------

  function Build_Active_Find_Request_Json
    (Query : Tactical_Object_Query) return String
  is
    AF_Query : Tactical_Object_Query := Query;
  begin
    AF_Query.Mode := (Has => True, Value => Active_Find);
    return Build_Read_Request_Json(AF_Query);
  end Build_Active_Find_Request_Json;

  -- -- Ordinal conversions (C++ StreamingCodec wire-format ordinals) ----------

  function Ordinal_To_Affiliation
    (V : Streaming_Codec.Byte) return Affiliation is
  begin
    case V is
      when 0      => return Friendly;
      when 1      => return Hostile;
      when 2      => return Neutral;
      when 3      => return Unknown;
      when 4      => return Assumed_Friend;
      when 5      => return Suspect;
      when 6      => return Joker;
      when 7      => return Faker;
      when 8      => return Pending;
      when others => return Affiliation_Unspecified;
    end case;
  end Ordinal_To_Affiliation;

  function Ordinal_To_Object_Type
    (V : Streaming_Codec.Byte) return Object_Type is
  begin
    case V is
      when 0      => return Platform;
      when 1      => return Person;
      when 2      => return Equipment;
      when 3      => return Unit;
      when 4      => return Formation;
      when 5      => return Installation;
      when 6      => return Feature;
      when 7      => return Route;
      when 8      => return Point;
      when 9      => return Area;
      when 10     => return Zone;
      when others => return Object_Type_Unspecified;
    end case;
  end Ordinal_To_Object_Type;

  function Ordinal_To_Lifecycle_Status
    (V : Streaming_Codec.Byte) return Lifecycle_Status is
  begin
    case V is
      when 0      => return Active;
      when 1      => return Stale;
      when 2      => return Retired;
      when others => return Lifecycle_Unspecified;
    end case;
  end Ordinal_To_Lifecycle_Status;

  -- -- Frame_To_Tactical_Object -----------------------------------------------

  function Frame_To_Tactical_Object
    (Frame : Streaming_Codec.Entity_Update_Frame) return Tactical_Object
  is
    Obj : Tactical_Object;
  begin
    if Frame.Has_Object_Type then
      Obj.Obj_Type := Ordinal_To_Object_Type(Frame.Object_Type_Val);
    end if;

    if Frame.Has_Position then
      Obj.Pos := (Lat => Frame.Pos.Lat,
                  Lon => Frame.Pos.Lon,
                  Alt => Frame.Pos.Alt);
      Obj.Has_Position := True;
    end if;

    if Frame.Has_Velocity then
      Obj.Vel := (North => Frame.Vel.North,
                  East  => Frame.Vel.East,
                  Down  => Frame.Vel.Down);
      Obj.Has_Velocity := True;
    end if;

    if Frame.Has_Affiliation then
      Obj.Affil := Ordinal_To_Affiliation(Frame.Affiliation_Val);
    end if;

    if Frame.Has_Lifecycle then
      Obj.Status := Ordinal_To_Lifecycle_Status(Frame.Lifecycle_Val);
    end if;

    Obj.Version := Frame.Version;

    return Obj;
  end Frame_To_Tactical_Object;

  -- -- Tactical_Object_Image --------------------------------------------------

  function Tactical_Object_Image
    (Obj : Tactical_Object) return String
  is
    function Affil_Name (A : Affiliation) return String is
    begin
      case A is
        when Friendly       => return "Friendly";
        when Hostile        => return "Hostile";
        when Neutral        => return "Neutral";
        when Unknown        => return "Unknown";
        when Assumed_Friend => return "AssumedFriend";
        when Suspect        => return "Suspect";
        when Joker          => return "Joker";
        when Faker          => return "Faker";
        when Pending        => return "Pending";
        when others         => return "?";
      end case;
    end Affil_Name;

    function ObjType_Name (T : Object_Type) return String is
    begin
      case T is
        when Platform     => return "Platform";
        when Person       => return "Person";
        when Equipment    => return "Equipment";
        when Unit         => return "Unit";
        when Formation    => return "Formation";
        when Installation => return "Installation";
        when Feature      => return "Feature";
        when Route        => return "Route";
        when Point        => return "Point";
        when Area         => return "Area";
        when Zone         => return "Zone";
        when others       => return "?";
      end case;
    end ObjType_Name;

    S : Unbounded_String;
  begin
    Append(S, "type=" & ObjType_Name(Obj.Obj_Type));
    Append(S, " affil=" & Affil_Name(Obj.Affil));
    if Obj.Has_Position then
      Append(S, " lat=" & Double_Image(Obj.Pos.Lat) &
                " lon=" & Double_Image(Obj.Pos.Lon));
    end if;
    Append(S, " v=" & Interfaces.C.unsigned_long'Image(Obj.Version));
    return To_String(S);
  end Tactical_Object_Image;

end Tactical_Objects_Service;
