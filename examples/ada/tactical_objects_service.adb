--  tactical_objects_service.adb
--
--  Body for Tactical_Objects_Service.
--
--  Handler stubs, Dispatch, and standard JSON builders are now in the
--  generated packages (Pyramid.Services.Tactical_Objects.Provided / Consumed).
--  This body retains only the legacy query JSON builders and codec helpers
--  that are not proto-derived.

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;

package body Tactical_Objects_Service is

  -- -- Internal helpers ------------------------------------------------------

  function Double_Image (V : Interfaces.C.double) return String is
  begin
    return Interfaces.C.double'Image(V);
  end Double_Image;

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
