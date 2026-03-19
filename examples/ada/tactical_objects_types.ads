--  tactical_objects_types.ads
--  Ada types mirroring the standard pyramid data model proto files:
--    proto/pyramid/data_model/base.proto
--    proto/pyramid/data_model/common.proto
--    proto/pyramid/data_model/tactical.proto
--    proto/pyramid/components/tactical_objects.proto  (legacy)
--
--  Standard types (from common/tactical protos) are prefixed with their
--  conceptual origin in comments.  Legacy types (Affiliation, Object_Type,
--  etc.) are retained for backward compatibility with the streaming wire
--  format decoded by Streaming_Codec — ordinals match C++ StreamingCodec.
--
--  Enum ordinal values for LEGACY types match the C++ codec ordinal helpers
--  in StreamingCodec (not the standard proto ordinals).

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;

package Tactical_Objects_Types is

  -- -- base.proto: Identifier --------------------------------------------------

  subtype Identifier is Unbounded_String;
  Null_Identifier : constant Identifier := Null_Unbounded_String;

  -- -- common.proto: Ack / Query -----------------------------------------------

  type Ack is record
    Success : Boolean := False;
  end record;

  type Query is record
    One_Shot : Boolean := True;
    --  ID filter omitted: subscribe_interest uses predicate filters instead.
  end record;

  Ack_Ok   : constant Ack := (Success => True);
  Ack_Fail : constant Ack := (Success => False);

  -- -- common.proto: StandardIdentity ------------------------------------------
  --  Ordinals match pyramid.data_model.common.StandardIdentity proto values.

  type Standard_Identity is
    (Identity_Unspecified,   --  0
     Identity_Unknown,       --  1
     Identity_Friendly,      --  2
     Identity_Hostile,       --  3
     Identity_Suspect,       --  4
     Identity_Neutral,       --  5
     Identity_Pending,       --  6
     Identity_Joker,         --  7
     Identity_Faker,         --  8
     Identity_Assumed_Friendly);  --  9
  for Standard_Identity use
    (Identity_Unspecified       => 0,
     Identity_Unknown           => 1,
     Identity_Friendly          => 2,
     Identity_Hostile           => 3,
     Identity_Suspect           => 4,
     Identity_Neutral           => 5,
     Identity_Pending           => 6,
     Identity_Joker             => 7,
     Identity_Faker             => 8,
     Identity_Assumed_Friendly  => 9);

  -- -- common.proto: BattleDimension (standard ordinals) ----------------------
  --  Ordinals match pyramid.data_model.common.BattleDimension proto values.
  --  NOTE: standard ordinals differ from legacy internal proto.
  --        Ground=1, Subsurface=2, Sea_Surface=3, Air=4, Unknown=5.

  type Battle_Dimension is
    (Dimension_Unspecified,  --  0
     Ground,                  --  1
     Subsurface,              --  2
     Sea_Surface,             --  3
     Air,                     --  4
     Dimension_Unknown);      --  5
  for Battle_Dimension use
    (Dimension_Unspecified => 0,
     Ground      => 1,
     Subsurface  => 2,
     Sea_Surface => 3,
     Air         => 4,
     Dimension_Unknown => 5);

  -- -- common.proto: DataPolicy ------------------------------------------------

  type Data_Policy is
    (Policy_Unspecified,   --  0
     Policy_Query,         --  1  -- query existing data only
     Policy_Obtain);       --  2  -- actively collect new data
  for Data_Policy use
    (Policy_Unspecified => 0,
     Policy_Query       => 1,
     Policy_Obtain      => 2);

  -- -- common.proto: GeodeticPosition -----------------------------------------
  --  Geodetic coordinates using Angle (radians).  Internally we store as
  --  doubles (radians) matching the proto Angle.radians field.

  type Geodetic_Position is record
    Latitude  : Interfaces.C.double := 0.0;  -- radians
    Longitude : Interfaces.C.double := 0.0;  -- radians
  end record;

  -- -- common.proto: PolyArea / CircleArea / Point ----------------------------

  Max_Poly_Points : constant := 64;
  type Poly_Points_Array is array (1 .. Max_Poly_Points) of Geodetic_Position;

  type Poly_Area is record
    Points : Poly_Points_Array;
    Count  : Natural := 0;
  end record;

  type Circle_Area is record
    Position : Geodetic_Position;
    Radius_M : Interfaces.C.double := 0.0;  -- meters
  end record;

  -- -- common.proto: area oneof discriminant ----------------------------------

  type Area_Kind is (Area_None, Area_Poly, Area_Circle, Area_Point);

  type Location_Area is record
    Kind   : Area_Kind         := Area_None;
    Poly   : Poly_Area;
    Circle : Circle_Area;
    Point  : Geodetic_Position;
  end record;

  -- -- tactical.proto: ObjectSource --------------------------------------------

  type Object_Source is
    (Source_Unspecified,  --  0
     Source_Radar,        --  1
     Source_Local);       --  2
  for Object_Source use
    (Source_Unspecified => 0,
     Source_Radar       => 1,
     Source_Local       => 2);

  -- -- tactical.proto: ObjectDetail -------------------------------------------
  --  Maps to pyramid.data_model.tactical.ObjectDetail.

  type Object_Detail is record
    Id          : Identifier;
    Source_Id   : Identifier;            --  common.Entity.source
    Position    : Geodetic_Position;
    Quality     : Interfaces.C.double    := 0.0;
    Has_Quality : Boolean                := False;
    Course      : Interfaces.C.double    := 0.0;  -- radians
    Has_Course  : Boolean                := False;
    Speed_Mps   : Interfaces.C.double    := 0.0;  -- m/s
    Has_Speed   : Boolean                := False;
    Length_M    : Interfaces.C.double    := 0.0;  -- meters
    Has_Length  : Boolean                := False;
    Identity    : Standard_Identity      := Identity_Unspecified;
    Dimension   : Battle_Dimension       := Dimension_Unspecified;
    Obj_Source  : Object_Source          := Source_Unspecified;
  end record;

  -- -- tactical.proto: ObjectMatch --------------------------------------------
  --  Reference type -- just an entity ID pair.

  type Object_Match is record
    Match_Id           : Identifier;  --  common.Entity.id
    Matching_Object_Id : Identifier;  --  the correlated tactical object
  end record;

  -- Named array types for dimension filters
  Max_Req_Dimensions : constant := 8;
  type Dimension_Filter is array (1 .. Max_Req_Dimensions) of Battle_Dimension;

  -- -- tactical.proto: ObjectInterestRequirement ------------------------------
  --  Maps to pyramid.data_model.tactical.ObjectInterestRequirement.

  type Object_Interest_Requirement is record
    Id             : Identifier;
    Source         : Object_Source   := Source_Unspecified;
    Has_Source     : Boolean         := False;
    Policy         : Data_Policy     := Policy_Unspecified;
    Location       : Location_Area;
    Dimension_Count: Natural         := 0;
    Dimensions     : Dimension_Filter := (others => Dimension_Unspecified);
  end record;

  -- -- tactical.proto: ObjectEvidenceRequirement ------------------------------
  --  Maps to pyramid.data_model.tactical.ObjectEvidenceRequirement.

  type Object_Evidence_Requirement is record
    Id             : Identifier;
    Policy         : Data_Policy     := Policy_Unspecified;
    Location       : Location_Area;
    Dimension_Count: Natural         := 0;
    Dimensions     : Dimension_Filter := (others => Dimension_Unspecified);
  end record;

  -- =========================================================================
  --  Legacy types: kept for Streaming_Codec wire-format decode compatibility.
  --  C++ StreamingCodec ordinals differ from standard proto ordinals for
  --  Affiliation and ObjectType.
  -- =========================================================================

  --  Affiliation — ordinal matches C++ StreamingCodec.affiliationToOrdinal
  type Affiliation is
    (Affiliation_Unspecified,  --  0
     Friendly,                  --  1
     Hostile,                   --  2
     Neutral,                   --  3
     Unknown,                   --  4
     Assumed_Friend,            --  5
     Suspect,                   --  6
     Joker,                     --  7
     Faker,                     --  8
     Pending);                  --  9
  for Affiliation use
    (Affiliation_Unspecified => 0,
     Friendly      => 1,
     Hostile       => 2,
     Neutral       => 3,
     Unknown       => 4,
     Assumed_Friend => 5,
     Suspect       => 6,
     Joker         => 7,
     Faker         => 8,
     Pending       => 9);

  --  ObjectType — ordinal matches C++ StreamingCodec.objectTypeToOrdinal
  type Object_Type is
    (Object_Type_Unspecified,  --  0
     Platform,                  --  1
     Person,                    --  2
     Equipment,                 --  3
     Unit,                      --  4
     Formation,                 --  5
     Installation,              --  6
     Feature,                   --  7
     Route,                     --  8
     Point,                     --  9
     Area,                      --  10
     Zone);                     --  11
  for Object_Type use
    (Object_Type_Unspecified => 0,
     Platform     => 1,
     Person       => 2,
     Equipment    => 3,
     Unit         => 4,
     Formation    => 5,
     Installation => 6,
     Feature      => 7,
     Route        => 8,
     Point        => 9,
     Area         => 10,
     Zone         => 11);

  type Lifecycle_Status is
    (Lifecycle_Unspecified,  --  0
     Active,                  --  1
     Stale,                   --  2
     Retired);                --  3
  for Lifecycle_Status use
    (Lifecycle_Unspecified => 0,
     Active   => 1,
     Stale    => 2,
     Retired  => 3);

  --  Optional wrappers
  type Optional_Object_Type is record
    Has   : Boolean     := False;
    Value : Object_Type := Object_Type_Unspecified;
  end record;

  type Optional_Affiliation is record
    Has   : Boolean     := False;
    Value : Affiliation := Affiliation_Unspecified;
  end record;

  type Optional_Battle_Dimension is record
    Has   : Boolean          := False;
    Value : Battle_Dimension := Dimension_Unspecified;
  end record;

  type Optional_Double is record
    Has   : Boolean             := False;
    Value : Interfaces.C.double := 0.0;
  end record;

  type Optional_String is record
    Has   : Boolean          := False;
    Value : Unbounded_String := Null_Unbounded_String;
  end record;

  --  QueryMode — matches C++ QueryMode enum
  type Query_Mode is (Read_Current, Active_Find);

  type Optional_Query_Mode is record
    Has   : Boolean    := False;
    Value : Query_Mode := Read_Current;
  end record;

  --  Geometry: bounding box (legacy, degrees)
  type Bounding_Box is record
    Min_Lat : Interfaces.C.double := 0.0;
    Max_Lat : Interfaces.C.double := 0.0;
    Min_Lon : Interfaces.C.double := 0.0;
    Max_Lon : Interfaces.C.double := 0.0;
  end record;

  type Optional_Bounding_Box is record
    Has   : Boolean     := False;
    Value : Bounding_Box;
  end record;

  --  Legacy Position / Velocity (lat/lon in degrees)
  type Position is record
    Lat : Interfaces.C.double := 0.0;
    Lon : Interfaces.C.double := 0.0;
    Alt : Interfaces.C.double := 0.0;
  end record;

  type Velocity is record
    North : Interfaces.C.double := 0.0;
    East  : Interfaces.C.double := 0.0;
    Down  : Interfaces.C.double := 0.0;
  end record;

  --  Legacy Tactical_Object_Query (used by existing e2e clients)
  type Tactical_Object_Query is record
    One_Shot            : Boolean              := True;
    Mode                : Optional_Query_Mode;
    By_Type             : Optional_Object_Type;
    By_Affiliation      : Optional_Affiliation;
    By_Battle_Dimension : Optional_Battle_Dimension;
    By_Region           : Optional_Bounding_Box;
    Max_Age_Seconds     : Optional_Double;
    By_Source_System    : Optional_String;
    By_Source_Entity_Id : Optional_String;
  end record;

  --  Legacy Tactical_Object entity (decoded from streaming wire format)
  type Tactical_Object is record
    Id               : Identifier;
    Obj_Type         : Object_Type         := Object_Type_Unspecified;
    Pos              : Position;
    Vel              : Velocity;
    Affil            : Affiliation         := Affiliation_Unspecified;
    Identity_Name    : Unbounded_String    := Null_Unbounded_String;
    Version          : Interfaces.C.unsigned_long := 0;
    Status           : Lifecycle_Status    := Lifecycle_Unspecified;
    Operational_State: Unbounded_String    := Null_Unbounded_String;
    Behavior_Pattern : Unbounded_String    := Null_Unbounded_String;
    Has_Position     : Boolean             := False;
    Has_Velocity     : Boolean             := False;
  end record;

  --  Legacy Zone types
  type Zone_Type_Enum is
    (Zone_Type_Unspecified,
     Zone_Type_Aoi,
     Zone_Type_Patrol_Area,
     Zone_Type_Restricted_Area,
     Zone_Type_No_Go_Area,
     Zone_Type_Kill_Box,
     Zone_Type_Phase_Line,
     Zone_Type_Boundary,
     Zone_Type_Route_Corridor,
     Zone_Type_Sensor_Coverage_Area);

  type Zone_Entity is record
    Id           : Identifier;
    Zone_Type_V  : Zone_Type_Enum          := Zone_Type_Unspecified;
    Active_From  : Interfaces.C.double     := 0.0;
    Active_Until : Interfaces.C.double     := 0.0;
    Priority     : Integer                 := 0;
    Owner        : Unbounded_String        := Null_Unbounded_String;
    Semantics    : Unbounded_String        := Null_Unbounded_String;
  end record;

  type Zone_Query is record
    One_Shot      : Boolean       := True;
    By_Zone_Type  : Zone_Type_Enum := Zone_Type_Unspecified;
    Has_Zone_Type : Boolean       := False;
  end record;

  --  Legacy Observation entity
  type Observation is record
    Observation_Id      : Identifier;
    Received_At         : Interfaces.C.double  := 0.0;
    Observed_At         : Interfaces.C.double  := 0.0;
    Object_Hint_Type    : Object_Type          := Object_Type_Unspecified;
    Pos                 : Position;
    Vel                 : Velocity;
    Affiliation_Hint    : Affiliation          := Affiliation_Unspecified;
    Confidence          : Interfaces.C.double  := 0.0;
    Uncertainty_Radius_M: Interfaces.C.double  := 0.0;
  end record;

end Tactical_Objects_Types;
