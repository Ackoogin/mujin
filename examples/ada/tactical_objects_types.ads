--  tactical_objects_types.ads
--  Ada types mirroring pyramid/components/tactical_objects.proto.
--
--  Hand-written to be consistent with:
--    proto/pyramid/data_model/base.proto
--    proto/pyramid/components/tactical_objects.proto
--    pyramid/tactical_objects/include/TacticalObjectsTypes.h
--
--  These are value types — no heap allocation, no tagged types.
--  Enum ordinal values match the C++ codec ordinal helpers in StreamingCodec.

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;

package Tactical_Objects_Types is

  -- ── Base EntityActions types (base.proto) ─────────────────────────────────

  subtype Identifier is Unbounded_String;

  type Query is record
    One_Shot : Boolean := True;
    --  ID filter omitted: subscribe_interest uses predicate filters instead.
  end record;

  type Ack is record
    Success : Boolean := False;
  end record;

  -- ── Enumerations (matching C++ enum class + proto ordinals) ───────────────

  --  ObjectType — ordinal matches StreamingCodec.objectTypeToOrdinal
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

  --  Affiliation — ordinal matches StreamingCodec.affiliationToOrdinal
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

  -- ── Geometry types ────────────────────────────────────────────────────────

  --  Maps to proto Position / tactical_objects::Position
  type Position is record
    Lat : Interfaces.C.double := 0.0;
    Lon : Interfaces.C.double := 0.0;
    Alt : Interfaces.C.double := 0.0;
  end record;

  --  Maps to proto Velocity / tactical_objects::Velocity
  type Velocity is record
    North : Interfaces.C.double := 0.0;
    East  : Interfaces.C.double := 0.0;
    Down  : Interfaces.C.double := 0.0;
  end record;

  --  Maps to proto BoundingBox / tactical_objects::BoundingBox
  type Bounding_Box is record
    Min_Lat : Interfaces.C.double := 0.0;
    Max_Lat : Interfaces.C.double := 0.0;
    Min_Lon : Interfaces.C.double := 0.0;
    Max_Lon : Interfaces.C.double := 0.0;
  end record;

  -- ── Optional wrappers ─────────────────────────────────────────────────────
  --  Ada 2012 has no built-in optional; mirror proto "optional" fields
  --  with a Has/Value pair.

  type Optional_Object_Type is record
    Has   : Boolean     := False;
    Value : Object_Type := Object_Type_Unspecified;
  end record;

  type Optional_Affiliation is record
    Has   : Boolean     := False;
    Value : Affiliation := Affiliation_Unspecified;
  end record;

  type Optional_Bounding_Box is record
    Has   : Boolean     := False;
    Value : Bounding_Box;
  end record;

  type Optional_Double is record
    Has   : Boolean             := False;
    Value : Interfaces.C.double := 0.0;
  end record;

  type Optional_String is record
    Has   : Boolean          := False;
    Value : Unbounded_String := Null_Unbounded_String;
  end record;

  -- ── TacticalObjectQuery (maps to proto TacticalObjectQuery) ───────────────
  --  Superset of base Query with compound predicates.
  --  Corresponds to: tactical_objects::QueryRequest

  type Tactical_Object_Query is record
    One_Shot            : Boolean              := True;
    By_Type             : Optional_Object_Type;
    By_Affiliation      : Optional_Affiliation;
    By_Region           : Optional_Bounding_Box;
    Max_Age_Seconds     : Optional_Double;
    By_Source_System    : Optional_String;
    By_Source_Entity_Id : Optional_String;
  end record;

  -- ── TacticalObject entity (maps to proto TacticalObject) ─────────────────
  --  Full denormalized view — used for Create, Read, and Update responses.

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

end Tactical_Objects_Types;
