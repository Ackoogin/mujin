-- streaming_codec.ads
-- Minimal Ada decode-only codec for TacticalObjects binary batch frames.
--
-- Wire format (little-endian):
--   Batch header: type(1)=0x03 + entity_count(4 LE) + tick_timestamp(8 LE)
--   Per entity:   size(4 LE) + inner_frame
--   Inner frame:  msg_type(1) + uuid(16) + version(8 LE) + timestamp(8 LE)
--                 + field_mask(2 LE) + field payloads (variable)

with Interfaces.C;
with System;

package Streaming_Codec is

  type Byte is mod 256;
  for Byte'Size use 8;

  type Byte_Array is array (Natural range <>) of Byte;

  type UUID_Bytes is array (0 .. 15) of Byte;

  -- Field mask bit positions (matching C++ FieldMaskBit namespace)
  FIELD_POSITION        : constant := 16#0001#;
  FIELD_VELOCITY        : constant := 16#0002#;
  FIELD_AFFILIATION     : constant := 16#0004#;
  FIELD_OBJECT_TYPE     : constant := 16#0008#;
  FIELD_CONFIDENCE      : constant := 16#0010#;
  FIELD_LIFECYCLE_STATUS: constant := 16#0020#;
  FIELD_MIL_CLASS       : constant := 16#0040#;
  FIELD_BEHAVIOR        : constant := 16#0080#;
  FIELD_IDENTITY_NAME   : constant := 16#0100#;

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

  type Entity_Update_Frame is record
    Message_Type : Byte := 0;
    Entity_Id    : UUID_Bytes := (others => 0);
    Version      : Interfaces.C.unsigned_long := 0;
    Timestamp    : Interfaces.C.double := 0.0;
    Field_Mask   : Interfaces.C.unsigned_short := 0;

    Has_Position    : Boolean := False;
    Pos             : Position;

    Has_Velocity    : Boolean := False;
    Vel             : Velocity;

    Has_Affiliation : Boolean := False;
    Affiliation_Val : Byte := 0;

    Has_Object_Type : Boolean := False;
    Object_Type_Val : Byte := 0;

    Has_Confidence  : Boolean := False;
    Confidence_Val  : Interfaces.C.double := 0.0;

    Has_Lifecycle   : Boolean := False;
    Lifecycle_Val   : Byte := 0;
  end record;

  type Entity_Frame_Array is array (Natural range <>) of Entity_Update_Frame;

  Max_Entities_Per_Batch : constant := 1024;

  type Decode_Result is record
    Frames : Entity_Frame_Array (0 .. Max_Entities_Per_Batch - 1);
    Count  : Natural := 0;
  end record;

  -- Decode a batch frame from raw bytes.
  -- Data points to the buffer, Len is the number of bytes.
  -- Returns a Decode_Result with Count decoded frames.
  function Decode_Batch
    (Data : System.Address;
     Len  : Interfaces.C.unsigned) return Decode_Result;

end Streaming_Codec;
