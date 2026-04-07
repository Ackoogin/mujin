-- streaming_codec.adb
-- Decode-only implementation for TacticalObjects binary batch frames.

with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces;
with System;
with System.Storage_Elements;

package body Streaming_Codec is
  use type Interfaces.C.unsigned;
  use type Interfaces.C.unsigned_short;
  use type Interfaces.Unsigned_64;
  use type Interfaces.Unsigned_32;
  use type Interfaces.Unsigned_16;
  use type System.Storage_Elements.Storage_Offset;

  subtype U8  is Interfaces.Unsigned_8;
  subtype U16 is Interfaces.Unsigned_16;
  subtype U32 is Interfaces.Unsigned_32;
  subtype U64 is Interfaces.Unsigned_64;

  type Byte_Access is access all Byte;
  pragma Convention(C, Byte_Access);

  type Byte_Array_Access is access all Byte_Array;

  function U64_To_Double is new Ada.Unchecked_Conversion
    (Source => U64, Target => Interfaces.C.double);

  -- Read a single byte at offset Off from buffer at Data.
  -- Returns False if out of bounds.
  function Read_U8
    (Data : System.Address;
     Len  : Natural;
     Off  : in out Natural;
     Val  : out Byte) return Boolean
  is
    use System.Storage_Elements;
    B : Byte;
    for B'Address use Data + Storage_Offset(Off);
    pragma Import(Ada, B);
  begin
    if Off + 1 > Len then
      Val := 0;
      return False;
    end if;
    Val := B;
    Off := Off + 1;
    return True;
  end Read_U8;

  function Read_U16_LE
    (Data : System.Address;
     Len  : Natural;
     Off  : in out Natural;
     Val  : out U16) return Boolean
  is
    use System.Storage_Elements;
    B0, B1 : Byte;
  begin
    if Off + 2 > Len then
      Val := 0;
      return False;
    end if;
    declare
      A0 : Byte;
      for A0'Address use Data + Storage_Offset(Off);
      pragma Import(Ada, A0);
      A1 : Byte;
      for A1'Address use Data + Storage_Offset(Off + 1);
      pragma Import(Ada, A1);
    begin
      B0 := A0;
      B1 := A1;
    end;
    Val := U16(B0) or (U16(B1) * 256);
    Off := Off + 2;
    return True;
  end Read_U16_LE;

  function Read_U32_LE
    (Data : System.Address;
     Len  : Natural;
     Off  : in out Natural;
     Val  : out U32) return Boolean
  is
    use System.Storage_Elements;
  begin
    if Off + 4 > Len then
      Val := 0;
      return False;
    end if;
    declare
      Result : U32 := 0;
    begin
      for I in 0 .. 3 loop
        declare
          B : Byte;
          for B'Address use Data + Storage_Offset(Off + I);
          pragma Import(Ada, B);
        begin
          Result := Result or (U32(B) * (2 ** (8 * I)));
        end;
      end loop;
      Val := Result;
    end;
    Off := Off + 4;
    return True;
  end Read_U32_LE;

  function Read_U64_LE
    (Data : System.Address;
     Len  : Natural;
     Off  : in out Natural;
     Val  : out U64) return Boolean
  is
    use System.Storage_Elements;
  begin
    if Off + 8 > Len then
      Val := 0;
      return False;
    end if;
    declare
      Result : U64 := 0;
    begin
      for I in 0 .. 7 loop
        declare
          B : Byte;
          for B'Address use Data + Storage_Offset(Off + I);
          pragma Import(Ada, B);
        begin
          Result := Result or (U64(B) * (2 ** (8 * I)));
        end;
      end loop;
      Val := Result;
    end;
    Off := Off + 8;
    return True;
  end Read_U64_LE;

  function Read_F64_LE
    (Data : System.Address;
     Len  : Natural;
     Off  : in out Natural;
     Val  : out Interfaces.C.double) return Boolean
  is
    Bits : U64;
  begin
    if not Read_U64_LE(Data, Len, Off, Bits) then
      Val := 0.0;
      return False;
    end if;
    Val := U64_To_Double(Bits);
    return True;
  end Read_F64_LE;

  -- Skip a string16 field (2-byte length prefix + string bytes)
  function Skip_String16
    (Data : System.Address;
     Len  : Natural;
     Off  : in out Natural) return Boolean
  is
    Slen : U16;
  begin
    if not Read_U16_LE(Data, Len, Off, Slen) then
      return False;
    end if;
    if Off + Natural(Slen) > Len then
      return False;
    end if;
    Off := Off + Natural(Slen);
    return True;
  end Skip_String16;

  -- Decode a single inner frame starting at Data + Frame_Off,
  -- with Frame_Len bytes available.
  function Decode_Inner_Frame
    (Data      : System.Address;
     Frame_Off : Natural;
     Frame_Len : Natural) return Entity_Update_Frame
  is
    use System.Storage_Elements;
    Frame     : Entity_Update_Frame;
    Inner_Addr : constant System.Address :=
      Data + Storage_Offset(Frame_Off);
    Off       : Natural := 0;
    Msg_Type  : Byte;
    Mask      : U16;
    V64       : U64;
    Dummy_Byte : Byte;
  begin
    -- msg_type (1 byte)
    if not Read_U8(Inner_Addr, Frame_Len, Off, Msg_Type) then
      return Frame;
    end if;
    Frame.Message_Type := Msg_Type;

    -- entity UUID (16 bytes)
    if Off + 16 > Frame_Len then
      Frame.Message_Type := 0;
      return Frame;
    end if;
    for I in 0 .. 15 loop
      declare
        B : Byte;
        for B'Address use Inner_Addr + Storage_Offset(Off + I);
        pragma Import(Ada, B);
      begin
        Frame.Entity_Id(I) := B;
      end;
    end loop;
    Off := Off + 16;

    -- version (8 bytes LE)
    if not Read_U64_LE(Inner_Addr, Frame_Len, Off, V64) then
      Frame.Message_Type := 0;
      return Frame;
    end if;
    Frame.Version := Interfaces.C.unsigned_long(V64);

    -- timestamp (8 bytes LE, IEEE 754 double)
    if not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Timestamp) then
      Frame.Message_Type := 0;
      return Frame;
    end if;

    -- Delete frames (0x02) have no field_mask or payload
    if Msg_Type = 16#02# then
      return Frame;
    end if;

    -- field_mask (2 bytes LE)
    if not Read_U16_LE(Inner_Addr, Frame_Len, Off, Mask) then
      Frame.Message_Type := 0;
      return Frame;
    end if;
    Frame.Field_Mask := Interfaces.C.unsigned_short(Mask);

    -- Decode fields based on mask bits
    if (Mask and U16(FIELD_POSITION)) /= 0 then
      Frame.Has_Position := True;
      if not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Pos.Lat) or else
         not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Pos.Lon) or else
         not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Pos.Alt) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    if (Mask and U16(FIELD_VELOCITY)) /= 0 then
      Frame.Has_Velocity := True;
      if not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Vel.North) or else
         not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Vel.East) or else
         not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Vel.Down) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    if (Mask and U16(FIELD_AFFILIATION)) /= 0 then
      Frame.Has_Affiliation := True;
      if not Read_U8(Inner_Addr, Frame_Len, Off, Frame.Affiliation_Val) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    if (Mask and U16(FIELD_OBJECT_TYPE)) /= 0 then
      Frame.Has_Object_Type := True;
      if not Read_U8(Inner_Addr, Frame_Len, Off, Frame.Object_Type_Val) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    if (Mask and U16(FIELD_CONFIDENCE)) /= 0 then
      Frame.Has_Confidence := True;
      if not Read_F64_LE(Inner_Addr, Frame_Len, Off, Frame.Confidence_Val) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    if (Mask and U16(FIELD_LIFECYCLE_STATUS)) /= 0 then
      Frame.Has_Lifecycle := True;
      if not Read_U8(Inner_Addr, Frame_Len, Off, Frame.Lifecycle_Val) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    -- MIL_CLASS: 6 ordinal bytes + 2 string16 fields — skip for now
    if (Mask and U16(FIELD_MIL_CLASS)) /= 0 then
      for I in 1 .. 6 loop
        if not Read_U8(Inner_Addr, Frame_Len, Off, Dummy_Byte) then
          Frame.Message_Type := 0;
          return Frame;
        end if;
      end loop;
      if not Skip_String16(Inner_Addr, Frame_Len, Off) or else
         not Skip_String16(Inner_Addr, Frame_Len, Off) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    -- BEHAVIOR: 2 string16 fields — skip
    if (Mask and U16(FIELD_BEHAVIOR)) /= 0 then
      if not Skip_String16(Inner_Addr, Frame_Len, Off) or else
         not Skip_String16(Inner_Addr, Frame_Len, Off) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    -- IDENTITY_NAME: 1 string16 field — skip
    if (Mask and U16(FIELD_IDENTITY_NAME)) /= 0 then
      if not Skip_String16(Inner_Addr, Frame_Len, Off) then
        Frame.Message_Type := 0;
        return Frame;
      end if;
    end if;

    return Frame;
  end Decode_Inner_Frame;

  function Decode_Batch
    (Data : System.Address;
     Len  : Interfaces.C.unsigned) return Decode_Result
  is
    Result    : Decode_Result;
    Off       : Natural := 0;
    Total_Len : constant Natural := Natural(Len);
    Frame_Type : Byte;
    Count_U32  : U32;
    Tick_Ts    : Interfaces.C.double;
    Frame_Size : U32;
    Frame      : Entity_Update_Frame;
  begin
    -- Batch header: type(1)=0x03 + entity_count(4) + tick_timestamp(8)
    if not Read_U8(Data, Total_Len, Off, Frame_Type) then
      return Result;
    end if;
    if Frame_Type /= 16#03# then
      return Result;
    end if;

    if not Read_U32_LE(Data, Total_Len, Off, Count_U32) then
      return Result;
    end if;

    if not Read_F64_LE(Data, Total_Len, Off, Tick_Ts) then
      return Result;
    end if;

    for I in 0 .. Natural(Count_U32) - 1 loop
      exit when Result.Count >= Max_Entities_Per_Batch;

      if not Read_U32_LE(Data, Total_Len, Off, Frame_Size) then
        exit;
      end if;
      if Off + Natural(Frame_Size) > Total_Len then
        exit;
      end if;

      Frame := Decode_Inner_Frame(Data, Off, Natural(Frame_Size));
      Off := Off + Natural(Frame_Size);

      if Frame.Message_Type /= 0 then
        Result.Frames(Result.Count) := Frame;
        Result.Count := Result.Count + 1;
      end if;
    end loop;

    return Result;
  end Decode_Batch;

end Streaming_Codec;
