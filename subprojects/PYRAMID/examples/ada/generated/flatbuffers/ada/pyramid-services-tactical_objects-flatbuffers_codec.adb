--  Auto-generated tactical service FlatBuffers codec

with Interfaces; use Interfaces;

package body Pyramid.Services.Tactical_Objects.Flatbuffers_Codec is
   function Round_Up_4 (Value : Natural) return Natural is
   begin
      return ((Value + 3) / 4) * 4;
   end Round_Up_4;

   procedure Put_U16_LE (Buf : in out String; Pos : Positive; Value : Unsigned_16) is
   begin
      Buf (Pos)     := Character'Val (Integer (Value mod 16#100#));
      Buf (Pos + 1) := Character'Val (Integer ((Value / 16#100#) mod 16#100#));
   end Put_U16_LE;

   procedure Put_U32_LE (Buf : in out String; Pos : Positive; Value : Unsigned_32) is
      Tmp : Unsigned_32 := Value;
   begin
      for Offset in 0 .. 3 loop
         Buf (Pos + Offset) := Character'Val (Integer (Tmp mod 16#100#));
         Tmp := Tmp / 16#100#;
      end loop;
   end Put_U32_LE;

   function Get_U16_LE (Buf : String; Pos : Positive) return Unsigned_16 is
   begin
      return Unsigned_16 (Character'Pos (Buf (Pos)))
        + Unsigned_16 (Character'Pos (Buf (Pos + 1))) * 16#100#;
   end Get_U16_LE;

   function Get_U32_LE (Buf : String; Pos : Positive) return Unsigned_32 is
      Result : Unsigned_32 := 0;
   begin
      for Offset in reverse 0 .. 3 loop
         Result := Result * 16#100#
           + Unsigned_32 (Character'Pos (Buf (Pos + Offset)));
      end loop;
      return Result;
   end Get_U32_LE;

   function Encode_Payload (Payload : String) return String is
      String_Size   : constant Natural := 4 + Payload'Length + 1;
      String_Padded : constant Natural := Round_Up_4 (String_Size);
      Root_Offset   : constant Natural := 12;
      Table_Size    : constant Natural := 8;
      String_Start  : constant Natural := Root_Offset + Table_Size;
      Result        : String (1 .. String_Start + String_Padded);
   begin
      Result := (others => Character'Val (0));
      Put_U32_LE (Result, 1, Unsigned_32 (Root_Offset));
      Put_U16_LE (Result, 5, 6);
      Put_U16_LE (Result, 7, Unsigned_16 (Table_Size));
      Put_U16_LE (Result, 9, 4);
      Put_U32_LE (Result, Root_Offset + 1, Unsigned_32 (Root_Offset - 4));
      Put_U32_LE (Result, Root_Offset + 5, Unsigned_32 (String_Start - (Root_Offset + 4)));
      Put_U32_LE (Result, String_Start + 1, Unsigned_32 (Payload'Length));
      if Payload'Length > 0 then
         Result (String_Start + 5 .. String_Start + 4 + Payload'Length) := Payload;
      end if;
      Result (String_Start + 5 + Payload'Length - 1 + 1) := Character'Val (0);
      return Result;
   end Encode_Payload;

   function Decode_Payload (Payload : String) return String is
      procedure Raise_Invalid is
      begin
         raise Constraint_Error with "Invalid tactical flatbuffers payload";
      end Raise_Invalid;
      Root_Offset  : Natural;
      Table_Pos    : Natural;
      VTable_Diff  : Natural;
      VTable_Pos   : Natural;
      Field_Offset : Natural;
      String_Pos   : Natural;
      Str_Len      : Natural;
   begin
      if Payload'Length < 20 then
         Raise_Invalid;
         return "";
      end if;
      Root_Offset := Natural (Get_U32_LE (Payload, Payload'First));
      if Root_Offset + 8 > Payload'Length then
         Raise_Invalid;
         return "";
      end if;
      Table_Pos := Payload'First + Root_Offset;
      VTable_Diff := Natural (Get_U32_LE (Payload, Table_Pos));
      if VTable_Diff = 0 or else VTable_Diff > Root_Offset then
         Raise_Invalid;
         return "";
      end if;
      VTable_Pos := Table_Pos - VTable_Diff;
      if VTable_Pos < Payload'First or else VTable_Pos + 5 > Payload'Last then
         Raise_Invalid;
         return "";
      end if;
      if Natural (Get_U16_LE (Payload, VTable_Pos)) < 6 then
         Raise_Invalid;
         return "";
      end if;
      Field_Offset := Natural (Get_U16_LE (Payload, VTable_Pos + 4));
      if Field_Offset = 0 then
         return "";
      end if;
      if Table_Pos + Field_Offset + 3 > Payload'Last then
         Raise_Invalid;
         return "";
      end if;
      String_Pos := Table_Pos + Field_Offset + Natural (Get_U32_LE (Payload, Table_Pos + Field_Offset));
      if String_Pos + 4 > Payload'Last then
         Raise_Invalid;
         return "";
      end if;
      Str_Len := Natural (Get_U32_LE (Payload, String_Pos));
      if String_Pos + 4 + Str_Len > Payload'Last then
         Raise_Invalid;
         return "";
      end if;
      if Str_Len = 0 then
         return "";
      end if;
      return Payload (String_Pos + 4 .. String_Pos + 3 + Str_Len);
   end Decode_Payload;
end Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
