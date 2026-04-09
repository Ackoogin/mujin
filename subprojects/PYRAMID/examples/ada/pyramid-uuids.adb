with Ada.Characters.Handling;
with Ada.Numerics.Discrete_Random;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces;

package body Pyramid.UUIDs is
   use type Interfaces.Unsigned_8;
   use type Interfaces.Unsigned_32;
   use type Interfaces.Unsigned_64;

   subtype Word is Interfaces.Unsigned_32;

   package Word_Random is new Ada.Numerics.Discrete_Random (Word);
   Generator : Word_Random.Generator;

   function Hex_Value (Value : Character) return Integer is
      Lower : constant Character := Ada.Characters.Handling.To_Lower (Value);
   begin
      if Value in '0' .. '9' then
         return Character'Pos (Value) - Character'Pos ('0');
      elsif Lower in 'a' .. 'f' then
         return Character'Pos (Lower) - Character'Pos ('a') + 10;
      else
         return -1;
      end if;
   end Hex_Value;

   function Hex_Character (Value : Byte) return Character is
   begin
      if Value < 10 then
         return Character'Val (Character'Pos ('0') + Integer (Value));
      else
         return Character'Val (Character'Pos ('a') + Integer (Value) - 10);
      end if;
   end Hex_Character;

   function SHA1 (Value : Byte_Array) return Byte_Array is
      type Word_Array is array (Natural range <>) of Word;

      function Rotate_Left (Value : Word; Amount : Natural) return Word is
      begin
         return Interfaces.Shift_Left (Value, Amount) or
           Interfaces.Shift_Right (Value, 32 - Amount);
      end Rotate_Left;

      function To_Word (Data : Byte_Array; Start : Natural) return Word is
      begin
         return Interfaces.Shift_Left (Word (Data (Start)), 24) or
           Interfaces.Shift_Left (Word (Data (Start + 1)), 16) or
           Interfaces.Shift_Left (Word (Data (Start + 2)), 8) or
           Word (Data (Start + 3));
      end To_Word;

      Message_Bits    : constant Long_Long_Integer :=
        Long_Long_Integer (Value'Length) * 8;
      Block_Count     : constant Natural := ((Value'Length + 9 + 63) / 64);
      Padded_Length   : constant Natural := Block_Count * 64;
      Padded_Message  : Byte_Array (0 .. Padded_Length - 1) := (others => 0);
      Schedule        : Word_Array (0 .. 79);
      H0              : Word := 16#67452301#;
      H1              : Word := 16#EFCDAB89#;
      H2              : Word := 16#98BADCFE#;
      H3              : Word := 16#10325476#;
      H4              : Word := 16#C3D2E1F0#;
   begin
      for I in Value'Range loop
         Padded_Message (I - Value'First) := Value (I);
      end loop;

      Padded_Message (Value'Length) := 16#80#;

      for Offset in 0 .. 7 loop
         Padded_Message (Padded_Message'Last - 7 + Offset) :=
           Byte
             (Interfaces.Shift_Right
                (Interfaces.Unsigned_64 (Message_Bits), (7 - Offset) * 8) and
              16#FF#);
      end loop;

      for Block in 0 .. Block_Count - 1 loop
         declare
            Base : constant Natural := Block * 64;
         begin
            for I in 0 .. 15 loop
               Schedule (I) := To_Word (Padded_Message, Base + I * 4);
            end loop;

            for I in 16 .. 79 loop
               Schedule (I) :=
                 Rotate_Left
                   (Schedule (I - 3) xor Schedule (I - 8) xor
                    Schedule (I - 14) xor Schedule (I - 16),
                    1);
            end loop;

            declare
               A : Word := H0;
               B : Word := H1;
               C : Word := H2;
               D : Word := H3;
               E : Word := H4;
               F : Word;
               K : Word;
               T : Word;
            begin
               for I in 0 .. 79 loop
                  if I <= 19 then
                     F := (B and C) or ((not B) and D);
                     K := 16#5A827999#;
                  elsif I <= 39 then
                     F := B xor C xor D;
                     K := 16#6ED9EBA1#;
                  elsif I <= 59 then
                     F := (B and C) or (B and D) or (C and D);
                     K := 16#8F1BBCDC#;
                  else
                     F := B xor C xor D;
                     K := 16#CA62C1D6#;
                  end if;

                  T := Rotate_Left (A, 5) + F + E + K + Schedule (I);
                  E := D;
                  D := C;
                  C := Rotate_Left (B, 30);
                  B := A;
                  A := T;
               end loop;

               H0 := H0 + A;
               H1 := H1 + B;
               H2 := H2 + C;
               H3 := H3 + D;
               H4 := H4 + E;
            end;
         end;
      end loop;

      declare
         Result : Byte_Array (0 .. 19) := (others => 0);
         State  : constant Word_Array (0 .. 4) := (H0, H1, H2, H3, H4);
      begin
         for I in State'Range loop
            Result (I * 4) :=
              Byte (Interfaces.Shift_Right (State (I), 24) and 16#FF#);
            Result (I * 4 + 1) :=
              Byte (Interfaces.Shift_Right (State (I), 16) and 16#FF#);
            Result (I * 4 + 2) :=
              Byte (Interfaces.Shift_Right (State (I), 8) and 16#FF#);
            Result (I * 4 + 3) := Byte (State (I) and 16#FF#);
         end loop;
         return Result;
      end;
   end SHA1;

   function Is_Null (Value : UUID) return Boolean is
   begin
      return Value = Null_UUID;
   end Is_Null;

   function To_String (Value : UUID) return String is
      Result : String (1 .. 36);
      Index  : Positive := Result'First;
   begin
      for I in Value.Bytes'Range loop
         if I = 4 or else I = 6 or else I = 8 or else I = 10 then
            Result (Index) := '-';
            Index := Index + 1;
         end if;

         Result (Index) := Hex_Character (Interfaces.Shift_Right (Value.Bytes (I), 4));
         Result (Index + 1) := Hex_Character (Value.Bytes (I) and 16#0F#);
         Index := Index + 2;
      end loop;

      return Result;
   end To_String;

   function To_Identifier (Value : UUID) return Identifier is
   begin
      return To_Unbounded_String (To_String (Value));
   end To_Identifier;

   function To_Bytes (Value : UUID) return Byte_Array is
   begin
      return Value.Bytes;
   end To_Bytes;

   function From_Bytes (Value : Byte_Array) return UUID is
      Result : UUID := Null_UUID;
   begin
      if Value'Length /= 16 then
         raise Invalid_UUID with "UUID byte array must contain exactly 16 bytes";
      end if;

      for I in 0 .. 15 loop
         Result.Bytes (I) := Value (Value'First + I);
      end loop;
      return Result;
   end From_Bytes;

   function Try_Parse
     (Value  : String;
      Result : out UUID) return Boolean
   is
      Byte_Index  : Natural := 0;
      High_Nibble : Boolean := True;
      Hex_Count   : Natural := 0;
      Nibble      : Integer;
   begin
      Result := Null_UUID;

      for C of Value loop
         if C = '-' then
            if not High_Nibble then
               return False;
            end if;

            if Hex_Count /= 8 and then Hex_Count /= 12 and then
              Hex_Count /= 16 and then Hex_Count /= 20
            then
               return False;
            end if;
         else
            Nibble := Hex_Value (C);
            if Nibble < 0 or else Byte_Index > 15 then
               return False;
            end if;

            if High_Nibble then
               Result.Bytes (Byte_Index) := Byte (Nibble * 16);
            else
               Result.Bytes (Byte_Index) :=
                 Result.Bytes (Byte_Index) or Byte (Nibble);
               Byte_Index := Byte_Index + 1;
            end if;

            High_Nibble := not High_Nibble;
            Hex_Count := Hex_Count + 1;
         end if;
      end loop;

      return Hex_Count = 32 and then High_Nibble;
   end Try_Parse;

   function Try_Parse
     (Value  : Identifier;
      Result : out UUID) return Boolean
   is
   begin
      return Try_Parse (To_String (Value), Result);
   end Try_Parse;

   function Parse (Value : String) return UUID is
      Result : UUID;
   begin
      if not Try_Parse (Value, Result) then
         raise Invalid_UUID with "Invalid UUID string: " & Value;
      end if;
      return Result;
   end Parse;

   function Parse (Value : Identifier) return UUID is
   begin
      return Parse (To_String (Value));
   end Parse;

   function Is_Valid (Value : String) return Boolean is
      Result : UUID;
   begin
      return Try_Parse (Value, Result);
   end Is_Valid;

   function Is_Valid (Value : Identifier) return Boolean is
      Result : UUID;
   begin
      return Try_Parse (Value, Result);
   end Is_Valid;

   function Generate_V4 return UUID is
      Result    : UUID := Null_UUID;
      Random_32 : Word;
   begin
      for Chunk in 0 .. 3 loop
         Random_32 := Word_Random.Random (Generator);
         for Offset in 0 .. 3 loop
            Result.Bytes (Chunk * 4 + Offset) :=
              Byte
                (Interfaces.Shift_Right
                   (Random_32, Offset * 8) and 16#FF#);
         end loop;
      end loop;

      Result.Bytes (6) := (Result.Bytes (6) and 16#0F#) or 16#40#;
      Result.Bytes (8) := (Result.Bytes (8) and 16#3F#) or 16#80#;
      return Result;
   end Generate_V4;

   function Generate_V4_Identifier return Identifier is
   begin
      return To_Identifier (Generate_V4);
   end Generate_V4_Identifier;

   function Generate_V5
     (Namespace_UUID : UUID;
      Name           : String) return UUID
   is
      Data   : Byte_Array (0 .. 16 + Name'Length - 1);
      Result : UUID := Null_UUID;
   begin
      for I in Namespace_UUID.Bytes'Range loop
         Data (I) := Namespace_UUID.Bytes (I);
      end loop;

      for I in Name'Range loop
         Data (16 + (I - Name'First)) := Byte (Character'Pos (Name (I)));
      end loop;

      declare
         Digest : constant Byte_Array := SHA1 (Data);
      begin
         for I in 0 .. 15 loop
            Result.Bytes (I) := Digest (I);
         end loop;
      end;

      Result.Bytes (6) := (Result.Bytes (6) and 16#0F#) or 16#50#;
      Result.Bytes (8) := (Result.Bytes (8) and 16#3F#) or 16#80#;
      return Result;
   end Generate_V5;

   function Generate_V5
     (Namespace_ID : Identifier;
      Name         : String) return Identifier
   is
   begin
      return To_Identifier (Generate_V5 (Parse (Namespace_ID), Name));
   end Generate_V5;

begin
   Word_Random.Reset (Generator);
end Pyramid.UUIDs;
