--  Auto-generated Ada C-ABI marshalling body
--  Package body: Pyramid.Data_Model.Base.Cabi

with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;

package body Pyramid.Data_Model.Base.Cabi is
   use type Interfaces.C.unsigned;
   use type Interfaces.C.unsigned_char;
   use type Interfaces.C.Strings.chars_ptr;
   use type System.Address;

   function Malloc (Size : Interfaces.C.size_t)
     return System.Address;
   pragma Import (C, Malloc, "malloc");

   function To_Chars_Ptr is new Ada.Unchecked_Conversion
     (System.Address, Interfaces.C.Strings.chars_ptr);

   function To_Address is new Ada.Unchecked_Conversion
     (Interfaces.C.Strings.chars_ptr, System.Address);

   function Bytes_For
     (Count : Natural;
      Element_Bits : Natural) return Interfaces.C.size_t
   is
   begin
      return Interfaces.C.size_t
        (Count * (Element_Bits / System.Storage_Unit));
   end Bytes_For;

   function To_Ada_String (S : Pyramid_Str_T) return String is
   begin
      if S.Ptr = Interfaces.C.Strings.Null_Ptr or else S.Len = 0 then
         return "";
      end if;
      return Interfaces.C.Strings.Value
        (S.Ptr, Interfaces.C.size_t (S.Len));
   end To_Ada_String;

   procedure Dup_Str (Out_Value : out Pyramid_Str_T; S : String) is
   begin
      Out_Value := (Ptr => Interfaces.C.Strings.Null_Ptr, Len => 0);
      if S'Length = 0 then
         return;
      end if;
      Out_Value.Len := Interfaces.C.unsigned (S'Length);
      Out_Value.Ptr := To_Chars_Ptr
        (Malloc (Interfaces.C.size_t (Out_Value.Len)));
      declare
         type Char_Array is array (Positive range 1 .. S'Length)
           of Interfaces.C.char;
         pragma Convention (C, Char_Array);
         Chars : Char_Array;
         for Chars'Address use To_Address (Out_Value.Ptr);
         pragma Import (Ada, Chars);
      begin
         for I in S'Range loop
            Chars (I - S'First + 1) :=
              Interfaces.C.char'Val (Character'Pos (S (I)));
         end loop;
      end;
   end Dup_Str;

end Pyramid.Data_Model.Base.Cabi;
