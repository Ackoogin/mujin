--  Auto-generated JSON codec — do not edit

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with GNATCOLL.JSON; use GNATCOLL.JSON;

package body Pyramid.Data_model.Base.Json_Codec is

   function To_Json (Msg : Angle) return String is
   begin
      return "{" &
        """radians"":" & Long_Float'Image (Msg.Radians) &
        "}";
   end To_Json;

   function From_Json (S : String) return Angle is
      Result : Angle;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Length) return String is
   begin
      return "{" &
        """meters"":" & Long_Float'Image (Msg.Meters) &
        "}";
   end To_Json;

   function From_Json (S : String) return Length is
      Result : Length;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Timestamp) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """value"":" & To_Json (Msg.Value));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Timestamp is
      Result : Timestamp;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Identifier) return String is
   begin
      return "{" &
        """value"":" & "\"" & To_String (Msg.Value) & "\""  &
        "}";
   end To_Json;

   function From_Json (S : String) return Identifier is
      Result : Identifier;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Speed) return String is
   begin
      return "{" &
        """meters_per_second"":" & Long_Float'Image (Msg.Meters_Per_Second) &
        "}";
   end To_Json;

   function From_Json (S : String) return Speed is
      Result : Speed;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Percentage) return String is
   begin
      return "{" &
        """value"":" & Long_Float'Image (Msg.Value) &
        "}";
   end To_Json;

   function From_Json (S : String) return Percentage is
      Result : Percentage;
   begin
      return Result;
   end From_Json;

end Pyramid.Data_model.Base.Json_Codec;
