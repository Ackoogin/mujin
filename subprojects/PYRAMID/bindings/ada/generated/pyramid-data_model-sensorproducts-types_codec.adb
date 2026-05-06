--  Auto-generated data model JSON codec body
--  Package: Pyramid.Data_Model.Sensorproducts.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Pyramid.Data_Model.Common.Types_Codec;
pragma Warnings (Off);

package body Pyramid.Data_Model.Sensorproducts.Types_Codec is

   function To_Json (Msg : Radar_Display_Product_Requirement) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Base));
      Comma;
      Append (Result, """status"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Status));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Radar_Display_Product_Requirement) return Radar_Display_Product_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Radar_Display_Product_Requirement;
   begin
      if Has_Field (J, "base") then
         declare
            Sub : constant String := Write (Get (J, "base"));
         begin
            Result.Base := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Sub : constant String := Write (Get (J, "status"));
         begin
            Result.Status := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Radar_Product_Requirement) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Base));
      Comma;
      Append (Result, """status"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Status));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Radar_Product_Requirement) return Radar_Product_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Radar_Product_Requirement;
   begin
      if Has_Field (J, "base") then
         declare
            Sub : constant String := Write (Get (J, "base"));
         begin
            Result.Base := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Sub : constant String := Write (Get (J, "status"));
         begin
            Result.Status := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

end Pyramid.Data_Model.Sensorproducts.Types_Codec;
