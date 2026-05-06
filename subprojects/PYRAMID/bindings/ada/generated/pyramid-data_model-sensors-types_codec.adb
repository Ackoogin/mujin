--  Auto-generated data model JSON codec body
--  Package: Pyramid.Data_Model.Sensors.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Radar.Types_Codec;
pragma Warnings (Off);

package body Pyramid.Data_Model.Sensors.Types_Codec is

   function To_String (V : Interpretation_Policy) return String is
   begin
      case V is
         when Policy_Unspecified => return "INTERPRETATION_POLICY_UNSPECIFIED";
         when Policy_IgnoreObjects => return "INTERPRETATION_POLICY_IGNORE_OBJECTS";
         when Policy_IncludeObjects => return "INTERPRETATION_POLICY_INCLUDE_OBJECTS";
      end case;
   end To_String;

   function Interpretation_Policy_From_String (S : String) return Interpretation_Policy is
   begin
      if S = "INTERPRETATION_POLICY_UNSPECIFIED" then return Policy_Unspecified; end if;
      if S = "INTERPRETATION_POLICY_IGNORE_OBJECTS" then return Policy_IgnoreObjects; end if;
      if S = "INTERPRETATION_POLICY_INCLUDE_OBJECTS" then return Policy_IncludeObjects; end if;
      return Policy_Unspecified;
   end Interpretation_Policy_From_String;

   function To_String (V : Interpretation_Type) return String is
   begin
      case V is
         when Type_Unspecified => return "INTERPRETATION_TYPE_UNSPECIFIED";
         when Type_LocateSeaSurfaceObjects => return "INTERPRETATION_TYPE_LOCATE_SEA_SURFACE_OBJECTS";
      end case;
   end To_String;

   function Interpretation_Type_From_String (S : String) return Interpretation_Type is
   begin
      if S = "INTERPRETATION_TYPE_UNSPECIFIED" then return Type_Unspecified; end if;
      if S = "INTERPRETATION_TYPE_LOCATE_SEA_SURFACE_OBJECTS" then return Type_LocateSeaSurfaceObjects; end if;
      return Type_Unspecified;
   end Interpretation_Type_From_String;

   function To_Json (Msg : Interpretation_Requirement) return String is
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
      Comma;
      Append (Result, """policy"":" & """" & To_String (Msg.Policy) & """");
      Comma;
      Append (Result, """type"":" & """" & To_String (Msg.Type_Field) & """");
      if Msg.Has_Val_Poly_Area then
         Comma;
         Append (Result, """poly_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Poly_Area));
      end if;
      if Msg.Has_Val_Circle_Area then
         Comma;
         Append (Result, """circle_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Circle_Area));
      end if;
      if Msg.Has_Val_Point then
         Comma;
         Append (Result, """point"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Point));
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Interpretation_Requirement) return Interpretation_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Interpretation_Requirement;
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
      if Has_Field (J, "policy") then
         declare
            Val : constant JSON_Value := Get (J, "policy");
            Str : constant String := Get (Val);
         begin
            Result.Policy := Interpretation_Policy_From_String (Str);
         end;
      end if;
      if Has_Field (J, "type") then
         declare
            Val : constant JSON_Value := Get (J, "type");
            Str : constant String := Get (Val);
         begin
            Result.Type_Field := Interpretation_Type_From_String (Str);
         end;
      end if;
      if Has_Field (J, "poly_area") then
         Result.Has_Val_Poly_Area := True;
         declare
            Sub : constant String := Write (Get (J, "poly_area"));
         begin
            Result.Val_Poly_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "circle_area") then
         Result.Has_Val_Circle_Area := True;
         declare
            Sub : constant String := Write (Get (J, "circle_area"));
         begin
            Result.Val_Circle_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "point") then
         Result.Has_Val_Point := True;
         declare
            Sub : constant String := Write (Get (J, "point"));
         begin
            Result.Val_Point := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Manual_Track_Requirement) return String is
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
      Comma;
      Append (Result, """position"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Position));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Manual_Track_Requirement) return Manual_Track_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Manual_Track_Requirement;
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
      if Has_Field (J, "position") then
         declare
            Sub : constant String := Write (Get (J, "position"));
         begin
            Result.Position := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Ati_Requirement) return String is
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
      Comma;
      Append (Result, """auto_zone"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Auto_Zone));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Ati_Requirement) return Ati_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Ati_Requirement;
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
      if Has_Field (J, "auto_zone") then
         declare
            Sub : constant String := Write (Get (J, "auto_zone"));
         begin
            Result.Auto_Zone := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Track_Provision_Requirement) return String is
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
      if Msg.Has_Val_Poly_Area then
         Comma;
         Append (Result, """poly_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Poly_Area));
      end if;
      if Msg.Has_Val_Circle_Area then
         Comma;
         Append (Result, """circle_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Circle_Area));
      end if;
      if Msg.Has_Val_Point then
         Comma;
         Append (Result, """point"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Point));
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Track_Provision_Requirement) return Track_Provision_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Track_Provision_Requirement;
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
      if Has_Field (J, "poly_area") then
         Result.Has_Val_Poly_Area := True;
         declare
            Sub : constant String := Write (Get (J, "poly_area"));
         begin
            Result.Val_Poly_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "circle_area") then
         Result.Has_Val_Circle_Area := True;
         declare
            Sub : constant String := Write (Get (J, "circle_area"));
         begin
            Result.Val_Circle_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "point") then
         Result.Has_Val_Point := True;
         declare
            Sub : constant String := Write (Get (J, "point"));
         begin
            Result.Val_Point := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Object_Evidence_Provision_Requirement) return String is
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
      if Msg.Has_Val_Poly_Area then
         Comma;
         Append (Result, """poly_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Poly_Area));
      end if;
      if Msg.Has_Val_Circle_Area then
         Comma;
         Append (Result, """circle_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Circle_Area));
      end if;
      if Msg.Has_Val_Point then
         Comma;
         Append (Result, """point"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Point));
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Evidence_Provision_Requirement) return Object_Evidence_Provision_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Object_Evidence_Provision_Requirement;
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
      if Has_Field (J, "poly_area") then
         Result.Has_Val_Poly_Area := True;
         declare
            Sub : constant String := Write (Get (J, "poly_area"));
         begin
            Result.Val_Poly_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "circle_area") then
         Result.Has_Val_Circle_Area := True;
         declare
            Sub : constant String := Write (Get (J, "circle_area"));
         begin
            Result.Val_Circle_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "point") then
         Result.Has_Val_Point := True;
         declare
            Sub : constant String := Write (Get (J, "point"));
         begin
            Result.Val_Point := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Object_Aquisition_Requirement) return String is
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
      if Msg.Has_Val_Poly_Area then
         Comma;
         Append (Result, """poly_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Poly_Area));
      end if;
      if Msg.Has_Val_Circle_Area then
         Comma;
         Append (Result, """circle_area"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Circle_Area));
      end if;
      if Msg.Has_Val_Point then
         Comma;
         Append (Result, """point"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Point));
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Aquisition_Requirement) return Object_Aquisition_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Object_Aquisition_Requirement;
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
      if Has_Field (J, "poly_area") then
         Result.Has_Val_Poly_Area := True;
         declare
            Sub : constant String := Write (Get (J, "poly_area"));
         begin
            Result.Val_Poly_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "circle_area") then
         Result.Has_Val_Circle_Area := True;
         declare
            Sub : constant String := Write (Get (J, "circle_area"));
         begin
            Result.Val_Circle_Area := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "point") then
         Result.Has_Val_Point := True;
         declare
            Sub : constant String := Write (Get (J, "point"));
         begin
            Result.Val_Point := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Sensor_Object) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """position"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Position));
      Comma;
      Append (Result, """creation_time"":" & Long_Float'Image (Msg.Creation_Time));
      Comma;
      Append (Result, """quality"":" & Long_Float'Image (Msg.Quality));
      Comma;
      Append (Result, """course"":" & Long_Float'Image (Msg.Course));
      Comma;
      Append (Result, """speed"":" & Long_Float'Image (Msg.Speed));
      Comma;
      Append (Result, """length"":" & Long_Float'Image (Msg.Length));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Sensor_Object) return Sensor_Object is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Sensor_Object;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "position") then
         declare
            Sub : constant String := Write (Get (J, "position"));
         begin
            Result.Position := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "creation_time") then
         Result.Creation_Time := Get_Long_Float (Get (J, "creation_time"));
      end if;
      if Has_Field (J, "quality") then
         Result.Quality := Get_Long_Float (Get (J, "quality"));
      end if;
      if Has_Field (J, "course") then
         Result.Course := Get_Long_Float (Get (J, "course"));
      end if;
      if Has_Field (J, "speed") then
         Result.Speed := Get_Long_Float (Get (J, "speed"));
      end if;
      if Has_Field (J, "length") then
         Result.Length := Get_Long_Float (Get (J, "length"));
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Radar_Mode_Change_Requirement) return String is
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
      Comma;
      Append (Result, """mode"":" & """" & Pyramid.Data_Model.Radar.Types_Codec.To_String (Msg.Mode) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Radar_Mode_Change_Requirement) return Radar_Mode_Change_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Radar_Mode_Change_Requirement;
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
      if Has_Field (J, "mode") then
         declare
            Val : constant JSON_Value := Get (J, "mode");
            Str : constant String := Get (Val);
         begin
            Result.Mode := Pyramid.Data_Model.Radar.Types_Codec.Radar_Operational_Mode_From_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

end Pyramid.Data_Model.Sensors.Types_Codec;
