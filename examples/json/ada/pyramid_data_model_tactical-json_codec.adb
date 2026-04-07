--  Auto-generated JSON codec — do not edit

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with GNATCOLL.JSON; use GNATCOLL.JSON;

package body Pyramid.Data_model.Tactical.Json_Codec is

   function To_String (V : Object_Source) return String is
   begin
      case V is
         when Unspecified => return "OBJECT_SOURCE_UNSPECIFIED";
         when Radar => return "OBJECT_SOURCE_RADAR";
         when Local => return "OBJECT_SOURCE_LOCAL";
      end case;
   end To_String;

   function From_String (S : String) return Object_Source is
   begin
      if S = "OBJECT_SOURCE_UNSPECIFIED" then
         return Unspecified;
      elsif S = "OBJECT_SOURCE_RADAR" then
         return Radar;
      elsif S = "OBJECT_SOURCE_LOCAL" then
         return Local;
      else
         return Unspecified;
      end if;
   end From_String;

   function To_Json (Msg : Object_Detail) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & To_Json (Msg.Base));
      --  repeated: source
      Comma;
      Append (Result, """position"":" & To_Json (Msg.Position));
      Comma;
      Append (Result, """creation_time"":" & To_Json (Msg.Creation_Time));
      Comma;
      Append (Result, """quality"":" & To_Json (Msg.Quality));
      Comma;
      Append (Result, """course"":" & To_Json (Msg.Course));
      Comma;
      Append (Result, """speed"":" & To_Json (Msg.Speed));
      Comma;
      Append (Result, """length"":" & To_Json (Msg.Length));
      Comma;
      Append (Result, """identity"":" & "\"" & To_String (Msg.Identity) & "\"" );
      Comma;
      Append (Result, """dimension"":" & "\"" & To_String (Msg.Dimension) & "\"" );
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Object_Detail is
      Result : Object_Detail;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Object_Evidence_Requirement) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & To_Json (Msg.Base));
      Comma;
      Append (Result, """policy"":" & "\"" & To_String (Msg.Policy) & "\"" );
      --  repeated: dimension
      Comma;
      Append (Result, """poly_area"":" & To_Json (Msg.Poly_Area));
      Comma;
      Append (Result, """circle_area"":" & To_Json (Msg.Circle_Area));
      Comma;
      Append (Result, """point"":" & To_Json (Msg.Point));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Object_Evidence_Requirement is
      Result : Object_Evidence_Requirement;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Object_Interest_Requirement) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & To_Json (Msg.Base));
      Comma;
      Append (Result, """source"":" & "\"" & To_String (Msg.Source) & "\"" );
      Comma;
      Append (Result, """policy"":" & "\"" & To_String (Msg.Policy) & "\"" );
      --  repeated: dimension
      Comma;
      Append (Result, """poly_area"":" & To_Json (Msg.Poly_Area));
      Comma;
      Append (Result, """circle_area"":" & To_Json (Msg.Circle_Area));
      Comma;
      Append (Result, """point"":" & To_Json (Msg.Point));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Object_Interest_Requirement is
      Result : Object_Interest_Requirement;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Object_Match) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & To_Json (Msg.Base));
      Comma;
      Append (Result, """matching_object_id"":" & To_Json (Msg.Matching_Object_Id));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Object_Match is
      Result : Object_Match;
   begin
      return Result;
   end From_Json;

end Pyramid.Data_model.Tactical.Json_Codec;
