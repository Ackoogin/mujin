--  Auto-generated JSON codec — do not edit

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with GNATCOLL.JSON; use GNATCOLL.JSON;

package body Pyramid.Data_model.Common.Json_Codec is

   function To_String (V : Feasibility) return String is
   begin
      case V is
         when Unspecified => return "FEASIBILITY_UNSPECIFIED";
         when Feasible => return "FEASIBILITY_FEASIBLE";
         when Not_Feasible => return "FEASIBILITY_NOT_FEASIBLE";
         when Partially_Feasible => return "FEASIBILITY_PARTIALLY_FEASIBLE";
         when Pending => return "FEASIBILITY_PENDING";
      end case;
   end To_String;

   function From_String (S : String) return Feasibility is
   begin
      if S = "FEASIBILITY_UNSPECIFIED" then
         return Unspecified;
      elsif S = "FEASIBILITY_FEASIBLE" then
         return Feasible;
      elsif S = "FEASIBILITY_NOT_FEASIBLE" then
         return Not_Feasible;
      elsif S = "FEASIBILITY_PARTIALLY_FEASIBLE" then
         return Partially_Feasible;
      elsif S = "FEASIBILITY_PENDING" then
         return Pending;
      else
         return Unspecified;
      end if;
   end From_String;

   function To_String (V : Progress) return String is
   begin
      case V is
         when Unspecified => return "PROGRESS_UNSPECIFIED";
         when Not_Started => return "PROGRESS_NOT_STARTED";
         when In_Progress => return "PROGRESS_IN_PROGRESS";
         when Completed => return "PROGRESS_COMPLETED";
         when Cancelled => return "PROGRESS_CANCELLED";
         when Failed => return "PROGRESS_FAILED";
      end case;
   end To_String;

   function From_String (S : String) return Progress is
   begin
      if S = "PROGRESS_UNSPECIFIED" then
         return Unspecified;
      elsif S = "PROGRESS_NOT_STARTED" then
         return Not_Started;
      elsif S = "PROGRESS_IN_PROGRESS" then
         return In_Progress;
      elsif S = "PROGRESS_COMPLETED" then
         return Completed;
      elsif S = "PROGRESS_CANCELLED" then
         return Cancelled;
      elsif S = "PROGRESS_FAILED" then
         return Failed;
      else
         return Unspecified;
      end if;
   end From_String;

   function To_String (V : Standard_Identity) return String is
   begin
      case V is
         when Unspecified => return "STANDARD_IDENTITY_UNSPECIFIED";
         when Unknown => return "STANDARD_IDENTITY_UNKNOWN";
         when Friendly => return "STANDARD_IDENTITY_FRIENDLY";
         when Hostile => return "STANDARD_IDENTITY_HOSTILE";
         when Suspect => return "STANDARD_IDENTITY_SUSPECT";
         when Neutral => return "STANDARD_IDENTITY_NEUTRAL";
         when Pending => return "STANDARD_IDENTITY_PENDING";
         when Joker => return "STANDARD_IDENTITY_JOKER";
         when Faker => return "STANDARD_IDENTITY_FAKER";
         when Assumed_Friendly => return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
      end case;
   end To_String;

   function From_String (S : String) return Standard_Identity is
   begin
      if S = "STANDARD_IDENTITY_UNSPECIFIED" then
         return Unspecified;
      elsif S = "STANDARD_IDENTITY_UNKNOWN" then
         return Unknown;
      elsif S = "STANDARD_IDENTITY_FRIENDLY" then
         return Friendly;
      elsif S = "STANDARD_IDENTITY_HOSTILE" then
         return Hostile;
      elsif S = "STANDARD_IDENTITY_SUSPECT" then
         return Suspect;
      elsif S = "STANDARD_IDENTITY_NEUTRAL" then
         return Neutral;
      elsif S = "STANDARD_IDENTITY_PENDING" then
         return Pending;
      elsif S = "STANDARD_IDENTITY_JOKER" then
         return Joker;
      elsif S = "STANDARD_IDENTITY_FAKER" then
         return Faker;
      elsif S = "STANDARD_IDENTITY_ASSUMED_FRIENDLY" then
         return Assumed_Friendly;
      else
         return Unspecified;
      end if;
   end From_String;

   function To_String (V : Battle_Dimension) return String is
   begin
      case V is
         when Unspecified => return "BATTLE_DIMENSION_UNSPECIFIED";
         when Ground => return "BATTLE_DIMENSION_GROUND";
         when Subsurface => return "BATTLE_DIMENSION_SUBSURFACE";
         when Sea_Surface => return "BATTLE_DIMENSION_SEA_SURFACE";
         when Air => return "BATTLE_DIMENSION_AIR";
         when Unknown => return "BATTLE_DIMENSION_UNKNOWN";
      end case;
   end To_String;

   function From_String (S : String) return Battle_Dimension is
   begin
      if S = "BATTLE_DIMENSION_UNSPECIFIED" then
         return Unspecified;
      elsif S = "BATTLE_DIMENSION_GROUND" then
         return Ground;
      elsif S = "BATTLE_DIMENSION_SUBSURFACE" then
         return Subsurface;
      elsif S = "BATTLE_DIMENSION_SEA_SURFACE" then
         return Sea_Surface;
      elsif S = "BATTLE_DIMENSION_AIR" then
         return Air;
      elsif S = "BATTLE_DIMENSION_UNKNOWN" then
         return Unknown;
      else
         return Unspecified;
      end if;
   end From_String;

   function To_String (V : Data_Policy) return String is
   begin
      case V is
         when Unspecified => return "DATA_POLICY_UNSPECIFIED";
         when Query => return "DATA_POLICY_QUERY";
         when Obtain => return "DATA_POLICY_OBTAIN";
      end case;
   end To_String;

   function From_String (S : String) return Data_Policy is
   begin
      if S = "DATA_POLICY_UNSPECIFIED" then
         return Unspecified;
      elsif S = "DATA_POLICY_QUERY" then
         return Query;
      elsif S = "DATA_POLICY_OBTAIN" then
         return Obtain;
      else
         return Unspecified;
      end if;
   end From_String;

   function To_Json (Msg : Geodetic_Position) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """latitude"":" & To_Json (Msg.Latitude));
      Comma;
      Append (Result, """longitude"":" & To_Json (Msg.Longitude));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Geodetic_Position is
      Result : Geodetic_Position;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Poly_Area) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      --  repeated: points
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Poly_Area is
      Result : Poly_Area;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Achievement) return String is
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
      Append (Result, """status"":" & "\"" & To_String (Msg.Status) & "\"" );
      Comma;
      Append (Result, """quality"":" & To_Json (Msg.Quality));
      Comma;
      Append (Result, """achieveability"":" & "\"" & To_String (Msg.Achieveability) & "\"" );
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Achievement is
      Result : Achievement;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Requirement) return String is
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
      Append (Result, """status"":" & To_Json (Msg.Status));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Requirement is
      Result : Requirement;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Capability) return String is
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
      Append (Result, """availability"":" & (if Msg.Availability then "true" else "false"));
      Comma;
      Append (Result, """name"":" & "\"" & To_String (Msg.Name) & "\"" );
      --  repeated: contraint
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Capability is
      Result : Capability;
   begin
      for I in S'First .. S'Last - 3 loop
         if S (I .. I + 3) = "true" then
            Result.Availability := True;
            exit;
         end if;
      end loop;
      return Result;
   end From_Json;

   function To_Json (Msg : Entity) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & To_Json (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & To_Json (Msg.Id));
      Comma;
      Append (Result, """source"":" & To_Json (Msg.Source));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Entity is
      Result : Entity;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Circle_Area) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """position"":" & To_Json (Msg.Position));
      Comma;
      Append (Result, """radius"":" & To_Json (Msg.Radius));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Circle_Area is
      Result : Circle_Area;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Point) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """position"":" & To_Json (Msg.Position));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Point is
      Result : Point;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Contraint) return String is
   begin
      return "{" &
        """name"":" & "\"" & To_String (Msg.Name) & "\""  &
        "," &
        """value"":" & Integer'Image (Msg.Value) &
        "}";
   end To_Json;

   function From_Json (S : String) return Contraint is
      Result : Contraint;
   begin
      return Result;
   end From_Json;

   function To_Json (Msg : Ack) return String is
   begin
      return "{" &
        """success"":" & (if Msg.Success then "true" else "false") &
        "}";
   end To_Json;

   function From_Json (S : String) return Ack is
      Result : Ack;
   begin
      for I in S'First .. S'Last - 3 loop
         if S (I .. I + 3) = "true" then
            Result.Success := True;
            exit;
         end if;
      end loop;
      return Result;
   end From_Json;

   function To_Json (Msg : Query) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      --  repeated: id
      Comma;
      Append (Result, """one_shot"":" & (if Msg.One_Shot then "true" else "false"));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String) return Query is
      Result : Query;
   begin
      for I in S'First .. S'Last - 3 loop
         if S (I .. I + 3) = "true" then
            Result.One_Shot := True;
            exit;
         end if;
      end loop;
      return Result;
   end From_Json;

end Pyramid.Data_model.Common.Json_Codec;
