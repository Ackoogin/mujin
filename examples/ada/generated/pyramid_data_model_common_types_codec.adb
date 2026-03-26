--  Auto-generated data model JSON codec body
--  Package: Pyramid_Data_Model_Common_Types_Codec

with GNATCOLL.JSON;  use GNATCOLL.JSON;
pragma Warnings (Off);

package body Pyramid_Data_Model_Common_Types_Codec is

   function To_String (V : Feasibility) return String is
   begin
      case V is
         when Feasibility_Unspecified => return "FEASIBILITY_UNSPECIFIED";
         when Feasibility_Feasible => return "FEASIBILITY_FEASIBLE";
         when Feasibility_NotFeasible => return "FEASIBILITY_NOT_FEASIBLE";
         when Feasibility_PartiallyFeasible => return "FEASIBILITY_PARTIALLY_FEASIBLE";
         when Feasibility_Pending => return "FEASIBILITY_PENDING";
      end case;
   end To_String;

   function Feasibility_From_String (S : String) return Feasibility is
   begin
      if S = "FEASIBILITY_UNSPECIFIED" then return Feasibility_Unspecified; end if;
      if S = "FEASIBILITY_FEASIBLE" then return Feasibility_Feasible; end if;
      if S = "FEASIBILITY_NOT_FEASIBLE" then return Feasibility_NotFeasible; end if;
      if S = "FEASIBILITY_PARTIALLY_FEASIBLE" then return Feasibility_PartiallyFeasible; end if;
      if S = "FEASIBILITY_PENDING" then return Feasibility_Pending; end if;
      return Feasibility_Unspecified;
   end Feasibility_From_String;

   function To_String (V : Progress) return String is
   begin
      case V is
         when Progress_Unspecified => return "PROGRESS_UNSPECIFIED";
         when Progress_NotStarted => return "PROGRESS_NOT_STARTED";
         when Progress_InProgress => return "PROGRESS_IN_PROGRESS";
         when Progress_Completed => return "PROGRESS_COMPLETED";
         when Progress_Cancelled => return "PROGRESS_CANCELLED";
         when Progress_Failed => return "PROGRESS_FAILED";
      end case;
   end To_String;

   function Progress_From_String (S : String) return Progress is
   begin
      if S = "PROGRESS_UNSPECIFIED" then return Progress_Unspecified; end if;
      if S = "PROGRESS_NOT_STARTED" then return Progress_NotStarted; end if;
      if S = "PROGRESS_IN_PROGRESS" then return Progress_InProgress; end if;
      if S = "PROGRESS_COMPLETED" then return Progress_Completed; end if;
      if S = "PROGRESS_CANCELLED" then return Progress_Cancelled; end if;
      if S = "PROGRESS_FAILED" then return Progress_Failed; end if;
      return Progress_Unspecified;
   end Progress_From_String;

   function To_String (V : Standard_Identity) return String is
   begin
      case V is
         when Identity_Unspecified => return "STANDARD_IDENTITY_UNSPECIFIED";
         when Identity_Unknown => return "STANDARD_IDENTITY_UNKNOWN";
         when Identity_Friendly => return "STANDARD_IDENTITY_FRIENDLY";
         when Identity_Hostile => return "STANDARD_IDENTITY_HOSTILE";
         when Identity_Suspect => return "STANDARD_IDENTITY_SUSPECT";
         when Identity_Neutral => return "STANDARD_IDENTITY_NEUTRAL";
         when Identity_Pending => return "STANDARD_IDENTITY_PENDING";
         when Identity_Joker => return "STANDARD_IDENTITY_JOKER";
         when Identity_Faker => return "STANDARD_IDENTITY_FAKER";
         when Identity_AssumedFriendly => return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
      end case;
   end To_String;

   function Standard_Identity_From_String (S : String) return Standard_Identity is
   begin
      if S = "STANDARD_IDENTITY_UNSPECIFIED" then return Identity_Unspecified; end if;
      if S = "STANDARD_IDENTITY_UNKNOWN" then return Identity_Unknown; end if;
      if S = "STANDARD_IDENTITY_FRIENDLY" then return Identity_Friendly; end if;
      if S = "STANDARD_IDENTITY_HOSTILE" then return Identity_Hostile; end if;
      if S = "STANDARD_IDENTITY_SUSPECT" then return Identity_Suspect; end if;
      if S = "STANDARD_IDENTITY_NEUTRAL" then return Identity_Neutral; end if;
      if S = "STANDARD_IDENTITY_PENDING" then return Identity_Pending; end if;
      if S = "STANDARD_IDENTITY_JOKER" then return Identity_Joker; end if;
      if S = "STANDARD_IDENTITY_FAKER" then return Identity_Faker; end if;
      if S = "STANDARD_IDENTITY_ASSUMED_FRIENDLY" then return Identity_AssumedFriendly; end if;
      return Identity_Unspecified;
   end Standard_Identity_From_String;

   function To_String (V : Battle_Dimension) return String is
   begin
      case V is
         when Dimension_Unspecified => return "BATTLE_DIMENSION_UNSPECIFIED";
         when Dimension_Ground => return "BATTLE_DIMENSION_GROUND";
         when Dimension_Subsurface => return "BATTLE_DIMENSION_SUBSURFACE";
         when Dimension_SeaSurface => return "BATTLE_DIMENSION_SEA_SURFACE";
         when Dimension_Air => return "BATTLE_DIMENSION_AIR";
         when Dimension_Unknown => return "BATTLE_DIMENSION_UNKNOWN";
      end case;
   end To_String;

   function Battle_Dimension_From_String (S : String) return Battle_Dimension is
   begin
      if S = "BATTLE_DIMENSION_UNSPECIFIED" then return Dimension_Unspecified; end if;
      if S = "BATTLE_DIMENSION_GROUND" then return Dimension_Ground; end if;
      if S = "BATTLE_DIMENSION_SUBSURFACE" then return Dimension_Subsurface; end if;
      if S = "BATTLE_DIMENSION_SEA_SURFACE" then return Dimension_SeaSurface; end if;
      if S = "BATTLE_DIMENSION_AIR" then return Dimension_Air; end if;
      if S = "BATTLE_DIMENSION_UNKNOWN" then return Dimension_Unknown; end if;
      return Dimension_Unspecified;
   end Battle_Dimension_From_String;

   function To_String (V : Data_Policy) return String is
   begin
      case V is
         when Policy_Unspecified => return "DATA_POLICY_UNSPECIFIED";
         when Policy_Query => return "DATA_POLICY_QUERY";
         when Policy_Obtain => return "DATA_POLICY_OBTAIN";
      end case;
   end To_String;

   function Data_Policy_From_String (S : String) return Data_Policy is
   begin
      if S = "DATA_POLICY_UNSPECIFIED" then return Policy_Unspecified; end if;
      if S = "DATA_POLICY_QUERY" then return Policy_Query; end if;
      if S = "DATA_POLICY_OBTAIN" then return Policy_Obtain; end if;
      return Policy_Unspecified;
   end Data_Policy_From_String;

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
      Append (Result, """latitude"":" & Long_Float'Image (Msg.Latitude));
      Comma;
      Append (Result, """longitude"":" & Long_Float'Image (Msg.Longitude));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Geodetic_Position) return Geodetic_Position is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Geodetic_Position;
   begin
      if Has_Field (J, "latitude") then
         Result.Latitude := Get_Long_Float (Get (J, "latitude"));
      end if;
      if Has_Field (J, "longitude") then
         Result.Longitude := Get_Long_Float (Get (J, "longitude"));
      end if;
      return Result;
   exception
      when others => return Result;
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
      if Msg.Points /= null then
         Comma;
         Append (Result, """points"":[");
         for I in Msg.Points'Range loop
            if I > Msg.Points'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Points (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Poly_Area) return Poly_Area is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Poly_Area;
   begin
      if Has_Field (J, "points") then
         declare
            Arr : constant JSON_Value := Get (J, "points");
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Points := new Points_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Sub : constant String := Write (Get (Arr, I));
                  begin
                     Result.Points (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """ & To_String (Msg.Id) & """);
      Comma;
      Append (Result, """source"":" & """ & To_String (Msg.Source) & """);
      Comma;
      Append (Result, """status"":" & """ & To_String (Msg.Status) & """);
      Comma;
      Append (Result, """quality"":" & Long_Float'Image (Msg.Quality));
      Comma;
      Append (Result, """achieveability"":" & """ & To_String (Msg.Achieveability) & """);
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Achievement) return Achievement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Achievement;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         Result.Id := To_Unbounded_String (Get (Get (J, "id")));
      end if;
      if Has_Field (J, "source") then
         Result.Source := To_Unbounded_String (Get (Get (J, "source")));
      end if;
      if Has_Field (J, "status") then
         Result.Status := Progress_From_String (Get (Get (J, "status")));
      end if;
      if Has_Field (J, "quality") then
         Result.Quality := Get_Long_Float (Get (J, "quality"));
      end if;
      if Has_Field (J, "achieveability") then
         Result.Achieveability := Feasibility_From_String (Get (Get (J, "achieveability")));
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """ & To_String (Msg.Id) & """);
      Comma;
      Append (Result, """source"":" & """ & To_String (Msg.Source) & """);
      Comma;
      Append (Result, """status"":" & To_Json (Msg.Status));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Requirement) return Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Requirement;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         Result.Id := To_Unbounded_String (Get (Get (J, "id")));
      end if;
      if Has_Field (J, "source") then
         Result.Source := To_Unbounded_String (Get (Get (J, "source")));
      end if;
      if Has_Field (J, "status") then
         declare
            Sub : constant String := Write (Get (J, "status"));
         begin
            Result.Status := From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """ & To_String (Msg.Id) & """);
      Comma;
      Append (Result, """source"":" & """ & To_String (Msg.Source) & """);
      Comma;
      Append (Result, """availability"":" & (if Msg.Availability then "true" else "false"));
      Comma;
      Append (Result, """name"":" & """ & To_String (Msg.Name) & """);
      if Msg.Contraint /= null then
         Comma;
         Append (Result, """contraint"":[");
         for I in Msg.Contraint'Range loop
            if I > Msg.Contraint'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Contraint (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Capability) return Capability is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Capability;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         Result.Id := To_Unbounded_String (Get (Get (J, "id")));
      end if;
      if Has_Field (J, "source") then
         Result.Source := To_Unbounded_String (Get (Get (J, "source")));
      end if;
      if Has_Field (J, "availability") then
         Result.Availability := Get (Get (J, "availability"));
      end if;
      if Has_Field (J, "name") then
         Result.Name := To_Unbounded_String (Get (Get (J, "name")));
      end if;
      if Has_Field (J, "contraint") then
         declare
            Arr : constant JSON_Value := Get (J, "contraint");
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Contraint := new Contraint_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Sub : constant String := Write (Get (Arr, I));
                  begin
                     Result.Contraint (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """ & To_String (Msg.Id) & """);
      Comma;
      Append (Result, """source"":" & """ & To_String (Msg.Source) & """);
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Entity) return Entity is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Entity;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         Result.Id := To_Unbounded_String (Get (Get (J, "id")));
      end if;
      if Has_Field (J, "source") then
         Result.Source := To_Unbounded_String (Get (Get (J, "source")));
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """radius"":" & Long_Float'Image (Msg.Radius));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Circle_Area) return Circle_Area is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Circle_Area;
   begin
      if Has_Field (J, "position") then
         declare
            Sub : constant String := Write (Get (J, "position"));
         begin
            Result.Position := From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "radius") then
         Result.Radius := Get_Long_Float (Get (J, "radius"));
      end if;
      return Result;
   exception
      when others => return Result;
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

   function From_Json (S : String; Tag : access Point) return Point is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Point;
   begin
      if Has_Field (J, "position") then
         declare
            Sub : constant String := Write (Get (J, "position"));
         begin
            Result.Position := From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Contraint) return String is
   begin
      return "{" &
        """name"":" & """ & To_String (Msg.Name) & """ &
        "," &
        """value"":" & Integer'Image (Msg.Value) &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Contraint) return Contraint is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Contraint;
   begin
      if Has_Field (J, "name") then
         Result.Name := To_Unbounded_String (Get (Get (J, "name")));
      end if;
      if Has_Field (J, "value") then
         Result.Value := Integer (Get_Long_Float (Get (J, "value")));
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Ack) return String is
   begin
      return "{" &
        """success"":" & (if Msg.Success then "true" else "false") &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Ack) return Ack is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Ack;
   begin
      if Has_Field (J, "success") then
         Result.Success := Get (Get (J, "success"));
      end if;
      return Result;
   exception
      when others => return Result;
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
      if Msg.Id /= null then
         Comma;
         Append (Result, """id"":[");
         for I in Msg.Id'Range loop
            if I > Msg.Id'First then
               Append (Result, ",");
            end if;
            Append (Result, """ & To_String (Msg.Id (I)) & """);
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """one_shot"":" & (if Msg.One_Shot then "true" else "false"));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Query) return Query is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Query;
   begin
      if Has_Field (J, "id") then
         declare
            Arr : constant JSON_Value := Get (J, "id");
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Id := new Id_Array (1 .. Len);
               for I in 1 .. Len loop
                  Result.Id (I) := To_Unbounded_String (Get (Get (Arr, I)));
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "one_shot") then
         Result.One_Shot := Get (Get (J, "one_shot"));
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

end Pyramid_Data_Model_Common_Types_Codec;
