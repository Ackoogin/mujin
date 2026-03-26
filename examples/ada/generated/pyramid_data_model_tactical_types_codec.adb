--  Auto-generated data model JSON codec body
--  Package: Pyramid_Data_Model_Tactical_Types_Codec

with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Pyramid_Data_Model_Common_Types_Codec;
pragma Warnings (Off);

package body Pyramid_Data_Model_Tactical_Types_Codec is

   function To_String (V : Object_Source) return String is
   begin
      case V is
         when Source_Unspecified => return "OBJECT_SOURCE_UNSPECIFIED";
         when Source_Radar => return "OBJECT_SOURCE_RADAR";
         when Source_Local => return "OBJECT_SOURCE_LOCAL";
      end case;
   end To_String;

   function Object_Source_From_String (S : String) return Object_Source is
   begin
      if S = "OBJECT_SOURCE_UNSPECIFIED" then return Source_Unspecified; end if;
      if S = "OBJECT_SOURCE_RADAR" then return Source_Radar; end if;
      if S = "OBJECT_SOURCE_LOCAL" then return Source_Local; end if;
      return Source_Unspecified;
   end Object_Source_From_String;

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
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """ & To_String (Msg.Id) & """);
      Comma;
      Append (Result, """entity_source"":" & """ & To_String (Msg.Entity_Source) & """);
      if Msg.Source /= null then
         Comma;
         Append (Result, """source"":[");
         for I in Msg.Source'Range loop
            if I > Msg.Source'First then
               Append (Result, ",");
            end if;
            Append (Result, """ & To_String (Msg.Source (I)) & """);
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """position"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Position));
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
      Comma;
      Append (Result, """identity"":" & """ & To_String (Msg.Identity) & """);
      Comma;
      Append (Result, """dimension"":" & """ & To_String (Msg.Dimension) & """);
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Detail) return Object_Detail is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Object_Detail;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         Result.Id := To_Unbounded_String (Get (Get (J, "id")));
      end if;
      if Has_Field (J, "entity_source") then
         Result.Entity_Source := To_Unbounded_String (Get (Get (J, "entity_source")));
      end if;
      if Has_Field (J, "source") then
         declare
            Arr : constant JSON_Value := Get (J, "source");
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Source := new Source_Array (1 .. Len);
               for I in 1 .. Len loop
                  Result.Source (I) := Object_Source_From_String (Get (Get (Arr, I)));
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "position") then
         declare
            Sub : constant String := Write (Get (J, "position"));
         begin
            Result.Position := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
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
      if Has_Field (J, "identity") then
         Result.Identity := Standard_Identity_From_String (Get (Get (J, "identity")));
      end if;
      if Has_Field (J, "dimension") then
         Result.Dimension := Battle_Dimension_From_String (Get (Get (J, "dimension")));
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """base"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Base));
      Comma;
      Append (Result, """status"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Status));
      Comma;
      Append (Result, """policy"":" & """ & To_String (Msg.Policy) & """);
      if Msg.Dimension /= null then
         Comma;
         Append (Result, """dimension"":[");
         for I in Msg.Dimension'Range loop
            if I > Msg.Dimension'First then
               Append (Result, ",");
            end if;
            Append (Result, """ & To_String (Msg.Dimension (I)) & """);
         end loop;
         Append (Result, "]");
      end if;
      if Msg.Has_Val_Poly_Area then
         Comma;
         Append (Result, """poly_area"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Val_Poly_Area));
      end if;
      if Msg.Has_Val_Circle_Area then
         Comma;
         Append (Result, """circle_area"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Val_Circle_Area));
      end if;
      if Msg.Has_Val_Point then
         Comma;
         Append (Result, """point"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Val_Point));
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Evidence_Requirement) return Object_Evidence_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Object_Evidence_Requirement;
   begin
      if Has_Field (J, "base") then
         declare
            Sub : constant String := Write (Get (J, "base"));
         begin
            Result.Base := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Sub : constant String := Write (Get (J, "status"));
         begin
            Result.Status := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "policy") then
         Result.Policy := Data_Policy_From_String (Get (Get (J, "policy")));
      end if;
      if Has_Field (J, "dimension") then
         declare
            Arr : constant JSON_Value := Get (J, "dimension");
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Dimension := new Dimension_Array (1 .. Len);
               for I in 1 .. Len loop
                  Result.Dimension (I) := Battle_Dimension_From_String (Get (Get (Arr, I)));
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "poly_area") then
         Result.Has_Val_Poly_Area := True;
         declare
            Sub : constant String := Write (Get (J, "poly_area"));
         begin
            Result.Val_Poly_Area := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "circle_area") then
         Result.Has_Val_Circle_Area := True;
         declare
            Sub : constant String := Write (Get (J, "circle_area"));
         begin
            Result.Val_Circle_Area := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "point") then
         Result.Has_Val_Point := True;
         declare
            Sub : constant String := Write (Get (J, "point"));
         begin
            Result.Val_Point := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """base"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Base));
      Comma;
      Append (Result, """status"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Status));
      Comma;
      Append (Result, """source"":" & """ & To_String (Msg.Source) & """);
      Comma;
      Append (Result, """policy"":" & """ & To_String (Msg.Policy) & """);
      if Msg.Dimension /= null then
         Comma;
         Append (Result, """dimension"":[");
         for I in Msg.Dimension'Range loop
            if I > Msg.Dimension'First then
               Append (Result, ",");
            end if;
            Append (Result, """ & To_String (Msg.Dimension (I)) & """);
         end loop;
         Append (Result, "]");
      end if;
      if Msg.Has_Val_Poly_Area then
         Comma;
         Append (Result, """poly_area"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Val_Poly_Area));
      end if;
      if Msg.Has_Val_Circle_Area then
         Comma;
         Append (Result, """circle_area"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Val_Circle_Area));
      end if;
      if Msg.Has_Val_Point then
         Comma;
         Append (Result, """point"":" & Pyramid_Data_Model_Common_Types_Codec.To_Json (Msg.Val_Point));
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Interest_Requirement) return Object_Interest_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Object_Interest_Requirement;
   begin
      if Has_Field (J, "base") then
         declare
            Sub : constant String := Write (Get (J, "base"));
         begin
            Result.Base := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Sub : constant String := Write (Get (J, "status"));
         begin
            Result.Status := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "source") then
         Result.Source := Object_Source_From_String (Get (Get (J, "source")));
      end if;
      if Has_Field (J, "policy") then
         Result.Policy := Data_Policy_From_String (Get (Get (J, "policy")));
      end if;
      if Has_Field (J, "dimension") then
         declare
            Arr : constant JSON_Value := Get (J, "dimension");
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Dimension := new Dimension_Array (1 .. Len);
               for I in 1 .. Len loop
                  Result.Dimension (I) := Battle_Dimension_From_String (Get (Get (Arr, I)));
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "poly_area") then
         Result.Has_Val_Poly_Area := True;
         declare
            Sub : constant String := Write (Get (J, "poly_area"));
         begin
            Result.Val_Poly_Area := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "circle_area") then
         Result.Has_Val_Circle_Area := True;
         declare
            Sub : constant String := Write (Get (J, "circle_area"));
         begin
            Result.Val_Circle_Area := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "point") then
         Result.Has_Val_Point := True;
         declare
            Sub : constant String := Write (Get (J, "point"));
         begin
            Result.Val_Point := Pyramid_Data_Model_Common_Types_Codec.From_Json (Sub, null);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
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
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """ & To_String (Msg.Id) & """);
      Comma;
      Append (Result, """source"":" & """ & To_String (Msg.Source) & """);
      Comma;
      Append (Result, """matching_object_id"":" & """ & To_String (Msg.Matching_Object_Id) & """);
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Match) return Object_Match is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Object_Match;
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
      if Has_Field (J, "matching_object_id") then
         Result.Matching_Object_Id := To_Unbounded_String (Get (Get (J, "matching_object_id")));
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

end Pyramid_Data_Model_Tactical_Types_Codec;
