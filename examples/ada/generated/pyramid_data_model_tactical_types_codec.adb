--  Auto-generated data model JSON codec body
--  Package: Pyramid_Data_Model_Tactical_Types_Codec

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
      Append (Result, """position"":" & To_Json (Msg.Position));
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
      Result : Object_Detail;
   begin
      --  repeated: source (deserialised via array access)
      --  nested: position
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
      Append (Result, """status"":" & To_Json (Msg.Status));
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
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Evidence_Requirement) return Object_Evidence_Requirement is
      pragma Unreferenced (Tag);
      Result : Object_Evidence_Requirement;
   begin
      --  nested: base
      --  nested: status
      --  repeated: dimension (deserialised via array access)
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
      Append (Result, """status"":" & To_Json (Msg.Status));
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
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Object_Interest_Requirement) return Object_Interest_Requirement is
      pragma Unreferenced (Tag);
      Result : Object_Interest_Requirement;
   begin
      --  nested: base
      --  nested: status
      --  repeated: dimension (deserialised via array access)
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
      Result : Object_Match;
   begin
      return Result;
   end From_Json;

end Pyramid_Data_Model_Tactical_Types_Codec;
