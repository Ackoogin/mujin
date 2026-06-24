with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with System;
with System.Address_To_Access_Conversions;

package body Pyramid.Services.Tactical_Objects.Json_Codec_Plugin is
   use type Interfaces.C.Strings.chars_ptr;
   use type Interfaces.C.unsigned;
   use type Pcl_Bindings.Pcl_Status;
   use type System.Address;

   Json_Content_Type : constant String := "application/json";
   Json_Content_Type_C : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String (Json_Content_Type);

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);
   function To_Chars_Ptr is new
     Ada.Unchecked_Conversion (System.Address, Interfaces.C.Strings.chars_ptr);

   package Identifier_Pointers is
     new System.Address_To_Access_Conversions (Unbounded_String);
   package Query_Pointers is
     new System.Address_To_Access_Conversions (Query);
   package Ack_Pointers is
     new System.Address_To_Access_Conversions (Ack);
   package Capability_Pointers is
     new System.Address_To_Access_Conversions (Capability);
   package Object_Detail_Pointers is
     new System.Address_To_Access_Conversions (Object_Detail);
   package Object_Evidence_Requirement_Pointers is
     new System.Address_To_Access_Conversions (Object_Evidence_Requirement);
   package Object_Interest_Requirement_Pointers is
     new System.Address_To_Access_Conversions (Object_Interest_Requirement);
   package Object_Match_Pointers is
     new System.Address_To_Access_Conversions (Object_Match);

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String
   is
      type Char_Array is array (1 .. Natural (Size)) of Character;
      pragma Pack (Char_Array);
      Chars : Char_Array;
      for Chars'Address use Data;
      pragma Import (Ada, Chars);
   begin
      return String (Chars);
   end Msg_To_String;

   function Quote (S : String) return String is
      R : Unbounded_String := To_Unbounded_String ("""");
   begin
      for C of S loop
         if C = '"' or else C = '\' then
            Append (R, "\");
         end if;
         Append (R, C);
      end loop;
      Append (R, """");
      return To_String (R);
   end Quote;

   function Enum (V : Object_Source) return String is
   begin
      case V is
         when Source_Unspecified => return "OBJECT_SOURCE_UNSPECIFIED";
         when Source_Radar => return "OBJECT_SOURCE_RADAR";
         when Source_Local => return "OBJECT_SOURCE_LOCAL";
      end case;
   end Enum;

   function Source_From_String (S : String) return Object_Source is
   begin
      if S = "OBJECT_SOURCE_RADAR" then
         return Source_Radar;
      elsif S = "OBJECT_SOURCE_LOCAL" then
         return Source_Local;
      end if;
      return Source_Unspecified;
   end Source_From_String;

   function Enum (V : Standard_Identity) return String is
   begin
      case V is
         when Identity_Unknown => return "STANDARD_IDENTITY_UNKNOWN";
         when Identity_Friendly => return "STANDARD_IDENTITY_FRIENDLY";
         when Identity_Hostile => return "STANDARD_IDENTITY_HOSTILE";
         when Identity_Suspect => return "STANDARD_IDENTITY_SUSPECT";
         when Identity_Neutral => return "STANDARD_IDENTITY_NEUTRAL";
         when Identity_Pending => return "STANDARD_IDENTITY_PENDING";
         when Identity_Joker => return "STANDARD_IDENTITY_JOKER";
         when Identity_Faker => return "STANDARD_IDENTITY_FAKER";
         when Identity_AssumedFriendly =>
            return "STANDARD_IDENTITY_ASSUMED_FRIENDLY";
         when Identity_Unspecified =>
            return "STANDARD_IDENTITY_UNSPECIFIED";
      end case;
   end Enum;

   function Identity_From_String (S : String) return Standard_Identity is
   begin
      if S = "STANDARD_IDENTITY_HOSTILE" then
         return Identity_Hostile;
      elsif S = "STANDARD_IDENTITY_FRIENDLY" then
         return Identity_Friendly;
      elsif S = "STANDARD_IDENTITY_NEUTRAL" then
         return Identity_Neutral;
      elsif S = "STANDARD_IDENTITY_UNKNOWN" then
         return Identity_Unknown;
      end if;
      return Identity_Unspecified;
   end Identity_From_String;

   function Enum (V : Battle_Dimension) return String is
   begin
      case V is
         when Dimension_Ground => return "BATTLE_DIMENSION_GROUND";
         when Dimension_Subsurface => return "BATTLE_DIMENSION_SUBSURFACE";
         when Dimension_SeaSurface => return "BATTLE_DIMENSION_SEA_SURFACE";
         when Dimension_Air => return "BATTLE_DIMENSION_AIR";
         when Dimension_Unknown => return "BATTLE_DIMENSION_UNKNOWN";
         when Dimension_Unspecified => return "BATTLE_DIMENSION_UNSPECIFIED";
      end case;
   end Enum;

   function Dimension_From_String (S : String) return Battle_Dimension is
   begin
      if S = "BATTLE_DIMENSION_SEA_SURFACE" then
         return Dimension_SeaSurface;
      elsif S = "BATTLE_DIMENSION_GROUND" then
         return Dimension_Ground;
      elsif S = "BATTLE_DIMENSION_SUBSURFACE" then
         return Dimension_Subsurface;
      elsif S = "BATTLE_DIMENSION_AIR" then
         return Dimension_Air;
      elsif S = "BATTLE_DIMENSION_UNKNOWN" then
         return Dimension_Unknown;
      end if;
      return Dimension_Unspecified;
   end Dimension_From_String;

   function Enum (V : Data_Policy) return String is
   begin
      case V is
         when Policy_Query => return "DATA_POLICY_QUERY";
         when Policy_Obtain => return "DATA_POLICY_OBTAIN";
         when Policy_Unspecified => return "DATA_POLICY_UNSPECIFIED";
      end case;
   end Enum;

   function Policy_From_String (S : String) return Data_Policy is
   begin
      if S = "DATA_POLICY_QUERY" then
         return Policy_Query;
      elsif S = "DATA_POLICY_OBTAIN" then
         return Policy_Obtain;
      end if;
      return Policy_Unspecified;
   end Policy_From_String;

   function Field_Value (S, Key : String) return String is
      Pattern : constant String := """" & Key & """";
      K : constant Natural := Ada.Strings.Fixed.Index (S, Pattern);
      Colon : Natural := 0;
      First : Natural := 0;
      Last  : Natural := 0;
      Depth : Natural := 0;
      In_String : Boolean := False;
      Escape : Boolean := False;
   begin
      if K = 0 then
         return "";
      end if;
      for I in K + Pattern'Length .. S'Last loop
         if S (I) = ':' then
            Colon := I;
            exit;
         end if;
      end loop;
      if Colon = 0 then
         return "";
      end if;
      First := Colon + 1;
      while First <= S'Last and then S (First) = ' ' loop
         First := First + 1;
      end loop;
      if First > S'Last then
         return "";
      end if;

      if S (First) = '"' then
         Last := First + 1;
         while Last <= S'Last loop
            if Escape then
               Escape := False;
            elsif S (Last) = '\' then
               Escape := True;
            elsif S (Last) = '"' then
               return S (First .. Last);
            end if;
            Last := Last + 1;
         end loop;
         return "";
      elsif S (First) = '{' or else S (First) = '[' then
         Last := First;
         while Last <= S'Last loop
            if Escape then
               Escape := False;
            elsif In_String and then S (Last) = '\' then
               Escape := True;
            elsif S (Last) = '"' then
               In_String := not In_String;
            elsif not In_String then
               if S (Last) = '{' or else S (Last) = '[' then
                  Depth := Depth + 1;
               elsif S (Last) = '}' or else S (Last) = ']' then
                  Depth := Depth - 1;
                  if Depth = 0 then
                     return S (First .. Last);
                  end if;
               end if;
            end if;
            Last := Last + 1;
         end loop;
         return "";
      else
         Last := First;
         while Last <= S'Last
           and then S (Last) /= ','
           and then S (Last) /= '}'
           and then S (Last) /= ']'
         loop
            Last := Last + 1;
         end loop;
         return S (First .. Last - 1);
      end if;
   end Field_Value;

   function String_Field (S, Key : String) return String is
      Raw : constant String := Field_Value (S, Key);
      R : Unbounded_String := Null_Unbounded_String;
      I : Natural;
   begin
      if Raw'Length < 2 or else Raw (Raw'First) /= '"' then
         return "";
      end if;
      I := Raw'First + 1;
      while I < Raw'Last loop
         if Raw (I) = '\' and then I + 1 < Raw'Last then
            I := I + 1;
         end if;
         Append (R, Raw (I));
         I := I + 1;
      end loop;
      return To_String (R);
   end String_Field;

   function Number_Field
     (S, Key : String;
      Default : Long_Float := 0.0) return Long_Float
   is
      Raw : constant String := Field_Value (S, Key);
   begin
      if Raw = "" then
         return Default;
      end if;
      return Long_Float'Value (Raw);
   exception
      when others =>
         return Default;
   end Number_Field;

   function Bool_Field
     (S, Key : String;
      Default : Boolean := False) return Boolean
   is
      Raw : constant String := Field_Value (S, Key);
   begin
      if Raw = "true" then
         return True;
      elsif Raw = "false" then
         return False;
      end if;
      return Default;
   end Bool_Field;

   function Position_To_Json (P : Geodetic_Position) return String is
   begin
      return "{""latitude"":" & Long_Float'Image (P.Latitude) &
        ",""longitude"":" & Long_Float'Image (P.Longitude) & "}";
   end Position_To_Json;

   function Position_From_Json (S : String) return Geodetic_Position is
   begin
      return
        (Latitude  => Number_Field (S, "latitude"),
         Longitude => Number_Field (S, "longitude"));
   end Position_From_Json;

   function Entity_To_Json (E : Entity) return String is
   begin
      return "{""update_time"":" & Long_Float'Image (E.Update_Time) &
        ",""id"":" & Quote (To_String (E.Id)) &
        ",""source"":" & Quote (To_String (E.Source)) & "}";
   end Entity_To_Json;

   function Entity_From_Json (S : String) return Entity is
      E : Entity;
   begin
      E.Update_Time := Number_Field (S, "update_time");
      E.Id := To_Unbounded_String (String_Field (S, "id"));
      E.Source := To_Unbounded_String (String_Field (S, "source"));
      return E;
   end Entity_From_Json;

   function Dimension_Array_To_Json (A : Dimension_Array_Acc) return String is
      R : Unbounded_String := To_Unbounded_String ("[");
   begin
      if A /= null then
         for I in A'Range loop
            if I > A'First then
               Append (R, ",");
            end if;
            Append (R, Quote (Enum (A (I))));
         end loop;
      end if;
      Append (R, "]");
      return To_String (R);
   end Dimension_Array_To_Json;

   function Source_Array_To_Json (A : Source_Array_Acc) return String is
      R : Unbounded_String := To_Unbounded_String ("[");
   begin
      if A /= null then
         for I in A'Range loop
            if I > A'First then
               Append (R, ",");
            end if;
            Append (R, Quote (Enum (A (I))));
         end loop;
      end if;
      Append (R, "]");
      return To_String (R);
   end Source_Array_To_Json;

   function First_Array_String (Raw : String) return String is
      I : Natural := Raw'First;
      Start : Natural := 0;
   begin
      while I <= Raw'Last loop
         if Raw (I) = '"' then
            Start := I;
            I := I + 1;
            while I <= Raw'Last loop
               if Raw (I) = '"' then
                  return Raw (Start + 1 .. I - 1);
               end if;
               I := I + 1;
            end loop;
         end if;
         I := I + 1;
      end loop;
      return "";
   end First_Array_String;

   function Decode_Source_Array (Raw : String) return Source_Array_Acc is
      Count : Natural := 0;
      I : Natural := Raw'First;
   begin
      while I <= Raw'Last loop
         if Raw (I) = '"' then
            Count := Count + 1;
            I := I + 1;
            while I <= Raw'Last and then Raw (I) /= '"' loop
               I := I + 1;
            end loop;
         end if;
         I := I + 1;
      end loop;
      if Count = 0 then
         return null;
      end if;
      declare
         Result : Source_Array_Acc := new Source_Array (1 .. Count);
         Index : Positive := 1;
      begin
         I := Raw'First;
         while I <= Raw'Last loop
            if Raw (I) = '"' then
               declare
                  Start : constant Natural := I + 1;
               begin
                  I := Start;
                  while I <= Raw'Last and then Raw (I) /= '"' loop
                     I := I + 1;
                  end loop;
                  Result (Index) := Source_From_String (Raw (Start .. I - 1));
                  Index := Index + 1;
               end;
            end if;
            I := I + 1;
         end loop;
         return Result;
      end;
   end Decode_Source_Array;

   function Object_Detail_To_Json (Msg : Object_Detail) return String is
   begin
      return "{""update_time"":" & Long_Float'Image (Msg.Update_Time) &
        ",""id"":" & Quote (To_String (Msg.Id)) &
        ",""entity_source"":" & Quote (To_String (Msg.Entity_Source)) &
        ",""source"":" & Source_Array_To_Json (Msg.Source) &
        ",""position"":" & Position_To_Json (Msg.Position) &
        ",""creation_time"":" & Long_Float'Image (Msg.Creation_Time) &
        ",""quality"":" & Long_Float'Image (Msg.Quality) &
        ",""course"":" & Long_Float'Image (Msg.Course) &
        ",""speed"":" & Long_Float'Image (Msg.Speed) &
        ",""length"":" & Long_Float'Image (Msg.Length) &
        ",""identity"":" & Quote (Enum (Msg.Identity)) &
        ",""dimension"":" & Quote (Enum (Msg.Dimension)) & "}";
   end Object_Detail_To_Json;

   function Object_Detail_From_Json (S : String) return Object_Detail is
      Result : Object_Detail;
      Position_Raw : constant String := Field_Value (S, "position");
      Source_Raw : constant String := Field_Value (S, "source");
   begin
      Result.Update_Time := Number_Field (S, "update_time");
      Result.Id := To_Unbounded_String (String_Field (S, "id"));
      Result.Entity_Source :=
        To_Unbounded_String (String_Field (S, "entity_source"));
      Result.Source := Decode_Source_Array (Source_Raw);
      if Position_Raw /= "" then
         Result.Position := Position_From_Json (Position_Raw);
      end if;
      Result.Creation_Time := Number_Field (S, "creation_time");
      Result.Quality := Number_Field (S, "quality");
      Result.Course := Number_Field (S, "course");
      Result.Speed := Number_Field (S, "speed");
      Result.Length := Number_Field (S, "length");
      Result.Identity := Identity_From_String (String_Field (S, "identity"));
      Result.Dimension := Dimension_From_String (String_Field (S, "dimension"));
      return Result;
   end Object_Detail_From_Json;

   function Query_To_Json (Msg : Query) return String is
      R : Unbounded_String := To_Unbounded_String ("{""id"":[");
   begin
      if Msg.Id /= null then
         for I in Msg.Id'Range loop
            if I > Msg.Id'First then
               Append (R, ",");
            end if;
            Append (R, Quote (To_String (Msg.Id (I))));
         end loop;
      end if;
      Append (R, "]");
      if Msg.Has_One_Shot then
         Append (R, ",""one_shot"":");
         Append (R, (if Msg.One_Shot then "true" else "false"));
      end if;
      Append (R, "}");
      return To_String (R);
   end Query_To_Json;

   function Query_From_Json (S : String) return Query is
      Result : Query;
   begin
      if Field_Value (S, "one_shot") /= "" then
         Result.Has_One_Shot := True;
         Result.One_Shot := Bool_Field (S, "one_shot");
      end if;
      return Result;
   end Query_From_Json;

   function Ack_To_Json (Msg : Ack) return String is
   begin
      return "{""success"":" & (if Msg.Success then "true" else "false") & "}";
   end Ack_To_Json;

   function Ack_From_Json (S : String) return Ack is
   begin
      return (Success => Bool_Field (S, "success"));
   end Ack_From_Json;

   function Capability_To_Json (Msg : Capability) return String is
   begin
      return "{""update_time"":" & Long_Float'Image (Msg.Update_Time) &
        ",""id"":" & Quote (To_String (Msg.Id)) &
        ",""source"":" & Quote (To_String (Msg.Source)) &
        ",""availability"":" &
        (if Msg.Availability then "true" else "false") &
        ",""name"":" & Quote (To_String (Msg.Name)) & "}";
   end Capability_To_Json;

   function Capability_From_Json (S : String) return Capability is
      Result : Capability;
   begin
      Result.Update_Time := Number_Field (S, "update_time");
      Result.Id := To_Unbounded_String (String_Field (S, "id"));
      Result.Source := To_Unbounded_String (String_Field (S, "source"));
      Result.Availability := Bool_Field (S, "availability");
      Result.Name := To_Unbounded_String (String_Field (S, "name"));
      return Result;
   end Capability_From_Json;

   function Requirement_To_Json
     (Base : Entity;
      Policy : Data_Policy;
      Dimension : Dimension_Array_Acc;
      Source : Object_Source := Source_Unspecified) return String
   is
      Prefix : constant String :=
        "{""base"":" & Entity_To_Json (Base) &
        ",""status"":{""status"":""PROGRESS_UNSPECIFIED""}" &
        ",""policy"":" & Quote (Enum (Policy)) &
        ",""dimension"":" & Dimension_Array_To_Json (Dimension);
   begin
      if Source = Source_Unspecified then
         return Prefix & "}";
      end if;
      return Prefix & ",""source"":" & Quote (Enum (Source)) & "}";
   end Requirement_To_Json;

   function Evidence_Requirement_From_Json
     (S : String) return Object_Evidence_Requirement
   is
      Result : Object_Evidence_Requirement;
      Base_Raw : constant String := Field_Value (S, "base");
      Dimension_Raw : constant String := Field_Value (S, "dimension");
      First_Dimension : constant String := First_Array_String (Dimension_Raw);
   begin
      if Base_Raw /= "" then
         Result.Base := Entity_From_Json (Base_Raw);
      end if;
      Result.Policy := Policy_From_String (String_Field (S, "policy"));
      if First_Dimension /= "" then
         Result.Dimension := new Dimension_Array'
           (1 => Dimension_From_String (First_Dimension));
      end if;
      return Result;
   end Evidence_Requirement_From_Json;

   function Interest_Requirement_From_Json
     (S : String) return Object_Interest_Requirement
   is
      Result : Object_Interest_Requirement;
      Base_Raw : constant String := Field_Value (S, "base");
      Dimension_Raw : constant String := Field_Value (S, "dimension");
      First_Dimension : constant String := First_Array_String (Dimension_Raw);
   begin
      if Base_Raw /= "" then
         Result.Base := Entity_From_Json (Base_Raw);
      end if;
      Result.Source := Source_From_String (String_Field (S, "source"));
      Result.Policy := Policy_From_String (String_Field (S, "policy"));
      if First_Dimension /= "" then
         Result.Dimension := new Dimension_Array'
           (1 => Dimension_From_String (First_Dimension));
      end if;
      return Result;
   end Interest_Requirement_From_Json;

   function Object_Match_To_Json (Msg : Object_Match) return String is
   begin
      return "{""update_time"":" & Long_Float'Image (Msg.Update_Time) &
        ",""id"":" & Quote (To_String (Msg.Id)) &
        ",""source"":" & Quote (To_String (Msg.Source)) &
        ",""matching_object_id"":" &
        Quote (To_String (Msg.Matching_Object_Id)) & "}";
   end Object_Match_To_Json;

   function Object_Match_From_Json (S : String) return Object_Match is
      Result : Object_Match;
   begin
      Result.Update_Time := Number_Field (S, "update_time");
      Result.Id := To_Unbounded_String (String_Field (S, "id"));
      Result.Source := To_Unbounded_String (String_Field (S, "source"));
      Result.Matching_Object_Id :=
        To_Unbounded_String (String_Field (S, "matching_object_id"));
      return Result;
   end Object_Match_From_Json;

   procedure Assign_Payload
     (Payload : String;
      Out_Msg : access Pcl_Bindings.Pcl_Msg)
   is
      Data_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Type_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Json_Content_Type);
   begin
      Out_Msg.Data := To_Address (Data_C);
      Out_Msg.Size := Interfaces.C.unsigned (Payload'Length);
      Out_Msg.Type_Name := Type_C;
   exception
      when others =>
         if Data_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Data_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         raise;
   end Assign_Payload;

   function Identifier_To_Json (Value : Identifier) return String is
   begin
      return Quote (To_String (Value));
   end Identifier_To_Json;

   function Identifier_From_Json (Payload : String) return Identifier is
   begin
      if Payload'Length >= 2 and then Payload (Payload'First) = '"' then
         return To_Unbounded_String (String_Field ("{""v"":" & Payload & "}", "v"));
      end if;
      return To_Unbounded_String (Payload);
   end Identifier_From_Json;

   function Encode
     (Codec_Ctx : System.Address;
      Schema_Id : Interfaces.C.Strings.chars_ptr;
      Value     : System.Address;
      Out_Msg   : access Pcl_Bindings.Pcl_Msg)
      return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Encode);

   function Decode
     (Codec_Ctx : System.Address;
      Schema_Id : Interfaces.C.Strings.chars_ptr;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      Out_Value : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Decode);

   procedure Free_Msg
     (Codec_Ctx : System.Address;
      Msg       : access Pcl_Bindings.Pcl_Msg);
   pragma Convention (C, Free_Msg);

   function Encode
     (Codec_Ctx : System.Address;
      Schema_Id : Interfaces.C.Strings.chars_ptr;
      Value     : System.Address;
      Out_Msg   : access Pcl_Bindings.Pcl_Msg)
      return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Codec_Ctx);
   begin
      if Schema_Id = Interfaces.C.Strings.Null_Ptr
        or else Value = System.Null_Address
        or else Out_Msg = null
      then
         return Pcl_Bindings.PCL_ERR_INVALID;
      end if;

      declare
         Schema : constant String := Interfaces.C.Strings.Value (Schema_Id);
      begin
         if Schema = "Identifier" then
            Assign_Payload
              (Identifier_To_Json (Identifier_Pointers.To_Pointer (Value).all),
               Out_Msg);
         elsif Schema = "Query" then
            Assign_Payload
              (Query_To_Json (Query_Pointers.To_Pointer (Value).all), Out_Msg);
         elsif Schema = "Ack" then
            Assign_Payload
              (Ack_To_Json (Ack_Pointers.To_Pointer (Value).all), Out_Msg);
         elsif Schema = "Capability" then
            Assign_Payload
              (Capability_To_Json (Capability_Pointers.To_Pointer (Value).all),
               Out_Msg);
         elsif Schema = "ObjectDetail" then
            Assign_Payload
              (Object_Detail_To_Json
                 (Object_Detail_Pointers.To_Pointer (Value).all),
               Out_Msg);
         elsif Schema = "ObjectEvidenceRequirement" then
            declare
               V : constant Object_Evidence_Requirement :=
                 Object_Evidence_Requirement_Pointers.To_Pointer (Value).all;
            begin
               Assign_Payload
                 (Requirement_To_Json (V.Base, V.Policy, V.Dimension),
                  Out_Msg);
            end;
         elsif Schema = "ObjectInterestRequirement" then
            declare
               V : constant Object_Interest_Requirement :=
                 Object_Interest_Requirement_Pointers.To_Pointer (Value).all;
            begin
               Assign_Payload
                 (Requirement_To_Json
                    (V.Base, V.Policy, V.Dimension, V.Source),
                  Out_Msg);
            end;
         elsif Schema = "ObjectMatch" then
            Assign_Payload
              (Object_Match_To_Json
                 (Object_Match_Pointers.To_Pointer (Value).all),
               Out_Msg);
         else
            return Pcl_Bindings.PCL_ERR_NOT_FOUND;
         end if;
      end;

      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         return Pcl_Bindings.PCL_ERR_CALLBACK;
   end Encode;

   function Decode
     (Codec_Ctx : System.Address;
      Schema_Id : Interfaces.C.Strings.chars_ptr;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      Out_Value : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Codec_Ctx);
   begin
      if Schema_Id = Interfaces.C.Strings.Null_Ptr
        or else Msg = null
        or else (Msg.Data = System.Null_Address and then Msg.Size /= 0)
        or else Out_Value = System.Null_Address
      then
         return Pcl_Bindings.PCL_ERR_INVALID;
      end if;

      declare
         Schema  : constant String := Interfaces.C.Strings.Value (Schema_Id);
         Payload : constant String :=
           (if Msg.Data = System.Null_Address
            then ""
            else Msg_To_String (Msg.Data, Msg.Size));
      begin
         if Schema = "Identifier" then
            Identifier_Pointers.To_Pointer (Out_Value).all :=
              Identifier_From_Json (Payload);
         elsif Schema = "Query" then
            Query_Pointers.To_Pointer (Out_Value).all :=
              Query_From_Json (Payload);
         elsif Schema = "Ack" then
            Ack_Pointers.To_Pointer (Out_Value).all :=
              Ack_From_Json (Payload);
         elsif Schema = "Capability" then
            Capability_Pointers.To_Pointer (Out_Value).all :=
              Capability_From_Json (Payload);
         elsif Schema = "ObjectDetail" then
            Object_Detail_Pointers.To_Pointer (Out_Value).all :=
              Object_Detail_From_Json (Payload);
         elsif Schema = "ObjectEvidenceRequirement" then
            Object_Evidence_Requirement_Pointers.To_Pointer (Out_Value).all :=
              Evidence_Requirement_From_Json (Payload);
         elsif Schema = "ObjectInterestRequirement" then
            Object_Interest_Requirement_Pointers.To_Pointer (Out_Value).all :=
              Interest_Requirement_From_Json (Payload);
         elsif Schema = "ObjectMatch" then
            Object_Match_Pointers.To_Pointer (Out_Value).all :=
              Object_Match_From_Json (Payload);
         else
            return Pcl_Bindings.PCL_ERR_NOT_FOUND;
         end if;
      end;

      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         return Pcl_Bindings.PCL_ERR_CALLBACK;
   end Decode;

   procedure Free_Msg
     (Codec_Ctx : System.Address;
      Msg       : access Pcl_Bindings.Pcl_Msg)
   is
      pragma Unreferenced (Codec_Ctx);
      Data_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
   begin
      if Msg = null then
         return;
      end if;

      if Msg.Data /= System.Null_Address then
         Data_C := To_Chars_Ptr (Msg.Data);
         Interfaces.C.Strings.Free (Data_C);
      end if;
      if Msg.Type_Name /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Msg.Type_Name);
      end if;
      Msg.Data := System.Null_Address;
      Msg.Size := 0;
      Msg.Type_Name := Interfaces.C.Strings.Null_Ptr;
   end Free_Msg;

   Codec : aliased constant Pcl_Plugins.Pcl_Codec :=
     (Abi_Version  => Pcl_Plugins.Pcl_Codec_Abi_Version,
      Content_Type => Json_Content_Type_C,
      Encode       => Encode'Access,
      Decode       => Decode'Access,
      Free_Msg     => Free_Msg'Access,
      Codec_Ctx    => System.Null_Address);

   function Plugin_Entry return Pcl_Plugins.Pcl_Codec_Const_Access is
   begin
      return Codec'Access;
   end Plugin_Entry;

end Pyramid.Services.Tactical_Objects.Json_Codec_Plugin;
