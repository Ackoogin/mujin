--  Auto-generated JSON codec body
--  Package body: Pyramid.Services.Tactical_Objects.Json_Codec

with GNATCOLL.JSON;  use GNATCOLL.JSON;

package body Pyramid.Services.Tactical_Objects.Json_Codec is

   --  -- Internal helpers -------------------------------------------

   procedure Set_F
     (Obj  : in out JSON_Value;
      Name : String;
      Val  : Long_Float)
   is
   begin
      Set_Field (Obj, Name, Create (Val));
   end Set_F;

   function Get_F
     (J    : JSON_Value;
      Name : String;
      Def  : Long_Float := 0.0) return Long_Float
   is
   begin
      if Has_Field (J, Name) then
         return Get_Long_Float (Get (J, Name));
      end if;
      return Def;
   exception
      when others => return Def;
   end Get_F;

   function Get_S
     (J    : JSON_Value;
      Name : String;
      Def  : String := "") return String
   is
   begin
      if Has_Field (J, Name) then
         return Get (Get (J, Name));
      end if;
      return Def;
   exception
      when others => return Def;
   end Get_S;

   --  -- Enum string converters -----------------------------------------

   function Standard_Identity_To_String (V : Standard_Identity) return String is
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
   end Standard_Identity_To_String;

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

   function Battle_Dimension_To_String (V : Battle_Dimension) return String is
   begin
      case V is
         when Dimension_Unspecified => return "BATTLE_DIMENSION_UNSPECIFIED";
         when Dimension_Ground => return "BATTLE_DIMENSION_GROUND";
         when Dimension_Subsurface => return "BATTLE_DIMENSION_SUBSURFACE";
         when Dimension_SeaSurface => return "BATTLE_DIMENSION_SEA_SURFACE";
         when Dimension_Air => return "BATTLE_DIMENSION_AIR";
         when Dimension_Unknown => return "BATTLE_DIMENSION_UNKNOWN";
      end case;
   end Battle_Dimension_To_String;

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

   function Data_Policy_To_String (V : Data_Policy) return String is
   begin
      case V is
         when Policy_Unspecified => return "DATA_POLICY_UNSPECIFIED";
         when Policy_Query => return "DATA_POLICY_QUERY";
         when Policy_Obtain => return "DATA_POLICY_OBTAIN";
      end case;
   end Data_Policy_To_String;

   function Data_Policy_From_String (S : String) return Data_Policy is
   begin
      if S = "DATA_POLICY_UNSPECIFIED" then return Policy_Unspecified; end if;
      if S = "DATA_POLICY_QUERY" then return Policy_Query; end if;
      if S = "DATA_POLICY_OBTAIN" then return Policy_Obtain; end if;
      return Policy_Unspecified;
   end Data_Policy_From_String;

   --  -- To_Json ------------------------------------------------------

   function To_Json (Msg : Create_Requirement_Request) return String is
      Obj : JSON_Value := Create_Object;
   begin
      Set_Field (Obj, "policy", Data_Policy_To_String (Msg.Policy));
      Set_Field (Obj, "identity", Standard_Identity_To_String (Msg.Identity));
      if Msg.Dimension /= Dimension_Unspecified then
         Set_Field (Obj, "dimension", Battle_Dimension_To_String (Msg.Dimension));
      end if;
      if Msg.Min_Lat_Rad /= 0.0 then
         Set_F (Obj, "min_lat_rad", Msg.Min_Lat_Rad);
      end if;
      if Msg.Max_Lat_Rad /= 0.0 then
         Set_F (Obj, "max_lat_rad", Msg.Max_Lat_Rad);
      end if;
      if Msg.Min_Lon_Rad /= 0.0 then
         Set_F (Obj, "min_lon_rad", Msg.Min_Lon_Rad);
      end if;
      if Msg.Max_Lon_Rad /= 0.0 then
         Set_F (Obj, "max_lon_rad", Msg.Max_Lon_Rad);
      end if;
      return Write (Obj);
   end To_Json;

   function To_Json (Msg : Create_Requirement_Response) return String is
      Obj : JSON_Value := Create_Object;
   begin
      if Msg.Interest_Id /= Null_Unbounded_String then
         Set_Field (Obj, "interest_id", To_String (Msg.Interest_Id));
      end if;
      return Write (Obj);
   end To_Json;

   function To_Json (Msg : Entity_Match) return String is
      Obj : JSON_Value := Create_Object;
   begin
      Set_Field (Obj, "object_id", To_String (Msg.Object_Id));
      Set_Field (Obj, "identity", Standard_Identity_To_String (Msg.Identity));
      if Msg.Dimension /= Dimension_Unspecified then
         Set_Field (Obj, "dimension", Battle_Dimension_To_String (Msg.Dimension));
      end if;
      if Msg.Latitude_Rad /= 0.0 then
         Set_F (Obj, "latitude_rad", Msg.Latitude_Rad);
      end if;
      if Msg.Longitude_Rad /= 0.0 then
         Set_F (Obj, "longitude_rad", Msg.Longitude_Rad);
      end if;
      if Msg.Confidence /= 0.0 then
         Set_F (Obj, "confidence", Msg.Confidence);
      end if;
      return Write (Obj);
   end To_Json;

   function To_Json (Msg : Object_Evidence) return String is
      Obj : JSON_Value := Create_Object;
   begin
      Set_Field (Obj, "identity", Standard_Identity_To_String (Msg.Identity));
      Set_Field (Obj, "dimension", Battle_Dimension_To_String (Msg.Dimension));
      Set_F (Obj, "latitude_rad", Msg.Latitude_Rad);
      Set_F (Obj, "longitude_rad", Msg.Longitude_Rad);
      Set_F (Obj, "confidence", Msg.Confidence);
      if Msg.Observed_At /= 0.0 then
         Set_F (Obj, "observed_at", Msg.Observed_At);
      end if;
      return Write (Obj);
   end To_Json;

   function To_Json (Msg : Evidence_Requirement) return String is
      Obj : JSON_Value := Create_Object;
   begin
      if Msg.Id /= Null_Unbounded_String then
         Set_Field (Obj, "id", To_String (Msg.Id));
      end if;
      if Msg.Policy /= Policy_Unspecified then
         Set_Field (Obj, "policy", Data_Policy_To_String (Msg.Policy));
      end if;
      if Msg.Dimension /= Dimension_Unspecified then
         Set_Field (Obj, "dimension", Battle_Dimension_To_String (Msg.Dimension));
      end if;
      if Msg.Min_Lat_Rad /= 0.0 then
         Set_F (Obj, "min_lat_rad", Msg.Min_Lat_Rad);
      end if;
      if Msg.Max_Lat_Rad /= 0.0 then
         Set_F (Obj, "max_lat_rad", Msg.Max_Lat_Rad);
      end if;
      if Msg.Min_Lon_Rad /= 0.0 then
         Set_F (Obj, "min_lon_rad", Msg.Min_Lon_Rad);
      end if;
      if Msg.Max_Lon_Rad /= 0.0 then
         Set_F (Obj, "max_lon_rad", Msg.Max_Lon_Rad);
      end if;
      return Write (Obj);
   end To_Json;

   --  -- From_Json ----------------------------------------------------

   function From_Json
     (S : String) return Create_Requirement_Request
   is
      Result : Create_Requirement_Request;
      J      : JSON_Value;
      R      : Read_Result;
   begin
      R := Read (S);
      if not R.Success or else R.Value.Kind /= JSON_Object_Type then
         return Result;
      end if;
      J := R.Value;
      Result.Policy :=
        Data_Policy_From_String (Get_S (J, "policy"));
      Result.Identity :=
        Standard_Identity_From_String (Get_S (J, "identity"));
      Result.Dimension :=
        Battle_Dimension_From_String (Get_S (J, "dimension"));
      Result.Min_Lat_Rad := Get_F (J, "min_lat_rad");
      Result.Max_Lat_Rad := Get_F (J, "max_lat_rad");
      Result.Min_Lon_Rad := Get_F (J, "min_lon_rad");
      Result.Max_Lon_Rad := Get_F (J, "max_lon_rad");
      return Result;
   end From_Json;

   function From_Json
     (S : String) return Create_Requirement_Response
   is
      Result : Create_Requirement_Response;
      J      : JSON_Value;
      R      : Read_Result;
   begin
      R := Read (S);
      if not R.Success or else R.Value.Kind /= JSON_Object_Type then
         return Result;
      end if;
      J := R.Value;
      Result.Interest_Id :=
        To_Unbounded_String (Get_S (J, "interest_id"));
      return Result;
   end From_Json;

   function From_Json
     (S : String) return Entity_Match
   is
      Result : Entity_Match;
      J      : JSON_Value;
      R      : Read_Result;
   begin
      R := Read (S);
      if not R.Success or else R.Value.Kind /= JSON_Object_Type then
         return Result;
      end if;
      J := R.Value;
      Result.Object_Id :=
        To_Unbounded_String (Get_S (J, "object_id"));
      Result.Identity :=
        Standard_Identity_From_String (Get_S (J, "identity"));
      Result.Dimension :=
        Battle_Dimension_From_String (Get_S (J, "dimension"));
      Result.Latitude_Rad := Get_F (J, "latitude_rad");
      Result.Longitude_Rad := Get_F (J, "longitude_rad");
      Result.Confidence := Get_F (J, "confidence");
      return Result;
   end From_Json;

   function From_Json
     (S : String) return Object_Evidence
   is
      Result : Object_Evidence;
      J      : JSON_Value;
      R      : Read_Result;
   begin
      R := Read (S);
      if not R.Success or else R.Value.Kind /= JSON_Object_Type then
         return Result;
      end if;
      J := R.Value;
      Result.Identity :=
        Standard_Identity_From_String (Get_S (J, "identity"));
      Result.Dimension :=
        Battle_Dimension_From_String (Get_S (J, "dimension"));
      Result.Latitude_Rad := Get_F (J, "latitude_rad");
      Result.Longitude_Rad := Get_F (J, "longitude_rad");
      Result.Confidence := Get_F (J, "confidence");
      Result.Observed_At := Get_F (J, "observed_at");
      return Result;
   end From_Json;

   function From_Json
     (S : String) return Evidence_Requirement
   is
      Result : Evidence_Requirement;
      J      : JSON_Value;
      R      : Read_Result;
   begin
      R := Read (S);
      if not R.Success or else R.Value.Kind /= JSON_Object_Type then
         return Result;
      end if;
      J := R.Value;
      Result.Id :=
        To_Unbounded_String (Get_S (J, "id"));
      Result.Policy :=
        Data_Policy_From_String (Get_S (J, "policy"));
      Result.Dimension :=
        Battle_Dimension_From_String (Get_S (J, "dimension"));
      Result.Min_Lat_Rad := Get_F (J, "min_lat_rad");
      Result.Max_Lat_Rad := Get_F (J, "max_lat_rad");
      Result.Min_Lon_Rad := Get_F (J, "min_lon_rad");
      Result.Max_Lon_Rad := Get_F (J, "max_lon_rad");
      return Result;
   end From_Json;

   --  -- Entity_Matches_From_Json ------------------------------------

   function Entity_Matches_From_Json
     (S : String) return Entity_Match_Array
   is
      R   : Read_Result;
      Arr : JSON_Array;
   begin
      R := Read (S);
      if not R.Success or else R.Value.Kind /= JSON_Array_Type then
         return (1 .. 0 => <>);
      end if;
      Arr := Get (R.Value);
      declare
         Result : Entity_Match_Array (1 .. GNATCOLL.JSON.Length (Arr));
      begin
         for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
            declare
               Elem : constant JSON_Value := Get (Arr, I);
               M    : Entity_Match;
            begin
               M.Object_Id :=
                 To_Unbounded_String (Get_S (Elem, "object_id"));
               M.Identity :=
                 Standard_Identity_From_String (Get_S (Elem, "identity"));
               M.Dimension :=
                 Battle_Dimension_From_String (Get_S (Elem, "dimension"));
               M.Latitude_Rad := Get_F (Elem, "latitude_rad");
               M.Longitude_Rad := Get_F (Elem, "longitude_rad");
               M.Confidence := Get_F (Elem, "confidence");
               Result (I) := M;
            end;
         end loop;
         return Result;
      end;
   end Entity_Matches_From_Json;

end Pyramid.Services.Tactical_Objects.Json_Codec;
