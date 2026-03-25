--  Auto-generated data model JSON codec body
--  Package: Pyramid_Data_Model_Tactical_Types_Codec

with pyramid_data_model_tactical_types_codec.ads;
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
   begin
      --  TODO: serialise Object_Detail fields to JSON
      pragma Unreferenced (Msg);
      return "{}";
   end To_Json;

   function From_Json (S : String; Tag : access Object_Detail) return Object_Detail is
      pragma Unreferenced (S, Tag);
      Result : Object_Detail;
   begin
      --  TODO: deserialise JSON to Object_Detail fields
      return Result;
   end From_Json;

   function To_Json (Msg : Object_Evidence_Requirement) return String is
   begin
      --  TODO: serialise Object_Evidence_Requirement fields to JSON
      pragma Unreferenced (Msg);
      return "{}";
   end To_Json;

   function From_Json (S : String; Tag : access Object_Evidence_Requirement) return Object_Evidence_Requirement is
      pragma Unreferenced (S, Tag);
      Result : Object_Evidence_Requirement;
   begin
      --  TODO: deserialise JSON to Object_Evidence_Requirement fields
      return Result;
   end From_Json;

   function To_Json (Msg : Object_Interest_Requirement) return String is
   begin
      --  TODO: serialise Object_Interest_Requirement fields to JSON
      pragma Unreferenced (Msg);
      return "{}";
   end To_Json;

   function From_Json (S : String; Tag : access Object_Interest_Requirement) return Object_Interest_Requirement is
      pragma Unreferenced (S, Tag);
      Result : Object_Interest_Requirement;
   begin
      --  TODO: deserialise JSON to Object_Interest_Requirement fields
      return Result;
   end From_Json;

   function To_Json (Msg : Object_Match) return String is
   begin
      --  TODO: serialise Object_Match fields to JSON
      pragma Unreferenced (Msg);
      return "{}";
   end To_Json;

   function From_Json (S : String; Tag : access Object_Match) return Object_Match is
      pragma Unreferenced (S, Tag);
      Result : Object_Match;
   begin
      --  TODO: deserialise JSON to Object_Match fields
      return Result;
   end From_Json;

end Pyramid_Data_Model_Tactical_Types_Codec;
