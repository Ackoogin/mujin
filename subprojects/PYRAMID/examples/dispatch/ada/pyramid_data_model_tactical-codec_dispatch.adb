--  Auto-generated codec dispatch — do not edit

with Ada.Exceptions;

package body Pyramid.Data_model.Tactical.Codec_Dispatch is

   function Serialize_Object_Detail
     (Msg          : Object_Detail;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Object_Detail;

   function Deserialize_Object_Detail
     (Data         : String;
      Content_Type : String) return Object_Detail
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Object_Detail;

   function Serialize_Object_Evidence_Requirement
     (Msg          : Object_Evidence_Requirement;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Object_Evidence_Requirement;

   function Deserialize_Object_Evidence_Requirement
     (Data         : String;
      Content_Type : String) return Object_Evidence_Requirement
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Object_Evidence_Requirement;

   function Serialize_Object_Interest_Requirement
     (Msg          : Object_Interest_Requirement;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Object_Interest_Requirement;

   function Deserialize_Object_Interest_Requirement
     (Data         : String;
      Content_Type : String) return Object_Interest_Requirement
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Object_Interest_Requirement;

   function Serialize_Object_Match
     (Msg          : Object_Match;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Object_Match;

   function Deserialize_Object_Match
     (Data         : String;
      Content_Type : String) return Object_Match
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Object_Match;

end Pyramid.Data_model.Tactical.Codec_Dispatch;
