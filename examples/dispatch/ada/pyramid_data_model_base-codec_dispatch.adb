--  Auto-generated codec dispatch — do not edit

with Ada.Exceptions;

package body Pyramid.Data_model.Base.Codec_Dispatch is

   function Serialize_Angle
     (Msg          : Angle;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Angle;

   function Deserialize_Angle
     (Data         : String;
      Content_Type : String) return Angle
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Angle;

   function Serialize_Length
     (Msg          : Length;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Length;

   function Deserialize_Length
     (Data         : String;
      Content_Type : String) return Length
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Length;

   function Serialize_Timestamp
     (Msg          : Timestamp;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Timestamp;

   function Deserialize_Timestamp
     (Data         : String;
      Content_Type : String) return Timestamp
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Timestamp;

   function Serialize_Identifier
     (Msg          : Identifier;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Identifier;

   function Deserialize_Identifier
     (Data         : String;
      Content_Type : String) return Identifier
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Identifier;

   function Serialize_Speed
     (Msg          : Speed;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Speed;

   function Deserialize_Speed
     (Data         : String;
      Content_Type : String) return Speed
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Speed;

   function Serialize_Percentage
     (Msg          : Percentage;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Percentage;

   function Deserialize_Percentage
     (Data         : String;
      Content_Type : String) return Percentage
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Percentage;

end Pyramid.Data_model.Base.Codec_Dispatch;
