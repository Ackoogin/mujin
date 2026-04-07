--  Auto-generated codec dispatch — do not edit

with Ada.Exceptions;

package body Pyramid.Data_model.Common.Codec_Dispatch is

   function Serialize_Geodetic_Position
     (Msg          : Geodetic_Position;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Geodetic_Position;

   function Deserialize_Geodetic_Position
     (Data         : String;
      Content_Type : String) return Geodetic_Position
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Geodetic_Position;

   function Serialize_Poly_Area
     (Msg          : Poly_Area;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Poly_Area;

   function Deserialize_Poly_Area
     (Data         : String;
      Content_Type : String) return Poly_Area
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Poly_Area;

   function Serialize_Achievement
     (Msg          : Achievement;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Achievement;

   function Deserialize_Achievement
     (Data         : String;
      Content_Type : String) return Achievement
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Achievement;

   function Serialize_Requirement
     (Msg          : Requirement;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Requirement;

   function Deserialize_Requirement
     (Data         : String;
      Content_Type : String) return Requirement
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Requirement;

   function Serialize_Capability
     (Msg          : Capability;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Capability;

   function Deserialize_Capability
     (Data         : String;
      Content_Type : String) return Capability
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Capability;

   function Serialize_Entity
     (Msg          : Entity;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Entity;

   function Deserialize_Entity
     (Data         : String;
      Content_Type : String) return Entity
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Entity;

   function Serialize_Circle_Area
     (Msg          : Circle_Area;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Circle_Area;

   function Deserialize_Circle_Area
     (Data         : String;
      Content_Type : String) return Circle_Area
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Circle_Area;

   function Serialize_Point
     (Msg          : Point;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Point;

   function Deserialize_Point
     (Data         : String;
      Content_Type : String) return Point
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Point;

   function Serialize_Contraint
     (Msg          : Contraint;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Contraint;

   function Deserialize_Contraint
     (Data         : String;
      Content_Type : String) return Contraint
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Contraint;

   function Serialize_Ack
     (Msg          : Ack;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Ack;

   function Deserialize_Ack
     (Data         : String;
      Content_Type : String) return Ack
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Ack;

   function Serialize_Query
     (Msg          : Query;
      Content_Type : String) return String
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.To_Json (Msg);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Serialize_Query;

   function Deserialize_Query
     (Data         : String;
      Content_Type : String) return Query
   is
   begin
      if Content_Type = Json_Content_Type then
         return Json_Codec.From_Json (Data);
      end if;
      raise Program_Error with
        "Unsupported codec: " & Content_Type;
   end Deserialize_Query;

end Pyramid.Data_model.Common.Codec_Dispatch;
