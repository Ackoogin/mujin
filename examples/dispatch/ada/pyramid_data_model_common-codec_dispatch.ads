--  Auto-generated codec dispatch — do not edit
--  Package: Pyramid.Data_model.Common.Codec_Dispatch
--
--  Port-level codec routing for Ada components.
--  Routes to Json_Codec, Flatbuffers_Codec, or Protobuf_Codec
--  based on the content type string from port configuration.

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;

package Pyramid.Data_model.Common.Codec_Dispatch is

   --  Content type constants
   Json_Content_Type        : constant String := "application/json";
   Flatbuffers_Content_Type  : constant String := "application/flatbuffers";
   Protobuf_Content_Type     : constant String := "application/protobuf";

   --  Serialize GeodeticPosition using the codec identified by Content_Type.
   function Serialize_Geodetic_Position
     (Msg          : Geodetic_Position;
      Content_Type : String) return String;

   --  Deserialize GeodeticPosition from raw bytes using Content_Type.
   function Deserialize_Geodetic_Position
     (Data         : String;
      Content_Type : String) return Geodetic_Position;

   --  Serialize PolyArea using the codec identified by Content_Type.
   function Serialize_Poly_Area
     (Msg          : Poly_Area;
      Content_Type : String) return String;

   --  Deserialize PolyArea from raw bytes using Content_Type.
   function Deserialize_Poly_Area
     (Data         : String;
      Content_Type : String) return Poly_Area;

   --  Serialize Achievement using the codec identified by Content_Type.
   function Serialize_Achievement
     (Msg          : Achievement;
      Content_Type : String) return String;

   --  Deserialize Achievement from raw bytes using Content_Type.
   function Deserialize_Achievement
     (Data         : String;
      Content_Type : String) return Achievement;

   --  Serialize Requirement using the codec identified by Content_Type.
   function Serialize_Requirement
     (Msg          : Requirement;
      Content_Type : String) return String;

   --  Deserialize Requirement from raw bytes using Content_Type.
   function Deserialize_Requirement
     (Data         : String;
      Content_Type : String) return Requirement;

   --  Serialize Capability using the codec identified by Content_Type.
   function Serialize_Capability
     (Msg          : Capability;
      Content_Type : String) return String;

   --  Deserialize Capability from raw bytes using Content_Type.
   function Deserialize_Capability
     (Data         : String;
      Content_Type : String) return Capability;

   --  Serialize Entity using the codec identified by Content_Type.
   function Serialize_Entity
     (Msg          : Entity;
      Content_Type : String) return String;

   --  Deserialize Entity from raw bytes using Content_Type.
   function Deserialize_Entity
     (Data         : String;
      Content_Type : String) return Entity;

   --  Serialize CircleArea using the codec identified by Content_Type.
   function Serialize_Circle_Area
     (Msg          : Circle_Area;
      Content_Type : String) return String;

   --  Deserialize CircleArea from raw bytes using Content_Type.
   function Deserialize_Circle_Area
     (Data         : String;
      Content_Type : String) return Circle_Area;

   --  Serialize Point using the codec identified by Content_Type.
   function Serialize_Point
     (Msg          : Point;
      Content_Type : String) return String;

   --  Deserialize Point from raw bytes using Content_Type.
   function Deserialize_Point
     (Data         : String;
      Content_Type : String) return Point;

   --  Serialize Contraint using the codec identified by Content_Type.
   function Serialize_Contraint
     (Msg          : Contraint;
      Content_Type : String) return String;

   --  Deserialize Contraint from raw bytes using Content_Type.
   function Deserialize_Contraint
     (Data         : String;
      Content_Type : String) return Contraint;

   --  Serialize Ack using the codec identified by Content_Type.
   function Serialize_Ack
     (Msg          : Ack;
      Content_Type : String) return String;

   --  Deserialize Ack from raw bytes using Content_Type.
   function Deserialize_Ack
     (Data         : String;
      Content_Type : String) return Ack;

   --  Serialize Query using the codec identified by Content_Type.
   function Serialize_Query
     (Msg          : Query;
      Content_Type : String) return String;

   --  Deserialize Query from raw bytes using Content_Type.
   function Deserialize_Query
     (Data         : String;
      Content_Type : String) return Query;

end Pyramid.Data_model.Common.Codec_Dispatch;
