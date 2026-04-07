--  Auto-generated codec dispatch — do not edit
--  Package: Pyramid.Data_model.Base.Codec_Dispatch
--
--  Port-level codec routing for Ada components.
--  Routes to Json_Codec, Flatbuffers_Codec, or Protobuf_Codec
--  based on the content type string from port configuration.

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;

package Pyramid.Data_model.Base.Codec_Dispatch is

   --  Content type constants
   Json_Content_Type        : constant String := "application/json";
   Flatbuffers_Content_Type  : constant String := "application/flatbuffers";
   Protobuf_Content_Type     : constant String := "application/protobuf";

   --  Serialize Angle using the codec identified by Content_Type.
   function Serialize_Angle
     (Msg          : Angle;
      Content_Type : String) return String;

   --  Deserialize Angle from raw bytes using Content_Type.
   function Deserialize_Angle
     (Data         : String;
      Content_Type : String) return Angle;

   --  Serialize Length using the codec identified by Content_Type.
   function Serialize_Length
     (Msg          : Length;
      Content_Type : String) return String;

   --  Deserialize Length from raw bytes using Content_Type.
   function Deserialize_Length
     (Data         : String;
      Content_Type : String) return Length;

   --  Serialize Timestamp using the codec identified by Content_Type.
   function Serialize_Timestamp
     (Msg          : Timestamp;
      Content_Type : String) return String;

   --  Deserialize Timestamp from raw bytes using Content_Type.
   function Deserialize_Timestamp
     (Data         : String;
      Content_Type : String) return Timestamp;

   --  Serialize Identifier using the codec identified by Content_Type.
   function Serialize_Identifier
     (Msg          : Identifier;
      Content_Type : String) return String;

   --  Deserialize Identifier from raw bytes using Content_Type.
   function Deserialize_Identifier
     (Data         : String;
      Content_Type : String) return Identifier;

   --  Serialize Speed using the codec identified by Content_Type.
   function Serialize_Speed
     (Msg          : Speed;
      Content_Type : String) return String;

   --  Deserialize Speed from raw bytes using Content_Type.
   function Deserialize_Speed
     (Data         : String;
      Content_Type : String) return Speed;

   --  Serialize Percentage using the codec identified by Content_Type.
   function Serialize_Percentage
     (Msg          : Percentage;
      Content_Type : String) return String;

   --  Deserialize Percentage from raw bytes using Content_Type.
   function Deserialize_Percentage
     (Data         : String;
      Content_Type : String) return Percentage;

end Pyramid.Data_model.Base.Codec_Dispatch;
