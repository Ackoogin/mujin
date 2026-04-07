--  Auto-generated codec dispatch — do not edit
--  Package: Pyramid.Data_model.Tactical.Codec_Dispatch
--
--  Port-level codec routing for Ada components.
--  Routes to Json_Codec, Flatbuffers_Codec, or Protobuf_Codec
--  based on the content type string from port configuration.

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;

package Pyramid.Data_model.Tactical.Codec_Dispatch is

   --  Content type constants
   Json_Content_Type        : constant String := "application/json";
   Flatbuffers_Content_Type  : constant String := "application/flatbuffers";
   Protobuf_Content_Type     : constant String := "application/protobuf";

   --  Serialize ObjectDetail using the codec identified by Content_Type.
   function Serialize_Object_Detail
     (Msg          : Object_Detail;
      Content_Type : String) return String;

   --  Deserialize ObjectDetail from raw bytes using Content_Type.
   function Deserialize_Object_Detail
     (Data         : String;
      Content_Type : String) return Object_Detail;

   --  Serialize ObjectEvidenceRequirement using the codec identified by Content_Type.
   function Serialize_Object_Evidence_Requirement
     (Msg          : Object_Evidence_Requirement;
      Content_Type : String) return String;

   --  Deserialize ObjectEvidenceRequirement from raw bytes using Content_Type.
   function Deserialize_Object_Evidence_Requirement
     (Data         : String;
      Content_Type : String) return Object_Evidence_Requirement;

   --  Serialize ObjectInterestRequirement using the codec identified by Content_Type.
   function Serialize_Object_Interest_Requirement
     (Msg          : Object_Interest_Requirement;
      Content_Type : String) return String;

   --  Deserialize ObjectInterestRequirement from raw bytes using Content_Type.
   function Deserialize_Object_Interest_Requirement
     (Data         : String;
      Content_Type : String) return Object_Interest_Requirement;

   --  Serialize ObjectMatch using the codec identified by Content_Type.
   function Serialize_Object_Match
     (Msg          : Object_Match;
      Content_Type : String) return String;

   --  Deserialize ObjectMatch from raw bytes using Content_Type.
   function Deserialize_Object_Match
     (Data         : String;
      Content_Type : String) return Object_Match;

end Pyramid.Data_model.Tactical.Codec_Dispatch;
