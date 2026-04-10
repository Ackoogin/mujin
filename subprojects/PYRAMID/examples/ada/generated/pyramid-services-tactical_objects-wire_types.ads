--  Auto-generated service wire types specification
--  Package: Pyramid.Services.Tactical_Objects.Wire_Types
--  Generated from pim/json_schema.py

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;

package Pyramid.Services.Tactical_Objects.Wire_Types is

   --  element of standard.entity_matches array (proto: ObjectMatch + position/confidence)
   type Entity_Match is record
      Object_Id               : Unbounded_String := Null_Unbounded_String;
      Identity                : Standard_Identity := Identity_Unspecified;
      Dimension               : Battle_Dimension := Dimension_Unspecified;  --  optional
      Latitude_Rad            : Long_Float := 0.0;  --  optional
      Longitude_Rad           : Long_Float := 0.0;  --  optional
      Confidence              : Long_Float := 0.0;  --  optional
   end record;

   --  standard.object_evidence publish payload (proto: ObjectDetail)
   type Object_Evidence is record
      Identity                : Standard_Identity := Identity_Unspecified;
      Dimension               : Battle_Dimension := Dimension_Unspecified;
      Latitude_Rad            : Long_Float := 0.0;
      Longitude_Rad           : Long_Float := 0.0;
      Confidence              : Long_Float := 0.0;
      Observed_At             : Long_Float := 0.0;  --  Observation timestamp, seconds
   end record;

   --  standard.evidence_requirements subscribe payload (proto: ObjectEvidenceRequirement)
   type Evidence_Requirement is record
      Id                      : Unbounded_String := Null_Unbounded_String;  --  optional
      Policy                  : Data_Policy := Policy_Unspecified;  --  optional
      Dimension               : Battle_Dimension := Dimension_Unspecified;  --  optional
      Min_Lat_Rad             : Long_Float := 0.0;  --  optional
      Max_Lat_Rad             : Long_Float := 0.0;  --  optional
      Min_Lon_Rad             : Long_Float := 0.0;  --  optional
      Max_Lon_Rad             : Long_Float := 0.0;  --  optional
   end record;

   type Entity_Match_Array is
     array (Positive range <>) of Entity_Match;

end Pyramid.Services.Tactical_Objects.Wire_Types;
