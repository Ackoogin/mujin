--  Auto-generated data model JSON codec specification
--  Generated from: pyramid.data_model.sensors.proto by generate_bindings.py (codec)
--  Package: Pyramid.Data_Model.Sensors.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Sensors.Types;  use Pyramid.Data_Model.Sensors.Types;

package Pyramid.Data_Model.Sensors.Types_Codec is

   function To_String (V : Interpretation_Policy) return String;
   function Interpretation_Policy_From_String (S : String) return Interpretation_Policy;
   function To_String (V : Interpretation_Type) return String;
   function Interpretation_Type_From_String (S : String) return Interpretation_Type;

   function To_Json (Msg : Interpretation_Requirement) return String;
   function From_Json (S : String; Tag : access Interpretation_Requirement) return Interpretation_Requirement;
   function To_Json (Msg : Manual_Track_Requirement) return String;
   function From_Json (S : String; Tag : access Manual_Track_Requirement) return Manual_Track_Requirement;
   function To_Json (Msg : Ati_Requirement) return String;
   function From_Json (S : String; Tag : access Ati_Requirement) return Ati_Requirement;
   function To_Json (Msg : Track_Provision_Requirement) return String;
   function From_Json (S : String; Tag : access Track_Provision_Requirement) return Track_Provision_Requirement;
   function To_Json (Msg : Object_Evidence_Provision_Requirement) return String;
   function From_Json (S : String; Tag : access Object_Evidence_Provision_Requirement) return Object_Evidence_Provision_Requirement;
   function To_Json (Msg : Object_Aquisition_Requirement) return String;
   function From_Json (S : String; Tag : access Object_Aquisition_Requirement) return Object_Aquisition_Requirement;
   function To_Json (Msg : Sensor_Object) return String;
   function From_Json (S : String; Tag : access Sensor_Object) return Sensor_Object;
   function To_Json (Msg : Radar_Mode_Change_Requirement) return String;
   function From_Json (S : String; Tag : access Radar_Mode_Change_Requirement) return Radar_Mode_Change_Requirement;

end Pyramid.Data_Model.Sensors.Types_Codec;
