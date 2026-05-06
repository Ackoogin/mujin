--  Auto-generated data model JSON codec specification
--  Generated from: pyramid.data_model.sensorproducts.proto by generate_bindings.py (codec)
--  Package: Pyramid.Data_Model.Sensorproducts.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Sensorproducts.Types;  use Pyramid.Data_Model.Sensorproducts.Types;

package Pyramid.Data_Model.Sensorproducts.Types_Codec is

   function To_Json (Msg : Radar_Display_Product_Requirement) return String;
   function From_Json (S : String; Tag : access Radar_Display_Product_Requirement) return Radar_Display_Product_Requirement;
   function To_Json (Msg : Radar_Product_Requirement) return String;
   function From_Json (S : String; Tag : access Radar_Product_Requirement) return Radar_Product_Requirement;

end Pyramid.Data_Model.Sensorproducts.Types_Codec;
