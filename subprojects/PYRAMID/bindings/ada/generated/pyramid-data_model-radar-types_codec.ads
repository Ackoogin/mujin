--  Auto-generated data model JSON codec specification
--  Generated from: pyramid.data_model.radar.proto by generate_bindings.py (codec)
--  Package: Pyramid.Data_Model.Radar.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Radar.Types;  use Pyramid.Data_Model.Radar.Types;

package Pyramid.Data_Model.Radar.Types_Codec is

   function To_String (V : Radar_Operational_Mode) return String;
   function Radar_Operational_Mode_From_String (S : String) return Radar_Operational_Mode;
   function To_String (V : Radar_Operational_State) return String;
   function Radar_Operational_State_From_String (S : String) return Radar_Operational_State;

end Pyramid.Data_Model.Radar.Types_Codec;
