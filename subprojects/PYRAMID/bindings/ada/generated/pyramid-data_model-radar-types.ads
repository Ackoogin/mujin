--  Auto-generated types specification
--  Generated from: pyramid.data_model.radar.proto by generate_bindings.py (types)
--  Package: Pyramid.Data_Model.Radar.Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;

package Pyramid.Data_Model.Radar.Types is


   type Radar_Operational_Mode is
     (Mode_Unspecified,
      Mode_MaritimeSurveillance,
      Mode_ShortRangeAwareness,
      Mode_Transponder,
      Mode_AMti,
      Mode_EnhancedMovingTargetIndicator,
      Mode_Weather,
      Mode_AirToAirSurveillance,
      Mode_SmallTarget,
      Mode_Turbulence,
      Mode_Isar,
      Mode_SpotSar,
      Mode_StripSar,
      Mode_Hrr,
      Mode_PlannedSar,
      Mode_GMti,
      Mode_MMti);

   type Radar_Operational_State is
     (State_Unspecified);

end Pyramid.Data_Model.Radar.Types;
