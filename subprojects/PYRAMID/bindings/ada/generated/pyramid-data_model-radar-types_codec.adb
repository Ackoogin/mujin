--  Auto-generated data model JSON codec body
--  Package: Pyramid.Data_Model.Radar.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
pragma Warnings (Off);

package body Pyramid.Data_Model.Radar.Types_Codec is

   function To_String (V : Radar_Operational_Mode) return String is
   begin
      case V is
         when Mode_Unspecified => return "RADAR_OPERATIONAL_MODE_UNSPECIFIED";
         when Mode_MaritimeSurveillance => return "RADAR_OPERATIONAL_MODE_MARITIME_SURVEILLANCE";
         when Mode_ShortRangeAwareness => return "RADAR_OPERATIONAL_MODE_SHORT_RANGE_AWARENESS";
         when Mode_Transponder => return "RADAR_OPERATIONAL_MODE_TRANSPONDER";
         when Mode_AMti => return "RADAR_OPERATIONAL_MODE_A_MTI";
         when Mode_EnhancedMovingTargetIndicator => return "RADAR_OPERATIONAL_MODE_ENHANCED_MOVING_TARGET_INDICATOR";
         when Mode_Weather => return "RADAR_OPERATIONAL_MODE_WEATHER";
         when Mode_AirToAirSurveillance => return "RADAR_OPERATIONAL_MODE_AIR_TO_AIR_SURVEILLANCE";
         when Mode_SmallTarget => return "RADAR_OPERATIONAL_MODE_SMALL_TARGET";
         when Mode_Turbulence => return "RADAR_OPERATIONAL_MODE_TURBULENCE";
         when Mode_Isar => return "RADAR_OPERATIONAL_MODE_ISAR";
         when Mode_SpotSar => return "RADAR_OPERATIONAL_MODE_SPOT_SAR";
         when Mode_StripSar => return "RADAR_OPERATIONAL_MODE_STRIP_SAR";
         when Mode_Hrr => return "RADAR_OPERATIONAL_MODE_HRR";
         when Mode_PlannedSar => return "RADAR_OPERATIONAL_MODE_PLANNED_SAR";
         when Mode_GMti => return "RADAR_OPERATIONAL_MODE_G_MTI";
         when Mode_MMti => return "RADAR_OPERATIONAL_MODE_M_MTI";
      end case;
   end To_String;

   function Radar_Operational_Mode_From_String (S : String) return Radar_Operational_Mode is
   begin
      if S = "RADAR_OPERATIONAL_MODE_UNSPECIFIED" then return Mode_Unspecified; end if;
      if S = "RADAR_OPERATIONAL_MODE_MARITIME_SURVEILLANCE" then return Mode_MaritimeSurveillance; end if;
      if S = "RADAR_OPERATIONAL_MODE_SHORT_RANGE_AWARENESS" then return Mode_ShortRangeAwareness; end if;
      if S = "RADAR_OPERATIONAL_MODE_TRANSPONDER" then return Mode_Transponder; end if;
      if S = "RADAR_OPERATIONAL_MODE_A_MTI" then return Mode_AMti; end if;
      if S = "RADAR_OPERATIONAL_MODE_ENHANCED_MOVING_TARGET_INDICATOR" then return Mode_EnhancedMovingTargetIndicator; end if;
      if S = "RADAR_OPERATIONAL_MODE_WEATHER" then return Mode_Weather; end if;
      if S = "RADAR_OPERATIONAL_MODE_AIR_TO_AIR_SURVEILLANCE" then return Mode_AirToAirSurveillance; end if;
      if S = "RADAR_OPERATIONAL_MODE_SMALL_TARGET" then return Mode_SmallTarget; end if;
      if S = "RADAR_OPERATIONAL_MODE_TURBULENCE" then return Mode_Turbulence; end if;
      if S = "RADAR_OPERATIONAL_MODE_ISAR" then return Mode_Isar; end if;
      if S = "RADAR_OPERATIONAL_MODE_SPOT_SAR" then return Mode_SpotSar; end if;
      if S = "RADAR_OPERATIONAL_MODE_STRIP_SAR" then return Mode_StripSar; end if;
      if S = "RADAR_OPERATIONAL_MODE_HRR" then return Mode_Hrr; end if;
      if S = "RADAR_OPERATIONAL_MODE_PLANNED_SAR" then return Mode_PlannedSar; end if;
      if S = "RADAR_OPERATIONAL_MODE_G_MTI" then return Mode_GMti; end if;
      if S = "RADAR_OPERATIONAL_MODE_M_MTI" then return Mode_MMti; end if;
      return Mode_Unspecified;
   end Radar_Operational_Mode_From_String;

   function To_String (V : Radar_Operational_State) return String is
   begin
      case V is
         when State_Unspecified => return "RADAR_OPERATIONAL_STATE_UNSPECIFIED";
      end case;
   end To_String;

   function Radar_Operational_State_From_String (S : String) return Radar_Operational_State is
   begin
      if S = "RADAR_OPERATIONAL_STATE_UNSPECIFIED" then return State_Unspecified; end if;
      return State_Unspecified;
   end Radar_Operational_State_From_String;

end Pyramid.Data_Model.Radar.Types_Codec;
